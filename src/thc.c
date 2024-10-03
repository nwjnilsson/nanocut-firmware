#include "grbl.h"
#include "nuts_bolts.h"

volatile uint32_t millis;
volatile uint32_t arc_stablization_timer;
volatile uint16_t thc_pulse_counter;
volatile uint16_t thc_ctrl_counter;
volatile int32_t thc_speed_offset;
int32_t z_axis_steps_limit;
volatile uint16_t thc_pulse_period;
uint16_t thc_pulse_period_target;
uint16_t pp_change_rate;  // amount to increase/decrease pulse period
                                  // when accelerating/decelerating

#define THC_UPDATE_PERIOD_US 5000U
#define THC_UPDATE_PERIOD_MS 5
#define TIM2_LOAD_VAL 206
#define THC_PULSE_PERIOD_MAX 0x7FFF
#define THC_SPEED_OFFSET_LIMIT ((int32_t)THC_PULSE_PERIOD_MAX - (int32_t)thc_pulse_period_target)


void thc_update()
{
    if(PINC & (1<<PC1))
    {
        //We don't have an arc_ok signal
        jog_z_action = STAY;
        arc_stablization_timer = millis;
    }
    else
    {
        //We have an arc_ok signal!
        //Out ADC input is 2:1 voltage divider so pre-divider is 0-10V and post divider is 0-5V. 
        //ADC resolution is 0-1024; Each ADC tick is 0.488 Volts pre-divider (AV+) at 1:50th scale!
        //or 0.009 volts at scaled scale (0-10)
        //Wait 3 secends for arc voltage to stabalize
        if ((millis - arc_stablization_timer) > 3000)
        {
            if (analogSetVal > 30) //THC is turned on
            {
                if ((analogVal > (analogSetVal - 10)) && (analogVal < (analogSetVal + 10))) //We are within our ok range
                {
                    jog_z_action = STAY;
                }
                else //We are not in range and need to deterimine direction needed to put us in range
                {
                    if (analogVal > analogSetVal) //Torch is too high
                    {
                        jog_z_action = DOWN;
                    }
                    else //Torch is too low
                    {
                        jog_z_action = UP;
                    }
                }
            }
        }
    }
}

//void debug_print(uint32_t num) {
//    while (num--) {
//        PORTD |= (1 << PD7);
//        delay_us(2);
//        PORTD &= ~(1 << PD7);
//        delay_us(2);
//    }
//}
void setup_timer_2(uint8_t count) {
    TCCR2B = 0x00;          //Disable Timer2 while we set it up
    TCNT2  = count; //Reset Timer Count
    TIFR2  = 0x00;          //Timer2 INT Flag Reg: Clear Timer Overflow Flag
    TIMSK2 = 0x01;          //Timer2 INT Reg: Timer2 Overflow Interrupt Enable
    TCCR2A = 0x00;          //Timer2 Control Reg A: Wave Gen Mode normal
    TCCR2B = (1 << CS21) | (1 << CS20);   // Prescaler 32
}
// Runs every time settings are written to the Z-axis as well as on startup
void thc_init()
{
    arc_stablization_timer = 0;
    millis = 0;
    // Calculate target pulse period from max feedrate
    thc_pulse_period_target = ceil((1e6 * 60.f) / (settings.max_rate[Z_AXIS] * settings.steps_per_mm[Z_AXIS]));
    thc_pulse_period = THC_PULSE_PERIOD_MAX;
    // Acceleration stuff
    float delta_pulse_period = THC_PULSE_PERIOD_MAX - thc_pulse_period_target;
    float tick_change_rate_minute =
        (delta_pulse_period * settings.acceleration[Z_AXIS]) / settings.max_rate[Z_AXIS]; // change in thc ticks per minute
    pp_change_rate = ceil(tick_change_rate_minute / (1e4 * 60.f)); // divide by number of thc ticks per minute
    // ---
    z_axis_steps_limit = (int32_t)settings.max_travel[Z_AXIS]*settings.steps_per_mm[Z_AXIS];
    thc_pulse_counter = 0;
    thc_ctrl_counter = 0;
    thc_speed_offset = 0;

    //Setup Timer2
    setup_timer_2(TIM2_LOAD_VAL);
}

void decrement_speed() {
    thc_speed_offset -= pp_change_rate;
}
void increment_speed() {
    thc_speed_offset += pp_change_rate;
}

// Flips the step bit and updates the machine position
void thc_step(int8_t heading)
{
    int32_t current_steps = sys_position[Z_AXIS];
    // This assumes that the z-axis range is [-D, 0]
    if ( ((heading == UP) && (current_steps < 0)) ||
         ((heading == DOWN) && (current_steps > z_axis_steps_limit)) )
    {
        // Step
        PORTD |= (1 << PD4);
        delay_us(THC_PULSE_TIME_US);
        PORTD &= ~(1 << PD4);
        sys_position[Z_AXIS] += heading;
    }
}

/* Timer 2 overflow interrupt subroutine
* The output compare unit (OC2A and OC2B) are connected to physical pins PB3 and PD3, both of which 
* are already in use. Therefore, overflow interrupts are used to generate the THC stepping pulses.
* The processor frequency is 16 MHz, the prescaler is set to 32, and an interrupt is generated every 40
* ticks. This gives an interrupt frequency of ~10 KHz. This was not tested extensively but the machine seems
* to move approximately the correct distance for the given time.
*
* This stuff is pretty hacky to get around GRBL control of the Z axis. If the Z axis is moved
* here while GRBL is doing a touch-off for example, it will cause problems. The idea is that the
* THC should only be able to move the torch when there is an arc OK signal or when PGDOWN/PGUP is
* pressed in the control software.
*/
ISR(TIMER2_OVF_vect) 
{
    uint8_t start = TCNT2;
    thc_ctrl_counter += 100;
    thc_pulse_counter += 100;

    if( thc_ctrl_counter >= THC_UPDATE_PERIOD_US )
    {
        millis += THC_UPDATE_PERIOD_MS;
        thc_ctrl_counter = thc_ctrl_counter - THC_UPDATE_PERIOD_US;
        if (machine_in_motion) thc_update(); // Evaluate what the THC should be doing
    }
    
    bool invert = false; // invert direction? if direction quickly changes from up to down, we invert
                         // the direction while decelerating. when the speed reaches 0, we stop inverting
    int8_t decel_h = STAY;
    if (((jog_z_action == STAY) || (jog_z_action == DOWN)) && (thc_speed_offset > (int32_t)pp_change_rate))
        decel_h = UP;
    else if (((jog_z_action == STAY) || (jog_z_action == UP)) && (thc_speed_offset < -((int32_t)pp_change_rate)))
        decel_h = DOWN;

    if (decel_h == DOWN) {
        increment_speed();
    }
    else if (decel_h == UP) {
        decrement_speed();
    }
    else {
        // thc_speed_offset is in the vicinity of 0, disable the pulses
        if (jog_z_action == UP && (thc_speed_offset < THC_SPEED_OFFSET_LIMIT)) {
            increment_speed();
            invert = thc_speed_offset < 0;
        }
        else if (jog_z_action == DOWN && (thc_speed_offset > -THC_SPEED_OFFSET_LIMIT)) {
            decrement_speed();
            invert = thc_speed_offset > 0;
        }
        else if (jog_z_action == STAY)
            // decel_h == STAY && jog_z_action == STAY
            thc_pulse_counter = 0;

    }
    invert = !invert != !(settings.dir_invert_mask & (1 << Z_AXIS)); // xor, as two inverts cancel each other out

    if(thc_pulse_counter >= thc_pulse_period)
    {
        // Generate pulse
        // Set to the overflow to try to compensate for inaccuracies
        thc_pulse_counter = thc_pulse_counter - thc_pulse_period;
        // Set direction
        int8_t heading = (decel_h == STAY) ? jog_z_action : decel_h;
        if ((heading == UP && invert)    ||
            (heading == DOWN && !invert) )
        {
            PORTD |= (1 << PD7);
        }
        else
        {
            PORTD &= ~(1 << PD7);
        }
        thc_step(heading);

        // Update current pulse period
        thc_pulse_period = THC_PULSE_PERIOD_MAX - abs(thc_speed_offset);

    }

    // Try to compensate for irq overhead by adding TCNT2
    uint8_t load_val = min(255, (TCNT2 - start) + TIM2_LOAD_VAL);
    setup_timer_2(load_val);
}

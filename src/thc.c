#include "grbl.h"

unsigned long thc_step_delay;
unsigned long arc_stablization_timer;
volatile unsigned long millis;
volatile uint16_t thc_ctrl_counter;
volatile uint16_t thc_pulse_counter;
volatile uint16_t thc_pulse_period;
volatile uint8_t z_dir_invert;

#define THC_UPDATE_PERIOD_MICROS 5000
#define TIM2_LOAD_VAL 206


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


void thc_init()
{
    arc_stablization_timer = 0;
    millis = 0;
    // Calculate pulse period from feedrate
    thc_pulse_period = ceil((1e6 * 60.f) / (settings.max_rate[Z_AXIS] * settings.steps_per_mm[Z_AXIS]));
    thc_pulse_counter = 0;
    thc_ctrl_counter = 0;

    //Setup Timer2
    TCCR2B = 0x00;          //Disable Timer2 while we set it up
    TCNT2  = TIM2_LOAD_VAL; //Reset Timer Count to 0 out of 255
    TIFR2  = 0x00;          //Timer2 INT Flag Reg: Clear Timer Overflow Flag
    TIMSK2 = 0x01;          //Timer2 INT Reg: Timer2 Overflow Interrupt Enable
    TCCR2A = 0x00;          //Timer2 Control Reg A: Wave Gen Mode normal
    TCCR2B = (1 << CS21) | (1 << CS20);   // Prescaler 32
}

// Flips the step bit and updates the machine position
void thc_step()
{
    int32_t current_steps[N_AXIS]; // Copy current state of the system position variable
    memcpy(current_steps, sys_position, sizeof(sys_position));
    float current_position[N_AXIS];
    system_convert_array_steps_to_mpos(current_position, current_steps);

    if ( (jog_z_action == UP && (current_position[Z_AXIS] < 0.f - __FLT_EPSILON__)) ||
         (jog_z_action == DOWN && (current_position[Z_AXIS] > settings.max_travel[Z_AXIS])) )
    {
        //Step
        PORTD |= (1 << PD4);
        delay_us(THC_PULSE_TIME);
        PORTD &= ~(1 << PD4);
        sys_position[Z_AXIS] += jog_z_action;
    }
}

/* Timer 2 overflow interrupt subroutine
* The output compare unit (OC2A and OC2B) are connected to physical pins PB3 and PD3, both of which 
* are already in use. Therefore, overflow interrupts are used to generate the THC stepping pulses.
* The processor frequency is 16 MHz, the prescaler is set to 32, and an interrupt is generated every 40
* ticks. This gives an interrupt frequency of ~10 KHz. This was not tested extensively but the machine seems
* to move approximately the correct distance for the given time. 
*/
ISR(TIMER2_OVF_vect) 
{
    uint8_t start = TCNT2;
    thc_ctrl_counter += 100;
    thc_pulse_counter += 100;

    if( thc_ctrl_counter >= THC_UPDATE_PERIOD_MICROS )
    {
        millis += THC_UPDATE_PERIOD_MICROS / 1000;
        thc_ctrl_counter = thc_ctrl_counter - THC_UPDATE_PERIOD_MICROS;
        if (machine_in_motion == true) thc_update(); //Once a millisecond, evaluate what the THC should be doing
        // Calculate pulse period again in case it has changed (this should be done upon a write event but whatever)
        thc_pulse_period = ceil((1e6 * 60.f) / (settings.max_rate[Z_AXIS] * settings.steps_per_mm[Z_AXIS]));
        z_dir_invert = (settings.dir_invert_mask & (1 << Z_AXIS));
    }
    
    // Raise step pulse
    if(thc_pulse_counter >= thc_pulse_period)
    {
        // Set to the overflow to try to compensate for inaccuracies
        thc_pulse_counter = thc_pulse_counter - thc_pulse_period;
        // Set direction
        if ((jog_z_action == UP && z_dir_invert)    ||
            (jog_z_action == DOWN && !z_dir_invert) )
        {
            PORTD |= (1 << PD7);
        }
        else
        {
            PORTD &= ~(1 << PD7);
        }
        thc_step();
    }

    // Try to compensate for irq overhead by adding TCNT2
    uint16_t load_val = min(255, (TCNT2 - start) + TIM2_LOAD_VAL);
    TCCR2B = 0x00;        // Disable Timer2
    TIFR2 = 0x00;         // Clear flag
    TCNT2 = load_val;
    TCCR2B = (1 << CS21) | (1 << CS20);   // Prescaler 32
}
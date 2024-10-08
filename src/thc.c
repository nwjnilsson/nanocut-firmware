#include "grbl.h"

// Pulse period / acceleration related
#define SPEED_PROFILE_RESOLUTION 100
#define THC_UPDATE_PERIOD_TICKS 50
#define THC_UPDATE_PERIOD_MS 5
#define TIM2_LOAD_VAL 206
#define TIM2_LOAD_VAL_MAX 255
#define THC_PULSE_PERIOD_MAX 0x7FFFU
#define THC_SPEED_LIMIT (SPEED_PROFILE_RESOLUTION - 1)
#define ISR_TICKS_PER_MINUTE (1e4 * 60.f)

// globals used by THC, I'm too lazy to make something better
volatile uint32_t millis                 = 0;
volatile uint32_t arc_stablization_timer = 0;
volatile uint16_t thc_pulse_counter      = 0;
volatile uint16_t thc_ctrl_counter       = 0;
int32_t           z_axis_steps_limit;

// Array of pulse periods, slowest at index 0
uint16_t speed_profile[SPEED_PROFILE_RESOLUTION];
// used to index the speed profile, sign indicates direction
volatile int16_t  thc_speed = 0;
uint16_t          accel_threshold;
volatile uint16_t accel_counter = 0;

void thc_update()
{
  if (CONTROL_PIN & (1 << ARC_OK_BIT)) {
    // We don't have an arc_ok signal
    jog_z_action           = STAY;
    arc_stablization_timer = millis;
  }
  else {
    // We have an arc_ok signal!
    // Our ADC input is 2:1 voltage divider so pre-divider is 0-10V and post
    // divider is 0-5V. ADC resolution is 0-1024; Each ADC tick is 0.488 Volts
    // pre-divider (AV+) at 1:50th scale! or 0.009 volts at scaled scale (0-10)
    // Wait 3 secends for arc voltage to stabalize
    if ((millis - arc_stablization_timer) > ARC_STABILIZATION_TIME_MS) {
      if (analogSetVal > THC_ON_THRESHOLD) {
        if ((analogVal > (analogSetVal - THC_ALLOWED_ERROR)) &&
            (analogVal < (analogSetVal + THC_ALLOWED_ERROR))) {
          jog_z_action = STAY;
        }
        else {
          if (analogVal > analogSetVal) {
            jog_z_action = DOWN;
          }
          else {
            jog_z_action = UP;
          }
        }
      }
    }
  }
}

// void debug_print(uint32_t num) {
//     while (num--) {
//         PORTD |= (1 << PD7);
//         delay_us(1);
//         PORTD &= ~(1 << PD7);
//         delay_us(1);
//     }
// }

void setup_timer_2(uint8_t count)
{
  TCCR2B = 0x00;  // Disable Timer2 while we set it up
  TCNT2  = count; // Reset Timer Count
  TIFR2  = 0x00;  // Timer2 INT Flag Reg: Clear Timer Overflow Flag
  TIMSK2 = 0x01;  // Timer2 INT Reg: Timer2 Overflow Interrupt Enable
  TCCR2A = 0x00;  // Timer2 Control Reg A: Wave Gen Mode normal
  TCCR2B = (1 << CS21) | (1 << CS20); // Prescaler 32
}

// Runs every time settings are written to the Z-axis as well as on startup
void thc_init()
{
  // Speed profile calculation
  float speed_step = settings.max_rate[Z_AXIS] / SPEED_PROFILE_RESOLUTION;
  speed_profile[0] = THC_PULSE_PERIOD_MAX;
  for (int16_t i = 0; i < SPEED_PROFILE_RESOLUTION; ++i) {
    speed_profile[i] =
      min(THC_PULSE_PERIOD_MAX,
          ceil((ISR_TICKS_PER_MINUTE) /
               ((i + 1) * speed_step * settings.steps_per_mm[Z_AXIS])));
  }

  // After this many ISR ticks, we can increase/decrease the speed
  accel_threshold =
    ceil(speed_step * ISR_TICKS_PER_MINUTE / settings.acceleration[Z_AXIS]);

  z_axis_steps_limit =
    (int32_t) settings.max_travel[Z_AXIS] * settings.steps_per_mm[Z_AXIS];

  // Setup Timer2
  setup_timer_2(TIM2_LOAD_VAL);
}

void decrement_speed()
{
  if (thc_speed > -THC_SPEED_LIMIT) {
    if (accel_counter++ >= accel_threshold) {
      thc_speed--;
      accel_counter = 0;
    }
  }
}

void increment_speed()
{
  if (thc_speed < THC_SPEED_LIMIT) {
    if (accel_counter++ >= accel_threshold) {
      thc_speed++;
      accel_counter = 0;
    }
  }
}

void set_dir_up()
{
  if (settings.dir_invert_mask & (1 << Z_AXIS))
    DIRECTION_PORT |= (1 << Z_DIRECTION_BIT);
  else
    DIRECTION_PORT &= ~(1 << Z_DIRECTION_BIT);
}

void set_dir_down()
{
  if (settings.dir_invert_mask & (1 << Z_AXIS))
    DIRECTION_PORT &= ~(1 << Z_DIRECTION_BIT);
  else {
    DIRECTION_PORT |= (1 << Z_DIRECTION_BIT);
  }
}

// If it is time to generate a step pulse, flip the step bit and updates the
// machine position. It is assumed that the Z-axis range is [-Z, 0]
void thc_step(int8_t heading)
{
  uint16_t current_period = speed_profile[abs(thc_speed)];
  if (thc_pulse_counter >= current_period) {
    thc_pulse_counter = thc_pulse_counter - current_period;
    // Step
    STEP_PORT |= (1 << Z_STEP_BIT);
    delay_us(THC_PULSE_TIME_US);
    STEP_PORT &= ~(1 << Z_STEP_BIT);
    sys_position[Z_AXIS] += heading;
  }
}

void thc_step_down()
{
  int32_t current_steps = sys_position[Z_AXIS];
  if (current_steps > z_axis_steps_limit)
    thc_step(DOWN);
}

void thc_step_up()
{
  int32_t current_steps = sys_position[Z_AXIS];
  if (current_steps < 0)
    thc_step(UP);
}

// Update speed, direction etc. Assumes that the current action is UP or STAY.
// but the torch is still moving down due to ongoing deceleration.
bool decelerating_down()
{
  if (thc_speed < 0) {
    increment_speed();
    set_dir_down();
    thc_step_down();
    return true;
  }
  else
    return false;
}

// Same as above, but opposite
bool decelerating_up()
{
  if (thc_speed > 0) {
    decrement_speed();
    set_dir_up();
    thc_step_up();
    return true;
  }
  else
    return false;
}

/* Timer 2 overflow interrupt subroutine
 * The output compare unit (OC2A and OC2B) are connected to physical pins PB3
 * and PD3, both of which are already in use. Therefore, overflow interrupts are
 * used to generate the THC stepping pulses. The processor frequency is 16 MHz,
 * the prescaler is set to 32, and an interrupt is generated every 40 ticks.
 * This gives an interrupt frequency of ~10 KHz. This was not tested extensively
 * but the machine seems to move approximately the correct distance for the
 * given time.
 *
 * This stuff is pretty hacky to get around GRBL control of the Z axis. If the Z
 * axis is moved here while GRBL is doing a touch-off for example, it will cause
 * problems. The idea is that the THC should only be able to move the torch when
 * there is an arc OK signal or when PGDOWN/PGUP is pressed in the control
 * software.
 */
ISR(TIMER2_OVF_vect)
{
  uint8_t start = TCNT2;
  thc_ctrl_counter++;
  thc_pulse_counter++;

  if (thc_ctrl_counter >= THC_UPDATE_PERIOD_TICKS) {
    millis += THC_UPDATE_PERIOD_MS;
    thc_ctrl_counter = thc_ctrl_counter - THC_UPDATE_PERIOD_TICKS;
    if (machine_in_motion)
      thc_update(); // Evaluate what the THC should be doing
  }

  if (jog_z_action == DOWN) {
    if (!decelerating_up()) {
      decrement_speed(); // accelerate down
      set_dir_down();
      thc_step_down();
    }
  }
  else if (jog_z_action == UP) {
    if (!decelerating_down()) {
      increment_speed(); // accelerate up
      set_dir_up();
      thc_step_up();
    }
  }
  // jog_z_action == STAY
  else if (!decelerating_up() && !decelerating_down()) {
    thc_pulse_counter = 0;
  }
  // Try to compensate for irq overhead by adding TCNT2
  uint8_t load_val = min(TIM2_LOAD_VAL_MAX, (TCNT2 - start) + TIM2_LOAD_VAL);
  setup_timer_2(load_val);
}

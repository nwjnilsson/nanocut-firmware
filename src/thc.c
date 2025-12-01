#include "grbl.h"

#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

// Pulse period / acceleration related
#define SPEED_PROFILE_RESOLUTION 100 // Can be reduced to save memory
#define THC_UPDATE_PERIOD_TICKS 50
#define THC_UPDATE_PERIOD_MS 5
#define TIM2_LOAD_VAL 208
#define TIM2_LOAD_VAL_MAX 255
#define THC_PULSE_PERIOD_MAX 0x7FFFU
#define THC_SPEED_LIMIT (SPEED_PROFILE_RESOLUTION - 1)
#define ISR_TICKS_PER_MINUTE (1e4 * 60.f)

// Globals
volatile uint32_t millis         = 0;
volatile int      thc_action     = STAY;
volatile uint16_t thc_adc_value  = 0;
volatile uint16_t thc_adc_target = 0;

// Internal
static uint32_t arc_stablization_timer = 0;
static uint16_t thc_pulse_counter      = 0;
static uint16_t thc_ctrl_counter       = 0;
static uint16_t current_period         = 0;
static int32_t  z_axis_steps_limit;
// Array of pulse periods, slowest at index 0
static uint16_t speed_profile[SPEED_PROFILE_RESOLUTION];
// used to index the speed profile, sign indicates direction
static int16_t  thc_speed = 0;
static uint16_t accel_threshold;
static uint16_t accel_counter = 0;
static bool     thc_busy      = false;

enum ArcStatus { ARC_NOT_OK, ARC_OK = 1 << ARC_OK_BIT };

// static void debug_print(uint32_t num)
// {
//   while (num--) {
//     PORTD |= (1 << PD7);
//     __asm__("nop\n\t");
//     PORTD &= ~(1 << PD7);
//     __asm__("nop\n\t");
//   }
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

  // The assumption is made that max_travel is positive
  z_axis_steps_limit =
    (int32_t) settings.max_travel[Z_AXIS] * settings.steps_per_mm[Z_AXIS];

  current_period = THC_PULSE_PERIOD_MAX;

  thc_speed = 0;

  // Setup Timer2
  setup_timer_2(TIM2_LOAD_VAL);
}

bool check_step(int dir)
{
#ifdef HOMING_FORCE_SET_ORIGIN
  if (bit_istrue(settings.homing_dir_mask, bit(Z_AXIS))) {
    if (sys_position[Z_AXIS] + dir <= -z_axis_steps_limit && new_pos >= 0 &&
        thc_pulse_counter >= current_period)
      return true;
  }
  else {
    if (sys_position[Z_AXIS] + dir >= z_axis_steps_limit && new_pos <= 0 &&
        thc_pulse_counter >= current_period) {
      return true;
    }
  }
  return false;
#else
  return sys_position[Z_AXIS] + dir >= z_axis_steps_limit &&
         sys_position[Z_AXIS] + dir <= 0 && thc_pulse_counter >= current_period;
#endif
}

// If it is time to generate a step pulse, flip the step bit and updates the
// machine position. It is assumed that the Z-axis range is [-Z, 0]
void thc_step(int8_t heading)
{
  // Step
  thc_pulse_counter = thc_pulse_counter - current_period;
  STEP_PORT |= (1 << Z_STEP_BIT);
  delay_us(THC_PULSE_TIME_US);
  STEP_PORT &= ~(1 << Z_STEP_BIT);
  sys_position[Z_AXIS] += heading;
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
 *
 * NOTE: TIMER2_OVF_vect has a higher priority than other timer interrupts on
 * the AVR platform, so one has to be careful here, otherwise GRBL can break
 */
ISR(TIMER2_OVF_vect)
{
  // grbl stepper interrupts worst case execution time is 25us, which leaves
  // 7-8us up to its total period of 33us @ 30kHz. So to not break grbl, I aim
  // to have less than 8us execution time for the non-interruptible part.
  // ===========================================================================
  // Un-interruptible part, approx 1.25us
  // ===========================================================================
  // Reload timer
  if (unlikely(thc_busy)) {
    // Avoid nesting interrupts
    return;
  }
  // ===========================================================================
  // Interruptible part
  // ===========================================================================
  // If this part of the code is interrupted for long enough, it may result in
  // a missed step, but I think that's pretty unlikely. The "downtime" between
  // ISR ticks is almost 3 full periods of grbls stepper interrupt, so we
  // should be good.
  // ===========================================================================
  thc_busy = true;
  setup_timer_2(TIM2_LOAD_VAL);
  sei();
  thc_ctrl_counter++;
  thc_pulse_counter++;
  enum THC_Action action = STAY; // The action that will really be taken
  int             accel  = thc_action;
  switch (thc_action) {
    case WITHDRAW: {
      action = WITHDRAW + (((int) (thc_speed > 0)) << 1);
    } break;
    case APPROACH: {
      action = APPROACH - (((int) (thc_speed < 0)) << 1);
    } break;
    case STAY: {
      // Action stays STAY if thc_speed = 0
      action -= (int) (thc_speed < 0); // action = withdraw
      action += (int) (thc_speed > 0); // action = approach
      accel = -action;
    }
  }

  //  Set current speed
  int new_speed = thc_speed + accel;
  if (new_speed > -THC_SPEED_LIMIT && new_speed < THC_SPEED_LIMIT &&
      accel_counter++ >= accel_threshold) {
    thc_speed += accel;
    accel_counter  = 0;
    current_period = speed_profile[abs(thc_speed)];
  }

  thc_pulse_counter *= abs(action); // if STAY, don't generate pulses
  if (check_step(action)) {
    if ((action < 0) ^ (0x1 & (settings.dir_invert_mask >> Z_AXIS))) {
      DIRECTION_PORT |= (1 << Z_DIRECTION_BIT);
    }
    else {
      DIRECTION_PORT &= ~(1 << Z_DIRECTION_BIT);
    }
    thc_pulse_counter = thc_pulse_counter - current_period;
    STEP_PORT |= (1 << Z_STEP_BIT);
    delay_us(THC_PULSE_TIME_US);
    STEP_PORT &= ~(1 << Z_STEP_BIT);
    sys_position[Z_AXIS] += action;
  }

  if (thc_ctrl_counter >= THC_UPDATE_PERIOD_TICKS) {
    millis += THC_UPDATE_PERIOD_MS;
    thc_ctrl_counter = thc_ctrl_counter - THC_UPDATE_PERIOD_TICKS;
    // Using "likely" here to avoid penalties while machine is running
    if (likely(machine_in_motion)) {
      if (CONTROL_PIN & (1 << ARC_OK_BIT)) {
        // The assumption is made that arc okay is a dry close from the plasma
        // and the input pin is then pulled to ground.
        // We don't have an arc_ok signal. Stay and update timestamp.
        thc_action             = STAY;
        arc_stablization_timer = millis;
      }
      else if (thc_adc_target > THC_ON_THRESHOLD &&
               (millis - arc_stablization_timer) > ARC_STABILIZATION_TIME_MS) {
        // Update THC
        // We have an arc_ok signal, and THC is 'on'
        // Wait 3 seconds for arc voltage to stabalize
        if ((millis - arc_stablization_timer) > ARC_STABILIZATION_TIME_MS) {
          if (thc_adc_value > thc_adc_target + THC_ALLOWED_ERROR) {
            thc_action = WITHDRAW;
          }
          else if (thc_adc_value < thc_adc_target - THC_ALLOWED_ERROR) {
            thc_action = APPROACH;
          }
          else {
            thc_action = STAY;
          }
        }
      }
    }
  }
  thc_busy = false;
}

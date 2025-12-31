#include "grbl.h"

#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

// Pulse period / acceleration related
#define SPEED_PROFILE_RESOLUTION 50
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
static uint16_t   thc_adc_target = 0;

// ADC filtering (exponential moving average)
static volatile int32_t thc_adc_accumulator = 0;
// Filter constant: higher = slower filter. 3 gives us ~63% response in
// 2^3=8 samples, which is ~0.8 ms @ 10 kHz
#define ADC_FILTER_ALPHA 3

static uint16_t thc_on_threshold   = 0; // in adc ticks
static int      thc_allowed_error  = 0; // in adc ticks
static float    thc_adc_multiplier = 0.f;

// Internal
static uint32_t arc_stablization_timer = 0;
static uint16_t current_period         = 0;
static int32_t  z_axis_steps_limit;
// Array of pulse periods, slowest at index 0
static uint16_t speed_profile[SPEED_PROFILE_RESOLUTION];
// used to index the speed profile, sign indicates direction
static int16_t       thc_speed = 0;
static uint16_t      accel_threshold;
static uint16_t      accel_counter     = 0;
static uint16_t      thc_pulse_counter = 0;
static uint16_t      thc_ctrl_counter  = 0;
static volatile bool thc_busy          = false;

static void setup_timer_2(uint8_t val);
// static void debug_print(uint32_t num)
// {
//   while (num--) {
//     PORTD |= (1 << PD7);
//     __asm__("nop\n\t");
//     PORTD &= ~(1 << PD7);
//     __asm__("nop\n\t");
//   }
// }

// Runs every time settings are written to the Z-axis as well as on startup
void thc_init()
{
  thc_adc_multiplier = 1024.f / (5.f * settings.arc_voltage_divider);
  // Initialize filter accumulator to prevent startup transient
  thc_adc_accumulator = 0;
  // Speed profile calculation
  float speed_step = settings.max_rate[Z_AXIS] / SPEED_PROFILE_RESOLUTION;
  speed_profile[0] = THC_PULSE_PERIOD_MAX;
  for (int16_t i = 1; i < SPEED_PROFILE_RESOLUTION; ++i) {
    speed_profile[i] = min(
      THC_PULSE_PERIOD_MAX,
      (uint16_t) ceil((ISR_TICKS_PER_MINUTE) /
                      ((i + 1) * speed_step * settings.steps_per_mm[Z_AXIS])));
  }

  // After this many ISR ticks, we can increase/decrease the speed
  accel_threshold = (uint16_t) ceil(speed_step * ISR_TICKS_PER_MINUTE /
                                    settings.acceleration[Z_AXIS]);
  // The assumption is made that max_travel is positive
  z_axis_steps_limit =
    (int32_t) settings.max_travel[Z_AXIS] * settings.steps_per_mm[Z_AXIS];
  current_period = THC_PULSE_PERIOD_MAX;
  thc_speed      = 0;

  // clear ADLAR in ADMUX (0x7C) to right-adjust the result
  // ADCL will contain lower 8 bits, ADCH upper 2 (in last two bits)
  ADMUX &= 0b11011111;
  // Set REFS1..0 in ADMUX (0x7C) to change reference voltage to the
  // proper source (01)
  ADMUX |= 0b01000000;
  // Clear MUX3..0 in ADMUX (0x7C) in preparation for setting the analog
  // input
  ADMUX &= 0b11110000;
  // Set MUX3..0 in ADMUX (0x7C) to read from AD8 (Internal temp)
  // Do not set above 15! You will overrun other parts of ADMUX. A full
  // list of possible inputs is available in Table 24-4 of the ATMega328
  // datasheet
  ADMUX |= 8;
  // ADMUX |= B00001000; // Binary equivalent
  // Set ADEN in ADCSRA (0x7A) to enable the ADC.
  // Note, this instruction takes 12 ADC clocks to execute
  ADCSRA |= 0b10000000;
  // Set ADATE in ADCSRA (0x7A) to enable auto-triggering.
  ADCSRA |= 0b00100000;
  // Clear ADTS2..0 in ADCSRB (0x7B) to set trigger mode to free running.
  // This means that as soon as an ADC has finished, the next will be
  // immediately started.
  ADCSRB &= 0b11111000;
  // Set the Prescaler to 128 (16000KHz/128 = 125KHz)
  // Above 200KHz 10-bit results are not reliable.
  ADCSRA |= 0b00000111;
  // Set ADIE in ADCSRA (0x7A) to enable the ADC interrupt.
  // Without this, the internal interrupt will not trigger.
  ADCSRA |= 0b00001000;
  // Set ADSC in ADCSRA (0x7A) to start the ADC conversion
  ADCSRA |= 0b01000000;
  DDRC &= ~(1 << ARC_OK_BIT); // Set arc ok as input for Arc Ok
  PORTC |= (1 << ARC_OK_BIT); // Set arc ok internally pulled-up

  // Start first ADC conversion
  ADMUX = (ADMUX & 0xF0) | (CONTROL_ARC_VOLTAGE_PIN_BIT & 0x0F);
  ADCSRA |= (1 << ADSC);

  // Setup Timer2
  setup_timer_2(TIM2_LOAD_VAL);
}

float thc_get_voltage() { return ((float) thc_adc_value / thc_adc_multiplier); }

// Sets the target THC voltage from e.g $T=112.5
bool thc_set_voltage_target(float ftarget)
{
  int itarget = roundf(ftarget * thc_adc_multiplier);
  // If target is outside of 10 bit ADC range, return false so error can
  // be raised
  if (itarget < 0 || itarget > 1023) {
    return false;
  }
  thc_adc_target = (uint16_t) itarget;
  return true;
}

// Reloads the timer for overflow interrupts
static void setup_timer_2(uint8_t count)
{
  TCCR2B = 0x00;  // Disable Timer2 while we set it up
  TCNT2  = count; // Reset Timer Count
  TIFR2  = 0x00;  // Timer2 INT Flag Reg: Clear Timer Overflow Flag
  TIMSK2 = 0x01;  // Timer2 INT Reg: Timer2 Overflow Interrupt Enable
  TCCR2A = 0x00;  // Timer2 Control Reg A: Wave Gen Mode normal
  TCCR2B = (1 << CS21) | (1 << CS20); // Prescaler 32
}

static bool check_step(int dir)
{
  int32_t current_pos = sys_position[Z_AXIS];
#ifdef HOMING_FORCE_SET_ORIGIN
  if (bit_istrue(settings.homing_dir_mask, bit(Z_AXIS))) {
    return (current_pos + dir <= -z_axis_steps_limit) &&
           (current_pos + dir >= 0);
  }
  else {
    return (current_pos + dir >= z_axis_steps_limit) &&
           (current_pos + dir <= 0);
  }
  return false;
#else
  return (current_pos + dir >= z_axis_steps_limit) && (current_pos + dir <= 0);
#endif
}

/* ADC interrupt
 *
 * Exponential moving average filter: y[n] = y[n-1] + α(x[n] - y[n-1])
 * Using fixed-point arithmetic with α = 1/2^ADC_FILTER_ALPHA
 */
ISR(ADC_vect)
{
  // Must read low first
  uint16_t raw_adc = ADCL | (ADCH << 8); // 10 bits
  int32_t  target  = (int32_t) raw_adc << 8;
  thc_adc_accumulator += (target - thc_adc_accumulator) >> ADC_FILTER_ALPHA;
  thc_adc_value = (uint16_t) (thc_adc_accumulator >> 8);
  // Not needed because free-running mode is enabled.
  // Set ADSC in ADCSRA (0x7A) to start another ADC conversion
  // ADCSRA |= B01000000;
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
  thc_ctrl_counter++;
  thc_pulse_counter++;
  accel_counter++;
  sei();
  int16_t action = STAY; // The action that will really be taken
  int16_t accel  = thc_action;
  switch (thc_action) {
    case WITHDRAW: {
      action = WITHDRAW + (((int16_t) (thc_speed > 0)) << 1);
    } break;
    case APPROACH: {
      action = APPROACH - (((int16_t) (thc_speed < 0)) << 1);
    } break;
    case STAY: {
      // Action stays STAY if thc_speed = 0
      action -= (int16_t) (thc_speed < 0); // action = withdraw
      action += (int16_t) (thc_speed > 0); // action = approach
      accel = -action;
    }
  }
  thc_pulse_counter *= abs(action); // disable pulses if STAY

  //  Set current speed
  int16_t new_speed = thc_speed + accel;
  if (accel_counter >= accel_threshold) {
    accel_counter -= accel_threshold;
    if (new_speed > -THC_SPEED_LIMIT && new_speed < THC_SPEED_LIMIT) {
      thc_speed      = new_speed;
      current_period = speed_profile[abs(thc_speed)];
    }
  }

  if (thc_pulse_counter >= current_period) {
    thc_pulse_counter -= current_period;
    if (check_step(action)) {
      if ((action < 0) ^ (0x1 & (settings.dir_invert_mask >> Z_AXIS))) {
        DIRECTION_PORT |= (1 << Z_DIRECTION_BIT);
      }
      else {
        DIRECTION_PORT &= ~(1 << Z_DIRECTION_BIT);
      }
      STEP_PORT |= (1 << Z_STEP_BIT);
      delay_us(THC_PULSE_TIME_US);
      STEP_PORT &= ~(1 << Z_STEP_BIT);
      cli();
      sys_position[Z_AXIS] += action;
      sei();
    }
  }

  if (thc_ctrl_counter >= THC_UPDATE_PERIOD_TICKS) {
    thc_ctrl_counter = thc_ctrl_counter - THC_UPDATE_PERIOD_TICKS;
    millis += THC_UPDATE_PERIOD_MS;
    // Using "likely" here to avoid penalties while machine is running
    if (likely(machine_in_motion)) {
      if (CONTROL_PIN & (1 << ARC_OK_BIT)) {
        // The assumption is made that arc okay is a dry close from the plasma
        // and the input pin is then pulled to ground.
        // We don't have an arc_ok signal. Stay and update timestamp.
        thc_action             = STAY;
        arc_stablization_timer = millis;
      }
      else if (thc_adc_target > thc_on_threshold &&
               (millis - arc_stablization_timer) > ARC_STABILIZATION_TIME_MS) {
        // Update THC
        // We have an arc_ok signal, and THC is 'on'
        // Wait 3 seconds for arc voltage to stabalize
        if ((millis - arc_stablization_timer) > ARC_STABILIZATION_TIME_MS) {
          if (thc_adc_value > thc_adc_target + thc_allowed_error) {
            thc_action = WITHDRAW;
          }
          else if (thc_adc_value < thc_adc_target - thc_allowed_error) {
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

#include "grbl.h"

#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

// Pulse period / acceleration related
#define SPEED_PROFILE_RESOLUTION 40
#define THC_UPDATE_PERIOD_TICKS 50
#define THC_UPDATE_PERIOD_MS 5
#define TIM2_LOAD_VAL 208
#define TIM2_LOAD_VAL_MAX 255
#define THC_PULSE_PERIOD_MAX 0x7FFFU
#define THC_SPEED_LIMIT (SPEED_PROFILE_RESOLUTION - 1)
#define ISR_TICKS_PER_MINUTE (1e4 * 60.f)

// Globals
volatile uint16_t millis         = 0;
volatile uint16_t thc_adc_value  = 0;
static uint16_t   thc_adc_target = 0;

// ADC filtering: decimate 16:1 (~600 Hz), then 7-sample centered moving
// average. Group delay is constant at 3 decimated samples (5 ms). No tails.
#define DECIMATE_N 16
#define MA_N 4
static uint16_t adc_decimate_sum   = 0;
static uint8_t  adc_decimate_count = 0;
static uint16_t adc_ma_buf[MA_N];
static uint8_t  adc_ma_head = 0;
static uint16_t adc_ma_sum  = 0;

static uint16_t thc_on_threshold         = 0; // in adc ticks
static uint16_t thc_allowed_error        = 0; // in adc ticks
static uint16_t thc_allowed_error_medium = 0;
static uint16_t thc_allowed_error_fast   = 0;
static float    thc_adc_multiplier       = 0.f;
static uint8_t  thc_antidive_hold_ticks;
static uint16_t thc_prev_adc_value;

// Internal
static uint16_t arc_stablization_timer = 0;
static uint16_t current_period         = 0;
static int32_t  z_axis_steps_limit;
// Array of pulse periods, slowest at index 0
static uint16_t speed_profile[SPEED_PROFILE_RESOLUTION];
// used to index the speed profile, sign indicates direction
static int8_t          thc_speed         = 0;
static int8_t          thc_target_speed  = 0;
static volatile int8_t thc_manual_action = STAY;
static uint16_t        accel_threshold;
static uint16_t        accel_counter     = 0;
static uint16_t        thc_pulse_counter = 0;
static uint8_t         thc_ctrl_counter  = 0;
static volatile bool   thc_busy          = false;

// Arc okay debounce. The raw input is read every THC_UPDATE_PERIOD_MS. The
// filtered state flips only after ARC_OK_DEBOUNCE_SAMPLES consecutive reads
// agree on a new value. Stored as the "arc present" sense (i.e. opposite of
// the raw pin level, since the pin is pulled high when no arc is present).
static volatile bool arc_ok_filtered     = false;
static uint8_t       arc_ok_debounce_cnt = 0;

bool thc_arc_ok() { return arc_ok_filtered; }

static void setup_timer_2(uint8_t val);
static bool thc_manual_control_active()
{
  return !(sys.state & (STATE_CYCLE | STATE_HOLD | STATE_HOMING |
                        STATE_SAFETY_DOOR | STATE_SLEEP));
}

static int16_t thc_target_speed_from_action(int8_t action, uint8_t speed_index)
{
  switch (action) {
    case WITHDRAW:
      return -(int16_t) speed_index;
    case APPROACH:
      return (int16_t) speed_index;
    default:
      return 0;
  }
}

static int16_t thc_select_auto_target_speed(int16_t error, bool antidive_hold)
{
  uint16_t abs_error = abs(error);
  uint8_t  speed_index;

  if (abs_error <= thc_allowed_error) {
    return 0;
  }

  speed_index = THC_AUTO_SLOW_IDX;
  if (abs_error > thc_allowed_error_fast) {
    speed_index = THC_AUTO_FAST_IDX;
  }
  else if (abs_error > thc_allowed_error_medium) {
    speed_index = THC_AUTO_MED_IDX;
  }

  if (error > 0) {
    return (int16_t) speed_index;
  }
  if (antidive_hold) {
    return 0;
  }
  return -(int16_t) speed_index;
}
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
  thc_on_threshold =
    max(1u, (uint16_t) roundf(THC_ON_THRESHOLD_V * thc_adc_multiplier));
  thc_allowed_error =
    max(1u, (uint16_t) roundf(THC_ALLOWED_ERROR_V * thc_adc_multiplier));
  thc_allowed_error_medium = thc_allowed_error << 1;
  thc_allowed_error_fast   = thc_allowed_error << 2;
  // Initialize filter state
  adc_decimate_sum   = 0;
  adc_decimate_count = 0;
  adc_ma_head        = 0;
  adc_ma_sum         = 0;
  for (uint8_t i = 0; i < MA_N; i++)
    adc_ma_buf[i] = 0;
  thc_antidive_hold_ticks = 0;
  thc_prev_adc_value      = 0;
  thc_target_speed        = 0;
  thc_manual_action       = STAY;
  arc_ok_filtered         = false;
  arc_ok_debounce_cnt     = 0;
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

void thc_set_manual_action(enum THC_Action action)
{
  if (thc_manual_control_active()) {
    thc_manual_action = (int8_t) action;
  }
}

void thc_clear_manual_action() { thc_manual_action = STAY; }

static bool thc_feed_lock_active()
{
  plan_block_t* block = plan_get_current_block();
  if (block == NULL) {
    return false;
  }
  if (block->condition &
      (PL_COND_FLAG_RAPID_MOTION | PL_COND_FLAG_SYSTEM_MOTION)) {
    return true;
  }

  float nominal_rate = plan_compute_profile_nominal_speed(block);
  float actual_rate  = st_get_realtime_rate();
  if (nominal_rate <= 0.0f || actual_rate <= 0.0f) {
    return true;
  }

  return (actual_rate * 100.0f) <
         (nominal_rate * (float) THC_FEED_LOCKOUT_PERCENT);
}

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
 * Decimates 16:1 by summing raw samples, then feeds the average into a
 * 7-sample moving average ring buffer. Output rate ~600 Hz, group delay
 * is a constant 3 decimated samples (~5 ms). True symmetric FIR — no tails.
 */
ISR(ADC_vect)
{
  // Must read low first
  uint16_t raw_adc = ADCL | (ADCH << 8);
  adc_decimate_sum += raw_adc;
  if (++adc_decimate_count >= DECIMATE_N) {
    adc_decimate_count = 0;
    uint16_t decimated = adc_decimate_sum >> 4; // divide by 16
    adc_decimate_sum   = 0;
    adc_ma_sum -= adc_ma_buf[adc_ma_head];
    adc_ma_buf[adc_ma_head] = decimated;
    adc_ma_sum += decimated;
    adc_ma_head   = (adc_ma_head + 1 == MA_N) ? 0 : adc_ma_head + 1;
    thc_adc_value = adc_ma_sum / MA_N;
  }
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
  thc_busy = true;
  setup_timer_2(TIM2_LOAD_VAL);
  thc_ctrl_counter++;
  thc_pulse_counter++;
  accel_counter++;
  const uint16_t thc_adc_value_safe = thc_adc_value;
  sei();
  // ===========================================================================
  // Interruptible part
  // ===========================================================================
  // If this part of the code is interrupted for long enough, it may result in
  // a missed step, but I think that's pretty unlikely. The "downtime" between
  // ISR ticks is almost 3 full periods of grbls stepper interrupt, so we
  // should be good.
  // ===========================================================================
  int8_t accel = 0;
  if (thc_speed < thc_target_speed) {
    accel = 1;
  }
  else if (thc_speed > thc_target_speed) {
    accel = -1;
  }

  int8_t action = STAY; // The action that will really be taken
  if (thc_speed < 0) {
    action = WITHDRAW;
  }
  else if (thc_speed > 0) {
    action = APPROACH;
  }
  else {
    thc_pulse_counter = 0;
  }

  //  Set current speed
  int8_t new_speed = thc_speed + accel;
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

    // Debounce arc okay input. Pin is pulled high when no arc is present, so
    // an active arc corresponds to the pin reading low.
    bool arc_ok_raw = !(CONTROL_PIN & (1 << ARC_OK_BIT));
    if (arc_ok_raw == arc_ok_filtered) {
      arc_ok_debounce_cnt = 0;
    }
    else if (++arc_ok_debounce_cnt >= ARC_OK_DEBOUNCE_SAMPLES) {
      arc_ok_filtered     = arc_ok_raw;
      arc_ok_debounce_cnt = 0;
    }

    if (thc_manual_control_active()) {
      thc_target_speed =
        thc_target_speed_from_action(thc_manual_action, THC_SPEED_LIMIT);
      thc_antidive_hold_ticks = 0;
      thc_prev_adc_value      = thc_adc_value_safe;
    }
    else if (likely(machine_in_motion)) {
      thc_manual_action = STAY;
      if (!arc_ok_filtered) {
        // We don't have a (debounced) arc okay signal. Stay and reset the
        // arc stabilization timer so we restart the countdown the next time
        // the arc comes back.
        arc_stablization_timer  = millis;
        thc_target_speed        = 0;
        thc_antidive_hold_ticks = 0;
        thc_prev_adc_value      = thc_adc_value_safe;
      }
      else if (thc_adc_target > thc_on_threshold &&
               (millis - arc_stablization_timer) > ARC_STABILIZATION_TIME_MS) {
        // Update THC
        // We have an arc_ok signal, and THC is 'on'
        // Wait 3 seconds for arc voltage to stabalize
        if (thc_feed_lock_active()) {
          thc_target_speed        = 0;
          thc_antidive_hold_ticks = 0;
          thc_prev_adc_value      = thc_adc_value_safe;
        }
        else {
          int16_t adc_delta =
            (int16_t) thc_adc_value_safe - (int16_t) thc_prev_adc_value;
          thc_prev_adc_value = thc_adc_value_safe;

          if (adc_delta >
                ((int16_t) thc_allowed_error * THC_ANTIDIVE_DV_MULTIPLIER) &&
              thc_adc_value_safe > thc_adc_target + thc_allowed_error) {
            thc_antidive_hold_ticks =
              (uint8_t) (THC_ANTIDIVE_HOLD_MS / THC_UPDATE_PERIOD_MS);
          }
          else if (thc_antidive_hold_ticks) {
            thc_antidive_hold_ticks--;
          }

          thc_target_speed = thc_select_auto_target_speed(
            (int16_t) thc_adc_target - (int16_t) thc_adc_value_safe,
            thc_antidive_hold_ticks != 0);
        }
      }
      else {
        thc_target_speed        = 0;
        thc_antidive_hold_ticks = 0;
        thc_prev_adc_value      = thc_adc_value_safe;
      }
    }
    else {
      // Not in motion. Hold the arc stabilization timer so that the
      // ARC_STABILIZATION_TIME_MS countdown does not elapse during a stationary
      // period (e.g. pierce delay). The timer should only count while we are
      // both arc-OK and actually moving the torch along the cut.
      arc_stablization_timer  = millis;
      thc_manual_action       = STAY;
      thc_target_speed        = 0;
      thc_antidive_hold_ticks = 0;
      thc_prev_adc_value      = thc_adc_value_safe;
    }
  }
  thc_busy = false;
}

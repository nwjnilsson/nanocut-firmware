#ifndef thc_h
#define thc_h
/**
 * This file contains some defines that one may want to tune depending on your
 * setup
 */

// The duration that a step pulse is high. Normally around 10 microseconds, but
// I'm using a busy wait to generate the step pulse so it's better to make it as
// short as the hardware will allow.
// NOTE: I have not tested at what point things start to break down, but my
// recommendation is to stay below ~7us.
#define THC_PULSE_TIME_US 3

// The amount of time to wait (after cutting starts) before the THC is engaged.
// The arc stabilization time should be fetched from settings. The setting
// has been created for the control software but the implementation was never
// finished so it is not communicated to the controller. Since the ARC OK signal
// is connected to the feed hold, I suppose grbl won't start cutting until the
// arc is fairly stable anyway. Maybe this is why the original author settled
// for hardcoding this value here.
#define ARC_STABILIZATION_TIME_MS 3000

// The threshold for when the THC is considered on. The target value is set in
// the control software. The target value is in the range [0-1024]
// ('ADC ticks'), which is mapped to the real voltage (0-5V on the analog
// input).
#define THC_ON_THRESHOLD 150

// The allowed diff between the target value and the actual value (in ADC ticks)
#define THC_ALLOWED_ERROR 10

// -----------------------------------------------------------------------------
// Don't touch this part
// -----------------------------------------------------------------------------
#define ARC_OK_BIT CONTROL_FEED_HOLD_BIT

// Action in terms of movement relative to origin. "Approach" means moving to
// origin/home.
enum THC_Action { WITHDRAW = -1, STAY = 0, APPROACH = 1 };
void thc_init();
void thc_update();

#endif // thc_h

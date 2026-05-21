#ifndef thc_h
#define thc_h
#include "grbl.h"
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
#define ARC_STABILIZATION_TIME_MS 1500

// The threshold for when the THC is considered on and active. Volts.
#define THC_ON_THRESHOLD_V 25.0

// The allowed diff between the target value and the actual value. Volts.
#define THC_ALLOWED_ERROR_V 1.75

// Freeze THC when the real feed rate falls below this percent of the planned
// block speed. This prevents corner/slowdown dives.
#define THC_FEED_LOCKOUT_PERCENT 85

// Hold off over-voltage correction for a short time after a fast positive
// voltage step, which typically indicates crossing a void or slat.
#define THC_ANTIDIVE_HOLD_MS 100

// Positive ADC step threshold in multiples of thc_allowed_error.
#define THC_ANTIDIVE_DV_MULTIPLIER 12

// -----------------------------------------------------------------------------
// Don't touch this part
// -----------------------------------------------------------------------------
#define ARC_OK_BIT CONTROL_FEED_HOLD_BIT

// Action in terms of movement relative to origin. "Approach" means moving to
// origin/home.
enum THC_Action { WITHDRAW = -1, STAY = 0, APPROACH = 1 };
void  thc_init();
void  thc_update();
bool  thc_set_voltage_target(float input);
float thc_get_voltage();

#endif // thc_h

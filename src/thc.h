#ifndef thc_h
#define thc_h

#define THC_PULSE_TIME_US 5 // I'm using a busy wait to generate the step pulse, so this must be pretty short
#define ARC_OK_BIT CONTROL_FEED_HOLD_BIT

enum THC_Action { DOWN = -1, STAY = 0, UP = 1};

void thc_init();
void thc_update();

#endif // thc_h

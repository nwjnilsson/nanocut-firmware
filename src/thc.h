#ifndef thc_h
#define thc_h

#define THC_PULSE_TIME 10 // Same as GRBL default pulse duration

enum THC_Action { DOWN = -1, STAY = 0, UP = 1};

void thc_init();
void thc_update();

#endif // thc_h
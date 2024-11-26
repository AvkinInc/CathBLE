#ifndef TIMER_H_
#define TIMER_H_

#include "sl_sleeptimer.h"
#include "pins.h"
#include "em_gpio.h"

void sleepTimerLEDOff(void);

void sleepTimerBuzzerOff(void);

void sleepTimerSolenoidOff(void);

#endif /* TIMER_H_ */

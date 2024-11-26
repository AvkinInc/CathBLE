#ifndef ADC_H_
#define ADC_H_

/* GATT database */
#include "gatt_db.h"
#include "em_iadc.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_core.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "pins.h"
#include "pin_config.h"
// ^^ these are used for ADC


#include "sl_bluetooth.h"


void initIADC(void);

void readBatteryADC(void);
void readAnalogPin(void);


#endif /* ADC_H_ */

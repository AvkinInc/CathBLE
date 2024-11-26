/*
 * nvm.h
 *
 *  Created on: Jan 5, 2022
 *      Author: olivi
 */

#ifndef NVM_H_
#define NVM_H_

#include "nvm3.h"
#include "nvm3_hal_flash.h"
#include "nvm3_default.h"

/* GATT database */
#include "gatt_db.h"

#include "sl_bluetooth.h"

void nvm_lookUpAndSetName(void);
void nvm_saveName(unsigned char length, unsigned char *dataIn);

#endif /* NVM_H_ */

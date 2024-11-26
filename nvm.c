/*
 * nvm.c
 *
 *  Created on: Jan 5, 2022
 *      Author: olivi
 */

#include "nvm.h"

uint8_t const NAME_SUFFIX_KEY = 2;
uint8_t const PUMP_SUFFIX_KEY = 3;
size_t const BASE_NAME_LENGTH = 6;
size_t const PUMP_SPEED_LENGTH = 1;
unsigned char baseName[30] = {'A', 'v', 'c', 'a', 't', 'h'};
uint8_t pumpSpeed = 0x00;

void nvm_lookUpAndSetName(void){
    Ecode_t status;
    size_t numberOfObjects;
    uint32_t objectType;
    size_t dataLen;
  //  size_t nameLen;
    unsigned char dataOut[10]; // max 10
    unsigned char name[30]; // so it is large enough for the base and suffix

    // nvm3_default_common_linker.c
    status = nvm3_initDefault();
    if (status != ECODE_NVM3_OK) { // we get 0 now! it is ok
        // Handle error
    }
    numberOfObjects = nvm3_countObjects(nvm3_defaultHandle);
    if(numberOfObjects == 0){
        // nothing stored
        sl_bt_gatt_server_write_attribute_value (gattdb_device_name, 0, BASE_NAME_LENGTH, &baseName);
    }else{
        // Find size of data for object with key identifiers and read out
        status = nvm3_getObjectInfo(nvm3_defaultHandle, NAME_SUFFIX_KEY, &objectType, &dataLen);
        if (objectType == NVM3_OBJECTTYPE_DATA) {
            status = nvm3_readData(nvm3_defaultHandle, NAME_SUFFIX_KEY, &dataOut, dataLen);
         //   sl_bt_gatt_server_write_attribute_value (gattdb_device_name, 0, dataLen, &dataOut); // this works
          //  return;
            size_t const nameLen = (BASE_NAME_LENGTH + dataLen);
            for (size_t i = 0; i < BASE_NAME_LENGTH; i++){
                name[i] = baseName[i];
            }
            for (size_t i = BASE_NAME_LENGTH; i < nameLen; i++){
                 name[i] = dataOut[i - BASE_NAME_LENGTH]; // this works - 'Avband941'
            }
            status = sl_bt_gatt_server_write_attribute_value (gattdb_device_name, 0, nameLen, &name);
        }
    }
}

void nvm_saveName(unsigned char length, unsigned char *dataIn){
    Ecode_t status;

     // nvm3_default_common_linker.c
     status = nvm3_initDefault();
     if (status != ECODE_NVM3_OK) { // we get 0 now! it is ok
       //  setLedRed(100);
         // Handle error
     }
     status = nvm3_writeData(nvm3_defaultHandle, NAME_SUFFIX_KEY, dataIn, length);
     if (status != ECODE_NVM3_OK) {
         // Handle error
        // setLedRed(100);
     }else{
         NVIC_SystemReset(); // reset and boot to main() - the is ARM code
      //   status = nvm3_readData(nvm3_defaultHandle, NAME_SUFFIX_KEY, &dataOut, length); // &handle, 1, dataOut, dataLen1);
             //  setLedGreen(100);
          //     sl_bt_gatt_server_write_attribute_value (gattdb_device_name, 0, length, &dataOut);
     }
}

/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"
#include "pins.h"

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;
int BladderbuzzerIsBuzzed = 0;
int BladderbuzzerIsBuzzing = 0;
int FemGenbuzzerIsBuzzed = 0;
int FemGenbuzzerIsBuzzing = 0;
uint8_t holdBluetoothConnectionHandle = 1;

#define CB_VERSION_NUMBER 0xAA
#define BGM220_SW_VERSION_NUMBER 0xAA

#define BLUETOOTH_UNLOCK_CMD 0xEE
#define STOP_CMD 0x7A
#define BLUETOOTH_DISCONNECT_CMD 0x41 // 'A'
#define START_UP_LED_DELAY 500

bool bluetoothIsLocked = true;
bool stopAll = false;

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
  CMU_ClockEnable(cmuClock_GPIO, true);

  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1); //ensure device doesn't go to sleep

  //setup RGB LED pins to output
  GPIO_PinModeSet(RGB_Port,GREEN_Pin, gpioModePushPull, 1);
  GPIO_PinModeSet(RGB_Port, BLUE_Pin, gpioModePushPull, 1);
  GPIO_PinModeSet(RGB_Port, RED_Pin, gpioModePushPull, 1);

  //set all RGB pins low to initialize LED off
  GPIO_PinOutSet(RGB_Port, GREEN_Pin);
  GPIO_PinOutSet(RGB_Port, BLUE_Pin);
  GPIO_PinOutSet(RGB_Port, RED_Pin);

  //set all the Motor Driver pins to output
  GPIO_PinModeSet(DRIVER_PORT, STBY_PIN,gpioModePushPull, 1);
  GPIO_PinModeSet(DRIVER_PORT, PWM_PIN,gpioModePushPull, 1);
  GPIO_PinModeSet(AIN_PORT, AIN_PIN, gpioModePushPull, 1);

  //set all the pins to initialize low
  GPIO_PinOutClear(DRIVER_PORT, STBY_PIN);
  GPIO_PinOutClear(DRIVER_PORT, PWM_PIN);
  GPIO_PinOutClear(AIN_PORT, AIN_PIN);
  //enable the STBY Pin
  GPIO_PinOutSet(DRIVER_PORT, STBY_PIN);

  //setup connection LED
  GPIO_PinModeSet(CONNECTION_PORT, CONNECTION_PIN, gpioModePushPull, 1);

  //initialize connection LED off
  GPIO_PinOutClear(CONNECTION_PORT, CONNECTION_PIN);

  //setup the bladder sensor as an input
  GPIO_PinModeSet(BLADDER_PORT, BLADDER_PIN, gpioModeInput, 1);

  //setup the femgen pin for input
  GPIO_PinModeSet(FEMGEN_PORT, FEMGEN_PIN, gpioModeInput, 1);

  //setup the buzzer pin for output
  GPIO_PinModeSet(BUZZER_PORT, BUZZER_PIN, gpioModePushPull, 1);
  //initialize low
  GPIO_PinOutSet(BUZZER_PORT, BUZZER_PIN);

  initIADC();

  ledStartup(); //execute the startup light sequence

  nvm_lookUpAndSetName();


  uint8_t cb = (uint8_t)CB_VERSION_NUMBER;
  uint8_t sw = (uint8_t)BGM220_SW_VERSION_NUMBER;
  sl_bt_gatt_server_write_attribute_value (gattdb_cb_version, 0, 1, &cb);
  sl_bt_gatt_server_write_attribute_value (gattdb_sw_version, 0, 1, &sw);



}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////

  if(stopAll == false){
  //scan for the bladder sensor to be triggered
  if(GPIO_PinInGet(BLADDER_PORT, BLADDER_PIN) == 0){
      if((BladderbuzzerIsBuzzed == 0) && (BladderbuzzerIsBuzzing == 0)){
      //buzzer for 1 second only on the first press
      sl_sleeptimer_delay_millisecond (150); //account for bouncing
      BladderbuzzerIsBuzzed = 1;
      BladderbuzzerIsBuzzing = 1;
      GPIO_PinOutClear(BUZZER_PORT, BUZZER_PIN);
      sleepTimerBuzzerOff();
      }
      //solenoid turns on
      GPIO_PinOutSet(DRIVER_PORT, PWM_PIN);
      GPIO_PinOutSet(AIN_PORT, AIN_PIN);
  }
  else{
      if((BladderbuzzerIsBuzzed == 1) && (BladderbuzzerIsBuzzing == 1)){
      GPIO_PinOutClear(DRIVER_PORT, PWM_PIN);
      GPIO_PinOutClear(AIN_PORT, AIN_PIN);
      }
      //switch is released, reset buzzer
      BladderbuzzerIsBuzzed = 0;
      BladderbuzzerIsBuzzing = 0;
  }

  //scan for the fem gen sensor
  if(GPIO_PinInGet(FEMGEN_PORT, FEMGEN_PIN) == 0){
      if((FemGenbuzzerIsBuzzed == 0) && (FemGenbuzzerIsBuzzing == 0)){
            //buzzer for 1 second only on the first press
          sl_sleeptimer_delay_millisecond (150); //account for bouncing
           FemGenbuzzerIsBuzzed = 1;
           FemGenbuzzerIsBuzzing = 1;
           GPIO_PinOutClear(BUZZER_PORT, BUZZER_PIN);
           sleepTimerBuzzerOff();
           }
  }
  else{
      FemGenbuzzerIsBuzzed = 0;
      FemGenbuzzerIsBuzzing = 0;
  }



  readBatteryADC();
  }

}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;
  uint8_t system_id[8];

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:

      // Extract unique ID from BT Address.
      sc = sl_bt_system_get_identity_address(&address, &address_type);
      app_assert_status(sc);

      // Pad and reverse unique ID to get System ID.
      system_id[0] = address.addr[5];
      system_id[1] = address.addr[4];
      system_id[2] = address.addr[3];
      system_id[3] = 0xFF;
      system_id[4] = 0xFE;
      system_id[5] = address.addr[2];
      system_id[6] = address.addr[1];
      system_id[7] = address.addr[0];

      sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id,
                                                   0,
                                                   sizeof(system_id),
                                                   system_id);
      app_assert_status(sc);

      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      app_assert_status(sc);

      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
        advertising_set_handle,
        160, // min. adv. interval (milliseconds * 1.6)
        160, // max. adv. interval (milliseconds * 1.6)
        0,   // adv. duration
        0);  // max. num. adv. events
      app_assert_status(sc);
      // Start general advertising and enable connections.
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        sl_bt_advertiser_general_discoverable,
        sl_bt_advertiser_connectable_scannable);
      app_assert_status(sc);
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      GPIO_PinOutSet(CONNECTION_PORT, CONNECTION_PIN); //connection LED on
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      // Restart advertising after client has disconnected.
      GPIO_PinOutClear(CONNECTION_PORT, CONNECTION_PIN); //connection LED Off
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        sl_bt_advertiser_general_discoverable,
        sl_bt_advertiser_connectable_scannable);
      app_assert_status(sc);
      bluetoothIsLocked = true;
      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////
    case sl_bt_evt_gatt_server_attribute_value_id:

    case sl_bt_evt_gatt_server_user_write_request_id:
    //check if ble locked
    readPrimingSwitch(evt);
    break;


    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}


void readPrimingSwitch(sl_bt_msg_t *evt){
  if(evt->data.evt_gatt_server_attribute_value.value.data[0] == BLUETOOTH_UNLOCK_CMD){
      //unlock
      bluetoothIsLocked = false;
      return;
  }
  else if(bluetoothIsLocked){
      //stay locked
      return;
  }
  switch(evt->data.evt_gatt_server_user_write_request.characteristic){
    case gattdb_primingSwitch: {
      switch(evt->data.evt_gatt_server_attribute_value.value.data[0]){
        case 0x20: {
          //priming switch pressed, buzzer for 1 second, solenoid on for time period
          //solenoid on
          GPIO_PinOutSet(DRIVER_PORT, PWM_PIN);
          GPIO_PinOutSet(AIN_PORT, AIN_PIN);
          sleepTimerSolenoidOff();
          //buzzer
          GPIO_PinOutSet(BUZZER_PORT, BUZZER_PIN);
          GPIO_PinOutClear(RGB_Port, RED_Pin);
          sleepTimerBuzzerOff();
          sleepTimerLEDOff();
          break;
        }
        case BLUETOOTH_DISCONNECT_CMD: {
          //disconnect bluetooth, unused but reserved
          sl_bt_connection_close(holdBluetoothConnectionHandle);
          break;
        }
        case STOP_CMD: {
          stopAll = true;
          GPIO_PinOutSet(RGB_Port, GREEN_Pin);
          GPIO_PinOutSet(RGB_Port, BLUE_Pin);
          GPIO_PinOutSet(RGB_Port, RED_Pin);
          GPIO_PinOutClear(BUZZER_PORT, BUZZER_PIN);
          GPIO_PinOutClear(DRIVER_PORT, PWM_PIN);
          GPIO_PinOutClear(AIN_PORT, AIN_PIN);
        }
        default:{
          break;
          }
        }
      break;
      }
    // switch for set name
     case gattdb_renaming_char: {
              // takes in up to 10 characters for the name suffix
              // store length and data array in NVM
              unsigned char len = evt->data.evt_gatt_server_attribute_value.value.len;
              if(len > 10){
                  // too many
                  return;
              }
              unsigned char nameSuffix[10]; // set length to the max length
              // ^^ cannot initialize to a variable size 'len' - must be constant
              // copy over data
              // memcpy(evt->data.evt_gatt_server_attribute_value.value.data, nameSuffix, len);
              for (int i = 0; i < len; i++){
                  nameSuffix[i] = evt->data.evt_gatt_server_attribute_value.value.data[i];
              }
              // why does it say that nameSuffix is uint8_t[14427] now ._.
              nvm_saveName(len, &nameSuffix);
              break;
        }
    }
  }


void ledStartup(void){
  //turn off all LEDs
  GPIO_PinOutSet(RGB_Port, GREEN_Pin);
  GPIO_PinOutSet(RGB_Port, BLUE_Pin);
  GPIO_PinOutSet(RGB_Port, RED_Pin);
  //red
  GPIO_PinOutClear(RGB_Port, RED_Pin);
  sl_sleeptimer_delay_millisecond (START_UP_LED_DELAY);
  //yellow (red + green)
  GPIO_PinOutClear(RGB_Port, GREEN_Pin);
  sl_sleeptimer_delay_millisecond (350);
  //green
  GPIO_PinOutSet(RGB_Port, RED_Pin);
  GPIO_PinOutClear(RGB_Port, GREEN_Pin);
  sl_sleeptimer_delay_millisecond (350);
  //cyan
  GPIO_PinOutClear(RGB_Port, GREEN_Pin);
  GPIO_PinOutClear(RGB_Port, BLUE_Pin);
  sl_sleeptimer_delay_millisecond (350);
  //blue
  GPIO_PinOutSet(RGB_Port, GREEN_Pin);
  GPIO_PinOutClear(RGB_Port, BLUE_Pin);
  sl_sleeptimer_delay_millisecond (350);
  //purple
  GPIO_PinOutClear(RGB_Port, BLUE_Pin);
  GPIO_PinOutClear(RGB_Port, RED_Pin);
  sl_sleeptimer_delay_millisecond (350);
  //LEDs remain off
  GPIO_PinOutSet(RGB_Port, GREEN_Pin);
  GPIO_PinOutSet(RGB_Port, BLUE_Pin);
  GPIO_PinOutSet(RGB_Port, RED_Pin);
}






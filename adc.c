
#include "adc.h"

// Declare init structs
  IADC_Init_t init = IADC_INIT_DEFAULT;
  IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
  IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
  IADC_SingleInput_t BatteryReadingSingleInput = IADC_SINGLEINPUT_DEFAULT;
  IADC_SingleInput_t AnalogReadingSingleInput = IADC_SINGLEINPUT_DEFAULT;


  volatile uint16_t sample;
  volatile uint8_t battery;
  float vRead;

  volatile uint32_t analog;
  volatile uint8_t analog2;

  #define HYSTERESIS_ON .10
  #define HYSTERESIS_OFF 0

  float green_blue_hysteresis = HYSTERESIS_OFF;
  float blue_red_hysteresis = HYSTERESIS_OFF;


  void initIADC (void){
   CMU_ClockEnable(cmuClock_IADC0, true);
   CMU_ClockEnable(cmuClock_GPIO, true);


   //reset ADC
   IADC_reset(IADC0);


   //Set clock
   CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_FSRCO);  // FSRCO - 20MHz

   // Modify init structs and initialize
   init.warmup = iadcWarmupKeepWarm;

   // Set the HFSCLK prescale value here
   init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);

   // Configuration 0 is used by both scan and single conversions by default
   // Use unbuffered AVDD (supply voltage in mV) as reference
   // Resolution is not configurable directly but is based on the
   // selected oversampling ratio (osrHighSpeed), which defaults to
   // 2x and generates 12-bit results.
   initAllConfigs.configs[0].reference = iadcCfgReferenceVddx;
   initAllConfigs.configs[0].vRef = 3300;
   initAllConfigs.configs[0].osrHighSpeed = iadcCfgOsrHighSpeed2x;

   // Divides CLK_SRC_ADC to set the CLK_ADC frequency
   initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                                                      CLK_ADC_FREQ,
                                                                      0,
                                                                      iadcCfgModeNormal,
                                                                      init.srcClkPrescale);

   BatteryReadingSingleInput.posInput = IADC_BATTERY_READING_PORT_PIN;
   BatteryReadingSingleInput.negInput = iadcNegInputGnd;
   AnalogReadingSingleInput.posInput = IADC_ANALOG_READING_PORT_PIN;
   AnalogReadingSingleInput.negInput = iadcNegInputGnd;




 //init ADC
 IADC_init(IADC0, &init, &initAllConfigs);

  }


void readBatteryADC(void){
  // Initialize the Single conversion inputs
  if(GPIO_PinInGet(BLADDER_PORT, BLADDER_PIN) == 0){
     return;
  }


  IADC_initSingle(IADC0, &initSingle, &BatteryReadingSingleInput);
  //Analog bus for ADC0 input
  GPIO->CDBUSALLOC |= IADC_BATTERY_READING_BUSALLOC;

  IADC_command(IADC0, iadcCmdStartSingle);
  // Wait for conversion to be complete
  while((IADC0->STATUS & (_IADC_STATUS_CONVERTING_MASK
                              | _IADC_STATUS_SINGLEFIFODV_MASK)) != IADC_STATUS_SINGLEFIFODV); //while combined status bits 8 & 6 don't equal 1 and 0 respectively

   // Get ADC result
   sample = IADC_pullSingleFifoResult(IADC0).data;

   uint8_t *  put_sent_len_here; // not used, but needed for a write

   //send Result to Server
   sl_bt_gatt_server_write_attribute_value(gattdb_battery_Reading, 0, 4, &sample);
   sl_bt_gatt_server_notify_all(gattdb_battery_Reading, 4, &sample);
   //sl_bt_gatt_server_write_attribute_value(gattdb_battery_Reading, 0, 2, &sample2);
   // sl_bt_gatt_server_notify_all(gattdb_battery_Reading, 2, &sample2);

    vRead = ((sample * 3.3)/4096)*4;


    if((vRead + green_blue_hysteresis)> 11 && (vRead + green_blue_hysteresis) < 12.6){
        GPIO_PinOutSet(RGB_Port, RED_Pin);
        GPIO_PinOutSet(RGB_Port, BLUE_Pin);
        GPIO_PinOutClear(RGB_Port, GREEN_Pin);
        green_blue_hysteresis = HYSTERESIS_ON;
    }
    else if((vRead + blue_red_hysteresis) > 10 && (vRead + blue_red_hysteresis) < 11){
        GPIO_PinOutSet(RGB_Port, RED_Pin);
        GPIO_PinOutSet(RGB_Port, GREEN_Pin);
        GPIO_PinOutClear(RGB_Port, BLUE_Pin);
        blue_red_hysteresis = HYSTERESIS_ON;
    }
    else if(vRead < 10){
        GPIO_PinOutSet(RGB_Port, BLUE_Pin);
        GPIO_PinOutSet(RGB_Port, GREEN_Pin);
        GPIO_PinOutClear(RGB_Port, RED_Pin);
    }


     }

void readAnalogPin(void){
  //initialize the single conversion inputs
  IADC_initSingle(IADC0, &initSingle, &AnalogReadingSingleInput);

  //Analog bus for ADC0 input
  GPIO->CDBUSALLOC |= IADC_ANALOG_READING_BUSALLOC;

  IADC_command(IADC0, iadcCmdStartSingle);
  //wait for conversion to complete
  while((IADC0->STATUS & (_IADC_STATUS_CONVERTING_MASK
                               | _IADC_STATUS_SINGLEFIFODV_MASK)) != IADC_STATUS_SINGLEFIFODV); //while combined status bits 8 & 6 don't equal 1 and 0 respectively


  //get adc result
  analog = IADC_pullSingleFifoResult(IADC0).data;


  uint8_t * put_sent_len_here;

  //send result to server
  sl_bt_gatt_server_write_attribute_value(gattdb_analog_Reading, 0, 4, &analog);
  sl_bt_gatt_server_notify_all(gattdb_analog_Reading, 4, &analog);



  vRead = ((analog * 3.3)/4096)*4;


      if((vRead + green_blue_hysteresis)> 11 && (vRead + green_blue_hysteresis) < 12.6){
          GPIO_PinOutSet(RGB_Port, RED_Pin);
          GPIO_PinOutSet(RGB_Port, BLUE_Pin);
          GPIO_PinOutClear(RGB_Port, GREEN_Pin);
          green_blue_hysteresis = HYSTERESIS_ON;
      }
      else if((vRead + blue_red_hysteresis) > 10 && (vRead + blue_red_hysteresis) < 11){
          GPIO_PinOutSet(RGB_Port, RED_Pin);
          GPIO_PinOutSet(RGB_Port, GREEN_Pin);
          GPIO_PinOutClear(RGB_Port, BLUE_Pin);
          blue_red_hysteresis = HYSTERESIS_ON;
      }
      else if(vRead < 10){
          GPIO_PinOutSet(RGB_Port, BLUE_Pin);
          GPIO_PinOutSet(RGB_Port, GREEN_Pin);
          GPIO_PinOutClear(RGB_Port, RED_Pin);
      }


       }

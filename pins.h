#ifndef PINS_H_
#define PINS_H_

#define PCB_BOARD


#ifdef DEMO_BOARD

  //rgb LED
  #define RGB_PORT  gpioPortB
  #define GREEN_PIN 1
  #define BLUE_PIN 2
  #define RED_PIN 3

  //Connection LED
  #define CONNECTION_PORT gpioPortB
  #define CONNECTION_PIN 4

  //Motor Driver
  #define DRIVER_PORT gpioPortC
  #define STBY_PIN 3
  #define PWM_PIN 2
  //AIN1
  #define AIN_PORT gpioPortC
  #define AIN_PIN 6

  //Bladder Sensor
  #define BLADDER_PORT gpioPortC
  #define BLADDER_PIN 1

  //Female Genital Sensor
  #define FEMGEN_PORT gpioPortB
  #define FEMGEN_PIN 0

  //Buzzer
  #define BUZZER_PORT gpioPortD
  #define BUZZER_PIN 3

  //[IADC0]$
  #define IADC_BATTERY_READING_PORT_PIN     iadcPosInputPortDPin2;
  #define IADC_BATTERY_READING_BUS          CDBUSALLOC
  #define IADC_BATTERY_READING_BUSALLOC     GPIO_CDBUSALLOC_CDEVEN0_ADC0


#else

  //RGB LED
#define RGB_Port gpioPortA
#define BLUE_Pin 6
#define RED_Pin 7
#define GREEN_Pin 5

  //Connection LED
  #define CONNECTION_PORT gpioPortB
  #define CONNECTION_PIN 0

  //Motor Driver
  #define DRIVER_PORT gpioPortC
  #define STBY_PIN 1
  #define PWM_PIN 3
  //AIN1
  #define AIN_PORT gpioPortA
  #define AIN_PIN 0

  //Bladder Sensor
  #define BLADDER_PORT gpioPortC
  #define BLADDER_PIN 5


  //Female Genital Sensor
  #define FEMGEN_PORT gpioPortC
  #define FEMGEN_PIN 4

  //Buzzer
  #define BUZZER_PORT gpioPortB
  #define BUZZER_PIN 3

  // [IADC0]$
#define IADC_BATTERY_READING_PORT_PIN iadcPosInputPortCPin0;
#define IADC_BATTERY_READING_BUS CDBUSALLOC;
#define IADC_BATTERY_READING_BUSALLOC GPIO_CDBUSALLOC_CDEVEN0_ADC0;

#define IADC_ANALOG_READING_PORT_PIN iadcPosInputPortDPin3;
#define IADC_ANALOG_READING_BUS CDBUSALLOC;
#define IADC_ANALOG_READING_BUSALLOC GPIO_CDBUSALLOC_CDODD0_ADC0;

#endif /* DEMO_BOARD */

#endif /*PINS_H_ */

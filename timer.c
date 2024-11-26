#include "timer.h"





#define DELAY_LED_OFF 500
#define DELAY_BUZZER_OFF 1000
#define DELAY_SOLENOID_OFF 5000

sl_sleeptimer_timer_handle_t led_timer;
sl_sleeptimer_timer_handle_t buzzer_timer;
sl_sleeptimer_timer_handle_t solenoid_timer;

//callback define

static void sleepTimerLEDOff_timeout(sl_sleeptimer_timer_handle_t *handle,
                       void *data);

static void sleepTimerBuzzerOff_timeout(sl_sleeptimer_timer_handle_t *handle,
                       void *data);

static void sleepTimerSolenoidOff_timeout(sl_sleeptimer_timer_handle_t *handle, void *data);

//start timer function

void sleepTimerLEDOff(void){
  // stop any running timer
   sl_sleeptimer_stop_timer (&led_timer);
   sl_sleeptimer_start_timer_ms (&led_timer,
                                  DELAY_LED_OFF,
                                  sleepTimerLEDOff_timeout,
                                  NULL,
                                  0,
                                  SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
 }

void sleepTimerBuzzerOff(void){
  sl_sleeptimer_stop_timer (&buzzer_timer);
  sl_sleeptimer_start_timer_ms (&buzzer_timer,
                                DELAY_LED_OFF,
                                sleepTimerBuzzerOff_timeout,
                                NULL,
                                0,
                                SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
}

void sleepTimerSolenoidOff(void){
  sl_sleeptimer_stop_timer (&solenoid_timer);
  sl_sleeptimer_start_timer_ms (&solenoid_timer,
                                DELAY_SOLENOID_OFF,
                                sleepTimerSolenoidOff_timeout,
                                NULL,
                                0,
                                SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
}

static void sleepTimerLEDOff_timeout(sl_sleeptimer_timer_handle_t *handle,
                                      void *data){
    (void)&handle;
    (void)&data;
    //turn off the LED
    GPIO_PinOutSet(RGB_Port, GREEN_Pin);
    GPIO_PinOutSet(RGB_Port, BLUE_Pin);
    GPIO_PinOutSet(RGB_Port, RED_Pin);

}

static void sleepTimerBuzzerOff_timeout(sl_sleeptimer_timer_handle_t *handle,
                                        void *data){
  (void)&handle;
  (void)&data;
  //turn off the buzzer
  GPIO_PinOutSet(BUZZER_PORT, BUZZER_PIN);

}

static void sleepTimerSolenoidOff_timeout(sl_sleeptimer_timer_handle_t *handle,
                                          void *data){
  (void)&handle;
  (void)&data;
  //turn off the solenoid
  GPIO_PinOutClear(DRIVER_PORT, PWM_PIN);
  GPIO_PinOutClear(AIN_PORT, AIN_PIN);
}


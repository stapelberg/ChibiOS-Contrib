/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include "ch_test.h"
#include "rt_test_root.h"
#include "oslib_test_root.h"
#include "chprintf.h"
#include <string.h>

#include "usbcfg.h"

/*
 * LED blinker thread.
 */
static THD_WORKING_AREA(waThread1, 64);
static THD_FUNCTION(Thread1, arg) {

    (void)arg;
    chRegSetThreadName("LEDBlinker");
    while (true) {
      palTogglePad(TEENSY_PIN13_IOPORT, TEENSY_PIN13);
      chThdSleepSeconds(1);
    }
}

extern void printf_debug(const char *format, ...);


/*
 * Application entry point.
 */
int main(void) {
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();

  // This is still visible, but then the device seems to restart
  printf_debug("halInit done\n");

  chSysInit();
  
  //printf_debug("chSysInit done\n");  
  //printf_debug("IRQn = %d\n", MIMXRT1062_SERIAL3_IRQ_VECTOR);  

  #define MYSERIAL &SD4
  //#define MYSERIAL &SD1
  
  /*
   * Activates serial 1 (UART0) using the driver default configuration.
   */
  sdStart(MYSERIAL, NULL);
  sdWrite(MYSERIAL, (unsigned char*)"Hello world!\r\n", strlen("Hello world!\r\n"));

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

#if 1 /* USB serial */
  /*
   * Initializes a serial-over-USB CDC driver.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1500);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);
#endif  

  // Temporarily disabled to make serial debug available without interruption:

  //test_execute((BaseSequentialStream *)MYSERIAL, &rt_test_suite);
  //test_execute((BaseSequentialStream *)MYSERIAL, &oslib_test_suite);
  while (true) {
    chThdSleepSeconds(1);
  }

#if 0
  while (true) {
      	  GPIO7->DR_SET = (1<<3); // digitalWrite(13, HIGH);
      //palTogglePad(TEENSY_PIN13_IOPORT, TEENSY_PIN13);
	  systime_t before = chVTGetSystemTimeX();
	  delay(600000000); // 1s
	  systime_t after = chVTGetSystemTimeX();
	  //printf_debug("systicks = %d\n", after-before);
	  chThdSleepSeconds(1);
	  GPIO7->DR_CLEAR = (1<<3); // digitalWrite(13, LOW);      
	  chThdSleepSeconds(1);
	  //delay(600000000); // 1s
      //chThdSleepMilliseconds(1000);
  }
#endif
  return 0;
}

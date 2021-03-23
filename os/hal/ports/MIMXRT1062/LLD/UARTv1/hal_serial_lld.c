/*
    ChibiOS - Copyright (C) 2013-2015 Fabio Utzig
              Copyright (C) 2017 Wim Lewis

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

/**
 * @file    UARTv1/hal_serial_lld.c
 * @brief   Kinetis KL2x Serial Driver subsystem low level driver source.
 *
 * @addtogroup SERIAL
 * @{
 */

#include "osal.h"
#include "hal.h"

#include "fsl_lpuart.h"

#define printf_init()
#define printf(...)
#define printf_debug_init()
#define printf_debug(...)

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   SD1 driver identifier.
 */
#if MIMXRT1062_SERIAL_USE_UART0 || defined(__DOXYGEN__)
SerialDriver SD1;
#endif

#if MIMXRT1062_SERIAL_USE_UART1 || defined(__DOXYGEN__)
SerialDriver SD2;
#endif

#if MIMXRT1062_SERIAL_USE_UART2 || defined(__DOXYGEN__)
SerialDriver SD3;
#endif

#if MIMXRT1062_SERIAL_USE_UART3 || defined(__DOXYGEN__)
SerialDriver SD4;
#endif

#if MIMXRT1062_SERIAL_USE_UART4 || defined(__DOXYGEN__)
SerialDriver SD5;
#endif

#if MIMXRT1062_SERIAL_USE_UART5 || defined(__DOXYGEN__)
SerialDriver SD6;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver default configuration.
 */
static const SerialConfig default_config = {
  SERIAL_DEFAULT_BITRATE
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/
/**
 * @brief   Error handling routine.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] isr       UART s1 register value
 */
static void set_error(SerialDriver *sdp, uint8_t s1) {
  eventflags_t sts = 0;

  if (s1 & LPUART_STAT_OR_MASK)
    sts |= SD_OVERRUN_ERROR;
  if (s1 & LPUART_STAT_PF_MASK)
    sts |= SD_PARITY_ERROR;
  if (s1 & LPUART_STAT_FE_MASK)
    sts |= SD_FRAMING_ERROR;
  if (s1 & LPUART_STAT_NF_MASK)
    sts |= SD_NOISE_ERROR;
  osalSysLockFromISR();
  chnAddFlagsI(sdp, sts);
  osalSysUnlockFromISR();
}

/**
 * @brief   Common error IRQ handler.
 *
 * @param[in] sdp       communication channel associated to the UART
 */
static void serve_error_interrupt(SerialDriver *sdp) {
  UART_w_TypeDef *u = &(sdp->uart);
  uint8_t s1 = u->lpuart_p->STAT;

  if(s1 & LPUART_STAT_IDLE_MASK) {
    
    (void)u->lpuart_p->DATA; // TODO: to clear IDLE, write logic 1 to the IDLE flag
  }

  if(s1 & (LPUART_STAT_OR_MASK | LPUART_STAT_NF_MASK | LPUART_STAT_FE_MASK | LPUART_STAT_PF_MASK)) {
    set_error(sdp, s1);
    (void)u->lpuart_p->DATA; // TODO: write logic 1 to clear
  }
}

/**
 * @brief   Common IRQ handler.
 * @note    Tries hard to clear all the pending interrupt sources, we don't
 *          want to go through the whole ISR and have another interrupt soon
 *          after.
 *
 * @param[in] sdp       communication channel associated to the UART
 */
static void serve_interrupt(SerialDriver *sdp) {
  UART_w_TypeDef *u = &(sdp->uart);
  uint8_t s1 = u->lpuart_p->STAT;

  if (s1 & LPUART_STAT_RDRF_MASK) {
    osalSysLockFromISR();
    if (iqIsEmptyI(&sdp->iqueue))
      chnAddFlagsI(sdp, CHN_INPUT_AVAILABLE);
    if (iqPutI(&sdp->iqueue, u->lpuart_p->DATA) < Q_OK)
      chnAddFlagsI(sdp, SD_OVERRUN_ERROR);
    osalSysUnlockFromISR();
  }

  if (s1 & LPUART_STAT_TDRE_MASK) {
    msg_t b;

    osalSysLockFromISR();
    b = oqGetI(&sdp->oqueue);
    osalSysUnlockFromISR();

    if (b < Q_OK) {
      osalSysLockFromISR();
      chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
      osalSysUnlockFromISR();
      // Disable Transfer Interrupt
      u->lpuart_p->CTRL &= ~LPUART_CTRL_TIE_MASK;
    } else {
      u->lpuart_p->DATA = b;
      // Ensure all data in the transmit buffer is sent out to bus. See
      // https://github.com/NXPmicro/mcux-sdk/blob/66fdc6ff81de5334ea252b50977c7d277b6c5d77/drivers/lpuart/fsl_lpuart.c#L912
      while ((u->lpuart_p->STAT & LPUART_STAT_TC_MASK) == 0) {}
    }
  }

  serve_error_interrupt(sdp);
}

#if 0
/**
 * @brief   Attempts a TX preload
 */
static void preload(SerialDriver *sdp) {
  UART_w_TypeDef *u = &(sdp->uart);
  uint8_t s1 = u->lpuart_p->STAT;

  if (s1 & LPUART_STAT_TDRE_MASK) {
    msg_t b = oqGetI(&sdp->oqueue);
    if (b < Q_OK) {
      chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
      return;
    }
    u->lpuart_p->DATA = b;
    u->lpuart_p->CTRL |= LPUART_CTRL_TIE(1);
  }
}
#endif

/**
 * @brief   Driver output notification.
 */
static void notify(io_queue_t *qp)
{
  SerialDriver *sdp = qp->q_link;
  UART_w_TypeDef *u = &(sdp->uart);

  //printf_debug("notify()\n");
  //preload(qp->q_link);

  // TODO: figure out why the interrupt code path does not work
  // Blockingly drain the queue into the serial buffer:
  for (;;) {
    msg_t b = oqGetI(&sdp->oqueue);
    if (b < Q_OK) {
      return;
    }
    LPUART_WriteBlocking(u->lpuart_p, (uint8_t *)&b, 1);
  }
}

/**
 * @brief   Common driver initialization, except LP.
 */
static void sd_lld_init_driver(SerialDriver *SDn, LPUART_Type *UARTn) {
  /* Driver initialization.*/
  sdObjectInit(SDn, NULL, notify);
  SDn->uart.lpuart_p = UARTn;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if MIMXRT1062_SERIAL_USE_UART0 || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(KINETIS_SERIAL0_IRQ_VECTOR) {
  OSAL_IRQ_PROLOGUE();
  serve_interrupt(&SD1);
  OSAL_IRQ_EPILOGUE();
}
#endif

#if MIMXRT1062_SERIAL_USE_UART1 || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(KINETIS_SERIAL1_IRQ_VECTOR) {
  OSAL_IRQ_PROLOGUE();
  serve_interrupt(&SD2);
  OSAL_IRQ_EPILOGUE();
}
#endif

#if MIMXRT1062_SERIAL_USE_UART2 || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(KINETIS_SERIAL2_IRQ_VECTOR) {
  OSAL_IRQ_PROLOGUE();
  serve_interrupt(&SD3);
  OSAL_IRQ_EPILOGUE();
}
#endif

#if MIMXRT1062_SERIAL_USE_UART3 || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(MIMXRT1062_SERIAL3_IRQ_VECTOR) {
  OSAL_IRQ_PROLOGUE();
  serve_interrupt(&SD4);
  OSAL_IRQ_EPILOGUE();
}
#endif

#if MIMXRT1062_SERIAL_USE_UART4 || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(KINETIS_SERIAL4_IRQ_VECTOR) {
  OSAL_IRQ_PROLOGUE();
  serve_interrupt(&SD5);
  OSAL_IRQ_EPILOGUE();
}
#endif

#if MIMXRT1062_SERIAL_USE_UART5 || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(KINETIS_SERIAL5_IRQ_VECTOR) {
  OSAL_IRQ_PROLOGUE();
  serve_interrupt(&SD6);
  OSAL_IRQ_EPILOGUE();
}
#endif

#if MIMXRT1062_HAS_SERIAL_ERROR_IRQ

#if MIMXRT1062_SERIAL_USE_UART0 || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(KINETIS_SERIAL0_ERROR_IRQ_VECTOR) {
  OSAL_IRQ_PROLOGUE();
#if defined(KL2x)
  serve_error_interrupt_uart0();
#else
  serve_error_interrupt(&SD1);
#endif
  OSAL_IRQ_EPILOGUE();
}
#endif

#if MIMXRT1062_SERIAL_USE_UART1 || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(KINETIS_SERIAL1_ERROR_IRQ_VECTOR) {
  OSAL_IRQ_PROLOGUE();
  serve_error_interrupt(&SD2);
  OSAL_IRQ_EPILOGUE();
}
#endif

#if MIMXRT1062_SERIAL_USE_UART2 || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(KINETIS_SERIAL2_ERROR_IRQ_VECTOR) {
  OSAL_IRQ_PROLOGUE();
  serve_error_interrupt(&SD3);
  OSAL_IRQ_EPILOGUE();
}
#endif

#if MIMXRT1062_SERIAL_USE_UART3 || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(KINETIS_SERIAL3_ERROR_IRQ_VECTOR) {
  OSAL_IRQ_PROLOGUE();
  serve_error_interrupt(&SD4);
  OSAL_IRQ_EPILOGUE();
}
#endif

#if MIMXRT1062_SERIAL_USE_UART4 || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(KINETIS_SERIAL4_ERROR_IRQ_VECTOR) {
  OSAL_IRQ_PROLOGUE();
  serve_error_interrupt(&SD5);
  OSAL_IRQ_EPILOGUE();
}
#endif

#if MIMXRT1062_SERIAL_USE_UART5 || defined(__DOXYGEN__)
OSAL_IRQ_HANDLER(KINETIS_SERIAL5_ERROR_IRQ_VECTOR) {
  OSAL_IRQ_PROLOGUE();
  serve_error_interrupt(&SD6);
  OSAL_IRQ_EPILOGUE();
}
#endif

#endif /* KINETIS_HAS_SERIAL_ERROR_IRQ */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level serial driver initialization.
 *
 * @notapi
 */
void sd_lld_init(void) {

#if MIMXRT1062_SERIAL_USE_UART0
  /* Driver initialization.*/
  sd_lld_init_driver(&SD1, LPUART1);
#endif /* MIMXRT1062_SERIAL_USE_UART0 */

#if MIMXRT1062_SERIAL_USE_UART1
  sd_lld_init_driver(&SD2, LPUART2);
#endif /* MIMXRT1062_SERIAL_USE_UART1 */

#if MIMXRT1062_SERIAL_USE_UART2
  sd_lld_init_driver(&SD3, LPUART3);
#endif /* MIMXRT1062_SERIAL_USE_UART2 */

#if MIMXRT1062_SERIAL_USE_UART3
  sd_lld_init_driver(&SD4, LPUART4);
#endif /* MIMXRT1062_SERIAL_USE_UART3 */

#if MIMXRT1062_SERIAL_USE_UART4
  sd_lld_init_driver(&SD5, LPUART5);
#endif /* MIMXRT1062_SERIAL_USE_UART4 */

#if MIMXRT1062_SERIAL_USE_UART5
  sd_lld_init_driver(&SD6, LPUART6);
#endif /* MIMXRT1062_SERIAL_USE_UART5 */
}

// TODO: UartSrcFreq copied from
// https://github.com/adafruit/circuitpython/blob/main/ports/mimxrt10xx/supervisor/serial.c
// — is this correct/applicable?
static uint32_t UartSrcFreq(void) {
    uint32_t freq;

    /* To make it simple, we assume default PLL and divider settings, and the only variable
         from application is use PLL3 source or OSC source */
    /* PLL3 div6 80M */
    if (CLOCK_GetMux(kCLOCK_UartMux) == 0) {
        freq = (CLOCK_GetPllFreq(kCLOCK_PllUsb1) / 6U) / (CLOCK_GetDiv(kCLOCK_UartDiv) + 1U);
    } else {
        freq = CLOCK_GetOscFreq() / (CLOCK_GetDiv(kCLOCK_UartDiv) + 1U);
    }

    return freq;
}

/**
 * @brief   Low level serial driver configuration and (re)start.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration.
 *                      If this parameter is set to @p NULL then a default
 *                      configuration is used.
 *
 * @notapi
 */
void sd_lld_start(SerialDriver *sdp, const SerialConfig *config) {

  if (config == NULL)
    config = &default_config;

  if (sdp->state == SD_STOP) {
    /* Enables the peripheral.*/

    lpuart_config_t uconfig;

    LPUART_GetDefaultConfig(&uconfig);
    uconfig.baudRate_Bps = config->sc_speed;
    uconfig.enableTx = true;
    uconfig.enableRx = true;

#if MIMXRT1062_SERIAL_USE_UART0
    if (sdp == &SD1) {
      LPUART_Init(LPUART1, &uconfig, UartSrcFreq());
    }
#endif /* MIMXRT1062_SERIAL_USE_UART0 */

#if MIMXRT1062_SERIAL_USE_UART1
    if (sdp == &SD2) {
      LPUART_Init(LPUART2, &uconfig, UartSrcFreq());
    }
#endif /* MIMXRT1062_SERIAL_USE_UART1 */

#if MIMXRT1062_SERIAL_USE_UART2
    if (sdp == &SD3) {
      LPUART_Init(LPUART3, &uconfig, UartSrcFreq());
      IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_06] = 2; // Arduino pin 17
    }
#endif /* MIMXRT1062_SERIAL_USE_UART2 */

#if MIMXRT1062_SERIAL_USE_UART3
    if (sdp == &SD4) {
      LPUART_Init(LPUART4, &uconfig, UartSrcFreq());
    }
#endif /* MIMXRT1062_SERIAL_USE_UART3 */

#if MIMXRT1062_SERIAL_USE_UART4
    if (sdp == &SD5) {
      LPUART_Init(LPUART4, &uconfig, UartSrcFreq());
    }
#endif /* MIMXRT1062_SERIAL_USE_UART4 */

#if MIMXRT1062_SERIAL_USE_UART5
    if (sdp == &SD6) {
      LPUART_Init(LPUART5, &uconfig, UartSrcFreq());
    }
#endif /* MIMXRT1062_SERIAL_USE_UART5 */

  }
  /* Configures the peripheral.*/

}

/**
 * @brief   Low level serial driver stop.
 * @details De-initializes the USART, stops the associated clock, resets the
 *          interrupt vector.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 *
 * @notapi
 */
void sd_lld_stop(SerialDriver *sdp) {

  if (sdp->state == SD_READY) {

#if MIMXRT1062_SERIAL_USE_UART0
    if (sdp == &SD1) {
      LPUART_Deinit(LPUART1);
    }
#endif

#if MIMXRT1062_SERIAL_USE_UART1
    if (sdp == &SD2) {
      LPUART_Deinit(LPUART2);
    }
#endif

#if MIMXRT1062_SERIAL_USE_UART2
    if (sdp == &SD3) {
      LPUART_Deinit(LPUART3);
    }
#endif

#if MIMXRT1062_SERIAL_USE_UART3
    if (sdp == &SD4) {
      LPUART_Deinit(LPUART4);
    }
#endif

#if MIMXRT1062_SERIAL_USE_UART4
    if (sdp == &SD5) {
      LPUART_Deinit(LPUART5);
    }
#endif

#if MIMXRT1062_SERIAL_USE_UART5
    if (sdp == &SD6) {
      LPUART_Deinit(LPUART6);
    }
#endif
  }
}

#endif /* HAL_USE_SERIAL */

/** @} */

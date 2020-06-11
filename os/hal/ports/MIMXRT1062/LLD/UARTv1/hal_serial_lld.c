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
    while (!(u->lpuart_p->STAT & LPUART_STAT_TDRE_MASK)) ; // wait
    u->lpuart_p->DATA = b;
  }
}

/**
 * @brief   Common driver initialization, except LP.
 */
static void sd_lld_init_driver(SerialDriver *SDn, LPUART_Type *UARTn) {
  /* Driver initialization.*/
  sdObjectInit(SDn, NULL, notify);
#if 0
  SDn->uart.bdh_p = &(UARTn->BDH);
  SDn->uart.bdl_p = &(UARTn->BDL);
  SDn->uart.c1_p =  &(UARTn->C1);
  SDn->uart.c2_p =  &(UARTn->C2);
  SDn->uart.c3_p =  &(UARTn->C3);
  SDn->uart.c4_p =  &(UARTn->C4);
  SDn->uart.s1_p =  (volatile uint8_t *)&(UARTn->S1);
  SDn->uart.s2_p =  &(UARTn->S2);
  SDn->uart.d_p =   &(UARTn->D);
  SDn->uart.uart_p = UARTn;
#endif
  (void)UARTn;
}

/**
 * @brief   Common UART configuration.
 *
 */
static void configure_uart(SerialDriver *sdp, const SerialConfig *config) {
  UART_w_TypeDef *uart = &(sdp->uart);

  /* Discard any incoming data. */
  while (uart->lpuart_p->STAT & LPUART_STAT_RDRF_MASK) {
    // IMXRT1060, page 2863: To clear RDRF, read the DATA register […]
    (void)uart->lpuart_p->DATA;
  }
  
  // IMXRT1060, page 2861: The 13-bit baud rate setting must be updated only
  // when the transmitter and receiver are both disabled (CTRL[RE] and CTRL[TE]
  // are both 0).
  /* Disable UART while configuring */
  uart->lpuart_p->CTRL &= ~(LPUART_CTRL_RE_MASK | LPUART_CTRL_TE_MASK);

#define UART_CLOCK 24000000

  float base = (float)UART_CLOCK / (float)config->sc_speed;
  float besterr = 1e20;
  int bestdiv = 1;
  int bestosr = 4;
  for (int osr=4; osr <= 32; osr++) {
    float div = base / (float)osr;
    int divint = (int)(div + 0.5f);
    if (divint < 1) divint = 1;
    else if (divint > 8191) divint = 8191;
    float err = ((float)divint - div) / div;
    if (err < 0.0f) err = -err;
    if (err <= besterr) {
      besterr = err;
      bestdiv = divint;
      bestosr = osr;
    }
  }

  uart->lpuart_p->BAUD =
    LPUART_BAUD_OSR(bestosr - 1) |
    LPUART_BAUD_SBR(bestdiv) |
    (bestosr <= 8 ? LPUART_BAUD_BOTHEDGE(1) : 0);
  
  uart->lpuart_p->CTRL = 
    LPUART_CTRL_ORIE(1) | /* overrun interrupt enable */
    LPUART_CTRL_NEIE(1) | /* Noise Error Interrupt Enable */
    LPUART_CTRL_FEIE(1) | /* Framing Error Interrupt Enable */
    LPUART_CTRL_PEIE(1) | /* Parity Error Interrupt Enable */
    LPUART_CTRL_RE(1) | /* Receiver Enable */
    LPUART_CTRL_RIE(1) | /* Receiver Interrupt Enable */
    LPUART_CTRL_TE(1); /* Transmitter Enable */

  // TODO: enable interrupt
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
#if ! MIMXRT1062_SERIAL0_IS_LPUART
  sd_lld_init_driver(&SD1, (UART_TypeDef *)UART0);
#else /* ! MIMXRT1062_SERIAL0_IS_LPUART */
  /* little endian! */
  sdObjectInit(&SD1, NULL, notify);
  SD1.uart.bdh_p = ((uint8_t *)&(LPUART0->BAUD)) + 1; /* BDH: BAUD, byte 3 */
  SD1.uart.bdl_p = ((uint8_t *)&(LPUART0->BAUD)) + 0; /* BDL: BAUD, byte 4 */
  SD1.uart.c1_p =  ((uint8_t *)&(LPUART0->CTRL)) + 0; /* C1: CTRL, byte 4 */
  SD1.uart.c2_p =  ((uint8_t *)&(LPUART0->CTRL)) + 2; /* C2: CTRL, byte 2 */
  SD1.uart.c3_p =  ((uint8_t *)&(LPUART0->CTRL)) + 3; /* C3: CTRL, byte 1 */
  SD1.uart.c4_p =  ((uint8_t *)&(LPUART0->BAUD)) + 3; /* C4: BAUD, byte 1 */
  SD1.uart.s1_p =  ((uint8_t *)&(LPUART0->STAT)) + 2; /* S1: STAT, byte 2 */
  SD1.uart.s2_p =  ((uint8_t *)&(LPUART0->STAT)) + 3; /* S2: STAT, byte 1 */
  SD1.uart.d_p =   ((uint8_t *)&(LPUART0->DATA)) + 0; /* D: DATA, byte 4 */
#endif /* ! MIMXRT1062_SERIAL0_IS_LPUART */
#if MIMXRT1062_SERIAL0_IS_UARTLP
  SD1.uart.c4_p = &(UART0->C4);  /* fix up misconfigured C4 register */
  SD1.uart.uartlp_p = UART0;
  SD1.uart.uart_p = NULL;
#elif MIMXRT1062_SERIAL0_IS_LPUART
  SD1.uart.lpuart_p = LPUART0;
  SD1.uart.uart_p = NULL;
#else /* MIMXRT1062_SERIAL0_IS_LPUART */
  SD1.uart.uart_p = UART0;
#endif /* MIMXRT1062_SERIAL0_IS_LPUART */
#endif /* MIMXRT1062_SERIAL_USE_UART0 */

#if MIMXRT1062_SERIAL_USE_UART1
  /* Driver initialization.*/
#if ! MIMXRT1062_SERIAL1_IS_LPUART
  sd_lld_init_driver(&SD2, UART1);
#else /* ! MIMXRT1062_SERIAL1_IS_LPUART */
  /* little endian! */
  sdObjectInit(&SD2, NULL, notify);
  SD2.uart.bdh_p = ((uint8_t *)&(LPUART1->BAUD)) + 1; /* BDH: BAUD, byte 3 */
  SD2.uart.bdl_p = ((uint8_t *)&(LPUART1->BAUD)) + 0; /* BDL: BAUD, byte 4 */
  SD2.uart.c1_p =  ((uint8_t *)&(LPUART1->CTRL)) + 0; /* C1: CTRL, byte 4 */
  SD2.uart.c2_p =  ((uint8_t *)&(LPUART1->CTRL)) + 2; /* C2: CTRL, byte 2 */
  SD2.uart.c3_p =  ((uint8_t *)&(LPUART1->CTRL)) + 3; /* C3: CTRL, byte 1 */
  SD2.uart.c4_p =  ((uint8_t *)&(LPUART1->BAUD)) + 3; /* C4: BAUD, byte 1 */
  SD2.uart.s1_p =  ((uint8_t *)&(LPUART1->STAT)) + 2; /* S1: STAT, byte 2 */
  SD2.uart.s2_p =  ((uint8_t *)&(LPUART1->STAT)) + 3; /* S2: STAT, byte 1 */
  SD2.uart.d_p =   ((uint8_t *)&(LPUART1->DATA)) + 0; /* D: DATA, byte 4 */
  SD2.uart.lpuart_p = LPUART1;
  SD2.uart.uart_p = NULL;
#endif /* ! MIMXRT1062_SERIAL1_IS_LPUART */
#endif /* MIMXRT1062_SERIAL_USE_UART1 */

#if MIMXRT1062_SERIAL_USE_UART2
  sd_lld_init_driver(&SD3, UART2);
#endif /* MIMXRT1062_SERIAL_USE_UART2 */

#if MIMXRT1062_SERIAL_USE_UART3
  sd_lld_init_driver(&SD4, NULL);
#if 0
  SD4.uart.bdh_p = ((uint8_t *)&(LPUART3->BAUD)) + 1; /* BDH: BAUD, byte 3 */
  SD4.uart.bdl_p = ((uint8_t *)&(LPUART3->BAUD)) + 0; /* BDL: BAUD, byte 4 */
  SD4.uart.c1_p =  ((uint8_t *)&(LPUART3->CTRL)) + 0; /* C1: CTRL, byte 4 */
  SD4.uart.c2_p =  ((uint8_t *)&(LPUART3->CTRL)) + 2; /* C2: CTRL, byte 2 */
  SD4.uart.c3_p =  ((uint8_t *)&(LPUART3->CTRL)) + 3; /* C3: CTRL, byte 1 */
  SD4.uart.c4_p =  ((uint8_t *)&(LPUART3->BAUD)) + 3; /* C4: BAUD, byte 1 */
  SD4.uart.s1_p =  ((uint8_t *)&(LPUART3->STAT)) + 2; /* S1: STAT, byte 2 */
  SD4.uart.s2_p =  ((uint8_t *)&(LPUART3->STAT)) + 3; /* S2: STAT, byte 1 */
  SD4.uart.d_p =   ((uint8_t *)&(LPUART3->DATA)) + 0; /* D: DATA, byte 4 */
#endif
  SD4.uart.lpuart_p = LPUART3;
#endif /* MIMXRT1062_SERIAL_USE_UART3 */

#if MIMXRT1062_SERIAL_USE_UART4
  sd_lld_init_driver(&SD5, UART4);
#endif /* MIMXRT1062_SERIAL_USE_UART4 */

#if MIMXRT1062_SERIAL_USE_UART5
  sd_lld_init_driver(&SD6, UART5);
#endif /* MIMXRT1062_SERIAL_USE_UART5 */
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

#if MIMXRT1062_SERIAL_USE_UART0
    if (sdp == &SD1) {
#if MIMXRT1062_SERIAL0_IS_LPUART
      SIM->SCGC5 |= SIM_SCGC5_LPUART0;
      SIM->SOPT2 =
              (SIM->SOPT2 & ~SIM_SOPT2_LPUART0SRC_MASK) |
              SIM_SOPT2_LPUART0SRC(KINETIS_UART0_CLOCK_SRC);
#else /* MIMXRT1062_SERIAL0_IS_LPUART */
      SIM->SCGC4 |= SIM_SCGC4_UART0;
#endif /* MIMXRT1062_SERIAL0_IS_LPUART */
#if MIMXRT1062_SERIAL0_IS_UARTLP
      SIM->SOPT2 =
              (SIM->SOPT2 & ~SIM_SOPT2_UART0SRC_MASK) |
              SIM_SOPT2_UART0SRC(KINETIS_UART0_CLOCK_SRC);
#endif /* MIMXRT1062_SERIAL0_IS_UARTLP */
      configure_uart(sdp, config);
#if KINETIS_HAS_SERIAL_ERROR_IRQ
      nvicEnableVector(UART0Status_IRQn, MIMXRT1062_SERIAL_UART0_PRIORITY);
      nvicEnableVector(UART0Error_IRQn, MIMXRT1062_SERIAL_UART0_PRIORITY);
#else /* KINETIS_HAS_SERIAL_ERROR_IRQ */
#if MIMXRT1062_SERIAL0_IS_LPUART
      nvicEnableVector(LPUART0_IRQn, MIMXRT1062_SERIAL_UART0_PRIORITY);
#else /* MIMXRT1062_SERIAL0_IS_LPUART */
      nvicEnableVector(UART0_IRQn, MIMXRT1062_SERIAL_UART0_PRIORITY);
#endif /* MIMXRT1062_SERIAL0_IS_LPUART */
#endif /* KINETIS_HAS_SERIAL_ERROR_IRQ */
    }
#endif /* MIMXRT1062_SERIAL_USE_UART0 */

#if MIMXRT1062_SERIAL_USE_UART1
    if (sdp == &SD2) {
#if MIMXRT1062_SERIAL1_IS_LPUART
      SIM->SCGC5 |= SIM_SCGC5_LPUART1;
      SIM->SOPT2 =
              (SIM->SOPT2 & ~SIM_SOPT2_LPUART1SRC_MASK) |
              SIM_SOPT2_LPUART1SRC(KINETIS_UART1_CLOCK_SRC);
#else /* MIMXRT1062_SERIAL1_IS_LPUART */
      SIM->SCGC4 |= SIM_SCGC4_UART1;
#endif /* MIMXRT1062_SERIAL1_IS_LPUART */
      configure_uart(sdp, config);
#if MIMXRT1062_HAS_SERIAL_ERROR_IRQ
      nvicEnableVector(UART1Status_IRQn, MIMXRT1062_SERIAL_UART1_PRIORITY);
      nvicEnableVector(UART1Error_IRQn, MIMXRT1062_SERIAL_UART0_PRIORITY);
#else /* KINETIS_HAS_SERIAL_ERROR_IRQ */
#if MIMXRT1062_SERIAL1_IS_LPUART
      nvicEnableVector(LPUART1_IRQn, MIMXRT1062_SERIAL_UART1_PRIORITY);
#else /* MIMXRT1062_SERIAL1_IS_LPUART */
      nvicEnableVector(UART1_IRQn, MIMXRT1062_SERIAL_UART1_PRIORITY);
#endif /* MIMXRT1062_SERIAL1_IS_LPUART */
#endif /* KINETIS_HAS_SERIAL_ERROR_IRQ */
    }
#endif /* MIMXRT1062_SERIAL_USE_UART1 */

#if MIMXRT1062_SERIAL_USE_UART2
    if (sdp == &SD3) {
      SIM->SCGC4 |= SIM_SCGC4_UART2;
      configure_uart(sdp, config);
#if KINETIS_HAS_SERIAL_ERROR_IRQ
      nvicEnableVector(UART2Status_IRQn, MIMXRT1062_SERIAL_UART2_PRIORITY);
      nvicEnableVector(UART2Error_IRQn, MIMXRT1062_SERIAL_UART0_PRIORITY);
#else /* KINETIS_HAS_SERIAL_ERROR_IRQ */
      nvicEnableVector(UART2_IRQn, MIMXRT1062_SERIAL_UART2_PRIORITY);
#endif /* KINETIS_HAS_SERIAL_ERROR_IRQ */
    }
#endif /* MIMXRT1062_SERIAL_USE_UART2 */

#if MIMXRT1062_SERIAL_USE_UART3
    if (sdp == &SD4) {
      //SIM->SCGC4 |= SIM_SCGC4_UART3;
      //configure_uart(sdp, config);
      /* SIM->SCGC5 |= SIM_SCGC5_LPUART0; */
      /* SIM->SOPT2 = */
      /*         (SIM->SOPT2 & ~SIM_SOPT2_LPUART0SRC_MASK) | */
      /*         SIM_SOPT2_LPUART0SRC(KINETIS_UART0_CLOCK_SRC); */

      CCM->CCGR0 |= CCM_CCGR0_CG6(1); // turn on Serial4
      IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_06] = 2; // Arduino pin 17
      configure_uart(sdp, config);      
      nvicEnableVector(LPUART4_IRQn, MIMXRT1062_SERIAL_UART3_PRIORITY);
    }
#endif /* MIMXRT1062_SERIAL_USE_UART3 */

#if MIMXRT1062_SERIAL_USE_UART4
    if (sdp == &SD5) {
      SIM->SCGC1 |= SIM_SCGC1_UART4;
      configure_uart(sdp, config);
      nvicEnableVector(UART4Status_IRQn, MIMXRT1062_SERIAL_UART4_PRIORITY);
      nvicEnableVector(UART4Error_IRQn, MIMXRT1062_SERIAL_UART4_PRIORITY);
    }
#endif /* MIMXRT1062_SERIAL_USE_UART4 */

#if MIMXRT1062_SERIAL_USE_UART5
    if (sdp == &SD6) {
      SIM->SCGC1 |= SIM_SCGC1_UART5;
      configure_uart(sdp, config);
      nvicEnableVector(UART5Status_IRQn, MIMXRT1062_SERIAL_UART5_PRIORITY);
      nvicEnableVector(UART5Error_IRQn, MIMXRT1062_SERIAL_UART5_PRIORITY);
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
    /* TODO: Resets the peripheral.*/

#if MIMXRT1062_SERIAL_USE_UART0
    if (sdp == &SD1) {
#if KINETIS_HAS_SERIAL_ERROR_IRQ
      nvicDisableVector(UART0Status_IRQn);
      nvicDisableVector(UART0Error_IRQn);
#else /* KINETIS_HAS_SERIAL_ERROR_IRQ */
#if MIMXRT1062_SERIAL0_IS_LPUART
      nvicDisableVector(LPUART0_IRQn);
#else /* MIMXRT1062_SERIAL0_IS_LPUART */
      nvicDisableVector(UART0_IRQn);
#endif /* MIMXRT1062_SERIAL0_IS_LPUART */
#endif /* KINETIS_HAS_SERIAL_ERROR_IRQ */
#if MIMXRT1062_SERIAL0_IS_LPUART
      SIM->SCGC5 &= ~SIM_SCGC5_LPUART0;
#else /* MIMXRT1062_SERIAL0_IS_LPUART */
      SIM->SCGC4 &= ~SIM_SCGC4_UART0;
#endif /* MIMXRT1062_SERIAL0_IS_LPUART */
    }
#endif

#if MIMXRT1062_SERIAL_USE_UART1
    if (sdp == &SD2) {
#if KINETIS_HAS_SERIAL_ERROR_IRQ
      nvicDisableVector(UART1Status_IRQn);
      nvicDisableVector(UART1Error_IRQn);
#else /* KINETIS_HAS_SERIAL_ERROR_IRQ */
#if MIMXRT1062_SERIAL1_IS_LPUART
      nvicDisableVector(LPUART1_IRQn);
#else /* MIMXRT1062_SERIAL1_IS_LPUART */
      nvicDisableVector(UART1_IRQn);
#endif /* MIMXRT1062_SERIAL1_IS_LPUART */
#endif /* KINETIS_HAS_SERIAL_ERROR_IRQ */
#if MIMXRT1062_SERIAL1_IS_LPUART
      SIM->SCGC5 &= ~SIM_SCGC5_LPUART1;
#else /* MIMXRT1062_SERIAL1_IS_LPUART */
      SIM->SCGC4 &= ~SIM_SCGC4_UART1;
#endif /* MIMXRT1062_SERIAL1_IS_LPUART */
    }
#endif

#if MIMXRT1062_SERIAL_USE_UART2
    if (sdp == &SD3) {
#if KINETIS_HAS_SERIAL_ERROR_IRQ
      nvicDisableVector(UART2Status_IRQn);
      nvicDisableVector(UART2Error_IRQn);
#else /* KINETIS_HAS_SERIAL_ERROR_IRQ */
      nvicDisableVector(UART2_IRQn);
#endif /* KINETIS_HAS_SERIAL_ERROR_IRQ */
      SIM->SCGC4 &= ~SIM_SCGC4_UART2;
    }
#endif

#if MIMXRT1062_SERIAL_USE_UART3
    if (sdp == &SD4) {
      nvicDisableVector(LPUART4_IRQn);      
      CCM->CCGR0 &= ~CCM_CCGR0_CG6(1); // turn off Serial4
    }
#endif

#if MIMXRT1062_SERIAL_USE_UART4
    if (sdp == &SD5) {
      nvicDisableVector(UART4Status_IRQn);
      nvicDisableVector(UART4Error_IRQn);
      SIM->SCGC1 &= ~SIM_SCGC1_UART4;
    }
#endif

#if MIMXRT1062_SERIAL_USE_UART5
    if (sdp == &SD6) {
      nvicDisableVector(UART5Status_IRQn);
      nvicDisableVector(UART5Error_IRQn);
      SIM->SCGC1 &= ~SIM_SCGC1_UART5;
    }
#endif
  }
}

#endif /* HAL_USE_SERIAL */

/** @} */

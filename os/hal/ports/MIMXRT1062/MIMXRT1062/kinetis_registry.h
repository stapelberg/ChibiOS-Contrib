/*
    ChibiOS - Copyright (C) 2014 Derek Mulcahy
                        (C) 2016 flabbergast <s3+flabbergast@sdfeu.org>

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
 * @file    MK66F18/kinetis_registry.h
 * @brief   MK66F18 capabilities registry.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef MIMXRT1062_REGISTRY_H_
#define MIMXRT1062_REGISTRY_H_

#if !defined(MIMXRT1062) || defined(__DOXYGEN__)
#define MIMXRT1062 
#endif

/*===========================================================================*/
/* Platform capabilities.                                                    */
/*===========================================================================*/

/**
 * @brief   Maximum system and core clock (f_SYS) frequency.
 */
#define MIMXRT1062_SYSCLK_MAX      180000000L

/**
 * @brief   Maximum bus clock (f_BUS) frequency.
 */
#define MIMXRT1062_BUSCLK_MAX      60000000L

/**
 * @brief   Maximum flash clock (f_FLASH) frequency.
 */
#define MIMXRT1062_FLASHCLK_MAX    28000000L

/* ADC attributes.*/
#define MIMXRT1062_HAS_ADC0            TRUE
#define MIMXRT1062_ADC0_IRQ_VECTOR     VectorDC
#define MIMXRT1062_HAS_ADC1            TRUE
#define MIMXRT1062_ADC1_IRQ_VECTOR     Vector164

/* DAC attributes.*/
#define MIMXRT1062_HAS_DAC0            TRUE
#define MIMXRT1062_DAC0_IRQ_VECTOR     Vector120
#define MIMXRT1062_HAS_DAC1            TRUE
#define MIMXRT1062_DAC1_IRQ_VECTOR     Vector160

/* DMA attributes.*/
#define MIMXRT1062_DMA0_IRQ_VECTOR     Vector40 // 0
#define MIMXRT1062_DMA1_IRQ_VECTOR     Vector44 
#define MIMXRT1062_DMA2_IRQ_VECTOR     Vector48
#define MIMXRT1062_DMA3_IRQ_VECTOR     Vector4C
#define MIMXRT1062_DMA4_IRQ_VECTOR     Vector50
#define MIMXRT1062_DMA5_IRQ_VECTOR     Vector54
#define MIMXRT1062_DMA6_IRQ_VECTOR     Vector58
#define MIMXRT1062_DMA7_IRQ_VECTOR     Vector5C
#define MIMXRT1062_DMA8_IRQ_VECTOR     Vector60
#define MIMXRT1062_DMA9_IRQ_VECTOR     Vector64
#define MIMXRT1062_DMA10_IRQ_VECTOR     Vector68
#define MIMXRT1062_DMA11_IRQ_VECTOR     Vector6C
#define MIMXRT1062_DMA12_IRQ_VECTOR     Vector70
#define MIMXRT1062_DMA13_IRQ_VECTOR     Vector74
#define MIMXRT1062_DMA14_IRQ_VECTOR     Vector78
#define MIMXRT1062_DMA15_IRQ_VECTOR     Vector7C
#define MIMXRT1062_HAS_DMA_ERROR_IRQ   TRUE
#define MIMXRT1062_DMA_ERROR_IRQ_VECTOR Vector80

/* EXT attributes.*/
#define MIMXRT1062_PORTA_IRQ_VECTOR    Vector12C
#define MIMXRT1062_PORTB_IRQ_VECTOR    Vector130
#define MIMXRT1062_PORTC_IRQ_VECTOR    Vector134
#define MIMXRT1062_PORTD_IRQ_VECTOR    Vector138
#define MIMXRT1062_PORTE_IRQ_VECTOR    Vector13C
#define MIMXRT1062_EXT_HAS_COMMON_CD_IRQ   FALSE
#define MIMXRT1062_EXT_HAS_COMMON_BCDE_IRQ FALSE
#define MIMXRT1062_GPIO_HAS_OPENDRAIN  TRUE

/* I2C attributes.*/
#define MIMXRT1062_HAS_I2C0            TRUE
#define MIMXRT1062_I2C0_IRQ_VECTOR     VectorA0
#define MIMXRT1062_HAS_I2C1            TRUE
#define MIMXRT1062_I2C1_IRQ_VECTOR     VectorA4

/* Serial attributes.*/
#define MIMXRT1062_HAS_SERIAL0         TRUE
#define MIMXRT1062_SERIAL0_IRQ_VECTOR  VectorBC
#define MIMXRT1062_HAS_SERIAL1         TRUE
#define MIMXRT1062_SERIAL1_IRQ_VECTOR  VectorC4
#define MIMXRT1062_HAS_SERIAL2         TRUE
#define MIMXRT1062_SERIAL2_IRQ_VECTOR  VectorCC
#define MIMXRT1062_HAS_SERIAL3         TRUE
#define MIMXRT1062_SERIAL3_IRQ_VECTOR  Vector98
#define MIMXRT1062_HAS_SERIAL_ERROR_IRQ TRUE
#define MIMXRT1062_SERIAL0_ERROR_IRQ_VECTOR VectorC0
#define MIMXRT1062_SERIAL1_ERROR_IRQ_VECTOR VectorC8
#define MIMXRT1062_SERIAL2_ERROR_IRQ_VECTOR VectorD0
#define MIMXRT1062_SERIAL3_ERROR_IRQ_VECTOR VectorD8
#define MIMXRT1062_SERIAL0_IS_LPUART   FALSE
#define MIMXRT1062_SERIAL0_IS_UARTLP   FALSE
#define MIMXRT1062_SERIAL1_IS_LPUART   FALSE
#define MIMXRT1062_SERIAL1_IS_UARTLP   FALSE
#define MIMXRT1062_SERIAL2_IS_LPUART   FALSE
#define MIMXRT1062_SERIAL2_IS_UARTLP   FALSE
#define MIMXRT1062_SERIAL3_IS_LPUART   TRUE
#define MIMXRT1062_SERIAL3_IS_UARTLP   FALSE

/* SPI attributes.*/
#define MIMXRT1062_HAS_SPI0            TRUE
#define MIMXRT1062_SPI0_IRQ_VECTOR     VectorA8
#define MIMXRT1062_HAS_SPI1            TRUE
#define MIMXRT1062_SPI1_IRQ_VECTOR     VectorAC

/* FlexTimer attributes.*/
#define MIMXRT1062_FTM0_CHANNELS 8
#define MIMXRT1062_FTM1_CHANNELS 2
#define MIMXRT1062_FTM2_CHANNELS 2
#define MIMXRT1062_FTM3_CHANNELS 8

#define MIMXRT1062_HAS_FTM0            TRUE
#define MIMXRT1062_FTM0_IRQ_VECTOR     VectorE8
#define MIMXRT1062_HAS_FTM1            TRUE
#define MIMXRT1062_FTM1_IRQ_VECTOR     VectorEC
#define MIMXRT1062_HAS_FTM2            TRUE
#define MIMXRT1062_FTM2_IRQ_VECTOR     VectorF0
#define MIMXRT1062_HAS_FTM3            TRUE
#define MIMXRT1062_FTM3_IRQ_VECTOR     Vector15C

/* GPT attributes.*/
#define MIMXRT1062_HAS_PIT0            TRUE
#define MIMXRT1062_PIT0_IRQ_VECTOR     Vector100
#define MIMXRT1062_HAS_PIT1            TRUE
#define MIMXRT1062_PIT1_IRQ_VECTOR     Vector104
#define MIMXRT1062_HAS_PIT2            TRUE
#define MIMXRT1062_PIT2_IRQ_VECTOR     Vector108
#define MIMXRT1062_HAS_PIT3            TRUE
#define MIMXRT1062_PIT3_IRQ_VECTOR     Vector10C
#define MIMXRT1062_HAS_PIT_COMMON_IRQ  FALSE

/* USB attributes.*/
#define MIMXRT1062_HAS_USB             TRUE
#define MIMXRT1062_USB_IRQ_VECTOR      Vector114
#define MIMXRT1062_USB0_IS_USBOTG      TRUE
#define MIMXRT1062_HAS_USB_CLOCK_RECOVERY TRUE

/* LPTMR attributes.*/
#define MIMXRT1062_LPTMR0_IRQ_VECTOR   Vector128

/* SDHC (SDC, MMC, SDIO) attributes */
#define MIMXRT1062_HAS_SDHC            TRUE
#define MIMXRT1062_SDHC_IRQ_VECTOR     Vector184

/** @} */

#endif /* MIMXRT1062_REGISTRY_H_ */

/** @} */

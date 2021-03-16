/*
    ChibiOS - Copyright (C) 2014-2015 Fabio Utzig

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.

    Portions Copyright (C) 2017 PJRC.COM, LLC.

    Permission is hereby granted, free of charge, to any person obtaining
    a copy of this software and associated documentation files (the
    "Software"), to deal in the Software without restriction, including
    without limitation the rights to use, copy, modify, merge, publish,
    distribute, sublicense, and/or sell copies of the Software, and to
    permit persons to whom the Software is furnished to do so, subject to
    the following conditions:
    
    1. The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.
    
    2. If the Software is incorporated into a build system that allows
    selection among a list of target devices, then similar target
    devices manufactured by PJRC.COM must be included in the list of
    target devices and selectable in the same manner.
    
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
    BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
    ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
    CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.

*/

/**
 * @file    MK66F18/hal_lld.c
 * @brief   Kinetis MK66F18 HAL Driver subsystem low level driver source template.
 *
 * @addtogroup HAL
 * @{
 */

#include "hal.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

#ifdef __CC_ARM
__attribute__ ((section(".ARM.__at_0x400")))
#else
__attribute__ ((used,section(".cfmconfig")))
#endif
const uint8_t _cfm[0x10] = {
  0xFF,  /* NV_BACKKEY3: KEY=0xFF */
  0xFF,  /* NV_BACKKEY2: KEY=0xFF */
  0xFF,  /* NV_BACKKEY1: KEY=0xFF */
  0xFF,  /* NV_BACKKEY0: KEY=0xFF */
  0xFF,  /* NV_BACKKEY7: KEY=0xFF */
  0xFF,  /* NV_BACKKEY6: KEY=0xFF */
  0xFF,  /* NV_BACKKEY5: KEY=0xFF */
  0xFF,  /* NV_BACKKEY4: KEY=0xFF */
  0xFF,  /* NV_FPROT3: PROT=0xFF */
  0xFF,  /* NV_FPROT2: PROT=0xFF */
  0xFF,  /* NV_FPROT1: PROT=0xFF */
  0xFF,  /* NV_FPROT0: PROT=0xFF */
  0x7E,  /* NV_FSEC: KEYEN=1,MEEN=3,FSLACC=3,SEC=2 */
  0xFF,  /* NV_FOPT: ??=1,??=1,FAST_INIT=1,LPBOOT1=1,RESET_PIN_CFG=1,
                      NMI_DIS=1,EZPORT_DIS=1,LPBOOT0=1 */
  0xFF,
  0xFF
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level HAL driver initialization.
 * @todo    Use a macro to define the system clock frequency.
 *
 * @notapi
 */
void hal_lld_init(void) {

#if defined(MK66F18)
  /* Disable the MPU by default */
  SYSMPU->CESR &= ~SYSMPU_CESR_VLD_MASK;
#endif

}

#if 1 /* DELAY */

// delay sleeps for |cycles| (e.g. sleeping for F_CPU will sleep 1s).
// delay’s precision is ± 166 ns (due to 30 cycles of overhead).
void delay(const uint32_t cycles) {
  // Reset cycle counter
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;

  while (DWT->CYCCNT < cycles) {
    // busy-loop until time has passed
  }
}

#endif

#if 1 /* PRINT_DEBUG_STUFF */
void putchar_debug(char c);
static void puint_debug(unsigned int num);


 void printf_debug(const char *format, ...)
{
	va_list args;
	unsigned int val;
	int n;

	va_start(args, format);
	for (; *format != 0; format++) { // no-frills stand-alone printf
		if (*format == '%') {
			++format;
			if (*format == '%') goto out;
			if (*format == '-') format++; // ignore size
			while (*format >= '0' && *format <= '9') format++; // ignore size
			if (*format == 'l') format++; // ignore long
			if (*format == '\0') break;
			if (*format == 's') {
				printf_debug((char *)va_arg(args, int));
			} else if (*format == 'd') {
				n = va_arg(args, int);
				if (n < 0) {
					n = -n;
					putchar_debug('-');
				}
				puint_debug(n);
			} else if (*format == 'u') {
				puint_debug(va_arg(args, unsigned int));
			} else if (*format == 'x' || *format == 'X') {
				val = va_arg(args, unsigned int);
				for (n=0; n < 8; n++) {
					unsigned int d = (val >> 28) & 15;
					putchar_debug((d < 10) ? d + '0' : d - 10 + 'A');
					val <<= 4;
				}
			} else if (*format == 'c' ) {
				putchar_debug((char)va_arg(args, int));
			}
		} else {
			out:
			if (*format == '\n') putchar_debug('\r');
			putchar_debug(*format);
		}
	}
	va_end(args);
}

static void puint_debug(unsigned int num)
{
	char buf[12];
	unsigned int i = sizeof(buf)-2;

	buf[sizeof(buf)-1] = 0;
	while (1) {
		buf[i] = (num % 10) + '0';
		num /= 10;
		if (num == 0) break;
		i--;
	}
	printf_debug(buf + i);
}

void putchar_debug(char c)
{
  while (!(LPUART3->STAT & LPUART_STAT_TDRE(1))) ; // wait
	LPUART3->DATA = c;
}

void printf_debug_init(void)
{
  CCM->CCGR0 |= CCM_CCGR0_CG6(1); // turn on Serial4
  IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_06] = 2; // Arduino pin 17
  LPUART3->BAUD = LPUART_BAUD_OSR(25) | LPUART_BAUD_SBR(8); // ~115200 baud
  LPUART3->CTRL = LPUART_CTRL_TE(1);
}

#endif /* PRINT_DEBUG_STUFF */

void usb_pll_start(void) {
	while (1) {
		uint32_t n = CCM_ANALOG->PLL_USB1; // pg 759
		printf_debug("CCM_ANALOG_PLL_USB1=%08lX\n", n);
		if (n & CCM_ANALOG_PLL_USB1_DIV_SELECT(1)) {
			printf_debug("  ERROR, 528 MHz mode!\n"); // never supposed to use this mode!
			CCM_ANALOG->PLL_USB1_CLR = 0xC000;			// bypass 24 MHz
			CCM_ANALOG->PLL_USB1_SET = CCM_ANALOG_PLL_USB1_BYPASS(1);	// bypass
			CCM_ANALOG->PLL_USB1_CLR = CCM_ANALOG_PLL_USB1_POWER(1) |	// power down
			  CCM_ANALOG_PLL_USB1_DIV_SELECT(1) |		// use 480 MHz
			  CCM_ANALOG_PLL_USB1_ENABLE(1) |			// disable
			  CCM_ANALOG_PLL_USB1_EN_USB_CLKS(1);		// disable usb
			continue;
		}
		if (!(n & CCM_ANALOG_PLL_USB1_ENABLE(1))) {
			printf_debug("  enable PLL\n");
			// TODO: should this be done so early, or later??
			CCM_ANALOG->PLL_USB1_SET = CCM_ANALOG_PLL_USB1_ENABLE(1);
			continue;
		}
		if (!(n & CCM_ANALOG_PLL_USB1_POWER(1))) {
			printf_debug("  power up PLL\n");
			CCM_ANALOG->PLL_USB1_SET = CCM_ANALOG_PLL_USB1_POWER(1);
			continue;
		}
		if (!(n & CCM_ANALOG_PLL_USB1_LOCK(1))) {
			printf_debug("  wait for lock\n");
			continue;
		}
		if (n & CCM_ANALOG_PLL_USB1_BYPASS(1)) {
			printf_debug("  turn off bypass\n");
			CCM_ANALOG->PLL_USB1_CLR = CCM_ANALOG_PLL_USB1_BYPASS(1);
			continue;
		}
		if (!(n & CCM_ANALOG_PLL_USB1_EN_USB_CLKS(1))) {
			printf_debug("  enable USB clocks\n");
			CCM_ANALOG->PLL_USB1_SET = CCM_ANALOG_PLL_USB1_EN_USB_CLKS(1);
			continue;
		}
		return; // everything is as it should be  :-)
	}
}

// TODO: port this to CMSIS style headers
#define SCB_MPU_TYPE            (*(volatile uint32_t *)0xE000ED90) // 
#define SCB_MPU_CTRL            (*(volatile uint32_t *)0xE000ED94) // 
#define SCB_MPU_CTRL_PRIVDEFENA         ((uint32_t)(1<<2)) // Enables default memory map
#define SCB_MPU_CTRL_HFNMIENA           ((uint32_t)(1<<1)) // Use MPU for HardFault & NMI
#define SCB_MPU_CTRL_ENABLE             ((uint32_t)(1<<0)) // Enables MPU
#define SCB_MPU_RNR             (*(volatile uint32_t *)0xE000ED98) // 
#define SCB_MPU_RBAR            (*(volatile uint32_t *)0xE000ED9C) // 
#define SCB_MPU_RBAR_ADDR_MASK          ((uint32_t)(0xFFFFFFE0))
#define SCB_MPU_RBAR_VALID              ((uint32_t)(1<<4))
#define SCB_MPU_RBAR_REGION(n)          ((uint32_t)((n) & 15))
#define SCB_MPU_RASR            (*(volatile uint32_t *)0xE000EDA0) // ARM DDI0403E, pg 696
#define SCB_MPU_RASR_XN                 ((uint32_t)(1<<28))
#define SCB_MPU_RASR_AP(n)              ((uint32_t)(((n) & 7) << 24))
#define SCB_MPU_RASR_TEX(n)             ((uint32_t)(((n) & 7) << 19))
#define SCB_MPU_RASR_S                  ((uint32_t)(1<<18))
#define SCB_MPU_RASR_C                  ((uint32_t)(1<<17))
#define SCB_MPU_RASR_B                  ((uint32_t)(1<<16))
#define SCB_MPU_RASR_SRD(n)             ((uint32_t)(((n) & 255) << 8))
#define SCB_MPU_RASR_SIZE(n)            ((uint32_t)(((n) & 31) << 1))
#define SCB_MPU_RASR_ENABLE             ((uint32_t)(1<<0))
#define SCB_MPU_RBAR_A1         (*(volatile uint32_t *)0xE000EDA4) // 
#define SCB_MPU_RASR_A1         (*(volatile uint32_t *)0xE000EDA8) // 
#define SCB_MPU_RBAR_A2         (*(volatile uint32_t *)0xE000EDAC) // 
#define SCB_MPU_RASR_A2         (*(volatile uint32_t *)0xE000EDB0) // 
#define SCB_MPU_RBAR_A3         (*(volatile uint32_t *)0xE000EDB4) // 
#define SCB_MPU_RASR_A3         (*(volatile uint32_t *)0xE000EDB8) // 


// concise defines for SCB_MPU_RASR and SCB_MPU_RBAR, ARM DDI0403E, pg 696
#define NOEXEC		SCB_MPU_RASR_XN
#define READONLY	SCB_MPU_RASR_AP(7)
#define READWRITE	SCB_MPU_RASR_AP(3)
#define NOACCESS	SCB_MPU_RASR_AP(0)
#define MEM_CACHE_WT	SCB_MPU_RASR_TEX(0) | SCB_MPU_RASR_C
#define MEM_CACHE_WB	SCB_MPU_RASR_TEX(0) | SCB_MPU_RASR_C | SCB_MPU_RASR_B
#define MEM_CACHE_WBWA	SCB_MPU_RASR_TEX(1) | SCB_MPU_RASR_C | SCB_MPU_RASR_B
#define MEM_NOCACHE	SCB_MPU_RASR_TEX(1)
#define DEV_NOCACHE	SCB_MPU_RASR_TEX(2)
#define SIZE_32B	(SCB_MPU_RASR_SIZE(4) | SCB_MPU_RASR_ENABLE)
#define SIZE_64B	(SCB_MPU_RASR_SIZE(5) | SCB_MPU_RASR_ENABLE)
#define SIZE_128B	(SCB_MPU_RASR_SIZE(6) | SCB_MPU_RASR_ENABLE)
#define SIZE_256B	(SCB_MPU_RASR_SIZE(7) | SCB_MPU_RASR_ENABLE)
#define SIZE_512B	(SCB_MPU_RASR_SIZE(8) | SCB_MPU_RASR_ENABLE)
#define SIZE_1K		(SCB_MPU_RASR_SIZE(9) | SCB_MPU_RASR_ENABLE)
#define SIZE_2K		(SCB_MPU_RASR_SIZE(10) | SCB_MPU_RASR_ENABLE)
#define SIZE_4K		(SCB_MPU_RASR_SIZE(11) | SCB_MPU_RASR_ENABLE)
#define SIZE_8K		(SCB_MPU_RASR_SIZE(12) | SCB_MPU_RASR_ENABLE)
#define SIZE_16K	(SCB_MPU_RASR_SIZE(13) | SCB_MPU_RASR_ENABLE)
#define SIZE_32K	(SCB_MPU_RASR_SIZE(14) | SCB_MPU_RASR_ENABLE)
#define SIZE_64K	(SCB_MPU_RASR_SIZE(15) | SCB_MPU_RASR_ENABLE)
#define SIZE_128K	(SCB_MPU_RASR_SIZE(16) | SCB_MPU_RASR_ENABLE)
#define SIZE_256K	(SCB_MPU_RASR_SIZE(17) | SCB_MPU_RASR_ENABLE)
#define SIZE_512K	(SCB_MPU_RASR_SIZE(18) | SCB_MPU_RASR_ENABLE)
#define SIZE_1M		(SCB_MPU_RASR_SIZE(19) | SCB_MPU_RASR_ENABLE)
#define SIZE_2M		(SCB_MPU_RASR_SIZE(20) | SCB_MPU_RASR_ENABLE)
#define SIZE_4M		(SCB_MPU_RASR_SIZE(21) | SCB_MPU_RASR_ENABLE)
#define SIZE_8M		(SCB_MPU_RASR_SIZE(22) | SCB_MPU_RASR_ENABLE)
#define SIZE_16M	(SCB_MPU_RASR_SIZE(23) | SCB_MPU_RASR_ENABLE)
#define SIZE_32M	(SCB_MPU_RASR_SIZE(24) | SCB_MPU_RASR_ENABLE)
#define SIZE_64M	(SCB_MPU_RASR_SIZE(25) | SCB_MPU_RASR_ENABLE)
#define SIZE_128M	(SCB_MPU_RASR_SIZE(26) | SCB_MPU_RASR_ENABLE)
#define SIZE_256M	(SCB_MPU_RASR_SIZE(27) | SCB_MPU_RASR_ENABLE)
#define SIZE_512M	(SCB_MPU_RASR_SIZE(28) | SCB_MPU_RASR_ENABLE)
#define SIZE_1G		(SCB_MPU_RASR_SIZE(29) | SCB_MPU_RASR_ENABLE)
#define SIZE_2G		(SCB_MPU_RASR_SIZE(30) | SCB_MPU_RASR_ENABLE)
#define SIZE_4G		(SCB_MPU_RASR_SIZE(31) | SCB_MPU_RASR_ENABLE)
#define REGION(n)	(SCB_MPU_RBAR_REGION(n) | SCB_MPU_RBAR_VALID)

extern unsigned long _ebss;

void configure_cache(void)
{
	//printf("MPU_TYPE = %08lX\n", SCB_MPU_TYPE);
	//printf("CCR = %08lX\n", SCB_CCR);

	// TODO: check if caches already active - skip?

	MPU->CTRL = 0; // turn off MPU
#if 1 
	uint32_t i = 0;
	MPU->RBAR = 0x00000000 | REGION(i++); //https://developer.arm.com/docs/146793866/10/why-does-the-cortex-m7-initiate-axim-read-accesses-to-memory-addresses-that-do-not-fall-under-a-defined-mpu-region
	MPU->RASR = SCB_MPU_RASR_TEX(0) | NOACCESS | NOEXEC | SIZE_4G;
	
	MPU->RBAR = 0x00000000 | REGION(i++); // ITCM
	MPU->RASR = MEM_NOCACHE | READWRITE | SIZE_512K;

	// TODO: trap regions should be created last, because the hardware gives
	//  priority to the higher number ones.
	MPU->RBAR = 0x00000000 | REGION(i++); // trap NULL pointer deref
	MPU->RASR =  DEV_NOCACHE | NOACCESS | SIZE_32B;

	MPU->RBAR = 0x00200000 | REGION(i++); // Boot ROM
	MPU->RASR = MEM_CACHE_WT | READONLY | SIZE_128K;

	MPU->RBAR = 0x20000000 | REGION(i++); // DTCM
	MPU->RASR = MEM_NOCACHE | READWRITE | NOEXEC | SIZE_512K;

	// teensy4/startup.c sets up a region on the stack to detect stack overflow.
	// ChibiOS, at the time of writing, does not.
//	MPU->RBAR = ((uint32_t)&_ebss) | REGION(i++); // trap stack overflow
//	MPU->RASR = SCB_MPU_RASR_TEX(0) | NOACCESS | NOEXEC | SIZE_32B;

	MPU->RBAR = 0x20200000 | REGION(i++); // RAM (AXI bus)
	MPU->RASR = MEM_CACHE_WBWA | READWRITE | NOEXEC | SIZE_1M;

	MPU->RBAR = 0x40000000 | REGION(i++); // Peripherals
	MPU->RASR = DEV_NOCACHE | READWRITE | NOEXEC | SIZE_64M;

	MPU->RBAR = 0x60000000 | REGION(i++); // QSPI Flash
	MPU->RASR = MEM_CACHE_WBWA | READONLY | SIZE_16M;

	MPU->RBAR = 0x70000000 | REGION(i++); // FlexSPI2
	MPU->RASR = MEM_CACHE_WBWA | READONLY | NOEXEC | SIZE_256M;

	MPU->RBAR = 0x70000000 | REGION(i++); // FlexSPI2
	MPU->RASR = MEM_CACHE_WBWA | READWRITE | NOEXEC | SIZE_16M;

	// TODO: protect access to power supply config

	MPU->CTRL = SCB_MPU_CTRL_ENABLE;
#endif

	// ChibiOS initializes the cache in __cpu_init
	
	/* // cache enable, ARM DDI0403E, pg 628 */
	/* asm("dsb"); */
	/* asm("isb"); */
	/* SCB->ICIALLU = 0; */

	/* asm("dsb"); */
	/* asm("isb"); */
	/* SCB->CCR |= (SCB_CCR_IC_Msk | SCB_CCR_DC_Msk); */
}

uint32_t set_arm_clock(uint32_t frequency)
{
	uint32_t cbcdr = CCM->CBCDR; // pg 1021
	uint32_t cbcmr = CCM->CBCMR; // pg 1023
	uint32_t dcdc = DCDC->REG3;

	// compute required voltage
	uint32_t voltage = 1150; // default = 1.15V
	if (frequency > 528000000) {
		voltage = 1250; // 1.25V
#if defined(OVERCLOCK_STEPSIZE) && defined(OVERCLOCK_MAX_VOLT)
		if (frequency > 600000000) {
			voltage += ((frequency - 600000000) / OVERCLOCK_STEPSIZE) * 25;
			if (voltage > OVERCLOCK_MAX_VOLT) voltage = OVERCLOCK_MAX_VOLT;
		}
#endif
	} else if (frequency <= 24000000) {
		voltage = 950; // 0.95
	}

	// if voltage needs to increase, do it before switch clock speed
	CCM->CCGR6 |= CCM_CCGR6_CG3(1);
	if ((dcdc & DCDC_REG3_TRG_MASK) < DCDC_REG3_TRG((voltage - 800) / 25)) {
		printf_debug("Increasing voltage to %u mV\n", voltage);
		dcdc &= ~DCDC_REG3_TRG_MASK;
		dcdc |= DCDC_REG3_TRG((voltage - 800) / 25);
		DCDC->REG3 = dcdc;
		while (!(DCDC->REG0 & DCDC_REG0_STS_DC_OK(1))) ; // wait voltage settling
	}

	if (!(cbcdr & CCM_CBCDR_PERIPH_CLK_SEL(1))) {
		printf_debug("need to switch to alternate clock during reconfigure of ARM PLL\n");
		const uint32_t need1s =
		  CCM_ANALOG_PLL_USB1_ENABLE(1) |
		  CCM_ANALOG_PLL_USB1_POWER(1) |
		  CCM_ANALOG_PLL_USB1_LOCK(1) |
		  CCM_ANALOG_PLL_USB1_EN_USB_CLKS(1);
		uint32_t sel, div;
		if ((CCM_ANALOG->PLL_USB1 & need1s) == need1s) {
			printf_debug("USB PLL is running, so we can use 120 MHz\n");
			sel = 0;
			div = 3; // divide down to 120 MHz, so IPG is ok even if IPG_PODF=0
		} else {
			printf_debug("USB PLL is off, use 24 MHz crystal\n");
			sel = 1;
			div = 0;
		}
		if ((cbcdr & CCM_CBCDR_PERIPH_CLK2_PODF_MASK) != CCM_CBCDR_PERIPH_CLK2_PODF(div)) {
			// PERIPH_CLK2 divider needs to be changed
			cbcdr &= ~CCM_CBCDR_PERIPH_CLK2_PODF_MASK;
			cbcdr |= CCM_CBCDR_PERIPH_CLK2_PODF(div);
			CCM->CBCDR = cbcdr;
		}
		if ((cbcmr & CCM_CBCMR_PERIPH_CLK2_SEL_MASK) != CCM_CBCMR_PERIPH_CLK2_SEL(sel)) {
			// PERIPH_CLK2 source select needs to be changed
			cbcmr &= ~CCM_CBCMR_PERIPH_CLK2_SEL_MASK;
			cbcmr |= CCM_CBCMR_PERIPH_CLK2_SEL(sel);
			CCM->CBCMR = cbcmr;
			while (CCM->CDHIPR & CCM_CDHIPR_PERIPH2_CLK_SEL_BUSY(1)) ; // wait
		}
		// switch over to PERIPH_CLK2
		cbcdr |= CCM_CBCDR_PERIPH_CLK_SEL(1);
		CCM->CBCDR = cbcdr;
		while (CCM->CDHIPR & CCM_CDHIPR_PERIPH_CLK_SEL_BUSY(1)) ; // wait
	} else {
		printf_debug("already running from PERIPH_CLK2, safe to mess with ARM PLL\n");
	}

	// TODO: check if PLL2 running, can 352, 396 or 528 can work? (no need for ARM PLL)

	// DIV_SELECT: 54-108 = official range 648 to 1296 in 12 MHz steps
	uint32_t div_arm = 1;
	uint32_t div_ahb = 1;
	while (frequency * div_arm * div_ahb < 648000000) {
		if (div_arm < 8) {
			div_arm = div_arm + 1;
		} else {
			if (div_ahb < 5) {
				div_ahb = div_ahb + 1;
				div_arm = 1;
			} else {
				break;
			}
		}
	}
	uint32_t mult = (frequency * div_arm * div_ahb + 6000000) / 12000000;
	if (mult > 108) mult = 108;
	if (mult < 54) mult = 54;
	printf_debug("Freq: 12 MHz * %u / %u / %u\n", mult, div_arm, div_ahb);
	frequency = mult * 12000000 / div_arm / div_ahb;

	printf_debug("ARM PLL=%x\n", CCM_ANALOG->PLL_ARM);
	const uint32_t arm_pll_mask =
	  CCM_ANALOG_PLL_ARM_LOCK(1) |
	  CCM_ANALOG_PLL_ARM_BYPASS(1) |
	  CCM_ANALOG_PLL_ARM_ENABLE(1) |
	  CCM_ANALOG_PLL_ARM_POWERDOWN(1) |
	  CCM_ANALOG_PLL_ARM_DIV_SELECT_MASK;
	if ((CCM_ANALOG->PLL_ARM & arm_pll_mask) != (CCM_ANALOG_PLL_ARM_LOCK(1)
						     | CCM_ANALOG_PLL_ARM_ENABLE(1)
						     | CCM_ANALOG_PLL_ARM_DIV_SELECT(mult))) {
		printf_debug("ARM PLL needs reconfigure\n");
		CCM_ANALOG->PLL_ARM = CCM_ANALOG_PLL_ARM_POWERDOWN(1);
		// TODO: delay needed?
		CCM_ANALOG->PLL_ARM = CCM_ANALOG_PLL_ARM_ENABLE(1)
			| CCM_ANALOG_PLL_ARM_DIV_SELECT(mult);
		while (!(CCM_ANALOG->PLL_ARM & CCM_ANALOG_PLL_ARM_LOCK(1))) ; // wait for lock
		printf_debug("ARM PLL=%x\n", CCM_ANALOG->PLL_ARM);
	} else {
		printf_debug("ARM PLL already running at required frequency\n");
	}

	if ((CCM->CACRR & CCM_CACRR_ARM_PODF_MASK) != (div_arm - 1)) {
		CCM->CACRR = CCM_CACRR_ARM_PODF(div_arm - 1);
		while (CCM->CDHIPR & CCM_CDHIPR_ARM_PODF_BUSY(1)) ; // wait
	}

	if ((cbcdr & CCM_CBCDR_AHB_PODF_MASK) != CCM_CBCDR_AHB_PODF(div_ahb - 1)) {
		cbcdr &= ~CCM_CBCDR_AHB_PODF_MASK;
		cbcdr |= CCM_CBCDR_AHB_PODF(div_ahb - 1);
		CCM->CBCDR = cbcdr;
		while (CCM->CDHIPR & CCM_CDHIPR_AHB_PODF_BUSY(1)); // wait
	}

	uint32_t div_ipg = (frequency + 149999999) / 150000000;
	if (div_ipg > 4) div_ipg = 4;
	if ((cbcdr & CCM_CBCDR_IPG_PODF_MASK) != (CCM_CBCDR_IPG_PODF(div_ipg - 1))) {
		cbcdr &= ~CCM_CBCDR_IPG_PODF_MASK;
		cbcdr |= CCM_CBCDR_IPG_PODF(div_ipg - 1);
		// TODO: how to safely change IPG_PODF ??
		CCM->CBCDR = cbcdr;
	}

	//cbcdr &= ~CCM_CBCDR_PERIPH_CLK_SEL;
	//CCM_CBCDR = cbcdr;  // why does this not work at 24 MHz?
	CCM->CBCDR &= ~CCM_CBCDR_PERIPH_CLK_SEL(1);
	while (CCM->CDHIPR & CCM_CDHIPR_PERIPH_CLK_SEL_BUSY(1)) ; // wait

	//F_CPU_ACTUAL = frequency;
	//F_BUS_ACTUAL = frequency / div_ipg;
	//scale_cpu_cycles_to_microseconds = 0xFFFFFFFFu / (uint32_t)(frequency / 1000000u);

	printf_debug("New Frequency: ARM=%u, IPG=%u\n", frequency, frequency / div_ipg);

	// if voltage needs to decrease, do it after switch clock speed
	if ((dcdc & DCDC_REG3_TRG_MASK) > DCDC_REG3_TRG((voltage - 800) / 25)) {
		printf_debug("Decreasing voltage to %u mV\n", voltage);
		dcdc &= ~DCDC_REG3_TRG_MASK;
		dcdc |= DCDC_REG3_TRG((voltage - 800) / 25);
		DCDC->REG3 = dcdc;
		while (!(DCDC->REG0 & DCDC_REG0_STS_DC_OK(1))) ; // wait voltage settling
	}

	return frequency;
}

void reset_PFD(void) {
  //Reset PLL2 PFDs, set default frequencies:
  CCM_ANALOG->PFD_528_SET = (1 << 31) | (1 << 23) | (1 << 15) | (1 << 7);
  CCM_ANALOG->PFD_528 = 0x2018101B; // PFD0:352, PFD1:594, PFD2:396, PFD3:297 MHz 	
  //PLL3:
  CCM_ANALOG->PFD_480_SET = (1 << 31) | (1 << 23) | (1 << 15) | (1 << 7);	
  CCM_ANALOG->PFD_480 = 0x13110D0C; // PFD0:720, PFD1:664, PFD2:508, PFD3:454 MHz
}

#define NVIC_NUM_INTERRUPTS 160
extern uint32_t _vectors[NVIC_NUM_INTERRUPTS];

/* __attribute__ ((used, aligned(1024))) */
/* void (* _VectorsRam[NVIC_NUM_INTERRUPTS+16])(void); */

// Stack frame
//  xPSR
//  ReturnAddress
//  LR (R14) - typically FFFFFFF9 for IRQ or Exception
//  R12
//  R3
//  R2
//  R1
//  R0
// Code from :: https://community.nxp.com/thread/389002
__attribute__((naked))
void unused_interrupt_vector(void)
{
  __asm( ".syntax unified\n"
         "MOVS R0, #4 \n"
         "MOV R1, LR \n"
         "TST R0, R1 \n"
         "BEQ _MSP \n"
         "MRS R0, PSP \n"
         "B HardFault_HandlerC \n"
         "_MSP: \n"
         "MRS R0, MSP \n"
         "B HardFault_HandlerC \n"
         ".syntax divided\n") ;
}

#define HALT_IF_DEBUGGING()                              \
  do {                                                   \
    if ((*(volatile uint32_t *)0xE000EDF0) & (1 << 0)) { \
      __asm("bkpt 1");                                   \
    }                                                    \
} while (0)

__attribute__((weak,used))
void HardFault_HandlerC(unsigned int *hardfault_args)
{
  //HALT_IF_DEBUGGING();
  volatile unsigned int nn ;
#if 1 /* PRINT_DEBUG_STUFF*/
  volatile unsigned int stacked_r0 ;
  volatile unsigned int stacked_r1 ;
  volatile unsigned int stacked_r2 ;
  volatile unsigned int stacked_r3 ;
  volatile unsigned int stacked_r12 ;
  volatile unsigned int stacked_lr ;
  volatile unsigned int stacked_pc ;
  volatile unsigned int stacked_psr ;
  volatile unsigned int _CFSR ;
  volatile unsigned int _HFSR ;
  volatile unsigned int _DFSR ;
  volatile unsigned int _AFSR ;
  volatile unsigned int _BFAR ;
  volatile unsigned int _MMAR ;
  volatile unsigned int addr ;

  stacked_r0 = ((unsigned int)hardfault_args[0]) ;
  stacked_r1 = ((unsigned int)hardfault_args[1]) ;
  stacked_r2 = ((unsigned int)hardfault_args[2]) ;
  stacked_r3 = ((unsigned int)hardfault_args[3]) ;
  stacked_r12 = ((unsigned int)hardfault_args[4]) ;
  stacked_lr = ((unsigned int)hardfault_args[5]) ;
  stacked_pc = ((unsigned int)hardfault_args[6]) ;
  stacked_psr = ((unsigned int)hardfault_args[7]) ;
  // Configurable Fault Status Register
  // Consists of MMSR, BFSR and UFSR
  //(n & ( 1 << k )) >> k
  _CFSR = (*((volatile unsigned int *)(0xE000ED28))) ;  
  // Hard Fault Status Register
  _HFSR = (*((volatile unsigned int *)(0xE000ED2C))) ;
  // Debug Fault Status Register
  _DFSR = (*((volatile unsigned int *)(0xE000ED30))) ;
  // Auxiliary Fault Status Register
  _AFSR = (*((volatile unsigned int *)(0xE000ED3C))) ;
  // Read the Fault Address Registers. These may not contain valid values.
  // Check BFARVALID/MMARVALID to see if they are valid values
  // MemManage Fault Address Register
  _MMAR = (*((volatile unsigned int *)(0xE000ED34))) ;
  // Bus Fault Address Register
  _BFAR = (*((volatile unsigned int *)(0xE000ED38))) ;
  //__asm("BKPT #0\n") ; // Break into the debugger // NO Debugger here.

  asm volatile("mrs %0, ipsr\n" : "=r" (addr)::);
  printf_debug("\nFault irq %d\n", addr & 0x1FF);
  printf_debug(" stacked_r0 ::  %x\n", stacked_r0);
  printf_debug(" stacked_r1 ::  %x\n", stacked_r1);
  printf_debug(" stacked_r2 ::  %x\n", stacked_r2);
  printf_debug(" stacked_r3 ::  %x\n", stacked_r3);
  printf_debug(" stacked_r12 ::  %x\n", stacked_r12);
  printf_debug(" stacked_lr ::  %x\n", stacked_lr);
  printf_debug(" stacked_pc ::  %x\n", stacked_pc);
  printf_debug(" stacked_psr ::  %x\n", stacked_psr);
  printf_debug(" _CFSR ::  %x\n", _CFSR);
 
  if(_CFSR > 0){
	  //Memory Management Faults
	  if((_CFSR & 1) == 1){
		printf_debug("      (IACCVIOL) Instruction Access Violation\n");
	  } else  if(((_CFSR & (0x02))>>1) == 1){
		printf_debug("      (DACCVIOL) Data Access Violation\n");
	  } else if(((_CFSR & (0x08))>>3) == 1){
		printf_debug("      (MUNSTKERR) MemMange Fault on Unstacking\n");
	  } else if(((_CFSR & (0x10))>>4) == 1){
		printf_debug("      (MSTKERR) MemMange Fault on stacking\n");
	  } else if(((_CFSR & (0x20))>>5) == 1){
		printf_debug("      (MLSPERR) MemMange Fault on FP Lazy State\n");
	  }
	  if(((_CFSR & (0x80))>>7) == 1){
		printf_debug("      (MMARVALID) MemMange Fault Address Valid\n");
	  }
	  //Bus Fault Status Register
	  if(((_CFSR & 0x100)>>8) == 1){
		printf_debug("      (IBUSERR) Instruction Bus Error\n");
	  } else  if(((_CFSR & (0x200))>>9) == 1){
		printf_debug("      (PRECISERR) Data bus error(address in BFAR)\n");
	  } else if(((_CFSR & (0x400))>>10) == 1){
		printf_debug("      (IMPRECISERR) Data bus error but address not related to instruction\n");
	  } else if(((_CFSR & (0x800))>>11) == 1){
		printf_debug("      (UNSTKERR) Bus Fault on unstacking for a return from exception \n");
	  } else if(((_CFSR & (0x1000))>>12) == 1){
		printf_debug("      (STKERR) Bus Fault on stacking for exception entry\n");
	  } else if(((_CFSR & (0x2000))>>13) == 1){
		printf_debug("      (LSPERR) Bus Fault on FP lazy state preservation\n");
	  }
	  if(((_CFSR & (0x8000))>>15) == 1){
		printf_debug("      (BFARVALID) Bus Fault Address Valid\n");
	  }  
	  //Usuage Fault Status Register
	  if(((_CFSR & 0x10000)>>16) == 1){
		printf_debug("      (UNDEFINSTR) Undefined instruction\n");
	  } else  if(((_CFSR & (0x20000))>>17) == 1){
		printf_debug("      (INVSTATE) Instruction makes illegal use of EPSR)\n");
	  } else if(((_CFSR & (0x40000))>>18) == 1){
		printf_debug("      (INVPC) Usage fault: invalid EXC_RETURN\n");
	  } else if(((_CFSR & (0x80000))>>19) == 1){
		printf_debug("      (NOCP) No Coprocessor \n");
	  } else if(((_CFSR & (0x1000000))>>24) == 1){
		printf_debug("      (UNALIGNED) Unaligned access UsageFault\n");
	  } else if(((_CFSR & (0x2000000))>>25) == 1){
		printf_debug("      (DIVBYZERO) Divide by zero\n");
	  }
  }
  printf_debug(" _HFSR ::  %x\n", _HFSR);
  if(_HFSR > 0){
	  //Memory Management Faults
	  if(((_HFSR & (0x02))>>1) == 1){
		printf_debug("      (VECTTBL) Bus Fault on Vec Table Read\n");
	  } else if(((_HFSR & (0x40000000))>>30) == 1){
		printf_debug("      (FORCED) Forced Hard Fault\n");
	  } else if(((_HFSR & (0x80000000))>>31) == 31){
		printf_debug("      (DEBUGEVT) Reserved for Debug\n");
	  } 
  }
  printf_debug(" _DFSR ::  %x\n", _DFSR);
  printf_debug(" _AFSR ::  %x\n", _AFSR);
  printf_debug(" _BFAR ::  %x\n", _BFAR);
  printf_debug(" _MMAR ::  %x\n", _MMAR);
#endif

  #if 0
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 5; // pin 13
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_03 = IOMUXC_PAD_DSE(7);
  GPIO7_GDIR |= (1 << 3);
  GPIO7_DR_SET = (1 << 3);
  #endif
  GPIO7->DR_CLEAR = (1 << 3); //digitalWrite(13, LOW);

  //if ( F_CPU_ACTUAL >= 600000000 )
    set_arm_clock(300000000);

  while (1)
  {
    GPIO7->DR_SET = (1 << 3); //digitalWrite(13, HIGH);
    // digitalWrite(13, HIGH);
    for (nn = 0; nn < 2000000/2; nn++) ;
    GPIO7->DR_CLEAR = (1 << 3); //digitalWrite(13, LOW);
    // digitalWrite(13, LOW);
    for (nn = 0; nn < 18000000/2; nn++) ;
  }
}

extern uint32_t __main_stack_end__;
extern uint32_t __process_stack_end__;

// flexram initialization must happen before crt0 initializes stack pointers
void __flexram_init(void) {
	/* See also https://www.nxp.com/docs/en/application-note/AN12077.pdf */

  // TODO: AN12077 says that OCRAM cannot be 0, but that’s what the teensy does?!
  
	// we have 16 banks of memory, and 512 KB of FlexRAM, divided into 512 KB / 16 = 32 KB
	// 0xAAAAAAAB
	// = 10101010101010101010101010101011
	// where 11b is ITCM
	// and 10b is DTCM
	// GPR17 = FLEXRAM_BANK_CFG
	//IOMUXC_GPR->GPR17 = 0xAAAAAAAB; // TODO: (uint32_t)&_flexram_bank_config;
	IOMUXC_GPR->GPR17 = 0xAAAAAAAA; // TODO: (uint32_t)&_flexram_bank_config;

	// TODO: when is CM7_INIT_VTOR used? it says VTOR out of reset, but… is
	// that only relevant for the next reset?

	// GPR16 contains CM7_INIT_VTOR (0x00200) | FLEXRAM_BANK_CFG(1) | INIT_DTCM_EN(1) | INIT_ITCM_EN(1)
	IOMUXC_GPR->GPR16 =
	  0x00200000 |
	  IOMUXC_GPR_GPR16_FLEXRAM_BANK_CFG_SEL(1) |
	  IOMUXC_GPR_GPR16_INIT_DTCM_EN(1) |
	  IOMUXC_GPR_GPR16_INIT_ITCM_EN(0);
	  // 0x00200007;

	// GPR14
	// ( 11111111111111111111111111111111b)
	// = xxxxxxxx101010100000000000000000b
	//           ^^^^ = CM7_CFGDTCMSZ(512K)
	//               ^^^^ = CM7_CFGITCMSZ(512K)
	IOMUXC_GPR->GPR14 =
	  IOMUXC_GPR_GPR14_CM7_CFGDTCMSZ(10 /* 512 KB */) |
	  IOMUXC_GPR_GPR14_CM7_CFGITCMSZ(0 /* No ITCM */);
	//	  0x00AA0000;

	// ChibiOS sets up stack pointers in crt0.
	//__asm__ volatile("mov sp, %0" : : "r" ((uint32_t)&_estack) : );

	// Set up separate MSP/PSP for ChibiOS
	// https://interrupt.memfault.com/blog/cortex-m-rtos-context-switching
	/* asm volatile("msr msp, %0" : : "r" ((uint32_t)&__main_stack_end__) : ); */
	/* asm volatile("msr psp, %0" : : "r" ((uint32_t)&__process_stack_end__) : ); */
}

/**
 * @brief   MIMXRT1062 clock initialization.
 * @note    All the involved constants come from the file @p board.h.
 * @note    This function is meant to be invoked early during the system
 *          initialization, it is usually invoked from the file
 *          @p board.c.
 * @note    This is based on https://github.com/PaulStoffregen/cores/blob/master/teensy4/startup.c
 *
 * @special
 */
void MIMXRT1062_clock_init(void) {
  //unsigned int i;

#if 0
	for (int c = 0; c < 1; c++) {
	  GPIO7->DR_SET = (1<<3); // digitalWrite(13, HIGH);
	  delay(600000000); // 1s
	  GPIO7->DR_CLEAR = (1<<3); // digitalWrite(13, LOW);
	  delay(600000000); // 1s
	}
#endif
  
  
	PMU->MISC0_SET = 1<<3; //Use bandgap-based bias currents for best performance (Page 1175)

	// pin 13 - if startup crashes, use this to turn on the LED early for troubleshooting
	IOMUXC->SW_MUX_CTL_PAD[kIOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03] = 5;
	IOMUXC->SW_PAD_CTL_PAD[kIOMUXC_SW_PAD_CTL_PAD_GPIO_B0_03] = IOMUXC_SW_PAD_CTL_PAD_DSE(7);
	IOMUXC_GPR->GPR27 = 0xFFFFFFFF;
	GPIO7->GDIR |= (1<<3);
	GPIO7->DR_SET = (1<<3); // digitalWrite(13, HIGH);
}

// In ChibiOS, memory set up happens after the clock initialization:
// https://github.com/ChibiOS/ChibiOS/blob/19a236b0de2c1f97ce9b82ac29675c80fa629778/os/common/startup/ARMCMx/compilers/GCC/crt0_v7m.S#L256

// In ChibiOS, the FPU is enabled conditionally at
// https://github.com/ChibiOS/ChibiOS/blob/19a236b0de2c1f97ce9b82ac29675c80fa629778/os/common/startup/ARMCMx/compilers/GCC/crt0_v7m.S#L209

void MIMXRT1062_late_init(void) {

  #define NVIC_SET_PRIORITY(irqnum, priority)  (*((volatile uint8_t *)0xE000E400 + (irqnum)) = (uint8_t)(priority))

	// teensy4/startup.c copies the vector table into RAM here.
	// We don’t need that, as we don’t modify the vector table at runtime.
	
	// TODO: set up interrupts, or add reference to where ChibiOS does it
	//for (i=0; i < NVIC_NUM_INTERRUPTS + 16; i++) _VectorsRam[i] = &unused_interrupt_vector;
	//for (i=0; i < NVIC_NUM_INTERRUPTS; i++) NVIC_SET_PRIORITY(i, 128);
	//for (i=0; i < NVIC_NUM_INTERRUPTS; i++) nvicSetSystemHandlerPriority(i, 128);
	//SCB->VTOR = (uint32_t)_VectorsRam;

	reset_PFD();

	// Configure clocks
	// TODO: make sure all affected peripherals are turned off!
	// PIT & GPT timers to run from 24 MHz clock (independent of CPU speed)
	CCM->CSCMR1 = (CCM->CSCMR1 & ~CCM_CSCMR1_PERCLK_PODF(0x3F)) | CCM_CSCMR1_PERCLK_CLK_SEL_MASK;
	// UARTs run from 24 MHz clock (works if PLL3 off or bypassed)
	CCM->CSCDR1 = (CCM->CSCDR1 & ~CCM_CSCDR1_UART_CLK_PODF(0x3F)) | CCM_CSCDR1_UART_CLK_SEL_MASK;

#if 1 // GPIO does not work without this:
	// Use fast GPIO6, GPIO7, GPIO8, GPIO9
	IOMUXC_GPR->GPR26 = 0xFFFFFFFF;
	IOMUXC_GPR->GPR27 = 0xFFFFFFFF;
	IOMUXC_GPR->GPR28 = 0xFFFFFFFF;
	IOMUXC_GPR->GPR29 = 0xFFFFFFFF;
#endif
	
	printf_debug_init();
	printf_debug("\n***********IMXRT Chibi Startup**********\n");
	printf_debug("test %d %d %d\n", 1, -1234567, 3);

	// TODO: this should be the address of _vectors,
	// which is currently not correctly aligned!
	// I suspect this was not a problem by accident
	// because the vectors went to the beginning of the flash before.
	printf_debug("SCB->VTOR = %x\n", SCB->VTOR);
	//printf_debug("vectors: %d\n", CORTEX_NUM_VECTORS); // 104, but should be 160
	
	configure_cache();
	
	// TODO: should be done by ChibiOS-Contrib/os/hal/ports/MIMXRT1062/LLD/PITv1/hal_st_lld.c i guess?	
	//configure_systick();

#if 0
	for (int c = 0; c < 1; c++) {
	  GPIO7->DR_SET = (1<<3); // digitalWrite(13, HIGH);
	  delay(600000000); // 1s
	  GPIO7->DR_CLEAR = (1<<3); // digitalWrite(13, LOW);
	  delay(600000000); // 1s
	}
#endif
	
	reset_PFD();
	usb_pll_start();

	set_arm_clock(600000000);

	// TODO: why is this here?
	asm volatile("nop\n nop\n nop\n nop": : :"memory"); // why oh why?

#if 0	
	for (int c = 0; c < 1; c++) {
	  GPIO7->DR_SET = (1<<3); // digitalWrite(13, HIGH);
	  delay(600000000); // 1s
	  GPIO7->DR_CLEAR = (1<<3); // digitalWrite(13, LOW);
	  delay(600000000); // 1s
	}
#endif	

}


extern void Reset_Handler(void);
extern unsigned long _flashimagelen;

// IMXRT1060RM: 9.5.5 Exception handling
// A minimal vector table with only the first 2 elements
__attribute__ ((section(".vectors"), used, aligned(1024)))
const uint32_t vector_table[2] = {
  // initial SP (stack pointer) value when booting:
#if 1 // defined(__IMXRT1062__)
	0x20010000, // 64K DTCM for boot, ResetHandler configures stack after ITCM/DTCM setup
#endif
	// initial PC (program count) value when booting:
	(uint32_t)&Reset_Handler
};

/* See section 2.5.2 in https://www.nxp.com/docs/en/application-note/AN12107.pdf */
__attribute__ ((section(".bootdata"), used))
const uint32_t BootData[3] = {
  // destination address is equal to the external flash address, so the i.MX
  // BootROM will skip any remaining memory copies and start up the application
  // binary directly in the flash address space.
  // See section 3.2.2 in https://www.nxp.com/docs/en/nxp/application-notes/AN12238.pdf
  	0x60000000,                // absolute address of the bootable image
	// The following size normally determines how many bytes are copied from
	// flash to RAM, but because the destination address is equal to the
	// flash address, no copying is taking place.
	//
	// So, logically, we would set this field to 0, but 32 is the minimum
	// size that works in practice. My guess is that 32 is the size of the
	// IVT, and perhaps the BootROM code needs the IVT to be present and
	// accounted for.
	32,
	0,                         // plugin flag, 0 = normal boot image
};


/* See section 2.5.1 in https://www.nxp.com/docs/en/application-note/AN12107.pdf */
__attribute__ ((section(".ivt"), used))
const uint32_t ImageVectorTable[8] = {
	0x402000D1,		// header
	(uint32_t)vector_table, // docs are wrong, needs to be vec table, not start addr
	0,			// reserved
	0,			// DCD (Device Configuration Data), e.g. for SDRAM during boot
	(uint32_t)BootData,	// abs address of boot data
	(uint32_t)ImageVectorTable, // self
	0,			// command sequence file
	0			// reserved
};

__attribute__ ((section(".flashconfig"), used))
uint32_t FlexSPI_NOR_Config[128] = {
	// 448 byte common FlexSPI configuration block, 8.6.3.1 page 223 (RT1060 rev 0)
	// MCU_Flashloader_Reference_Manual.pdf, 8.2.1, Table 8-2, page 72-75
	0x42464346,		// Tag				0x00
	0x56010000,		// Version
	0,			// reserved
	0x00020101,		// columnAdressWidth,dataSetupTime,dataHoldTime,readSampleClkSrc

	0x00000000,		// waitTimeCfgCommands,-,deviceModeCfgEnable
	0,			// deviceModeSeq
	0, 			// deviceModeArg
	0x00000000,		// -,-,-,configCmdEnable

	0,			// configCmdSeqs		0x20
	0,
	0,
	0,

	0,			// cfgCmdArgs			0x30
	0,
	0,
	0,

	0x00000000,		// controllerMiscOption		0x40
	0x00030401,		// lutCustomSeqEnable,serialClkFreq,sflashPadType,deviceType
	0,			// reserved
	0,			// reserved
// TODO:
#define ARDUINO_TEENSY41 1
	
#if defined(ARDUINO_TEENSY40)
	0x00200000,		// sflashA1Size			0x50
#elif defined(ARDUINO_TEENSY41)
	0x00800000,		// sflashA1Size			0x50
#else
#error "Unknow flash chip size";
#endif
	0,			// sflashA2Size
	0,			// sflashB1Size
	0,			// sflashB2Size

	0,			// csPadSettingOverride		0x60
	0,			// sclkPadSettingOverride
	0,			// dataPadSettingOverride
	0,			// dqsPadSettingOverride

	0,			// timeoutInMs			0x70
	0,			// commandInterval
	0,			// dataValidTime
	0x00000000,		// busyBitPolarity,busyOffset

	0x0A1804EB,		// lookupTable[0]		0x80
	0x26043206,		// lookupTable[1]
	0,			// lookupTable[2]
	0,			// lookupTable[3]

	0x24040405,		// lookupTable[4]		0x90
	0,			// lookupTable[5]
	0,			// lookupTable[6]
	0,			// lookupTable[7]

	0,			// lookupTable[8]		0xA0
	0,			// lookupTable[9]
	0,			// lookupTable[10]
	0,			// lookupTable[11]

	0x00000406,		// lookupTable[12]		0xB0
	0,			// lookupTable[13]
	0,			// lookupTable[14]
	0,			// lookupTable[15]

	0,			// lookupTable[16]		0xC0
	0,			// lookupTable[17]
	0,			// lookupTable[18]
	0,			// lookupTable[19]

	0x08180420,		// lookupTable[20]		0xD0
	0,			// lookupTable[21]
	0,			// lookupTable[22]
	0,			// lookupTable[23]

	0,			// lookupTable[24]		0xE0
	0,			// lookupTable[25]
	0,			// lookupTable[26]
	0,			// lookupTable[27]

	0,			// lookupTable[28]		0xF0
	0,			// lookupTable[29]
	0,			// lookupTable[30]
	0,			// lookupTable[31]

	0x081804D8,		// lookupTable[32]		0x100
	0,			// lookupTable[33]
	0,			// lookupTable[34]
	0,			// lookupTable[35]

	0x08180402,		// lookupTable[36]		0x110
	0x00002004,		// lookupTable[37]
	0,			// lookupTable[38]
	0,			// lookupTable[39]

	0,			// lookupTable[40]		0x120
	0,			// lookupTable[41]
	0,			// lookupTable[42]
	0,			// lookupTable[43]

	0x00000460,		// lookupTable[44]		0x130
	0,			// lookupTable[45]
	0,			// lookupTable[46]
	0,			// lookupTable[47]

	0,			// lookupTable[48]		0x140
	0,			// lookupTable[49]
	0,			// lookupTable[50]
	0,			// lookupTable[51]

	0,			// lookupTable[52]		0x150
	0,			// lookupTable[53]
	0,			// lookupTable[54]
	0,			// lookupTable[55]

	0,			// lookupTable[56]		0x160
	0,			// lookupTable[57]
	0,			// lookupTable[58]
	0,			// lookupTable[59]

	0,			// lookupTable[60]		0x170
	0,			// lookupTable[61]
	0,			// lookupTable[62]
	0,			// lookupTable[63]

	0,			// LUT 0: Read			0x180
	0,			// LUT 1: ReadStatus
	0,			// LUT 3: WriteEnable
	0,			// LUT 5: EraseSector

	0,			// LUT 9: PageProgram		0x190
	0,			// LUT 11: ChipErase
	0,			// LUT 15: Dummy
	0,			// LUT unused?

	0,			// LUT unused?			0x1A0
	0,			// LUT unused?
	0,			// LUT unused?
	0,			// LUT unused?

	0,			// reserved			0x1B0
	0,			// reserved
	0,			// reserved
	0,			// reserved

	// 64 byte Serial NOR configuration block, 8.6.3.2, page 346

	256,			// pageSize			0x1C0
	4096,			// sectorSize
	1,			// ipCmdSerialClkFreq
	0,			// reserved

	0x00010000,		// block size			0x1D0
	0,			// reserved
	0,			// reserved
	0,			// reserved

	0,			// reserved			0x1E0
	0,			// reserved
	0,			// reserved
	0,			// reserved

	0,			// reserved			0x1F0
	0,			// reserved
	0,			// reserved
	0			// reserved
};



/** @} */

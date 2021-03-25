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

#include "clock_config.h"

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

// Enable to get startup debug on ChibiOS LPUART3 (== NXP peripheral LPUART4),
// i.e. pin 17 on the Teensy 4.1:
#if 0 /* PRINT_DEBUG_STUFF */
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
#else

#define printf_init()
#define printf(...)
#define printf_debug_init()
#define printf_debug(...)

#endif /* PRINT_DEBUG_STUFF */

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
	
	/* MPU->RBAR = 0x00000000 | REGION(i++); // ITCM */
	/* MPU->RASR = MEM_NOCACHE | READWRITE | SIZE_512K; */

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
	MPU->RASR = MEM_CACHE_WBWA | READWRITE | NOEXEC | SIZE_16M;

	// TODO: protect access to power supply config

	MPU->CTRL = SCB_MPU_CTRL_ENABLE;
#endif

	// ChibiOS initializes the cache in __cpu_init
}

#define NVIC_NUM_INTERRUPTS 160
extern uint32_t _vectors[NVIC_NUM_INTERRUPTS];

// In ChibiOS, memory set up happens after the clock initialization:
// https://github.com/ChibiOS/ChibiOS/blob/19a236b0de2c1f97ce9b82ac29675c80fa629778/os/common/startup/ARMCMx/compilers/GCC/crt0_v7m.S#L256

// In ChibiOS, the FPU is enabled conditionally at
// https://github.com/ChibiOS/ChibiOS/blob/19a236b0de2c1f97ce9b82ac29675c80fa629778/os/common/startup/ARMCMx/compilers/GCC/crt0_v7m.S#L209

uint32_t SystemCoreClock = 528000000UL; // default system clock
  
void MIMXRT1062_late_init(void) {
  // TODO: do we only want the clock enable from BOARD_InitPins(), or also the
  // GPIO mux definitions?
  CLOCK_EnableClock(kCLOCK_Iomuxc);

  BOARD_InitBootClocks();
  //SystemCoreClockUpdate();

  // TODO: is the following covered by NXP startup code somewhere?
  // Use fast GPIO6, GPIO7, GPIO8, GPIO9
  IOMUXC_GPR->GPR26 = 0xFFFFFFFF;
  IOMUXC_GPR->GPR27 = 0xFFFFFFFF;
  IOMUXC_GPR->GPR28 = 0xFFFFFFFF;
  IOMUXC_GPR->GPR29 = 0xFFFFFFFF;
  
  // TODO: turn on power LED
  
  printf_debug_init();
  printf_debug("\n***********IMXRT Chibi Startup**********\n");
  printf_debug("test %d %d %d\n", 1, -1234567, 3);

  // Explicitly warn for a few issues I ran into, to catch possible
  // problems automatically when working on startup code:
  if (SCB->VTOR != (uint32_t)&_vectors) {
    printf_debug("WARNING: unexpected SCB->VTOR value %x, expected &_vectors = %x\n", SCB->VTOR, (uint32_t)&_vectors);
  }
  if ((uint32_t)&_vectors % 4 != 0) {
    printf_debug("WARNING: &_vectors = %x is unexpectedly not aligned by 4\n", (uint32_t)&_vectors);
  }
  if (CORTEX_NUM_VECTORS != 160) {
    printf_debug("WARNING: unexpected CORTEX_NUM_VECTORS = %d, want %d", CORTEX_NUM_VECTORS, 160);
  }
	
  configure_cache();
}


extern void Reset_Handler(void);
extern unsigned long _flashimagelen;

// trampoline_reset_handler initializes FlexRAM, then jumps to the ChibiOS
// Reset_Handler. This is required because the ChibiOS crt0 does not provide a
// hook in a suitable place to integrate FlexRAM configuration.
//
// Note that when loading an image into the debugger, the ELF entry point
// (specified by ENTRY(Reset_Handler) in rules_code.ld) is used directly, and
// our trampoline_reset_handler that we configure in the Image Vector Table
// (IVT) is not called. Instead, the debugger Connect Script
// (e.g. rt1060_connect.scp) is responsible for setting up FlexRAM.  Instead of
// modifying the Connect Script accordingly, it might be easier to change
// ENTRY(Reset_Handler) to ENTRY(trampoline_reset_handler) for debugging.
__attribute__((target("thumb"), aligned(2)))
void trampoline_reset_handler(void) {
  __disable_irq();

  // Switch to final VTOR as quickly as possible to have fault handlers set up
  // (e.g. HardFault) for easier debugging. When encountering a fault without a
  // usable VTOR table, the MCU will raise another fault and end up in lockup.
  SCB->VTOR = (uint32_t)&_vectors;
  
  IOMUXC_GPR->GPR17 = 0xaaaaaaaa;
  __DSB();
  __ISB();
  IOMUXC_GPR->GPR16 &= ~IOMUXC_GPR_GPR16_INIT_ITCM_EN_MASK;
  IOMUXC_GPR->GPR16 |=
    IOMUXC_GPR_GPR16_FLEXRAM_BANK_CFG_SEL(1) |
    IOMUXC_GPR_GPR16_INIT_DTCM_EN(1) |
    IOMUXC_GPR_GPR16_INIT_ITCM_EN(0);

  __DSB();
  __ISB();

  uint32_t current_gpr14 = IOMUXC_GPR->GPR14;
  current_gpr14 &= ~IOMUXC_GPR_GPR14_CM7_CFGDTCMSZ_MASK;
  current_gpr14 |= IOMUXC_GPR_GPR14_CM7_CFGDTCMSZ(10);
  current_gpr14 &= ~IOMUXC_GPR_GPR14_CM7_CFGITCMSZ_MASK;
  current_gpr14 |= IOMUXC_GPR_GPR14_CM7_CFGITCMSZ(0);
  IOMUXC_GPR->GPR14 = current_gpr14;

  __DSB();
  __ISB();

  Reset_Handler();
}

// IMXRT1060RM: 9.5.5 Exception handling
// A minimal vector table with only the first 2 elements, i.e. initial SP (stack
// pointer) and PC (program count) values.
__attribute__ ((section(".vectors"), used, aligned(1024)))
const uint32_t vector_table[] = {
  // Initial SP (stack pointer) value when booting.
  // Will be updated to point to DTCM FlexRAM by ChibiOS crt0_v7m.S.
  0x20201000, // OCRAM, always available regardless of FlexRAM setting.

  // Initial PC (program count) value when booting.
  (uint32_t)&trampoline_reset_handler, // jumps to Reset_Handler
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
	// accounted for. The NXP examples just set this to the size of the
	// flash, so we do the same:
	1984*1024,
	0,                         // plugin flag, 0 = normal boot image
};


/* See section 2.5.1 in https://www.nxp.com/docs/en/application-note/AN12107.pdf */
__attribute__ ((section(".ivt"), used))
const uint32_t ImageVectorTable[8] = {
	0x402000D1,		// header
	(uint32_t)vector_table, // docs are wrong, needs to be vec table, not start addr
	0,			// reserved

	// DCD (Device Configuration Data), e.g. for SDRAM during boot.  Note
	// that the BootROM of the i.MX RT 1060 does not actually support DCD,
	// so this field must always be set to 0:
	//
	// IMXRT1060RM: NOTE: The DCD is not supported in the BootROM, in this
	// device. It must be set to 0x00.
	0,
	
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
	0x00030301,		// columnAdressWidth,dataSetupTime,dataHoldTime,readSampleClkSrc

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
	// The Teensy 4 config used to run the FlexSPI Serial Clock at 60 MHz:
	// https://github.com/PaulStoffregen/cores/commit/c346fc36ed97dcaed2fa1d70626fbd80cf35586d
	// whereas NXP is running it with 100 MHz. With the old default of 60
	// MHz, I occasionally get hard faults when reading data from flash.
	0x00080401,		// lutCustomSeqEnable,serialClkFreq,sflashPadType,deviceType
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

/*
 * Copyright (C) 2013-2016 Fabio Utzig, http://fabioutzig.com
 *           (C) 2016 flabbergast <s3+flabbergast@sdfeu.org>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

MEMORY
{
	ITCM (rwx):  ORIGIN = 0x00000000, LENGTH = 512K
	DTCM (rwx):  ORIGIN = 0x20000000, LENGTH = 512K
	RAM (rwx):   ORIGIN = 0x20200000, LENGTH = 512K
	FLASH (rwx): ORIGIN = 0x60000000, LENGTH = 1984K
}

ENTRY(ImageVectorTable)

SECTIONS
{
	.text.progmem : {
		KEEP(*(.flashconfig))
		FILL(0xFF)
		. = ORIGIN(FLASH) + 0x1000;
		KEEP(*(.ivt))
		KEEP(*(.bootdata))
		KEEP(*(.vectors))
		KEEP(*(.startup))
		*(.flashmem*)
		*(.progmem*)
                . = ALIGN(4);
                KEEP(*(.init))
                __preinit_array_start = .;
                KEEP (*(.preinit_array))
                __preinit_array_end = .;
                __init_array_start = .;
                KEEP (*(.init_array))
                __init_array_end = .;
		. = ALIGN(16);
	} > FLASH

	.text.itcm : {
		. = . + 32; /* MPU to trap NULL pointer deref */
		*(.fastrun)
		*(.text*)
		. = ALIGN(16);
	} > ITCM  AT> FLASH

	.text.itcm.padding (NOLOAD) : {
		. = ALIGN(32768);
	} > ITCM

	.data : {
		*(.rodata*)
		*(.data*)
		. = ALIGN(16);
	} > DTCM  AT> FLASH

	.bss ALIGN(4) : {
		*(.bss*)
		*(COMMON)
		. = ALIGN(32);
		. = . + 32; /* MPU to trap stack overflow */
	} > DTCM

	.bss.dma (NOLOAD) : {
		*(.dmabuffers)
		. = ALIGN(32);
	} > RAM

    /* Special section for exceptions stack.*/
    .mstack (NOLOAD) :
    {
        . = ALIGN(8);
        __main_stack_base__ = .;
        . += __main_stack_size__;
        . = ALIGN(8);
        __main_stack_end__ = .;
    } > DTCM

    /* Special section for process stack.*/
    .pstack (NOLOAD) :
    {
        . = ALIGN(8);
        __process_stack_base__ = .;
        __main_thread_stack_base__ = .;
        . += __process_stack_size__;
        . = ALIGN(8);
        __process_stack_end__ = .;
        __main_thread_stack_end__ = .;
    } > DTCM


	_stext = ADDR(.text.itcm);
	_etext = ADDR(.text.itcm) + SIZEOF(.text.itcm);
	_stextload = LOADADDR(.text.itcm);

	_sdata = ADDR(.data);
	_edata = ADDR(.data) + SIZEOF(.data);
	_sdataload = LOADADDR(.data);

	_sbss = ADDR(.bss);
	_ebss = ADDR(.bss) + SIZEOF(.bss);

	_heap_start = ADDR(.bss.dma) + SIZEOF(.bss.dma);
	__heap_base__ = _heap_start;
	_heap_end = ORIGIN(RAM) + LENGTH(RAM);
	__heap_end__ = _heap_end;

	_itcm_block_count = (SIZEOF(.text.itcm) + 0x7FFF) >> 15;
	_flexram_bank_config = 0xAAAAAAAA | ((1 << (_itcm_block_count * 2)) - 1);
	_estack = ORIGIN(DTCM) + ((16 - _itcm_block_count) << 15);

	_flashimagelen = SIZEOF(.text.progmem) + SIZEOF(.text.itcm) + SIZEOF(.data);
	_teensy_model_identifier = 0x24;

	.debug_info     0 : { *(.debug_info) }
	.debug_abbrev   0 : { *(.debug_abbrev) }
	.debug_line     0 : { *(.debug_line) }
	.debug_frame    0 : { *(.debug_frame) }
	.debug_str      0 : { *(.debug_str) }
	.debug_loc      0 : { *(.debug_loc) }

}


/* /\* */
/*  * MIMXRT1062 memory setup. */
/*  *\/ */
/* MEMORY */
/* { */
/*   /\* */
/*    * The top 64K of flash are unavailable: */
/*    * */
/*    * 60K are reserved for EEPROM emulation, */
/*    *  4K are special read-only memory containing a known-good blink program */
/*    * */
/*    * See also https://www.pjrc.com/store/teensy40.html#memory_layout */
/*    *\/ */
/*   flash0   : org = 0x60000000, len = 1984K  /\* FLASH *\/ */
/*   flash1   : org = 0x00000000, len = 0 */
/*   flash2   : org = 0x00000000, len = 0 */
/*   flash3   : org = 0x00000000, len = 0 */

/*   /\* Possible optimization: place code marked as FASTRUN in ram0 *\/ */
/*   /\* ram0: ITCM, can be TCM or general purpose */
/*      IMXRT1060RM: Page 36 Table 3-1 System memory map (CM7) */
/*      0000_0000 Size: 512KB ITCM  *\/ */
/*   ram0     : org = 0x00000000, len = 128k */
/*   /\* ram1: DTCM, general purpose only */
/*      IMXRT1060RM: Page 36 Table 3-1 System memory map (CM7) */
/*      2000_0000 Size: 512KB DTCM  *\/ */
/*   ram1     : org = 0x20000000, len = 128k */
/*   /\* ram2: OCRAM (On-Chip RAM), for new/malloc() and DMAMEM variables */
/*      IMXRT1060RM: Page 36 Table 3-1 System memory map (CM7) */
/*      2020_0000 Size: 512KB OCRAM2  *\/ */
/*   ram2     : org = 0x20200000, len = 512k */

/*   ram3     : org = 0x00000000, len = 0 */
/*   ram4     : org = 0x00000000, len = 0 */
/*   ram5     : org = 0x00000000, len = 0 */
/*   ram6     : org = 0x00000000, len = 0 */
/*   ram7     : org = 0x00000000, len = 0 */
/* } */

/* /\* Flash region for the configuration bytes.*\/ */
/* SECTIONS */
/* { */
/*   .text.progmem : ALIGN(4) */
/*   { */
/*     . = 0; */
/*     KEEP(*(.flashconfig)) */
/*     FILL(0xFF) */
/*     /\* IVT offset is 0x1000 from the bootable image as per section 2.5 */
/*        of https://www.nxp.com/docs/en/application-note/AN12107.pdf *\/ */
/*     . = ORIGIN(flash0) + 0x1000; */
/*     KEEP(*(.ivt)) */
/*     KEEP(*(.bootdata)) */
/*     /\* .vectors get placed in rules_code.ld *\/ */
/*   } > flash0 */

/*   .cfmprotect : ALIGN(4) SUBALIGN(4) */
/*   { */
/*     KEEP(*(.cfmconfig)) */
/*   } > flash0 */
/* } */

/* /\* For each data/text section two region are defined, a virtual region */
/*    and a load region (_LMA suffix).*\/ */

/* /\* Flash region to be used for exception vectors.*\/ */
/* REGION_ALIAS("VECTORS_FLASH", flash0); */
/* REGION_ALIAS("VECTORS_FLASH_LMA", flash0); */

/* /\* Flash region to be used for constructors and destructors.*\/ */
/* REGION_ALIAS("XTORS_FLASH", flash0); */
/* REGION_ALIAS("XTORS_FLASH_LMA", flash0); */

/* /\* Flash region to be used for code text.*\/ */
/* REGION_ALIAS("TEXT_FLASH", flash0); */
/* REGION_ALIAS("TEXT_FLASH_LMA", flash0); */

/* /\* Flash region to be used for read only data.*\/ */
/* REGION_ALIAS("RODATA_FLASH", flash0); */
/* REGION_ALIAS("RODATA_FLASH_LMA", flash0); */

/* /\* Flash region to be used for various.*\/ */
/* REGION_ALIAS("VARIOUS_FLASH", flash0); */
/* REGION_ALIAS("VARIOUS_FLASH_LMA", flash0); */

/* /\* Flash region to be used for RAM(n) initialization data.*\/ */
/* REGION_ALIAS("RAM_INIT_FLASH_LMA", flash0); */

/* /\* RAM region to be used for Main stack. This stack accommodates the processing */
/*    of all exceptions and interrupts.*\/ */
/* REGION_ALIAS("MAIN_STACK_RAM", ram1); */

/* /\* RAM region to be used for the process stack. This is the stack used by */
/*    the main() function.*\/ */
/* REGION_ALIAS("PROCESS_STACK_RAM", ram1); */

/* /\* RAM region to be used for data segment.*\/ */
/* /\* .data in teensy-arduino-core/imxrt1062.ld *\/ */
/* REGION_ALIAS("DATA_RAM", ram1); */
/* REGION_ALIAS("DATA_RAM_LMA", flash0); */

/* /\* RAM region to be used for BSS segment.*\/ */
/* /\* .bss in imxrt1062.ld *\/ */
/* REGION_ALIAS("BSS_RAM", ram1); */

/* /\* RAM region to be used for the default heap.*\/ */
/* /\* _heap_start in imxrt1062.ld *\/ */
/* REGION_ALIAS("HEAP_RAM", ram2); */

/* /\* Generic rules inclusion.*\/ */
/* INCLUDE rules.ld */

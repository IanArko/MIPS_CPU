/* Linker script for EE180 Lab2 MIPS processor
 *
 * This is a Harvard architecture with separate instruction
 * and data ports.
 *
 * Author: Grant Ayers (ayers@cs.stanford.edu)
 */

/* Memory Section
 *
 * Configuration for 64KB of instruction memory
 * and 16KB of data memory
 *
 * Instruction and data memories start at address 0.
 *
 *   Instructions :    0x00000000 -> 0x0000ffff    ( 64KB)
 *   Data / BSS   :    0x00000000 -> 0x00001fff    (  8KB)
 *   Stack / Heap :    0x00002000 -> 0x00003fff    (  8KB)
 */

SECTIONS
{
    _sp = 0x00004000;

    .text :
	{
        . = 0 ;

        *(.boot)
        *(.*text.startup)
		*(.*text*)

        . = 0x00010000 ;
	}

    .data :
    {
        . = 0 ;

        *(.rodata*)
        *(.data*)
    }

    . = ALIGN(1024) ;
    _gp = . ;

    .sdata :
    {
        *(.*sdata*)
    }

    _bss_start = . ;

    .sbss :
    {
        *(.*sbss)
    }

    .bss :
    {
        *(.*bss)
    }

    _bss_end = . ;

    /*. = 0x2000;*/
}

/*
// TI File $Revision: /main/9 $
// Checkin $Date: August 28, 2007   11:23:31 $
//###########################################################################
//
// FILE:    28335_RAM_lnk.cmd
//
// TITLE:   Linker Command File For 28335 examples that run out of RAM
//
//          This ONLY includes all SARAM blocks on the 28335 device.
//          This does not include flash or OTP. 
//
//          Keep in mind that L0 and L1 are protected by the code
//          security module.
//
//          What this means is in most cases you will want to move to 
//          another memory map file which has more memory defined.  
//
//###########################################################################
// $TI Release: DSP2833x Header Files V1.01 $
// $Release Date: September 26, 2007 $
//###########################################################################
*/

/* ======================================================
// For Code Composer Studio V2.2 and later
// ---------------------------------------
// In addition to this memory linker command file, 
// add the header linker command file directly to the project. 
// The header linker command file is required to link the
// peripheral structures to the proper locations within 
// the memory map.
//
// The header linker files are found in <base>\DSP2833x_Headers\cmd
//   
// For BIOS applications add:      DSP2833x_Headers_BIOS.cmd
// For nonBIOS applications add:   DSP2833x_Headers_nonBIOS.cmd    
========================================================= */

/* ======================================================
// For Code Composer Studio prior to V2.2
// --------------------------------------
// 1) Use one of the following -l statements to include the 
// header linker command file in the project. The header linker
// file is required to link the peripheral structures to the proper 
// locations within the memory map                                    */

/* Uncomment this line to include file only for non-BIOS applications */
/* -l DSP2833x_Headers_nonBIOS.cmd */

/* Uncomment this line to include file only for BIOS applications */
/* -l DSP2833x_Headers_BIOS.cmd */

/* 2) In your project add the path to <base>\DSP2833x_headers\cmd to the
   library search path under project->build options, linker tab, 
   library search path (-i).
/*========================================================= */

/* Define the memory block start/length for the F28335  
   PAGE 0 will be used to organize program sections
   PAGE 1 will be used to organize data sections

   Notes: 
         Memory blocks on F28335 are uniform (ie same
         physical memory) in both PAGE 0 and PAGE 1.  
         That is the same memory region should not be
         defined for both PAGE 0 and PAGE 1.
         Doing so will result in corruption of program 
         and/or data. 
         
         L0/L1/L2 and L3 memory blocks are mirrored - that is
         they can be accessed in high memory or low memory.
         For simplicity only one instance is used in this
         linker file. 
         
         Contiguous SARAM memory blocks can be combined 
         if required to create a larger memory block. 
*/


MEMORY
{
PAGE 0 :    /* Program Memory */
    BEGIN       : origin = 0x000000, length = 0x000002
    BOOT_RSVD   : origin = 0x000002, length = 0x00004E     /* Part of M0, BOOT rom will use this for stack */
    RAMM0       : origin = 0x000050, length = 0x0003B0
    RAML0       : origin = 0x008000, length = 0x001000
    RAML1_L3    : origin = 0x009000, length = 0x003000     /* 合并RAML1(0x9000) + RAML2(0xA000) + RAML3(0xB000) */
    ZONE6A      : origin = 0x100000, length = 0x00FC00     /* XINTF Zone 6A */
    CSM_RSVD    : origin = 0x33FF80, length = 0x000076     /* Part of FLASH, CSM Password Locations */
    CSM_PWL     : origin = 0x33FFF8, length = 0x000008     /* Part of FLASH, CSM Password Locations */
    ADC_CAL     : origin = 0x380080, length = 0x000009     /* ADC Calibration Data in OTP */
    RESET       : origin = 0x3FFFC0, length = 0x000002     /* Reset Vector Location (FLASH) */
    IQTABLES    : origin = 0x3FE000, length = 0x000b50     /* IQ Math Tables in BOOT ROM */
    IQTABLES2   : origin = 0x3FEB50, length = 0x00008c     /* IQ Math Tables in BOOT ROM */
    FPUTABLES   : origin = 0x3FEBDC, length = 0x0006A0     /* FPU Tables in BOOT ROM */
    BOOTROM     : origin = 0x3FF27C, length = 0x000D44     /* Boot ROM Code */

PAGE 1 :    /* Data Memory */
    RAMM1       : origin = 0x000400, length = 0x000400     /* On-Chip RAM M1 */
    RAML4_7     : origin = 0x00C000, length = 0x004000     /* On-Chip RAM L4~L7 (combined) */
    ZONE7B      : origin = 0x20FC00, length = 0x000400     /* XINTF Zone 7B */
}

/* ====================================================== */
/* Section Allocation Specifier                           */
/* ====================================================== */
SECTIONS
{
    /* Program Sections */
    codestart        : > BEGIN,     PAGE = 0
    ramfuncs         : > RAML0,     PAGE = 0
    .text            : > RAML1_L3,  PAGE = 0  /* 代码段指向合并后的内存块 */
    .cinit           : > RAML0,     PAGE = 0
    .pinit           : > RAML0,     PAGE = 0
    .switch          : > RAML0,     PAGE = 0
    .cio             : > RAML0,     PAGE = 0  /* 解决.cio段警告 */

    /* Data Sections */
    .stack           : > RAMM1,     PAGE = 1
    .ebss            : > RAML4_7,   PAGE = 1
    .econst          : > RAML4_7,   PAGE = 1
    .esysmem         : > RAML4_7,   PAGE = 1

    /* IQ Math Sections */
    IQmath           : > RAML1_L3,  PAGE = 0
    IQmathTables     : > IQTABLES,  PAGE = 0, TYPE = NOLOAD
    IQmathTables2    : > IQTABLES2, PAGE = 0, TYPE = NOLOAD
    FPUmathTables    : > FPUTABLES, PAGE = 0, TYPE = NOLOAD

    /* Data Memory Sections */
    DMARAML4         : > RAML4_7,   PAGE = 1
    DMARAML5         : > RAML4_7,   PAGE = 1
    DMARAML6         : > RAML4_7,   PAGE = 1
    DMARAML7         : > RAML4_7,   PAGE = 1
    ZONE7DATA        : > ZONE7B,    PAGE = 1

    /* Misc Sections */
    .reset           : > RESET,     PAGE = 0, TYPE = DSECT
    csm_rsvd         : > CSM_RSVD,  PAGE = 0, TYPE = DSECT
    csmpasswds       : > CSM_PWL,   PAGE = 0, TYPE = DSECT
    .adc_cal         : > ADC_CAL,   PAGE = 0, TYPE = NOLOAD
}

/*
//===========================================================================
// End of file.
//===========================================================================
*/

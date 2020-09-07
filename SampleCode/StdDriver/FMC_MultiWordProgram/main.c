/****************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 16/10/17 2:06p $
 * @brief    Show how to multiple word program embedded flash by ISP function.
 * @note     Multiple word program flash function code need to locate on SRAM.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NuMicro.h"

extern int32_t multi_word_program(void);

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))
                    |(SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Unlock protected registers to operate FMC ISP function */
    SYS_UnlockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Checking if target device supports the feature */
    if( (GET_CHIP_SERIES_NUM == CHIP_SERIES_NUM_I) || (GET_CHIP_SERIES_NUM == CHIP_SERIES_NUM_G) )
    {
        /* Checking if flash size matches with target device */
		if(FMC_FLASH_PAGE_SIZE != 2048)
        {
            /* FMC_FLASH_PAGE_SIZE is different from target device's */
            printf("Please enable the compiler option - PAGE_SIZE_2048 in fmc.h\n");
            while(SYS->PDID);
        }
    }
    else
    {
        printf("The target device didn't support the feature\n");
        while(SYS->PDID);
    }
	
    printf("\n\n");
    printf("+-------------------------------------+\n");
    printf("|   FMC_MultiWordProgram Sample Code  |\n");
    printf("+-------------------------------------+\n");

    /* Enable FMC ISP function */
    FMC_Open();

    /* Enable APROM erase/program */
    FMC_ENABLE_AP_UPDATE();

#if defined (__ICCARM__)
#pragma section = "fastcode"
#pragma section = "fastcode_init"

    printf("Source Addr = 0x%x\n", (uint32_t)__section_begin("fastcode_init"));
    printf("DST Addr = 0x%x\n", (uint32_t)__section_begin("fastcode"));
    memcpy((void *) __section_begin("fastcode"), __section_begin("fastcode_init"), (unsigned long) __section_size("fastcode"));

#elif defined (__GNUC__)

#endif

    if( multi_word_program() >= 0)
        printf("\n\nMulti-word program demo done.\n");
    else
        printf("\n\nMulti-word program demo fail.\n");
    while(1);

}
/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/


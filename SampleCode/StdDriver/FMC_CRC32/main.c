/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Demonstrate how to use FMC CRC32 ISP command to calculate the CRC32 checksum of APROM and LDROM.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define FMC_MIN_LDROM_SIZE  0x800

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set PLL to Power-down mode and PLLSTB bit in CLK_STATUS register will be cleared by hardware.*/
    CLK_DisablePLL();

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))
                    |(SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);
}



int32_t main(void)
{
    uint32_t    u32Data, u32ChkSum;    /* temporary data */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

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
        if(FMC_FLASH_PAGE_SIZE != 512)
        {
            /* FMC_FLASH_PAGE_SIZE is different from target device's */
            printf("Please disable the compiler option - PAGE_SIZE_2048 in fmc.h\n");
            while(SYS->PDID);
        }
    }

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/
    printf("\n");
    printf("+------------------------------------+\n");
    printf("|   M031 FMC CRC32 Sample Demo       |\n");
    printf("+------------------------------------+\n");

    /* Unlock protected registers to operate FMC ISP function */
    SYS_UnlockReg();                   

    /* Enable FMC ISP Function */
    FMC_Open();

    /* FMC_ReadCID */
    u32Data = FMC_ReadCID();
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_ReadCID failed!\n");
        goto lexit;
    }
    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    /* FMC_ReadPID */
    u32Data = FMC_ReadPID();
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_ReadPID failed!\n");
        goto lexit;
    }
    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    /* Read User Configuration CONFIG0 */
    printf("  User Config 0 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE));
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_Read(FMC_CONFIG_BASE) failed!\n");
        goto lexit;
    }
    /* Read User Configuration CONFIG1 */
    printf("  User Config 1 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE + 4));
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_Read(FMC_CONFIG_BASE+4) failed!\n");
        goto lexit;
    }

    printf("\nLDROM (0x100000 ~ 0x%x) CRC32 checksum =>  ", (uint32_t)(FMC_LDROM_BASE + FMC_MIN_LDROM_SIZE));

    /* Erase the first page of LDROM */
    FMC_ENABLE_LD_UPDATE();

    FMC_Erase(0x100000);
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_Erase failed!\n");
        goto lexit;
    }
    /* Write one word on LD */
    FMC_Write(0x100000, 0x55AABBCC);
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_Write failed!\n");
        goto lexit;
    }
    /*
     *  Request FMC hardware to run CRC32 calculation on LDROM.
     */
    u32ChkSum = FMC_GetChkSum(FMC_LDROM_BASE, FMC_MIN_LDROM_SIZE);
    if (u32ChkSum == 0xFFFFFFFF)
    {
        printf("Failed on calculating LDROM CRC32 checksum!\n");
        goto lexit;                    /* failed */
    }
    printf("0x%x\n", u32ChkSum);       /* print out LDROM CRC32 check sum value */

    printf("\nSPROM (0x200000 ~ 0x%x) CRC32 checksum =>  ", (0x200000+FMC_SPROM_SIZE));
    /*
    *  Request FMC hardware to run CRC32 calculation on SPROM.
    */
    u32ChkSum = FMC_GetChkSum(FMC_SPROM_BASE, FMC_SPROM_SIZE);
    if (u32ChkSum == 0xFFFFFFFF)
    {
        printf("Failed on calculating SPROM CRC32 checksum!\n");
        goto lexit;
    }
    printf("0x%x\n", u32ChkSum);       /* print out SPROM CRC32 check sum value */


    printf("\nAPROM address (0x0 ~ 0x4000) CRC32 checksum =>  ");
    /*
     *  Request FMC hardware to run CRC32 calculation on APROM address 0 ~ 0x4000.
     *  Note that FMC CRC32 checksum calculation area must page alignment.
     */
    u32ChkSum = FMC_GetChkSum(FMC_APROM_BASE, 0x4000);
    if (u32ChkSum == 0xFFFFFFFF)
    {
        printf("Failed on calculating APROM bank0 CRC32 checksum!\n");
        goto lexit;
    }
    printf("0x%x\n", u32ChkSum);       /* print out APROM CRC32 check sum value */

    printf("\nAPROM (0x4000 ~ 0x8000) CRC32 checksum =>  ");
    /*
     *  Request FMC hardware to run CRC32 calculation on APROM address 0x4000 ~ 0x8000.
     *  Note that FMC CRC32 checksum calculation area must page alignment.
     */
    u32ChkSum = FMC_GetChkSum(FMC_APROM_BASE + 0x4000, 0x4000);
    if (u32ChkSum == 0xFFFFFFFF)
    {
        printf("Failed on calculating APROM CRC32 checksum!\n");
        goto lexit;
    }
    printf("0x%x\n", u32ChkSum);       /* print out APROM CRC32 check sum value */

    printf("\nFMC CRC32 checksum test done.\n");
lexit:
    FMC_Close();                       /* Disable FMC ISP function */
    SYS_LockReg();                     /* Lock protected registers */

    while (1);
}
/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/



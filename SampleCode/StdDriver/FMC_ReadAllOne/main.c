/****************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to use FMC Read-All-One ISP command to verify APROM/LDROM pages are all 0xFFFFFFFF or not.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
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
    uint32_t u32Data, u32ret;

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
    printf("+------------------------------------------+\n");
    printf("|        FMC_ReadAllOne Sample Code        |\n");
    printf("+------------------------------------------+\n");

    /* Enable FMC ISP function */
    FMC_Open();

    /* Read company ID. Should be 0xDA. */
    u32Data = FMC_ReadCID();
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_ReadCID failed!\n");
        goto lexit;
    }
    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    /* Read product ID. */
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

    /* Enable LDROM update. */
    FMC_ENABLE_LD_UPDATE();

    /* Erase LDROM page 0. */
    if (FMC_Erase(FMC_LDROM_BASE) != 0)
    {
        printf("FMC_Erase(FMC_LDROM_BASE) failed!\n");
        goto lexit;
    }

    /* Run and check flash contents are all 0xFFFFFFFF. */
    u32ret = FMC_CheckAllOne(FMC_LDROM_BASE, FMC_FLASH_PAGE_SIZE);
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_CheckAllOne failed!\n");
        goto lexit;
    }

    if (u32ret == READ_ALLONE_YES)               /* return value READ_ALLONE_YES means all flash contents are 0xFFFFFFFF */
        printf("READ_ALLONE_YES success.\n");    /* FMC_CheckAllOne() READ_ALLONE_YES passed on LDROM page 0. */
    else
        printf("READ_ALLONE_YES failed!\n");     /* FMC_CheckAllOne() READ_ALLONE_YES failed on LDROM page 0. */

    /* Program a 0 to LDROM to make it not all 0xFFFFFFFF. */
    if (FMC_Write(FMC_LDROM_BASE, 0) != 0)
    {
        printf("FMC_Write FMC_LDROM_BASE failed!\n");
        goto lexit;
    }

    /* Run and check flash contents are not all 0xFFFFFFFF. */
    u32ret = FMC_CheckAllOne(FMC_LDROM_BASE, FMC_FLASH_PAGE_SIZE);
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_CheckAllOne failed!\n");
        goto lexit;
    }

    if (u32ret == READ_ALLONE_NOT)
        printf("READ_ALLONE_NOT success.\n");   /* FMC_CheckAllOne() READ_ALLONE_NOT passed on LDROM page 0. */
    else
        printf("READ_ALLONE_NOT failed!\n");    /* FMC_CheckAllOne() READ_ALLONE_NOT failed on LDROM page 0. */

    printf("\nFMC Read-All-One test done.\n");

lexit:
    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    while (1);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/

/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Demonstrate how to use FMC Bank Remap ISP command to remap different bank to address 0
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

static int SetIAPBoot(void)
{
    uint32_t  au32Config[2];
    uint32_t u32CBS;

    FMC_ENABLE_CFG_UPDATE();
    /* Read current boot mode */
    u32CBS = (FMC->ISPSTS & FMC_ISPSTS_CBS_Msk) >> FMC_ISPSTS_CBS_Pos;

    if (u32CBS & 1)
    {
        /* Modify User Configuration when it is not in IAP mode */

        if (FMC_ReadConfig(au32Config, 2) < 0)
        {
            printf("\nRead User Config failed!\n");
            return -1;
        }
        if (au32Config[0] & 0x40)
        {
            FMC_ENABLE_CFG_UPDATE();
            au32Config[0] &= ~0x40;

            FMC_Erase(FMC_CONFIG_BASE);
            if (g_FMC_i32ErrCode != 0)
            {
                printf("FMC_Erase failed!\n");
                return -1;
            }
            if (FMC_WriteConfig(au32Config, 2) != 0) /* Update User Configuration CONFIG0 and CONFIG1. */
            {
                printf("FMC_WriteConfig failed!\n");
                return -1;
            }
            // Perform chip reset to make new User Config take effect
            SYS_ResetChip();
        }
    }

    return 0;
}


int32_t main(void)
{
    int ch;
    uint32_t u32Config[3];

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Unlock protected registers to operate FMC ISP function */
    SYS_UnlockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Judge the chip version if support remap bank function. */
    if( (GET_CHIP_SERIES_NUM != CHIP_SERIES_NUM_I) && (GET_CHIP_SERIES_NUM != CHIP_SERIES_NUM_G) )
    {
        printf("The chip version didn't support the remap bank function\n");
        goto lexit;
    }

    /* Enable FMC ISP function */
    FMC_Open();
    SetIAPBoot();
    if (FMC_ReadConfig(u32Config, 3) < 0)
    {
        printf("\nRead User Config failed!\n");
        while (1);
    }
    printf("Config0 = 0x%x\n", u32Config[0]);
    printf("Boot from Loader0\n");

    do
    {
        printf("+-------------------------------+\n");
        printf("|   Bank Remapping Sample Code  |\n");
        printf("+-------------------------------+\n");
        printf("| [ 0 ]: Switch to bank0        |\n");
        printf("| [ 1 ]: Switch to bank1        |\n");
        printf("| [ Other ]: Exit               |\n");
        printf("| Current Bank %d               |\n", (FMC->ISPSTS&BIT30)?1:0);
        printf("+-------------------------------+\n");
        ch = getchar();
        if(ch=='0')
        {
            printf("Bank0 App0 remap to address 0\n");
            while(UART_IS_TX_EMPTY(UART0)==0);

            /* All ISP commands are relative to physical address except FMC_ISPCMD_VECMAP */
            FMC_SetVectorPageAddr(0x4000);
            if (g_FMC_i32ErrCode != 0)
            {
                printf("FMC_SetVectorPageAddr failed!\n");
                return -1;
            }
            /* After bank swap command, Bank0 virtual address 0x4000 will remap to 0 */
            FMC_RemapBank(0);
            if (g_FMC_i32ErrCode != 0)
            {
                printf("FMC_RemapBank failed!\n");
                return -1;
            }
            NVIC_SystemReset();
        }
        else if(ch=='1')
        {
            printf("Bank1 App1 remap to address 0\n");
            while(UART_IS_TX_EMPTY(UART0)==0);

            /* All ISP commands are relative to physical address except FMC_ISPCMD_VECMAP */
            FMC_SetVectorPageAddr(0x4000);
            if (g_FMC_i32ErrCode != 0)
            {
                printf("FMC_SetVectorPageAddr failed!\n");
                return -1;
            }
            /* After bank swap command, Bank1 virtual address 0x4000 will remap to 0 */
            FMC_RemapBank(1);
            if (g_FMC_i32ErrCode != 0)
            {
                printf("FMC_RemapBank failed!\n");
                return -1;
            }
            NVIC_SystemReset();
        }
        else
            break;
    }
    while(1);

lexit:
    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    while (1);
}
/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/



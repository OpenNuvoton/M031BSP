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

int32_t  VerifyData(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t    u32Addr;
    uint32_t    u32Data;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        u32Data = FMC_Read(u32Addr);

        if (g_FMC_i32ErrCode != 0)
        {
            printf("FMC_Read address 0x%x failed!\n", u32Addr);
            return -1;
        }

        if (u32Data != u32Pattern)
        {
            printf("\nFMC_Read data verify failed at address 0x%x, read=0x%x, expect=0x%x\n", u32Addr, u32Data, u32Pattern);
            return -1;
        }
    }

    return 0;
}


int32_t FillAddrPattern(uint32_t u32StartAddr, uint32_t u32EndAddr)
{
    uint32_t u32Addr;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        FMC_Write(u32Addr, u32Addr);

        if (g_FMC_i32ErrCode != 0)
        {
            printf("FMC_Write address 0x%x failed!\n", u32Addr);
            return -1;
        }
    }
    return 0;
}

int32_t  VerifyAddrPattern(uint32_t u32StartAddr, uint32_t u32EndAddr)
{
    uint32_t    u32Addr;
    uint32_t    u32Data;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        u32Data = FMC_Read(u32Addr);

        if (g_FMC_i32ErrCode != 0)
        {
            printf("FMC_Read address 0x%x failed!\n", u32Addr);
            return -1;
        }

        if (u32Data != u32Addr)
        {
            printf("\nFMC_Read data verify failed at address 0x%x, read=0x%x, expect=0x%x\n", u32Addr, u32Data, u32Addr);
            return -1;
        }
    }

    return 0;
}


int32_t  ProgramFlash(uint32_t u32StartAddr, uint32_t u32EndAddr)
{
    uint32_t    u32Addr;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += FMC_FLASH_PAGE_SIZE)
    {
        printf("    Flash test address: 0x%x    \r", u32Addr);

        // Erase page
        FMC_Erase(u32Addr);

        if (g_FMC_i32ErrCode != 0)
        {
            printf("FMC_Erase address 0x%x failed!\n", u32Addr);
            return -1;
        }
        // Verify if page contents are all 0xFFFFFFFF
        if (VerifyData(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, 0xFFFFFFFF) < 0)
        {
            printf("\nPage 0x%x erase verify failed!\n", u32Addr);
            return -1;
        }

        // Write test pattern to fill the whole page
        if (FillAddrPattern(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE) < 0)
        {
            printf("Failed to write page 0x%x!\n", u32Addr);
            return -1;
        }

        // Verify if page contents are all equal to test pattern
        if (VerifyAddrPattern(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE) < 0)
        {
            printf("\nData verify failed!\n ");
            return -1;
        }
    }

    printf("\r    Flash Test Passed.          \n");
    return 0;
}

void cpu_dump(uint32_t start_addr, uint32_t end_addr)
{
    uint32_t  u32Addr, u32Data;

    printf("\n[CPU DUMP: 0x%x ~ 0x%x]\n", start_addr, end_addr);
    for (u32Addr = start_addr; u32Addr < end_addr; u32Addr += 4)
    {
        u32Data = inp32(u32Addr);
        if (u32Addr % 32 == 0)
            printf("\n");
        printf("0x%08x ", u32Data);
    }
    printf("\n\n");
}

int32_t main(void)
{
    int ch;
    uint32_t u32Bank0ProgAddr = 0x10000, u32Bank1ProgAddr = 0x50000;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Unlock protected registers to operate FMC ISP function */
    SYS_UnlockReg();

    /* Configure UART1: 115200, 8-bit word, no parity bit, 1 stop bit. ==> For SerialComm */
    UART_Open(UART0, 115200);

    printf("\n\n\nApplication 0 on bank 0\n");
    while(UART_IS_TX_EMPTY(UART0)==0);
    /* Enable FMC ISP function */
    FMC_Open();

    do
    {
        printf("+------------------------------------------------+\n");
        printf("|   Bank Remapping Sample Code                   |\n");
        printf("+------------------------------------------------+\n");
        printf("| [ 0 ]: Program a page to bank0                 |\n");
        printf("| [ 1 ]: Program a page to bank1                 |\n");
        printf("| [ 2 ]: CPU dumps the programmed page in bank0  |\n");
        printf("| [ 3 ]: CPU dumps the programmed page in bank1  |\n");
        printf("| [ 4 ]: Switch to Loader0                       |\n");
        printf("| [ q/Q ]: Quit                                  |\n");
        printf("| Current Bank %d                                |\n", (FMC->ISPSTS&BIT30)?1:0);
        printf("+------------------------------------------------+\n");
        ch = getchar();
        switch(ch)
        {
        case '0':
            FMC_ENABLE_AP_UPDATE();
            ProgramFlash(u32Bank0ProgAddr, u32Bank0ProgAddr+FMC_FLASH_PAGE_SIZE);
            break;
        case '1':
            FMC_ENABLE_AP_UPDATE();
            ProgramFlash(u32Bank1ProgAddr, u32Bank1ProgAddr+FMC_FLASH_PAGE_SIZE);
            break;
        case '2':
            cpu_dump(u32Bank0ProgAddr, u32Bank0ProgAddr+FMC_FLASH_PAGE_SIZE);
            break;
        case '3':
            cpu_dump(u32Bank1ProgAddr, u32Bank1ProgAddr+FMC_FLASH_PAGE_SIZE);
            break;
        case '4':
            printf("Switch to Loader 0\n");
            UART_WAIT_TX_EMPTY(UART0);
            while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }

            FMC_DISABLE_ISP_INT();
            NVIC_DisableIRQ(ISP_IRQn);

            FMC_SetVectorPageAddr(0x0);     /* Switch To bank0 Loader */
            if (g_FMC_i32ErrCode != 0)
            {
                printf("FMC_SetVectorPageAddr failed!\n");
                return -1;
            }
            NVIC_SystemReset();
            break;
        }
        if((ch == 'q') || (ch == 'Q'))
            break;
    }
    while(1);

    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    while (1);
}
/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/



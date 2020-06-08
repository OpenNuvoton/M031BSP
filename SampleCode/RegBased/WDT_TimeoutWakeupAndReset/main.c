/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 7 $
 * $Date: 18/07/10 11:22a $
 * @brief
 *           Demonstrate how to cause WDT time-out reset system event while WDT time-out reset delay period expired.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
extern int IsDebugFifoEmpty(void);
volatile uint32_t g_u32WDTINTCounts;
volatile uint8_t g_u8IsWDTWakeupINT;

/**
 * @brief       IRQ Handler for WDT and WWDT Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The WDT_IRQHandler is default IRQ of WDT and WWDT, declared in startup_NUC123.s.
 */
void WDT_IRQHandler(void)
{
    if(g_u32WDTINTCounts < 10)
        WDT->RSTCNT = WDT_RESET_COUNTER_KEYWORD;

    if(WDT->CTL & WDT_CTL_IF_Msk)
    {
        /* Clear WDT time-out interrupt flag */
        WDT->CTL = (WDT->CTL & ~(WDT_CTL_RSTF_Msk | WDT_CTL_WKF_Msk)) | WDT_CTL_IF_Msk;

        g_u32WDTINTCounts++;
    }

    if(WDT->CTL & WDT_CTL_WKF_Msk)
    {
        /* Clear WDT time-out wake-up flag */
        WDT->CTL = (WDT->CTL & ~(WDT_CTL_RSTF_Msk | WDT_CTL_IF_Msk)) | WDT_CTL_WKF_Msk;

        g_u8IsWDTWakeupINT = 1;
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) != CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk ) | CLK_CLKSEL0_HCLKSEL_HIRC ;

    /* Enable UART0 clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk ;

    /* Switch UART0 clock source to HIRC */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART0SEL_Msk) | CLK_CLKSEL1_UART0SEL_HIRC;

    /* Switch WDT clock source to LIRC */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_WDTSEL_Msk) | CLK_CLKSEL1_WDTSEL_LIRC;

    /* Enable WDT clock */
    CLK->APBCLK0 |= CLK_APBCLK0_WDTCKEN_Msk ;

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))
                    |(SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Lock protected registers */
    SYS_LockReg();

}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 to 115200-8n1 for print message */
    /* Reset UART0 */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+----------------------------------------+\n");
    printf("|    WDT Time-out Wake-up Sample Code    |\n");
    printf("+----------------------------------------+\n\n");
    /* To check if system has been reset by WDT time-out reset or not */
    if(WDT->CTL & WDT_CTL_RSTF_Msk)
    {
        WDT->CTL = (WDT->CTL & ~(WDT_CTL_IF_Msk | WDT_CTL_WKF_Msk)) | WDT_CTL_RSTF_Msk;
        printf("*** System has been reset by WDT time-out event ***\n\n");
        while(1);
    }

    printf("# WDT Settings:\n");
    printf("    - Clock source is LIRC                  \n");
    printf("    - Time-out interval is 2^14 * WDT clock \n");
    printf("      (around 0.4266 ~ 0.4270 s)            \n");
    printf("    - Interrupt enable                      \n");
    printf("    - Wake-up function enable               \n");
    printf("    - Reset function enable                 \n");
    printf("    - Reset delay period is 18 * WDT clock  \n");
    printf("# System will generate a WDT time-out interrupt event after 0.4266 ~ 0.4270 s, \n");
    printf("    and will be wake up from power-down mode also.\n");
    printf("    (Use PA.0 high/low period time to check WDT time-out interval)\n");
    printf("# When WDT interrupt counts large than 10, \n");
    printf("    we will not reset WDT counter value and system will be reset immediately by WDT time-out reset signal.\n");

    /* Use PA.0 to check WWDT compare match interrupt period time */
    PA->MODE = 0xFFFFFFFD;
    PA0 = 1;

    /* Because of all bits can be written in WDT Control Register are write-protected;
       To program it needs to disable register protection first. */
    SYS_UnlockReg();

    /* Configure WDT settings and start WDT counting */
    g_u32WDTINTCounts = g_u8IsWDTWakeupINT = 0;

    WDT->ALTCTL = WDT_RESET_DELAY_18CLK;

    WDT->CTL = WDT_TIMEOUT_2POW14 | WDT_CTL_WDTEN_Msk |
               (1 << WDT_CTL_RSTEN_Pos) |
               (1 << WDT_CTL_WKEN_Pos);

    /* Enable WDT NVIC */
    NVIC_EnableIRQ(WDT_IRQn);

    /* Enable WDT interrupt function */
    WDT->CTL |= WDT_CTL_INTEN_Msk;
    while(WDT->CTL & WDT_CTL_SYNC_Msk); // Wait enable WDTEN bit completed, it needs 2 * WDT_CLK.

    while(1)
    {
        /* System enter to Power-down */
        /* To program PWRCTL register, it needs to disable register protection first. */
        SYS_UnlockReg();
        printf("\nSystem enter to power-down mode ...\n");
        /* To check if all the debug messages are finished */
        while(((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) != 0) == 0);

        /* Set the processor uses deep sleep as its low power mode */
        SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

        /* Set system Power-down enabled*/
        CLK->PWRCTL |= CLK_PWRCTL_PDEN_Msk;

        /* Chip enter Power-down mode after CPU run WFI instruction */
        __WFI();

        /* Check if WDT time-out interrupt and wake-up occurred or not */
        while(g_u8IsWDTWakeupINT == 0);

        g_u8IsWDTWakeupINT = 0;
        PA0 ^= 1;

        printf("System has been waken up done. WDT interrupt counts: %d.\n\n", g_u32WDTINTCounts);
    }
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

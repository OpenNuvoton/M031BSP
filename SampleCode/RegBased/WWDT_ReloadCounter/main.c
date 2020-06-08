/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 20 $
 * $Date: 18/07/23 9:33a $
 * @brief    Demonstrate how to reload the WWDT counter value without resetting the MCU.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

#define RELOAD_CONDITION  3
/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
extern int IsDebugFifoEmpty(void);

void UserAlgorithm(void)
{
    printf("-> Reload WWDT counter (%d) in UserAlgorithm()\n",WWDT->CNT);

    while(!IsDebugFifoEmpty());

    /* To reload the WWDT counter value to 0x3F */
    WWDT->RLDCNT = WWDT_RELOAD_WORD;
    /* Algorithm */
    PA0 ^= 1;
}
void Delay_ms(uint32_t ms)
{
    uint32_t i;
    uint32_t cnt = 0;

    printf("\nDelay %d ms before UserAlgorithm()\n",ms);
    printf("WWDT Counter\n");

    for(i=0 ; i<ms ; i++)
    {
        CLK_SysTickDelay(1000);

        if(WWDT->CNT != cnt)
        {
            cnt = WWDT->CNT;
            printf("%2d ",WWDT->CNT);
        }
    }
}


volatile uint8_t g_u32WWDTINTCounts;

/**
 * @brief       IRQ Handler for WWDT Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The WDT_IRQHandler is default IRQ of WWDT
 */
void WDT_IRQHandler(void)
{
    if(WWDT->STATUS & WWDT_STATUS_WWDTIF_Msk)
    {
        /* Clear WWDT compare match interrupt flag */
        WWDT->STATUS = WWDT_STATUS_WWDTIF_Msk;

        g_u32WWDTINTCounts++;

        printf("\nWWDT compare match interrupt occurred. (Interrupt Count %d)\n", g_u32WWDTINTCounts);
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48 MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Wait for HIRC clock ready */
    while((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) != CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk ) | CLK_CLKSEL0_HCLKSEL_HIRC ;

    /* Enable UART0 clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk ;

    /* Enable WWDT clock */
    CLK->APBCLK0 |= CLK_APBCLK0_WDTCKEN_Msk ;

    /* Enable peripheral clock */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART0SEL_Msk) | CLK_CLKSEL1_UART0SEL_HIRC;

    /* Switch WWDT clock source to HCLK/2048 */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_WWDTSEL_Msk) | CLK_CLKSEL1_WWDTSEL_HCLK_DIV2048;

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}


/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    double dTimeOutPeriodTime;
    double dCompareMatchPeriodTime;

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
    printf("+------------------------------------------------+\n");
    printf("|         WWDT Reload Counter Sample Code        |\n");
    printf("+------------------------------------------------+\n\n");

    printf(" Test Condition :\n");
    printf(" Condition 1 - Reload before the WWDT window region \n");
    printf(" Condition 2 - Reload after the WWDT window region  \n");
    printf(" Condition 3 - Reload inside the WWDT window region \n\n");

    /* To check if system has been reset by WWDT time-out reset or not */
    if(WWDT->STATUS & WWDT_STATUS_WWDTRF_Msk)
    {
        printf("*** System has been reset by WWDT time-out reset event. [WWDT_CTL: 0x%08X] ***\n\n", WWDT->CTL);
        WWDT->STATUS = WWDT_STATUS_WWDTRF_Msk;
        while(1);
    }

    /* WWDT clock source is HCLK / 2048 */
    dTimeOutPeriodTime      = (((double)(1000000 * 2048) / (double)SystemCoreClock) * 1024) * 64 / 1000;
    dCompareMatchPeriodTime = (((double)(1000000 * 2048) / (double)SystemCoreClock) * 1024) * 32 / 1000;

    printf("# WWDT Settings: \n");
    printf("    - Clock source is HCLK/2048 (%d Hz)    \n", SystemCoreClock / 2048);
    printf("    - WWDT counter prescale period is 1024  \n");
    printf("    - Interrupt enable                      \n");
    printf("    - Window Compare value is 32            \n");
    printf("      WWDT time-out period is 1024 * (64 * WWDT_CLK)      = %.2f ms\n", dTimeOutPeriodTime);
    printf("      WWDT compare match period is 1024 * (32 * WWDT_CLK) = %.2f ms\n\n", dCompareMatchPeriodTime);

    printf("# System will generate first WWDT compare match interrupt event after %.2f ms.\n", dCompareMatchPeriodTime);
    printf("    1.) use PA.0 high/low period to check reload WWDT counter period time\n");
    printf("    2.) reload WWDT counter value to avoid WWDT time-out reset\n");
    printf("        when interrupt counter value is less than 11.\n");
    printf("    3.) do not reload WWDT counter value to generate WWDT time-out reset system event\n");
    printf("        when interrupt counts large than 10.\n\n");

    /* Use PA.0 to check reload WWDT counter period time */
    PA->MODE = 0xFFFFFFFD;
    PA0 = 1;

    /* Enable WWDT NVIC */
    NVIC_EnableIRQ(WDT_IRQn);

    g_u32WWDTINTCounts = 0;

    printf("[WWDT_CTL: 0x%08X]\n\n", WWDT->CTL);
    printf("WWDT Counter 63                       32                        0\n");
    printf("              +------------------------+------------------------+\n");
    printf("Timeline (ms) 0                     %.2f                  %.2f\n", dCompareMatchPeriodTime, dTimeOutPeriodTime);
    printf("              |   Reset When Reload    |   WWDT Window Region   |  WWDT Time-Out Reset\n\n");

#if RELOAD_CONDITION == 1
    printf("Test Condition 1 - Reload before the WWDT window region \n");
#elif RELOAD_CONDITION == 2
    printf("Test Condition 2 - Reload after the WWDT window region  \n");
#elif RELOAD_CONDITION == 3
    printf("Test Condition 3 - Reload inside the WWDT window region \n");
#endif

    printf("\n* Reload WWDT counter in UserAlgorithm()\n");

    /*
        WWDT max time-out period is 1024*(64*WWDT_CLK) = 2796.2 ms
        WWDT compare value is 32 = 1024*(32*WWDT_CLK) = 1398.1 ms
        Enable WWDT compare match interrupt
    */
    /* Note: WWDT_CTL register can be written only once after chip is powered on or reset */
    WWDT->CTL = (WWDT_PRESCALER_1024 |(32 << WWDT_CTL_CMPDAT_Pos) | WWDT_CTL_INTEN_Msk | WWDT_CTL_WWDTEN_Msk);

    while(1)
    {
#if RELOAD_CONDITION == 1
        /* Reload before the WWDT window region */
        /* CNTDAT > CMPDAT, Write RLDCNT 0x5AA5 to reload WWWDT counter will reset system */
        /* Delay_ms < 1398.1 ms */
        Delay_ms((uint32_t) dCompareMatchPeriodTime / 2);
#elif RELOAD_CONDITION == 2
        /* Reload after the WWDT window region */
        /* Wait CNTDAT count to 0, System reset immediately */
        /* Delay_ms > 2796.2 ms */
        Delay_ms((uint32_t) dTimeOutPeriodTime + 1);
#elif RELOAD_CONDITION == 3
        if(g_u32WWDTINTCounts > 10)
        {
            /* Reload after the WWDT window region */
            /* Wait CNTDAT count to 0, System reset immediately */
            Delay_ms((uint32_t) dTimeOutPeriodTime + 1);
        }
        else
        {
            /* Reload inside the WWDT window region */
            /* CNTDAT <= CMPDAT, Write RLDCNT 0x5AA5 will reload CNTDAT to 0x3F */
            /* 2796.2 ms > Delay_ms > 1398.1 ms */
            Delay_ms((uint32_t) dCompareMatchPeriodTime + 1);
        }
#endif

        UserAlgorithm();
    }
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

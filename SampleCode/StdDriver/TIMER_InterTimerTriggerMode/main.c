/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use the timer TM0 pin to demonstrate inter timer trigger mode
 *           function. Also display the measured input frequency to UART console.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


int volatile complete = 0;

/* Timer 0 is working in event count mode, and Timer 1 in capture mode, so we read the */
/* capture value from Timer 1, _not_ timer 0. */
void TMR1_IRQHandler(void)
{
    /* TIMER1 clock source = PCLK0 = HCLK / 2 = HIRC / 2 */
    /* Timer clock is 24 MHz, counter value records the duration for 100 event counts. */
    printf("Event frequency is %d Hz\n", (int32_t)(__HIRC/2) / TIMER_GetCounter(TIMER1) * 100);
    TIMER_ClearCaptureIntFlag(TIMER1);
    complete = 1;
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable IP clock */
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_PCLK0, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set timer event counting pin */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB5MFP_Msk)) |
                    (SYS_GPB_MFPL_PB5MFP_TM0);

    /* Lock protected registers */
    SYS_LockReg();
}

/*----------------------------------------------------------------------*/
/* Init UART0                                                           */
/*----------------------------------------------------------------------*/
void UART0_Init(void)
{
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

int main(void)
{
    int volatile i;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* This sample code demonstrate inter timer trigger mode using Timer0 and Timer1
     * In this mode, Timer0 is working as counter, and triggers Timer1. Using Timer1
     * to calculate the amount of time used by Timer0 to count specified amount of events.
     * By dividing the time period recorded in Timer1 by the event counts, we get
     * the event frequency.
     */
    printf("Inter timer trigger mode demo code\n");
    printf("Please connect input source with Timer 0 counter pin PB.5, press any key to continue\n");
    getchar();

    /* Give a dummy target frequency here. Will over write prescale and compare value with macro */
    TIMER_Open(TIMER0, TIMER_ONESHOT_MODE, 100);

    /* Update prescale and compare value. Calculate average frequency every 100 events */
    TIMER_SET_PRESCALE_VALUE(TIMER0, 0);
    TIMER_SET_CMP_VALUE(TIMER0, 100);

    /* Update Timer 1 prescale value. So Timer 0 clock is 24MHz */
    TIMER_SET_PRESCALE_VALUE(TIMER1, 0);

    /* We need capture interrupt */
    NVIC_EnableIRQ(TMR1_IRQn);

    while(1)
    {
        complete = 0;
        /* Count event by timer 0, disable drop count (set to 0), disable timeout (set to 0). Enable interrupt after complete */
        TIMER_EnableFreqCounter(TIMER0, 0, 0, TRUE);
        while(complete == 0);
    }
}

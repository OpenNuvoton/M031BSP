/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use the timer TM0_EXT pin to demonstrate timer free counting mode
 *           function. And displays the measured input frequency to
 *           UART console.
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *
 ****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


void TMR0_IRQHandler(void)
{
    static int cnt = 0;
    static uint32_t t0, t1;

    TIMER_ClearCaptureIntFlag(TIMER0);

    if(cnt == 0)
    {
        t0 = TIMER_GetCaptureData(TIMER0);
        cnt++;
    }
    else if(cnt == 1)
    {
        t1 = TIMER_GetCaptureData(TIMER0);
        cnt++;
        if(t0 >= t1)
        {
            // over run, drop this data and do nothing
        }
        else
        {
            printf("Input frequency is %dHz\n", 24000000 / (t1 - t0));
        }
    }
    else
    {
        cnt = 0;
    }
}


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~32 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 32MHz clock ready */
    while((CLK->STATUS & CLK_STATUS_HXTSTB_Msk) != CLK_STATUS_HXTSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk ) | CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Switch UART0 clock source to XTAL */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set Timer 0 capture pin */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB15MFP_Msk);
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB15MFP_TM0_EXT;

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

    printf("\nThis sample code demonstrate timer free counting mode.\n");
    printf("Please connect input source with Timer 0 capture pin PB.15, press any key to continue\n");
    getchar();

    // Give a dummy target frequency here. Will over write capture resolution with macro
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000000);

    // Update prescale to set proper resolution.
    TIMER_SET_PRESCALE_VALUE(TIMER0, 0);

    // Set compare value as large as possible, so don't need to worry about counter overrun too frequently.
    TIMER_SET_CMP_VALUE(TIMER0, 0xFFFFFF);

    // Configure Timer 0 free counting mode, capture TDR value on rising edge
    TIMER_EnableCapture(TIMER0, TIMER_CAPTURE_FREE_COUNTING_MODE, TIMER_CAPTURE_RISING_EDGE);

    // Start Timer 0
    TIMER_Start(TIMER0);

    // Enable timer interrupt
    TIMER_EnableCaptureInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);

    while(1);
}

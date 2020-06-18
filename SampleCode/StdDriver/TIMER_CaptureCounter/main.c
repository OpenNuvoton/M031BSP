/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to use the Timer capture function to capture Timer counter value.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


volatile uint32_t g_au32TMR1INTCount = 0;

void TMR1_IRQHandler(void)
{
    if(TIMER_GetCaptureIntFlag(TIMER1) == 1)
    {
        /* Clear Timer1 capture trigger interrupt flag */
        TIMER_ClearCaptureIntFlag(TIMER1);

        g_au32TMR1INTCount++;
    }
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

    /* Enable TIMER peripheral clock */
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_EnableModuleClock(TMR3_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_HIRC, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    /* Set PB multi-function pin for Timer1 external capture pin */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk | SYS_GPB_MFPH_PB14MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD | SYS_GPB_MFPH_PB14MFP_TM1_EXT);

    /* Set multi-function pins for Timer0/Timer3 toggle-output pin and Timer1 event counter pin */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB5MFP_Msk | SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk)) |
                    (SYS_GPB_MFPL_PB5MFP_TM0 | SYS_GPB_MFPL_PB4MFP_TM1 | SYS_GPB_MFPL_PB2MFP_TM3);

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
    uint32_t u32InitCount;
    uint32_t au32CAPValue[10], u32CAPDiff;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();


    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------+\n");
    printf("|    Timer1 Capture Counter Sample Code    |\n");
    printf("+------------------------------------------+\n\n");

    printf("# Timer0 Settings:\n");
    printf("    - Clock source is HIRC\n");
    printf("    - Time-out frequency is 1000 Hz\n");
    printf("    - Toggle-output mode and frequency is 500 Hz\n");
    printf("# Timer3 Settings:\n");
    printf("    - Clock source is HIRC\n");
    printf("    - Time-out frequency is 2 Hz\n");
    printf("    - Toggle-output mode and frequency is 1 Hz\n");
    printf("# Timer1 Settings:\n");
    printf("    - Clock source is HIRC              \n");
    printf("    - Continuous counting mode          \n");
    printf("    - Interrupt enable                  \n");
    printf("    - Compared value is 0xFFFFFF        \n");
    printf("    - Event counter mode enable         \n");
    printf("    - External capture mode enable      \n");
    printf("    - Capture trigger interrupt enable  \n");
    printf("# Connect TM0(PB.5) toggle-output pin to TM1(PB.4) event counter pin.\n");
    printf("# Connect TM3(PB.2) toggle-output pin to TM1_EXT(PB.14) external capture pin.\n\n");

    /* Enable Timer1 NVIC */
    NVIC_EnableIRQ(TMR1_IRQn);

    /* Open Timer0 in toggle-output mode and toggle-output frequency is 500 Hz*/
    TIMER_Open(TIMER0, TIMER_TOGGLE_MODE, 1000);

    /* Open Timer3 in toggle-output mode and toggle-output frequency is 1 Hz */
    TIMER_Open(TIMER3, TIMER_TOGGLE_MODE, 2);

    /* Enable Timer1 event counter input and external capture function */
    TIMER_Open(TIMER1, TIMER_CONTINUOUS_MODE, 1);
    TIMER_SET_PRESCALE_VALUE(TIMER1, 0);
    TIMER_SET_CMP_VALUE(TIMER1, TIMER_CMP_MAX_VALUE);
    TIMER_EnableEventCounter(TIMER1, TIMER_COUNTER_FALLING_EDGE);
    TIMER_EnableCapture(TIMER1, TIMER_CAPTURE_FREE_COUNTING_MODE, TIMER_CAPTURE_FALLING_EDGE);
    TIMER_EnableInt(TIMER1);
    TIMER_EnableCaptureInt(TIMER1);

    /* case 1. */
    printf("# Period between two falling edge captured event should be 500 counts.\n");

    /* Clear Timer1 interrupt counts to 0 */
    u32InitCount = g_au32TMR1INTCount = 0;

    /* Start Timer0, Timer3 and Timer1 counting */
    TIMER_Start(TIMER0);
    TIMER_Start(TIMER3);
    TIMER_Start(TIMER1);

    /* Check Timer1 capture trigger interrupt counts */
    while(g_au32TMR1INTCount <= 10)
    {
        if(g_au32TMR1INTCount != u32InitCount)
        {
            au32CAPValue[u32InitCount] = TIMER_GetCaptureData(TIMER1);
            if(u32InitCount ==  0)
            {
                printf("    [%2d]: %4d. (1st captured value)\n", g_au32TMR1INTCount, au32CAPValue[u32InitCount]);
            }
            else
            {
                u32CAPDiff = au32CAPValue[u32InitCount] - au32CAPValue[u32InitCount - 1];
                printf("    [%2d]: %4d. Diff: %d.\n", g_au32TMR1INTCount, au32CAPValue[u32InitCount], u32CAPDiff);
                if(u32CAPDiff != 500)
                {
                    printf("*** FAIL ***\n");
                    while(1);
                }
            }
            u32InitCount = g_au32TMR1INTCount;
        }
    }
    printf("*** PASS ***\n\n");

    /* case 2. */
    TIMER_DisableCapture(TIMER1);
    TIMER_Stop(TIMER1);
    while(TIMER_IS_ACTIVE(TIMER1));
    TIMER_ClearIntFlag(TIMER1);
    TIMER_ClearCaptureIntFlag(TIMER1);
    /* Enable Timer1 event counter input and external capture function */
    TIMER_Open(TIMER1, TIMER_CONTINUOUS_MODE, 1);
    TIMER_SET_PRESCALE_VALUE(TIMER1, 0);
    TIMER_SET_CMP_VALUE(TIMER1, TIMER_CMP_MAX_VALUE);
    TIMER_EnableEventCounter(TIMER1, TIMER_COUNTER_FALLING_EDGE);
    TIMER_EnableCapture(TIMER1, TIMER_CAPTURE_FREE_COUNTING_MODE, TIMER_CAPTURE_RISING_EDGE);
    TIMER_EnableInt(TIMER1);
    TIMER_EnableCaptureInt(TIMER1);
    TIMER_Start(TIMER1);

    printf("# Get first low duration should be about 250 counts.\n");
    printf("# And follows duration between two rising edge captured event should be 500 counts.\n");

    /* Clear Timer1 interrupt counts to 0 */
    u32InitCount = g_au32TMR1INTCount = 0;

    /* Enable Timer1 event counter input and external capture function */
    TIMER_Open(TIMER1, TIMER_CONTINUOUS_MODE, 1);
    TIMER_SET_PRESCALE_VALUE(TIMER1, 0);
    TIMER_SET_CMP_VALUE(TIMER1, TIMER_CMP_MAX_VALUE);
    TIMER_EnableEventCounter(TIMER1, TIMER_COUNTER_FALLING_EDGE);
    TIMER_EnableCapture(TIMER1, TIMER_CAPTURE_FREE_COUNTING_MODE, TIMER_CAPTURE_RISING_EDGE);
    TIMER_EnableInt(TIMER1);
    TIMER_EnableCaptureInt(TIMER1);
    TIMER_Start(TIMER1);

    /* Check Timer1 capture trigger interrupt counts */
    while ((g_au32TMR1INTCount <= 10) && (u32InitCount < 10))
    {
        if(g_au32TMR1INTCount != u32InitCount)
        {
            au32CAPValue[u32InitCount] = TIMER_GetCaptureData(TIMER1);
            if(u32InitCount ==  0)
            {
                printf("    [%2d]: %4d. (1st captured value)\n", g_au32TMR1INTCount, au32CAPValue[u32InitCount]);
            }
            else
            {
                u32CAPDiff = au32CAPValue[u32InitCount] - au32CAPValue[u32InitCount - 1];
                printf("    [%2d]: %4d. Diff: %d.\n", g_au32TMR1INTCount, au32CAPValue[u32InitCount], u32CAPDiff);
                if(u32CAPDiff != 500)
                {
                    printf("*** FAIL ***\n");
                    while(1);
                }
            }
            u32InitCount = g_au32TMR1INTCount;
        }
    }

    /* Stop Timer0, Timer1 and Timer3 counting */
    TIMER_Stop(TIMER0);
    TIMER_Stop(TIMER1);
    TIMER_Stop(TIMER3);

    printf("*** PASS ***\n");

    while(1);
}

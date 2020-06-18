/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Implement timer counting in periodic mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_au32TMRINTCount[4] = {0};


void TMR0_IRQHandler(void)
{
    if(TIMER0->INTSTS & TIMER_INTSTS_TIF_Msk)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER0->INTSTS = TIMER_INTSTS_TIF_Msk;

        g_au32TMRINTCount[0]++;
    }
}


void TMR1_IRQHandler(void)
{
    if(TIMER1->INTSTS & TIMER_INTSTS_TIF_Msk)
    {
        /* Clear Timer1 time-out interrupt flag */
        TIMER1->INTSTS = TIMER_INTSTS_TIF_Msk;

        g_au32TMRINTCount[1]++;
    }
}


void TMR2_IRQHandler(void)
{
    if(TIMER2->INTSTS & TIMER_INTSTS_TIF_Msk)
    {
        /* Clear Timer2 time-out interrupt flag */
        TIMER2->INTSTS = TIMER_INTSTS_TIF_Msk;

        g_au32TMRINTCount[2]++;
    }
}


void TMR3_IRQHandler(void)
{
    if(TIMER3->INTSTS & TIMER_INTSTS_TIF_Msk)
    {
        /* Clear Timer3 time-out interrupt flag */
        TIMER3->INTSTS = TIMER_INTSTS_TIF_Msk;

        g_au32TMRINTCount[3]++;
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) != CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(1);

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Switch UART0 clock source to HIRC */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART0SEL_Msk) | CLK_CLKSEL1_UART0SEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_UART0DIV_Msk) | CLK_CLKDIV0_UART0(1);

    /* Enable UART0 and Timer peripheral clock */
    CLK->APBCLK0 |= (CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_TMR0CKEN_Msk | CLK_APBCLK0_TMR1CKEN_Msk | CLK_APBCLK0_TMR2CKEN_Msk | CLK_APBCLK0_TMR3CKEN_Msk);
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~(CLK_CLKSEL1_TMR0SEL_Msk | CLK_CLKSEL1_TMR1SEL_Msk | CLK_CLKSEL1_TMR2SEL_Msk | CLK_CLKSEL1_TMR3SEL_Msk)) |
                   (CLK_CLKSEL1_TMR0SEL_HIRC | CLK_CLKSEL1_TMR1SEL_PCLK0 | CLK_CLKSEL1_TMR2SEL_HIRC | CLK_CLKSEL1_TMR3SEL_HIRC);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

/*----------------------------------------------------------------------*/
/* Init UART0                                                           */
/*----------------------------------------------------------------------*/
void UART0_Init(void)
{
    /* Reset UART0 */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

int main(void)
{
    uint32_t u32InitCount, au32Counts[4];
    uint32_t u32Prescale;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+--------------------------------------------+\n");
    printf("|    Timer Periodic Interrupt Sample Code    |\n");
    printf("+--------------------------------------------+\n\n");

    printf("# Timer0 Settings:\n");
    printf("    - Clock source is HIRC      \n");
    printf("    - Time-out frequency is 1 Hz\n");
    printf("    - Periodic mode             \n");
    printf("    - Interrupt enable          \n");
    printf("# Timer1 Settings:\n");
    printf("    - Clock source is PCLK0     \n");
    printf("    - Time-out frequency is 2 Hz\n");
    printf("    - Periodic mode             \n");
    printf("    - Interrupt enable          \n");
    printf("# Timer2 Settings:\n");
    printf("    - Clock source is HIRC     \n");
    printf("    - Time-out frequency is 4 Hz\n");
    printf("    - Periodic mode             \n");
    printf("    - Interrupt enable          \n");
    printf("# Timer3 Settings:\n");
    printf("    - Clock source is HIRC      \n");
    printf("    - Time-out frequency is 8 Hz\n");
    printf("    - Periodic mode             \n");
    printf("    - Interrupt enable          \n");
    printf("# Check Timer0 ~ Timer3 interrupt counts are reasonable or not.\n\n");

    /* Open Timer0 in periodic mode, enable interrupt and 1 interrupt tick per second */
    u32Prescale = 32;
    TIMER0->CTL = (TIMER0->CTL & ~(TIMER_CTL_OPMODE_Msk | TIMER_CTL_PSC_Msk)) |
                  (TIMER_PERIODIC_MODE | (u32Prescale-1) | TIMER_CTL_INTEN_Msk);
    TIMER0->CMP = __HIRC / u32Prescale;

    /* Open Timer1 in periodic mode, enable interrupt and 2 interrupt ticks per second */
    u32Prescale = 24;
    TIMER1->CTL = (TIMER1->CTL & ~(TIMER_CTL_OPMODE_Msk | TIMER_CTL_PSC_Msk)) |
                  (TIMER_PERIODIC_MODE | (u32Prescale-1) | TIMER_CTL_INTEN_Msk);
    TIMER1->CMP = (__HIRC/2) / u32Prescale / 2;

    /* Open Timer2 in periodic mode, enable interrupt and 4 interrupt ticks per second */
    u32Prescale = 48;
    TIMER2->CTL = (TIMER2->CTL & ~(TIMER_CTL_OPMODE_Msk | TIMER_CTL_PSC_Msk)) |
                  (TIMER_PERIODIC_MODE | (u32Prescale-1) | TIMER_CTL_INTEN_Msk);
    TIMER2->CMP = __HIRC / u32Prescale / 4;

    /* Open Timer3 in periodic mode, enable interrupt and 8 interrupt ticks per second */
    u32Prescale = 32;
    TIMER3->CTL = (TIMER3->CTL & ~(TIMER_CTL_OPMODE_Msk | TIMER_CTL_PSC_Msk)) |
                  (TIMER_PERIODIC_MODE | (u32Prescale-1) | TIMER_CTL_INTEN_Msk);
    TIMER3->CMP = __HIRC / u32Prescale / 8;

    /* Enable Timer0 ~ Timer3 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);
    NVIC_EnableIRQ(TMR1_IRQn);
    NVIC_EnableIRQ(TMR2_IRQn);
    NVIC_EnableIRQ(TMR3_IRQn);

    /* Clear Timer0 ~ Timer3 interrupt counts to 0 */
    g_au32TMRINTCount[0] = g_au32TMRINTCount[1] = g_au32TMRINTCount[2] = g_au32TMRINTCount[3] = 0;
    u32InitCount = g_au32TMRINTCount[0];

    /* Start Timer0 ~ Timer3 counting */
    TIMER0->CTL |= TIMER_CTL_CNTEN_Msk;
    TIMER1->CTL |= TIMER_CTL_CNTEN_Msk;
    TIMER2->CTL |= TIMER_CTL_CNTEN_Msk;
    TIMER3->CTL |= TIMER_CTL_CNTEN_Msk;

    /* Check Timer0 ~ Timer3 interrupt counts */
    printf("# Timer interrupt counts :\n");
    while(u32InitCount < 20)
    {
        if(g_au32TMRINTCount[0] != u32InitCount)
        {
            au32Counts[0] = g_au32TMRINTCount[0];
            au32Counts[1] = g_au32TMRINTCount[1];
            au32Counts[2] = g_au32TMRINTCount[2];
            au32Counts[3] = g_au32TMRINTCount[3];
            printf("    TMR0:%3d    TMR1:%3d    TMR2:%3d    TMR3:%3d\n",
                   au32Counts[0], au32Counts[1], au32Counts[2], au32Counts[3]);
            u32InitCount = g_au32TMRINTCount[0];

            if((au32Counts[1] > (au32Counts[0] * 2 + 1)) || (au32Counts[1] < (au32Counts[0] * 2 - 1)) ||
                    (au32Counts[2] > (au32Counts[0] * 4 + 1)) || (au32Counts[2] < (au32Counts[0] * 4 - 1)) ||
                    (au32Counts[3] > (au32Counts[0] * 8 + 1)) || (au32Counts[3] < (au32Counts[0] * 8 - 1)))
            {
                printf("*** FAIL ***\n");
                while(1) {}
            }
        }
    }

    printf("*** PASS ***\n");

    while(1);
}

/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use ACMP to trigger Timer0 counter reset mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


void TMR0_IRQHandler(void)
{
    printf("ACMP triggered timer reset while counter is at %d\n", TIMER0->CAP);

    /* Clear timer capture interrupt flag. */
    TIMER0->EINTSTS = TIMER_EINTSTS_CAPIF_Msk;
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

    /* Enable UART0, Timer0 and ACMP01 peripheral clock */
    CLK->APBCLK0 |= (CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_TMR0CKEN_Msk | CLK_APBCLK0_ACMP01CKEN_Msk);

    /* Select IP clock source */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_TMR0SEL_Msk) | CLK_CLKSEL1_TMR0SEL_HIRC;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set PB4 multi-function pin for ACMP0 positive input pin */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB4MFP_Msk) | SYS_GPB_MFPL_PB4MFP_ACMP1_P1;
    /* Disable digital input path of analog pin ACMP1_P1 to prevent leakage */
    PB->DINOFF |= (BIT4<<GPIO_DINOFF_DINOFF0_Pos);

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
    int volatile i;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    PB5 = 1;    /* Set init state to high */
    PB->MODE = (PB->MODE & ~GPIO_MODE_MODE5_Msk) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE5_Pos);  /* Set to output mode */

    printf("\nThis sample code demonstrate ACMP trigger timer counter reset mode.\n");
    printf("Please connect PB.5 with ACMP0 positive input pin PB.4,  press any key to continue\n");
    getchar();

    /* Update prescale to set proper resolution. */
    /* Set capture source from Internal event */
    TIMER0->CTL = (TIMER0->CTL & ~(TIMER_CTL_OPMODE_Msk | TIMER_CTL_PSC_Msk | TIMER_CTL_CAPSRC_Msk)) |
                  (TIMER_PERIODIC_MODE | TIMER_CAPSRC_INTERNAL);

    /* Set compare value as large as possible, so don't need to worry about counter overrun too frequently. */
    TIMER0->CMP = TIMER_CMP_MAX_VALUE;

    /* Configure Timer 0 free counting mode */
    /* Set capture source from Internal event ACMP1 */
    TIMER0->EXTCTL = (TIMER0->EXTCTL & ~(TIMER_EXTCTL_CAPFUNCS_Msk | TIMER_EXTCTL_CAPEDGE_Msk | TIMER_EXTCTL_INTERCAPSEL_Msk)) |
                     (TIMER_CAPTURE_COUNTER_RESET_MODE | TIMER_CAPTURE_RISING_EDGE | TIMER_EXTCTL_CAPEN_Msk | TIMER_INTERCAPSEL_ACMP1 | TIMER_EXTCTL_CAPIEN_Msk);

    /* Start Timer 0 */
    TIMER0->CTL |= TIMER_CTL_CNTEN_Msk;

    /* Configure ACMP1. Enable ACMP1 and select band-gap voltage as the source of ACMP negative input. */
    /* Select P1 as ACMP1 positive input channel */
    ACMP01->CTL[1] = (ACMP01->CTL[1] & ~(ACMP_CTL_NEGSEL_Msk | ACMP_CTL_HYSEN_Msk | ACMP_CTL_POSSEL_Msk)) |
                     (ACMP_CTL_NEGSEL_VBG | ACMP_CTL_HYSTERESIS_DISABLE | ACMP_CTL_ACMPEN_Msk | ACMP_CTL_POSSEL_P1);

    /* Enable timer interrupt */
    NVIC_EnableIRQ(TMR0_IRQn);

    while(1)
    {
        PB5 = 0;
        CLK_SysTickDelay(10000);
        PB5 = 1;
        CLK_SysTickDelay(10000);
    }
}

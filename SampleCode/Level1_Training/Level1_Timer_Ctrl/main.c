/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    TIMER function for level1 training course
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

#define LED_R   PC4
#define LED_B   PC3
#define LED_G   PC5
#define LEDR1   PB14

#define LED_ON      0
#define LED_OFF     1

void SYS_Init(void)
{
    /* Enable HXT clock (external XTAL 12MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_100MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);

    /* Set module clock */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, NULL);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HXT, NULL);
}

void UART0_Init()
{
    /* Set GPB multi-function pins to UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

void LED_Init(void)
{
    /* Set PC.3 ~ PC.5 to GPIO */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC3MFP_Msk | SYS_GPC_MFPL_PC4MFP_Msk | SYS_GPC_MFPL_PC5MFP_Msk)) |
                    (SYS_GPC_MFPL_PC3MFP_GPIO | SYS_GPC_MFPL_PC4MFP_GPIO | SYS_GPC_MFPL_PC5MFP_GPIO);

    /* Set PC.3 ~ PC.5 to GPIO output */
    GPIO_SetMode(PC, (BIT3 | BIT4 | BIT5), GPIO_MODE_OUTPUT);

    /* Let LED off after initialize */
    LED_R = LED_OFF;
    LED_G = LED_OFF;
    LED_B = LED_OFF;
}

void Timer_Init(void)
{
    /**************  TIMER0 ***************/
    /* Set TIMER0 in periodic mode ,frequency 1Hz and enable its interrupt */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1);
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);

    /**************  TIMER1 ***************/
    /* Set PB.14 as TM1_EXT pin */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB14MFP_Msk)) | (SYS_GPB_MFPH_PB14MFP_TM1_EXT);
    /* Set TIMER1 in periodic mode ,frequency 2Hz and enable its interrupt */
    TIMER_Open(TIMER1, TIMER_TOGGLE_MODE, 2);
    /* Set TM1_EXT as TOUT pin */
    TIMER_SELECT_TOUT_PIN(TIMER1, TIMER_TOUT_PIN_FROM_TX_EXT);
}

int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("+----------------------------------------+\n");
    printf("|    Level1 TIMER control Sample Code    |\n");
    printf("+----------------------------------------+\n\n");

    /* Init LED */
    LED_Init();

    /* Init TIMER */
    Timer_Init();

    /* Let TIMER0 & TIMER1 start to count */
    TIMER_Start(TIMER0);
    TIMER_Start(TIMER1);

    while(1);
}

void TMR0_IRQHandler(void)
{
    /* Check if the interrupt occurred */
    if (TIMER_GetIntFlag(TIMER0))
    {
        /* Toggle LED */
        LED_B ^= 1;
        /* Clear TIMER0 interrupt flag */
        TIMER_ClearIntFlag(TIMER0);
    }
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/

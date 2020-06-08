/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate the RTC function and displays current time to the
 *           UART console
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

volatile uint32_t  g_u32TICK = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* RTC Tick Handle                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void RTC_TickHandle(void)
{
    /* Get the current time */
    printf(" Current Time:20%X/%02X/%02X %02X:%02X:%02X\n",
           (RTC->CAL >> RTC_CAL_YEAR_Pos) & 0xFF, (RTC->CAL >> RTC_CAL_MON_Pos) & 0xFF, (RTC->CAL >> RTC_CAL_DAY_Pos) & 0xFF,
           (RTC->TIME >> RTC_TIME_HR_Pos) & 0xFF, (RTC->TIME >> RTC_TIME_MIN_Pos) & 0xFF, (RTC->TIME >> RTC_TIME_SEC_Pos) & 0xFF);

    g_u32TICK++;
}

/**
  * @brief  RTC ISR to handle interrupt event
  * @param  None
  * @retval None
  */
void RTC_IRQHandler(void)
{

    if ( (RTC->INTEN & RTC_INTEN_TICKIEN_Msk) && (RTC->INTSTS & RTC_INTSTS_TICKIF_Msk) )        /* tick interrupt occurred */
    {
        RTC->INTSTS = 0x2;

        RTC_TickHandle();
    }

}

void Delay(uint32_t ucnt)
{
    volatile uint32_t i = ucnt;

    while (i--);
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable LXT clock */
    CLK->PWRCTL |= CLK_PWRCTL_LXTEN_Msk;

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

    /* Enable RTC clock */
    CLK->APBCLK0 |= CLK_APBCLK0_RTCCKEN_Msk ;

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))
                    |(SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}


int32_t main(void)
{
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    /* Reset UART0 */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

    /* Open RTC */
    RTC->INIT = RTC_INIT_KEY;

    if (RTC->INIT != RTC_INIT_ACTIVE_Msk)
    {
        RTC->INIT = RTC_INIT_KEY;

        while (RTC->INIT != RTC_INIT_ACTIVE_Msk)
        {
        }
    }
    /* Time Setting */
    RTC->CLKFMT  = RTC_CLOCK_24;
    RTC->WEEKDAY = RTC_MONDAY;
    RTC->CAL     = 0x00170501;         /* Date: 2017/05/01 */
    RTC->TIME    = 0x00123000;         /* Time: 12:30:00 */


    printf("\n RTC Time Display Test (Exit after 5 seconds)\n\n");

    /* Set Tick Period */
    RTC->TICK    = RTC_TICK_1_SEC;     /* One RTC tick is 1 second */

    /* Enable RTC Tick Interrupt */
    RTC->INTEN  = RTC_INTEN_TICKIEN_Msk;

    NVIC_EnableIRQ(RTC_IRQn);

    g_u32TICK = 0;
    while(g_u32TICK < 5);

    printf("\n RTC Time Display Test End !!\n");

    /* Disable RTC Tick Interrupt */
    RTC->INTEN  &= ~RTC_INTEN_TICKIEN_Msk;

    NVIC_DisableIRQ(RTC_IRQn);

    while(1);

}






/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/




/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Use RTC alarm interrupt event to wake up system.
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
extern int IsDebugFifoEmpty(void);
volatile uint8_t g_u8IsRTCAlarmINT = 0;


/**
 * @brief       IRQ Handler for RTC Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The RTC_IRQHandler is default IRQ of RTC, declared in startup_M2351.s.
 */
void RTC_IRQHandler(void)
{
    /* To check if RTC alarm interrupt occurred */
    if(RTC->INTSTS & RTC_INTSTS_ALMIF_Msk)
    {
        /* Clear RTC alarm interrupt flag */
        RTC->INTSTS = RTC_INTSTS_ALMIF_Msk;

        g_u8IsRTCAlarmINT++;
    }
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

int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    /* Reset UART0 */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-------------------------------------+\n");
    printf("|    RTC Alarm Wake-up Sample Code    |\n");
    printf("+-------------------------------------+\n\n");

    /* Open RTC */
    RTC->INIT = RTC_INIT_KEY;

    if (RTC->INIT != RTC_INIT_ACTIVE_Msk)
    {
        RTC->INIT = RTC_INIT_KEY;

        while (RTC->INIT != RTC_INIT_ACTIVE_Msk)
        {
        }
    }

    RTC->CLKFMT  = RTC_CLOCK_24;
    RTC->WEEKDAY = RTC_SUNDAY;
    RTC->CAL     = 0x00170315;         /* Date: 2017/03/15 */
    RTC->TIME    = 0x00235950;         /* Time: 23:59:50 */
    RTC->INTEN   = RTC_INTEN_TICKIEN_Msk;
    RTC->TICK    = RTC_TICK_1_SEC;     /* One RTC tick is 1 second */

    /* Set RTC alarm date/time */
    /* Setting RTC alarm date/time and enable alarm interrupt */
    RTC->CALM  = 0x00170315;            /* Date: 2017/03/15 */
    RTC->TALM  = 0x00235955;            /* Time: 23:59:55 */

    /* Enable RTC alarm interrupt and wake-up function will be enabled also */
    RTC->INTEN = RTC_INTEN_ALMIEN_Msk;

    /* Enable RTC NVIC */
    NVIC_EnableIRQ(RTC_IRQn);

    printf("# Set RTC current date/time: 2017/03/15 23:59:50.\n");
    printf("# Set RTC alarm date/time:   2017/03/15 23:59:55.\n");
    printf("# Wait system waken-up by RTC alarm interrupt event.\n");

    g_u8IsRTCAlarmINT = 0;

    /* System enter to Power-down */
    /* To program PWRCTL register, it needs to disable register protection first. */
    SYS_UnlockReg();
    printf("\nSystem enter to power-down mode ...\n");

    /* To check if all the debug messages are finished */
    while(IsDebugFifoEmpty() == 0);

    /* Set the processor uses deep sleep as its low power mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Set system Power-down enabled*/
    CLK->PWRCTL |= CLK_PWRCTL_PDEN_Msk;

    /* Chip enter Power-down mode after CPU run WFI instruction */
    __WFI();

    while(g_u8IsRTCAlarmINT == 0);

    /* Read current RTC date/time */
    printf("System has been waken-up and current date/time is:\n");
    printf("    20%X/%02X/%02X %02X:%02X:%02X\n",
           (RTC->CAL >> RTC_CAL_YEAR_Pos) & 0xFF, (RTC->CAL >> RTC_CAL_MON_Pos) & 0xFF, (RTC->CAL >> RTC_CAL_DAY_Pos) & 0xFF,
           (RTC->TIME >> RTC_TIME_HR_Pos) & 0xFF, (RTC->TIME >> RTC_TIME_MIN_Pos) & 0xFF, (RTC->TIME >> RTC_TIME_SEC_Pos) & 0xFF);


    printf("\n\n");
    printf("# Set next RTC alarm date/time: 2017/03/16 00:00:05.\n");
    printf("# Wait system waken-up by RTC alarm interrupt event.\n");
    RTC->CALM  = 0x00170316;            /* Date: 2017/03/16 */
    RTC->TALM  = 0x00000005;            /* Time: 00:00:05 */

    g_u8IsRTCAlarmINT = 0;

    /* System enter to Power-down */
    /* To program PWRCTL register, it needs to disable register protection first. */
    SYS_UnlockReg();

    printf("\nSystem enter to power-down mode ...\n");
    /* To check if all the debug messages are finished */
    while(IsDebugFifoEmpty() == 0);

    /* Set the processor uses deep sleep as its low power mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Set system Power-down enabled*/
    CLK->PWRCTL |= CLK_PWRCTL_PDEN_Msk;

    /* Chip enter Power-down mode after CPU run WFI instruction */
    __WFI();

    while(g_u8IsRTCAlarmINT == 0);

    /* Read current RTC date/time */
    printf("System has been waken-up and current date/time is:\n");
    printf("    20%X/%02X/%02X %02X:%02X:%02X\n",
           (RTC->CAL >> RTC_CAL_YEAR_Pos) & 0xFF, (RTC->CAL >> RTC_CAL_MON_Pos) & 0xFF, (RTC->CAL >> RTC_CAL_DAY_Pos) & 0xFF,
           (RTC->TIME >> RTC_TIME_HR_Pos) & 0xFF, (RTC->TIME >> RTC_TIME_MIN_Pos) & 0xFF, (RTC->TIME >> RTC_TIME_SEC_Pos) & 0xFF);

    while(1)
    {
        ;
    }
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/

/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate the RTC alarm function. It sets an alarm 10 seconds
 *           after execution
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

volatile int32_t   g_bAlarm  = FALSE;


/*---------------------------------------------------------------------------------------------------------*/
/* RTC Alarm Handle                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void RTC_AlarmHandle(void)
{
    printf(" Alarm!!\n");
    g_bAlarm = TRUE;
}

/**
  * @brief  RTC ISR to handle interrupt event
  * @param  None
  * @retval None
  */
void RTC_IRQHandler(void)
{
    if ( (RTC->INTEN & RTC_INTEN_ALMIEN_Msk) && (RTC->INTSTS & RTC_INTSTS_ALMIF_Msk) )        /* alarm interrupt occurred */
    {
        RTC->INTSTS = 0x1;

        RTC_AlarmHandle();
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable RTC clock */
    CLK_EnableModuleClock(RTC_MODULE);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))
                    |(SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}


int32_t main(void)
{
    S_RTC_TIME_DATA_T sInitTime;
    S_RTC_TIME_DATA_T sCurTime;

    SYS_Init();

    UART0_Init();

    /* Time Setting */
    sInitTime.u32Year       = 2017;
    sInitTime.u32Month      = 5;
    sInitTime.u32Day        = 1;
    sInitTime.u32Hour       = 12;
    sInitTime.u32Minute     = 30;
    sInitTime.u32Second     = 0;
    sInitTime.u32DayOfWeek  = RTC_MONDAY;
    sInitTime.u32TimeScale  = RTC_CLOCK_24;

    if(RTC_Open(&sInitTime) != 0)
    {
        printf("\n RTC initial fail!!");
        printf("\n Please check h/w setting!!");
        while(1);
    }

    printf("\n RTC Alarm Test (Alarm after 10 seconds)\n\n");

    g_bAlarm = FALSE;

    /* Get the current time */
    RTC_GetDateAndTime(&sCurTime);

    printf(" Current Time:%d/%02d/%02d %02d:%02d:%02d\n",sCurTime.u32Year,sCurTime.u32Month,
           sCurTime.u32Day,sCurTime.u32Hour,sCurTime.u32Minute,sCurTime.u32Second);

    /* The alarm time setting */
    sCurTime.u32Second = sCurTime.u32Second + 10;

    /* Set the alarm time */
    RTC_SetAlarmDateAndTime(&sCurTime);

    /* Clear interrupt status */
    RTC->INTSTS = RTC_INTSTS_ALMIF_Msk;

    /* Enable RTC Alarm Interrupt */
    RTC_EnableInt(RTC_INTEN_ALMIEN_Msk);

    NVIC_EnableIRQ(RTC_IRQn);

    while(!g_bAlarm);

    /* Get the current time */
    RTC_GetDateAndTime(&sCurTime);

    printf(" Current Time:%d/%02d/%02d %02d:%02d:%02d\n",sCurTime.u32Year,sCurTime.u32Month,
           sCurTime.u32Day,sCurTime.u32Hour,sCurTime.u32Minute,sCurTime.u32Second);

    /* Disable RTC Alarm Interrupt */
    RTC_DisableInt(RTC_INTEN_ALMIEN_Msk);
    NVIC_DisableIRQ(RTC_IRQn);

    printf("\n RTC Alarm Test End !!\n");

    while(1);

}



/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/




/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 10 $
 * $Date: 18/07/17 6:05p $
 * @brief
 *           Change duty cycle of output waveform to show breathing effect of
                         Red LED.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define PWM_Prescaler       48
#define PWM_Period          1999

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t    g_u8Forward = 1;
volatile uint32_t g_u32BreathingCount = 0;

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable PWM1 module clock */
    CLK_EnableModuleClock(PWM1_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* PWM clock frequency configuration                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PLL clock as 96 MHz from HIRC/4 */
    CLK_EnablePLL(CLK_PLLCTL_PLLSRC_HIRC_DIV4, 96000000);

    /* Waiting for PLL clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Select HCLK clock source as PLL and and HCLK clock divider as 2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* PWM clock frequency can be set equal or double to HCLK by choosing case 1 or case 2 */
    /* case 1.PWM clock frequency is set equal to HCLK: select PWM module clock source as PCLK */
//    CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL2_PWM1SEL_PCLK0, 0);

    /* case 2.PWM clock frequency is set double to HCLK: select PWM module clock source as PLL */
    CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL2_PWM1SEL_PLL, NULL);
    /*---------------------------------------------------------------------------------------------------------*/

    /* Reset PWM1 module */
    SYS_ResetModule(PWM1_RST);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PC.4 multi-function pins for PWM1 Channel 1 for Red LED */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC4MFP_Msk)) |
                    SYS_GPC_MFPL_PC4MFP_PWM1_CH1;
}

void PWM_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init PWM1                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set prescaler of PWM1 Channel 1 as PWM_Prescaler for Red LED */
    PWM_SET_PRESCALER(PWM1, 1, PWM_Prescaler - 1);
    /* Set period of PWM1 Channel 1 as PWM_Period for Red LED */
    PWM_SET_CNR(PWM1, 1, PWM_Period);
    /* Set compare of PWM1 Channel 1 as 0 for Red LED */
    PWM_SET_CMR(PWM1, 1, 0);
    /* Set Counter Type */
    PWM_SET_ALIGNED_TYPE(PWM1, BIT1, PWM_UP_COUNTER);
    /* Set Zero Point Output Low, Compare Up Point Output Nothing, Period Point Output High, Compare Down Point Output High */
    PWM_SET_OUTPUT_LEVEL(PWM1, BIT1, PWM_OUTPUT_LOW, PWM_OUTPUT_HIGH, PWM_OUTPUT_NOTHING, PWM_OUTPUT_NOTHING);
    /* Enable PWM1 Output path for channel 1 */
    PWM_EnableOutput(PWM1, BIT1);
    /* Enable PWM1 channel 1 period interrupt */
    PWM_EnablePeriodInt(PWM1, 1, 0);
    NVIC_EnableIRQ(PWM1_IRQn);

    /* Start PWM1 Counter */
    PWM_Start(PWM1, BIT1);
}

void PWM1_IRQHandler(void)
{
    /* Increase LED brightness */
    if(g_u8Forward == 1)
    {
        if(g_u32BreathingCount < PWM_Period)
            g_u32BreathingCount++;
        else
            g_u8Forward = 0;
    }
    /* Decrease LED brightness */
    else
    {
        if(g_u32BreathingCount > 0)
            g_u32BreathingCount--;
        else
            g_u8Forward = 1;
    }

    /* Set compare of PWM1 Channel 1 for Red LED */
    PWM_SET_CMR(PWM1, 1, g_u32BreathingCount);

    /* Clear PWM1 channel 1 period interrupt flag */
    PWM_ClearPeriodIntFlag(PWM1, 1);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init PWM1 to drive RGB LED */
    PWM_Init();

    while(1);
}

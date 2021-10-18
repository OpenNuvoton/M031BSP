/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 9 $
 * $Date: 19/07/29 2:15p $
 * @brief    Use BPWM0 Channel 0(PA.0) to capture the BPWM1 Channel 0(PB.11) Waveform.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/**
 * @brief       BPWM0 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle BPWM0 interrupt event
 */
void BPWM0_IRQHandler(void)
{
    if (BPWM_GetCaptureIntFlag(BPWM0, 0) > 1)
    {
        BPWM_ClearCaptureIntFlag(BPWM0, 0, BPWM_CAPTURE_INT_FALLING_LATCH);
    }
}

/*--------------------------------------------------------------------------------------*/
/* Capture function to calculate the input waveform information                         */
/* u16Count[4] : Keep the internal counter value when input signal rising / falling     */
/*               happens                                                                */
/*                                                                                      */
/* time    A    B     C     D                                                           */
/*           ___   ___   ___   ___   ___   ___   ___   ___                              */
/*      ____|   |_|   |_|   |_|   |_|   |_|   |_|   |_|   |_____                        */
/* index              0 1   2 3                                                         */
/*                                                                                      */
/* The capture internal counter down count from 0x10000, and reload to 0x10000 after    */
/* input signal falling happens (Time B/C/D)                                            */
/*--------------------------------------------------------------------------------------*/
void CalPeriodTime(BPWM_T *BPWM, uint32_t u32Ch)
{
    uint16_t u16Count[4];
    uint32_t u32i;
    uint16_t u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod;
    uint32_t u32TimeOutCount;

    /* Clear Capture Falling Indicator (Time A) */
    BPWM_ClearCaptureIntFlag(BPWM, u32Ch, BPWM_CAPTURE_INT_FALLING_LATCH | BPWM_CAPTURE_INT_RISING_LATCH);

    /* setup timeout */
    u32TimeOutCount = SystemCoreClock;

    /* Wait for Capture Falling Indicator  */
    while (BPWM_GetCaptureIntFlag(BPWM, u32Ch) < 2)
    {
        if(u32TimeOutCount == 0)
        {
            printf("\nSomething is wrong, please check if pin connection is correct. \n");
            while(1);
        }
        u32TimeOutCount--;
    }

    /* Clear Capture Falling Indicator (Time B)*/
    BPWM_ClearCaptureIntFlag(BPWM, u32Ch, BPWM_CAPTURE_INT_FALLING_LATCH);

    u32i = 0;

    while (u32i < 4)
    {
        /* Wait for Capture Falling Indicator */
        while (BPWM_GetCaptureIntFlag(BPWM, u32Ch) < 2);

        /* Clear Capture Falling and Rising Indicator */
        BPWM_ClearCaptureIntFlag(BPWM, u32Ch, BPWM_CAPTURE_INT_FALLING_LATCH | BPWM_CAPTURE_INT_RISING_LATCH);

        /* Get Capture Falling Latch Counter Data */
        u16Count[u32i++] = BPWM_GET_CAPTURE_FALLING_DATA(BPWM, u32Ch);

        /* Wait for Capture Rising Indicator */
        while (BPWM_GetCaptureIntFlag(BPWM, u32Ch) < 1);

        /* Clear Capture Rising Indicator */
        BPWM_ClearCaptureIntFlag(BPWM, u32Ch, BPWM_CAPTURE_INT_RISING_LATCH);

        /* Get Capture Rising Latch Counter Data */
        u16Count[u32i++] = BPWM_GET_CAPTURE_RISING_DATA(BPWM, u32Ch);
    }

    u16RisingTime = u16Count[1];

    u16FallingTime = u16Count[0];

    u16HighPeriod = u16Count[1] - u16Count[2];

    u16LowPeriod = 0x10000 - u16Count[1];

    u16TotalPeriod = 0x10000 - u16Count[2];

    printf("\nPWM generate: \nHigh Period=14999 ~ 15001, Low Period=34999 ~ 35001, Total Period=49999 ~ 50001\n");
    printf("\nCapture Result: Rising Time = %d, Falling Time = %d \nHigh Period = %d, Low Period = %d, Total Period = %d.\n\n",
           u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod);

    if ((u16HighPeriod < 14399) || (u16HighPeriod > 14401) || (u16LowPeriod < 33599) || (u16LowPeriod > 33601) || (u16TotalPeriod < 47999) || (u16TotalPeriod > 48001))
        printf("Capture Test Fail!!\n");
    else
        printf("Capture Test Pass!!\n");
}

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable BPWM0 module clock */
    CLK_EnableModuleClock(BPWM0_MODULE);

    /* Enable BPWM1 module clock */
    CLK_EnableModuleClock(BPWM1_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* BPWM0 and BPWM1 clock frequency configuration                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* BPWM0 clock frequency can be set equal to HCLK by choosing case 1 */
    /* case 1. BPWM0 clock frequency is set equal to HCLK: select BPWM module clock source as PCLK */
    CLK_SetModuleClock(BPWM0_MODULE, CLK_CLKSEL2_BPWM0SEL_PCLK0, 0);

    /* BPWM1 clock frequency can be set equal to HCLK by choosing case 1 */
    /* case 1. BPWM1 clock frequency is set equal to HCLK: select BPWM1 module clock source as PCLK */
    CLK_SetModuleClock(BPWM1_MODULE, CLK_CLKSEL2_BPWM1SEL_PCLK1, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set PA.0 multi-function pin for BPWM0 channel 0 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA0MFP_Msk) | SYS_GPA_MFPL_PA0MFP_BPWM0_CH0;

    /* Set PB.11 multi-function pin for BPWM1 Channel 0 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB11MFP_Msk) | SYS_GPB_MFPH_PB11MFP_BPWM1_CH0;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32TimeOutCount;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("+------------------------------------------------------------------------+\n");
    printf("|                          BPWM Driver Sample Code                       |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use BPWM0 channel 0 to capture\n  the signal from BPWM1 channel 0.\n");
    printf("  I/O configuration:\n");
    printf("    BPWM0_CH0(PA.0 BPWM0 channel 0) <--> BPWM0_CH0(PB.11 BPWM1 channel 0)\n\n");
    printf("Use BPWM0 Channel 0(PA.0) to capture the BPWM1 Channel 0(PB.11) Waveform\n");

    while (1)
    {
        printf("Press any key to start BPWM Capture Test\n");
        getchar();

        /*--------------------------------------------------------------------------------------*/
        /* Set the BPWM1 Channel 0 as BPWM output function.                                       */
        /*--------------------------------------------------------------------------------------*/

        /* Assume BPWM1 output frequency is 250 Hz and duty ratio is 30%, user can calculate BPWM settings by follows.
           duty ratio = (CMR)/(CNR+1)
           cycle time = CNR+1
           High level = CMR
           BPWM1 clock source frequency = 48,000,000
           (CNR+1) = BPWM1 clock source frequency/prescaler/BPWM1 output frequency
                   = 48,000,000/4/250 = 48,000
           (Note: CNR is 16 bits, so if calculated value is larger than 65535, user should increase prescale value.)
           CNR = 47999
           duty ratio = 30% ==> (CMR)/(CNR+1) = 30%
           CMR = 14400
           Prescale value is 3 : prescaler= 4
        */
        /* set BPWM1 channel 0 output configuration */
        BPWM_ConfigOutputChannel(BPWM1, 0, 250, 30);

        /* Enable BPWM Output path for BPWM1 channel 0 */
        BPWM_EnableOutput(BPWM1, BPWM_CH_0_MASK);

        /* Enable Timer for BPWM1 channel 0 */
        BPWM_Start(BPWM1, BPWM_CH_0_MASK);

        /*--------------------------------------------------------------------------------------*/
        /* Set the BPWM0 channel 0 for capture function                                          */
        /*--------------------------------------------------------------------------------------*/

        /* If input minimum frequency is 250Hz, user can calculate capture settings by follows.
           Capture clock source frequency = HIRC = 48,000,000 in the sample code.
           (CNR+1) = Capture clock source frequency/prescaler/minimum input frequency
                   = 48000000/4/250 = 48000
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 0xFFFF
           (Note: In capture mode, user should set CNR to 0xFFFF to increase capture frequency range.)
        */
        /* set BPWM0 channel 0 capture configuration */
        BPWM_ConfigCaptureChannel(BPWM0, 0, 80, 0);

        /* Enable capture falling edge interrupt for BPWM0 channel 0 */
        //BPWM_EnableCaptureInt(BPWM0, 0, BPWM_CAPTURE_INT_FALLING_LATCH);

        /* Enable BPWM0 NVIC interrupt */
        //NVIC_EnableIRQ(BPWM0_IRQn);

        /* Enable Timer for BPWM0 channel 0 */
        BPWM_Start(BPWM0, BPWM_CH_0_MASK);

        /* Enable Capture Function for BPWM0 channel 0 */
        BPWM_EnableCapture(BPWM0, BPWM_CH_0_MASK);

        /* Enable falling capture reload */
        BPWM0->CAPCTL |= BPWM_CAPCTL_FCRLDEN0_Msk;

        /* setup timeout */
        u32TimeOutCount = SystemCoreClock;

        /* Wait until BPWM0 channel 0 Timer start to count */
        while ((BPWM0->CNT) == 0)
        {
            if(u32TimeOutCount == 0)
            {
                printf("BPWM encounters some errors, please check it. \n");
                while(1);
            }
            u32TimeOutCount--;
        }

        /* Capture the Input Waveform Data */
        CalPeriodTime(BPWM0, 0);

        /*---------------------------------------------------------------------------------------------------------*/
        /* Stop BPWM1 channel 0 (Recommended procedure method 1)                                                    */
        /* Set BPWM Timer loaded value(Period) as 0. When BPWM internal counter(CNT) reaches to 0, disable BPWM Timer */
        /*---------------------------------------------------------------------------------------------------------*/
        /* Set BPWM0 channel 0 loaded value as 0 */
        BPWM_Stop(BPWM1, BPWM_CH_0_MASK);

        /* Wait until BPWM1 channel 0 Timer Stop */
        while ((BPWM1->CNT & BPWM_CNT_CNT_Msk) != 0)
        {
            if(u32TimeOutCount == 0)
            {
                printf("BPWM encounters some errors, please check it. \n");
                while(1);
            }
            u32TimeOutCount--;
        }

        /* Disable Timer for BPWM1 channel 0 */
        BPWM_ForceStop(BPWM1, BPWM_CH_0_MASK);

        /* Disable BPWM Output path for BPWM1 channel 0 */
        BPWM_DisableOutput(BPWM1, BPWM_CH_0_MASK);

        /*---------------------------------------------------------------------------------------------------------*/
        /* Stop BPWM0 channel 0 (Recommended procedure method 1)                                                    */
        /* Set BPWM Timer loaded value(Period) as 0. When BPWM internal counter(CNT) reaches to 0, disable BPWM Timer */
        /*---------------------------------------------------------------------------------------------------------*/

        /* Disable BPWM0 NVIC */
        //NVIC_DisableIRQ(BPWM0_IRQn);

        /* Set loaded value as 0 for BPWM0 channel 0 */
        BPWM_Stop(BPWM0, BPWM_CH_0_MASK);

        /* setup timeout */
        u32TimeOutCount = SystemCoreClock;

        /* Wait until BPWM0 channel 0 current counter reach to 0 */
        while ((BPWM0->CNT & BPWM_CNT_CNT_Msk) != 0)
        {
            if(u32TimeOutCount == 0)
            {
                printf("BPWM encounters some errors, please check it. \n");
                while(1);
            }
            u32TimeOutCount--;
        }

        /* Disable Timer for BPWM0 channel 0 */
        BPWM_ForceStop(BPWM0, BPWM_CH_0_MASK);

        /* Disable Capture Function and Capture Input path for BPWM0 channel 0 */
        BPWM_DisableCapture(BPWM0, BPWM_CH_0_MASK);

        /* Clear Capture Interrupt flag for BPWM0 channel 0 */
        BPWM_ClearCaptureIntFlag(BPWM0, 0, BPWM_CAPTURE_INT_FALLING_LATCH);
    }
}


/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

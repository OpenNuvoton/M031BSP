/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/12/25 10:23a $
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
 * @brief       PWM0 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle BPWM0 interrupt event
 */
void BPWM0_IRQHandler(void)
{
    uint32_t u32CapIntFlag;

    u32CapIntFlag = BPWM0->CAPIF;

    if(u32CapIntFlag & BPWM_CAPIF_CAPFIF0_Msk)
    {
        BPWM0->CAPIF = BPWM_CAPIF_CAPFIF0_Msk;
    }
}

/*--------------------------------------------------------------------------------------*/
/* Capture function to calculate the input waveform information                         */
/* u32Count[4] : Keep the internal counter value when input signal rising / falling     */
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
void CalPeriodTime()
{
    uint16_t u32Count[4];
    uint32_t u32i;
    uint16_t u16RisingTime, u16FallingTime, u16HighPeroid, u16LowPeroid, u16TotalPeroid;
    uint32_t u32TimeOutCount;

    /* Clear Capture Falling Indicator (Time A) */
    BPWM0->CAPIF = BPWM_CAPIF_CAPFIF0_Msk;

    /* setup timeout */
    u32TimeOutCount = SystemCoreClock;

    /* Wait for Capture Falling Indicator  */
    while((BPWM0->CAPIF & BPWM_CAPIF_CAPFIF0_Msk) == 0)
    {
        if(u32TimeOutCount == 0)
        {
            printf("\nSomething is wrong, please check if pin connection is correct. \n");
            while(1);
        }
        u32TimeOutCount--;
    }

    /* Clear Capture Falling Indicator (Time B)*/
    BPWM0->CAPIF = BPWM_CAPIF_CAPFIF0_Msk;

    u32i = 0;

    while(u32i < 4)
    {
        /* Wait for Capture Falling Indicator */
        while((BPWM0->CAPIF & BPWM_CAPIF_CAPFIF0_Msk) == 0);

        /* Clear Capture Falling and Rising Indicator */
        BPWM0->CAPIF = BPWM_CAPIF_CAPFIF0_Msk | BPWM_CAPIF_CAPRIF0_Msk;

        /* Get Capture Falling Latch Counter Data */
        u32Count[u32i++] = BPWM_GET_CAPTURE_FALLING_DATA(BPWM0, 0);

        /* Wait for Capture Rising Indicator */
        while((BPWM0->CAPIF & BPWM_CAPIF_CAPFIF0_Msk) == 0);

        /* Clear Capture Rising Indicator */
        BPWM0->CAPIF = BPWM_CAPIF_CAPRIF0_Msk;

        /* Get Capture Rising Latch Counter Data */
        u32Count[u32i++] = BPWM_GET_CAPTURE_RISING_DATA(BPWM0, 0);
    }

    u16RisingTime = u32Count[1];

    u16FallingTime = u32Count[0];

    u16HighPeroid = u32Count[1] - u32Count[2];

    u16LowPeroid = 0x10000 - u32Count[1];

    u16TotalPeroid = 0x10000 - u32Count[2];

    printf("\nPWM generate: \nHigh Period=14999 ~ 15001, Low Period=34999 ~ 35001, Total Period=49999 ~ 50001\n");
    printf("\nCapture Result: Rising Time = %d, Falling Time = %d \nHigh Period = %d, Low Period = %d, Total Period = %d.\n\n",
           u16RisingTime, u16FallingTime, u16HighPeroid, u16LowPeroid, u16TotalPeroid);

    if ((u16HighPeroid < 14399) || (u16HighPeroid > 14401) || (u16LowPeroid < 33599) || (u16LowPeroid > 33601) || (u16TotalPeroid < 47999) || (u16TotalPeroid > 48001))
        printf("Capture Test Fail!!\n");
    else
        printf("Capture Test Pass!!\n");
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Switch UART0 clock source to HIRC */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART0SEL_Msk) | CLK_CLKSEL1_UART0SEL_HIRC;

    /* Enable PLL and Set PLL frequency */
//    CLK->PLLCTL = PLLCTL_SETTING;

//    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));
//    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_PLL;

    /* Enable BPWM0 and BPWM1 module clock */
    CLK->APBCLK1 |= (CLK_APBCLK1_BPWM0CKEN_Msk | CLK_APBCLK1_BPWM1CKEN_Msk);

    /*---------------------------------------------------------------------------------------------------------*/
    /* BPWM clock frequency configuration                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* BPWM clock frequency can be set equal or double to HCLK by choosing case 1 or case 2 */
    /* case 1.BPWM clock frequency is set equal to HCLK: select BPWM module clock source as PCLK */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_BPWM0SEL_Msk) | CLK_CLKSEL2_BPWM0SEL_PCLK0;
    CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_BPWM1SEL_Msk) | CLK_CLKSEL2_BPWM1SEL_PCLK1;

    /* case 2.BPWM clock frequency is set double to HCLK: select BPWM module clock source as PLL */
    //CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_BPWM0SEL_Msk) | CLK_CLKSEL2_BPWM0SEL_PLL;
    //CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_BPWM1SEL_Msk) | CLK_CLKSEL2_BPWM1SEL_PLL;
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable UART clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Reset BPWM0 and BPWM1 module */
    SYS->IPRST2 |= (SYS_IPRST2_BPWM0RST_Msk | SYS_IPRST2_BPWM1RST_Msk);
    SYS->IPRST2 &= ~(SYS_IPRST2_BPWM0RST_Msk | SYS_IPRST2_BPWM1RST_Msk);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set PA multi-function pin for BPWM0 Channel 0 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA0MFP_Msk) | SYS_GPA_MFPL_PA0MFP_BPWM0_CH0;

    /* Set PB.11 multi-function pin for BPWM1 Channel 0 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB11MFP_Msk) | SYS_GPB_MFPH_PB11MFP_BPWM1_CH0;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32TimeOutCount;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("+------------------------------------------------------------------------+\n");
    printf("|                          BPWM Driver Sample Code                        |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use BPWM0 channel 0 to capture\n  the signal from BPWM1 channel 0.\n");
    printf("  I/O configuration:\n");
    printf("    BPWM0_CH0 (PA.0 BPWM0 channel 0) <--> BPWM1_CH0(PB.11 BPWM1 channel 0)\n\n");
    printf("Use BPWM0 Channel 0(PA.0) to capture the BPWM1 Channel 0(PB.11) Waveform\n");

    while(1)
    {
        printf("\n\nPress any key to start BPWM Capture Test\n");
        getchar();

        /*--------------------------------------------------------------------------------------*/
        /* Set the BPWM1 Channel 0 as BPWM output function.                                       */
        /*--------------------------------------------------------------------------------------*/
        /* Assume BPWM1 output frequency is 250 Hz and duty ratio is 30%, user can calculate BPWM settings by follows.
           duty ratio = (CMR+1)/(CNR+1)
           cycle time = CNR+1
           High level = CMR+1
           BPWM clock source frequency = __HIRC = 48,000,000
           (CNR+1) = BPWM clock source frequency/prescaler/clock source divider/BPWM output frequency
                   = 48,000,000/4/1/250 = 48,000
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 47999
           duty ratio = 30% ==> (CMR+1)/(CNR+1) = 30%
           CMR = 14400
           Prescale value is 1 : prescaler= 2
           Clock divider is BPWM_CSR_DIV1 : clock divider =1
        */

        /*Set counter as down count*/
        BPWM1->CTL1 = (BPWM1->CTL1 & ~BPWM_CTL1_CNTTYPE0_Msk) | 0x1;

        /*Set BPWM Timer clock prescaler*/
        BPWM_SET_PRESCALER(BPWM1, 0, 3); // Divided by 4

        /*Set BPWM Timer duty*/
        BPWM_SET_CMR(BPWM1, 0, 14400);

        /*Set BPWM Timer period*/
        BPWM_SET_CNR(BPWM1, 0, 47999);

        /* Set waveform generation */
        BPWM1->WGCTL0 = 0x00010000;
        BPWM1->WGCTL1 = 0x00020000;

        /* Enable BPWM Output path for BPWM1 channel 0 */
        BPWM1->POEN |= BPWM_CH_0_MASK;

        /* Enable Timer for BPWM1 channel 0 */
        BPWM1->CNTEN |= BPWM_CH_0_MASK;

        /*--------------------------------------------------------------------------------------*/
        /* Set the BPWM0 channel 0 for capture function                                          */
        /*--------------------------------------------------------------------------------------*/

        /* If input minimum frequency is 250Hz, user can calculate capture settings by follows.
           Capture clock source frequency = __HIRC = 48,000,000 in the sample code.
           (CNR+1) = Capture clock source frequency/prescaler/clock source divider/minimum input frequency
                   = 48,000,000/4/1/250 = 48,000
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 0xFFFF
           (Note: In capture mode, user should set CNR to 0xFFFF to increase capture frequency range.)
        */

        /*Set counter as down count*/
        BPWM0->CTL1 = (BPWM0->CTL1 & ~BPWM_CTL1_CNTTYPE0_Msk) | (0x1 << BPWM_CTL1_CNTTYPE0_Pos);

        /*Set BPWM0 channel 0 Timer clock prescaler*/
        BPWM_SET_PRESCALER(BPWM0, 0, 3); // Divided by 4

        /*Set BPWM0 channel 0 Timer period*/
        BPWM_SET_CNR(BPWM0, 0, 0xFFFF);

        /* Enable capture falling edge interrupt for BPWM0 channel 0 */
        //BPWM0->CAPIEN |= BPWM_CAPIEN_CAPFIEN0_Msk;

        /* Enable capture function */
        BPWM0->CAPCTL |= BPWM_CAPCTL_CAPEN0_Msk;

        /* Enable falling capture reload */
        BPWM0->CAPCTL |= BPWM_CAPCTL_FCRLDEN0_Msk;

        //NVIC_EnableIRQ(BPWM0_IRQn);

        /* setup timeout */
        u32TimeOutCount = SystemCoreClock;

        // Start
        BPWM0->CNTEN |= BPWM_CNTEN_CNTEN0_Msk;

        /* Wait until BPWM0 channel 0 Timer start to count */
        while((BPWM0->CNT) == 0)
        {
            if(u32TimeOutCount == 0)
            {
                printf("BPWM encounters some errors, please check it. \n");
                while(1);
            }
            u32TimeOutCount--;
        }

        /* Enable capture input path for BPWM0 channel 0 */
        BPWM0->CAPINEN |= BPWM_CAPINEN_CAPINEN0_Msk;

        /* Capture the Input Waveform Data */
        CalPeriodTime();
        /*---------------------------------------------------------------------------------------------------------*/
        /* Stop BPWM1 channel 0 (Recommended procedure method 1)                                                    */
        /* Set BPWM Timer loaded value(Period) as 0. When BPWM internal counter(CNT) reaches to 0, disable BPWM Timer */
        /*---------------------------------------------------------------------------------------------------------*/

        /* Set BPWM1 channel 0 loaded value as 0 */
        BPWM1->PERIOD = 0;

        /* Wait until BPWM1 channel 0 Timer Stop */
        while((BPWM1->CNT & BPWM_CNT_CNT_Msk) != 0)
        {
            if(u32TimeOutCount == 0)
            {
                printf("BPWM encounters some errors, please check it. \n");
                while(1);
            }
            u32TimeOutCount--;
        }

        /* Disable Timer for BPWM1 channel 0 */
        BPWM1->CNTEN &= ~BPWM_CNTEN_CNTEN0_Msk;

        /* Disable BPWM Output path for BPWM1 channel 0 */
        BPWM1->POEN &= ~BPWM_CH_0_MASK;

        /*---------------------------------------------------------------------------------------------------------*/
        /* Stop BPWM0 channel 0 (Recommended procedure method 1)                                                    */
        /* Set BPWM Timer loaded value(Period) as 0. When BPWM internal counter(CNT) reaches to 0, disable BPWM Timer */
        /*---------------------------------------------------------------------------------------------------------*/

        /* Disable BPWM1 NVIC */
        //NVIC_DisableIRQ(BPWM0_IRQn);

        /* Set loaded value as 0 for BPWM0 channel 0 */
        BPWM0->PERIOD = 0;

        /* Wait until BPWM0 channel 0 current counter reach to 0 */
        while((BPWM0->CNT & BPWM_CNT_CNT_Msk) != 0)
        {
            if(u32TimeOutCount == 0)
            {
                printf("BPWM encounters some errors, please check it. \n");
                while(1);
            }
            u32TimeOutCount--;
        }

        /* Disable Timer for BPWM0 channel 0 */
        BPWM0->CNTEN &= ~BPWM_CNTEN_CNTEN0_Msk;

        /* Disable Capture Function and Capture Input path for  BPWM0 channel 0*/
        BPWM0->CAPCTL &= ~BPWM_CAPCTL_CAPEN0_Msk;
        BPWM0->CAPINEN &= ~BPWM_CAPINEN_CAPINEN0_Msk;

        /* Clear Capture Interrupt flag for BPWM0 channel 0 */
        BPWM0->CAPIF = BPWM_CAPIF_CAPFIF0_Msk;
    }
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/

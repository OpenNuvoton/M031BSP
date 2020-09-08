/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 8 $
 * $Date: 18/07/19 2:22p $
 * @brief    Demonstrate how to use PWM output waveform.
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

    /* Enable BPWM0 module clock */
    CLK->APBCLK1 |= CLK_APBCLK1_BPWM0CKEN_Msk;

    /* BPWM clock frequency can be set equal or double to HCLK by choosing case 1 or case 2 */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_BPWM0SEL_Msk) | CLK_CLKSEL2_BPWM0SEL_PCLK0;

    /* Enable UART clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Reset BPWM0 module */
    SYS->IPRST2 |= SYS_IPRST2_BPWM0RST_Msk;
    SYS->IPRST2 &= ~SYS_IPRST2_BPWM0RST_Msk;

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set multi-function pins for BPWM0 Channel0~5 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA0MFP_Msk)) | SYS_GPA_MFPL_PA0MFP_BPWM0_CH0;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA1MFP_Msk)) | SYS_GPA_MFPL_PA1MFP_BPWM0_CH1;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA2MFP_Msk)) | SYS_GPA_MFPL_PA2MFP_BPWM0_CH2;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA3MFP_Msk)) | SYS_GPA_MFPL_PA3MFP_BPWM0_CH3;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF5MFP_Msk)) | SYS_GPF_MFPL_PF5MFP_BPWM0_CH4;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF4MFP_Msk)) | SYS_GPF_MFPL_PF4MFP_BPWM0_CH5;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("BPWM0 clock is from %s\n", (CLK->CLKSEL2 & CLK_CLKSEL2_BPWM0SEL_Msk) ? "PCLK" : "PLL");
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          BPWM Driver Sample Code                       |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output waveform with BPWM0 channel 0~5.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: BPWM0_CH0(PA.0), BPWM0_CH1(PA.1), BPWM0_CH2(PA.2), BPWM0_CH3(PA.3), BPWM0_CH4(PF.5), BPWM0_CH5(PF.4)\n");

    /* BPWM0 channel 0~5 frequency and duty configuration are as follows */

    /* Set PBWM to up counter type(edge aligned) */
    BPWM0->CTL1 &= ~BPWM_CTL1_CNTTYPE0_Msk;

    /*
      Configure BPWM0 channel 0 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 48 MHz / (1 * (199 + 1)) = 240000 Hz
      Duty ratio = (180) / (200) = 90%
    */
    /* BPWM0 channel 0 frequency and duty configuration */
    /* Set PWM Timer clock prescaler */
    BPWM_SET_PRESCALER(BPWM0, 0, 0); /* Divided by 1 */
    /* Set PWM Timer duty */
    BPWM_SET_CMR(BPWM0, 0, 180);
    /* Set PWM Timer period */
    BPWM_SET_CNR(BPWM0, 0, 199);

    /*
      Configure BPWM0 channel 1 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 48 MHz / (1 * (199 + 1)) = 240000 Hz
      Duty ratio = (160) / (200) = 80%
    */
    /* BPWM0 channel 1 frequency and duty configuration */
    /* PWM Timer clock prescaler of Channel 1 is share with Channel 0 */
    /* Set PWM Timer duty */
    BPWM_SET_CMR(BPWM0, 1, 160);
    /* Set PWM Timer period */
    BPWM_SET_CNR(BPWM0, 1, 199);

    /*
      Configure BPWM0 channel 2 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 48 MHz / (1 * (199 + 1)) = 240000 Hz
      Duty ratio = (150) / (200) = 75%
    */
    /* BPWM0 channel 2 frequency and duty configuration */
    /* Set PWM Timer clock prescaler */
    BPWM_SET_PRESCALER(BPWM0, 2, 0); /* Divided by 1 */
    /* Set PWM Timer duty */
    BPWM_SET_CMR(BPWM0, 2, 150);
    /* Set PWM Timer period */
    BPWM_SET_CNR(BPWM0, 2, 199);

    /*
      Configure BPWM0 channel 3 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 48 MHz / (1 * (199 + 1)) = 240000 Hz
      Duty ratio = (140) / (200) = 70%
    */
    /* BPWM0 channel 3 frequency and duty configuration */
    /* PWM Timer clock prescaler of Channel 3 is share with Channel 2 */
    /* Set PWM Timer duty */
    BPWM_SET_CMR(BPWM0, 3, 140);
    /* Set PWM Timer period */
    BPWM_SET_CNR(BPWM0, 3, 199);

    /*
      Configure BPWM0 channel 4 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 48 MHz / (1 * (199 + 1)) = 240000 Hz
      Duty ratio = (120) / (200) = 60%
    */
    /* BPWM0 channel 4 frequency and duty configuration */
    /* Set PWM Timer clock prescaler */
    BPWM_SET_PRESCALER(BPWM0, 4, 0); /* Divided by 1 */
    /* Set PWM Timer duty */
    BPWM_SET_CMR(BPWM0, 4, 120);
    /* Set PWM Timer period */
    BPWM_SET_CNR(BPWM0, 4, 199);

    /*
      Configure BPWM0 channel 5 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 48 MHz / (1 * (199 + 1)) = 240000 Hz
      Duty ratio = (100) / (200) = 50%
    */
    /* BPWM0 channel 5 frequency and duty configuration */
    /* PWM Timer clock prescaler of Channel 5 is share with Channel 4 */
    /* Set PWM Timer duty */
    BPWM_SET_CMR(BPWM0, 5, 100);
    /* Set PWM Timer period */
    BPWM_SET_CNR(BPWM0, 5, 199);

    /* Set waveform generation */
    BPWM_SET_OUTPUT_LEVEL(BPWM0, 0x3F, BPWM_OUTPUT_HIGH, BPWM_OUTPUT_LOW, BPWM_OUTPUT_NOTHING, BPWM_OUTPUT_NOTHING);

    /* Enable output of BPWM0 channel 0 ~ 5 */
    BPWM0->POEN |= 0x3F;

    /* Enable counter synchronous start function for BPWM0 channel 0~5 */
    BPWM0->SSCTL = 0x3F | BPWM_SSCTL_SSRC_BPWM0;

    /* Start BPWM0 counter */
    BPWM0->CNTEN = 0x3F;

    printf("Press any key to stop.\n");
    getchar();

    /* Stop BPWM0 counter */
    BPWM0->CNTEN &= ~0x3F;

    printf("Done.\n");

    while(1);

}

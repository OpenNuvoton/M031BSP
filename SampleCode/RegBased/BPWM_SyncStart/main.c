/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 7 $
 * $Date: 18/07/19 2:23p $
 * @brief    Demonstrate how to use PWM counter synchronous start function.
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
 * @details     ISR to handle PWM0 interrupt event
 */
void PWM0_IRQHandler(void)
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

    /* Enable BPWM0 and BPWM1 module clock */
    CLK->APBCLK1 |= CLK_APBCLK1_BPWM0CKEN_Msk;
	CLK->APBCLK1 |= CLK_APBCLK1_BPWM1CKEN_Msk;	

    /* BPWM clock frequency can be set equal or double to HCLK by choosing case 1 or case 2 */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_BPWM0SEL_Msk) | CLK_CLKSEL2_BPWM0SEL_PCLK0;
    CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_BPWM1SEL_Msk) | CLK_CLKSEL2_BPWM1SEL_PCLK1;

    /* Enable UART clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Reset BPWM0 and BPWM1 modules */
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


    /* Set multi-function pins for BPWM0 Channel0~5 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA0MFP_Msk)) | SYS_GPA_MFPL_PA0MFP_BPWM0_CH0;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA1MFP_Msk)) | SYS_GPA_MFPL_PA1MFP_BPWM0_CH1;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA2MFP_Msk)) | SYS_GPA_MFPL_PA2MFP_BPWM0_CH2;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA3MFP_Msk)) | SYS_GPA_MFPL_PA3MFP_BPWM0_CH3;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF5MFP_Msk)) | SYS_GPF_MFPL_PF5MFP_BPWM0_CH4;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF4MFP_Msk)) | SYS_GPF_MFPL_PF4MFP_BPWM0_CH5;

    /* Set multi-function pins for BPWM1 Channel0~5 */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk)) | SYS_GPF_MFPL_PF3MFP_BPWM1_CH0;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk)) | SYS_GPF_MFPL_PF2MFP_BPWM1_CH1;
    SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA12MFP_Msk)) | SYS_GPA_MFPH_PA12MFP_BPWM1_CH2;
    SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA13MFP_Msk)) | SYS_GPA_MFPH_PA13MFP_BPWM1_CH3;
    SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA14MFP_Msk)) | SYS_GPA_MFPH_PA14MFP_BPWM1_CH4;
    SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA15MFP_Msk)) | SYS_GPA_MFPH_PA15MFP_BPWM1_CH5;
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
    printf("BPWM1 clock is from %s\n", (CLK->CLKSEL2 & CLK_CLKSEL2_BPWM1SEL_Msk) ? "PCLK" : "PLL");
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          BPWM Driver Sample Code                       |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output waveform with BPWM0 and BPWM1 channel 0~5 at the same time.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin:  \n");
    printf("		BPWM0_CH0(PA.0), BPWM0_CH1(PA.1), BPWM0_CH2(PA.2), BPWM0_CH3(PA.3), BPWM0_CH4(PF.5), BPWM0_CH5(PF.4)\n");
    printf("		BPWM1_CH0(PF.3), BPWM1_CH1(PF.2), BPWM1_CH2(PA.12), BPWM1_CH3(PA.13), BPWM1_CH4(PA.14), BPWM1_CH5(PA.15)\n");

    /* BPWM0 channel 0~5 frequency and duty configuration are as follows */
    /*
      Configure BPWM0 channel 0 init period and duty(up counter type).
      Period is HIRC / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 48 MHz / (2 * (199 + 1)) = 120,000 Hz
      Duty ratio = (100) / (199 + 1) = 50%
    */

    /* Set BPWM to up counter type(edge aligned) */
    BPWM0->CTL1 &= ~BPWM_CTL1_CNTTYPE0_Msk;
    BPWM1->CTL1 &= ~BPWM_CTL1_CNTTYPE0_Msk;	

    /* BPWM0 frequency and duty configuration */
    BPWM_SET_PRESCALER(BPWM0, 0, 0); /* Divided by 1 */
    BPWM_SET_CMR(BPWM0, 0, 200);
    BPWM_SET_CNR(BPWM0, 0, 399);
	
    BPWM_SET_CMR(BPWM0, 1, 200);
    BPWM_SET_CNR(BPWM0, 1, 399);
	
    BPWM_SET_CMR(BPWM0, 2, 200);
    BPWM_SET_CNR(BPWM0, 2, 399);
	
    BPWM_SET_CMR(BPWM0, 3, 200);
    BPWM_SET_CNR(BPWM0, 3, 399);
	
    BPWM_SET_CMR(BPWM0, 4, 200);
    BPWM_SET_CNR(BPWM0, 4, 399);
	
    BPWM_SET_CMR(BPWM0, 5, 200);
    BPWM_SET_CNR(BPWM0, 5, 399);

    /* BPWM0 frequency and duty configuration */
    BPWM_SET_PRESCALER(BPWM1, 0, 0); /* Divided by 1 */
    BPWM_SET_CMR(BPWM1, 0, 200);
    BPWM_SET_CNR(BPWM1, 0, 399);
	
    BPWM_SET_CMR(BPWM1, 1, 200);
    BPWM_SET_CNR(BPWM1, 1, 399);
	
    BPWM_SET_CMR(BPWM1, 2, 200);
    BPWM_SET_CNR(BPWM1, 2, 399);
	
    BPWM_SET_CMR(BPWM1, 3, 200);
    BPWM_SET_CNR(BPWM1, 3, 399);
	
    BPWM_SET_CMR(BPWM1, 4, 200);
    BPWM_SET_CNR(BPWM1, 4, 399);
	
    BPWM_SET_CMR(BPWM1, 5, 200);
    BPWM_SET_CNR(BPWM1, 5, 399);

    /* Set waveform generation */
    BPWM_SET_OUTPUT_LEVEL(BPWM0, 0x3F, BPWM_OUTPUT_HIGH, BPWM_OUTPUT_LOW, BPWM_OUTPUT_NOTHING, BPWM_OUTPUT_NOTHING);
    BPWM_SET_OUTPUT_LEVEL(BPWM1, 0x3F, BPWM_OUTPUT_HIGH, BPWM_OUTPUT_LOW, BPWM_OUTPUT_NOTHING, BPWM_OUTPUT_NOTHING);	

    /* Enable output of BPWM channel 0 ~ 5 */
    BPWM0->POEN |= 0x3F;
    BPWM1->POEN |= 0x3F;	

    /* Enable counter synchronous start function by BPWM0 */
    BPWM0->SSCTL = 0x01 | BPWM_SSCTL_SSRC_BPWM0;
    BPWM1->SSCTL = 0x01 | BPWM_SSCTL_SSRC_BPWM0;
	
    printf("Press any key to start.\n");
    getchar();

    /* Trigger BPWM counter synchronous start by BPWM0 */
    BPWM0->SSTRG = BPWM_SSTRG_CNTSEN_Msk;
    BPWM1->SSTRG = BPWM_SSTRG_CNTSEN_Msk;

    printf("Done.\n");
    while(1);

}

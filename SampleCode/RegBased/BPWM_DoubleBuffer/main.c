/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 2 $
 * $Date: 14/12/25 10:23a $
 * @brief    Change duty cycle and period of output waveform by BPWM Double Buffer function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define PLLCON_SETTING  CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK       50000000

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
    static int toggle = 0;

    // Update BPWM0 channel 0 period and duty
    if(toggle == 0)
    {
        BPWM_SET_CNR(BPWM0, 0, 99);
        BPWM_SET_CMR(BPWM0, 0, 39);
    }
    else
    {
        BPWM_SET_CNR(BPWM0, 0, 399);
        BPWM_SET_CMR(BPWM0, 0, 199);
    }
    toggle ^= 1;
    // Clear channel 0 period interrupt flag
    BPWM0->INTSTS = BPWM_INTSTS_PIF0_Msk;
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

    /* Reset BPWM0 and BPWM1 module */
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

    /* Set PA multi-function pin for BPWM0 Channel 0 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA0MFP_Msk) | SYS_GPA_MFPL_PA0MFP_BPWM0_CH0;	

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
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          BPWM Driver Sample Code                        |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use BPWM0 channel 0 to output waveform\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: BPWM0 channel 0(PC.0)\n");
    printf("\nUse double buffer feature.\n");

    /*
        BPWM0 channel 0 waveform of this sample shown below:

        |<-        CNR + 1  clk     ->|  CNR + 1 = 399 + 1 CLKs
                       |<-CMR+1 clk ->|  CMR + 1 = 199 + 1 CLKs
                                      |<-   CNR + 1  ->|  CNR + 1 = 99 + 1 CLKs
                                               |<CMR+1>|  CMR + 1 = 39 + 1 CLKs
      __                ______________          _______
        |______200_____|     200      |____60__|   40  |_____BPWM waveform

    */

    /*
      Configure BPWM0 channel 0 init period and duty.
      Period is PCLK / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 48 MHz / (1 * (399 + 1)) = 120000 Hz
      Duty ratio = (200) / (399 + 1) = 50%
    */


    // set BPWM to down count type(edge aligned)
    BPWM0->CTL1 &= ~BPWM_CTL1_CNTTYPE0_Msk;
    BPWM0->CTL1 |= 0x1;

    /*Set BPWM Timer clock prescaler*/
    BPWM_SET_PRESCALER(BPWM0, 0, 0); // Divided by 1

    /*Set BPWM Timer duty*/
    BPWM_SET_CMR(BPWM0, 0, 199);

    /*Set BPWM Timer period*/
    BPWM_SET_CNR(BPWM0, 0, 399);

    /* Set waveform generation */
    BPWM0->WGCTL0 = 0x10000;
    BPWM0->WGCTL1 = 0x20000;

    // Enable output of BPWM0 channel 0
    BPWM0->POEN |= BPWM_POEN_POEN0_Msk;

    // Enable BPWM0 channel 0 period interrupt, use channel 0 to measure time.
    BPWM0->INTEN = (BPWM0->INTEN & ~BPWM_INTEN_PIEN0_Msk) | BPWM_INTEN_PIEN0_Msk;
    NVIC_EnableIRQ(BPWM0_IRQn);

    // Start
    BPWM0->CNTEN |= BPWM_CNTEN_CNTEN0_Msk;

    while(1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/

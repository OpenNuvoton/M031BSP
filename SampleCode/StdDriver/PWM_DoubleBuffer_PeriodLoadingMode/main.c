/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 9 $
 * $Date: 18/07/09 10:29a $
 * @brief
 *           Change duty cycle and period of output waveform by PWM Double Buffer function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define PLL_CLOCK       96000000

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
    static int toggle = 0;

    /* Update PWM0 channel 0 period and duty */
    if(toggle == 0)
    {
        PWM_SET_CNR(PWM0, 0, 99);
        PWM_SET_CMR(PWM0, 0, 60);
    }
    else
    {
        PWM_SET_CNR(PWM0, 0, 399);
        PWM_SET_CMR(PWM0, 0, 200);
    }
    toggle ^= 1;
    /* Clear channel 0 period interrupt flag */
    PWM_ClearPeriodIntFlag(PWM0, 0);
}

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

    /* Select HIRC as the clock source of UART0 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Set core clock as PLL_CLOCK from PLL (no PLL in rev. B & C) */
//    CLK_SetCoreClock(PLL_CLOCK);

    /* Waiting for PLL clock ready */
//    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable PWM0 module clock */
    CLK_EnableModuleClock(PWM0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* PWM clock frequency configuration                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Select HCLK clock source as PLL and and HCLK clock divider as 2 */
//    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* PWM clock frequency can be set equal or double to HCLK by choosing case 1 or case 2 */
    /* case 1.PWM clock frequency is set equal to HCLK: select PWM module clock source as PCLK */
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL2_PWM0SEL_PCLK0,(uint32_t)NULL);

    /* case 2.PWM clock frequency is set double to HCLK: select PWM module clock source as PLL */
//    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL2_PWM0SEL_PLL, NULL);
    /*---------------------------------------------------------------------------------------------------------*/

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set PB multi-function pins for PWM0 Channel 0 */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB5MFP_Msk)) |
                    SYS_GPB_MFPL_PB5MFP_PWM0_CH0;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
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

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("PWM0 clock is from %s\n", (CLK->CLKSEL2 & CLK_CLKSEL2_PWM0SEL_Msk) ? "PCLK" : "PLL");
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          PWM Driver Sample Code                        |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use PWM0 channel 0 to output waveform\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: PWM0 channel 0 (PB.5)\n");
    printf("\nUse double buffer feature.\n");

    /*
        PWM0 channel 0 waveform of this sample shown below (down counter type):

        |<-        CNR + 1  clk     ->|  CNR + 1 = 399 + 1 CLKs
                       |<-  CMR clk ->|  CMR = 200 CLKs
                                      |<-   CNR + 1  ->|  CNR + 1 = 99 + 1 CLKs
                                               |<-CMR->|  CMR = 60 CLKs

         ______________                _______          ____
        |      200     |_____200______|   40  |____60__|     PWM waveform

    */

    /*
      Configure PWM0 channel 0 init period and duty(up counter type).
      Period is PCLK / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 48 MHz / (2 * (199 + 1)) = 120,000 Hz
      Duty ratio = (100) / (199 + 1) = 50%
    */
    /* PWM0 channel 0 frequency is 120,000 Hz, duty 50%, */
    PWM_ConfigOutputChannel(PWM0, 0, 120000, 50);

    /* Enable output of PWM0 channel 0 */
    PWM_EnableOutput(PWM0, PWM_CH_0_MASK);

    /* Enable PWM0 channel 0 period interrupt, use channel 0 to measure time. */
    PWM_EnablePeriodInt(PWM0, 0, 0);
    NVIC_EnableIRQ(PWM0_IRQn);

    /* ZeroLevel: High, CmpUpLevel: nothing, PeriodLevel: nothing, CmpDownLevel: Low */
    PWM_SET_OUTPUT_LEVEL(PWM0, PWM_CH_0_MASK, PWM_OUTPUT_NOTHING, PWM_OUTPUT_NOTHING, PWM_OUTPUT_HIGH, PWM_OUTPUT_LOW);

    /* Start */
    PWM_Start(PWM0, PWM_CH_0_MASK);

    while(1);
}

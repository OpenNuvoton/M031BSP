/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 11 $
 * $Date: 18/07/19 2:22p $
 * @brief   Demonstrate how to use PWM Dead Zone function.
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
    static uint32_t cnt;
    static uint32_t out;

    /* Channel 0 frequency is 600Hz, every 1 second enter this IRQ handler 600 times. */
    if(++cnt == 600)
    {
        if(out)
            PWM0->POEN |= (0xF);
        else
            PWM0->POEN &= ~(0xF);
        out ^= 1;
        cnt = 0;
    }
    /* Clear channel 0 period interrupt flag */
    PWM0->INTSTS0 = PWM_INTSTS0_PIF0_Msk;
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

    /* Enable PLL and Set PLL frequency (only rev. D & E support PLL) */
//    CLK->PLLCTL = PLLCTL_SETTING;

//    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));
//    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_PLL;

    /* Enable PWM0 module clock */
    CLK->APBCLK1 |= CLK_APBCLK1_PWM0CKEN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* PWM clock frequency configuration                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Select HCLK clock divider as 2 */
//    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(2);

    /* PWM clock frequency can be set equal or double to HCLK by choosing case 1 or case 2 */
    /* case 1.PWM clock frequency is set equal to HCLK: select PWM module clock source as PCLK */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_PWM0SEL_Msk) | CLK_CLKSEL2_PWM0SEL_PCLK0;

    /* case 2.PWM clock frequency is set double to HCLK: select PWM module clock source as PLL */
    //CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_PWM0SEL_Msk) | CLK_CLKSEL2_PWM0SEL_PLL;
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable UART clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Reset PWM0 module */
    SYS->IPRST2 |= SYS_IPRST2_PWM0RST_Msk;
    SYS->IPRST2 &= ~SYS_IPRST2_PWM0RST_Msk;

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set PB multi-function pins for PWM0 Channel 0 ~ 3 */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB5MFP_Msk |
                                       SYS_GPB_MFPL_PB4MFP_Msk |
                                       SYS_GPB_MFPL_PB3MFP_Msk |
                                       SYS_GPB_MFPL_PB2MFP_Msk)) |
                    (SYS_GPB_MFPL_PB5MFP_PWM0_CH0 |
                     SYS_GPB_MFPL_PB4MFP_PWM0_CH1 |
                     SYS_GPB_MFPL_PB3MFP_PWM0_CH2 |
                     SYS_GPB_MFPL_PB2MFP_PWM0_CH3);
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

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("PWM0 clock is from %s\n", (CLK->CLKSEL2 & CLK_CLKSEL2_PWM0SEL_Msk) ? "PCLK" : "PLL");
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          PWM Driver Sample Code                        |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output PWM0 channel 0~3 with different\n");
    printf("  frequency and duty, enable dead zone function of all PWM0 pairs.\n");
    printf("  And also enable/disable PWM output every 1 second.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: PWM0_CH0(PB.5), PWM0_CH1(PB.4), PWM0_CH2(PB.3), PWM0_CH3(PB.2)\n");

    /*---------------------------------------------------------------------------------------------------------*/
    /* Up counter type                                                                                         */
    /*---------------------------------------------------------------------------------------------------------*/
    /* PWM0 channel 0 frequency is 600Hz, duty 30% */
    /* Assume PWM output frequency is 600Hz and duty ratio is 30%, user can calculate PWM settings by follows.
       up counter type:
       duty ratio = (CMR)/(CNR+1)
       cycle time = CNR+1
       High level = CMR
       PWM clock source frequency = PLL = 48000000
       (CNR+1) = PWM clock source frequency/prescaler/PWM output frequency
               = 48000000/2/600 = 40000
       (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
       CNR = 39999
       duty ratio = 30% ==> (CMR)/(CNR+1) = 30% ==> CMR = (CNR+1)*0.3 = 40000*30/100
       CMR = 12000
       Prescale value is 1 : prescaler= 2
    */

    /* Set Pwm mode as complementary mode */
    PWM_ENABLE_COMPLEMENTARY_MODE(PWM0);

    /* Set PWM Timer clock prescaler */
    PWM_SET_PRESCALER(PWM0, 0, 1); /* Divided by 2 */

    /* Set PWM Timer duty */
    PWM_SET_CMR(PWM0, 0, 12000);
    PWM_SET_CMR(PWM0, 1, 12000);

    /* Set PWM Timer period */
    PWM_SET_CNR(PWM0, 0, 39999);

    /* Set waveform generation */
    PWM_SET_OUTPUT_LEVEL(PWM0, 0xF, PWM_OUTPUT_HIGH, PWM_OUTPUT_LOW, PWM_OUTPUT_NOTHING, PWM_OUTPUT_NOTHING);

    /* Enable and configure dead zone */
    SYS_UnlockReg();
    PWM0->DTCTL[0] = 400;
    PWM0->DTCTL[0] |= PWM_DTCTL0_1_DTEN_Msk;
    SYS_LockReg();

    /* PWM0 channel 2 frequency is 1200Hz, duty 50% */
    /* Assume PWM output frequency is 1800Hz and duty ratio is 50%, user can calculate PWM settings by follows.
       up counter type:
       duty ratio = (CMR)/(CNR+1)
       cycle time = CNR+1
       High level = CMR
       PWM clock source frequency = PLL = 48000000
       (CNR+1) = PWM clock source frequency/prescaler/PWM output frequency
               = 48000000/2/1200 = 20000
       (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
       CNR = 19999
       duty ratio = 50% ==> (CMR)/(CNR+1) = 50% ==> CMR = (CNR+1)*0.5 = 20000*50/100
       CMR = 10000
       Prescale value is 1 : prescaler = 2
    */

    /* Set PWM0 Timer clock prescaler */
    PWM_SET_PRESCALER(PWM0, 2, 1); /* Divided by 2 */

    /* Set PWM0 Timer duty */
    PWM_SET_CMR(PWM0, 2, 10000);
    PWM_SET_CMR(PWM0, 3, 10000);

    /* Set PWM0 Timer period */
    PWM_SET_CNR(PWM0, 2, 19999);

    /* Enable and configure dead zone */
    SYS_UnlockReg();
    PWM0->DTCTL[1] = 200;
    PWM0->DTCTL[1] |= PWM_DTCTL2_3_DTEN_Msk;
    SYS_LockReg();

    /* Enable output of PWM0 channel0~3 */
    PWM0->POEN |= (0xF);

    /* Enable PWM0 channel 0 period interrupt, use channel 0 to measure time. */
    PWM0->INTEN0 = (PWM0->INTEN0 & ~PWM_INTEN0_PIEN0_Msk) | PWM_INTEN0_PIEN0_Msk;
    NVIC_EnableIRQ(PWM0_IRQn);

    /* Start */
    PWM0->CNTEN = 0xF;

    while(1);
}

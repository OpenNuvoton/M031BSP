/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 7 $
 * $Date: 18/07/05 4:59p $
 * @brief
 *           Demonstrate how to set PWM0 channel 0 outputs 240 kHz waveform and
                         swtich duty in each 0.5%.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"


#define DEBUG_MSG printf

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define PLL_CLOCK       96000000
uint8_t InputString[16]= {0};
uint32_t sysGetNum(void)
{
	uint8_t cInputTemp=0x00;
    uint32_t nLoop = 0;
    while(cInputTemp != 0x0D)
    {
        cInputTemp = getchar();
        if(cInputTemp == 27) /* 27: ESC key */
        {
            return cInputTemp;
        }
        if(cInputTemp == 'x' || cInputTemp == 'X' || cInputTemp == 'f'||
                cInputTemp == 'F' || cInputTemp == 'r' || cInputTemp == 'R')
        {
            return cInputTemp;
        }
        if(cInputTemp == '-')
        {
            InputString[nLoop] = cInputTemp;
            DEBUG_MSG("%c",cInputTemp);
            nLoop++;
        }
        else if(cInputTemp >= '0' && cInputTemp <= '9')
        {
            InputString[nLoop] = cInputTemp;
            DEBUG_MSG("%c",cInputTemp);
            nLoop++;
        }
    }
    DEBUG_MSG("\n");

    return atoi((const char *)InputString);
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
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(1);

    /* Switch UART0 clock source to HIRC */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART0SEL_Msk) | CLK_CLKSEL1_UART0SEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_UART0DIV_Msk) | CLK_CLKDIV0_UART0(1);

    /* Enable UART0 peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Set core clock as PLL_CLOCK from PLL (no PLL in rev. B & C) */
    //CLK_SetCoreClock(PLL_CLOCK);

    /* Waiting for PLL clock ready */
    //CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);
    /*---------------------------------------------------------------------------------------------------------*/
    /* PWM clock frequency configuration                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Select HCLK clock source as PLL and and HCLK clock divider as 2 */
    //CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* PWM clock frequency can be set equal or double to HCLK by choosing case 1 or case 2 */
    /* case 1.PWM clock frequency is set equal to HCLK: select PWM module clock source as PCLK */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_PWM0SEL_Msk) | CLK_CLKSEL2_PWM0SEL_PCLK0;


    /* case 2.PWM clock frequency is set double to HCLK: select PWM module clock source as PLL */
    //CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_PWM0SEL_Msk) | CLK_CLKSEL2_PWM0SEL_PLL;
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable PWM0 module clock */
    CLK->APBCLK1 |= CLK_APBCLK1_PWM0CKEN_Msk;

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

    /* Set PB multi-function pins for PWM0 Channel 0 */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB5MFP_Msk)) |
                    SYS_GPB_MFPL_PB5MFP_PWM0_CH0;
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
    uint8_t  u8Duty;

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
    printf("PWM0 clock is from %s\n", (CLK->CLKSEL2 & CLK_CLKSEL2_PWM0SEL_Msk) ? "CPU" : "PLL");
    printf("+-----------------------------------------------------------------------------------+\n");
    printf("|                          PWM Driver Sample Code                                   |\n");
    printf("|                                                                                   |\n");
    printf("+-----------------------------------------------------------------------------------+\n");
    printf("  This sample code will use PWM0 channel 0 to output 240 kHz waveform.               \n");
    printf("  And user can change duty in each 0.5 %%.                                           \n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: PWM0 channel 0(PB.5)\n");

    /*
      Configure PWM0 channel 0 init period and duty(down counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = CMR / (CNR + 1)
      Period = 48 MHz / (1 * (199 + 1)) = 240000 Hz
      Duty ratio = 100 / (199 + 1) = 50%
    */

    /* PWM0 channel 0 frequency prescaler to 1 */
    PWM_SET_PRESCALER(PWM0, 0, 1-1);

    /* PWM0 channel 0 frequency period to 199 */
    PWM_SET_CNR(PWM0, 0, 199);

    /* PWM0 channel 0 frequency comparator to 100 */
    PWM_SET_CMR(PWM0, 0, 100);

    /* PWM0 channel 0 is edge-aligned and down counter type */
    PWM0->CTL1 = (PWM0->CTL1&(~PWM_CTL1_CNTTYPE0_Msk))| PWM_DOWN_COUNTER;

    /* ZeroLevel: Low, CmpUpLevel: nothing, PeriodLevel: nothing, CmpDownLevel: High */
    PWM_SET_OUTPUT_LEVEL(PWM0, PWM_CH_0_MASK, PWM_OUTPUT_LOW, PWM_OUTPUT_NOTHING, PWM_OUTPUT_NOTHING, PWM_OUTPUT_HIGH);

    /* Enable output of PWM0 channel 0 */
    PWM0->POEN |= PWM_POEN_POEN0_Msk;

    /* Start PWM counter */
    PWM0->CNTEN |= PWM_CNTEN_CNTEN0_Msk;

    while(1)
    {
        printf("\nDuty is equal to (duty level * 0.5) %% \n");
        printf("Please enter new duty level (0 ~ 200): \n");
        u8Duty = sysGetNum();

        if(u8Duty > 200)
            /* Get invalid value */
            printf("\nEnter value out of range\n");
        else
        {
            printf("Output Duty is %d.%d %%.\n", u8Duty/2, (u8Duty%2)*5);

            if(u8Duty == 200)
            {
                /* ZeroLevel: nothing, CmpUpLevel: nothing, PeriodLevel: High, CmpDownLevel: nothing */
                PWM_SET_OUTPUT_LEVEL(PWM0, PWM_CH_0_MASK, PWM_OUTPUT_NOTHING, PWM_OUTPUT_NOTHING, PWM_OUTPUT_HIGH, PWM_OUTPUT_NOTHING);

                /* PWM0 channel 0 duty to u8Duty */
                PWM_SET_CMR(PWM0, 0, u8Duty);
            }
            else
            {
                /* ZeroLevel: nothing, CmpUpLevel: nothing, PeriodLevel: High, CmpDownLevel: Low */
                PWM_SET_OUTPUT_LEVEL(PWM0, PWM_CH_0_MASK, PWM_OUTPUT_LOW, PWM_OUTPUT_NOTHING, PWM_OUTPUT_NOTHING, PWM_OUTPUT_HIGH);

                /* PWM0 channel 0 duty to u8Duty */
                PWM_SET_CMR(PWM0, 0, u8Duty);
            }
        }
    }
}

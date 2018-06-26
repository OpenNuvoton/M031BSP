/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 8 $
 * $Date: 18/05/31 4:26p $
 * @brief    Demonstrate how to use PWM counter output waveform.
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *
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


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~32 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable Internal RC 48MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Select HXT as the clock source of UART0 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Set core clock as PLL_CLOCK from PLL (no PLL in rev. B & C) */
//    CLK_SetCoreClock(PLL_CLOCK);

    /* Waiting for PLL clock ready */
//    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable PWM0 and PWM1 module clock */
    CLK_EnableModuleClock(PWM0_MODULE);
    CLK_EnableModuleClock(PWM1_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* PWM clock frequency configuration                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Select HCLK clock source as PLL and and HCLK clock divider as 2 */
//    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* PWM clock frequency can be set equal or double to HCLK by choosing case 1 or case 2 */
    /* case 1.PWM clock frequency is set equal to HCLK: select PWM module clock source as PCLK */
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL2_PWM0SEL_PCLK0, 0);
    CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL2_PWM1SEL_PCLK1, 0);

    /* case 2.PWM clock frequency is set double to HCLK: select PWM module clock source as PLL */
    //CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL2_PWM0SEL_PLL, NULL);
    //CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL2_PWM1SEL_PLL, NULL);
    /*---------------------------------------------------------------------------------------------------------*/

    /* Reset PWM0 and PWM1 module */
    SYS_ResetModule(PWM0_RST);
    SYS_ResetModule(PWM1_RST);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set PB multi-function pins for PWM0 Channel 0~5 */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB5MFP_Msk));
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB5MFP_PWM0_CH0;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB4MFP_Msk));
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB4MFP_PWM0_CH1;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB3MFP_Msk));
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB3MFP_PWM0_CH2;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB2MFP_Msk));
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB2MFP_PWM0_CH3;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk));
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB1MFP_PWM0_CH4;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk));
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB0MFP_PWM0_CH5;

    /* Set PB/PC/PA multi-function pins for PWM1 Channel 0~5 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB15MFP_Msk));
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB15MFP_PWM1_CH0;
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB14MFP_Msk));
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB14MFP_PWM1_CH1;
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC7MFP_Msk));
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC7MFP_PWM1_CH2;
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC6MFP_Msk ));
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC6MFP_PWM1_CH3;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA7MFP_Msk));
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA7MFP_PWM1_CH4;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA6MFP_Msk ));
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA6MFP_PWM1_CH5;
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
    printf("PWM0 clock is from %s\n", (CLK->CLKSEL2 & CLK_CLKSEL2_PWM0SEL_Msk) ? "CPU" : "PLL");
    printf("PWM1 clock is from %s\n", (CLK->CLKSEL2 & CLK_CLKSEL2_PWM1SEL_Msk) ? "CPU" : "PLL");
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          PWM Driver Sample Code                        |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output waveform with PWM0 and PWM1 channel 0~5.\n");
    printf("  I/O configuration:\n");
    printf("  PWM0 channel 0: 240000 Hz, duty 90%%.\n");
    printf("  PWM0 channel 1: 240000 Hz, duty 80%%.\n");
    printf("  PWM0 channel 2: 240000 Hz, duty 75%%.\n");
    printf("  PWM0 channel 3: 240000 Hz, duty 70%%.\n");
    printf("  PWM0 channel 4: 240000 Hz, duty 60%%.\n");
    printf("  PWM0 channel 5: 240000 Hz, duty 50%%.\n");
    printf("  PWM1 channel 0: 240000 Hz, duty 50%%.\n");
    printf("  PWM1 channel 1: 240000 Hz, duty 40%%.\n");
    printf("  PWM1 channel 2: 240000 Hz, duty 30%%.\n");
    printf("  PWM1 channel 3: 240000 Hz, duty 25%%.\n");
    printf("  PWM1 channel 4: 240000 Hz, duty 20%%.\n");
    printf("  PWM1 channel 5: 240000 Hz, duty 10%%.\n");
    printf("    waveform output pin: PWM0_CH0(PB.5), PWM0_CH1(PB.4), PWM0_CH2(PB.3), PWM0_CH3(PB.2), PWM0_CH4(PB.1), PWM0_CH5(PB.0)\n");
    printf("                         PWM1_CH0(PB.15), PWM1_CH1(PB.14), PWM1_CH2(PC.7), PWM1_CH3(PC.7), PWM1_CH4(PA.7), PWM1_CH5(PA.6)\n");


    /* PWM0 and PWM1 channel 0~5 frequency and duty configuration are as follows */
    PWM_ConfigOutputChannel(PWM0, 0, 240000, 90);
    PWM_ConfigOutputChannel(PWM0, 1, 240000, 80);
    PWM_ConfigOutputChannel(PWM0, 2, 240000, 75);
    PWM_ConfigOutputChannel(PWM0, 3, 240000, 70);
    PWM_ConfigOutputChannel(PWM0, 4, 240000, 60);
    PWM_ConfigOutputChannel(PWM0, 5, 240000, 50);
    PWM_ConfigOutputChannel(PWM1, 0, 240000, 50);
    PWM_ConfigOutputChannel(PWM1, 1, 240000, 40);
    PWM_ConfigOutputChannel(PWM1, 2, 240000, 30);
    PWM_ConfigOutputChannel(PWM1, 3, 240000, 25);
    PWM_ConfigOutputChannel(PWM1, 4, 240000, 20);
    PWM_ConfigOutputChannel(PWM1, 5, 240000, 10);

    /* Enable output of PWM0 and PWM1 channel 0~5 */
    PWM_EnableOutput(PWM0, 0x3F);
    PWM_EnableOutput(PWM1, 0x3F);

    /* Start PWM0 counter */
    PWM_Start(PWM0, 0x3F);
    /* Start PWM1 counter */
    PWM_Start(PWM1, 0x3F);

    printf("Press any key to stop.\n");
    getchar();

    /* Start PWM0 counter */
    PWM_ForceStop(PWM0, 0x3F);
    /* Start PWM1 counter */
    PWM_ForceStop(PWM1, 0x3F);

    printf("Done.");
    while(1);

}

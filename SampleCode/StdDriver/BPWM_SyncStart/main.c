/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 10 $
 * $Date: 18/07/19 2:18p $
 * @brief    Demonstrate how to use BPWM counter synchronous start function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));


    /* Enable BPWM0 and BPWM1 module clock */
    CLK_EnableModuleClock(BPWM0_MODULE);
    CLK_EnableModuleClock(BPWM1_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* BPWM clock frequency configuration                                                                      */
    /*---------------------------------------------------------------------------------------------------------*/
    /* BPWM clock frequency can be set equal to HCLK by choosing case 1 */
    /* case 1.BPWM clock frequency is set equal to HCLK: select BPWM module clock source as PCLK */
    CLK_SetModuleClock(BPWM0_MODULE, CLK_CLKSEL2_BPWM0SEL_PCLK0, 0);
    CLK_SetModuleClock(BPWM1_MODULE, CLK_CLKSEL2_BPWM1SEL_PCLK1, 0);
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Reset BPWM0 and BPWM1 module */
    SYS_ResetModule(BPWM0_RST);
    SYS_ResetModule(BPWM1_RST);

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


    /* BPWM0 and BPWM1 channel 0~5 frequency and duty configuration are as follows */
    BPWM_ConfigOutputChannel(BPWM0, 0, 1000, 50);
    BPWM_ConfigOutputChannel(BPWM0, 1, 1000, 50);
    BPWM_ConfigOutputChannel(BPWM0, 2, 1000, 50);
    BPWM_ConfigOutputChannel(BPWM0, 3, 1000, 50);
    BPWM_ConfigOutputChannel(BPWM0, 4, 1000, 50);
    BPWM_ConfigOutputChannel(BPWM0, 5, 1000, 50);
    BPWM_ConfigOutputChannel(BPWM1, 0, 1000, 50);
    BPWM_ConfigOutputChannel(BPWM1, 1, 1000, 50);
    BPWM_ConfigOutputChannel(BPWM1, 2, 1000, 50);
    BPWM_ConfigOutputChannel(BPWM1, 3, 1000, 50);
    BPWM_ConfigOutputChannel(BPWM1, 4, 1000, 50);
    BPWM_ConfigOutputChannel(BPWM1, 5, 1000, 50);

    /* Enable counter synchronous start function for BPWM0 and BPWM1 channel 0~5 */
    BPWM_ENABLE_TIMER_SYNC(BPWM0, 0x3F, BPWM_SSCTL_SSRC_BPWM0);
    BPWM_ENABLE_TIMER_SYNC(BPWM1, 0x3F, BPWM_SSCTL_SSRC_BPWM0);

    /* Enable output of BPWM0 and BPWM1 channel 0~5 */
    BPWM_EnableOutput(BPWM0, 0x3F);
    BPWM_EnableOutput(BPWM1, 0x3F);

    printf("Press any key to start.\n");
    getchar();

    /* Trigger BPWM counter synchronous start by BPWM0 */
    BPWM_TRIGGER_SYNC_START(BPWM0);

    printf("Done.");

    while (1);

}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate analog comparator (ACMP) comparison by comparing
 *           ACMP1_P1 input and VBG voltage and shows the result on UART console.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


void ACMP01_IRQHandler(void)
{
    static uint32_t u32Cnt = 0;

    /* Clear ACMP 1 interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP01, 1);
    /* Check Comparator 0 Output Status */
    if(ACMP_GET_OUTPUT(ACMP01, 1))
        printf("ACMP1_P voltage > VBG (%d)\n", u32Cnt);
    else
        printf("ACMP1_P voltage <= VBG (%d)\n", u32Cnt);

    u32Cnt++;
}


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable ACMP01 peripheral clock */
    CLK_EnableModuleClock(ACMP01_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set PB.4 multi-function pin for ACMP1 positive input pin and PB6 multi-function pin for ACMP1 output pin */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB6MFP_Msk)) |
                    (SYS_GPB_MFPL_PB4MFP_ACMP1_P1 | SYS_GPB_MFPL_PB6MFP_ACMP1_O);
    /* Disable digital input path of analog pin ACMP1_P1 to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT4);
    /* Set PB.4 to input mode for ACMP analog input pins */
    GPIO_SetMode(PB, BIT4, GPIO_MODE_INPUT);

    /* Lock protected registers */
    SYS_LockReg();
}

/*----------------------------------------------------------------------*/
/* Init UART0                                                           */
/*----------------------------------------------------------------------*/
void UART0_Init(void)
{
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*
 * When the voltage of the positive input is greater than the voltage of the
 * negative input, the analog comparator outputs logical one; otherwise, it outputs
 * logical zero. This sample code will show the expression of the comparator's
 * number when detecting a transition of analog comparator's output.
 */

int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\nThis sample code demonstrates ACMP1 function. Using ACMP1_P1 (PB4) as ACMP1\n");
    printf("positive input and using internal band-gap voltage as the negative input.\n");
    printf("The compare result reflects on ACMP1_O (PB6).\n");

    printf("Press any key to start ...");
    getchar();
    printf("\n");

    /* Configure ACMP1. Enable ACMP1 and select band-gap voltage as the source of ACMP negative input. */
    ACMP_Open(ACMP01, 1, ACMP_CTL_NEGSEL_VBG, ACMP_CTL_HYSTERESIS_DISABLE);
    /* Select P1 as ACMP positive input channel */
    ACMP_SELECT_P(ACMP01, 1, ACMP_CTL_POSSEL_P1);
    /* Enable interrupt */
    ACMP_ENABLE_INT(ACMP01, 1);

    /* Enable ACMP01 interrupt */
    NVIC_EnableIRQ(ACMP01_IRQn);

    while(1);
}

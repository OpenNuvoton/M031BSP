/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to monitor ACMP input with window compare function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


void ACMP01_IRQHandler(void)
{
    /* Clear interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP01, 0);
    ACMP_CLR_INT_FLAG(ACMP01, 1);

    if(ACMP01->STATUS & ACMP_STATUS_ACMPWO_Msk)
    {
        printf("The input voltage is within the window\n");
    }
    else
    {
        printf("The input voltage is not within the window\n");
    }
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
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);


    /* Set PA.11 multi-function pin for ACMP0_P0 positive input pin */
    SYS->GPA_MFPH = (SYS->GPA_MFPH & ~(SYS_GPA_MFPH_PA11MFP_Msk)) |
                    (SYS_GPA_MFPH_PA11MFP_ACMP0_P0);
    /* Disable digital input path of analog pin ACMP0_P0 to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PA, BIT11);
    /* Set PA.11 to input mode for ACMP analog input pins */
    GPIO_SetMode(PA, BIT11, GPIO_MODE_INPUT);

    /* Set PB.4 multi-function pin for ACMP1_P1 positive input pin */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB4MFP_Msk)) |
                    (SYS_GPB_MFPL_PB4MFP_ACMP1_P1);
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


int32_t main(void)
{
    uint32_t volatile i;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\nThis sample code demonstrates ACMP window compare function\n");
    printf("Connect the specific analog voltage source to the positive inputs\n");
    printf("of both comparators, PA11 and PB4. This sample code will monitor if the\n");
    printf("input is between the range of VDDA * 9 / 24 and bandgap.\n");
    printf("Press any key to continue ...");
    getchar();
    printf("\n");

    /* Select VDDA as CRV source */
    ACMP_SELECT_CRV_SRC(ACMP01, ACMP_VREF_CRVSSEL_VDDA);
    /* Select CRV level: VDDA * 9 / 24 */
    ACMP_CRV_SEL(ACMP01, 5);
    /* Configure ACMP0. Enable ACMP0 and select CRV as the source of ACMP negative input. */
    ACMP_Open(ACMP01, 0, ACMP_CTL_NEGSEL_CRV, ACMP_CTL_HYSTERESIS_DISABLE);
    /* Configure ACMP1. Enable ACMP1 and select VBG as the source of ACMP negative input. */
    ACMP_Open(ACMP01, 1, ACMP_CTL_NEGSEL_VBG, ACMP_CTL_HYSTERESIS_DISABLE);
    /* Select P0 as ACMP0 positive input channel */
    ACMP_SELECT_P(ACMP01, 0, ACMP_CTL_POSSEL_P0);
    /* Select P1 as ACMP1 positive input channel */
    ACMP_SELECT_P(ACMP01, 1, ACMP_CTL_POSSEL_P1);
    /* Enable window compare mode */
    ACMP_ENABLE_WINDOW_COMPARE(ACMP01, 0);
    ACMP_ENABLE_WINDOW_COMPARE(ACMP01, 1);

    /* Clear ACMP 0 and 1 interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP01, 0);
    ACMP_CLR_INT_FLAG(ACMP01, 1);

    /* Give ACMP some time to settle */
    for(i = 0; i < 1000; i++);

    if(ACMP01->STATUS & ACMP_STATUS_ACMPWO_Msk)
    {
        printf("The input voltage in inside the window\n");
    }
    else
    {
        printf("The input voltage in outside the window\n");
    }

    /* Enable interrupt */
    ACMP_ENABLE_INT(ACMP01, 0);
    ACMP_ENABLE_INT(ACMP01, 1);
    /* Enable ACMP01 interrupt */
    NVIC_EnableIRQ(ACMP01_IRQn);

    while(1);
}

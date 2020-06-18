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
    ACMP01->STATUS = (ACMP_STATUS_ACMPIF0_Msk | ACMP_STATUS_ACMPIF1_Msk);

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

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~32 MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for 32MHz clock ready */
    while((CLK->STATUS & CLK_STATUS_HXTSTB_Msk) != CLK_STATUS_HXTSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(1);

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Switch UART0 clock source to XTAL */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART0SEL_Msk) | CLK_CLKSEL1_UART0SEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_UART0DIV_Msk) | CLK_CLKDIV0_UART0(1);

    /* Enable UART0 and ACMP01 peripheral clock */
    CLK->APBCLK0 |= (CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_ACMP01CKEN_Msk);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set PA.11 and PB.4 to input mode */
    PA->MODE &= ~GPIO_MODE_MODE11_Msk;
    PB->MODE &= ~GPIO_MODE_MODE4_Msk;

    /* Set PA11 multi-function pin for ACMP0 positive input pin */
    SYS->GPA_MFPH |= SYS_GPA_MFPH_PA11MFP_ACMP0_P0;

    /* Set PB4 multi-function pin for ACMP1 positive input pin */
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB4MFP_ACMP1_P1;

    /* Disable digital input path of analog pin ACMP0_P0 and ACMP1_P1 to prevent leakage */
    PA->DINOFF |= (BIT11<<GPIO_DINOFF_DINOFF0_Pos);
    PB->DINOFF |= (BIT4<<GPIO_DINOFF_DINOFF0_Pos);

    /* Lock protected registers */
    SYS_LockReg();
}

/*----------------------------------------------------------------------*/
/* Init UART0                                                           */
/*----------------------------------------------------------------------*/
void UART0_Init(void)
{
    /* Reset UART0 */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
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
    /* Select CRV level: VDDA * (1/6 + 5/24) */
    ACMP01->VREF = (ACMP01->VREF & ~(ACMP_VREF_CRVSSEL_Msk | ACMP_VREF_CRVCTL_Msk)) |
                   (ACMP_VREF_CRVSSEL_VDDA | (5<<ACMP_VREF_CRVCTL_Pos));

    /* Configure ACMP0. Enable ACMP0 and select CRV as the source of ACMP negative input. */
    /* Select P0 as ACMP0 positive input channel */
    /* Enable window compare mode */
    ACMP01->CTL[0] = (ACMP01->CTL[0] & ~(ACMP_CTL_NEGSEL_Msk | ACMP_CTL_HYSEN_Msk | ACMP_CTL_POSSEL_Msk)) |
                     (ACMP_CTL_NEGSEL_CRV | ACMP_CTL_HYSTERESIS_DISABLE | ACMP_CTL_ACMPEN_Msk | ACMP_CTL_POSSEL_P0 | ACMP_CTL_WKEN_Msk);
    /* Configure ACMP1. Enable ACMP1 and select VBG as the source of ACMP negative input. */
    /* Select P1 as ACMP1 positive input channel */
    /* Enable window compare mode */
    ACMP01->CTL[1] = (ACMP01->CTL[1] & ~(ACMP_CTL_NEGSEL_Msk | ACMP_CTL_HYSEN_Msk | ACMP_CTL_POSSEL_Msk)) |
                     (ACMP_CTL_NEGSEL_VBG | ACMP_CTL_HYSTERESIS_DISABLE | ACMP_CTL_ACMPEN_Msk | ACMP_CTL_POSSEL_P1 | ACMP_CTL_WKEN_Msk);

    /* Clear ACMP 0 and 1 interrupt flag */
    ACMP01->STATUS = (ACMP_STATUS_ACMPIF0_Msk | ACMP_STATUS_ACMPIF1_Msk);

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
    ACMP01->CTL[0] |= ACMP_CTL_ACMPIE_Msk;
    ACMP01->CTL[1] |= ACMP_CTL_ACMPIE_Msk;
    /* Enable ACMP01 interrupt */
    NVIC_EnableIRQ(ACMP01_IRQn);

    while(1);
}

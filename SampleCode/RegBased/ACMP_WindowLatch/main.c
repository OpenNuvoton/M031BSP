/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to use ACMP window latch mode.
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
    ACMP01->STATUS = (ACMP_STATUS_ACMPIF1_Msk);
    /* Check Comparator 1 Output Status */
    if(((ACMP01->STATUS & (ACMP_STATUS_ACMPO1_Msk))?1:0))
        printf("ACMP1_P voltage > Band-gap voltage (%d)\n", u32Cnt);
    else
        printf("ACMP1_P voltage <= Band-gap voltage (%d)\n", u32Cnt);

    u32Cnt++;
}


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) != CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(1);

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Switch UART0 clock source to HIRC */
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

    /* Set PB4 multi-function pin for ACMP1 positive input pin and PB6 multi-function pin for ACMP1 output pin */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB4MFP_Msk | GPIO_MODE_MODE6_Msk)) |
                    (SYS_GPB_MFPL_PB4MFP_ACMP1_P1 | SYS_GPB_MFPL_PB6MFP_ACMP1_O);
    /* Set PB.4 and PB.6 to input mode */
    PB->MODE &= ~(GPIO_MODE_MODE4_Msk | GPIO_MODE_MODE6_Msk);
    /* Disable digital input path of analog pin ACMP1_P1 to prevent leakage */
    PB->DINOFF |= (BIT4<<GPIO_DINOFF_DINOFF0_Pos);

    /* Set PA.6 multi-function pin for ACMP1 window latch pin */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA6MFP_Msk)) |
                    (SYS_GPA_MFPL_PA6MFP_ACMP1_WLAT);

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

/*
 * In window latch mode and window latch pin is at high level, the voltage of the positive
 * input is greater than the  voltage of the negative input, the analog comparator outputs
 * logical one; otherwise, it outputs logical zero. When window latch pin is at low level,
 * the output level does not change no matter how positive  input voltage changes.
 */

int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\nThis sample code demonstrates ACMP1 window latch function. Using ACMP1_P1 (PB4) as ACMP1\n");
    printf("positive input and using internal band-gap voltage as the negative input. ACMP1_WLAT is at\n");
    printf("PA6, when PA6 is low, compare result on ACMP1_O (PB6) does not change with ACMP1_P1. When PA6\n");
    printf("is high, ACMP1_O works as usual\n");

    printf("Press any key to start ...");
    getchar();
    printf("\n");

    /* Configure ACMP1. Enable ACMP1 and select band-gap voltage as the source of ACMP negative input. */
    /* Select P1 as ACMP positive input channel */
    /* Enable interrupt */
    /* Enable window latch mode */
    ACMP01->CTL[1] = (ACMP01->CTL[1] & ~(ACMP_CTL_NEGSEL_Msk | ACMP_CTL_HYSEN_Msk | ACMP_CTL_POSSEL_Msk)) |
                     (ACMP_CTL_NEGSEL_VBG | ACMP_CTL_HYSTERESIS_DISABLE | ACMP_CTL_ACMPEN_Msk | ACMP_CTL_POSSEL_P1 | ACMP_CTL_ACMPIE_Msk | ACMP_CTL_WLATEN_Msk);

    /* Enable ACMP01 interrupt */
    NVIC_EnableIRQ(ACMP01_IRQn);

    while(1);
}

/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use ACMP to wake up system from Power-down mode while comparator
 *           output changes.
 *           Please refer to the sample code SYS_PowerDown_MinCurrent to set
 *           the minimum current of the system in Power-down mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


void ACMP01_IRQHandler(void)
{
    printf("\nACMP1 interrupt!\n");

    /* Clear ACMP1 and Wake-up interrupt flag */
    ACMP01->STATUS = (ACMP_STATUS_ACMPIF1_Msk | ACMP_STATUS_WKIF1_Msk);
}

void PowerDownFunction(void)
{
    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(UART0);

    /* Set the processor uses deep sleep as its low power mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Set system Power-down enabled*/
    CLK->PWRCTL |= CLK_PWRCTL_PDEN_Msk;

    /* Chip enter Power-down mode after CPU run WFI instruction */
    __WFI();
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

    /* Set PB.4 and PB.6 to input mode */
    PB->MODE &= ~(GPIO_MODE_MODE4_Msk | GPIO_MODE_MODE6_Msk);
    /* Set PB.4 multi-function pin for ACMP1 positive input pin and PB6 multi-function pin for ACMP1 output pin */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB6MFP_Msk)) |
                    (SYS_GPB_MFPL_PB4MFP_ACMP1_P1 | SYS_GPB_MFPL_PB6MFP_ACMP1_O);
    /* Disable digital input path of analog pin ACMP1_P1 to prevent leakage */
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

/*
 * When the voltage of the positive input is greater than the voltage of the negative input,
 * the analog comparator outputs logical one; otherwise, it outputs logical zero.
 * This chip will be waked up from power down mode when detecting a transition of analog comparator's output.
 */
int32_t main(void)
{
    uint32_t u32DelayCount;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\nThis sample code demonstrates ACMP1 function. Using ACMP1_P1 (PB4) as ACMP1\n");
    printf("positive input and using internal CRV as the negative input.\n");
    printf("The compare result reflects on ACMP1_O (PB6).\n");
    printf("Press any key to enter power down mode ...");
    getchar();
    printf("\n");

    /* Select VDDA as CRV source */
    /* Select CRV level: VDDA * (1/6 + 5/24) */
    ACMP01->VREF = (ACMP01->VREF & ~(ACMP_VREF_CRVSSEL_Msk | ACMP_VREF_CRVCTL_Msk)) |
                   (ACMP_VREF_CRVSSEL_VDDA | (5<<ACMP_VREF_CRVCTL_Pos));
    /* Configure ACMP1. Enable ACMP1 and select CRV as the source of ACMP negative input. */
    /* Select P1 as ACMP positive input channel */
    ACMP01->CTL[1] = (ACMP01->CTL[1] & ~(ACMP_CTL_NEGSEL_Msk | ACMP_CTL_HYSEN_Msk | ACMP_CTL_POSSEL_Msk)) |
                     (ACMP_CTL_NEGSEL_CRV | ACMP_CTL_HYSTERESIS_DISABLE | ACMP_CTL_ACMPEN_Msk | ACMP_CTL_POSSEL_P1);
    __NOP();
    for(u32DelayCount = 0; u32DelayCount < 100; u32DelayCount++); /* For ACMP setup time */
    __NOP();
    /* Clear ACMP 1 interrupt flag */
    ACMP01->STATUS = (ACMP_STATUS_ACMPIF1_Msk);

    /* Enable wake-up function */
    /* Enable interrupt */
    ACMP01->CTL[1] |= (ACMP_CTL_WKEN_Msk | ACMP_CTL_ACMPIE_Msk);
    /* Enable ACMP01 interrupt */
    NVIC_EnableIRQ(ACMP01_IRQn);

    printf("\nSystem enter power-down mode ... \n");
    /* To check if all the debug messages are finished */
    while(!((UART0->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) >> UART_FIFOSTS_TXEMPTYF_Pos));

    /* To program PWRCTL register, it needs to disable register protection first. */
    SYS_UnlockReg();

    PowerDownFunction();
    printf("Wake up by ACMP1!\n");
    while(1);
}

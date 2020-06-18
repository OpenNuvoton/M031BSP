/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Monitor the conversion result of channel 2 by the digital compare function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag;
volatile uint32_t g_u32AdcCmp0IntFlag;
volatile uint32_t g_u32AdcCmp1IntFlag;


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

    /* Enable UART0 and ADC peripheral clock */
    CLK->APBCLK0 |= (CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_ADCCKEN_Msk);

    /* ADC clock source is PCLK1, set divider to 1 */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_ADCSEL_Msk) | CLK_CLKSEL2_ADCSEL_PCLK1;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_ADCDIV_Msk) | CLK_CLKDIV0_ADC(1);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set PB.2 to input mode */
    PB->MODE &= ~(GPIO_MODE_MODE2_Msk);
    /* Configure the GPB2 ADC analog input pins.  */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB2MFP_Msk)) |
                    (SYS_GPB_MFPL_PB2MFP_ADC0_CH2);

    /* Disable the GPB2 digital input path to avoid the leakage current. */
    PB->DINOFF |= ((BIT2)<<GPIO_DINOFF_DINOFF0_Pos);

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

void ADC_FunctionTest()
{
    uint32_t u32ConversionData;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|            ADC compare function (result monitor) sample code         |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\nIn this test, software will compare the conversion result of channel 2.\n");

    /* Enable ADC converter */
    ADC->ADCR |= ADC_ADCR_ADEN_Msk;

    /* Do calibration for ADC to decrease the effect of electrical random noise. */
    ADC->ADCALSTSR |= ADC_ADCALSTSR_CALIF_Msk;  /* Clear Calibration Finish Interrupt Flag */
    ADC->ADCALR |= ADC_ADCALR_CALEN_Msk;        /* Enable Calibration function */
    ADC_START_CONV(ADC);                        /* Start to calibration */
    while((ADC->ADCALSTSR & ADC_ADCALSTSR_CALIF_Msk) != ADC_ADCALSTSR_CALIF_Msk);   /* Wait calibration finish */

    /* Set input mode as single-end, Single mode, and select channel 2 */
    ADC->ADCR = (ADC->ADCR & ~(ADC_ADCR_DIFFEN_Msk | ADC_ADCR_ADMD_Msk)) |
                (ADC_ADCR_DIFFEN_SINGLE_END | ADC_ADCR_ADMD_SINGLE);
    ADC->ADCHER = (ADC->ADCHER & ~ADC_ADCHER_CHEN_Msk) | (BIT2);

    /* Enable ADC comparator 0. Compare condition: conversion result < 0x800; match Count=5 */
    printf("   Set the compare condition of comparator 0: channel 2 is less than 0x800; match count is 5.\n");
    ADC->ADCMPR[0] = (2 << ADC_ADCMPR_CMPCH_Pos) |
                     (ADC_ADCMPR_CMPCOND_LESS_THAN) |
                     (0x800 << ADC_ADCMPR_CMPD_Pos) |
                     ((5 - 1) << ADC_ADCMPR_CMPMATCNT_Pos) |
                     ADC_ADCMPR_CMPEN_Msk;

    /* Enable ADC comparator 1. Compare condition: conversion result >= 0x800; match Count=5 */
    printf("   Set the compare condition of comparator 1: channel 2 is greater than or equal to 0x800; match count is 5.\n");
    ADC->ADCMPR[1] = (2 << ADC_ADCMPR_CMPCH_Pos) |
                     (ADC_ADCMPR_CMPCOND_GREATER_OR_EQUAL) |
                     (0x800 << ADC_ADCMPR_CMPD_Pos) |
                     ((5 - 1) << ADC_ADCMPR_CMPMATCNT_Pos) |
                     ADC_ADCMPR_CMPEN_Msk;

    /* Clear the A/D interrupt flag for safe */
    ADC->ADSR0 = ADC_ADF_INT;

    /* Enable the sample module interrupt */
    ADC->ADCR |= ADC_ADCR_ADIE_Msk;
    NVIC_EnableIRQ(ADC_IRQn);

    /* Clear the ADC comparator 0 interrupt flag for safe */
    ADC->ADSR0 = ADC_CMP0_INT;
    /* Enable ADC comparator 0 interrupt */
    ADC->ADCMPR[0] |= ADC_ADCMPR_CMPIE_Msk;

    /* Clear the ADC comparator 1 interrupt flag for safe */
    ADC->ADSR0 = ADC_CMP1_INT;
    /* Enable ADC comparator 1 interrupt */
    ADC->ADCMPR[1] |= ADC_ADCMPR_CMPIE_Msk;

    /* Reset the ADC interrupt indicator and trigger sample module to start A/D conversion */
    g_u32AdcIntFlag = 0;
    g_u32AdcCmp0IntFlag = 0;
    g_u32AdcCmp1IntFlag = 0;
    ADC->ADCR |= ADC_ADCR_ADST_Msk;

    /* Wait ADC compare interrupt */
    while(1)
    {
        if (g_u32AdcIntFlag == 1)
        {
            u32ConversionData = (ADC->ADDR[2] & ADC_ADDR_RSLT_Msk);
            printf("Conversion result of channel 2: 0x%03X (%d)\n", u32ConversionData, u32ConversionData);

            if ((g_u32AdcCmp0IntFlag == 1) || (g_u32AdcCmp1IntFlag == 1))
                break;

            g_u32AdcIntFlag = 0;
            ADC->ADCR |= ADC_ADCR_ADST_Msk;
        }
    }

    /* Disable ADC comparator interrupt */
    ADC->ADCMPR[0] &= ~ADC_ADCMPR_CMPIE_Msk;
    ADC->ADCMPR[1] &= ~ADC_ADCMPR_CMPIE_Msk;
    /* Disable compare function */
    ADC->ADCMPR[0] = 0;
    ADC->ADCMPR[1] = 0;

    if(g_u32AdcCmp0IntFlag == 1)
    {
        printf("Comparator 0 interrupt occurs.\nThe conversion result of channel 2 is less than 0x800\n");
    }
    else
    {
        printf("Comparator 1 interrupt occurs.\nThe conversion result of channel 2 is greater than or equal to 0x800\n");
    }
}

void ADC_IRQHandler(void)
{
    if(ADC->ADSR0 & ADC_CMP0_INT)
    {
        g_u32AdcCmp0IntFlag = 1;
        ADC->ADSR0 = ADC_CMP0_INT;  /* Clear the A/D compare flag 0 */
    }

    if(ADC->ADSR0 & ADC_CMP1_INT)
    {
        g_u32AdcCmp1IntFlag = 1;
        ADC->ADSR0 = ADC_CMP1_INT;  /* Clear the A/D compare flag 1 */
    }

    g_u32AdcIntFlag = 1;
    ADC->ADSR0 = ADC_ADF_INT;
}

int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* ADC function test */
    ADC_FunctionTest();

    /* Disable ADC IP clock */
    CLK->APBCLK0 &= ~(CLK_APBCLK0_ADCCKEN_Msk);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(ADC_IRQn);

    printf("Exit ADC sample code\n");

    while(1);
}

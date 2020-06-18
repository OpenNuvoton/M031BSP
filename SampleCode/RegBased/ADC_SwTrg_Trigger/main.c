/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Trigger ADC by writing ADC software trigger register.
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

    /* Set PB.2 ~ PB.3 to input mode */
    PB->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);
    /* Configure the PB.0 - PB.3 ADC analog input pins. */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk)) |
                    (SYS_GPB_MFPL_PB2MFP_ADC0_CH2 | SYS_GPB_MFPL_PB3MFP_ADC0_CH3);
    /* Disable the GPB2 digital input path to avoid the leakage current. */
    PB->DINOFF |= ((BIT2|BIT3)<<GPIO_DINOFF_DINOFF0_Pos);

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
    uint8_t  u8Option;
    int32_t  i32ConversionData;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                   ADC Software trigger mode test                     |\n");
    printf("+----------------------------------------------------------------------+\n");

    /* Enable ADC converter */
    ADC->ADCR |= ADC_ADCR_ADEN_Msk;

    /* Do calibration for ADC to decrease the effect of electrical random noise. */
    ADC->ADCALSTSR |= ADC_ADCALSTSR_CALIF_Msk;  /* Clear Calibration Finish Interrupt Flag */
    ADC->ADCALR |= ADC_ADCALR_CALEN_Msk;        /* Enable Calibration function */
    ADC_START_CONV(ADC);                        /* Start to calibration */
    while((ADC->ADCALSTSR & ADC_ADCALSTSR_CALIF_Msk) != ADC_ADCALSTSR_CALIF_Msk);   /* Wait calibration finish */

    while(1)
    {
        printf("Select input mode:\n");
        printf("  [1] Single end input (channel 2 only)\n");
        printf("  [2] Differential input (channel pair 1 only)\n");
        printf("  Other keys: exit single mode test\n");
        u8Option = getchar();

        if(u8Option == '1')
        {
            /* Set input mode as single-end, Single-cycle scan mode, and select channel 2 */
            ADC->ADCR = (ADC->ADCR & ~(ADC_ADCR_DIFFEN_Msk | ADC_ADCR_ADMD_Msk)) |
                        (ADC_ADCR_DIFFEN_SINGLE_END | ADC_ADCR_ADMD_SINGLE_CYCLE | ADC_ADCR_ADIE_Msk);
            ADC->ADCHER = (ADC->ADCHER & ~ADC_ADCHER_CHEN_Msk) | (BIT2);

            /* Clear the A/D interrupt flag for safe */
            ADC->ADSR0 = ADC_ADF_INT;

            /* Enable the sample module interrupt */
            NVIC_EnableIRQ(ADC_IRQn);

            /* Reset the ADC interrupt indicator and trigger sample module 0 to start A/D conversion */
            g_u32AdcIntFlag = 0;
            ADC->ADCR |= ADC_ADCR_ADST_Msk;

            /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
            while(g_u32AdcIntFlag == 0);

            /* Disable the sample module interrupt */
            ADC->ADCR &= ~ADC_ADCR_ADIE_Msk;

            /* Get the conversion result of ADC channel 2 */
            i32ConversionData = (ADC->ADDR[2] & ADC_ADDR_RSLT_Msk);
            printf("Conversion result of channel 2: 0x%X (%d)\n\n", i32ConversionData, i32ConversionData);
        }
        else if(u8Option == '2')
        {
            /* Set input mode as differential, Single-cycle scan mode, and select channel 2 */
            ADC->ADCR = (ADC->ADCR & ~(ADC_ADCR_DIFFEN_Msk | ADC_ADCR_ADMD_Msk)) |
                        (ADC_ADCR_DIFFEN_DIFFERENTIAL | ADC_ADCR_ADMD_SINGLE_CYCLE | ADC_ADCR_ADIE_Msk);
            ADC->ADCHER = (ADC->ADCHER & ~ADC_ADCHER_CHEN_Msk) | (BIT2);

            /* Clear the A/D interrupt flag for safe */
            ADC->ADSR0 = ADC_ADF_INT;

            /* Enable the sample module interrupt */
            NVIC_EnableIRQ(ADC_IRQn);

            /* Reset the ADC indicator and trigger sample module to start A/D conversion */
            g_u32AdcIntFlag = 0;
            ADC->ADCR |= ADC_ADCR_ADST_Msk;

            /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
            while(g_u32AdcIntFlag == 0);

            /* Disable the sample module interrupt */
            ADC->ADCR &= ~ADC_ADCR_ADIE_Msk;

            /* Get the conversion result of channel 2 */
            i32ConversionData = (ADC->ADDR[2] & ADC_ADDR_RSLT_Msk);
            printf("Conversion result of channel pair 1: 0x%X (%d)\n\n", i32ConversionData, i32ConversionData);
        }
        else
            return ;
    }
}


void ADC_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    ADC->ADSR0 = ADC_ADF_INT; /* Clear the A/D interrupt flag */
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

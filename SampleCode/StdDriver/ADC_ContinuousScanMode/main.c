/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Perform A/D Conversion with ADC continuous scan mode.
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

    /* Enable ADC module clock */
    CLK_EnableModuleClock(ADC_MODULE);

    /* ADC clock source is PCLK1, set divider to 1 */
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL2_ADCSEL_PCLK1, CLK_CLKDIV0_ADC(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set PB.0 - PB.3 to input mode */
    GPIO_SetMode(PB, BIT0|BIT1|BIT2|BIT3, GPIO_MODE_INPUT);
    /* Configure the PB.0 - PB.3 ADC analog input pins.  */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk)) |
                    (SYS_GPB_MFPL_PB0MFP_ADC0_CH0 | SYS_GPB_MFPL_PB1MFP_ADC0_CH1 | SYS_GPB_MFPL_PB2MFP_ADC0_CH2 | SYS_GPB_MFPL_PB3MFP_ADC0_CH3);
    /* Disable the PB.0 - PB.3 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT0|BIT1|BIT2|BIT3);

    /* Lock protected registers */
    SYS_LockReg();
}

void ADC_FunctionTest()
{
    uint8_t  u8Option;
    uint32_t u32ChannelCount;
    int32_t  i32ConversionData;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                 ADC continuous scan mode sample code                 |\n");
    printf("+----------------------------------------------------------------------+\n");

    /* Enable ADC converter */
    ADC_POWER_ON(ADC);

    while(1)
    {
        printf("Select input mode:\n");
        printf("  [1] Single end input (channel 0, 1, 2 and 3)\n");
        printf("  [2] Differential input (input channel pair 0 and 1)\n");
        printf("  Other keys: exit continuous scan mode test\n");
        u8Option = getchar();

        if(u8Option == '1')
        {
            /* Set the ADC operation mode as continuous scan, input mode as single-end and
                 enable the analog input channel 0, 1, 2 and 3 */
            ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_CONTINUOUS, BIT0|BIT1|BIT2|BIT3);

            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

            /* Enable the sample module interrupt */
            ADC_ENABLE_INT(ADC, ADC_ADF_INT);  /* Enable sample module A/D interrupt. */
            NVIC_EnableIRQ(ADC_IRQn);

            /* Reset the ADC interrupt indicator and trigger sample module 0 to start A/D conversion */
            g_u32AdcIntFlag = 0;
            ADC_START_CONV(ADC);

            /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
            while(g_u32AdcIntFlag == 0);

            /* Get the conversion result */
            for(u32ChannelCount = 0; u32ChannelCount < 4; u32ChannelCount++)
            {
                i32ConversionData = ADC_GET_CONVERSION_DATA(ADC, u32ChannelCount);
                printf("Conversion result of channel %d: 0x%X (%d)\n", u32ChannelCount, i32ConversionData, i32ConversionData);
            }

            /* Wait ADC interrupt of next round (g_u32AdcIntFlag will be set at IRQ_Handler function) */
            g_u32AdcIntFlag = 0;
            while(g_u32AdcIntFlag == 0);

            /* Stop A/D conversion */
            ADC_STOP_CONV(ADC);

            /* Disable the sample module interrupt */
            ADC_DISABLE_INT(ADC, ADC_ADF_INT);

            for(u32ChannelCount = 0; u32ChannelCount < 4; u32ChannelCount++)
            {
                i32ConversionData = ADC_GET_CONVERSION_DATA(ADC, u32ChannelCount);
                printf("Conversion result of channel %d: 0x%X (%d)\n", u32ChannelCount, i32ConversionData, i32ConversionData);
            }
        }
        else if(u8Option == '2')
        {
            /* Set the ADC operation mode as continuous scan, input mode as differential and
               enable analog input channel 0 and 2 */
            ADC_Open(ADC, ADC_ADCR_DIFFEN_DIFFERENTIAL, ADC_ADCR_ADMD_CONTINUOUS, BIT0|BIT2);

            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

            /* Enable the sample module interrupt */
            ADC_ENABLE_INT(ADC, ADC_ADF_INT);  /* Enable sample module A/D interrupt. */
            NVIC_EnableIRQ(ADC_IRQn);

            /* Reset the ADC indicator and trigger sample module to start A/D conversion */
            g_u32AdcIntFlag = 0;
            ADC_START_CONV(ADC);

            /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
            while(g_u32AdcIntFlag == 0);

            /* Get the conversion result */
            for(u32ChannelCount = 0; u32ChannelCount < 2; u32ChannelCount++)
            {
                i32ConversionData = ADC_GET_CONVERSION_DATA(ADC, u32ChannelCount * 2);
                printf("Conversion result of differential input pair %d: 0x%X (%d)\n", u32ChannelCount, i32ConversionData, i32ConversionData);
            }

            /* Wait ADC interrupt of next round (g_u32AdcIntFlag will be set at IRQ_Handler function) */
            g_u32AdcIntFlag = 0;
            while(g_u32AdcIntFlag == 0);

            /* Stop A/D conversion */
            ADC_STOP_CONV(ADC);

            /* Disable the sample module interrupt */
            ADC_DISABLE_INT(ADC, ADC_ADF_INT);

            for(u32ChannelCount = 0; u32ChannelCount < 2; u32ChannelCount++)
            {
                i32ConversionData = ADC_GET_CONVERSION_DATA(ADC, u32ChannelCount * 2);
                printf("Conversion result of differential input pair %d: 0x%X (%d)\n", u32ChannelCount, i32ConversionData, i32ConversionData);
            }
        }
        else
            return;

        printf("\n");
    }
}


void ADC_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT); /* Clear the A/D interrupt flag */
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
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* ADC function test */
    ADC_FunctionTest();

    /* Disable ADC IP clock */
    CLK_DisableModuleClock(ADC_MODULE);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(ADC_IRQn);

    printf("Exit ADC sample code\n");

    while(1);
}

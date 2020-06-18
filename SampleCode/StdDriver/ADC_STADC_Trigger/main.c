/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to trigger ADC by STADC pin.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag, g_u32COVNUMFlag = 0;


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

    /* Set PB.2 - PB.3 to input mode */
    GPIO_SetMode(PB, BIT2|BIT3, GPIO_MODE_INPUT);
    /* Configure the PB.2 - PB.3 ADC analog input pins.  */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk)) |
                    (SYS_GPB_MFPL_PB2MFP_ADC0_CH2 | SYS_GPB_MFPL_PB3MFP_ADC0_CH3);
    /* Disable the PB.2 - PB.3 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT2|BIT3);

    /* Set PF.5 to input mode */
    GPIO_SetMode(PF, BIT5, GPIO_MODE_INPUT);
    /* Configure the PF.5 ADC trigger pins. */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & ~(SYS_GPF_MFPL_PF5MFP_Msk)) |
                    (SYS_GPF_MFPL_PF5MFP_ADC0_ST);

    /* Lock protected registers */
    SYS_LockReg();
}

void ADC_FunctionTest()
{
    uint8_t  u8Option;
    int32_t  i32ConversionData[6] = {0};

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                   ADC trigger by STADC pin test                      |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("\nIn this test, software will get 6 conversion result from the specified channel that triggered by STADC pin (PF.5).\n");

    /* Enable ADC converter */
    ADC_POWER_ON(ADC);

    while(1)
    {
        printf("Select input mode:\n");
        printf("  [1] Single end input (channel 2 only)\n");
        printf("  [2] Differential input (channel pair 1 only)\n");
        printf("  Other keys: exit single mode test\n");
        u8Option = getchar();
        if(u8Option == '1')
        {
            /* Set input mode as single-end, Single mode, and select channel 2 */
            ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE, BIT2);

            /* Configure the sample module and enable STADC pin trigger source */
            ADC_EnableHWTrigger(ADC, ADC_ADCR_TRGS_STADC, ADC_ADCR_TRGCOND_FALLING_EDGE);

            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

            /* Enable the sample module interrupt */
            ADC_ENABLE_INT(ADC, ADC_ADF_INT);  /* Enable sample module A/D interrupt. */
            NVIC_EnableIRQ(ADC_IRQn);

            printf("Wait falling edge from STADC pin (PF.5) ...\n");

            /* Reset the ADC indicator and wait falling edge on STADC pin */
            g_u32AdcIntFlag = 0;
            g_u32COVNUMFlag = 0;

            while(1)
            {
                /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
                while(g_u32AdcIntFlag == 0);

                /* Reset the ADC interrupt indicator */
                g_u32AdcIntFlag = 0;

                /* Get the conversion result of ADC channel 2 */
                i32ConversionData[g_u32COVNUMFlag - 1] = ADC_GET_CONVERSION_DATA(ADC, 2);

                if(g_u32COVNUMFlag >= 6)
                    break;
            }

            /* Disable the sample module interrupt */
            ADC_DISABLE_INT(ADC, ADC_ADF_INT);

            printf("Conversion result of channel pair 1:\n");
            for(g_u32COVNUMFlag = 0; (g_u32COVNUMFlag) < 6; g_u32COVNUMFlag++)
                printf("                                0x%X (%d)\n", i32ConversionData[g_u32COVNUMFlag], i32ConversionData[g_u32COVNUMFlag]);
        }
        else if(u8Option == '2')
        {
            /* Set input mode as differential, Single mode, and select channel 2 */
            ADC_Open(ADC, ADC_ADCR_DIFFEN_DIFFERENTIAL, ADC_ADCR_ADMD_SINGLE, BIT2);

            /* Configure the sample module and enable STADC pin trigger source */
            ADC_EnableHWTrigger(ADC, ADC_ADCR_TRGS_STADC, ADC_ADCR_TRGCOND_FALLING_EDGE);

            /* Clear the A/D interrupt flag for safe */
            ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

            /* Enable the sample module interrupt */
            ADC_ENABLE_INT(ADC, ADC_ADF_INT);  /* Enable sample module A/D interrupt. */
            NVIC_EnableIRQ(ADC_IRQn);

            printf("Wait falling edge from STADC pin (PF.5) ...\n");

            /* Reset the ADC indicator and wait falling edge on STADC pin */
            g_u32AdcIntFlag = 0;
            g_u32COVNUMFlag = 0;

            while(1)
            {
                /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
                while(g_u32AdcIntFlag == 0);

                /* Reset the ADC interrupt indicator */
                g_u32AdcIntFlag = 0;

                /* Get the conversion result of the sample module 0 */
                i32ConversionData[g_u32COVNUMFlag - 1] = ADC_GET_CONVERSION_DATA(ADC, 2);

                if(g_u32COVNUMFlag >= 6)
                    break;
            }

            /* Disable the sample module interrupt */
            ADC_DISABLE_INT(ADC, ADC_ADF_INT);

            printf("Conversion result of channel pair 1:\n");
            for(g_u32COVNUMFlag = 0; (g_u32COVNUMFlag) < 6; g_u32COVNUMFlag++)
                printf("                                0x%X (%d)\n", i32ConversionData[g_u32COVNUMFlag], i32ConversionData[g_u32COVNUMFlag]);
        }
        else
            return ;
    }
}

void ADC_IRQHandler(void)
{
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT); /* Clear the A/D interrupt flag */
    g_u32AdcIntFlag = 1;
    g_u32COVNUMFlag++;
    printf("[#%d] ADC conversion done.\n", g_u32COVNUMFlag);
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

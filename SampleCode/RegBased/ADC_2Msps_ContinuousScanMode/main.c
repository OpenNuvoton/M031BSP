/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate A/D conversion with ADC single mode in 2 Msps.
 *           The ADC clock source is 34 MHz comes from PLL 68 MHz.
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PLL_CLOCK 68000000 /* PLL = 68MHz, HCLK = PLL/2 */
#define PLL_HCLK PLL_CLOCK/2
/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag;

void SetClockPLL(uint32_t u32Hclk)
{
    uint32_t u32NO, u32NR, u32NF;

    /* Switch HCLK clock source to HIRC clock for safe */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 &= (~CLK_CLKDIV0_HCLKDIV_Msk);

    /* Disable PLL first to avoid unstable when setting PLL */
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Enable and apply new PLL setting. */
    /* FOUT = (FIN * NF) / (NR * NO)
       --> NF = (FOUT * NR * NO) / FIN
       FOUT = (u32Hclk * 2) since the HCLK divider will be 2 */
    u32NO = 4;
    u32NR = 3;
    u32NF = ((u32Hclk * 2) * u32NR * u32NO) / (__HIRC/4);
    CLK->PLLCTL = CLK_PLLCTL_PLLSRC_HIRC_DIV4 |         /* PLL source clock is from HIRC/4 */
                  (3 << CLK_PLLCTL_OUTDIV_Pos) |        /* assign 3 for NO = 4 */
                  ((u32NR - 2) << CLK_PLLCTL_INDIV_Pos) |
                  ((u32NF - 2) << CLK_PLLCTL_FBDIV_Pos);

    /* Wait for PLL clock stable */
    while((CLK->STATUS & CLK_STATUS_PLLSTB_Msk) != CLK_STATUS_PLLSTB_Msk);

    SystemCoreClockUpdate();

    return;
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

    /* Switch UART0 clock source to HIRC */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART0SEL_Msk) | CLK_CLKSEL1_UART0SEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_UART0DIV_Msk) | CLK_CLKDIV0_UART0(1);

    /* Enable UART0 peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Set core clock as PLL_CLOCK from PLL */
    SetClockPLL(PLL_HCLK);

    /* Switch ADC clock source to PLL */
    CLK->CLKSEL2 = (CLK->CLKSEL1 & ~CLK_CLKSEL2_ADCSEL_Msk) | CLK_CLKSEL2_ADCSEL_PLL;

    /* Enable ADC module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_ADCCKEN_Msk;

    /* ADC clock source is 68 MHz from PLL, set divider to 2, ADC clock is 68/2 MHz */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_ADCSEL_Msk) | CLK_CLKSEL2_ADCSEL_PLL;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_ADCDIV_Msk) | CLK_CLKDIV0_ADC(2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate Pll Clock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))) \
                     | (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD));

    /* Set PB.2 ~ PB.3 to input mode */
    PB->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Configure the GPB2 - GPB3 ADC analog input pins.  */
    SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk))) \
                     | (SYS_GPB_MFPL_PB2MFP_ADC_CH2 | SYS_GPB_MFPL_PB3MFP_ADC_CH3));

    /* Disable the GPB2 digital input path to avoid the leakage current. */
    PB->DINOFF |= ((BIT3|BIT2)<<GPIO_DINOFF_DINOFF0_Pos);

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
    printf("|       Demonstrate how to perform the ADC in 2 Msps single mode .     |\n");
    printf("|       ADC clock = PLL/2 = 68/2 MHz = 34 MHz                          |\n");
    printf("|       ADC conversion rate = 34 MHz / 17 = 2 Msps                     |\n");
    printf("+----------------------------------------------------------------------+\n");


    /* Enable ADC converter */
    ADC->ADCR |= ADC_ADCR_ADEN_Msk;

    /* Do calibration for ADC to decrease the effect of electrical random noise. */
    ADC->ADCALSTSR |= ADC_ADCALSTSR_CALIF_Msk;  /* Clear Calibration Finish Interrupt Flag */
    ADC->ADCALR |= ADC_ADCALR_CALEN_Msk;        /* Enable Calibration function */
    ADC_START_CONV(ADC);                        /* Start to calibration */
    while((ADC->ADCALSTSR & ADC_ADCALSTSR_CALIF_Msk) != ADC_ADCALSTSR_CALIF_Msk);

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
            ADC->ADCR = (ADC->ADCR & ~(ADC_ADCR_DIFFEN_Msk | ADC_ADCR_ADMD_Msk)) |
                        (ADC_ADCR_DIFFEN_SINGLE_END | ADC_ADCR_ADMD_CONTINUOUS);
            ADC->ADCHER = (ADC->ADCHER & ~ADC_ADCHER_CHEN_Msk) | (BIT2);

            /* Clear the A/D interrupt flag for safe */
            ADC->ADSR0 = ADC_ADF_INT;

            /* Enable the sample module interrupt */
            ADC->ADCR |= ADC_ADCR_ADIE_Msk;
            NVIC_EnableIRQ(ADC_IRQn);

            /* Reset the ADC interrupt indicator and trigger sample module 0 to start A/D conversion */
            g_u32AdcIntFlag = 0;
            ADC->ADCR |= ADC_ADCR_ADST_Msk;

            /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
            while(g_u32AdcIntFlag == 0);

            /* Get the conversion result of ADC channel 2 */
            i32ConversionData = ADC_GET_CONVERSION_DATA(ADC, 2);
            printf("Conversion result of channel 2: 0x%X (%d)\n\n", i32ConversionData, i32ConversionData);
        }
        else if(u8Option == '2')
        {

            /* Set input mode as differential, Single mode, and select channel 2 */
            ADC->ADCR = (ADC->ADCR & ~(ADC_ADCR_DIFFEN_Msk | ADC_ADCR_ADMD_Msk)) |
                        (ADC_ADCR_DIFFEN_DIFFERENTIAL | ADC_ADCR_ADMD_CONTINUOUS);
            ADC->ADCHER = (ADC->ADCHER & ~ADC_ADCHER_CHEN_Msk) | (BIT2);

            /* Clear the A/D interrupt flag for safe */
            ADC->ADSR0 = ADC_ADF_INT;

            /* Enable the sample module interrupt */
            ADC->ADCR |= ADC_ADCR_ADIE_Msk;
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
            return;
    }
}


void ADC_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT); /* Clear the A/D interrupt flag */
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

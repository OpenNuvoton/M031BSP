/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Perform A/D Conversion with ADC single mode to simulate single cycle scan mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*----------------------------------------------------------------------*/
/* Define global variables and constants                                */
/*----------------------------------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag;

/* Define ADC_TOTAL_CH as the total number of ADC channels to convert */
#define ADC_TOTAL_CH    (4)
/* Define PDMA_CH_ADC as the PDMA channel that work with ADC */
#define PDMA_CH_ADC     (4)
typedef struct dma_desc_t
{
    uint32_t ctl;
    uint32_t src;
    uint32_t dest;
    uint32_t offset;
} DMA_DESC_T;
DMA_DESC_T DMA_ADC_DESC[(ADC_TOTAL_CH-1)*2+1];
uint32_t g_u32ValueTrue = 1;
uint32_t g_u32ADCR_PDMA;
uint32_t g_u32ADCHER[ADC_TOTAL_CH];


void PDMA_Init(uint32_t u32AdcTotalCh)
{
    uint32_t i = 0;

    /* Enable PDMA clock source */
    CLK->AHBCLK |= (CLK_AHBCLK_PDMACKEN_Msk);

    /* Open PDMA channel PDMA_CH_ADC */
    PDMA->DSCT[PDMA_CH_ADC].CTL = 0UL;
    PDMA->CHCTL |= (BIT0 << PDMA_CH_ADC);

    /* Initial PDMA scatter-gather table */
    for(i = 0; i < (u32AdcTotalCh-1); i++)
    {
        /* Table (i*2) to change ADC->ADCHER value to g_u32ADCHER[i+1] for next ADC channel and then reload next table*/
        DMA_ADC_DESC[i*2].ctl =
            ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |  /* Transfer count is 1 */
            PDMA_WIDTH_32 |     /* Transfer width is 32 bits (one word) */
            PDMA_SAR_FIX |      /* Source is fixed */
            PDMA_DAR_FIX |      /* Destination is fixed */
            PDMA_REQ_BURST |    /* Transfer type is burst transfer type */
            PDMA_BURST_1 |      /* Burst size is 1. No effect in single transfer type */
            PDMA_TBINTDIS_DISABLE | /* Disable transfer done and table empty interrupt */
            PDMA_OP_SCATTER;    /* Operation mode is scatter-gather mode */
        /* Configure source address */
        DMA_ADC_DESC[i*2].src = (uint32_t)&g_u32ADCHER[i+1];
        /* Configure destination address */
        DMA_ADC_DESC[i*2].dest = (uint32_t)&ADC->ADCHER;
        /* Configure next descriptor table address */
        DMA_ADC_DESC[i*2].offset = (uint32_t)&DMA_ADC_DESC[i*2+1] - (PDMA->SCATBA); /* next descriptor table is table (i*2)+1 */

        /* Table (i*2+1) to change ADC->ADCR value to g_u32ADCR_PDMA for ADC conversion enable and then wait next trigger*/
        DMA_ADC_DESC[i*2+1].ctl =
            ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |  /* Transfer count is 1 */
            PDMA_WIDTH_32 |     /* Transfer width is 32 bits (one word) */
            PDMA_SAR_FIX |      /* Source is fixed */
            PDMA_DAR_FIX |      /* Destination is fixed */
            PDMA_REQ_SINGLE |   /* Transfer type is single transfer type */
            PDMA_BURST_1 |      /* Burst size is 1. No effect in single transfer type */
            PDMA_TBINTDIS_DISABLE | /* Disable transfer done and table empty interrupt */
            PDMA_OP_SCATTER;    /* Operation mode is scatter-gather mode */
        /* Configure source address */
        DMA_ADC_DESC[i*2+1].src = (uint32_t)&g_u32ADCR_PDMA;
        /* Configure destination address */
        DMA_ADC_DESC[i*2+1].dest = (uint32_t)&ADC->ADCR;
        /* Configure next descriptor table address */
        DMA_ADC_DESC[i*2+1].offset = (uint32_t)&DMA_ADC_DESC[(i+1)*2] - (PDMA->SCATBA); /* next descriptor table is table (i+1)*2 */
    }

    /* Last table to change g_u32AdcIntFlag value to 1 */
    DMA_ADC_DESC[i*2].ctl =
        ((1 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |  /* Transfer count is 1 */
        PDMA_WIDTH_32 |     /* Transfer width is 32 bits(one word) */
        PDMA_SAR_FIX |      /* Source is fixed */
        PDMA_DAR_FIX |      /* Destination IS FIXED */
        PDMA_REQ_SINGLE |   /* Transfer type is single transfer type */
        PDMA_BURST_1 |      /* Burst size is 1. No effect in single transfer type */
        PDMA_OP_BASIC;      /* Operation mode is basic mode */
    /* Configure source address */
    DMA_ADC_DESC[i*2].src = (uint32_t)&g_u32ValueTrue;
    /* Configure destination address */
    DMA_ADC_DESC[i*2].dest = (uint32_t)&g_u32AdcIntFlag;
    /* Configure next descriptor table address */
    DMA_ADC_DESC[i*2].offset = 0; /* No next descriptor table */

    /* Enable Scatter Gather mode, assign the first scatter-gather descriptor table is table 0,
       and set PDMA trigger by ADC conversion finish. */
    PDMA->REQSEL4_7 = (PDMA->REQSEL4_7 & ~PDMA_REQSEL4_7_REQSRC4_Msk) | (PDMA_ADC_RX);
    PDMA->DSCT[PDMA_CH_ADC].CTL = (PDMA->DSCT[PDMA_CH_ADC].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | PDMA_OP_SCATTER;
    PDMA->DSCT[PDMA_CH_ADC].NEXT = ((uint32_t)&DMA_ADC_DESC[0]) - (PDMA->SCATBA);
}

void ADC_Init(uint32_t u32AdcTotalCh, uint32_t u32InputMode)
{
    uint32_t i;
    volatile uint32_t tmp;

    /* Initial PDMA to work with ADC single mode to simulate ADC single cycle scan mode */
    PDMA_Init(u32AdcTotalCh);

    /* Set input mode as u32InputMode, single mode, and select first channel g_u32ADCHER[0] */
    ADC->ADCR = (ADC->ADCR & (~(ADC_ADCR_DIFFEN_Msk | ADC_ADCR_ADMD_Msk))) |
                (u32InputMode) | (ADC_ADCR_ADMD_SINGLE);
    ADC->ADCHER = (ADC->ADCHER & ~ADC_ADCHER_CHEN_Msk) | (g_u32ADCHER[0]);

    /* Read out valid conversion data to avoid trigger PDMA in unexpect timing */
    if(ADC->ADSR1)
    {
        for(i = 0; i < 32; i++)
        {
            if ((ADC->ADSR1 & (BIT0<<i)) ? 1 : 0)
                tmp = (ADC->ADDR[i] & ADC_ADDR_RSLT_Msk);
        }
    }

    /* ADC enable PDMA transfer */
    ADC->ADCR |= ADC_ADCR_PTEN_Msk;

    /* Set ADC control register ADCR for each trigger from PDMA */
    g_u32ADCR_PDMA = ADC->ADCR | (ADC_ADCR_ADST_Msk | ADC_ADCR_PTEN_Msk | ADC_ADCR_ADEN_Msk);
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

    /* Set PB.0 ~ PB.3 to input mode */
    PB->MODE &= ~(GPIO_MODE_MODE0_Msk | GPIO_MODE_MODE1_Msk | GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);
    /* Configure the PB.0 - PB.3 ADC analog input pins. */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk)) |
                    (SYS_GPB_MFPL_PB0MFP_ADC0_CH0 | SYS_GPB_MFPL_PB1MFP_ADC0_CH1 | SYS_GPB_MFPL_PB2MFP_ADC0_CH2 | SYS_GPB_MFPL_PB3MFP_ADC0_CH3);

    /* Disable the GPB0 - GPB3 digital input path to avoid the leakage current. */
    PB->DINOFF |= ((BIT3|BIT2|BIT1|BIT0)<<GPIO_DINOFF_DINOFF0_Pos);

    /* Lock protected registers */
    SYS_LockReg();
}

void ADC_FunctionTest()
{
    uint8_t  u8Option;
    uint32_t u32ChannelCount;
    int32_t  i32ConversionData;
    volatile uint32_t tmp;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|    ADC single mode to simulate single cycle scan mode sample code    |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("     ADC single cycle scan mode can convert multiple channels by one ADC_START_CONV().\n");
    printf("     ADC single mode work with PDMA also can convert multiple channels by one ADC_START_CONV().\n");
    printf("     For example, ADC start channel 0 conversion by one ADC_START_CONV(). Once the conversion is\n");
    printf("       completed, ADC trigger PDMA to reset the ADC for the next ADC channel conversion. Repeat\n");
    printf("       the same steps until the last ADC channel conversion is completed and the last PDMA set\n");
    printf("       the flag g_u32AdcIntFlag to 1 to indicate all ADC channel conversions are completed.\n\n");

    /* Enable ADC converter */
    ADC->ADCR |= ADC_ADCR_ADEN_Msk;

    /* Do calibration for ADC to decrease the effect of electrical random noise. */
    ADC->ADCALSTSR |= ADC_ADCALSTSR_CALIF_Msk;  /* Clear Calibration Finish Interrupt Flag */
    ADC->ADCALR |= ADC_ADCALR_CALEN_Msk;        /* Enable Calibration function */
    ADC_START_CONV(ADC);                        /* Start to calibration */
    while((ADC->ADCALSTSR & ADC_ADCALSTSR_CALIF_Msk) != ADC_ADCALSTSR_CALIF_Msk);   /* Wait calibration finish */
    /* Read channel 0 ADDR to clear Valid flag of channel 0 that set by calibration. */
    tmp = ADC->ADDR[0];

    while(1)
    {
        printf("Select input mode:\n");
        printf("  [1] Single end input (channel 0, 1, 2 and 3)\n");
        printf("  [2] Differential input (input channel pair 0 and 1)\n");
        printf("  Other keys: exit test\n");
        u8Option = getchar();

        if(u8Option == '1')
        {
            /* Set ADC and PDMA to simulate ADC single cycle scan mode by ADC single mode. */
            /* MUST define g_u32ADCHER[] for each ADC channel that will be converted by ADC single mode */
            g_u32ADCHER[0] = BIT0;
            g_u32ADCHER[1] = BIT1;
            g_u32ADCHER[2] = BIT2;
            g_u32ADCHER[3] = BIT3;
            ADC_Init(ADC_TOTAL_CH, ADC_ADCR_DIFFEN_SINGLE_END);

            /* Clear the A/D interrupt flag for safe */
            ADC->ADSR0 = ADC_ADF_INT;

            /* Reset the flag g_u32AdcIntFlag and start A/D conversion */
            g_u32AdcIntFlag = 0;
            ADC->ADCR |= ADC_ADCR_ADST_Msk;

            /* Wait flag g_u32AdcIntFlag become 1. g_u32AdcIntFlag will be set at last PDMA transfer data. */
            while(g_u32AdcIntFlag == 0);

            /* Clear the A/D interrupt flag for safe */
            ADC->ADSR0 = ADC_ADF_INT;

            /* Get the conversion result */
            for(u32ChannelCount = 0; u32ChannelCount < ADC_TOTAL_CH; u32ChannelCount++)
            {
                i32ConversionData = (ADC->ADDR[u32ChannelCount] & ADC_ADDR_RSLT_Msk);
                printf("Conversion result of channel %d: 0x%X (%d)\n", u32ChannelCount, i32ConversionData, i32ConversionData);
            }
        }
        else if(u8Option == '2')
        {
            /* Set ADC and PDMA to simulate ADC single cycle scan mode by ADC single mode. */
            /* MUST define g_u32ADCHER[] for each ADC channel that will be converted by ADC single mode */
            g_u32ADCHER[0] = BIT0;
            g_u32ADCHER[1] = BIT2;
            ADC_Init(ADC_TOTAL_CH/2, ADC_ADCR_DIFFEN_DIFFERENTIAL);

            /* Clear the A/D interrupt flag for safe */
            ADC->ADSR0 = ADC_ADF_INT;

            /* Reset the flag g_u32AdcIntFlag and start A/D conversion */
            g_u32AdcIntFlag = 0;
            ADC->ADCR |= ADC_ADCR_ADST_Msk;

            /* Wait flag g_u32AdcIntFlag become 1. g_u32AdcIntFlag will be set at last PDMA transfer data. */
            while(g_u32AdcIntFlag == 0);

            /* Clear the A/D interrupt flag for safe */
            ADC->ADSR0 = ADC_ADF_INT;

            /* Get the conversion result */
            for(u32ChannelCount = 0; u32ChannelCount < (ADC_TOTAL_CH/2); u32ChannelCount++)
            {
                i32ConversionData = (ADC->ADDR[u32ChannelCount * 2] & ADC_ADDR_RSLT_Msk);
                printf("Conversion result of differential input pair %d: 0x%X (%d)\n", u32ChannelCount, i32ConversionData, i32ConversionData);
            }
        }
        else
            return;

        printf("\n");
    }
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

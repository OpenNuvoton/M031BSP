/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to use PLL as ADC clock source to achieve 2 Msps ADC conversion rate.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PLL_CLOCK 68000000 /* PLL = 68MHz, HCLK = PLL/2 */
#define PLL_HCLK PLL_CLOCK/2
#define PDMA_CH     1

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
typedef struct dma_desc_t
{
    uint32_t ctl;
    uint32_t src;
    uint32_t dest;
    uint32_t offset;
} DMA_DESC_T;
DMA_DESC_T DMA_DESC[2];

uint32_t g_u32DMAConfig = 0;


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

    /* Enable PDMA clock source */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;

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
                     | (SYS_GPB_MFPL_PB2MFP_ADC0_CH2 | SYS_GPB_MFPL_PB3MFP_ADC0_CH3));

    /* Disable the GPB2 digital input path to avoid the leakage current. */
    PB->DINOFF |= ((BIT3|BIT2)<<GPIO_DINOFF_DINOFF0_Pos);

    /* Set PA.0 ~ PA11 to GPIO output mode */
    PA->MODE = (PA->MODE & ~(0x00FFFFFF)) | 0x00555555;
    SYS->GPA_MFPL = 0;
    SYS->GPA_MFPH = SYS->GPA_MFPH & (~0xFFFF);

    /* Lock protected registers */
    SYS_LockReg();
}


void PDMA_Init()
{
    /* Configure PDMA to Scatter Gather mode with ping-pong buffer */
    /* to move ADC conversion data to GPIO output without PDMA interrupt. */

    /* Open Channel 1 */
    PDMA->CHCTL |= (1 << PDMA_CH);

    /* Enable Scatter Gather mode, assign the first scatter-gather descriptor table is table 1,
       and set transfer mode as ADC_RX to GPIO */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC1_Msk) | (PDMA_ADC_RX << PDMA_REQSEL0_3_REQSRC1_Pos);
    PDMA->DSCT[PDMA_CH].CTL = PDMA_OP_SCATTER;
    PDMA->DSCT[PDMA_CH].NEXT = (uint32_t)&DMA_DESC[0] - (PDMA->SCATBA);

    /* Scatter-Gather descriptor table configuration */
    g_u32DMAConfig = \
                     (1 << PDMA_DSCT_CTL_TXCNT_Pos) |   /* Transfer count is 2 */
                     PDMA_WIDTH_16 |    /* Transfer width is 16 bits */
                     PDMA_SAR_FIX |     /* Source increment size is fixed (no increment) */
                     PDMA_DAR_FIX |     /* Destination increment size is fixed (no increment) */
                     PDMA_REQ_SINGLE |  /* Transfer type is single transfer type */
                     PDMA_BURST_1 |     /* Burst size is 128. No effect in single transfer type */
                     PDMA_OP_SCATTER;   /* Operation mode is scatter-gather mode */

    DMA_DESC[0].ctl = g_u32DMAConfig;
    /* Configure source address */
    DMA_DESC[0].src = (uint32_t)&ADC->ADPDMA;   /* Ping-Pong buffer 1 */
    /* Configure destination address */
    DMA_DESC[0].dest = (uint32_t)&PA->DOUT;
    /* Configure next descriptor table address */
    DMA_DESC[0].offset = (uint32_t)&DMA_DESC[1] - (PDMA->SCATBA);   /* next operation table is table 2 */

    DMA_DESC[1].ctl = g_u32DMAConfig;
    /* Configure source address */
    DMA_DESC[1].src = (uint32_t)&ADC->ADPDMA;   /* Ping-Pong buffer 2 */
    /* Configure destination address */
    DMA_DESC[1].dest = (uint32_t)&PA->DOUT;
    /* Configure next descriptor table address */
    DMA_DESC[1].offset = (uint32_t)&DMA_DESC[0] - (PDMA->SCATBA);   /* next operation table is table 1 */

    /* Don't enable any interrupt to make ADC SPS can up to 2MHz */
    // PDMA_EnableInt(PDMA, PDMA_CH, PDMA_INT_TRANS_DONE);
    // NVIC_EnableIRQ(PDMA_IRQn);
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
    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|       Demonstrate how to perform the ADC in 2 Msps continuous mode.  |\n");
    printf("|       ADC clock = PLL/2 = 68/2 MHz = 34 MHz                          |\n");
    printf("|       ADC conversion rate = 34 MHz / 17 = 2 Msps                     |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("   ADC conversion data will be moved to GPIO pins PA11 ~ PA0 by PDMA.\n");
    printf("   Please connect PB2 to 0V and PB3 to 3.3V\n");
    printf("   and monitor PA11 (MSB of 12-bit ADC conversion data) on scope.\n");
    printf("   The real ADC SPS shoule be (PA11 frequency * 2).\n");

    /* Enable ADC converter */
    ADC->ADCR |= ADC_ADCR_ADEN_Msk;

    /* Do calibration for ADC to decrease the effect of electrical random noise. */
    ADC->ADCALSTSR |= ADC_ADCALSTSR_CALIF_Msk;  /* Clear Calibration Finish Interrupt Flag */
    ADC->ADCALR |= ADC_ADCALR_CALEN_Msk;        /* Enable Calibration function */
    ADC_START_CONV(ADC);                        /* Start to calibration */
    while((ADC->ADCALSTSR & ADC_ADCALSTSR_CALIF_Msk) != ADC_ADCALSTSR_CALIF_Msk);

    /* Set input mode as single-end, continuous mode, and select channel 2 and 3 */
    ADC->ADCR = (ADC->ADCR & (~(ADC_ADCR_DIFFEN_Msk | ADC_ADCR_ADMD_Msk))) |
                (ADC_ADCR_DIFFEN_SINGLE_END) | (ADC_ADCR_ADMD_CONTINUOUS);
    ADC->ADCHER = (ADC->ADCHER & ~ADC_ADCHER_CHEN_Msk) | (BIT2|BIT3);

    /* ADC enable PDMA transfer */
    ADC_ENABLE_PDMA(ADC);

    /* Start ADC conversion */
    ADC_START_CONV(ADC);

    /* Don't interrupt ADC or PDMA in order to make ADC SPS can up to 2MHz */

    printf("press any key to stop ADC conversion ...\n");
    getchar();
    ADC_STOP_CONV(ADC);

    return;
}


int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init PDMA for ADC */
    PDMA_Init();

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* ADC function test */
    ADC_FunctionTest();

    /* Disable ADC IP clock */
    CLK->APBCLK0 &= ~(CLK_APBCLK0_ADCCKEN_Msk);

    /* Disable PDMA clock source */
    CLK->AHBCLK &= ~(CLK_AHBCLK_PDMACKEN_Msk);

    printf("Exit ADC sample code\n");

    while(1);
}

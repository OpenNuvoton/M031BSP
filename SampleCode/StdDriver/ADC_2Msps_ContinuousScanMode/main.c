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

#define PLL_CLOCK   68000000
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


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable PLL module clock from HIRC */
    CLK_EnablePLL(CLK_PLLCTL_PLLSRC_HIRC_DIV4, PLL_CLOCK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable ADC module clock */
    CLK_EnableModuleClock(ADC_MODULE);

    /* ADC clock source is 68 MHz from PLL, set divider to 2, ADC clock is 68/2 MHz */
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL2_ADCSEL_PLL, CLK_CLKDIV0_ADC(2));

    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate Pll Clock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)))
                     | (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD));

    /* Configure the GPB2 - GPB3 ADC analog input pins.  */
    GPIO_SetMode(PB, BIT2, GPIO_MODE_INPUT);
    GPIO_SetMode(PB, BIT3, GPIO_MODE_INPUT);
    SYS->GPB_MFPL = ((SYS->GPB_MFPL & (~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk)))
                     | (SYS_GPB_MFPL_PB2MFP_ADC0_CH2 | SYS_GPB_MFPL_PB3MFP_ADC0_CH3));

    /* Disable the GPB2 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT2|BIT3);

    /* Set PA.0 ~ PA11 to GPIO output mode */
    GPIO_SetMode(PA, 0x0FFF, GPIO_MODE_OUTPUT);
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
    PDMA_Open(PDMA, 1 << PDMA_CH);

    /* Enable Scatter Gather mode, assign the first scatter-gather descriptor table is table 1,
       and set transfer mode as ADC_RX to GPIO */
    PDMA_SetTransferMode(PDMA, PDMA_CH, PDMA_ADC_RX, TRUE, (uint32_t)&DMA_DESC[0]);

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
    ADC_POWER_ON(ADC);

    /* Set input mode as single-end, continuous mode, and select channel 2 and 3 */
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_CONTINUOUS, BIT2|BIT3);

    /* ADC enable PDMA transfer */
    ADC_ENABLE_PDMA(ADC);

    /* Start ADC conversion */
    ADC_START_CONV(ADC);

    /* Don't interrupt ADC or PDMA in order to make ADC SPS can up to 2MHz */

    printf("press any key to stop ADC conversion ...\n");
    getchar();
    ADC_STOP_CONV(ADC);
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

    /* Init PDMA for ADC */
    PDMA_Init();

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* ADC function test */
    ADC_FunctionTest();

    /* Disable ADC IP clock */
    CLK_DisableModuleClock(ADC_MODULE);

    /* Disable PDMA clock source */
    CLK_DisableModuleClock(PDMA_MODULE);

    printf("Exit ADC sample code\n");

    while(1);
}

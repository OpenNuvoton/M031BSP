/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 18/06/01 11:45a $
 * @brief    M031 SPI Driver Sample Code
 *           This is a I2S demo for recording data and demonstrate how I2S works with PDMA.
 *
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define PLLCTL_SETTING      CLK_PLLCTL_96MHz_HXT

#define I2S_TX_DMA_CH 0
#define I2S_RX_DMA_CH 1

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define BUFF_LEN 16
#define TX_BUFF_LEN 32

typedef struct
{
    uint32_t CTL;
    uint32_t SA;
    uint32_t DA;
    uint32_t FIRST;
} DESC_TABLE_T;

DESC_TABLE_T g_asDescTable_DataTX[1], g_asDescTable_RX[2];

/* Function prototype declaration */
void SYS_Init(void);
void UART_Init(void);

/* Global variable declaration */
volatile uint8_t u8RxIdx = 0;
uint32_t PcmRxBuff[2][BUFF_LEN] = {0};
uint32_t PcmTxDataBuff[1][TX_BUFF_LEN] = {0};

/* Since ping-pong buffer would record infinitely, in this case we only want to make sure the first buffer of RX Buffer1 and Buffer2 are correct.
   Hence use g_PcmRxBuff and g_count to check and record the first buffer of RX Buffer1 and Buffer2*/
uint32_t g_PcmRxBuff[2][BUFF_LEN] = {0};
volatile uint8_t g_count = 0;

/* Once PDMA has transferred, software need to reset Scatter-Gather table */
void PDMA_ResetRxSGTable(uint8_t id)
{
    g_asDescTable_RX[id].CTL |= PDMA_OP_SCATTER;
    g_asDescTable_RX[id].CTL |= ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32InitValue, u32DataCount;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init system, peripheral clock and multi-function I/O */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART for print message */
    UART_Init();

    printf("\n");
    printf("+----------------------------------------------+\n");
    printf("|        I2S + PDMA  Record Sample Code        |\n");
    printf("+----------------------------------------------+\n");
    printf("  I2S configuration:\n");
    printf("      Sample rate 16 kHz\n");
    printf("      Word width 16 bits\n");
    printf("      Stereo mode\n");
    printf("      I2S format\n");
    printf("      TX value: 0x50005000, 0x50015001, ... \n");
    printf("  The I/O connection for I2S0 (SPI0):\n");
    printf("      I2S0_MCLK (PA.4)\n      I2S0_LRCLK (PA.3)\n      I2S0_BCLK (PA.2)\n");
    printf("      I2S0_DI (PA.1)\n      I2S0_DO (PA.0)\n\n");
    printf("      This sample code will transmit and receive data with PDMA transfer.\n");
    printf("      Connect I2S_DI and I2S_DO to check if the data which stored in two receive\n");
    printf("      buffers are the same with the transmitted values.\n");
    printf("      After PDMA transfer is finished, the received values will be printed.\n\n");

    /* Select PCLK as the clock source of SPI0 */
//    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI0SEL_Msk)) | CLK_CLKSEL2_SPI0SEL_PCLK0;

    /* Reset SPI/I2S */
    SYS->IPRST1 |= SYS_IPRST1_SPI0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_SPI0RST_Msk;

    /* Master mode, 16-bit word width, stereo mode, I2S format. */
    SPI0->I2SCTL = SPII2S_MODE_MASTER | SPII2S_DATABIT_16 | SPII2S_STEREO | SPII2S_FORMAT_I2S;
    /* Sampling rate 16000 Hz; bit clock rate 512 kHz. */
    SPI0->I2SCLK = (SPI0->I2SCLK & ~SPI_I2SCLK_BCLKDIV_Msk) | (11 << SPI_I2SCLK_BCLKDIV_Pos);
    /* Enable I2S */
    SPI0->I2SCTL |= SPI_I2SCTL_I2SEN_Msk;

    /* Data initiation */
    u32InitValue = 0x50005000;
    for(u32DataCount = 0; u32DataCount < TX_BUFF_LEN; u32DataCount++)
    {
        PcmTxDataBuff[0][u32DataCount] = u32InitValue;
        u32InitValue += 0x00010001;
    }

    /* Enable PDMA channels */
    PDMA->DSCT[0].CTL = 0;
    PDMA->DSCT[1].CTL = 0;
    PDMA->CHCTL |= (1 << I2S_TX_DMA_CH) | (1 << I2S_RX_DMA_CH);

    /* Tx description */
    g_asDescTable_DataTX[0].CTL = ((TX_BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_BASIC;
    g_asDescTable_DataTX[0].SA = (uint32_t)&PcmTxDataBuff[0];
    g_asDescTable_DataTX[0].DA = (uint32_t)&SPI0->TX;

    /* Rx(Record) description */
    g_asDescTable_RX[0].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    g_asDescTable_RX[0].SA = (uint32_t)&SPI0->RX;
    g_asDescTable_RX[0].DA = (uint32_t)&PcmRxBuff[0];
    g_asDescTable_RX[0].FIRST = (uint32_t)&g_asDescTable_RX[1] - (PDMA->SCATBA);

    g_asDescTable_RX[1].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    g_asDescTable_RX[1].SA = (uint32_t)&SPI0->RX;
    g_asDescTable_RX[1].DA = (uint32_t)&PcmRxBuff[1];
    g_asDescTable_RX[1].FIRST = (uint32_t)&g_asDescTable_RX[0] - (PDMA->SCATBA);   //link to first description

    /* Configure PDMA transfer mode */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~(0x3Ful << 0)) | (PDMA_SPI0_TX << 0);
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~(0x3Ful << 8)) | (PDMA_SPI0_RX << 8);

    PDMA->DSCT[0].CTL = (PDMA->DSCT[0].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | PDMA_OP_SCATTER;
    PDMA->DSCT[0].NEXT = (uint32_t)&g_asDescTable_DataTX[0] - (PDMA->SCATBA);

    PDMA->DSCT[1].CTL = (PDMA->DSCT[1].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | PDMA_OP_SCATTER;
    PDMA->DSCT[1].NEXT = (uint32_t)&g_asDescTable_RX[0] - (PDMA->SCATBA);

    /* Enable PDMA channel 1 interrupt */
    PDMA->INTEN |= (1 << I2S_RX_DMA_CH);

    NVIC_EnableIRQ(PDMA_IRQn);

    /* Clear RX FIFO */
    SPII2S_CLR_RX_FIFO(SPI0);

    /* Enable RX function and TX function */
    SPI0->I2SCTL |= (SPI_I2SCTL_RXEN_Msk | SPI_I2SCTL_TXEN_Msk);
    /* Enable RX PDMA and TX PDMA function */
    SPI0->PDMACTL = (SPI_PDMACTL_RXPDMAEN_Msk | SPI_PDMACTL_TXPDMAEN_Msk);

    /* wait RX Buffer 1 and RX Buffer 2 get first buffer */
//    while (g_count != 2);
    /* wait RX Buffer 1 and RX Buffer 2 get first buffer */
    while(g_count<2);

    /* Once I2S is enabled, CLK would be sent immediately, we still have chance to get zero data at beginning,
       in this case we only need to make sure the on-going data is correct */
    printf("RX Buffer 1\tRX Buffer 2\n");

    /* Print the received data */
    for (u32DataCount = 0; u32DataCount < BUFF_LEN; u32DataCount++)
    {
        printf("0x%X\t\t0x%X\n", g_PcmRxBuff[0][u32DataCount], g_PcmRxBuff[1][u32DataCount]);
    }

    printf("\n\nExit I2S sample code.\n");

    while(1);

}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~32 MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for 32MHz clock ready */
    while((CLK->STATUS & CLK_STATUS_HXTSTB_Msk) != CLK_STATUS_HXTSTB_Msk);

    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(1);

    /* Switch UART0 clock source to XTAL */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART0SEL_Msk) | CLK_CLKSEL1_UART0SEL_HXT;

    /* Switch STCLK source to HCLK/2 */
    CLK->CLKSEL0 = CLK_CLKSEL0_STCLKSEL_HCLK_DIV2;

    /* System optimization when CPU runs at 48 MHz */
    FMC->FTCTL |= 0x50;

    /* Select PCLK1 as the clock source of SPI0 */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI0SEL_Msk)) | CLK_CLKSEL2_SPI0SEL_PCLK1;

    /* Enable UART and SPI0 clock */
    CLK->APBCLK0 = CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_SPI0CKEN_Msk;

    /* Enable PDMA peripheral clock */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD;

    /* Setup SPI0 multi-function pins */
    /* PA.4 is SPI0_I2SMCLK */
    SYS->GPA_MFPL &= ~SYS_GPA_MFPL_PA4MFP_Msk;
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA4MFP_SPI0_I2SMCLK;
    /* PA.3 is SPI0_SS (I2S_LRCLK) */
    SYS->GPA_MFPL &= ~SYS_GPA_MFPL_PA3MFP_Msk;
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA3MFP_SPI0_SS;
    /* PA.2 is SPI0_CLK (I2S_BCLK) */
    SYS->GPA_MFPL &= ~SYS_GPA_MFPL_PA2MFP_Msk;
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA2MFP_SPI0_CLK;
    /* PA.1 is SPI0_MISO (I2S_DI) */
    SYS->GPA_MFPL &= ~SYS_GPA_MFPL_PA1MFP_Msk;
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA1MFP_SPI0_MISO;
    /* PA.0 is SPI0_MOSI (I2S_DO) */
    SYS->GPA_MFPL &= ~SYS_GPA_MFPL_PA0MFP_Msk;
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA0MFP_SPI0_MOSI;
}

void UART_Init(void)
{
    /* Word length is 8 bits; 1 stop bit; no parity bit. */
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
    /* UART peripheral clock rate 12MHz; UART bit rate 115200 bps. */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
}

void PDMA_IRQHandler(void)
{
    uint32_t u32DataCount = 0;
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA);

    if(u32Status & 0x1)    /* abort */
    {
        if(PDMA_GET_ABORT_STS(PDMA) & 0x4)
            PDMA_CLR_ABORT_FLAG(PDMA, PDMA_ABTSTS_ABTIF2_Msk);
    }
    else if(u32Status & 0x2)
    {
        if(PDMA_GET_TD_STS(PDMA) & 0x2)             /* channel 1 done */
        {
            /* record the first buffer */
            if (g_count == 0)
            {
                for (u32DataCount = 0; u32DataCount < BUFF_LEN; u32DataCount++)
                    g_PcmRxBuff[0][u32DataCount] = PcmRxBuff[0][u32DataCount];
            }
            else if (g_count == 1)
            {
                for (u32DataCount = 0; u32DataCount < BUFF_LEN; u32DataCount++)
                    g_PcmRxBuff[1][u32DataCount] = PcmRxBuff[1][u32DataCount];
            }

            ++g_count;
            /* Reset PDMA Scater-Gatter table */
            PDMA_ResetRxSGTable(u8RxIdx);
            u8RxIdx ^= 1;
        }
        PDMA_CLR_TD_FLAG(PDMA, PDMA_TDSTS_TDIF1_Msk);
    }
    else
        printf("unknown interrupt, status=0x%x!!\n", u32Status);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/

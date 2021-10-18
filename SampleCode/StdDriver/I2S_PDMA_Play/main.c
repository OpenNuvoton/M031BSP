/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 15 $
 * $Date: 18/07/20 5:18p $
 * @brief    M031 SPI Driver Sample Code
 *           This is a I2S demo for playing data and demonstrate how I2S works with PDMA.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define I2S_TX_DMA_CH   0
#define I2S_RX_DMA_CH   1
#define I2S_OPENED_CH   ((1 << I2S_TX_DMA_CH) | (1 << I2S_RX_DMA_CH))
#define TEST_PATTERN    0x50005000

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define BUFF_LEN 4
#define CHECK_BUFF_LEN 32

typedef struct
{
    uint32_t CTL;
    uint32_t SA;
    uint32_t DA;
    uint32_t FIRST;
} DESC_TABLE_T;

DESC_TABLE_T g_asDescTable_TX[2], g_asDescTable_DataRX[1];

/* Function prototype declaration */
void SYS_Init(void);

/* Global variable declaration */
volatile uint8_t u8TxIdx = 0;
volatile uint32_t u32PlayReady = 0;
uint32_t PcmRxDataBuff[1][CHECK_BUFF_LEN] = {0};
uint32_t PcmTxBuff[2][BUFF_LEN] = {0};

/* Once PDMA has transferred, software need to reset Scatter-Gather table */
void PDMA_ResetTxSGTable(uint8_t id)
{
    g_asDescTable_TX[id].CTL |= PDMA_OP_SCATTER;
    g_asDescTable_TX[id].CTL |= ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32InitValue, u32DataCount;
    uint32_t u32TimeOutCount;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init system, peripheral clock and multi-function I/O */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("\n");
    printf("+----------------------------------------------+\n");
    printf("|         I2S + PDMA  Play Sample Code         |\n");
    printf("+----------------------------------------------+\n");
    printf("  I2S configuration:\n");
    printf("      Sample rate 16 kHz\n");
    printf("      Word width 16 bits\n");
    printf("      Stereo mode\n");
    printf("      I2S format\n");
    printf("      TX 1/2 value: 0x50005000/0xA000A000, 0x50015001/0xA001A001, ... \n");
    printf("  The I/O connection for I2S0 (SPI0):\n");
    printf("      I2S0_MCLK (PA.4)\n      I2S0_LRCLK (PA.3)\n      I2S0_BCLK (PA.2)\n");
    printf("      I2S0_DI (PA.1)\n      I2S0_DO (PA.0)\n\n");
    printf("      This sample code will transmit and receive %d data with PDMA transfer.\n", CHECK_BUFF_LEN);
    printf("      Connect I2S_DI and I2S_DO to check if the received values and its' sequence\n");
    printf("      are the same with the data which stored in two transmit buffers.\n");
    printf("      After PDMA transfer is finished, the received values will be printed.\n\n");

    /* Enable I2S TX and RX functions */
    /* Sampling rate 16000 Hz; bit clock rate 512 kHz. */
    /* Master mode, 16-bit word width, stereo mode, I2S format. */
    SPII2S_Open(SPI0, SPII2S_MODE_MASTER, 16000, SPII2S_DATABIT_16, SPII2S_STEREO, SPII2S_FORMAT_I2S);

    /* Data initiation */
    u32InitValue = TEST_PATTERN;
    for(u32DataCount = 0; u32DataCount < BUFF_LEN; u32DataCount++)
    {
        PcmTxBuff[0][u32DataCount] = u32InitValue;
        PcmTxBuff[1][u32DataCount] = u32InitValue + TEST_PATTERN;
        u32InitValue += 0x00010001;
    }

    /* Enable PDMA channels */
    PDMA_Open(PDMA, I2S_OPENED_CH);

    /* Tx(Play) description */
    g_asDescTable_TX[0].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    g_asDescTable_TX[0].SA = (uint32_t)&PcmTxBuff[0];
    g_asDescTable_TX[0].DA = (uint32_t)&SPI0->TX;
    g_asDescTable_TX[0].FIRST = (uint32_t)&g_asDescTable_TX[1] - (PDMA->SCATBA);

    g_asDescTable_TX[1].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    g_asDescTable_TX[1].SA = (uint32_t)&PcmTxBuff[1];
    g_asDescTable_TX[1].DA = (uint32_t)&SPI0->TX;
    g_asDescTable_TX[1].FIRST = (uint32_t)&g_asDescTable_TX[0] - (PDMA->SCATBA);   //link to first description

    /* Rx description */
    g_asDescTable_DataRX[0].CTL = ((CHECK_BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_BASIC;
    g_asDescTable_DataRX[0].SA = (uint32_t)&SPI0->RX;
    g_asDescTable_DataRX[0].DA = (uint32_t)&PcmRxDataBuff[0];

    PDMA_SetTransferMode(PDMA, I2S_TX_DMA_CH, PDMA_SPI0_TX, 1, (uint32_t)&g_asDescTable_TX[0]);
    PDMA_SetTransferMode(PDMA, I2S_RX_DMA_CH, PDMA_SPI0_RX, 1, (uint32_t)&g_asDescTable_DataRX[0]);

    /* Enable PDMA channel 0 interrupt */
    PDMA_EnableInt(PDMA, I2S_TX_DMA_CH, PDMA_INT_TRANS_DONE);

    NVIC_EnableIRQ(PDMA_IRQn);

    /* Clear TX FIFO */
    SPII2S_CLR_TX_FIFO(SPI0);

    /* setup timeout */
    u32TimeOutCount = SystemCoreClock;

    while(!SPI_GET_TX_FIFO_EMPTY_FLAG(SPI0))
    {
        if(u32TimeOutCount == 0)
        {
            printf("SPI encounters some errors, please check it. \n");
            while(1);
        }
        u32TimeOutCount--;
    };

    /* Clear RX FIFO */
    SPII2S_CLR_RX_FIFO(SPI0);

    /* setup timeout */
    u32TimeOutCount = SystemCoreClock;

    while(!SPI_GET_RX_FIFO_EMPTY_FLAG(SPI0))
    {
        if(u32TimeOutCount == 0)
        {
            printf("SPI encounters some errors, please check it. \n");
            while(1);
        }
        u32TimeOutCount--;
    };

    /* Enable RX function and TX function */
    SPII2S_ENABLE_RX(SPI0);
    SPII2S_ENABLE_TX(SPI0);

    /* Enable RX PDMA and TX PDMA function */
    SPII2S_ENABLE_RXDMA(SPI0);
    SPII2S_ENABLE_TXDMA(SPI0);

    /* setup timeout */
    u32TimeOutCount = SystemCoreClock;

    /* Wait test finished */
    while(!u32PlayReady)
    {
        if(u32TimeOutCount == 0)
        {
            printf("Please check if PDMA setting is correct.\n");
            while(1);
        }
        u32TimeOutCount--;
    }

    /* Print the received data */
    for(u32DataCount = 0; u32DataCount < CHECK_BUFF_LEN; u32DataCount++)
    {
        printf("%d:\t0x%X\n", u32DataCount, PcmRxDataBuff[0][u32DataCount]);
    }

    printf("\n\nExit I2S sample code.\n");

    while(1);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Select HIRC as the clock source of UART0 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Select PCLK1 as the clock source of SPI0 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable SPI0 peripheral clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Enable PDMA peripheral clock */
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Setup SPI0 multi-function pins */
    /* PA.4 is SPI0_I2SMCLK,        PA.3 is SPI0_SS (I2S_LRCLK)
       PA.2 is SPI0_CLK (I2S_BCLK), PA.1 is SPI0_MISO (I2S_DI)
       PA.0 is SPI0_MOSI (I2S_DO) */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA4MFP_Msk |
                                       SYS_GPA_MFPL_PA3MFP_Msk |
                                       SYS_GPA_MFPL_PA2MFP_Msk |
                                       SYS_GPA_MFPL_PA1MFP_Msk |
                                       SYS_GPA_MFPL_PA0MFP_Msk)) |
                    (SYS_GPA_MFPL_PA4MFP_SPI0_I2SMCLK |
                     SYS_GPA_MFPL_PA3MFP_SPI0_SS |
                     SYS_GPA_MFPL_PA2MFP_SPI0_CLK |
                     SYS_GPA_MFPL_PA1MFP_SPI0_MISO |
                     SYS_GPA_MFPL_PA0MFP_SPI0_MOSI);
}

void PDMA_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA);

    if(u32Status & PDMA_INTSTS_ABTIF_Msk)    /* abort */
    {
        if(PDMA_GET_ABORT_STS(PDMA) & PDMA_ABTSTS_ABTIF0_Msk)
            PDMA_CLR_ABORT_FLAG(PDMA, PDMA_ABTSTS_ABTIF0_Msk);
    }
    else if(u32Status & PDMA_INTSTS_TDIF_Msk)
    {
        if(PDMA_GET_TD_STS(PDMA) & PDMA_TDSTS_TDIF0_Msk)             /* channel 0 done */
        {
            /* Reset PDMA Scater-Gatter table */
            PDMA_ResetTxSGTable(u8TxIdx);
            u8TxIdx ^= 1;
            u32PlayReady = 1;
        }
        PDMA_CLR_TD_FLAG(PDMA, PDMA_TDSTS_TDIF0_Msk);
    }
    else
        printf("unknown interrupt, status=0x%x!!\n", u32Status);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/

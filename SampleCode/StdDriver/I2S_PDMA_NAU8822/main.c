/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 8 $
 * $Date: 18/07/20 4:39p $
 * @brief    M031 I2S Driver Sample Code
 *           This is a I2S demo with PDMA function connected with NAU8822 codec.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

// use LIN as source, undefine it if MIC is used
#define INPUT_IS_LIN
#define I2S_TX_DMA_CH 0
#define I2S_RX_DMA_CH 1

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define BUFF_LEN        64
#define BUFF_HALF_LEN   (BUFF_LEN/2)

typedef struct
{
    uint32_t CTL;
    uint32_t SA;
    uint32_t DA;
    uint32_t FIRST;
} DESC_TABLE_T;

#define NAU8822_ADDR    0x1A                /* NAU8822 Device ID */
volatile uint8_t u8TxIdx = 0, u8RxIdx = 0;
uint32_t PcmRxBuff[2][BUFF_LEN] = {0};
uint32_t PcmTxBuff[2][BUFF_LEN] = {0};
uint32_t volatile u32BuffPos = 0;
DESC_TABLE_T g_asDescTable_TX[2], g_asDescTable_RX[2];

void Delay(int count)
{
    volatile uint32_t i;
    for(i = 0; i < count ; i++);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Write 9-bit data to 7-bit address register of NAU8822 with I2C1                                        */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_WriteNAU8822(uint8_t u8addr, uint16_t u16data)
{
    I2C_START(I2C1);
    I2C_WAIT_READY(I2C1);

    I2C_SET_DATA(I2C1, NAU8822_ADDR << 1);
    I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
    I2C_WAIT_READY(I2C1);

    I2C_SET_DATA(I2C1, (uint8_t)((u8addr << 1) | (u16data >> 8)));
    I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
    I2C_WAIT_READY(I2C1);

    I2C_SET_DATA(I2C1, (uint8_t)(u16data & 0x00FF));
    I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
    I2C_WAIT_READY(I2C1);

    I2C_STOP(I2C1);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  NAU8822 Settings with I2C interface                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void NAU8822_Setup()
{
    printf("\nConfigure NAU8822 ...");

    I2C_WriteNAU8822(0,  0x000);   /* Reset all registers */
    Delay(0x200);

#ifdef INPUT_IS_LIN //input source is LIN
    I2C_WriteNAU8822(1,  0x02F);
    I2C_WriteNAU8822(2,  0x1B3);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    I2C_WriteNAU8822(3,  0x07F);   /* Enable L/R main mixer, DAC */
    I2C_WriteNAU8822(4,  0x010);   /* 16-bit word length, I2S format, Stereo */
    I2C_WriteNAU8822(5,  0x000);   /* Companding control and loop back mode (all disable) */
    I2C_WriteNAU8822(6,  0x1AD);   /* Divide by 6, 16K */
    I2C_WriteNAU8822(7,  0x006);   /* 16K for internal filter coefficients */
    I2C_WriteNAU8822(10, 0x008);   /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    I2C_WriteNAU8822(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */
    I2C_WriteNAU8822(15, 0x1FF);   /* ADC left digital volume control */
    I2C_WriteNAU8822(16, 0x1FF);   /* ADC right digital volume control */

    I2C_WriteNAU8822(44, 0x000);   /* LLIN/RLIN is not connected to PGA */
    I2C_WriteNAU8822(47, 0x060);   /* LLIN connected, and its Gain value */
    I2C_WriteNAU8822(48, 0x060);   /* RLIN connected, and its Gain value */
    I2C_WriteNAU8822(50, 0x001);   /* Left DAC connected to LMIX */
    I2C_WriteNAU8822(51, 0x001);   /* Right DAC connected to RMIX */
#else   //input source is MIC
    I2C_WriteNAU8822(1,  0x03F);
    I2C_WriteNAU8822(2,  0x1BF);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    I2C_WriteNAU8822(3,  0x07F);   /* Enable L/R main mixer, DAC */
    I2C_WriteNAU8822(4,  0x010);   /* 16-bit word length, I2S format, Stereo */
    I2C_WriteNAU8822(5,  0x000);   /* Companding control and loop back mode (all disable) */
    I2C_WriteNAU8822(6,  0x1AD);   /* Divide by 6, 16K */
    I2C_WriteNAU8822(7,  0x006);   /* 16K for internal filter coefficients */
    I2C_WriteNAU8822(10, 0x008);   /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    I2C_WriteNAU8822(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */
    I2C_WriteNAU8822(15, 0x1EF);   /* ADC left digital volume control */
    I2C_WriteNAU8822(16, 0x1EF);   /* ADC right digital volume control */
    I2C_WriteNAU8822(44, 0x033);   /* LMICN/LMICP is connected to PGA */
    I2C_WriteNAU8822(50, 0x001);   /* Left DAC connected to LMIX */
    I2C_WriteNAU8822(51, 0x001);   /* Right DAC connected to RMIX */
#endif

    printf("[OK]\n");
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

    /* Enable I2C0 clock */
    CLK_EnableModuleClock(I2C1_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set PB multi-function pins for I2C1 */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB1MFP_I2C1_SCL | SYS_GPB_MFPL_PB0MFP_I2C1_SDA)) |
                    (SYS_GPB_MFPL_PB1MFP_I2C1_SCL | SYS_GPB_MFPL_PB0MFP_I2C1_SDA);

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

// Configure PDMA to Scatter Gather mode */
void PDMA_Init(void)
{
    /* Tx description */
    g_asDescTable_TX[0].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    g_asDescTable_TX[0].SA = (uint32_t)&PcmTxBuff[0];
    g_asDescTable_TX[0].DA = (uint32_t)&SPI0->TX;
    g_asDescTable_TX[0].FIRST = (uint32_t)&g_asDescTable_TX[1] - (PDMA->SCATBA);

    g_asDescTable_TX[1].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    g_asDescTable_TX[1].SA = (uint32_t)&PcmTxBuff[1];
    g_asDescTable_TX[1].DA = (uint32_t)&SPI0->TX;
    g_asDescTable_TX[1].FIRST = (uint32_t)&g_asDescTable_TX[0] - (PDMA->SCATBA);   //link to first description

    /* Rx description */
    g_asDescTable_RX[0].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    g_asDescTable_RX[0].SA = (uint32_t)&SPI0->RX;
    g_asDescTable_RX[0].DA = (uint32_t)&PcmRxBuff[0];
    g_asDescTable_RX[0].FIRST = (uint32_t)&g_asDescTable_RX[1] - (PDMA->SCATBA);

    g_asDescTable_RX[1].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    g_asDescTable_RX[1].SA = (uint32_t)&SPI0->RX;
    g_asDescTable_RX[1].DA = (uint32_t)&PcmRxBuff[1];
    g_asDescTable_RX[1].FIRST = (uint32_t)&g_asDescTable_RX[0] - (PDMA->SCATBA);   //link to first description

    /* Open PDMA channel 1 for SPI TX and channel 2 for SPI RX */
    PDMA_Open(PDMA, (1 << I2S_TX_DMA_CH) | (1 << I2S_RX_DMA_CH));

    /* Configure PDMA transfer mode */
    PDMA_SetTransferMode(PDMA, I2S_TX_DMA_CH, PDMA_SPI0_TX, 1, (uint32_t)&g_asDescTable_TX[0]);
    PDMA_SetTransferMode(PDMA, I2S_RX_DMA_CH, PDMA_SPI0_RX, 1, (uint32_t)&g_asDescTable_RX[0]);

    /* Enable PDMA channel 0&1 interrupt */
    PDMA_EnableInt(PDMA, I2S_TX_DMA_CH, 0);
    PDMA_EnableInt(PDMA, I2S_RX_DMA_CH, 0);

    NVIC_EnableIRQ(PDMA_IRQn);
}

/* Once PDMA has transferred, software need to reset Scatter-Gather table */
void PDMA_ResetTxSGTable(uint8_t id)
{
    g_asDescTable_TX[id].CTL |= PDMA_OP_SCATTER;
    g_asDescTable_TX[id].CTL |= ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos);
}

/* Once PDMA has transferred, software need to reset Scatter-Gather table */
void PDMA_ResetRxSGTable(uint8_t id)
{
    g_asDescTable_RX[id].CTL |= PDMA_OP_SCATTER;
    g_asDescTable_RX[id].CTL |= ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos);
}

/* Init I2C interface */
void I2C1_Init(void)
{
    /* Open I2C1 and set clock to 100k */
    I2C_Open(I2C1, 100000);

    /* Get I2C3 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C1));
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("+------------------------------------------------------------------------+\n");
    printf("|                   SPI Driver Sample Code with NAU8822                  |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  NOTE: This sample code needs to work with NAU8822.\n");

    /* Init I2C1 to access NAU8822 */
    I2C1_Init();

#define OPT_I2S_SLAVE_MODE
#ifdef OPT_I2S_SLAVE_MODE
    /* Set I2S in slave mode and let NAU8822 provide the exact WS/BCLK */
#ifdef INPUT_IS_LIN
    SPII2S_Open(SPI0, SPII2S_MODE_SLAVE, 16000, SPII2S_DATABIT_16, SPII2S_STEREO, SPII2S_FORMAT_I2S);
#else
    SPII2S_Open(SPI0, SPII2S_MODE_SLAVE, 16000, SPII2S_DATABIT_16, SPII2S_MONO, SPII2S_FORMAT_I2S);
#endif
#else
#ifdef INPUT_IS_LIN
    SPII2S_Open(SPI0, SPII2S_MODE_MASTER, 16000, SPII2S_DATABIT_16, SPII2S_STEREO, SPII2S_FORMAT_I2S);
#else
    SPII2S_Open(SPI0, SPII2S_MODE_MASTER, 16000, SPII2S_DATABIT_16, SPII2S_MONO, SPII2S_FORMAT_I2S);
#endif
#endif  // OPT_I2S_SLAVE_MODE

    /* Initialize NAU8822 codec */
    NAU8822_Setup();

    /* Set MCLK and enable MCLK (MCLK can provide clock source to NAU882) */
    SPII2S_EnableMCLK(SPI0, 12000000);

#ifndef INPUT_IS_LIN
    SPII2S_SET_MONO_RX_CHANNEL(SPI0, SPII2S_MONO_LEFT);       //NAU8822 will store data in left channel
#endif

    PDMA_Init();

    /* Enable I2S Rx function */
    SPII2S_ENABLE_RXDMA(SPI0);
    SPII2S_ENABLE_RX(SPI0);

    /* Enable I2S Tx function */
    SPII2S_ENABLE_TXDMA(SPI0);
    SPII2S_ENABLE_TX(SPI0);

    while(1);
}

void PDMA_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA);

    if(u32Status & PDMA_INTSTS_ABTIF_Msk)    /* abort */
    {
        if(PDMA_GET_ABORT_STS(PDMA) & PDMA_ABTSTS_ABTIF0_Msk)
            PDMA_CLR_ABORT_FLAG(PDMA, PDMA_ABTSTS_ABTIF0_Msk);
        else
            PDMA_CLR_ABORT_FLAG(PDMA, PDMA_ABTSTS_ABTIF1_Msk);
    }
    else if(u32Status & PDMA_INTSTS_TDIF_Msk)
    {
        if(PDMA_GET_TD_STS(PDMA) & PDMA_TDSTS_TDIF1_Msk)             /* channel 1 done */
        {
            /* Copy RX data to TX buffer */
            memcpy(&PcmTxBuff[u8TxIdx ^ 1], &PcmRxBuff[u8RxIdx], BUFF_LEN * 4);
            /* Reset PDMA Scatter-Gather table */
            PDMA_ResetRxSGTable(u8RxIdx);
            u8RxIdx ^= 1;
            PDMA_CLR_TD_FLAG(PDMA, PDMA_TDSTS_TDIF1_Msk);
        }
        if(PDMA_GET_TD_STS(PDMA) & PDMA_TDSTS_TDIF0_Msk)             /* channel 0 done */
        {
            /* Reset PDMA Scatter-Gather table */
            PDMA_ResetTxSGTable(u8TxIdx);
            u8TxIdx ^= 1;
            PDMA_CLR_TD_FLAG(PDMA, PDMA_TDSTS_TDIF0_Msk);
        }
    }
    else
        printf("unknown interrupt, status=0x%x!!\n", u32Status);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/

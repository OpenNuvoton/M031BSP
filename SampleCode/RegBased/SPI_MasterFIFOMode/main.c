/**************************************************************************//**
 * @file     main.c
 * @version  V1.0
 * $Revision: 7 $
 * $Date: 18/07/19 2:23p $
 * @brief    Configure SPI0 as Master mode and demonstrate how to communicate
 *           with an off-chip SPI Slave device with FIFO mode. This sample
 *           code needs to work with SPI_SlaveFifoMode sample code.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define DATA_COUNT      16
#define TEST_PATTERN    0x00550000
#define SPI_CLK_FREQ    2000000


uint32_t g_au32SourceData[DATA_COUNT];
uint32_t g_au32DestinationData[DATA_COUNT];
volatile uint32_t g_u32TxDataCount;
volatile uint32_t g_u32RxDataCount;

/* Function prototype declaration */
void SYS_Init(void);
void UART_Init(void);
void SPI_Init(void);

/* ------------- */
/* Main function */
/* ------------- */
int main(void)
{
    uint32_t u32DataCount;
    uint32_t u32TimeOutCount;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART for print message */
    UART_Init();

    /* Init SPI */
    SPI_Init();

    printf("\n\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                     SPI Master Mode Sample Code                      |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\n");
    printf("Configure SPI0 as a master.\n");
    printf("Bit length of a transaction: 32\n");
    printf("The I/O connection for SPI0:\n");
    printf("    SPI0_SS(PA.3)\n    SPI0_CLK(PA.2)\n");
    printf("    SPI0_MISO(PA.1)\n    SPI0_MOSI(PA.0)\n\n");
    printf("SPI controller will enable FIFO mode and transfer %d data to a off-chip slave device.\n", DATA_COUNT);
    printf("In the meanwhile the SPI controller will receive %d data from the off-chip slave device.\n", DATA_COUNT);
    printf("After the transfer is done, the %d received data will be printed out.\n", DATA_COUNT);
    printf("The SPI master configuration is ready.\n");

    for(u32DataCount = 0; u32DataCount < DATA_COUNT; u32DataCount++)
    {
        /* Write the initial value to source buffer */
        g_au32SourceData[u32DataCount] = TEST_PATTERN + u32DataCount;
        /* Clear destination buffer */
        g_au32DestinationData[u32DataCount] = 0;
    }

    printf("Before starting the data transfer, make sure the slave device is ready. Press any key to start the transfer.\n");
    getchar();
    printf("\n");

    /* Set TX FIFO threshold, enable TX FIFO threshold interrupt and RX FIFO time-out interrupt */
    SPI0->FIFOCTL = (2 << SPI_FIFOCTL_TXTH_Pos) | SPI_FIFOCTL_RXTOIEN_Msk | SPI_FIFOCTL_TXTHIEN_Msk;
    g_u32TxDataCount = 0;
    g_u32RxDataCount = 0;
    NVIC_EnableIRQ(SPI0_IRQn);

    /* setup timeout */
    u32TimeOutCount = SystemCoreClock;

    /* Wait for transfer done */
    while(g_u32RxDataCount < DATA_COUNT)
    {
        if(u32TimeOutCount == 0)
        {
            printf("\nSomething is wrong, please check if pin connection is correct. \n");
            while(1);
        }
        u32TimeOutCount--;
    }

    /* Print the received data */
    printf("Received data:\n");
    for(u32DataCount = 0; u32DataCount < DATA_COUNT; u32DataCount++)
    {
        printf("%d:\t0x%X\n", u32DataCount, g_au32DestinationData[u32DataCount]);
    }
    /* Disable TX FIFO threshold interrupt and RX FIFO time-out interrupt */
    SPI0->FIFOCTL = 0;
    NVIC_DisableIRQ(SPI0_IRQn);
    printf("The data transfer was done.\n");

    printf("\n\nExit SPI driver sample code.\n");

    /* Disable SPI0 peripheral clock */
    CLK->APBCLK0 &= (~CLK_APBCLK0_SPI0CKEN_Msk);
    while(1);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Switch UART0 clock source to HIRC */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART0SEL_Msk) | CLK_CLKSEL1_UART0SEL_HIRC;

    /* Select PCLK1 as the clock source of SPI0 */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI0SEL_Msk)) | CLK_CLKSEL2_SPI0SEL_PCLK1;

    /* Enable UART and SPI0 clock */
    CLK->APBCLK0 = CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_SPI0CKEN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Setup SPI0 multi-function pins */
    /* PA.3 is SPI0_SS,   PA.2 is SPI0_CLK,
       PA.1 is SPI0_MISO, PA.0 is SPI0_MOSI*/
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA3MFP_Msk |
                                       SYS_GPA_MFPL_PA2MFP_Msk |
                                       SYS_GPA_MFPL_PA1MFP_Msk |
                                       SYS_GPA_MFPL_PA0MFP_Msk)) |
                    (SYS_GPA_MFPL_PA3MFP_SPI0_SS |
                     SYS_GPA_MFPL_PA2MFP_SPI0_CLK |
                     SYS_GPA_MFPL_PA1MFP_SPI0_MISO |
                     SYS_GPA_MFPL_PA0MFP_SPI0_MOSI);
}

void UART_Init(void)
{
    /* Word length is 8 bits; 1 stop bit; no parity bit. */
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

    /* UART peripheral clock rate 48 MHz; UART bit rate 115200 bps. */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI0->SSCTL = SPI_SSCTL_AUTOSS_Msk | SPI_SSCTL_SS_Msk;

    /* Configure SPI0 as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    SPI0->CTL = SPI_MASTER | SPI_CTL_TXNEG_Msk | SPI_CTL_SPIEN_Msk;

    /* Set IP clock divider. SPI clock rate = f_PCLK0 / (5+1) */
    SPI0->CLKDIV = (SPI0->CLKDIV & (~SPI_CLKDIV_DIVIDER_Msk)) | 5;
}

void SPI0_IRQHandler(void)
{
    /* Check RX EMPTY flag */
    while((SPI0->STATUS & SPI_STATUS_RXEMPTY_Msk) == 0)
    {
        /* Read RX FIFO */
        g_au32DestinationData[g_u32RxDataCount++] = SPI0->RX;
    }
    /* Check TX FULL flag and TX data count */
    while(((SPI0->STATUS & SPI_STATUS_TXFULL_Msk) == 0) && (g_u32TxDataCount < DATA_COUNT))
    {
        /* Write to TX FIFO */
        SPI0->TX = g_au32SourceData[g_u32TxDataCount++];
    }
    if(g_u32TxDataCount >= DATA_COUNT)
        SPI0->FIFOCTL &= (~SPI_FIFOCTL_TXTHIEN_Msk); /* Disable TX FIFO threshold interrupt */

    /* Check the RX FIFO time-out interrupt flag */
    if(SPI0->STATUS & SPI_STATUS_RXTOIF_Msk)
    {
        /* If RX FIFO is not empty, read RX FIFO. */
        while((SPI0->STATUS & SPI_STATUS_RXEMPTY_Msk) == 0)
            g_au32DestinationData[g_u32RxDataCount++] = SPI0->RX;
    }
}


/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/

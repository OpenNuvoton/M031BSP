/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 18/07/16 1:22p $
 * @brief
 *          Configure USCI_SPI0 as Master mode and demonstrate how to communicate with an off-chip SPI slave device.
 *          This sample code needs to work with USCI_SPI_SlaveMode.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_COUNT 16

/*----------------------------------------------------------------------------------------------------------*/
/* Define Function Prototypes                                                                               */
/*----------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void USCI_SPI_Init(void);
void UART0_Init(void);

/*----------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                    */
/*----------------------------------------------------------------------------------------------------------*/
uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData[TEST_COUNT];
volatile uint32_t g_u32TxDataCount;
volatile uint32_t g_u32RxDataCount;

void USCI01_IRQHandler(void)
{
    uint32_t u32RxData;

    /* Clear TX end interrupt flag */
    USPI0->PROTSTS = USPI_PROTSTS_TXENDIF_Msk;

    /* Check RX EMPTY flag */
    while ((USPI0->BUFSTS & USPI_BUFSTS_RXEMPTY_Msk) == 0)
    {
        /* Read RX Buffer */
        u32RxData = USPI0->RXDAT;

        g_au32DestinationData[g_u32RxDataCount++] = u32RxData;
    }

    /* Check TX data count */
    if (g_u32TxDataCount < TEST_COUNT)
    {
        /* Write to TX Buffer */
        USPI0->TXDAT = g_au32SourceData[g_u32TxDataCount++];
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Wait for HIRC clock ready */
    while((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) != CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);

    /* Enable UART0 clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Enable USPI0 clock */
    CLK->APBCLK1 |= CLK_APBCLK1_USCI0CKEN_Msk;

    /* Switch UART0 clock source to HIRC and UART0 clock divider as 1 */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART0SEL_Msk)) | CLK_CLKSEL1_UART0SEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UART0DIV_Msk)) | CLK_CLKDIV0_UART0(1);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set USPI0 multi-function pins */
    SYS->GPA_MFPH = (SYS->GPA_MFPH & ~(SYS_GPA_MFPH_PA9MFP_Msk | SYS_GPA_MFPH_PA10MFP_Msk | SYS_GPA_MFPH_PA11MFP_Msk)) |
                    (SYS_GPA_MFPH_PA9MFP_USCI0_DAT1 | SYS_GPA_MFPH_PA10MFP_USCI0_DAT0 | SYS_GPA_MFPH_PA11MFP_USCI0_CLK);
    SYS->GPC_MFPH = (SYS->GPC_MFPH & ~(SYS_GPC_MFPH_PC14MFP_Msk)) | (SYS_GPC_MFPH_PC14MFP_USCI0_CTL0);

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void USCI_SPI_Init(void)
{
    uint32_t u32ClkDiv = 0;
    uint32_t PCLK0Div;
    uint32_t u32Pclk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Initiate USCI_SPI0                                                                                      */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure USCI_SPI0 as a master, USCI_SPI0 clock rate 2 MHz,
       clock idle low, 16-bit transaction, drive output on falling clock edge and latch input on rising edge. */

    /* Enable USPI protocol */
    USPI0->CTL &= ~USPI_CTL_FUNMODE_Msk;
    USPI0->CTL = 1 << USPI_CTL_FUNMODE_Pos;

    /* Data format configuration : 16-bit */
    USPI0->LINECTL &= ~USPI_LINECTL_DWIDTH_Msk;
    USPI0->LINECTL |= (0 << USPI_LINECTL_DWIDTH_Pos);

    /* MSB data format */
    USPI0->LINECTL &= ~USPI_LINECTL_LSB_Msk;

    /* Set slave selection signal active low : for master */
    USPI0->LINECTL |= USPI_LINECTL_CTLOINV_Msk;

    /* Set operating mode and transfer timing : for master and mode 0 */
    USPI0->PROTCTL &= ~(USPI_PROTCTL_SCLKMODE_Msk | USPI_PROTCTL_AUTOSS_Msk | USPI_PROTCTL_SLAVE_Msk);
    USPI0->PROTCTL |= (USPI_MASTER | USPI_MODE_0);

    /* Set USPI bus clock : 2000000Hz */
    PCLK0Div = (CLK->PCLKDIV & CLK_PCLKDIV_APB0DIV_Msk) >> CLK_PCLKDIV_APB0DIV_Pos;
    u32Pclk = (SystemCoreClock >> PCLK0Div);

    u32ClkDiv = (((((u32Pclk / 2) * 10) / (2000000)) + 5) / 10) - 1; /* Compute proper divider for USPI clock */
    USPI0->BRGEN &= ~USPI_BRGEN_CLKDIV_Msk;
    USPI0->BRGEN |= (u32ClkDiv << USPI_BRGEN_CLKDIV_Pos);

    USPI0->PROTCTL |= USPI_PROTCTL_PROTEN_Msk;

    /* Enable the automatic hardware slave selection function and configure USCI_SPI_SS pin as low-active. */
    USPI0->LINECTL = (USPI0->LINECTL & ~USPI_LINECTL_CTLOINV_Msk) | USPI_SS_ACTIVE_LOW;
    USPI0->PROTCTL |= USPI_PROTCTL_AUTOSS_Msk;
}

int main()
{
    uint32_t u32DataCount;

    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART0_Init();

    /* Init USCI_SPI0 */
    USCI_SPI_Init();

    printf("\n\n");
    printf("+--------------------------------------------------------+\n");
    printf("|             USCI_SPI Master Mode Sample Code           |\n");
    printf("+--------------------------------------------------------+\n");
    printf("\n");
    printf("Configure USCI_SPI0 as a master.\n");
    printf("Bit length of a transaction: 16\n");
    printf("The I/O connection for USCI_SPI0:\n");
    printf("    USCI_SPI0_SS (PC.14)\n    USCI_SPI0_CLK (PA.11)\n");
    printf("    USCI_SPI0_MISO (PA.9)\n    USCI_SPI0_MOSI (PA.10)\n\n");
    printf("USCI_SPI controller will transfer %d data to a off-chip slave device.\n", TEST_COUNT);
    printf("In the meanwhile the USCI_SPI controller will receive %d data from the off-chip slave device.\n", TEST_COUNT);
    printf("After the transfer is done, the %d received data will be printed out.\n", TEST_COUNT);
    printf("The USCI_SPI master configuration is ready.\n");

    for (u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        /* Write the initial value to source buffer */
        g_au32SourceData[u32DataCount] = 0x5500 + u32DataCount;
        /* Clear destination buffer */
        g_au32DestinationData[u32DataCount] = 0;
    }

    printf("Before starting the data transfer, make sure the slave device is ready.\n");
    printf("Press any key to start the transfer.");
    getchar();
    printf("\n");

    /* Enable TX end interrupt */
    USPI0->INTEN |= USPI_INTEN_TXENDIEN_Msk;

    g_u32TxDataCount = 0;
    g_u32RxDataCount = 0;

    NVIC_EnableIRQ(USCI_IRQn);

    /* Write to TX Buffer */
    USPI0->TXDAT = g_au32SourceData[g_u32TxDataCount++];

    /* Wait for transfer done */
    while (g_u32RxDataCount < TEST_COUNT);

    /* Print the received data */
    printf("Received data:\n");

    for (u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        printf("%d:\t0x%X\n", u32DataCount, g_au32DestinationData[u32DataCount]);
    }

    /* Disable TX end interrupt */
    USPI0->INTEN &= (~USPI_INTEN_TXENDIEN_Msk);

    NVIC_DisableIRQ(USCI_IRQn);

    printf("The data transfer was done.\n");

    printf("\n\nExit USCI_SPI driver sample code.\n");

    /* Close USCI_SPI0 */
    USPI0->CTL &= ~USPI_CTL_FUNMODE_Msk;

    while (1);
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

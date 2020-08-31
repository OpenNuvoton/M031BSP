/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 18/07/16 1:23p $
 * @brief
 *           Configure USCI_SPI0 as Slave mode and demonstrate how to communicate with an off-chip SPI master device.
 *           This sample code needs to work with USCI_SPI_MasterMode.
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

/*----------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                    */
/*----------------------------------------------------------------------------------------------------------*/
uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData[TEST_COUNT];
volatile uint32_t g_u32TxDataCount;
volatile uint32_t g_u32RxDataCount;

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
    /*---------------------------------------------------------------------------------------------------------*/
    /* Initiate USCI_SPI0                                                                                      */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure USCI_SPI0 as a slave, USCI_SPI0 clock rate = f_PCLK1,
       clock idle low, 16-bit transaction, drive output on falling clock edge and latch input on rising edge. */

    /* Enable USPI protocol */
    USPI0->CTL &= ~USPI_CTL_FUNMODE_Msk;
    USPI0->CTL = 1 << USPI_CTL_FUNMODE_Pos;

    /* Data format configuration : 16-bit */
    USPI0->LINECTL &= ~USPI_LINECTL_DWIDTH_Msk;
    USPI0->LINECTL |= (0 << USPI_LINECTL_DWIDTH_Pos);

    /* MSB data format */
    USPI0->LINECTL &= ~USPI_LINECTL_LSB_Msk;

    /* Set slave selection signal active low : for slave */
    USPI0->CTLIN0 |= USPI_CTLIN0_ININV_Msk;

    /* Set operating mode and transfer timing : for slave and mode 0 */
    USPI0->PROTCTL &= ~(USPI_PROTCTL_SCLKMODE_Msk | USPI_PROTCTL_AUTOSS_Msk | USPI_PROTCTL_SLAVE_Msk);
    USPI0->PROTCTL |= (USPI_SLAVE | USPI_MODE_0);

    /* Set USPI bus clock */
    USPI0->BRGEN &= ~USPI_BRGEN_CLKDIV_Msk;

    USPI0->PROTCTL |=  USPI_PROTCTL_PROTEN_Msk;
}

int main()
{
    uint32_t u32TxDataCount, u32RxDataCount;

    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART0_Init();

    /* Init USCI_SPI0 */
    USCI_SPI_Init();

    printf("\n\n");
    printf("+-----------------------------------------------------+\n");
    printf("|           USCI_SPI Slave Mode Sample Code           |\n");
    printf("+-----------------------------------------------------+\n");
    printf("\n");
    printf("Configure USCI_SPI0 as a slave.\n");
    printf("Bit length of a transaction: 16\n");
    printf("The I/O connection for USCI_SPI0:\n");
    printf("    USCI_SPI0_SS (PC.14)\n    USCI_SPI0_CLK (PA.11)\n");
    printf("    USCI_SPI0_MISO (PA.9)\n    USCI_SPI0_MOSI (PA.10)\n\n");
    printf("USCI_SPI controller will transfer %d data to a off-chip master device.\n", TEST_COUNT);
    printf("In the meanwhile the USCI_SPI controller will receive %d data from the off-chip master device.\n", TEST_COUNT);
    printf("After the transfer is done, the %d received data will be printed out.\n", TEST_COUNT);

    for (u32TxDataCount = 0; u32TxDataCount < TEST_COUNT; u32TxDataCount++)
    {
        /* Write the initial value to source buffer */
        g_au32SourceData[u32TxDataCount] = 0xAA00 + u32TxDataCount;
        /* Clear destination buffer */
        g_au32DestinationData[u32TxDataCount] = 0;
    }

    u32TxDataCount = 0;
    u32RxDataCount = 0;
    printf("Press any key if the master device configuration is ready.");
    getchar();
    printf("\n");

    /* Access TX and RX Buffer */
    while (u32RxDataCount < TEST_COUNT)
    {
        /* Check TX FULL flag and TX data count */
        if (((USPI0->BUFSTS & USPI_BUFSTS_TXFULL_Msk) == 0) && (u32TxDataCount < TEST_COUNT))
            /* Write to TX Buffer */
            USPI0->TXDAT = g_au32SourceData[u32TxDataCount++];

        /* Check RX EMPTY flag */
        if ((USPI0->BUFSTS & USPI_BUFSTS_RXEMPTY_Msk) == 0)
            /* Read RX Buffer */
            g_au32DestinationData[u32RxDataCount++] = USPI0->RXDAT;
    }

    /* Print the received data */
    printf("Received data:\n");

    for (u32RxDataCount = 0; u32RxDataCount < TEST_COUNT; u32RxDataCount++)
    {
        printf("%d:\t0x%X\n", u32RxDataCount, g_au32DestinationData[u32RxDataCount]);
    }

    printf("The data transfer was done.\n");

    printf("\n\nExit USCI_SPI driver sample code.\n");

    /* Close USCI_SPI0 */
    USPI0->CTL &= ~USPI_CTL_FUNMODE_Msk;

    while (1);
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

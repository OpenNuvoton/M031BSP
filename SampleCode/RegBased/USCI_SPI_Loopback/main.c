/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 18/07/16 1:19p $
 * @brief
 *           Implement USCI_SPI0 Master loop back transfer.
 *           This sample code needs to connect USCI_SPI0_MISO pin and USCI_SPI0_MOSI pin together.
 *           It will compare the received data with transmitted data.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_COUNT             64

uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData[TEST_COUNT];

/* Function prototype declaration */
void SYS_Init(void);
void UART0_Init(void);
void USCI_SPI_Init(void);

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
    /* Init USCI_SPI0                                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Select USCI_SPI0 protocol */
    USPI0->CTL &= ~USPI_CTL_FUNMODE_Msk;
    USPI0->CTL = 1 << USPI_CTL_FUNMODE_Pos;
    /* Configure USCI_SPI0 as a master, clock idle low, 16-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    USPI0->PROTCTL = USPI_MASTER | USPI_MODE_0;
    USPI0->LINECTL = 0;
    /* Set USCI_SPI0 clock rate = f_PCLK1 / 2*(5+1) */
    USPI0->BRGEN = (USPI0->BRGEN & (~USPI_BRGEN_CLKDIV_Msk)) | (5 << USPI_BRGEN_CLKDIV_Pos);
    /* Enable USCI_SPI0 protocol */
    USPI0->PROTCTL |= USPI_PROTCTL_PROTEN_Msk;
}

int main()
{
    uint32_t u32DataCount, u32TestCount, u32Err;

    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART0_Init();

    /* Init USCI_SPI0 */
    USCI_SPI_Init();

    printf("\n\n");
    printf("+------------------------------------------------------------------+\n");
    printf("|                   USCI_SPI Driver Sample Code                    |\n");
    printf("+------------------------------------------------------------------+\n");
    printf("\n");
    printf("\nThis sample code demonstrates USCI_SPI0 self loop back data transfer.\n");
    printf(" USCI_SPI0 configuration:\n");
    printf("     Master mode; data width 16 bits.\n");
    printf(" I/O connection:\n");
    printf("     PA.10 USCI_SPI0_MOSI <--> PA.9 USCI_SPI0_MISO \n");

    printf("\nUSCI_SPI0 Loopback test ");

    /* set the source data and clear the destination buffer */
    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        g_au32SourceData[u32DataCount] = u32DataCount;
        g_au32DestinationData[u32DataCount] = 0;
    }

    u32Err = 0;
    for(u32TestCount = 0; u32TestCount < 0x1000; u32TestCount++)
    {
        /* set the source data and clear the destination buffer */
        for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
        {
            g_au32SourceData[u32DataCount]++;
            g_au32DestinationData[u32DataCount] = 0;
        }

        u32DataCount = 0;

        if((u32TestCount & 0x1FF) == 0)
        {
            putchar('.');
        }

        while(1)
        {
            /* Write to TX register */
            USPI0->TXDAT = g_au32SourceData[u32DataCount];
            /* Check SPI0 busy status */
            while(USPI0->PROTSTS & USPI_PROTSTS_BUSY_Msk);
            /* Read received data */
            g_au32DestinationData[u32DataCount] = USPI0->RXDAT;
            u32DataCount++;
            if(u32DataCount == TEST_COUNT)
                break;
        }

        /*  Check the received data */
        for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
        {
            if(g_au32DestinationData[u32DataCount] != g_au32SourceData[u32DataCount])
                u32Err = 1;
        }

        if(u32Err)
            break;
    }

    if(u32Err)
        printf(" [FAIL]\n\n");
    else
        printf(" [PASS]\n\n");

    /* Disable USCI_SPI0 function mode */
    USPI0->CTL &= ~USPI_CTL_FUNMODE_Msk;

    while(1);
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

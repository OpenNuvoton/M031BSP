/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 11 $
 * $Date: 18/07/19 2:18p $
 * @brief    Access SPI flash using QSPI dual mode.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0 
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_NUMBER 1   /* page numbers */
#define TEST_LENGTH 256 /* length */

#define SPI_FLASH_PORT  QSPI0

uint8_t g_au8SrcArray[TEST_LENGTH];
uint8_t g_au8DestArray[TEST_LENGTH];

uint16_t SpiFlash_ReadMidDid(void)
{
    uint8_t u8RxData[6], u8IDCnt = 0;

    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x90, Read Manufacturer/Device ID
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x90);

    // send 24-bit '0', dummy
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    // receive 16-bit
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);

    while (!SPI_GET_RX_FIFO_EMPTY_FLAG(SPI_FLASH_PORT))
        u8RxData[u8IDCnt ++] = QSPI_READ_RX(SPI_FLASH_PORT);

    return ((u8RxData[4] << 8) | u8RxData[5]);
}

void SpiFlash_ChipErase(void)
{
    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);

    //////////////////////////////////////////

    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0xC7, Chip Erase
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0xC7);

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);

    SPI_FLASH_PORT->FIFOCTL |= QSPI_FIFOCTL_RXFBCLR_Msk;
}

uint8_t SpiFlash_ReadStatusReg(void)
{
    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x05, Read status register
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x05);

    // read status
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);

    // skip first rx data
    QSPI_READ_RX(SPI_FLASH_PORT);

    return (QSPI_READ_RX(SPI_FLASH_PORT) & 0xff);
}

void SpiFlash_WriteStatusReg(uint8_t u8Value)
{
    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);

    ///////////////////////////////////////

    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x01, Write status register
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x01);

    // write status
    QSPI_WRITE_TX(SPI_FLASH_PORT, u8Value);

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);
}

void SpiFlash_WaitReady(void)
{
    uint8_t u8ReturnValue;

    do
    {
        u8ReturnValue = SpiFlash_ReadStatusReg();
        u8ReturnValue = u8ReturnValue & 1;
    }
    while (u8ReturnValue != 0);   // check the BUSY bit
}

void SpiFlash_NormalPageProgram(uint32_t StartAddress, uint8_t *u8DataBuffer)
{
    uint32_t u32Cnt = 0;

    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);


    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x02, Page program
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x02);

    // send 24-bit start address
    QSPI_WRITE_TX(SPI_FLASH_PORT, (StartAddress >> 16) & 0xFF);
    QSPI_WRITE_TX(SPI_FLASH_PORT, (StartAddress >> 8)  & 0xFF);
    QSPI_WRITE_TX(SPI_FLASH_PORT, StartAddress       & 0xFF);

    // write data
    while (1)
    {
        if (!QSPI_GET_TX_FIFO_FULL_FLAG(SPI_FLASH_PORT))
        {
            QSPI_WRITE_TX(SPI_FLASH_PORT, u8DataBuffer[u32Cnt++]);

            if (u32Cnt > 255) break;
        }
    }

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);

    SPI_FLASH_PORT->FIFOCTL |= QSPI_FIFOCTL_RXFBCLR_Msk;
}

void SpiFlash_DualFastRead(uint32_t StartAddress, uint8_t *u8DataBuffer)
{
    uint32_t u32Cnt;

    // /CS: active
    QSPI_SET_SS_LOW(SPI_FLASH_PORT);

    // Command: 0x3B, Fast Read dual data
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x3B);

    // send 24-bit start address
    QSPI_WRITE_TX(SPI_FLASH_PORT, (StartAddress >> 16) & 0xFF);
    QSPI_WRITE_TX(SPI_FLASH_PORT, (StartAddress >> 8)  & 0xFF);
    QSPI_WRITE_TX(SPI_FLASH_PORT, StartAddress       & 0xFF);

    // dummy byte
    QSPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // clear RX buffer
    SPI_FLASH_PORT->FIFOCTL |= QSPI_FIFOCTL_RXFBCLR_Msk;

    // enable SPI dual IO mode and set direction to input
    QSPI_ENABLE_DUAL_INPUT_MODE(SPI_FLASH_PORT);

    // read data
    for (u32Cnt = 0; u32Cnt < 256; u32Cnt++)
    {
        QSPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

        while (QSPI_IS_BUSY(SPI_FLASH_PORT));

        u8DataBuffer[u32Cnt] = SPI_READ_RX(SPI_FLASH_PORT);
    }

    // wait tx finish
    while (QSPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    QSPI_SET_SS_HIGH(SPI_FLASH_PORT);

    QSPI_DISABLE_DUAL_MODE(SPI_FLASH_PORT);
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

    /* Select HIRC as the clock source of UART0 */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART0SEL_Msk)) | CLK_CLKSEL1_UART0SEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UART0DIV_Msk)) | CLK_CLKDIV0_UART0(1);

    /* Enable QSPI0 peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_QSPI0CKEN_Msk;

    /* Select PCLK0 as the clock source of QSPI0 */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_QSPI0SEL_Msk)) | CLK_CLKSEL2_QSPI0SEL_PCLK0;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Setup QSPI0 multi-function pins */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk | SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA0MFP_QSPI0_MOSI0 | SYS_GPA_MFPL_PA1MFP_QSPI0_MISO0 | SYS_GPA_MFPL_PA2MFP_QSPI0_CLK | SYS_GPA_MFPL_PA3MFP_QSPI0_SS);

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

void QSPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init QSPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI_FLASH_PORT->SSCTL = QSPI_SSCTL_AUTOSS_Msk | QSPI_SSCTL_SS_Msk;

    /* Configure SPI_FLASH_PORT as a master, MSB first, 8-bit transaction, SPI Mode-0 timing */
    SPI_FLASH_PORT->CTL = QSPI_MASTER | (8 << QSPI_CTL_DWIDTH_Pos) | QSPI_CTL_TXNEG_Msk | QSPI_CTL_SPIEN_Msk;

    /* Set IP clock divider. SPI clock rate = f_PCLK0 / (23+1) */
    SPI_FLASH_PORT->CLKDIV = (SPI_FLASH_PORT->CLKDIV & (~QSPI_CLKDIV_DIVIDER_Msk)) | 23;
}

int main(void)
{
    uint32_t u32ByteCount, u32FlashAddress, u32PageNumber;
    uint32_t u32Error = 0;
    uint16_t u16ID;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART0_Init();

    /* Init QSPI */
    QSPI_Init();

    printf("\n+------------------------------------------------------------------------+\n");
    printf("|                  M031 QSPI Dual Mode with Flash Sample Code            |\n");
    printf("+------------------------------------------------------------------------+\n");

    /* Wait ready */
    SpiFlash_WaitReady();

    if ((u16ID = SpiFlash_ReadMidDid()) != 0xEF14)
    {
        printf("Wrong ID, 0x%x\n", u16ID);

        while (1);
    }
    else
        printf("Flash found: W25X16 ...\n");

    printf("Erase chip ...");

    /* Erase SPI flash */
    SpiFlash_ChipErase();

    /* Wait ready */
    SpiFlash_WaitReady();

    printf("[OK]\n");

    /* init source data buffer */
    for (u32ByteCount = 0; u32ByteCount < TEST_LENGTH; u32ByteCount++)
    {
        g_au8SrcArray[u32ByteCount] = u32ByteCount;
    }

    printf("Start to normal write data to Flash ...");
    /* Program SPI flash */
    u32FlashAddress = 0;

    for (u32PageNumber = 0; u32PageNumber < TEST_NUMBER; u32PageNumber++)
    {
        /* page program */
        SpiFlash_NormalPageProgram(u32FlashAddress, g_au8SrcArray);
        SpiFlash_WaitReady();
        u32FlashAddress += 0x100;
    }

    printf("[OK]\n");

    /* clear destination data buffer */
    for (u32ByteCount = 0; u32ByteCount < TEST_LENGTH; u32ByteCount++)
    {
        g_au8DestArray[u32ByteCount] = 0;
    }

    printf("Dual Read & Compare ...");

    /* Read SPI flash */
    u32FlashAddress = 0;

    for (u32PageNumber = 0; u32PageNumber < TEST_NUMBER; u32PageNumber++)
    {
        /* page read */
        SpiFlash_DualFastRead(u32FlashAddress, g_au8DestArray);
        u32FlashAddress += 0x100;

        for (u32ByteCount = 0; u32ByteCount < TEST_LENGTH; u32ByteCount++)
        {
            if (g_au8DestArray[u32ByteCount] != g_au8SrcArray[u32ByteCount])
                u32Error ++;
        }
    }

    if (u32Error == 0)
        printf("[OK]\n");
    else
        printf("[FAIL]\n");

    while (1);
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

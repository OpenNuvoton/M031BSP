/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 8 $
 * $Date: 18/07/10 10:36a $
 * @brief    Implement CRC in CRC-32 mode with PDMA transfer.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) != CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk ) | CLK_CLKSEL0_HCLKSEL_HIRC ;

    /* Enable UART0 clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk ;

    /* Switch UART0 clock source to HIRC */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART0SEL_Msk) | CLK_CLKSEL1_UART0SEL_HIRC;

    /* Enable CRC clock */
    CLK->AHBCLK |= CLK_AHBCLK_CRCCKEN_Msk ;

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))
                    |(SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

uint32_t GetFMCChecksum(uint32_t u32Address, uint32_t u32Size)
{
    uint32_t u32CHKS;

    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk;

    FMC->ISPCMD  = FMC_ISPCMD_RUN_CKS;
    FMC->ISPADDR = u32Address;
    FMC->ISPDAT  = u32Size;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk) { }

    FMC->ISPCMD = FMC_ISPCMD_READ_CKS;
    FMC->ISPADDR    = u32Address;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk) { }

    u32CHKS = FMC->ISPDAT;

    return u32CHKS;
}

typedef struct dma_desc_t
{
    uint32_t ctl;
    uint32_t src;
    uint32_t dest;
    uint32_t offset;
} DMA_DESC_T;

DMA_DESC_T DMA_DESC[1];

uint32_t GetPDMAChecksum(uint32_t u32Address, uint32_t u32Size)
{
    volatile uint32_t loop = 0, reg;

    /* Enable PDMA module clock */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;

    /* Give valid source address and transfer count and program PDMA memory to memory, dest => CRC_WDATA */
    PDMA->CHCTL = (1 << 0); // use PDMA CH0
    DMA_DESC[0].ctl =
        (1 << PDMA_DSCT_CTL_OPMODE_Pos)  | (0 << PDMA_DSCT_CTL_TXTYPE_Pos) |
        (7 << PDMA_DSCT_CTL_BURSIZE_Pos) |
        (0 << PDMA_DSCT_CTL_SAINC_Pos)   | (3 << PDMA_DSCT_CTL_DAINC_Pos) |
        (2 << PDMA_DSCT_CTL_TXWIDTH_Pos) | (((u32Size / 4) - 1) << PDMA_DSCT_CTL_TXCNT_Pos);
    DMA_DESC[0].src = (uint32_t)u32Address;
    DMA_DESC[0].dest = (uint32_t) & (CRC->DAT);
    DMA_DESC[0].offset = 0;

    PDMA->DSCT[0].CTL = PDMA_OP_SCATTER;
    PDMA->DSCT[0].NEXT = (uint32_t)&DMA_DESC[0] - (PDMA->SCATBA);

    reg = PDMA->INTSTS;
    PDMA->INTSTS = reg;
    PDMA->INTEN = (1 << 0);

    /* Trigger PDMA CH0 transfer ... */
    PDMA->SWREQ = (1 << 0);

    while(PDMA->TRGSTS & 0x1)   // wait PDMA finish
    {
        if(loop++ > (SystemCoreClock / 100))
        {
            printf("\n[PDMA transfer time-out]\n");
            while(1);
        }
    }

    return CRC->CHECKSUM;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    volatile uint32_t addr, size, u32FMCChecksum, u32CRC32Checksum, u32PDMAChecksum;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    /* Reset UART0 */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

    size = 1024 * 2;

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-----------------------------------------------------+\n");
    printf("|    CRC32 with PDMA Sample Code                      |\n");
    printf("|       - Get APROM first %d bytes CRC result by    |\n", size);
    printf("|          a.) FMC checksum command                   |\n");
    printf("|          b.) CPU write CRC data register directly   |\n");
    printf("|          c.) PDMA write CRC data register           |\n");
    printf("+-----------------------------------------------------+\n\n");


    /* Disable CRC function */
    CRC->CTL = 0;

    CRC->CTL = CRC_CTL_CRCEN_Msk;
    CRC->CTL |= CRC_32;
    CRC->CTL |= CRC_WDATA_32 | CRC_WDATA_RVS | CRC_CHECKSUM_RVS | CRC_CHECKSUM_COM;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /*  Case a. */
    u32FMCChecksum = GetFMCChecksum(0x0, size);

    /* Lock protected registers */
    SYS_LockReg();
    /*  Case b. */
    /* Configure CRC controller for CRC-CRC32 mode */
    CRC->SEED = 0xFFFFFFFF;
    CRC->CTL = CRC_32 | (CRC_WDATA_RVS | CRC_CHECKSUM_RVS | CRC_CHECKSUM_COM) | CRC_WDATA_32 | CRC_CTL_CRCEN_Msk;

    /* Setting CRCRST bit will reload the initial seed value(CRC_SEED register) to CRC controller */
    CRC->CTL |= CRC_CTL_CHKSINIT_Msk;

    /* Start to execute CRC-CRC32 operation */
    for(addr = 0; addr < size; addr += 4)
    {
        CRC->DAT = inpw(addr);
    }
    u32CRC32Checksum = CRC->CHECKSUM;

    /*  Case c. */
    /* Configure CRC controller for CRC-CRC32 mode */

    CRC->SEED = 0xFFFFFFFF;
    CRC->CTL = CRC_32 | (CRC_WDATA_RVS | CRC_CHECKSUM_RVS | CRC_CHECKSUM_COM) | CRC_WDATA_32 | CRC_CTL_CRCEN_Msk;

    /* Setting CRCRST bit will reload the initial seed value(CRC_SEED register) to CRC controller */
    CRC->CTL |= CRC_CTL_CHKSINIT_Msk;

    u32PDMAChecksum = GetPDMAChecksum(0x0, size);

    printf("APROM first %d bytes checksum:\n", size);
    printf("   - by FMC command: 0x%08X\n", u32FMCChecksum);
    printf("   - by CPU write:   0x%08X\n", u32CRC32Checksum);
    printf("   - by PDMA write:  0x%08X\n", u32PDMAChecksum);

    if((u32FMCChecksum == u32CRC32Checksum) && (u32CRC32Checksum == u32PDMAChecksum))
    {
        if((u32FMCChecksum == 0) || (u32FMCChecksum == 0xFFFFFFFF))
        {
            printf("\n[Get checksum ... WRONG]");
        }
        else
        {
            printf("\n[Compare checksum ... PASS]");
        }
    }
    else
    {
        printf("\n[Compare checksum ... WRONG]");
    }

    /* Disable CRC function */
    CRC->CTL &= ~CRC_CTL_CRCEN_Msk;

    while(1);
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

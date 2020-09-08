/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 18/07/20 3:57p $
 * @brief    Configure EBI interface to access BS616LV4017 (SRAM) on EBI interface.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PDMA_CH     0

/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
extern void SRAM_BS616LV4017(uint32_t u32MaxSize);
void AccessEBIWithPDMA(void);

void Configure_EBI_16BIT_Pins(void)
{
    /* EBI AD0~5 pins on PC.0~5 */
    /* EBI AD8, AD9 pins on PC.6, PC.7 */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC0MFP_Msk |
                                       SYS_GPC_MFPL_PC1MFP_Msk |
                                       SYS_GPC_MFPL_PC2MFP_Msk |
                                       SYS_GPC_MFPL_PC3MFP_Msk |
                                       SYS_GPC_MFPL_PC4MFP_Msk |
                                       SYS_GPC_MFPL_PC5MFP_Msk |
                                       SYS_GPC_MFPL_PC6MFP_Msk |
                                       SYS_GPC_MFPL_PC7MFP_Msk)) |
                    (SYS_GPC_MFPL_PC0MFP_EBI_AD0 |
                     SYS_GPC_MFPL_PC1MFP_EBI_AD1 |
                     SYS_GPC_MFPL_PC2MFP_EBI_AD2 |
                     SYS_GPC_MFPL_PC3MFP_EBI_AD3 |
                     SYS_GPC_MFPL_PC4MFP_EBI_AD4 |
                     SYS_GPC_MFPL_PC5MFP_EBI_AD5 |
                     SYS_GPC_MFPL_PC6MFP_EBI_AD8 |
                     SYS_GPC_MFPL_PC7MFP_EBI_AD9);

    /* EBI AD6, AD7 pins on PA.6, PA.7 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA6MFP_Msk |
                                       SYS_GPA_MFPL_PA7MFP_Msk)) |
                    (SYS_GPA_MFPL_PA6MFP_EBI_AD6 |
                     SYS_GPA_MFPL_PA7MFP_EBI_AD7);

    /* EBI RD and WR pins on PA.11 and PA.10 */
    SYS->GPA_MFPH = (SYS->GPA_MFPH & ~(SYS_GPA_MFPH_PA10MFP_Msk |
                                       SYS_GPA_MFPH_PA11MFP_Msk)) |
                    (SYS_GPA_MFPH_PA10MFP_EBI_nWR |
                     SYS_GPA_MFPH_PA11MFP_EBI_nRD);

    /* EBI AD10, AD11 pins on PD.3, PD2*/
    SYS->GPD_MFPL = (SYS->GPD_MFPL & ~(SYS_GPD_MFPL_PD2MFP_Msk |
                                       SYS_GPD_MFPL_PD3MFP_Msk)) |
                    (SYS_GPD_MFPL_PD3MFP_EBI_AD10 |
                     SYS_GPD_MFPL_PD2MFP_EBI_AD11);

    /* EBI AD12, AD13 pins on PB.15, PB.14 */
    /* EBI AD14, AD15 pins on PB.13, PB.12 */
    /* EBI ADR16~18 pins on PB.11, PB.10, PB.9 and PB.8 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB8MFP_Msk |
                                       SYS_GPB_MFPH_PB9MFP_Msk |
                                       SYS_GPB_MFPH_PB10MFP_Msk |
                                       SYS_GPB_MFPH_PB11MFP_Msk |
                                       SYS_GPB_MFPH_PB12MFP_Msk |
                                       SYS_GPB_MFPH_PB13MFP_Msk |
                                       SYS_GPB_MFPH_PB14MFP_Msk |
                                       SYS_GPB_MFPH_PB15MFP_Msk)) |
                    (SYS_GPB_MFPH_PB8MFP_EBI_ADR19 |
                     SYS_GPB_MFPH_PB9MFP_EBI_ADR18 |
                     SYS_GPB_MFPH_PB10MFP_EBI_ADR17 |
                     SYS_GPB_MFPH_PB11MFP_EBI_ADR16 |
                     SYS_GPB_MFPH_PB12MFP_EBI_AD15 |
                     SYS_GPB_MFPH_PB13MFP_EBI_AD14 |
                     SYS_GPB_MFPH_PB14MFP_EBI_AD13 |
                     SYS_GPB_MFPH_PB15MFP_EBI_AD12);

    /* EBI WRL and WRH pins on PB.7 and PB.6 */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB6MFP_Msk |
                                       SYS_GPB_MFPL_PB7MFP_Msk)) |
                    (SYS_GPB_MFPL_PB6MFP_EBI_nWRH |
                     SYS_GPB_MFPL_PB7MFP_EBI_nWRL);

    /* EBI CS0~1 pins on PF3 and PF2 */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & ~(SYS_GPF_MFPL_PF2MFP_Msk |
                                       SYS_GPF_MFPL_PF3MFP_Msk)) |
                    (SYS_GPF_MFPL_PF3MFP_EBI_nCS0 |
                     SYS_GPF_MFPL_PF2MFP_EBI_nCS1);

    /* EBI ALE pin on PA.8 */
    /* EBI MCLK pin on PA.9 */
    SYS->GPA_MFPH = (SYS->GPA_MFPH & ~(SYS_GPA_MFPH_PA8MFP_Msk |
                                       SYS_GPA_MFPH_PA9MFP_Msk)) |
                    (SYS_GPA_MFPH_PA8MFP_EBI_ALE |
                     SYS_GPA_MFPH_PA9MFP_EBI_MCLK);
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

    /* Set HCLK clock divider to 1 */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(1);

    /* Switch UART0 clock source to HIRC */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART0SEL_Msk) | CLK_CLKSEL1_UART0SEL_HIRC;

    /* Enable PDMA peripheral clock */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;

    /* Enable peripheral clock */
    CLK->AHBCLK |= CLK_AHBCLK_EBICKEN_Msk;
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART0 RXD=PA.15 and TXD=PA.14 */
    SYS->GPA_MFPH = (SYS->GPA_MFPH & ~(SYS_GPA_MFPH_PA14MFP_Msk | SYS_GPA_MFPH_PA15MFP_Msk)) |
                    (SYS_GPA_MFPH_PA15MFP_UART0_RXD | SYS_GPA_MFPH_PA14MFP_UART0_TXD);
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------+\n");
    printf("|    EBI SRAM Sample Code on Bank0   |\n");
    printf("+------------------------------------+\n\n");

    printf("************************************************************************\n");
    printf("* Please connect BS616LV4017 SRAM to EBI bank0 before accessing !!     *\n");
    printf("* EBI pins settings:                                                   *\n");
    printf("*   - AD0 ~ AD5     on PC.0 ~ PC.5                                     *\n");
    printf("*   - AD6 ~ AD7     on PA.6 ~ PA.7                                     *\n");
    printf("*   - AD8 ~ AD9     on PC.6 ~ PC.7                                     *\n");
    printf("*   - AD10 ~ AD11   on PD.13  PD.2                                     *\n");
    printf("*   - AD12 ~ AD13   on PB.15 ~ PB.14                                   *\n");
    printf("*   - AD14 ~ AD15   on PB.13 ~ PB.12                                   *\n");
    printf("*   - ADR16 ~ ADR17 on PB.11 ~ PB.10                                   *\n");
    printf("*   - ADR18 ~ ADR19 on PB.9  ~ PB.8                                    *\n");
    printf("*   - nWR           on PA.10                                           *\n");
    printf("*   - nRD           on PA.11                                           *\n");
    printf("*   - nWRL          on PB.7                                            *\n");
    printf("*   - nWRH          on PB.6                                            *\n");
    printf("*   - nCS0          on PF.3                                            *\n");
    printf("*   - nCS1          on PF.2                                            *\n");
    printf("*   - nCS2          on PF.2                                            *\n");
    printf("*   - ALE           on PA.8                                            *\n");
    printf("*   - MCLK          on PA.9                                            *\n");
    printf("**********************************************************************\n\n");

    /* Configure multi-function pins for EBI 16-bit application */
    Configure_EBI_16BIT_Pins();

    /* Enable EBI function and configure data bus width is 16-bit, MCLK is HCLK/2 and CS active level is low */
    EBI->CTL0  = (EBI_MCLKDIV_2 << EBI_CTL_MCLKDIV_Pos) |
                 (0x3 << EBI_CTL_TALE_Pos) |
                 (EBI_CS_ACTIVE_LOW << EBI_CTL_CSPOLINV_Pos) |
                 EBI_CTL_DW16_Msk | EBI_CTL_EN_Msk ;
    EBI->TCTL0 = 0x03003318;

    /* Start SRAM test */
    SRAM_BS616LV4017(512 * 1024);

    /* EBI sram with PDMA test */
    AccessEBIWithPDMA();

    /* Disable EBI function */
    EBI->CTL0 &= ~EBI_CTL_EN_Msk;

    /* Disable EBI clock */
    CLK->AHBCLK &= ~CLK_AHBCLK_EBICKEN_Msk;

    printf("*** SRAM Test OK ***\n");

    while(1);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables for PDMA                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t PDMA_TEST_LENGTH = 64;
uint32_t SrcArray[64];
uint32_t DestArray[64];
uint32_t volatile u32IsTestOver = 0;

/**
 * @brief       DMA IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The DMA default IRQ, declared in startup_nuc400series.s.
 */
void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);

    if(status & PDMA_INTSTS_ABTIF_Msk)            /* abort */
    {
        if(PDMA_GET_ABORT_STS(PDMA) & PDMA_ABTSTS_ABTIF0_Msk)
            u32IsTestOver = 2;
        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_ABTSTS_ABTIF0_Msk);
    }
    else if(status & PDMA_INTSTS_TDIF_Msk)       /* done */
    {
        if(PDMA_GET_TD_STS(PDMA) & PDMA_TDSTS_TDIF0_Msk)
            u32IsTestOver = 1;
        PDMA_CLR_TD_FLAG(PDMA, PDMA_TDSTS_TDIF0_Msk);
    }
    else
        printf("unknown interrupt !!\n");
}

void AccessEBIWithPDMA(void)
{
    uint32_t i;
    uint32_t u32Result0 = 0x5A5A, u32Result1 = 0x5A5A;

    printf("[[ Access EBI with PDMA ]]\n");

    /* Enable PDMA clock source */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;

    for(i = 0; i < 64; i++)
    {
        SrcArray[i] = 0x76570000 + i;
        u32Result0 += SrcArray[i];
    }

    PDMA->CHCTL |= (1 << PDMA_CH);
    PDMA->DSCT[PDMA_CH].CTL &= ~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_TXWIDTH_Msk);
    /* transfer width is one word(32 bit) */
    PDMA->DSCT[PDMA_CH].CTL |= ((0x2 << PDMA_DSCT_CTL_TXWIDTH_Pos) | (PDMA_TEST_LENGTH << PDMA_DSCT_CTL_TXCNT_Pos));
    PDMA->DSCT[PDMA_CH].SA = (uint32_t)SrcArray;
    PDMA->DSCT[PDMA_CH].DA = EBI_BANK0_BASE_ADDR;
    PDMA->DSCT[PDMA_CH].CTL &= ~(PDMA_DSCT_CTL_SAINC_Msk | PDMA_DSCT_CTL_DAINC_Msk);
    PDMA->DSCT[PDMA_CH].CTL |= ((0x2 << PDMA_DSCT_CTL_SAINC_Pos) | (0x2 << PDMA_DSCT_CTL_DAINC_Pos));

    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC2_Msk) | (0x1F << PDMA_REQSEL0_3_REQSRC2_Pos);
    PDMA->DSCT[PDMA_CH].CTL = (PDMA->DSCT[PDMA_CH].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | 0x1;

    PDMA->DSCT[PDMA_CH].CTL &= ~(PDMA_DSCT_CTL_TXTYPE_Msk | PDMA_DSCT_CTL_BURSIZE_Msk);
    PDMA->DSCT[PDMA_CH].CTL |= (0x5 << PDMA_DSCT_CTL_BURSIZE_Pos); //burst size is 4

    PDMA->INTEN |= (1 << PDMA_CH);
    NVIC_EnableIRQ(PDMA_IRQn);

    u32IsTestOver = 0;
    PDMA->SWREQ = (1 << PDMA_CH);
    while(u32IsTestOver == 0);
    /* Transfer internal SRAM to EBI SRAM done */

    /* Clear internal SRAM data */
    for(i = 0; i < 64; i++)
    {
        SrcArray[i] = 0x0;
    }

    PDMA->DSCT[PDMA_CH].CTL &= ~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_TXWIDTH_Msk);
    /* transfer width is one word(32 bit) */
    PDMA->DSCT[PDMA_CH].CTL |= ((0x2 << PDMA_DSCT_CTL_TXWIDTH_Pos) | (PDMA_TEST_LENGTH << PDMA_DSCT_CTL_TXCNT_Pos));
    PDMA->DSCT[PDMA_CH].SA = EBI_BANK0_BASE_ADDR;
    PDMA->DSCT[PDMA_CH].DA = (uint32_t)SrcArray;
    PDMA->DSCT[PDMA_CH].CTL &= ~(PDMA_DSCT_CTL_SAINC_Msk | PDMA_DSCT_CTL_DAINC_Msk);
    PDMA->DSCT[PDMA_CH].CTL |= ((0x2 << PDMA_DSCT_CTL_SAINC_Pos) | (0x2 << PDMA_DSCT_CTL_DAINC_Pos));

    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC2_Msk) | (0x1F << PDMA_REQSEL0_3_REQSRC2_Pos);
    PDMA->DSCT[PDMA_CH].CTL = (PDMA->DSCT[PDMA_CH].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | 0x1;

    PDMA->DSCT[PDMA_CH].CTL &= ~(PDMA_DSCT_CTL_TXTYPE_Msk | PDMA_DSCT_CTL_BURSIZE_Msk);
    PDMA->DSCT[PDMA_CH].CTL |= (0x5 << PDMA_DSCT_CTL_BURSIZE_Pos); //burst size is 4

    PDMA->INTEN |= (1 << PDMA_CH);

    u32IsTestOver = 0;
    PDMA->SWREQ = (1 << PDMA_CH);
    while(u32IsTestOver == 0);
    /* Transfer EBI SRAM to internal SRAM done */
    for(i = 0; i < 64; i++)
    {
        u32Result1 += SrcArray[i];
    }

    if(u32IsTestOver == 1)
    {
        if((u32Result0 == u32Result1) && (u32Result0 != 0x5A5A))
        {
            printf("        PASS (0x%X)\n\n", u32Result0);
        }
        else
        {
            printf("        FAIL - data matched (0x%X)\n\n", u32Result0);
            while(1);
        }
    }
    else
    {
        printf("        PDMA fail\n\n");
        while(1);
    }

    PDMA->CHCTL = 0;
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/

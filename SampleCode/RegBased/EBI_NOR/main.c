/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 8 $
 * $Date: 18/07/20 3:57p $
 * @brief    Configure EBI interface to access MX29LV320T (NOR Flash) on EBI interface.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
extern void NOR_MX29LV320T_RESET(uint32_t u32Bank);
extern int32_t NOR_MX29LV320T_CheckStatus(uint32_t u32DstAddr, uint16_t u16Data, uint32_t u32TimeoutMs);
extern uint16_t NOR_MX29LV320T_READ(uint32_t u32Bank, uint32_t u32DstAddr);
extern int32_t NOR_MX29LV320T_WRITE(uint32_t u32Bank, uint32_t u32DstAddr, uint16_t u16Data);
extern void NOR_MX29LV320T_GET_ID(uint32_t u32Bank, uint16_t *pu16IDTable);
extern int32_t NOR_MX29LV320T_EraseChip(uint32_t u32Bank, uint32_t u32IsCheckBlank);


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
    uint32_t u32Addr, u32MaxEBISize;
    uint16_t u16WData, u16RData;
    uint16_t u16IDTable[2];

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-----------------------------------------+\n");
    printf("|    EBI Nor Flash Sample Code on Bank1   |\n");
    printf("+-----------------------------------------+\n\n");

    printf("************************************************************************\n");
    printf("* Please connect MX29LV320T nor flash to EBI bank1 before accessing !! *\n");
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
    printf("************************************************************************\n\n");

    /* Configure multi-function pins for EBI 16-bit application */
    Configure_EBI_16BIT_Pins();

    /* Enable EBI function and configure data bus width is 16-bit, MCLK is HCLK/2 and CS active level is low */
    EBI->CTL1  = (EBI_MCLKDIV_2 << EBI_CTL_MCLKDIV_Pos) |
                 (EBI_CS_ACTIVE_LOW << EBI_CTL_CSPOLINV_Pos) |
                 EBI_CTL_DW16_Msk | EBI_CTL_EN_Msk ;
    EBI->TCTL1 = 0x03003318;

    /* Step 1, check ID */
    NOR_MX29LV320T_GET_ID(EBI_BANK1, (uint16_t *)u16IDTable);
    printf(">> Manufacture ID: 0x%X, Device ID: 0x%X .... ", u16IDTable[0], u16IDTable[1]);
    if((u16IDTable[0] != 0xC2) || (u16IDTable[1] != 0x22A8))
    {
        printf("FAIL !!!\n\n");
        while(1);
    }
    else
    {
        printf("PASS !!!\n\n");
    }


    /* Step 2, erase chip */
    if(NOR_MX29LV320T_EraseChip(EBI_BANK1, TRUE) < 0)
        while(1);


    /* Step 3, program flash and compare data */
    printf(">> Run program flash test ......\n");
    u32MaxEBISize = EBI_MAX_SIZE;
    for(u32Addr = 0; u32Addr < u32MaxEBISize; u32Addr += 2)
    {
        u16WData = (0x7657 + u32Addr / 2) & 0xFFFF;
        if(NOR_MX29LV320T_WRITE(EBI_BANK1, u32Addr, u16WData) < 0)
        {
            printf("Program [0x%08X]: [0x%08X] FAIL !!!\n\n", (uint32_t)(EBI_BANK0_BASE_ADDR + (0x100000 * EBI_BANK1) + u32Addr), u16WData);
            while(1);
        }
        else
        {
            /* Show UART message ...... */
            if((u32Addr % 256) == 0)
                printf("Program [0x%X]:[0x%X] !!!       \r", (uint32_t)(EBI_BANK0_BASE_ADDR + (0x100000 * EBI_BANK1) + u32Addr), u16WData);
        }
    }

    for(u32Addr = 0; u32Addr < u32MaxEBISize; u32Addr += 2)
    {
        u16WData = (0x7657 + u32Addr / 2) & 0xFFFF;
        u16RData = NOR_MX29LV320T_READ(EBI_BANK1, u32Addr);
        if(u16WData != u16RData)
        {
            printf("Compare [0x%08X] FAIL !!! (W:0x%08X, R:0x%08X)\n\n", (uint32_t)(EBI_BANK0_BASE_ADDR + (0x100000 * EBI_BANK1) + u32Addr), u16WData, u16RData);
            while(1);
        }
        else
        {
            /* Show UART message ...... */
            if((u32Addr % 256) == 0)
                printf("Read [0x%08X]: [0x%08X] !!!         \r", (uint32_t)(EBI_BANK0_BASE_ADDR + (0x100000 * EBI_BANK1) + u32Addr), u16RData);
        }
    }
    printf(">> Program flash OK !!!                             \n\n");

    /* Disable EBI function */
    EBI->CTL1 &= ~EBI_CTL_EN_Msk;

    /* Disable EBI clock */
    CLK->AHBCLK &= ~CLK_AHBCLK_EBICKEN_Msk;

    while(1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/

/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 11 $
 * $Date: 18/06/20 10:17a $
 * @brief    Implement CRC in CRC-CCITT mode and get the CRC checksum result.
 *
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~32 MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for 32MHz clock ready */
    while((CLK->STATUS & CLK_STATUS_HXTSTB_Msk) != CLK_STATUS_HXTSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk ) | CLK_CLKSEL0_HCLKSEL_HIRC ;

    /* Switch UART0 clock source to XTAL */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART0SEL_Msk) | CLK_CLKSEL1_UART0SEL_HXT;

    /* Enable UART0 clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk ;

    /* Enable CRC clock */
    CLK->AHBCLK |= CLK_AHBCLK_CRCCKEN_Msk ;

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    const uint8_t acCRCSrcPattern[] = {0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38};
    uint32_t i, u32TargetChecksum = 0xA12B, u32CalChecksum = 0;
    uint16_t *p16SrcAddr;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+---------------------------------------------+\n");
    printf("|    CRC-CCITT Polynomial Mode Sample Code    |\n");
    printf("+---------------------------------------------+\n\n");

    printf("# Calculate [0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38] CRC-CCITT checksum value.\n");
    printf("    - Seed value is 0xFFFF             \n");
    printf("    - CPU write data length is 16-bit \n");
    printf("    - Checksum complement disable    \n");
    printf("    - Checksum reverse disable       \n");
    printf("    - Write data complement disable  \n");
    printf("    - Write data reverse disable     \n");
    printf("    - Checksum should be 0x%X        \n\n", u32TargetChecksum);

    /* Configure CRC controller for CRC-CCITT CPU mode */
    CRC_Open(CRC_CCITT, 0, 0xFFFF, CRC_CPU_WDATA_16);

    /* Convert 16-bit source data */
    p16SrcAddr = (uint16_t *)acCRCSrcPattern;

    /* Start to execute CRC-CCITT operation */
    for(i = 0; i < sizeof(acCRCSrcPattern) / 2; i++)
    {
        CRC_WRITE_DATA((p16SrcAddr[i] & 0xFFFF));
    }

    /* Get CRC-CCITT checksum value */
    u32CalChecksum = CRC_GetChecksum();
    printf("CRC checksum is 0x%X ... %s.\n", u32CalChecksum, (u32CalChecksum == u32TargetChecksum) ? "PASS" : "FAIL");

    /* Disable CRC function */
    CLK_DisableModuleClock(CRC_MODULE);

    while(1);
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

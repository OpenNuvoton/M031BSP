/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 18/07/09 7:01p $
 * @brief    Show how to use I2C interface to access EEPROM.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define I2C_EEPROM_ADDRESS  0x50

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable I2C0 clock */
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set I2C0 multi-function pins */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC0MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk)) |
                    (SYS_GPC_MFPL_PC0MFP_I2C0_SDA | SYS_GPC_MFPL_PC1MFP_I2C0_SCL);

    /* Lock protected registers */
    SYS_LockReg();
}

void I2C0_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C_DisableInt(I2C0);
    NVIC_DisableIRQ(I2C0_IRQn);

    /* Disable I2C0 and close I2C0 clock */
    I2C_Close(I2C0);
    CLK_DisableModuleClock(I2C0_MODULE);

}

void I2C0_Init(void)
{
    /* Open I2C module and set bus clock */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));
}

int main()
{
    uint32_t i;
    uint8_t txbuf[10] = {0}, rDataBuf[10] = {0};

    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("+-------------------------------------+\n");
    printf("|    Level1 I2C EEPROM Sample Code    |\n");
    printf("+-------------------------------------+\n\n");
    printf("\n");

    /* Init I2C0 to access EEPROM */
    I2C0_Init();

    /* Prepare data for transmission */
    printf("Write Data: ");
    for (i = 0; i < 10; i++)
    {
        txbuf[i] = (uint8_t) i + 3;
        printf("  %d",txbuf[i]);
    }
    printf("\n");

    /* Write 10 bytes data to EEPROM */
    while(I2C_WriteMultiBytesTwoRegs(I2C0, I2C_EEPROM_ADDRESS, 0x0000, txbuf, 10) < 10);
    printf("Multi bytes Write access Pass.....\n\n");

    /* Use Multi Bytes Read from EEPROM (Two Registers) */
    while(I2C_ReadMultiBytesTwoRegs(I2C0, I2C_EEPROM_ADDRESS, 0x0000, rDataBuf, 10) < 10);

    printf("Read Data:  ");
    for(i = 0; i < 10; i++)
        printf("  %d",rDataBuf[i]);
    printf("\n");
    printf("Multi bytes Read access Pass.....\n\n");

    /* Compare TX data and RX data */
    for(i = 0; i < 10; i++)
    {
        if(txbuf[i] != rDataBuf[i])
        {
            printf("Data compare fail... R[%d] Data: 0x%X\n", i, rDataBuf[i]);
            while(1);
        }
    }
    printf("Data compare done... [PASS]\n");

    while(1);
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 18/07/16 3:06p $
 * @brief    Show how to set USCI_I2C use Multi bytes API Read and Write data to Slave.
 *           This sample code needs to work with USCI_I2C_Slave.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_LENGTH    256
#define WRITE_LENGTH    32

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8DeviceAddr;
uint8_t g_au8MstTxData[TEST_LENGTH];
uint8_t g_u8MstRxData[TEST_LENGTH];

uint32_t UI2C_WriteMultiBytesTwoRegs(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, const uint8_t *data, uint32_t u32wLen)
{
    uint8_t u8Xfering = 1U, u8Addr = 1U, u8Ctrl = 0U;
    uint32_t u32txLen = 0U;
    enum UI2C_MASTER_EVENT eEvent = MASTER_SEND_START;

    UI2C_START(ui2c);                                                           /* Send START */

    while (u8Xfering)
    {
        while (!(UI2C_GET_PROT_STATUS(ui2c) & 0x3F00U));                     /* Wait UI2C new status occur */

        switch (UI2C_GET_PROT_STATUS(ui2c) & 0x3F00U)
        {
        case UI2C_PROTSTS_STARIF_Msk:
            UI2C_CLR_PROT_INT_FLAG(ui2c, UI2C_PROTSTS_STARIF_Msk);         /* Clear START INT Flag */
            UI2C_SET_DATA(ui2c, (u8SlaveAddr << 1U) | 0x00U);                 /* Write SLA+W to Register UI2C_TXDAT */
            eEvent = MASTER_SEND_ADDRESS;
            u8Ctrl = UI2C_CTL_PTRG;                                         /* Clear SI */
            break;

        case UI2C_PROTSTS_ACKIF_Msk:
            UI2C_CLR_PROT_INT_FLAG(ui2c, UI2C_PROTSTS_ACKIF_Msk);          /* Clear ACK INT Flag */

            if (eEvent == MASTER_SEND_ADDRESS)
            {
                UI2C_SET_DATA(ui2c, (uint8_t)(u16DataAddr & 0xFF00U) >> 8U);  /* Write Hi byte data address to UI2C_TXDAT */
                eEvent = MASTER_SEND_DATA;
            }
            else if (eEvent == MASTER_SEND_DATA)
            {
                if (u8Addr)
                {
                    UI2C_SET_DATA(ui2c, (uint8_t)(u16DataAddr & 0xFFU));         /* Write Lo byte data address to UI2C_TXDAT */
                    u8Addr = 0;
                }
                else
                {
                    if (u32txLen < u32wLen)
                    {
                        UI2C_SET_DATA(ui2c, data[u32txLen++]);                  /* Write data to UI2C_TXDAT */
                    }
                    else
                    {
                        u8Ctrl = (UI2C_CTL_PTRG | UI2C_CTL_STO);                /* Clear SI and send STOP */
                    }
                }
            }

            break;

        case UI2C_PROTSTS_NACKIF_Msk:
            UI2C_CLR_PROT_INT_FLAG(ui2c, UI2C_PROTSTS_NACKIF_Msk);         /* Clear NACK INT Flag */
            u8Ctrl = (UI2C_CTL_PTRG | UI2C_CTL_STO);                        /* Clear SI and send STOP */
            break;

        case UI2C_PROTSTS_STORIF_Msk:
            UI2C_CLR_PROT_INT_FLAG(ui2c, UI2C_PROTSTS_STORIF_Msk);     /* Clear STOP INT Flag */
            u8Ctrl = UI2C_CTL_PTRG;                                     /* Clear SI */
            u8Xfering = 0U;
            break;

        case UI2C_PROTSTS_ARBLOIF_Msk:                                      /* Arbitration Lost */
        default:                                                            /* Unknow status */
            u8Ctrl = (UI2C_CTL_PTRG | UI2C_CTL_STO);                        /* Clear SI and send STOP */
            break;
        }

        UI2C_SET_CONTROL_REG(ui2c, u8Ctrl);                                     /* Write controlbit to UI2C_CTL register */
    }

    return u32txLen;                                                            /* Return bytes length that have been transmitted */
}

uint32_t UI2C_ReadMultiBytesTwoRegs(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t *rdata, uint32_t u32rLen)
{
    uint8_t u8Xfering = 1U, u8Addr = 1U, u8Ctrl = 0U;
    uint32_t u32rxLen = 0U;
    enum UI2C_MASTER_EVENT eEvent = MASTER_SEND_START;

    UI2C_START(ui2c);                                                       /* Send START */

    while (u8Xfering)
    {
        while (!(UI2C_GET_PROT_STATUS(ui2c) & 0x3F00U));                     /* Wait UI2C new status occur */

        switch (UI2C_GET_PROT_STATUS(ui2c) & 0x3F00U)
        {
        case UI2C_PROTSTS_STARIF_Msk:
            UI2C_CLR_PROT_INT_FLAG(ui2c, UI2C_PROTSTS_STARIF_Msk);     /* Clear START INT Flag */

            if (eEvent == MASTER_SEND_START)
            {
                UI2C_SET_DATA(ui2c, (u8SlaveAddr << 1U) | 0x00U);         /* Write SLA+W to Register UI2C_TXDAT */
                eEvent = MASTER_SEND_ADDRESS;
            }
            else if (eEvent == MASTER_SEND_REPEAT_START)
            {
                UI2C_SET_DATA(ui2c, (u8SlaveAddr << 1U) | 0x01U);        /* Write SLA+R to Register TXDAT */
                eEvent = MASTER_SEND_H_RD_ADDRESS;
            }

            u8Ctrl = UI2C_CTL_PTRG;
            break;

        case UI2C_PROTSTS_ACKIF_Msk:
            UI2C_CLR_PROT_INT_FLAG(ui2c, UI2C_PROTSTS_ACKIF_Msk);      /* Clear ACK INT Flag */

            if (eEvent == MASTER_SEND_ADDRESS)
            {
                UI2C_SET_DATA(ui2c, (uint8_t)(u16DataAddr & 0xFF00U) >> 8U);  /* Write Hi byte address of register */
                eEvent = MASTER_SEND_DATA;
            }
            else if (eEvent == MASTER_SEND_DATA)
            {
                if (u8Addr)
                {
                    UI2C_SET_DATA(ui2c, (uint8_t)(u16DataAddr & 0xFFU));       /* Write Lo byte address of register */
                    u8Addr = 0;
                }
                else
                {
                    u8Ctrl = (UI2C_CTL_PTRG | UI2C_CTL_STA);                /* Send repeat START signal */
                    eEvent = MASTER_SEND_REPEAT_START;
                }
            }
            else if (eEvent == MASTER_SEND_H_RD_ADDRESS)
            {
                u8Ctrl = (UI2C_CTL_PTRG | UI2C_CTL_AA);
                eEvent = MASTER_READ_DATA;
            }
            else
            {
                rdata[u32rxLen++] = (uint8_t) UI2C_GET_DATA(ui2c);      /* Receive Data */

                if (u32rxLen < u32rLen - 1U)
                    u8Ctrl = (UI2C_CTL_PTRG | UI2C_CTL_AA);
                else
                    u8Ctrl = UI2C_CTL_PTRG;
            }

            break;

        case UI2C_PROTSTS_NACKIF_Msk:
            UI2C_CLR_PROT_INT_FLAG(ui2c, UI2C_PROTSTS_NACKIF_Msk);     /* Clear NACK INT Flag */

            if (eEvent == MASTER_READ_DATA)
                rdata[u32rxLen++] = (uint8_t) UI2C_GET_DATA(ui2c);                  /* Receive Data */

            u8Ctrl = (UI2C_CTL_PTRG | UI2C_CTL_STO);                        /* Clear SI and send STOP */

            break;

        case UI2C_PROTSTS_STORIF_Msk:
            UI2C_CLR_PROT_INT_FLAG(ui2c, UI2C_PROTSTS_STORIF_Msk);     /* Clear STOP INT Flag */
            u8Ctrl = UI2C_CTL_PTRG;                                     /* Clear SI */
            u8Xfering = 0U;
            break;

        case UI2C_PROTSTS_ARBLOIF_Msk:                                  /* Arbitration Lost */
        default:                                                        /* Unknow status */
            u8Ctrl = (UI2C_CTL_PTRG | UI2C_CTL_STO);                    /* Clear SI and send STOP */
            break;
        }

        UI2C_SET_CONTROL_REG(ui2c, u8Ctrl);                                 /* Write controlbit to UI2C_PROTCTL register */
    }

    return u32rxLen;                                                        /* Return bytes length that have been received */
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

    /* Enable UI2C0 clock */
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

    /* Set UI2C0 multi-function pins */
    SYS->GPA_MFPH = (SYS->GPA_MFPH & ~(SYS_GPA_MFPH_PA11MFP_Msk | SYS_GPA_MFPH_PA10MFP_Msk)) |
                    (SYS_GPA_MFPH_PA11MFP_USCI0_CLK | SYS_GPA_MFPH_PA10MFP_USCI0_DAT0);

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

void UI2C0_Init(uint32_t u32ClkSpeed)
{
    uint32_t u32Clkdiv;

    u32Clkdiv = (((((SystemCoreClock / 2) * 10) / (u32ClkSpeed)) + 5) / 10) - 1;

    if(u32Clkdiv < 8)
        u32Clkdiv = 8;

    /* Open USCI_I2C0 and set clock to 100k */
    UI2C0->CTL &= ~UI2C_CTL_FUNMODE_Msk;
    UI2C0->CTL = 4 << UI2C_CTL_FUNMODE_Pos;

    //Data format configuration
    // 8 bit data length
    UI2C0->LINECTL &= ~UI2C_LINECTL_DWIDTH_Msk;
    UI2C0->LINECTL |= 8 << UI2C_LINECTL_DWIDTH_Pos;

    // MSB data format
    UI2C0->LINECTL &= ~UI2C_LINECTL_LSB_Msk;

    UI2C0->BRGEN &= ~UI2C_BRGEN_CLKDIV_Msk;
    UI2C0->BRGEN |= (u32Clkdiv << UI2C_BRGEN_CLKDIV_Pos);


    /* Set UI2C1 Slave Addresses */
    UI2C0->DEVADDR0 = 0x15;   /* Slave Address : 0x15 */
    UI2C0->DEVADDR1 = 0x35;   /* Slave Address : 0x35 */

    /* Set UI2C1 Slave Addresses Msk */
    UI2C0->ADDRMSK0 = 0x1;   /* Slave Address : 0x1 */
    UI2C0->ADDRMSK1 = 0x4;   /* Slave Address : 0x4 */

    /* Enable UI2C0 protocol */
    UI2C0->PROTCTL |=  UI2C_PROTCTL_PROTEN_Msk;
}

int main()
{
    uint32_t i;

    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART0_Init();

    /* Init USCI_I2C0 */
    UI2C0_Init(100000);

    /*
        This sample code sets UI2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */
    printf("+---------------------------------------------------------+\n");
    printf("| UI2C Driver Sample Code for Multi Bytes Read/Write Test |\n");
    printf("| Needs to work with USCI_I2C_Slave sample code           |\n");
    printf("|                                                         |\n");
    printf("|      UI2C Master (I2C0) <---> UI2C Slave (I2C0)         |\n");
    printf("| !! This sample code requires two boards to test !!      |\n");
    printf("+---------------------------------------------------------+\n");

    printf("\n");
    printf("Configure UI2C0 as a Master\n");
    printf("The I/O connection to UI2C0:\n");
    printf("UI2C0_SDA(PA10), UI2C0_SCL(PA11)\n");
    printf("Press any key to continue\n");
    getchar();

    /* Slave address */
    g_u8DeviceAddr = 0x15;

    /* Prepare data for transmission */
    for (i = 0; i < TEST_LENGTH; i++)
    {
        g_au8MstTxData[i] = (uint8_t) i + 3;
    }

    for (i = 0; i < TEST_LENGTH; i += WRITE_LENGTH)
    {
        /* Write 32 bytes data to Slave */
        while (UI2C_WriteMultiBytesTwoRegs(UI2C0, g_u8DeviceAddr, i, &g_au8MstTxData[i], WRITE_LENGTH) < WRITE_LENGTH);
    }

    printf("Multi bytes Write access Pass.....\n");

    printf("\n");

    /* Use Multi Bytes Read from Slave (Two Registers) */
    while (UI2C_ReadMultiBytesTwoRegs(UI2C0, g_u8DeviceAddr, 0x0000, g_u8MstRxData, TEST_LENGTH) < TEST_LENGTH);

    /* Compare TX data and RX data */
    for (i = 0; i < TEST_LENGTH; i++)
    {
        if (g_au8MstTxData[i] != g_u8MstRxData[i])
            printf("Data compare fail... R[%d] Data: 0x%X\n", i, g_u8MstRxData[i]);
    }

    printf("Multi bytes Read access Pass.....\n");

    while (1);
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

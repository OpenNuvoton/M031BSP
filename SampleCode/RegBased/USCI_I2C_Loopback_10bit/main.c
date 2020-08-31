/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show a Master how to access 10-bit address Slave (loopback).
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define SLV_10BIT_ADDR (0x1E<<2)             //1111+0xx+r/w

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8DeviceHAddr;
volatile uint8_t g_u8DeviceLAddr;
volatile uint8_t g_au8SlvData[256];
volatile uint8_t g_au8SlvRxData[4];
volatile uint8_t g_au8MstTxData[3];
volatile uint8_t g_u8MstDataLen;
volatile uint8_t g_u8SlvDataLen;
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_u8MstRxData;
volatile uint8_t g_u8MstEndFlag = 0;
volatile uint32_t g_u32SlaveBuffAddr;
volatile uint16_t g_u16SlvRcvAddr;


volatile enum UI2C_MASTER_EVENT g_eMEvent;
volatile enum UI2C_SLAVE_EVENT g_eSEvent;

typedef void (*UI2C_FUNC)(uint32_t u32Status);

volatile static UI2C_FUNC s_UI2C0HandlerFn = NULL;
volatile static UI2C_FUNC s_UI2C1HandlerFn = NULL;
/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C IRQ Handler                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void USCI01_IRQHandler(void)
{
    uint32_t u32Status0, u32Status1;

    //UI2C0 Interrupt status
    u32Status0 = UI2C_GET_PROT_STATUS(UI2C0);
    //USCI1 Interrupt status
    u32Status1 = UI2C_GET_PROT_STATUS(UI2C1);

    if (u32Status0 != (uint32_t)NULL)
    {
        if (s_UI2C0HandlerFn != NULL)
            s_UI2C0HandlerFn(u32Status0);
    }

    if (u32Status1 != (uint32_t)NULL)
    {
        if (s_UI2C1HandlerFn != NULL)
            s_UI2C1HandlerFn(u32Status1);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C0 Rx Callback Function                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void UI2C_MasterRx(uint32_t u32Status)
{
    if ((UI2C0->PROTSTS & UI2C_PROTSTS_TOIF_Msk) == UI2C_PROTSTS_TOIF_Msk)
    {
        /* Clear USCI_I2C0 Timeout Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_TOIF_Msk);
    }
    else if ((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);  /* Clear START INT Flag */

        if (g_eMEvent == MASTER_SEND_START)
        {
            UI2C_SET_DATA(UI2C0, (g_u8DeviceHAddr << 1) | 0x00); /* Write SLA+W to Register TXDAT */
            g_eMEvent = MASTER_SEND_H_WR_ADDRESS;
        }
        else if (g_eMEvent == MASTER_SEND_REPEAT_START)
        {
            UI2C_SET_DATA(UI2C0, (g_u8DeviceHAddr << 1) | 0x01); /* Write SLA+R to Register TXDAT */
            g_eMEvent = MASTER_SEND_H_RD_ADDRESS;
        }

        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
    }
    else if ((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);  /* Clear ACK INT Flag */

        if (g_eMEvent == MASTER_SEND_H_WR_ADDRESS)
        {
            UI2C_SET_DATA(UI2C0, g_u8DeviceLAddr);
            g_eMEvent = MASTER_SEND_L_ADDRESS;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
        else if (g_eMEvent == MASTER_SEND_L_ADDRESS)
        {
            UI2C_SET_DATA(UI2C0, g_au8MstTxData[g_u8MstDataLen++]);  /* SLA+W has been transmitted and write ADDRESS to Register TXDAT */
            g_eMEvent = MASTER_SEND_DATA;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
        else if (g_eMEvent == MASTER_SEND_DATA)
        {
            if (g_u8MstDataLen != 2)
            {
                UI2C_SET_DATA(UI2C0, g_au8MstTxData[g_u8MstDataLen++]);  /* ADDRESS has been transmitted and write DATA to Register TXDAT */
                UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
            }
            else
            {
                g_eMEvent = MASTER_SEND_REPEAT_START;
                UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STA));    /* Send repeat START signal */
            }
        }
        else if (g_eMEvent == MASTER_SEND_H_RD_ADDRESS)
        {
            g_eMEvent = MASTER_READ_DATA;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
    }
    else if ((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk);      /* Clear NACK INT Flag */

        if (g_eMEvent == MASTER_SEND_H_WR_ADDRESS)
        {
            g_eMEvent = MASTER_SEND_START;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STA));    /* Send START signal */
        }
        else if (g_eMEvent == MASTER_SEND_L_ADDRESS)
        {
            g_eMEvent = MASTER_STOP;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));
        }
        else if (g_eMEvent == MASTER_READ_DATA)
        {
            g_u8MstRxData = (uint8_t) UI2C_GET_DATA(UI2C0);
            g_eMEvent = MASTER_STOP;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));    /* DATA has been received and send STOP signal */
        }
        else
            /* TO DO */
            printf("Status 0x%x is NOT processed\n", u32Status);
    }
    else if ((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STORIF_Msk);               /* Clear STOP INT Flag */
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        g_u8MstEndFlag = 1;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C0 Tx Callback Function                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void UI2C_MasterTx(uint32_t u32Status)
{
    if ((UI2C0->PROTSTS & UI2C_PROTSTS_TOIF_Msk) == UI2C_PROTSTS_TOIF_Msk)
    {
        /* Clear USCI_I2C0 Timeout Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_TOIF_Msk);
    }
    else if ((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);               /* Clear START INT Flag */

        UI2C_SET_DATA(UI2C0, (g_u8DeviceHAddr << 1) | 0x00);     /* Write SLA+W to Register TXDAT */
        g_eMEvent = MASTER_SEND_H_WR_ADDRESS;

        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
    }
    else if ((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);   /* Clear ACK INT Flag */

        /* Event process */
        if (g_eMEvent == MASTER_SEND_H_WR_ADDRESS)
        {
            UI2C_SET_DATA(UI2C0, g_u8DeviceLAddr);
            g_eMEvent = MASTER_SEND_L_ADDRESS;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
        else if (g_eMEvent == MASTER_SEND_L_ADDRESS)
        {
            UI2C_SET_DATA(UI2C0, g_au8MstTxData[g_u8MstDataLen++]);  /* SLA+W has been transmitted and write ADDRESS to Register TXDAT */
            g_eMEvent = MASTER_SEND_DATA;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
        else if (g_eMEvent == MASTER_SEND_DATA)
        {
            if (g_u8MstDataLen != 3)
            {
                UI2C_SET_DATA(UI2C0, g_au8MstTxData[g_u8MstDataLen++]);  /* ADDRESS has been transmitted and write DATA to Register TXDAT */
                UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
            }
            else
            {
                g_eMEvent = MASTER_STOP;
                UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));        /* Send STOP signal */
            }
        }
    }
    else if ((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk);  /* Clear NACK INT Flag */
        g_u8MstEndFlag = 0;

        if (g_eMEvent == MASTER_SEND_H_WR_ADDRESS)
        {
            /* SLA+W has been transmitted and NACK has been received */
            g_eMEvent = MASTER_SEND_START;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STA));            /* Send START signal */
        }
        else if (g_eMEvent == MASTER_SEND_L_ADDRESS)
        {
            /* ADDRESS has been transmitted and NACK has been received */
            g_eMEvent = MASTER_STOP;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));            /* Send STOP signal */
        }
        else if (g_eMEvent == MASTER_SEND_DATA)
        {
            /* ADDRESS has been transmitted and NACK has been received */
            g_eMEvent = MASTER_STOP;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));            /* Send STOP signal */
        }
        else
            printf("Get Wrong NACK Event\n");
    }
    else if ((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STORIF_Msk);               /* Clear STOP INT Flag */
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        g_u8MstEndFlag = 1;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C1 TRx Callback Function                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void UI2C_LB_SlaveTRx(uint32_t u32Status)
{
    uint32_t u32Temp;

    if ((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        /* Clear START INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_STARIF_Msk);

        /* Event process */
        g_u8SlvDataLen = 0;
        g_eSEvent = SLAVE_H_RD_ADDRESS_ACK;

        UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        /* Clear ACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_ACKIF_Msk);

        /* Event process */
        if (g_eSEvent == SLAVE_H_WR_ADDRESS_ACK)
        {
            g_u8SlvDataLen = 0;

            g_eSEvent = SLAVE_L_WR_ADDRESS_ACK;
            g_u16SlvRcvAddr = (uint8_t)UI2C_GET_DATA(UI2C1);
        }
        else if (g_eSEvent == SLAVE_H_RD_ADDRESS_ACK)
        {
            g_u8SlvDataLen = 0;

            UI2C_SET_DATA(UI2C1, g_au8SlvData[g_u32SlaveBuffAddr]);
            g_u32SlaveBuffAddr++;
            g_u16SlvRcvAddr = (uint8_t)UI2C_GET_DATA(UI2C1);
        }
        else if (g_eSEvent == SLAVE_L_WR_ADDRESS_ACK)
        {
            g_u8SlvDataLen = 0;

            if ((UI2C1->PROTSTS & UI2C_PROTSTS_SLAREAD_Msk) == UI2C_PROTSTS_SLAREAD_Msk)
            {
                UI2C_SET_DATA(UI2C1, g_au8SlvData[g_u32SlaveBuffAddr]);
                g_u32SlaveBuffAddr++;
            }
            else
            {
                g_eSEvent = SLAVE_GET_DATA;
            }

            g_u16SlvRcvAddr = (uint8_t)UI2C_GET_DATA(UI2C1);
            g_u16SlvRcvAddr = (uint8_t)UI2C_GET_DATA(UI2C1);
        }
        else if (g_eSEvent == SLAVE_L_RD_ADDRESS_ACK)
        {
            UI2C_SET_DATA(UI2C1, g_au8SlvData[g_u32SlaveBuffAddr]);
            g_u32SlaveBuffAddr++;
        }
        else if (g_eSEvent == SLAVE_GET_DATA)
        {
            u32Temp = UI2C_GET_DATA(UI2C1);
            g_au8SlvRxData[g_u8SlvDataLen] = u32Temp;
            g_u8SlvDataLen++;

            if (g_u8SlvDataLen == 2)
            {
                u32Temp = (g_au8SlvRxData[0] << 8);
                u32Temp += g_au8SlvRxData[1];
                g_u32SlaveBuffAddr = u32Temp;
            }

            if (g_u8SlvDataLen == 3)
            {
                u32Temp = g_au8SlvRxData[2];
                g_au8SlvData[g_u32SlaveBuffAddr] = u32Temp;
                g_u8SlvDataLen = 0;
            }
        }

        UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        /* Clear NACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_NACKIF_Msk);
        /* Event process */
        g_u8SlvDataLen = 0;
        g_eSEvent = SLAVE_H_WR_ADDRESS_ACK;

        UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        /* Clear STOP INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_STORIF_Msk);
        /* Event process */
        g_u8SlvDataLen = 0;
        g_eSEvent = SLAVE_L_WR_ADDRESS_ACK;
        UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_AA));
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

    /* Switch UART0 clock source to HIRC and UART0 clock divider as 1 */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART0SEL_Msk)) | CLK_CLKSEL1_UART0SEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UART0DIV_Msk)) | CLK_CLKDIV0_UART0(1);

    /* Enable peripheral clock */
    CLK->APBCLK1 |= (CLK_APBCLK1_USCI0CKEN_Msk | CLK_APBCLK1_USCI1CKEN_Msk);

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

    /* Set UI2C1 multi-function pins */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB1MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk)) |
                    (SYS_GPB_MFPL_PB1MFP_USCI1_CLK | SYS_GPB_MFPL_PB2MFP_USCI1_DAT0);

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

    /* Open UI2C0 and set clock to 100k */
    UI2C0->CTL &= ~UI2C_CTL_FUNMODE_Msk;
    UI2C0->CTL = 4 << UI2C_CTL_FUNMODE_Pos;

    /* 8-bit length data format configuration */
    UI2C0->LINECTL &= ~UI2C_LINECTL_DWIDTH_Msk;
    UI2C0->LINECTL |= 8 << UI2C_LINECTL_DWIDTH_Pos;

    /* Set MSB data format */
    UI2C0->LINECTL &= ~UI2C_LINECTL_LSB_Msk;

    /* Set USCI I2C baud rate */
    UI2C0->BRGEN &= ~UI2C_BRGEN_CLKDIV_Msk;
    UI2C0->BRGEN |= (u32Clkdiv << UI2C_BRGEN_CLKDIV_Pos);

    /* Set UI2C0 Slave Addresses */
    UI2C0->DEVADDR0 = 0x015;   /* Slave Address : 0x015 */
    UI2C0->DEVADDR1 = 0x035;   /* Slave Address : 0x035 */

    /* Set UI2C0 Slave Addresses Msk */
    UI2C0->ADDRMSK0 = 0x1;   /* Slave Address : 0x1 */
    UI2C0->ADDRMSK1 = 0x4;   /* Slave Address : 0x4 */

    /* Enable UI2C0 protocol interrupt */
    UI2C_ENABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk));
    NVIC_EnableIRQ(USCI_IRQn);

    /* Enable UI2C0 protocol */
    UI2C0->PROTCTL |=  UI2C_PROTCTL_PROTEN_Msk;
}

void UI2C1_Init(uint32_t u32ClkSpeed)
{
    uint32_t u32Clkdiv;

    u32Clkdiv = (((((SystemCoreClock / 2) * 10) / (u32ClkSpeed)) + 5) / 10) - 1;

    if(u32Clkdiv < 8)
        u32Clkdiv = 8;

    /* Open UI2C1 and set clock to 100k */
    UI2C1->CTL &= ~UI2C_CTL_FUNMODE_Msk;
    UI2C1->CTL = 4 << UI2C_CTL_FUNMODE_Pos;

    /* 8-bit legnt data format configuration */
    UI2C1->LINECTL &= ~UI2C_LINECTL_DWIDTH_Msk;
    UI2C1->LINECTL |= 8 << UI2C_LINECTL_DWIDTH_Pos;

    /* MSB data format */
    UI2C1->LINECTL &= ~UI2C_LINECTL_LSB_Msk;

    /* Set UI2C1 baud rate */
    UI2C1->BRGEN &= ~UI2C_BRGEN_CLKDIV_Msk;
    UI2C1->BRGEN |= (u32Clkdiv << UI2C_BRGEN_CLKDIV_Pos);

    /* Set UI2C1 Slave Addresses */
    UI2C1->DEVADDR0 = 0x116;   /* Slave Address : 0x116 */
    UI2C1->DEVADDR1 = 0x136;   /* Slave Address : 0x136 */

    /* Set UI2C1 Slave Addresses Msk */
    UI2C1->ADDRMSK0 = 0x4;   /* Slave Address : 0x4 */
    UI2C1->ADDRMSK1 = 0x2;   /* Slave Address : 0x2 */

    /* Enable UI2C1 protocol interrupt */
    UI2C_ENABLE_PROT_INT(UI2C1, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk));
    NVIC_EnableIRQ(USCI_IRQn);

    /* Enable UI2C1 10-bit address mode */
    UI2C_ENABLE_10BIT_ADDR_MODE(UI2C1);

    /* Enable UI2C1 protocol */
    UI2C1->PROTCTL |=  UI2C_PROTCTL_PROTEN_Msk;
}

int32_t ReadWriteSlave(uint16_t u16SlvAddr)
{
    uint32_t u32Idx;
    uint8_t u8Temp;

    /* Init Send 10-bit Addr */
    g_u8DeviceHAddr = (u16SlvAddr >> 8) | SLV_10BIT_ADDR;
    g_u8DeviceLAddr = u16SlvAddr & 0xFF;

    for (u32Idx = 0; u32Idx < 0x100; u32Idx++)
    {
        g_au8MstTxData[0] = (uint8_t)((u32Idx & 0xFF00) >> 8);
        g_au8MstTxData[1] = (uint8_t)(u32Idx & 0x00FF);
        g_au8MstTxData[2] = (uint8_t)(g_au8MstTxData[1] + 3);

        g_u8MstDataLen = 0;
        g_u8MstEndFlag = 0;

        /* USCI_I2C function to write data to slave */
        s_UI2C0HandlerFn = (UI2C_FUNC)UI2C_MasterTx;

        /* USCI_I2C as master sends START signal */
        g_eMEvent = MASTER_SEND_START;
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_STA);

        /* Wait USCI_I2C Tx Finish */
        while (g_u8MstEndFlag == 0);

        g_u8MstEndFlag = 0;

        /* USCI_I2C function to read data from slave */
        s_UI2C0HandlerFn = (UI2C_FUNC)UI2C_MasterRx;

        g_u8MstDataLen = 0;
        g_u8DeviceAddr = u16SlvAddr;

        g_eMEvent = MASTER_SEND_START;
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_STA);

        /* Wait USCI_I2C Rx Finish */
        while (g_u8MstEndFlag == 0);

        g_u8MstEndFlag = 0;

        /* Compare data */
        u8Temp = g_au8MstTxData[2];

        if (g_u8MstRxData != u8Temp)
        {
            printf("USCI_I2C Byte Write/Read Failed, Data 0x%x\n", g_u8MstRxData);
            return -1;
        }
    }

    printf("Master Access Slave (0x%X) Test OK\n", u16SlvAddr);
    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32Idx;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART0_Init();

    /*
        This sample code sets USCI_I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("+-------------------------------------------------------+\n");
    printf("|  USCI_I2C Driver Sample Code for Master access        |\n");
    printf("|  10-bit address Slave (Loopback)                      |\n");
    printf("+-------------------------------------------------------+\n");

    printf("\n");
    printf("Configure UI2C0 as a master, UI2C1 as Slave.\n");
    printf("The I/O connection for UI2C0:\n");
    printf("UI2C0_SDA(PA10), UI2C0_SCL(PA11)\n");
    printf("UI2C1_SDA(PB2), UI2C1_SCL(PB1)\n");

    /* Init USCI_I2C0, USCI_I2C1 */
    UI2C0_Init(100000);
    UI2C1_Init(100000);

    g_eSEvent = SLAVE_L_WR_ADDRESS_ACK;
    UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_AA));

    for (u32Idx = 0; u32Idx < 0x100; u32Idx++)
        g_au8SlvData[u32Idx] = 0;

    /* I2C1 function to Slave receive/transmit data */
    s_UI2C1HandlerFn = UI2C_LB_SlaveTRx;

    /* Master Access Slave with no address mask */
    printf("\n");
    printf(" == No Mask Address ==\n");
    ReadWriteSlave(0x116);
    ReadWriteSlave(0x136);
    printf("SLAVE Address test OK.\n");
    /* Master Access Slave with address mask */
    printf("\n");
    printf(" == Mask Address ==\n");
    ReadWriteSlave(0x116 & ~0x04);
    ReadWriteSlave(0x136 & ~0x02);
    printf("SLAVE Address Mask test OK.\n");

    while (1);
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

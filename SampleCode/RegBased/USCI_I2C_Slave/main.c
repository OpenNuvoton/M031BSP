/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 18/07/16 3:07p $
 * @brief    Show how a slave receives data from a master.
 *           This sample code needs to work with USCI_I2C_Master.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_LENGTH    256

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_au8SlvData[TEST_LENGTH];
volatile uint8_t g_u8DeviceAddr;
uint8_t g_au8SlvRxData[4];
volatile uint16_t g_u16RecvAddr;

volatile uint32_t slave_buff_addr;
volatile uint16_t g_u16SlvRcvAddr;
volatile uint8_t g_u8SlvDataLen;

volatile enum UI2C_SLAVE_EVENT s_Event;

typedef void (*UI2C_FUNC)(uint32_t u32Status);

volatile static UI2C_FUNC s_UI2C0HandlerFn = NULL;
/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C IRQ Handler                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void USCI01_IRQHandler(void)
{
    uint32_t u32Status;

    //UI2C0 Interrupt
    u32Status = UI2C_GET_PROT_STATUS(UI2C0);

    if (s_UI2C0HandlerFn != NULL)
    {
        s_UI2C0HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C0 TRx Callback Function                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void UI2C_SlaveTRx(uint32_t u32Status)
{
    uint8_t u8data;

    if ((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)                   /* Re-Start been received */
    {
        /* Clear START INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);

        /* Event process */
        s_Event = SLAVE_ADDRESS_ACK;

        /* Trigger USCI I2C */
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)                /* USCI I2C Bus have been received ACK */
    {
        /* Clear ACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);

        /* Event process */
        if (s_Event == SLAVE_ADDRESS_ACK)                                                   /* Address Data has been received */
        {
            /* Check address if match address 0 or address 1*/
            if ((UI2C0->ADMAT & UI2C_ADMAT_ADMAT0_Msk) == UI2C_ADMAT_ADMAT0_Msk)
            {
                /* Address 0 match */
                UI2C0->ADMAT = UI2C_ADMAT_ADMAT0_Msk;
            }
            else if ((UI2C0->ADMAT & UI2C_ADMAT_ADMAT1_Msk) == UI2C_ADMAT_ADMAT1_Msk)
            {
                /* Address 1 match */
                UI2C0->ADMAT = UI2C_ADMAT_ADMAT1_Msk;
            }
            else
            {
                printf("No Address Match!!!\n");

                while (1);
            }

            /* USCI I2C receives Slave command type */
            if ((UI2C0->PROTSTS & UI2C_PROTSTS_SLAREAD_Msk) == UI2C_PROTSTS_SLAREAD_Msk)
            {
                s_Event = SLAVE_SEND_DATA;                                                  /* Slave address read has been received */
                UI2C_SET_DATA(UI2C0, g_au8SlvData[slave_buff_addr]);
                slave_buff_addr++;
            }
            else
            {
                g_u8SlvDataLen = 0;                                                         /* Slave address write has been received */
                s_Event = SLAVE_GET_DATA;
            }

            /* Read address from USCI I2C RXDAT*/
            g_u16RecvAddr = (uint8_t)UI2C_GET_DATA(UI2C0);
        }
        else if (s_Event == SLAVE_GET_DATA)
        {
            /* Read data from USCI I2C RXDAT*/
            u8data = (uint8_t)UI2C_GET_DATA(UI2C0);

            if (g_u8SlvDataLen < 2)
            {
                g_au8SlvRxData[g_u8SlvDataLen++] = u8data;
                slave_buff_addr = (g_au8SlvRxData[0] << 8) + g_au8SlvRxData[1];
            }
            else
            {
                g_au8SlvData[slave_buff_addr++] = u8data;

                if (slave_buff_addr == TEST_LENGTH)
                {
                    slave_buff_addr = 0;
                }
            }
        }
        else if (s_Event == SLAVE_SEND_DATA)
        {
            /* Write transmit data to USCI I2C TXDAT*/
            UI2C0->TXDAT = g_au8SlvData[slave_buff_addr];
            slave_buff_addr++;

            if (slave_buff_addr > TEST_LENGTH)
            {
                slave_buff_addr = 0;
            }
        }

        /* Trigger USCI I2C */
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        /* Clear NACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk);

        /* Event process */
        g_u8SlvDataLen = 0;
        s_Event = SLAVE_ADDRESS_ACK;

        /* Trigger USCI I2C */
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        /* Clear STOP INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STORIF_Msk);

        g_u8SlvDataLen = 0;
        s_Event = SLAVE_ADDRESS_ACK;

        /* Trigger USCI I2C */
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
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

    /* Open UI2C0 and set clock to 100k */
    UI2C0->CTL &= ~UI2C_CTL_FUNMODE_Msk;
    UI2C0->CTL = 4 << UI2C_CTL_FUNMODE_Pos;

    //Data format configuration
    // 8 bit data length
    UI2C0->LINECTL &= ~UI2C_LINECTL_DWIDTH_Msk;
    UI2C0->LINECTL |= 8 << UI2C_LINECTL_DWIDTH_Pos;

    // MSB data format
    UI2C0->LINECTL &= ~UI2C_LINECTL_LSB_Msk;

    /* Set UI2C0 Clock divider */
    UI2C0->BRGEN &= ~UI2C_BRGEN_CLKDIV_Msk;
    UI2C0->BRGEN |= (u32Clkdiv << UI2C_BRGEN_CLKDIV_Pos);


    /* Set UI2C0 Slave Addresses */
    UI2C0->DEVADDR0 = 0x15;   /* Slave Address : 0x15 */
    UI2C0->DEVADDR1 = 0x35;   /* Slave Address : 0x35 */

    /* Set UI2C0 Slave Addresses Msk */
    UI2C0->ADDRMSK0 = 0x1;   /* Slave Address : 0x1 */
    UI2C0->ADDRMSK1 = 0x4;   /* Slave Address : 0x4 */

    /* Enable UI2C0 protocol interrupt */
    UI2C_ENABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk));
    NVIC_EnableIRQ(USCI_IRQn);

    /* Enable UI2C0 protocol */
    UI2C0->PROTCTL |=  UI2C_PROTCTL_PROTEN_Msk;
}

int main()
{
    uint32_t i;

    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART0_Init();

    /*
        This sample code sets USCI_I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("+--------------------------------------------------------+\n");
    printf("|  USCI_I2C Driver Sample Code for Master access 7-bit   |\n");
    printf("|  address Slave. This sample code needs work with       |\n");
    printf("|  USCI_I2C_Master sample code                           |\n");
    printf("|         UI2C0(Master)  <----> UI2C0(Slave)             |\n");
    printf("|  This sample code requires two boards for testing      |\n");
    printf("+--------------------------------------------------------+\n");

    printf("\n");
    printf("Configure UI2C0 as Slave.\n");
    printf("The I/O connection for UI2C0:\n");
    printf("UI2C0_SDA(PA10), UI2C0_SCL(PA11)\n");

    /* Init USCI_I2C0 */
    UI2C0_Init(100000);

    s_Event = SLAVE_ADDRESS_ACK;

    UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));

    for (i = 0; i < TEST_LENGTH; i++)
    {
        g_au8SlvData[i] = 0;
    }

    /* I2C function to Slave receive/transmit data */
    s_UI2C0HandlerFn = UI2C_SlaveTRx;

    while (1);
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

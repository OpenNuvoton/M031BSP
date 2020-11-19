/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 18/07/13 3:28p $
 * @brief
 *           Show how to control SMBus interface and use SMBus protocol between Host and Slave.
 *           This sample code needs to work with I2C_SMBus_Slave.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define SMBUS_ALERT_RESPONSE_ADDRESS 0x0C
#define SMBUS_DEFAULT_ADDRESS        0x61
#define ARP_COMMAND 0x01

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
const uint8_t g_u8MasterAddr[4] = {0x15, 0x35, 0x55, 0x75};
const uint8_t g_u8SlaveAddr[4] = {0x18, 0x38, 0x58, 0x78};

volatile uint8_t g_au8RxData[4];
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_au8TxData[4];
volatile uint8_t g_u8RxData;
volatile uint8_t g_u8DataLen0;
volatile uint8_t g_u8EndFlag = 0;
volatile uint8_t g_u8SendPEC = 0;
volatile uint8_t g_u8AlertInt0 = 0;
volatile uint8_t g_u8AlertAddrAck0 = 0;
volatile uint8_t g_u8PECErr = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);

static volatile I2C_FUNC s_I2C0HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);

    /* Check Transmit byte done interrupt flag */
    if ((I2C0->BUSSTS & I2C_BUSSTS_BCDONE_Msk) == I2C_BUSSTS_BCDONE_Msk)
    {
        I2C0->BUSSTS = I2C_BUSSTS_BCDONE_Msk;
        return;
    }

    /* Occur receive PEC packet error */
    if ((I2C0->BUSSTS & I2C_BUSSTS_PECERR_Msk) == I2C_BUSSTS_PECERR_Msk)
    {
        I2C0->BUSSTS = I2C_BUSSTS_PECERR_Msk;
        return;
    }

    /* Check Alert Interrupt when I2C0 is Host */
    if (((I2C0->BUSSTS & I2C_BUSSTS_ALERT_Msk) == I2C_BUSSTS_ALERT_Msk) &
            ((I2C0->BUSCTL & I2C_BUSCTL_BMHEN_Msk) == I2C_BUSCTL_BMHEN_Msk))
    {
        I2C0->BUSSTS = I2C_BUSSTS_ALERT_Msk;
        g_u8AlertInt0 = 1;
        return ;
    }

    if (I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        /* Clear I2C0 Timeout Flag */
        I2C0->TOCTL |= I2C_TOCTL_TOIF_Msk;
    }
    else
    {
        if (s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Master Rx Callback Function                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterRx(uint32_t u32Status)
{
    if (u32Status == 0x08)                           /* START has been transmitted and prepare SLA+W */
    {
        I2C_SET_DATA(I2C0, g_u8DeviceAddr << 1);     /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x18)                      /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen0++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x20)                      /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if (u32Status == 0x28)                      /* DATA has been transmitted and ACK has been received */
    {
        if (g_u8DataLen0 != 2)
        {
            I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen0++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA_SI);
        }
    }
    else if (u32Status == 0x10)                 /* Repeat START has been transmitted and prepare SLA+R */
    {
        I2C_SET_DATA(I2C0, ((g_u8DeviceAddr << 1) | 0x01));   /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x40)                 /* SLA+R has been transmitted and ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x58)                 /* DATA has been received and NACK has been returned */
    {
        g_u8RxData = (unsigned char) I2C_GET_DATA(I2C0);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
        g_u8EndFlag = 1;
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Master Tx Callback Function                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterTx(uint32_t u32Status)
{
    if (u32Status == 0x08)                      /* START has been transmitted */
    {
        I2C_SET_DATA(I2C0, g_u8DeviceAddr << 1);     /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x18)                 /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen0++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x20)                 /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
    {
        if (g_u8DataLen0 != 3)
        {
            I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen0++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            if (g_u8SendPEC == 0)
            {
                g_u8SendPEC = 1;
                I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
            }
            else
            {
                I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
                g_u8EndFlag = 1;
            }
        }
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Master Alert Callback Function                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterAlert(uint32_t u32Status)
{
    if (u32Status == 0x08)                      /* START has been transmitted */
    {
        I2C_SET_DATA(I2C0, (g_u8DeviceAddr << 1) + 1);             /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x18)                 /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen0++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x20)                 /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if (u32Status == 0x40)
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0x50)
    {
        if (g_u8DataLen0 == 0)
        {
            g_au8RxData[g_u8DataLen0] = (unsigned char) I2C_GET_DATA(I2C0);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
            g_u8DataLen0++;
        }
        else
        {
            g_au8RxData[g_u8DataLen0] = (unsigned char) I2C_GET_DATA(I2C0);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI_AA);
            g_u8AlertAddrAck0 = 1;
        }
    }
    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
    {
        if (g_u8DataLen0 != 3)
        {
            I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen0++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            if (g_u8SendPEC == 0)
            {
                g_u8SendPEC = 1;
                I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
            }
            else
            {
                I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
                g_u8EndFlag = 1;
            }
        }
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Master Default Address and Acknowledge by Manual Callback Function                                 */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterDefaultAddrACKM(uint32_t u32Status)
{
    if (u32Status == 0x08)                           /* START has been transmitted */
    {
        I2C_SET_DATA(I2C0, g_u8DeviceAddr << 1);     /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x18)                      /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen0++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x20)                      /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if (u32Status == 0x28)                      /* DATA has been transmitted and ACK has been received */
    {
        if (g_u8SendPEC == 0)
        {
            g_u8SendPEC = 1;
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            g_u8EndFlag = 1;
        }
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
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

    /* Enable UART0 clock and I2C controller */
    CLK->APBCLK0 |= (CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_I2C0CKEN_Msk);

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

    /* Set I2C0 multi-function pins */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk)) |
                    (SYS_GPB_MFPL_PB4MFP_I2C0_SDA | SYS_GPB_MFPL_PB5MFP_I2C0_SCL);

    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC2MFP_Msk | SYS_GPC_MFPL_PC3MFP_Msk)) |
                    (SYS_GPC_MFPL_PC2MFP_I2C0_SMBSUS | SYS_GPC_MFPL_PC3MFP_I2C0_SMBAL);

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

void I2C0_Init(void)
{
    uint32_t u32BusClock;

    /* Reset I2C0 */
    SYS->IPRST1 |=  SYS_IPRST1_I2C0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_I2C0RST_Msk;

    /* Enable I2C0 Controller */
    I2C0->CTL0 |= I2C_CTL0_I2CEN_Msk;

    /* I2C0 bus clock 100K divider setting, I2CLK = PCLK/(100K*4)-1 */
    u32BusClock = 100000;
    I2C0->CLKDIV = (uint32_t)(((SystemCoreClock * 10) / (u32BusClock * 4) + 5) / 10 - 1); /* Compute proper divider for I2C clock */

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", (SystemCoreClock / (((I2C0->CLKDIV) + 1) << 2)));

    /* Set I2C0 4 Slave Addresses */
    /* Slave Address : 0x15 */
    I2C0->ADDR0 = (I2C0->ADDR0 & ~I2C_ADDR0_ADDR_Msk) | (0x15 << I2C_ADDR0_ADDR_Pos);
    /* Slave Address : 0x35 */
    I2C0->ADDR1 = (I2C0->ADDR1 & ~I2C_ADDR1_ADDR_Msk) | (0x35 << I2C_ADDR1_ADDR_Pos);
    /* Slave Address : 0x55 */
    I2C0->ADDR2 = (I2C0->ADDR2 & ~I2C_ADDR2_ADDR_Msk) | (0x55 << I2C_ADDR2_ADDR_Pos);
    /* Slave Address : 0x75 */
    I2C0->ADDR3 = (I2C0->ADDR3 & ~I2C_ADDR3_ADDR_Msk) | (0x75 << I2C_ADDR3_ADDR_Pos);

    /* Enable I2C0 interrupt and set corresponding NVIC bit */
    I2C0->CTL0 |= I2C_CTL0_INTEN_Msk;
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C0_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C0->CTL0 &= ~I2C_CTL0_INTEN_Msk;
    NVIC_DisableIRQ(I2C0_IRQn);

    /* Disable I2C0 and close I2C0 clock */
    I2C0->CTL0 &= ~I2C_CTL0_I2CEN_Msk;
    CLK->APBCLK0 &= ~CLK_APBCLK0_I2C0CKEN_Msk;

}

int32_t SMBusSendByteTest(uint8_t slvaddr)
{
    uint32_t i;

    g_u8DeviceAddr = slvaddr;

    for (i = 0; i < 0x100; i++)
    {
        /* Init transmission bytes */
        g_au8TxData[0] = (uint8_t)((i & 0xFF00) >> 8);
        g_au8TxData[1] = (uint8_t)(i & 0x00FF);
        g_au8TxData[2] = (uint8_t)(g_au8TxData[1] + 3);

        g_u8DataLen0 = 0;
        g_u8EndFlag = 0;
        g_u8SendPEC = 0;

        /* I2C0 function to write data to slave */
        s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx;

        /* I2C0 as master sends START signal */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

        /* Wait I2C0 transmit finish */
        while (g_u8EndFlag == 0);

        g_u8EndFlag = 0;

        if (g_u8PECErr)
        {
            printf("PEC Check Error !\n");

            while (1);
        }
    }

    return 0;
}

int32_t SMBusAlertTest(uint8_t slvaddr)
{
    g_u8DeviceAddr = slvaddr;

    /* I2C function to Send Alert Response Address to bus */
    s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterAlert;

    /* I2C0 Send Start condition */
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

    /* Init receive data index */
    g_u8DataLen0 = 0;

    /* Waiting for Get Alert Address*/
    while (g_u8AlertAddrAck0 == 0);

    g_u8AlertAddrAck0 = 0;

    if (g_u8PECErr)
    {
        printf("PEC Check Error !\n");

        while (1);
    }

    return 0;
}

int32_t SMBusDefaultAddressTest(uint8_t slvaddr)
{
    g_u8DeviceAddr = slvaddr;

    /* Set Transmission ARP command */
    g_au8TxData[0] = ARP_COMMAND;

    g_u8DataLen0 = 0;
    g_u8EndFlag = 0;
    g_u8SendPEC = 0;

    /* I2C0 function to write data to slave */
    s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterDefaultAddrACKM;

    /* I2C0 as master sends START signal */
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

    /* Wait I2C0 transmit finish */
    while (g_u8EndFlag == 0);

    g_u8EndFlag = 0;

    if (g_u8PECErr)
    {
        printf("PEC Check Error !\n");

        while (1);
    }

    printf("\n");
    printf("Master Sends ARP Command(0x01) to Slave (0x%X) Test OK\n", slvaddr);
    return 0;
}


int main()
{
    uint32_t ch = 0;

    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART0_Init();

    while (ch != 0x30)
    {
        /*
            This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
            and Byte Read operations, and check if the read data is equal to the programmed data.
        */
        printf("\n");
        printf("+-------------------------------------------------------+\n");
        printf("|      I2C SMBUS Driver Sample Code                     |\n");
        printf("|                                                       |\n");
        printf("| I2C Master (I2C0) <---> I2C Slave(I2C0)               |\n");
        printf("+-------------------------------------------------------+\n");

        printf("Configure I2C0 as master.\n");
        printf("\n");
        printf("I2C0_SDA(PB.4), I2C0_SCL(PB.5), I2C_ALERT(PC.3), I2C_SUSPEND(PC.2)\n");

        /* Init I2C0 */
        I2C0_Init();

        printf("\n");
        printf("[1] SMBus Send Bytes Protocol with PEC Test\n");
        printf("[2] SMBus Alert Function Test\n");
        printf("[3] Simple ARP and ACK Control by Manual Test\n");
        printf("[0] Exit\n");
        ch = getchar();

        if (ch == '1')
        {
            /* I2C0 Bus Management enable */
            I2C0->BUSCTL = (I2C0->BUSCTL & ~I2C_BUSCTL_BMDEN_Msk) | (I2C_BUSCTL_BMHEN_Msk | I2C_BUSCTL_BUSEN_Msk);

            /* I2C0 Bus PEC Check and transmit enable */
            I2C0->BUSCTL |= (I2C_BUSCTL_PECEN_Msk | I2C_BUSCTL_PECTXEN_Msk);

            /* Set I2C0 Payload bytes */
            I2C0->PKTSIZE = 4;            // I2C0: 1byte address + 3byte data

            g_u8PECErr = 0;

            /* Access Slave with no address mask */
            printf("\n");
            printf(" == SMBus Send Bytes Protocol test ==\n");

            /* SMBus send byte protocol test*/
            SMBusSendByteTest(g_u8SlaveAddr[0]);

            printf("\n");
            printf("SMBus transmit data done.\n");
            printf("Press any key to exit\n");
            getchar();
        }
        else if (ch == '2')
        {
            /* I2C0 Bus Management Enable */
            I2C0->BUSCTL = (I2C0->BUSCTL & ~I2C_BUSCTL_BMDEN_Msk) | (I2C_BUSCTL_BMHEN_Msk | I2C_BUSCTL_BUSEN_Msk);

            /* I2C0 Bus PEC Check and transmit Enable */
            I2C0->BUSCTL = (I2C0->BUSCTL & ~I2C_BUSCTL_PECTXEN_Msk) | I2C_BUSCTL_PECEN_Msk;

            /* Set I2C0 Payload bytes */
            I2C0->PKTSIZE = 2;            // I2C0: 1byte address + 1byte data

            /* Alert pin support if BMHEN(I2C0->BUSCTL[4]) = 0 */
            I2C_SMBUS_ENABLE_ALERT(I2C0);

            /* Enable Host SUSCON pin function and output Hi */
            I2C_SMBUS_SET_SUSCON_OUT(I2C0);
            I2C_SMBUS_SET_SUSCON_HIGH(I2C0);

            g_u8PECErr = 0;

            /* Access Slave with no address mask */
            printf("\n");
            printf(" == SMBus Alert Function Test ==\n");

            /* Wait I2C0 get Alert interrupt */
            while (g_u8AlertInt0 == 0);

            /* I2C0 Get Alert Request */
            g_u8AlertInt0 = 0;
            printf("\n");
            printf("I2C0 Get Alert Interrupt Request\n");

            /* I2C0 Send Alert Response Address(ARA) to I2C bus */
            SMBusAlertTest(SMBUS_ALERT_RESPONSE_ADDRESS);

            /* Printf the Alert Slave address */
            printf("Get Alert Address 0x%X test OK.\n", g_au8RxData[0]);

            printf("\n");
            printf("Press any key to pull I2C0 SUSCON Pin output Lo\n");
            getchar();

            /* Output I2C0 SUSCON pin Low */
            I2C_SMBUS_SET_SUSCON_LOW(I2C0);
            printf("I2C0 SUSCON Pin output Lo\n");

            printf("\n");
            printf("SMBus Alert Test Done\n");
            printf("Press any key to exit\n");
            getchar();
        }
        else if (ch == '3')
        {
            /* I2C0 Bus management enable */
            I2C0->BUSCTL = (I2C0->BUSCTL & ~I2C_BUSCTL_BMDEN_Msk) | (I2C_BUSCTL_BMHEN_Msk | I2C_BUSCTL_BUSEN_Msk);

            /* I2C0 Bus PEC check and transmit enable */
            I2C0->BUSCTL |= (I2C_BUSCTL_PECEN_Msk | I2C_BUSCTL_PECTXEN_Msk);

            /* Set I2C0 Payload bytes */
            I2C0->PKTSIZE = 2;            // I2C0: 1byte address + 1byte data

            printf("\n");
            printf("== Simple ARP and Acknowledge by Manual Test ==\n");

            /* I2C0 sends Default Address and ARP Command (0x01) to Slave */
            SMBusDefaultAddressTest(SMBUS_DEFAULT_ADDRESS);

            printf("Press any key to exit\n");
            getchar();
        }
    }

    s_I2C0HandlerFn = NULL;

    printf("SMBus Test Exit\n");

    /* Close I2C0 */
    I2C0_Close();

    while (1);
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

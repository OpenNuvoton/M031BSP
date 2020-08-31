/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 18/07/13 3:29p $
 * @brief
 *           Show how to use I2C Single byte API Read and Write data to Slave.
 *           This sample code needs to work with I2C_Slave.
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
volatile uint8_t g_u8DeviceAddr;

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
}

void I2C0_Close(void)
{
    /* Disable I2C0 and close I2C0 clock */
    I2C0->CTL0 &= ~I2C_CTL0_I2CEN_Msk;
    CLK->APBCLK0 &= ~CLK_APBCLK0_I2C0CKEN_Msk;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Write Single Byte                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t I2C_WriteByteTwoRegs(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, const uint8_t data)
{
    uint8_t u8Xfering = 1, u8Err = 0, u8Addr = 1, u8Ctrl = 0;
    uint32_t u32txLen = 0;

    I2C_START(i2c);                                                         /* Send START */

    while (u8Xfering && (u8Err == 0))
    {
        I2C_WAIT_READY(i2c);

        switch (I2C_GET_STATUS(i2c))
        {
        case 0x08:
            I2C_SET_DATA(i2c, (u8SlaveAddr << 1 | 0x00));               /* Write SLA+W to Register I2CDAT */
            u8Ctrl = I2C_CTL_SI;                                        /* Clear SI */
            break;

        case 0x18:                                                      /* Slave Address ACK */
            I2C_SET_DATA(i2c, (uint8_t)(u16DataAddr & 0xFF00) >> 8);    /* Write Hi byte address of register */
            break;

        case 0x20:                                                      /* Slave Address NACK */
        case 0x30:                                                      /* Master transmit data NACK */
            u8Ctrl = I2C_CTL_STO_SI;                                    /* Clear SI and send STOP */
            u8Err = 1;
            break;

        case 0x28:
            if (u8Addr)
            {
                I2C_SET_DATA(i2c, (uint8_t)(u16DataAddr & 0xFF));       /* Write Lo byte address of register */
                u8Addr = 0;
            }
            else if ((u32txLen < 1) && (u8Addr == 0))
            {
                I2C_SET_DATA(i2c, data);
                u32txLen++;
            }
            else
            {
                u8Ctrl = I2C_CTL_STO_SI;                              /* Clear SI and send STOP */
                u8Xfering = 0;
            }

            break;

        case 0x38:                                                      /* Arbitration Lost */
        default:                                                        /* Unknow status */
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO_SI);                   /* Clear SI and send STOP */
            u8Ctrl = I2C_CTL_SI;
            u8Err = 1;
            break;
        }

        I2C_SET_CONTROL_REG(i2c, u8Ctrl);                                   /* Write controlbit to I2C_CTL register */
    }

    return (u8Err | u8Xfering);                                             /* return (Success)/(Fail) status */
}

/*---------------------------------------------------------------------------------------------------------*/
/* Read Single Byte                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t I2C_ReadByteTwoRegs(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr)
{
    uint8_t u8Xfering = 1, u8Err = 0, rdata = 0, u8Addr = 1, u8Ctrl = 0;

    I2C_START(i2c);                                                         /* Send START */

    while (u8Xfering && (u8Err == 0))
    {
        I2C_WAIT_READY(i2c);

        switch (I2C_GET_STATUS(i2c))
        {
        case 0x08:
            I2C_SET_DATA(i2c, (u8SlaveAddr << 1 | 0x00));               /* Write SLA+W to Register I2CDAT */
            u8Ctrl = I2C_CTL_SI;                                      /* Clear SI */
            break;

        case 0x18:                                                      /* Slave Address ACK */
            I2C_SET_DATA(i2c, (uint8_t)(u16DataAddr & 0xFF00) >> 8);    /* Write Hi byte address of register */
            break;

        case 0x20:                                                      /* Slave Address NACK */
        case 0x30:                                                      /* Master transmit data NACK */
            u8Ctrl = I2C_CTL_STO_SI;                                  /* Clear SI and send STOP */
            u8Err = 1;
            break;

        case 0x28:
            if (u8Addr)
            {
                I2C_SET_DATA(i2c, (uint8_t)(u16DataAddr & 0xFF));       /* Write Lo byte address of register */
                u8Addr = 0;
            }
            else
                u8Ctrl = I2C_CTL_STA_SI;                              /* Clear SI and send repeat START */

            break;

        case 0x10:
            I2C_SET_DATA(i2c, ((u8SlaveAddr << 1) | 0x01));             /* Write SLA+R to Register I2CDAT */
            u8Ctrl = I2C_CTL_SI;                                      /* Clear SI */
            break;

        case 0x40:                                                      /* Slave Address ACK */
            u8Ctrl = I2C_CTL_SI;                                      /* Clear SI */
            break;

        case 0x48:                                                      /* Slave Address NACK */
            u8Ctrl = I2C_CTL_STO_SI;                                  /* Clear SI and send STOP */
            u8Err = 1;
            break;

        case 0x58:
            rdata = (unsigned char) I2C_GET_DATA(i2c);                  /* Receive Data */
            u8Ctrl = I2C_CTL_STO_SI;                                  /* Clear SI and send STOP */
            u8Xfering = 0;
            break;

        case 0x38:                                                      /* Arbitration Lost */
        default:                                                        /* Unknow status */
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO_SI);                   /* Clear SI and send STOP */
            u8Ctrl = I2C_CTL_SI;
            u8Err = 1;
            break;
        }

        I2C_SET_CONTROL_REG(i2c, u8Ctrl);                                   /* Write controlbit to I2C_CTL register */
    }

    if (u8Err)
        rdata = 0;                                                          /* If occurs error, return 0 */

    return rdata;                                                           /* Return read data */
}

int main()
{
    uint32_t i;
    uint8_t u8data, u8tmp, err;

    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART0_Init();

    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */
    printf("+--------------------------------------------------------+\n");
    printf("| I2C Driver Sample Code for Single Byte Read/Write Test |\n");
    printf("| Needs to work with I2C_Slave sample code               |\n");
    printf("|                                                        |\n");
    printf("| I2C Master (I2C0) <---> I2C Slave(I2C0)                |\n");
    printf("| !! This sample code requires two borads to test !!     |\n");
    printf("+--------------------------------------------------------+\n");

    printf("\n");
    printf("Configure I2C0 as Master\n");
    printf("The I/O connection to I2C0\n");
    printf("I2C0_SDA(PB.4), I2C0_SCL(PB.5)\n\n");

    /* Init I2C0 */
    I2C0_Init();

    /* Slave Address */
    g_u8DeviceAddr = 0x15;

    err = 0;

    for (i = 0; i < TEST_LENGTH; i++)
    {
        u8tmp = (uint8_t)i + 3;

        /* Single Byte Write (Two Registers) */
        while (I2C_WriteByteTwoRegs(I2C0, g_u8DeviceAddr, i, u8tmp));

        /* Single Byte Read (Two Registers) */
        u8data = I2C_ReadByteTwoRegs(I2C0, g_u8DeviceAddr, i);

        if (u8data != u8tmp)
        {
            err = 1;
            printf("%03d: Single byte write data fail,  W(0x%X)/R(0x%X) \n", i, u8tmp, u8data);
        }
    }

    printf("\n");

    if (err)
        printf("Single byte Read/Write access Fail.....\n");
    else
        printf("Single byte Read/Write access Pass.....\n");

    while (1);
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

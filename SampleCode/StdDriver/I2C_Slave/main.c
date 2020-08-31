/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 18/07/09 7:03p $
 * @brief
 *           Demonstrate how to set I2C in Slave mode to receive 256 bytes data from a master.
 *           This sample code needs to work with I2C_Master.
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
volatile uint32_t slave_buff_addr;
volatile uint8_t g_au8SlvData[TEST_LENGTH];
volatile uint8_t g_au8SlvRxData[3];

volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_u8SlvDataLen;

typedef void (*I2C_FUNC)(uint32_t u32Status);

volatile static I2C_FUNC s_I2C0HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);

    if(I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
    }
    else
    {
        if(s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C TRx Callback Function                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveTRx(uint32_t u32Status)
{
    uint8_t u8data;

    if(u32Status == 0x60)                       /* Own SLA+W has been receive; ACK has been return */
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        u8data = (unsigned char) I2C_GET_DATA(I2C0);
        if(g_u8SlvDataLen < 2)
        {
            g_au8SlvRxData[g_u8SlvDataLen++] = u8data;
            slave_buff_addr = (g_au8SlvRxData[0] << 8) + g_au8SlvRxData[1];
        }
        else
        {
            g_au8SlvData[slave_buff_addr++] = u8data;
            if(slave_buff_addr == TEST_LENGTH)
            {
                slave_buff_addr = 0;
            }
        }

        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA8)                  /* Own SLA+R has been receive; ACK has been return */
    {
        I2C_SET_DATA(I2C0, g_au8SlvData[slave_buff_addr]);
        slave_buff_addr++;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xB8)                  /* Data byte in I2CDAT has been transmitted ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_au8SlvData[slave_buff_addr++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
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
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk)) |
                    (SYS_GPB_MFPL_PB4MFP_I2C0_SDA | SYS_GPB_MFPL_PB5MFP_I2C0_SCL);

    /* Lock protected registers */
    SYS_LockReg();
}

void I2C0_Init(void)
{
    /* Open I2C module and set bus clock */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

    /* Set I2C 4 Slave Addresses */
    I2C_SetSlaveAddr(I2C0, 0, 0x15, 0);   /* Slave Address : 0x15 */
    I2C_SetSlaveAddr(I2C0, 1, 0x35, 0);   /* Slave Address : 0x35 */
    I2C_SetSlaveAddr(I2C0, 2, 0x55, 0);   /* Slave Address : 0x55 */
    I2C_SetSlaveAddr(I2C0, 3, 0x75, 0);   /* Slave Address : 0x75 */

    I2C_SetSlaveAddrMask(I2C0, 0, 0x01);
    I2C_SetSlaveAddrMask(I2C0, 1, 0x04);
    I2C_SetSlaveAddrMask(I2C0, 2, 0x01);
    I2C_SetSlaveAddrMask(I2C0, 3, 0x04);

    /* Enable I2C interrupt */
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
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

int main()
{
    uint32_t i;

    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */
    printf("\n");
    printf("+--------------------------------------------------------+\n");
    printf("| I2C Driver Sample Code(Slave) for access Slave         |\n");
    printf("|  Needs to work with I2C_Master sample code.            |\n");
    printf("| I2C Master (I2C0) <---> I2C Slave(I2C0)                |\n");
    printf("| !! This sample code requires two borads to test !!     |\n");
    printf("+--------------------------------------------------------+\n");

    printf("Configure I2C0 as a slave.\n");
    printf("The I/O connection for I2C0:\n");
    printf("I2C0_SDA(PB.4), I2C0_SCL(PB.5)\n");

    /* Init I2C0 */
    I2C0_Init();

    /* I2C enter no address SLV mode */
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);

    for(i = 0; i < TEST_LENGTH; i++)
    {
        g_au8SlvData[i] = 0;
    }

    /* I2C function to Slave receive/transmit data */
    s_I2C0HandlerFn = I2C_SlaveTRx;

    printf("\n");
    printf("I2C Slave Mode is Running.\n");

    while(1);

}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

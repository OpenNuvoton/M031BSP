/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 18/07/09 7:02p $
 * @brief
 *           Show how a master accesses a slave using PDMA TX and PDMA RX mode (Loopback).
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define I2C_PDMA_CH        1
#define I2C_PDMA_TX_LENGTH 100
#define I2C_PDMA_RX_LENGTH I2C_PDMA_TX_LENGTH - 3  //I2C1 will receive 97 bytes data (only data)
#define I2C_TEST_LENGTH    256
/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t slave_buff_addr;
volatile uint8_t g_au8SlvData[I2C_TEST_LENGTH];
volatile uint8_t g_au8SlvRxData[3];
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_au8MstTxData[I2C_TEST_LENGTH] = {0};
volatile uint8_t g_au8MstRxData[I2C_TEST_LENGTH] = {0};
volatile uint8_t g_u8MstRxData;
volatile uint8_t g_u8MstDataLen;
volatile uint8_t g_u8MstEndFlag = 0;
volatile int32_t g_u32IsTestOver;
volatile uint8_t g_u8MstTxSLA;
volatile uint16_t g_u16SlvDataLen;

typedef void (*I2C_FUNC)(uint32_t u32Status);
volatile static I2C_FUNC s_I2C0HandlerFn = NULL;
volatile static I2C_FUNC s_I2C1HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/* PDMA IRQ Interrupt                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);

    if(status & PDMA_INTSTS_ABTIF_Msk)    /* abort */
    {
        /* Check if channel 1 has abort error */
        if(PDMA_GET_ABORT_STS(PDMA) & PDMA_ABTSTS_ABTIF1_Msk)
            g_u32IsTestOver = 2;
        /* Clear abort flag of channel 1 */
        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_ABTSTS_ABTIF1_Msk);
    }
    else if(status & PDMA_INTSTS_TDIF_Msk)      /* done */
    {
        /* Check transmission of channel 1 has been transfer done */
        if(PDMA_GET_TD_STS(PDMA) & PDMA_TDSTS_TDIF1_Msk)
            g_u32IsTestOver = 1;
        /* Clear transfer done flag of channel 1 */
        PDMA_CLR_TD_FLAG(PDMA, PDMA_TDSTS_TDIF1_Msk);
    }
    else
        printf("unknown interrupt !!\n");
}

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
/*  I2C1 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C1_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C1);

    if(I2C_GET_TIMEOUT_FLAG(I2C1))
    {
        /* Clear I2C1 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C1);
    }
    else
    {
        if(s_I2C1HandlerFn != NULL)
            s_I2C1HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 Slave TRx Callback Function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveTRx(uint32_t u32Status)
{
    uint8_t u8data;

    if(u32Status == 0x60)                       /* Own SLA+W has been receive; ACK has been return */
    {
        g_u16SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        u8data = (unsigned char) I2C_GET_DATA(I2C0);
        if(g_u16SlvDataLen < 2)
        {
            g_au8SlvRxData[g_u16SlvDataLen++] = u8data;
            slave_buff_addr = (g_au8SlvRxData[0] << 8) + g_au8SlvRxData[1];
        }
        else if(g_u16SlvDataLen >= 2)
        {
            g_au8SlvData[slave_buff_addr++] = u8data;
            if(slave_buff_addr == I2C_TEST_LENGTH)
            {
                slave_buff_addr = 0;
            }
        }

        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA8)                  /* Own SLA+R has been receive; ACK has been return */
    {
        I2C_SET_DATA(I2C0, g_au8SlvData[slave_buff_addr++]);

        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xB8)
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
        g_u16SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u16SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 PDMA Master Tx Callback Function                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_PDMA_MasterTx(uint32_t u32Status)
{
    if(u32Status == 0x08)                       /* START has been transmitted */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur START interrupt
        */
    }
    else if(u32Status == 0x10)                  /* Repeat START has been transmitted */
    {

    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur address ACK interrupt
        */
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C1);
        I2C_START(I2C1);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur data ACK interrupt
        */
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 PDMA Master Rx Callback Function                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_PDMA_MasterRx(uint32_t u32Status)
{
    if(u32Status == 0x08)                          /* START has been transmitted and prepare SLA+W */
    {
        I2C1->DAT = (g_au8MstTxData[g_u8MstDataLen++] | 0x00);     /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
    }
    else if(u32Status == 0x18)                     /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C1, g_au8MstTxData[g_u8MstDataLen++]);
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                     /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C1);
        I2C_START(I2C1);
    }
    else if(u32Status == 0x28)                     /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8MstDataLen == 2)
        {
            I2C_SET_DATA(I2C1, g_au8MstTxData[g_u8MstDataLen++]);
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STA_SI);
        }
    }
    else if(u32Status == 0x10)                    /* Repeat START has been transmitted and prepare SLA+R */
    {
        g_u8MstDataLen = 0;
        I2C_SET_DATA(I2C1, (g_au8MstTxData[0] | 0x01));   /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
    }
    else if(u32Status == 0x40)                    /* SLA+R has been transmitted and ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
        g_u8MstTxSLA = 1;                         /* PDMA can start PDMA receive */
    }
    else if(u32Status == 0x50)                    /* DATA has been received and ACK has been returned */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur receive data ACK interrupt
        */
    }
    else if(u32Status == 0x58)                    /* DATA has been received and NACK has been returned */
    {
        I2C_STOP(I2C1);
        g_u8MstEndFlag = 1;
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* I2C Master PDMA Tx Channel Configuration                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_Master_PDMA_Tx_Init(void)
{
    /* Reset PDMA module */
    SYS_ResetModule(PDMA_RST);

    /* Open Channel 1 */
    PDMA_Open(PDMA, 1 << I2C_PDMA_CH);

    /* Transfer count is PDMA_TEST_LENGTH, transfer width is 8 bits(one byte) */
    PDMA_SetTransferCnt(PDMA, I2C_PDMA_CH, PDMA_WIDTH_8, I2C_PDMA_TX_LENGTH);

    /* Set source address is g_au8MstTxData, destination address is (I2C1->DAT), Source/Destination increment size is 32 bits(one word) */
    PDMA_SetTransferAddr(PDMA, I2C_PDMA_CH, (uint32_t)g_au8MstTxData, PDMA_SAR_INC, (uint32_t)(&(I2C1->DAT)), PDMA_DAR_FIX);

    /* Single request type */
    PDMA_SetBurstType(PDMA, I2C_PDMA_CH, PDMA_REQ_SINGLE, 0);

    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA, I2C_PDMA_CH, PDMA_I2C1_TX, FALSE, 0);

    /* Set PDMA Basic Mode */
    PDMA->DSCT[I2C_PDMA_CH].CTL = (PDMA->DSCT[I2C_PDMA_CH].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | PDMA_OP_BASIC;

    g_u32IsTestOver = 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/* I2C Master PDMA Rx Channel Configuration                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_Master_PDMA_Rx_Init(void)
{
    /* Reset PDMA module */
    SYS_ResetModule(PDMA_RST);

    /* Open Channel 1 */
    PDMA_Open(PDMA, 1 << I2C_PDMA_CH);

    /* Transfer count is I2C_PDMA_RX_LENGTH, transfer width is 8 bits(one byte) */
    PDMA_SetTransferCnt(PDMA, I2C_PDMA_CH, PDMA_WIDTH_8, I2C_PDMA_RX_LENGTH);

    /* Set source address is (I2C1->DAT), destination address is g_au8MstRxData, Source/Destination increment size is 32 bits(one word) */
    PDMA_SetTransferAddr(PDMA, I2C_PDMA_CH, (uint32_t)(&(I2C1->DAT)), PDMA_SAR_FIX, (uint32_t)g_au8MstRxData, PDMA_DAR_INC);

    /* Single request type */
    PDMA_SetBurstType(PDMA, I2C_PDMA_CH, PDMA_REQ_SINGLE, 0);

    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA, I2C_PDMA_CH, PDMA_I2C1_RX, FALSE, 0);

    /* Set PDMA Basic Mode */
    PDMA->DSCT[I2C_PDMA_CH].CTL = (PDMA->DSCT[I2C_PDMA_CH].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | PDMA_OP_BASIC;

    g_u32IsTestOver = 0;
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

    /* Enable I2C1 clock */
    CLK_EnableModuleClock(I2C1_MODULE);

    /* Enable PDMA clock */
    CLK_EnableModuleClock(PDMA_MODULE);

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

    /* Set I2C1 multi-function pins */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk)) |
                    (SYS_GPA_MFPL_PA2MFP_I2C1_SDA | SYS_GPA_MFPL_PA3MFP_I2C1_SCL);

    /* Lock protected registers */
    SYS_LockReg();
}

void I2C1_Init(void)
{
    /* Open I2C module and set bus clock */
    I2C_Open(I2C1, 100000);

    /* Get I2C1 Bus Clock */
    printf("I2C1 clock %d Hz\n", I2C_GetBusClockFreq(I2C1));

    /* Set I2C 4 Slave Addresses */
    I2C_SetSlaveAddr(I2C1, 0, 0x15, 0);   /* Slave Address : 0x15 */
    I2C_SetSlaveAddr(I2C1, 1, 0x35, 0);   /* Slave Address : 0x35 */
    I2C_SetSlaveAddr(I2C1, 2, 0x55, 0);   /* Slave Address : 0x55 */
    I2C_SetSlaveAddr(I2C1, 3, 0x75, 0);   /* Slave Address : 0x75 */

    /* Enable I2C interrupt */
    I2C_EnableInt(I2C1);
    NVIC_EnableIRQ(I2C1_IRQn);

    /* Enable I2C1 interrupt */
    I2C_EnableInt(I2C1);
    NVIC_EnableIRQ(I2C1_IRQn);
}

void I2C0_Init(void)
{
    /* Open I2C0 module and set bus clock */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C0 clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

    /* Set I2C0 4 Slave Addresses */
    I2C_SetSlaveAddr(I2C0, 0, 0x16, 0);   /* Slave Address : 0x16 */
    I2C_SetSlaveAddr(I2C0, 1, 0x36, 0);   /* Slave Address : 0x36 */
    I2C_SetSlaveAddr(I2C0, 2, 0x56, 0);   /* Slave Address : 0x56 */
    I2C_SetSlaveAddr(I2C0, 3, 0x76, 0);   /* Slave Address : 0x76 */

    /* Set I2C0 4 Slave Addresses Mask */
    I2C_SetSlaveAddrMask(I2C0, 0, 0x04);
    I2C_SetSlaveAddrMask(I2C0, 1, 0x02);
    I2C_SetSlaveAddrMask(I2C0, 2, 0x04);
    I2C_SetSlaveAddrMask(I2C0, 3, 0x02);

    /* Enable I2C0 interrupt */
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C1_Close(void)
{
    /* Disable I2C1 interrupt and clear corresponding NVIC bit */
    I2C_DisableInt(I2C1);
    NVIC_DisableIRQ(I2C1_IRQn);

    /* Disable I2C1 and close I2C1 clock */
    I2C_Close(I2C1);
    CLK_DisableModuleClock(I2C1_MODULE);
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
    uint32_t i, err = 0;

    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */
    printf("\n");
    printf("+----------------------------------------------------------+\n");
    printf("| I2C Driver Sample Code. Show how a [Master] access Slave |\n");
    printf("| use [PDMA Tx] mode and [PDMA Rx] mode                    |\n");
    printf("| I2C Master (I2C1) <---> I2C Slave(I2C0)                  |\n");
    printf("+----------------------------------------------------------+\n");

    printf("\nConfigure I2C1 as a Master, I2C0 as a Slave.\n");
    printf("The I/O connection I2C1 to I2C0\n");
    printf("I2C0_SDA(PB.4), I2C0_SCL(PB.5)\n");
    printf("I2C1_SDA(PA.2), I2C1_SCL(PA.3)\n\n");

    /* Init I2C0, I2C1 */
    I2C0_Init();
    I2C1_Init();

    /* I2C0 enter no address SLV mode */
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);

    /* Clear Slave receive data buffer */
    for(i = 0; i < I2C_TEST_LENGTH; i++)
    {
        g_au8SlvData[i] = 0;
    }

    /* I2C0 function to Slave receive/transmit data */
    s_I2C0HandlerFn = I2C_SlaveTRx;
    printf("\nI2C0 Slave Mode is Running.\n\n");

    /* Init I2C1 Master PDMA transmit */
    g_u8DeviceAddr = 0x16;

    /* Prepare 100 data bytes */
    g_au8MstTxData[0] = ((g_u8DeviceAddr << 1) | 0x00);   //1 byte SLV + W
    g_au8MstTxData[1] = 0x00;                             //2 bytes Data address
    g_au8MstTxData[2] = 0x00;

    for(i = 3; i < I2C_PDMA_TX_LENGTH; i++)                              //Prepare others are transmit Data from 3 to 99 (97 bytes)
        g_au8MstTxData[i] = i;

    /* Enable interrupt */
    PDMA_EnableInt(PDMA, I2C_PDMA_CH, PDMA_INT_TRANS_DONE);

    /* Enable NVIC for PDMA */
    NVIC_EnableIRQ(PDMA_IRQn);

    /* Init Master PDMA Tx */
    I2C_Master_PDMA_Tx_Init();

    /* Enable I2C1 PDMA TX mode */
    I2C1->CTL1 |= I2C_CTL1_TXPDMAEN_Msk;

    /* Disable PDMA Stretch bus */
    I2C1->CTL1 &= ~I2C_CTL1_PDMASTR_Msk;       //I2C1 will send STOP automatically.

    s_I2C1HandlerFn = (I2C_FUNC)I2C_PDMA_MasterTx;

    printf("Start I2C1 Master transmit data with PDMA...\n");

    /* Send START condition, start the PDMA data transmit */
    I2C_START(I2C1);

    /* Waiting for PDMA transfer done */
    while(g_u32IsTestOver == 0);
    g_u32IsTestOver = 0;

    /* Waiting for I2C bus become free */
    while((I2C1->STATUS1 & I2C_STATUS1_ONBUSY_Msk) == I2C_STATUS1_ONBUSY_Msk);

    /* Disable I2C1 PDMA TX mode */
    I2C1->CTL1 &= ~I2C_CTL1_TXPDMAEN_Msk;
    printf("I2C1 PDMA transmit data pass...\n\n");

    g_u8MstEndFlag = 0;

    /* Clear Master receive data buffer */
    for(i = 1; i < I2C_TEST_LENGTH; i++)
        g_au8MstRxData[i] = 0;

    /* Init Master PDMA Rx */
    I2C_Master_PDMA_Rx_Init();

    /* I2C function to read data from slave */
    s_I2C1HandlerFn = (I2C_FUNC)I2C_PDMA_MasterRx;

    g_u8MstDataLen = 0;
    g_u8MstTxSLA = 0;
    g_u8MstDataLen = 0;

    printf("Start I2C1 Master receive data with PDMA...\n");

    /* Send START condition, start the PDMA data receive */
    I2C_START(I2C1);
    while(g_u8MstTxSLA == 0);

    /* Enable I2C1 PDMA RX after Slave address read ACK */
    I2C1->CTL1 |= I2C_CTL1_RXPDMAEN_Msk;      //Enalbe PDMA RX, Start receive data from Slave

    /* Waiting for PDMA receive done */
    while(g_u32IsTestOver == 0);

    /* Disable I2C1 PDMA RX */
    I2C1->CTL1 &= ~I2C_CTL1_RXPDMAEN_Msk;

    /* PDMA Transfer Done */
    g_u32IsTestOver = 0;

    /* Check Receive data ending */
    while(g_u8MstEndFlag == 0);

    /* Compare I2C1 transmit data and I2C1 receive data */
    for(i = 0; i < I2C_PDMA_RX_LENGTH; i++)
    {
        if(g_au8MstRxData[i] != g_au8MstTxData[i + 3])
        {
            /* Compare fail */
            err = 1;
            printf("[%03d]: Master Tx[0x%X] != Master Rx[0x%X]\n", i,  g_au8MstTxData[i + 3], g_au8MstRxData[i]);
        }
    }

    if(err)
        printf("I2C1 PDMA receive data fail...\n");
    else
        printf("I2C1 PDMA Tx/Rx data pass...\n");

    /* Disable PDMA channel */
    PDMA_Close(PDMA);

    /* Disable PDMA Interrupt */
    PDMA_DisableInt(PDMA, I2C_PDMA_CH, PDMA_INT_TRANS_DONE);
    NVIC_DisableIRQ(PDMA_IRQn);

    s_I2C0HandlerFn = NULL;
    s_I2C1HandlerFn = NULL;

    /* Close I2C0,I2C1 */
    I2C0_Close();
    I2C1_Close();

    while(1);

}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

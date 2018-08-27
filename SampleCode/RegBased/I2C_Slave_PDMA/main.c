/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 7 $
 * $Date: 18/07/24 11:35a $
 * @brief
 *           Demonstrate how a slave uses PDMA RX mode to receive data from a master (Loopback).
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define I2C_PDMA_CH        1
#define I2C_PDMA_RX_LENGTH 100
#define I2C_TEST_LENGTH    256

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t slave_buff_addr;
volatile uint8_t g_au8RxData[3];
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_au8TxData[3];
volatile uint8_t g_u8RxData;
volatile uint8_t g_u8DataLen;
volatile uint8_t g_u8EndFlag = 0;
volatile uint8_t g_u8DataLen1;
volatile uint8_t g_u8SlvTestLen;
volatile uint8_t g_u8SlvDataLen;
volatile uint8_t g_au8SlvData[I2C_TEST_LENGTH];
volatile int32_t g_u32IsTestOver;
volatile uint8_t g_au8MstTxData[I2C_TEST_LENGTH];
volatile uint8_t g_u8MstDataLen;
volatile uint8_t g_u8MstEndFlag = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);
static I2C_FUNC s_I2C0HandlerFn = NULL;
static I2C_FUNC s_I2C1HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/* PDMA IRQ Handler                                                                                        */
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
        I2C0->TOCTL |= I2C_TOCTL_TOIF_Msk;
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
        I2C1->TOCTL |= I2C_TOCTL_TOIF_Msk;
    }
    else
    {
        if(s_I2C1HandlerFn != NULL)
            s_I2C1HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 Master Tx Callback Function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterTx(uint32_t u32Status)
{
    if(u32Status == 0x08)                           /* START has been transmitted */
    {
        I2C_SET_DATA(I2C0, g_u8DeviceAddr << 1);    /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        g_u8MstDataLen++;
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8MstDataLen != I2C_PDMA_RX_LENGTH)
        {
            I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_STOP(I2C0);
            g_u8MstEndFlag = 1;
        }
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 Slave TRx Callback Function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_PDMA_SlaveRx(uint32_t u32Status)
{
    if(u32Status == 0x60)                       /* Own SLA+W has been receive; ACK has been return */
    {
        /*
            Note:
            During PDMA operation, I2C controller will not occur receive Address ACK interrupt
        */
    }
    else if(u32Status == 0x80)                 /* Previously address with own SLA address
                                                  Data has been received; ACK has been returned*/
    {
        /*
            Note:
            During PDMA operation, I2C controller will not occur receive Data ACK interrupt
        */
    }
    else if(u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
        while(1);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* I2C Slave PDMA Rx Channel Configuration                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_Slave_PDMA_Rx_Init(void)
{
    /* PDMA Reset */
    SYS->IPRST0 |= (1 << SYS_IPRST0_PDMARST_Msk);
    SYS->IPRST0 &= ~SYS_IPRST0_PDMARST_Msk;

    /* Initial PDMA and set PDMA ch1 */
    PDMA->CHCTL |= (1 << I2C_PDMA_CH);

    /* PDMA Setting */
    PDMA->DSCT[I2C_PDMA_CH].CTL =
        PDMA_OP_BASIC |         // Basic Mode
        PDMA_REQ_SINGLE |       // Single type transfer
        PDMA_SAR_FIX |          // Source address -> fixed(I2C1_DAT)
        PDMA_DAR_INC |          // Destination address -> incremented(g_au8SlvData[n])
        PDMA_WIDTH_8 |          // Transfer width -> 8-bit
        ((I2C_PDMA_RX_LENGTH - 1) << 16); // transfer count
    PDMA->DSCT[I2C_PDMA_CH].SA = (uint32_t)(&(I2C1->DAT));
    PDMA->DSCT[I2C_PDMA_CH].DA = (uint32_t)g_au8SlvData;

    /* Enalbe PDMA I2C1 RX mode */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC1_Msk) | (PDMA_I2C1_RX << PDMA_REQSEL0_3_REQSRC1_Pos);

    /* Enable PDMA Channel 1 INT */
    PDMA->INTEN |= (1 << I2C_PDMA_CH);
    NVIC_EnableIRQ(PDMA_IRQn);
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
    CLK->APBCLK0 |= (CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_I2C0CKEN_Msk | CLK_APBCLK0_I2C1CKEN_Msk);

    /* PDMA Clock Enable */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;

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

    /* Set I2C1 multi-function pins */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk)) |
                    (SYS_GPA_MFPL_PA2MFP_I2C1_SDA | SYS_GPA_MFPL_PA3MFP_I2C1_SCL);

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
    printf("I2C0 clock %d Hz\n", (SystemCoreClock / (((I2C0->CLKDIV) + 1) << 2)));

    /* Set I2C0 4 Slave Addresses */
    /* Slave Address : 0x15 */
    I2C0->ADDR0 = (I2C0->ADDR0 & ~I2C_ADDR0_ADDR_Msk) | (0x15 << I2C_ADDR0_ADDR_Pos);
    /* Slave Address : 0x35 */
    I2C0->ADDR1 = (I2C0->ADDR1 & ~I2C_ADDR1_ADDR_Msk) | (0x35 << I2C_ADDR1_ADDR_Pos);
    /* Slave Address : 0x55 */
    I2C0->ADDR2 = (I2C0->ADDR2 & ~I2C_ADDR2_ADDR_Msk) | (0x55 << I2C_ADDR2_ADDR_Pos);
    /* Slave Address : 0x75 */
    I2C0->ADDR3 = (I2C0->ADDR3 & ~I2C_ADDR3_ADDR_Msk) | (0x75 << I2C_ADDR3_ADDR_Pos);

    /* Set I2C0 4 Slave Addresses Mask Bits*/
    /* Slave Address Mask Bits: 0x01 */
    I2C0->ADDRMSK0 = (I2C0->ADDRMSK0 & ~I2C_ADDRMSK0_ADDRMSK_Msk) | (0x01 << I2C_ADDRMSK0_ADDRMSK_Pos);
    /* Slave Address Mask Bits: 0x04 */
    I2C0->ADDRMSK1 = (I2C0->ADDRMSK1 & ~I2C_ADDRMSK1_ADDRMSK_Msk) | (0x04 << I2C_ADDRMSK1_ADDRMSK_Pos);
    /* Slave Address Mask Bits: 0x01 */
    I2C0->ADDRMSK2 = (I2C0->ADDRMSK2 & ~I2C_ADDRMSK2_ADDRMSK_Msk) | (0x01 << I2C_ADDRMSK2_ADDRMSK_Pos);
    /* Slave Address Mask Bits: 0x04 */
    I2C0->ADDRMSK3 = (I2C0->ADDRMSK3 & ~I2C_ADDRMSK3_ADDRMSK_Msk) | (0x04 << I2C_ADDRMSK3_ADDRMSK_Pos);

    /* Enable I2C0 interrupt and set corresponding NVIC bit */
    I2C0->CTL0 |= I2C_CTL0_INTEN_Msk;
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C1_Init(void)
{
    uint32_t u32BusClock;

    /* Reset I2C1 */
    SYS->IPRST1 |=  SYS_IPRST1_I2C1RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_I2C1RST_Msk;

    /* Enable I2C1 Controller */
    I2C1->CTL0 |= I2C_CTL0_I2CEN_Msk;

    /* I2C1 bus clock 100K divider setting, I2CLK = PCLK/(100K*4)-1 */
    u32BusClock = 100000;
    I2C1->CLKDIV = (uint32_t)(((SystemCoreClock * 10) / (u32BusClock * 4) + 5) / 10 - 1); /* Compute proper divider for I2C clock */

    /* Get I2C1 Bus Clock */
    printf("I2C1 clock %d Hz\n", (SystemCoreClock / (((I2C1->CLKDIV) + 1) << 2)));

    /* Set I2C1 4 Slave Addresses */
    /* Slave Address : 0x16 */
    I2C1->ADDR0 = (I2C1->ADDR0 & ~I2C_ADDR0_ADDR_Msk) | (0x16 << I2C_ADDR0_ADDR_Pos);
    /* Slave Address : 0x36 */
    I2C1->ADDR1 = (I2C1->ADDR1 & ~I2C_ADDR1_ADDR_Msk) | (0x36 << I2C_ADDR1_ADDR_Pos);
    /* Slave Address : 0x56 */
    I2C1->ADDR2 = (I2C1->ADDR2 & ~I2C_ADDR2_ADDR_Msk) | (0x56 << I2C_ADDR2_ADDR_Pos);
    /* Slave Address : 0x76 */
    I2C1->ADDR3 = (I2C1->ADDR3 & ~I2C_ADDR3_ADDR_Msk) | (0x76 << I2C_ADDR3_ADDR_Pos);

    /* Set I2C1 4 Slave Addresses Mask Bits*/
    /* Slave Address Mask Bits: 0x04 */
    I2C1->ADDRMSK0 = (I2C1->ADDRMSK0 & ~I2C_ADDRMSK0_ADDRMSK_Msk) | (0x04 << I2C_ADDRMSK0_ADDRMSK_Pos);
    /* Slave Address Mask Bits: 0x02 */
    I2C1->ADDRMSK1 = (I2C1->ADDRMSK1 & ~I2C_ADDRMSK1_ADDRMSK_Msk) | (0x02 << I2C_ADDRMSK1_ADDRMSK_Pos);
    /* Slave Address Mask Bits: 0x04 */
    I2C1->ADDRMSK2 = (I2C1->ADDRMSK2 & ~I2C_ADDRMSK2_ADDRMSK_Msk) | (0x04 << I2C_ADDRMSK2_ADDRMSK_Pos);
    /* Slave Address Mask Bits: 0x02 */
    I2C1->ADDRMSK3 = (I2C1->ADDRMSK3 & ~I2C_ADDRMSK3_ADDRMSK_Msk) | (0x02 << I2C_ADDRMSK3_ADDRMSK_Pos);

    /* Enable I2C1 interrupt and set corresponding NVIC bit */
    I2C1->CTL0 |= I2C_CTL0_INTEN_Msk;
    NVIC_EnableIRQ(I2C1_IRQn);
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

void I2C1_Close(void)
{
    /* Disable I2C1 interrupt and clear corresponding NVIC bit */
    I2C1->CTL0 &= ~I2C_CTL0_INTEN_Msk;
    NVIC_DisableIRQ(I2C1_IRQn);

    /* Disable I2C1 and close I2C1 clock */
    I2C1->CTL0 &= ~I2C_CTL0_I2CEN_Msk;
    CLK->APBCLK0 &= ~CLK_APBCLK0_I2C1CKEN_Msk;
}

void I2C_Write_to_SLAVE_PDMA_RX(uint8_t slvaddr)
{
    uint32_t i;


    g_au8MstTxData[0] = ((slvaddr << 1) | 0x00);
    g_u8DeviceAddr = slvaddr;

    /* I2C0 prepare transmit data bytes */
    for(i = 1; i < I2C_TEST_LENGTH; i++)
    {
        g_au8MstTxData[i] = i;
    }

    g_u8MstDataLen = 0;
    g_u8MstEndFlag = 0;

    /* I2C0 function to write data to slave */
    s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx;

    /* I2C0 as master sends START signal */
    I2C_START(I2C0);

    /* Wait I2C0 Tx Finish */
    while(g_u8MstEndFlag == 0);
    g_u8MstEndFlag = 0;

    /* Waiting for I2C0 bus become free */
    while((I2C0->STATUS1 & I2C_STATUS1_ONBUSY_Msk) ==  I2C_STATUS1_ONBUSY_Msk);
}

int main()
{
    uint32_t i;
    uint8_t  err;

    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART0_Init();

    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("+----------------------------------------------------------+\n");
    printf("| I2C Driver Sample Code. Show how a Master access [Slave] |\n");
    printf("| use [PDMA Rx] mode                                       |\n");
    printf("| I2C Master (I2C0) <---> I2C Slave(I2C1)                  |\n");
    printf("+----------------------------------------------------------+\n");

    printf("\n");
    printf("Configure I2C0 as Master, and I2C1 as a Slave.\n");
    printf("The I/O connection I2C0 to I2C1:\n");
    printf("I2C0_SDA(PB.4), I2C0_SCL(PB.5)\n");
    printf("I2C1_SDA(PA.2), I2C1_SCL(PA.3)\n\n");

    /* Init I2C0 */
    I2C0_Init();

    /* Init I2C1 */
    I2C1_Init();

    /* I2C1 enter no address SLV mode */
    I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);

    /* Clear Slave receive data buffer */
    for(i = 0; i < I2C_TEST_LENGTH; i++)
    {
        g_au8SlvData[i] = 0;
    }

    I2C_Slave_PDMA_Rx_Init();

    /* Enable I2C1 PDMA RX mode */
    I2C1->CTL1 |= I2C_CTL1_RXPDMAEN_Msk;

    /* I2C1 function to Slave receive/transmit data */
    s_I2C1HandlerFn = I2C_PDMA_SlaveRx;
    printf("\nI2C Slave Mode is Running.\n\n");

    /* I2C0 access I2C1*/
    I2C_Write_to_SLAVE_PDMA_RX(0x16);

    /* Waiting for PDMA channel 1 transfer done*/
    while(g_u32IsTestOver == 0);
    g_u32IsTestOver = 0;

    err = 0;
    for(i = 0; i < I2C_PDMA_RX_LENGTH; i++)
    {
        /* Compare Master Tx data with Slave Rx data */
        if(g_au8MstTxData[i] != g_au8SlvData[i])
        {
            err = 1;
            printf("[%03d]: Tx[0x%X] != Rx[0x%X]\n", i, g_au8MstTxData[i], g_au8SlvData[i]);
        }
    }

    if(err)
        printf("Master write data to Slave(PDMA RX) fail...\n");
    else
        printf("Master write data to Slave(PDMA RX) pass...\n");


    s_I2C0HandlerFn = NULL;
    s_I2C1HandlerFn = NULL;

    /* Close I2C0,I2C1 */
    I2C0_Close();
    I2C1_Close();

    while(1);
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

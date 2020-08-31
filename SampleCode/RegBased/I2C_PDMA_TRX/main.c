/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief
 *           Demonstrate I2C PDMA mode and need to connect I2C0 (master) and I2C1 (slave).
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define I2C0_PDMA_TX_CH      0
#define I2C1_PDMA_RX_CH      1
#define I2C0_PDMA_RX_CH      2
#define I2C1_PDMA_TX_CH      3
#define PDMA_TEST_LENGTH    16

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static uint8_t g_u8MasterTx_Buffer[PDMA_TEST_LENGTH];
static uint8_t g_u8MasterRx_Buffer[PDMA_TEST_LENGTH];
static uint8_t g_u8SlaveTx_Buffer[PDMA_TEST_LENGTH];
static uint8_t g_u8SlaveRx_Buffer[PDMA_TEST_LENGTH];

volatile uint32_t PDMA_DONE = 0;
volatile uint8_t g_u8DeviceAddr = 0x16;
volatile uint8_t g_u8EndFlag = 0;
volatile uint8_t g_u8MasterDataLen = 0;
volatile uint8_t g_u8SlaveDataLen = 0;
volatile uint16_t g_u8SlaveBufferAddr = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);

volatile static I2C_FUNC s_I2C0HandlerFn = NULL;
volatile static I2C_FUNC s_I2C1HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  PDMA IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_IRQHandler(void)
{
    uint32_t u32Status = PDMA->TDSTS;

    //Master TX
    if (u32Status & (0x1 << I2C0_PDMA_TX_CH))
    {
        printf("\n I2C0 Tx done  ");
        PDMA->TDSTS = 0x1 << I2C0_PDMA_TX_CH;
    }

    //Master RX
    if (u32Status & (0x1 << I2C0_PDMA_RX_CH))
    {
        printf("\n I2C0 Rx done  ");
        PDMA->TDSTS = 0x1 << I2C0_PDMA_RX_CH;
        PDMA_DONE = 1;
    }

    //Slave RX
    if (u32Status & (0x1 << I2C1_PDMA_RX_CH))
    {
        printf("\n I2C1 Rx done  ");
        PDMA->TDSTS = 0x1 << I2C1_PDMA_RX_CH;
        PDMA_DONE = 1;
    }

    //Slave TX
    if (u32Status & (0x1 << I2C1_PDMA_TX_CH))
    {
        printf("\n I2C1 Tx done  ");
        PDMA->TDSTS = 0x1 << I2C1_PDMA_TX_CH;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);

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
/*  I2C1 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C1_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C1);

    if (I2C_GET_TIMEOUT_FLAG(I2C1))
    {
        /* Clear I2C1 Timeout Flag */
        I2C1->TOCTL |= I2C_TOCTL_TOIF_Msk;
    }
    else
    {
        if (s_I2C1HandlerFn != NULL)
            s_I2C1HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 PDMA Slave Rx Callback Function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_PDMA_SlaveRx(uint32_t u32Status)
{
    if (u32Status == 0x60)                      /* Own SLA+W has been receive; ACK has been return */
    {
        /*
            Note:
            During PDMA operation, I2C controller will not occur receive Address ACK interrupt
        */
    }
    else if (u32Status == 0x80)                 /* Previously address with own SLA address
                                                  Data has been received; ACK has been returned*/
    {
        /*
            Note:
            During PDMA operation, I2C controller will not occur receive Data ACK interrupt
        */
    }
    else if (u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
        while (1);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 PDMA Slave Tx Callback Function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_PDMA_SlaveTx(uint32_t u32Status)
{
    uint8_t u8data;

    if (u32Status == 0x60)                       /* Own SLA+W has been receive; ACK has been return */
    {
        g_u8SlaveDataLen = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        u8data = (unsigned char) I2C_GET_DATA(I2C1);
        g_u8SlaveRx_Buffer[g_u8SlaveDataLen++] = u8data;
        g_u8SlaveBufferAddr = (g_u8SlaveRx_Buffer[0] << 8) + g_u8SlaveRx_Buffer[1];

        if(g_u8SlaveDataLen == 2)
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
        }
    }
    else if (u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);

        /* Enable I2C1 Slave TX */
        I2C1->CTL1 = I2C_CTL1_TXPDMAEN_Msk;
    }
    else if (u32Status == 0xA8)                  /* Own SLA+R has been receive; ACK has been return */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur START interrupt
        */
    }
    else if (u32Status == 0xB8)                  /* Data byte in I2CDAT has been transmitted ACK has been received */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur START interrupt
        */
    }
    else if (u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);

        while (1);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 PDMA Master Tx Callback Function                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_PDMA_MasterTx(uint32_t u32Status)
{
    if (u32Status == 0x08)                      /* START has been transmitted */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur START interrupt
        */
    }
    else if (u32Status == 0x10)                 /* Repeat START has been transmitted */
    {

    }
    else if (u32Status == 0x18)                 /* SLA+W has been transmitted and ACK has been received */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur address ACK interrupt
        */
    }
    else if (u32Status == 0x20)                 /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
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
        I2C_SET_DATA(I2C0, (g_u8DeviceAddr << 1) | 0x00);     /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x18)                     /* SLA+W has been transmitted and ACK has been received */
    {
        g_u8MasterDataLen = 1;
        I2C_SET_DATA(I2C0, g_u8MasterTx_Buffer[g_u8MasterDataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                     /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x28)                     /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8MasterDataLen <= 2)
        {
            I2C_SET_DATA(I2C0, g_u8MasterTx_Buffer[g_u8MasterDataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA_SI);
        }
    }
    else if(u32Status == 0x10)                    /* Repeat START has been transmitted and prepare SLA+R */
    {
        I2C_SET_DATA(I2C0, (g_u8DeviceAddr << 1) | 0x01);   /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x40)                    /* SLA+R has been transmitted and ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);

        /* Enable I2C0 Master RX */
        I2C0->CTL1 = I2C_CTL1_RXPDMAEN_Msk;
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
        /* Clear SI and send STOP signal */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO | I2C_CTL_SI);
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
    CLK->APBCLK0 |= (CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_I2C0CKEN_Msk | CLK_APBCLK0_I2C1CKEN_Msk);

    /* Switch UART0 clock source to HIRC and UART0 clock divider as 1 */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART0SEL_Msk)) | CLK_CLKSEL1_UART0SEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UART0DIV_Msk)) | CLK_CLKDIV0_UART0(1);

    /* Enable PDMA clock */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;

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
    /* Enable I2C0 Controller */
    I2C0->CTL0 |= I2C_CTL0_I2CEN_Msk;

    /* I2C0 clock divider, I2C Bus Clock = PCLK(48Mhz) / (4*120) = 100kHz */
    I2C0->CLKDIV = 120 - 1;

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

    /* Enable I2C0 interrupt and set corresponding NVIC bit */
    I2C0->CTL0 |= I2C_CTL0_INTEN_Msk;
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C1_Init(void)
{
    /* Enable I2C1 Controller */
    I2C1->CTL0 |= I2C_CTL0_I2CEN_Msk;

    /* I2C1 clock divider, I2C Bus Clock = PCLK(48Mhz) / (4*120) = 100kHz */
    I2C1->CLKDIV = 120 - 1;

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

    /* Enable I2C1 interrupt and set corresponding NVIC bit */
    I2C1->CTL0 |= I2C_CTL0_INTEN_Msk;
    NVIC_EnableIRQ(I2C1_IRQn);
}

void PDMA_Init(void)
{
    /* Initial PDMA Channel */
    PDMA->CHCTL |= (1 << I2C0_PDMA_TX_CH); // Channel 0 for I2C0 TX
    PDMA->CHCTL |= (1 << I2C1_PDMA_RX_CH); // Channel 1 for I2C1 RX
    PDMA->CHCTL |= (1 << I2C0_PDMA_RX_CH); // Channel 2 for I2C0 RX
    PDMA->CHCTL |= (1 << I2C1_PDMA_TX_CH); // Channel 3 for I2C1 TX

    /* PDMA Settings */
    PDMA->DSCT[I2C0_PDMA_TX_CH].CTL =
        PDMA_OP_BASIC |            /* Basic Mode */
        PDMA_REQ_SINGLE |          /* single transfer type */
        PDMA_SAR_INC |             /* Source address -> incremented(g_u8MasterTx_Buffer[n]) */
        PDMA_DAR_FIX |             /* Destination address -> fixed(I2C0_DAT) */
        PDMA_WIDTH_8 |             /* transfer width -> 8-bit */
        ((PDMA_TEST_LENGTH - 1) << 16); /* transfer count */
    PDMA->DSCT[I2C0_PDMA_TX_CH].SA = (uint32_t)(&g_u8MasterTx_Buffer[0]);
    PDMA->DSCT[I2C0_PDMA_TX_CH].DA = (uint32_t)(&(I2C0->DAT));

    PDMA->DSCT[I2C1_PDMA_RX_CH].CTL =
        PDMA_OP_BASIC |            /* Basic mode */
        PDMA_REQ_SINGLE |          /* Single transfer type */
        PDMA_SAR_FIX  |            /* source address -> Fix(I2C1_DAT) */
        PDMA_DAR_INC  |            /* destination address -> incremented(g_u8SlaveRx_Buffer[n]) */
        PDMA_WIDTH_8 |             /* transfer width -> 8-bit */
        ((PDMA_TEST_LENGTH - 1) << 16); /* transfer count */
    PDMA->DSCT[I2C1_PDMA_RX_CH].SA = (uint32_t)(&(I2C1->DAT));
    PDMA->DSCT[I2C1_PDMA_RX_CH].DA = (uint32_t)(&g_u8SlaveRx_Buffer[0]);

    PDMA->DSCT[I2C0_PDMA_RX_CH].CTL =
        PDMA_OP_BASIC |            /* Basic mode */
        PDMA_REQ_SINGLE |          /* Single transfer type */
        PDMA_SAR_FIX  |            /* source address -> Fix(I2C0_DAT) */
        PDMA_DAR_INC  |            /* destination address -> incremented(g_u8MasterRx_Buffer[n]) */
        PDMA_WIDTH_8 |             /* transfer width -> 8-bit */
        ((PDMA_TEST_LENGTH - 1 - 3) << 16); /* transfer count, except Slave Address and two bytes Data Address */
    PDMA->DSCT[I2C0_PDMA_RX_CH].SA = (uint32_t)(&(I2C0->DAT));
    PDMA->DSCT[I2C0_PDMA_RX_CH].DA = (uint32_t)(&g_u8MasterRx_Buffer[0]);

    PDMA->DSCT[I2C1_PDMA_TX_CH].CTL =
        PDMA_OP_BASIC |            /* Basic Mode */
        PDMA_REQ_SINGLE |          /* single transfer type */
        PDMA_SAR_INC |             /* Source address -> incremented(g_u8SlaveTx_Buffer[n]) */
        PDMA_DAR_FIX |             /* Destination address -> fixed(I2C1_DAT) */
        PDMA_WIDTH_8 |             /* transfer width -> 8-bit */
        ((PDMA_TEST_LENGTH - 1 - 3) << 16); /* transfer count, except Slave Address and two bytes Data Address */
    PDMA->DSCT[I2C1_PDMA_TX_CH].SA = (uint32_t)(&g_u8SlaveTx_Buffer[0]);
    PDMA->DSCT[I2C1_PDMA_TX_CH].DA = (uint32_t)(&(I2C1->DAT));

    /* Enalbe PDMA I2C0 TX mode */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC0_Msk) | (PDMA_I2C0_TX << PDMA_REQSEL0_3_REQSRC0_Pos);
    /* Enalbe PDMA I2C1 RX mode */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC1_Msk) | (PDMA_I2C1_RX << PDMA_REQSEL0_3_REQSRC1_Pos);
    /* Enalbe PDMA I2C0 RX mode */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC2_Msk) | (PDMA_I2C0_RX << PDMA_REQSEL0_3_REQSRC2_Pos);
    /* Enalbe PDMA I2C1 TX mode */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC3_Msk) | (PDMA_I2C1_TX << PDMA_REQSEL0_3_REQSRC3_Pos);

    /* Enable PDMA Channel 0 INT */
    PDMA->INTEN |= (1 << I2C0_PDMA_TX_CH);
    /* Enable PDMA Channel 1 INT */
    PDMA->INTEN |= (1 << I2C1_PDMA_RX_CH);
    /* Enable PDMA Channel 2 INT */
    PDMA->INTEN |= (1 << I2C0_PDMA_RX_CH);
    /* Enable PDMA Channel 3 INT */
    PDMA->INTEN |= (1 << I2C1_PDMA_TX_CH);
    NVIC_EnableIRQ(PDMA_IRQn);
}

void I2C_PDMA(void)
{
    uint32_t i;

    for (i = 0; i < PDMA_TEST_LENGTH; i++)
    {
        g_u8MasterTx_Buffer[i] = i;
        g_u8SlaveRx_Buffer[i] = 0xff;
    }

    g_u8MasterTx_Buffer[0] = ((g_u8DeviceAddr << 1) | 0x00);   //1 byte SLV + W
    g_u8MasterTx_Buffer[1] = 0x00;                             //2 bytes Data address
    g_u8MasterTx_Buffer[2] = 0x00;

    PDMA_Init();

    /* I2C1 enter no address SLV mode */
    I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI | I2C_CTL_AA);
    /* Enable I2C1 PDMA RX mode */
    I2C1->CTL1 = I2C_CTL1_RXPDMAEN_Msk;

    /* I2C1 function to Slave receive data */
    s_I2C1HandlerFn = I2C_PDMA_SlaveRx;

    PDMA_DONE = 0;

    /* Enable I2C0 TX */
    I2C0->CTL1 = I2C_CTL1_TXPDMAEN_Msk;

    s_I2C0HandlerFn = (I2C_FUNC)I2C_PDMA_MasterTx;

    /* Send START condition, start the PDMA data transmit */
    I2C_START(I2C0);

    while (!PDMA_DONE);

    /* Disable I2C0 PDMA TX mode */
    I2C0->CTL1 &= ~I2C_CTL1_TXPDMAEN_Msk;

    /* Disable I2C1 PDMA RX mode */
    I2C1->CTL1 &= ~I2C_CTL1_RXPDMAEN_Msk;

    for (i = 0; i < PDMA_TEST_LENGTH; i++)
    {
        if (g_u8SlaveRx_Buffer[i] != g_u8MasterTx_Buffer[i])
        {
            printf("\n Slave Receive Data Compare Error !!");

            while (1);
        }
        else
        {
            if(i > 2)
                g_u8SlaveTx_Buffer[i-3] = g_u8MasterTx_Buffer[i];
        }
    }

    /* Test Master RX and Slave TX with PDMA function */

    /* I2C0 function to Master receive data */
    s_I2C0HandlerFn = (I2C_FUNC)I2C_PDMA_MasterRx;

    /* I2C1 function to Slave transmit data */
    s_I2C1HandlerFn = I2C_PDMA_SlaveTx;

    PDMA_DONE = 0;

    /* Send START condition */
    I2C_START(I2C0);

    while (!PDMA_DONE);

    /* Disable I2C0 PDMA RX mode */
    I2C0->CTL1 &= ~I2C_CTL1_RXPDMAEN_Msk;

    /* Disable I2C1 PDMA TX mode */
    I2C1->CTL1 &= ~I2C_CTL1_TXPDMAEN_Msk;

    for (i = 0; i < PDMA_TEST_LENGTH - 3; i++)
    {
        if (g_u8MasterRx_Buffer[i] != g_u8MasterTx_Buffer[i+3])
        {
            printf("\n Slave Receive Data Compare Error !!");

            while (1);
        }
    }

    printf("\nI2C PDMA test Pass.\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART0_Init();

    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("+-------------------------------------------------------+\n");
    printf("| I2C Driver Sample Code for PDMA                       |\n");
    printf("|                                                       |\n");
    printf("| I2C Master (I2C0) <---> I2C Slave(I2C1)               |\n");
    printf("+-------------------------------------------------------+\n");

    printf("\n");
    printf("Configure I2C0 as Master, and I2C1 as a slave.\n");
    printf("The I/O connection I2C0 to I2C1:\n");
    printf("I2C0_SDA(PB.4), I2C0_SCL(PB.5)\n");
    printf("I2C1_SDA(PA.2), I2C1_SCL(PA.3)\n\n");

    /* Init I2C0 */
    I2C0_Init();

    /* Init I2C1 */
    I2C1_Init();

    I2C_PDMA();

    while (1);
}

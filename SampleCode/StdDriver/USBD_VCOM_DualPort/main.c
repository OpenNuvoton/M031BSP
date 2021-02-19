/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 13 $
 * $Date: 18/07/26 12:17p $
 * @brief    Demonstrate how to implement a USB dual virtual com port device.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"
#include "vcom_serial.h"

#define CRYSTAL_LESS        1    /* CRYSTAL_LESS must be 1 if USB clock source is HIRC */
#define TRIM_INIT           (SYS_BASE+0x118)


int IsDebugFifoEmpty(void);
extern uint8_t volatile g_u8Suspend;

/*--------------------------------------------------------------------------*/
STR_VCOM_LINE_CODING gLineCoding0 = {115200, 0, 0, 8};   /* Baud rate : 115200    */
STR_VCOM_LINE_CODING gLineCoding1 = {115200, 0, 0, 8};   /* Baud rate : 115200    */
/* Stop bit     */
/* parity       */
/* data bits    */
uint16_t gCtrlSignal0 = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */
uint16_t gCtrlSignal1 = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

/*--------------------------------------------------------------------------*/
#define RX_BUFSIZE           512 /* RX buffer size */
#define TX_BUFSIZE           512 /* RX buffer size */

#define TX_FIFO_SIZE_0       16  /* TX Hardware FIFO size */
#define TX_FIFO_SIZE_1       16  /* TX Hardware FIFO size */

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
/* UART0 */
volatile uint8_t comRbuf0[RX_BUFSIZE];
volatile uint16_t comRbytes0 = 0;
volatile uint16_t comRhead0 = 0;
volatile uint16_t comRtail0 = 0;

volatile uint8_t comTbuf0[TX_BUFSIZE];
volatile uint16_t comTbytes0 = 0;
volatile uint16_t comThead0 = 0;
volatile uint16_t comTtail0 = 0;

uint8_t gRxBuf0[64] = {0};
uint8_t *gpu8RxBuf0 = 0;
uint32_t gu32RxSize0 = 0;
uint32_t gu32TxSize0 = 0;

volatile int8_t gi8BulkOutReady0 = 0;
extern uint8_t volatile g_u8Suspend;

/* UART1 */
volatile uint8_t comRbuf1[RX_BUFSIZE];
volatile uint16_t comRbytes1 = 0;
volatile uint16_t comRhead1 = 0;
volatile uint16_t comRtail1 = 0;

volatile uint8_t comTbuf1[TX_BUFSIZE];
volatile uint16_t comTbytes1 = 0;
volatile uint16_t comThead1 = 0;
volatile uint16_t comTtail1 = 0;

uint8_t gRxBuf1[64] = {0};
uint8_t *gpu8RxBuf1 = 0;
uint32_t gu32RxSize1 = 0;
uint32_t gu32TxSize1 = 0;

volatile int8_t gi8BulkOutReady1 = 0;

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Switch UART1 clock source to HIRC */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));

    /* Enable UART1 clock */
    CLK_EnableModuleClock(UART1_MODULE);

    /* Switch USB clock source to HIRC & USB Clock = HIRC / 1 */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL0_USBDSEL_HIRC, CLK_CLKDIV0_USB(1));

    /* Enable USB clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))
                    |(SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    UART_ENABLE_INT(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));

    /* Set PB multi-function pins for UART1 RXD=PB.2 and TXD=PB.3 */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk))
                    |(SYS_GPB_MFPL_PB2MFP_UART1_RXD | SYS_GPB_MFPL_PB3MFP_UART1_TXD);

    UART_ENABLE_INT(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));

    /* Lock protected registers */
    SYS_LockReg();
}


void PowerDown()
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    printf("Enter power down ...\n");
    while(!IsDebugFifoEmpty());

    /* Wakeup Enable */
    USBD_ENABLE_INT(USBD_INTEN_WKEN_Msk);

    CLK_PowerDown();

    /* Clear PWR_DOWN_EN if it is not clear by itself */
    if(CLK->PWRCTL & CLK_PWRCTL_PDEN_Msk)
        CLK->PWRCTL ^= CLK_PWRCTL_PDEN_Msk;

    printf("device wakeup!\n");

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);

    /* Enable UART0 RX Time-Out Interrupt and RX Data Available Interrupt */
    UART_EnableInt(UART0, UART_INTEN_RXTOIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RDAIEN_Msk);
}

void UART1_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART1_RST);

    /* Configure UART1 and set UART1 Baudrate */
    UART_Open(UART1, 115200);

    /* Enable UART1 RX Time-Out Interrupt and RX Data Available Interrupt */
    UART_EnableInt(UART1, UART_INTEN_RXTOIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RDAIEN_Msk);
}


/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART02_IRQHandler(void)
{
    uint32_t u32IntStatus;
    uint8_t bInChar;
    int32_t size;

    u32IntStatus = UART0->INTSTS;

    if((u32IntStatus & UART_INTSTS_RDAIF_Msk) || (u32IntStatus & UART_INTSTS_RXTOIF_Msk))
    {
        /* Receiver FIFO threshold level is reached or Rx time out */

        /* Get all the input characters */
        while (UART_GET_RX_EMPTY(UART0) == 0)
        {
            /* Get the character from UART Buffer */
            bInChar = UART0->DAT;

            /* Check if buffer full */
            if(comRbytes0 < RX_BUFSIZE)
            {
                /* Enqueue the character */
                comRbuf0[comRtail0++] = bInChar;
                if(comRtail0 >= RX_BUFSIZE)
                    comRtail0 = 0;
                comRbytes0++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if(u32IntStatus & UART_INTSTS_THREIF_Msk)
    {
        if(comTbytes0 && (UART0->INTEN & UART_INTEN_THREIEN_Msk))
        {
            /* Fill the Tx FIFO */
            size = comTbytes0;
            if(size >= TX_FIFO_SIZE_0)
            {
                size = TX_FIFO_SIZE_0;
            }

            while(size)
            {
                bInChar = comTbuf0[comThead0++];
                UART0->DAT = bInChar;
                if(comThead0 >= TX_BUFSIZE)
                    comThead0 = 0;
                comTbytes0--;
                size--;
            }
        }
        else
        {
            /* No more data, just stop Tx (Stop work) */
            UART_DISABLE_INT(UART0, UART_INTEN_THREIEN_Msk);
        }
    }
}

void UART13_IRQHandler(void)
{
    uint32_t u32IntStatus;
    uint8_t bInChar;
    int32_t size;

    u32IntStatus = UART1->INTSTS;

    if((u32IntStatus & UART_INTSTS_RDAIF_Msk) || (u32IntStatus & UART_INTSTS_RXTOIF_Msk))
    {
        /* Receiver FIFO threshold level is reached or Rx time out */

        /* Get all the input characters */
        while (UART_GET_RX_EMPTY(UART1) == 0)
        {
            /* Get the character from UART Buffer */
            bInChar = UART1->DAT;

            /* Check if buffer full */
            if(comRbytes1 < RX_BUFSIZE)
            {
                /* Enqueue the character */
                comRbuf1[comRtail1++] = bInChar;
                if(comRtail1 >= RX_BUFSIZE)
                    comRtail1 = 0;
                comRbytes1++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if(u32IntStatus & UART_INTSTS_THREIF_Msk)
    {
        if(comTbytes1 && (UART1->INTEN & UART_INTEN_THREIEN_Msk))
        {
            /* Fill the Tx FIFO */
            size = comTbytes1;
            if(size >= TX_FIFO_SIZE_1)
            {
                size = TX_FIFO_SIZE_1;
            }

            while(size)
            {
                bInChar = comTbuf1[comThead1++];
                UART1->DAT = bInChar;
                if(comThead1 >= TX_BUFSIZE)
                    comThead1 = 0;
                comTbytes1--;
                size--;
            }
        }
        else
        {
            /* No more data, just stop Tx (Stop work) */
            UART_DISABLE_INT(UART1, UART_INTEN_THREIEN_Msk);
        }
    }
}

void VCOM_TransferData(void)
{
    int32_t i, i32Len;

    /* Check whether USB is ready for next packet or not*/
    if(gu32TxSize0 == 0)
    {
        /* Check whether we have new COM Rx data to send to USB or not */
        if(comRbytes0)
        {
            i32Len = comRbytes0;
            if(i32Len > EP2_MAX_PKT_SIZE)
                i32Len = EP2_MAX_PKT_SIZE;

            for(i = 0; i < i32Len; i++)
            {
                gRxBuf0[i] = comRbuf0[comRhead0++];
                if(comRhead0 >= RX_BUFSIZE)
                    comRhead0 = 0;
            }

            __set_PRIMASK(1);
            comRbytes0 -= i32Len;
            __set_PRIMASK(0);

            gu32TxSize0 = i32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), (uint8_t *)gRxBuf0, i32Len);
            USBD_SET_PAYLOAD_LEN(EP2, i32Len);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is EP2_MAX_PKT_SIZE and
               no more data to send at this moment to note Host the transfer has been done */
            i32Len = USBD_GET_PAYLOAD_LEN(EP2);
            if(i32Len == EP2_MAX_PKT_SIZE)
                USBD_SET_PAYLOAD_LEN(EP2, 0);
        }
    }

    if(gu32TxSize1 == 0)
    {
        /* Check whether we have new COM Rx data to send to USB or not */
        if(comRbytes1)
        {
            i32Len = comRbytes1;
            if(i32Len > EP7_MAX_PKT_SIZE)
                i32Len = EP7_MAX_PKT_SIZE;

            for(i = 0; i < i32Len; i++)
            {
                gRxBuf1[i] = comRbuf1[comRhead1++];
                if(comRhead1 >= RX_BUFSIZE)
                    comRhead1 = 0;
            }

            __set_PRIMASK(1);
            comRbytes1 -= i32Len;
            __set_PRIMASK(0);

            gu32TxSize1 = i32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP7)), (uint8_t *)gRxBuf1, i32Len);
            USBD_SET_PAYLOAD_LEN(EP7, i32Len);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is EP7_MAX_PKT_SIZE and
               no more data to send at this moment to note Host the transfer has been done */
            i32Len = USBD_GET_PAYLOAD_LEN(EP7);
            if(i32Len == EP7_MAX_PKT_SIZE)
                USBD_SET_PAYLOAD_LEN(EP7, 0);
        }
    }

    /* Process the Bulk out data when bulk out data is ready. */
    if(gi8BulkOutReady0 && (gu32RxSize0 <= TX_BUFSIZE - comTbytes0))
    {
        for(i = 0; i < gu32RxSize0; i++)
        {
            comTbuf0[comTtail0++] = gpu8RxBuf0[i];
            if(comTtail0 >= TX_BUFSIZE)
                comTtail0 = 0;
        }

        __set_PRIMASK(1);
        comTbytes0 += gu32RxSize0;
        __set_PRIMASK(0);

        gu32RxSize0 = 0;
        gi8BulkOutReady0 = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    }

    if(gi8BulkOutReady1 && (gu32RxSize1 <= TX_BUFSIZE - comTbytes1))
    {
        for(i = 0; i < gu32RxSize1; i++)
        {
            comTbuf1[comTtail1++] = gpu8RxBuf1[i];
            if(comTtail1>= TX_BUFSIZE)
                comTtail1 = 0;
        }

        __set_PRIMASK(1);
        comTbytes1 += gu32RxSize1;
        __set_PRIMASK(0);

        gu32RxSize1 = 0;
        gi8BulkOutReady1 = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP6, EP6_MAX_PKT_SIZE);
    }

    /* Process the software Tx FIFO */
    if(comTbytes0)
    {
        /* Check if Tx is working */
        if((UART0->INTEN & UART_INTEN_THREIEN_Msk) == 0)
        {
            /* Send one bytes out */
            UART0->DAT = comTbuf0[comThead0++];
            if(comThead0 >= TX_BUFSIZE)
                comThead0 = 0;

            comTbytes0--;

            /* Enable Tx Empty Interrupt. (Trigger first one) */
            UART_ENABLE_INT(UART0, UART_INTEN_THREIEN_Msk);
        }
    }

    if(comTbytes1)
    {
        /* Check if Tx is working */
        if((UART1->INTEN & UART_INTEN_THREIEN_Msk) == 0)
        {
            /* Send one bytes out */
            UART1->DAT = comTbuf1[comThead1++];
            if(comThead1 >= TX_BUFSIZE)
                comThead1 = 0;

            comTbytes1--;

            /* Enable Tx Empty Interrupt. (Trigger first one) */
            UART_ENABLE_INT(UART1, UART_INTEN_THREIEN_Msk);
        }
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
#if CRYSTAL_LESS
    uint32_t u32TrimInit;
#endif

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    UART0_Init();

    UART1_Init();

    printf("\n\n");
    printf("+------------------------------------------------------------+\n");
    printf("|       NuMicro USB Virtual COM Dual Port Sample Code        |\n");
    printf("+------------------------------------------------------------+\n");
    printf("UART0 RXD=PB.12 and TXD=PB.13\n");
    printf("UART1 RXD=PB.2  and TXD=PB.3\n");

    /* Open USB controller */
    USBD_Open(&gsInfo, VCOM_ClassRequest, NULL);

    /* Endpoint configuration */
    VCOM_Init();

    /* Start USB device */
    USBD_Start();

    NVIC_EnableIRQ(USBD_IRQn);

    NVIC_EnableIRQ(UART02_IRQn);

    NVIC_EnableIRQ(UART1_IRQn);

#if CRYSTAL_LESS
    /* Backup default trim */
    u32TrimInit = M32(TRIM_INIT);
#endif

    /* Clear SOF */
    USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

    while(1)
    {
#if CRYSTAL_LESS
       /* Start USB trim if it is not enabled. */
        if((SYS->HIRCTRIMCTL & SYS_HIRCTRIMCTL_FREQSEL_Msk) != 1)
        {
            /* Start USB trim only when SOF */
            if(USBD->INTSTS & USBD_INTSTS_SOFIF_Msk)
            {
                /* Clear SOF */
                USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

                /* Re-enable crystal-less */
                SYS->HIRCTRIMCTL = 0x01;
                SYS->HIRCTRIMCTL |= SYS_HIRCTRIMCTL_REFCKSEL_Msk;
            }
        }

        /* Disable USB Trim when error */
        if(SYS->HIRCTRIMSTS & (SYS_HIRCTRIMSTS_CLKERIF_Msk | SYS_HIRCTRIMSTS_TFAILIF_Msk))
        {
            /* Init TRIM */
            M32(TRIM_INIT) = u32TrimInit;

            /* Disable crystal-less */
            SYS->HIRCTRIMCTL = 0;

            /* Clear error flags */
            SYS->HIRCTRIMSTS = SYS_HIRCTRIMSTS_CLKERIF_Msk | SYS_HIRCTRIMSTS_TFAILIF_Msk;

            /* Clear SOF */
            USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
        }
#endif
        /* Enter power down when USB suspend */
        if(g_u8Suspend)
            PowerDown();

        VCOM_TransferData();
    }
}



/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/


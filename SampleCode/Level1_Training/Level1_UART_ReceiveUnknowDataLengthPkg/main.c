/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 18/05/30 3:53p $
 * @brief    Show how to use the UART Rx timeout function for receiving unknown data length package.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define FIFO_THRESHOLD 4
#define RX_BUFFER_SIZE 128
#define RX_TIMEOUT_CNT 60 //40~255

enum
{
    eUART_RX_Received_Data_Finish = 0,
    eUART_RX_Received_Data_NOT_Finish
};

volatile uint8_t g_au8UART_RX_Buffer[RX_BUFFER_SIZE] = {0}; // UART Rx received data Buffer (RAM)
volatile uint8_t g_bUART_RX_Received_Data_State = eUART_RX_Received_Data_NOT_Finish;
volatile uint8_t g_u8UART_RDA_Trigger_Cnt = 0; // UART RDA interrupt trigger times counter
volatile uint8_t g_u8UART_RXTO_Trigger_Cnt = 0; // UART RXTO interrupt trigger times counter

void UART02_IRQHandler(void)
{
    uint8_t i;
    static uint16_t u16UART_RX_Buffer_Index = 0;

    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk))
    {
        /* UART receive data available flag */

        /* Record RDA interrupt trigger times */
        g_u8UART_RDA_Trigger_Cnt++;

        /* Move the data from Rx FIFO to sw buffer (RAM). */
        /* Every time leave 1 byte data in FIFO for Rx timeout */
        for(i = 0 ; i < (FIFO_THRESHOLD - 1) ; i++)
        {
            g_au8UART_RX_Buffer[u16UART_RX_Buffer_Index] = UART_READ(UART0);
            u16UART_RX_Buffer_Index ++;

            if (u16UART_RX_Buffer_Index >= RX_BUFFER_SIZE)
                u16UART_RX_Buffer_Index = 0;
        }
    }
    else if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RXTOINT_Msk))
    {
        /* When Rx timeout flag is set to 1, it means there is no data needs to be transmitted. */

        /* Record Timeout times */
        g_u8UART_RXTO_Trigger_Cnt++;

        /* Move the last data from Rx FIFO to sw buffer. */
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
            g_au8UART_RX_Buffer[u16UART_RX_Buffer_Index] = UART_READ(UART0);
            u16UART_RX_Buffer_Index ++;
        }

        /* Clear UART RX parameter */

        UART_DISABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
        u16UART_RX_Buffer_Index = 0;
        g_bUART_RX_Received_Data_State = eUART_RX_Received_Data_Finish;
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC and LIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC and LIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = ((SYS->GPB_MFPH & (~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)))
                     | (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD));

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

    /* Enable UART RDA and RX timeout interrupt */
    UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    NVIC_EnableIRQ(UART02_IRQn);

    /* Set RX Trigger Level as 4 bytes */
    UART0->FIFO = ((UART0->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_4BYTES);

    /* Set Timeout time counter in 60 bit-time and enable time-out counter */
    UART_SetTimeoutCnt(UART0, RX_TIMEOUT_CNT);
}

void DescriptionMessage(void)
{
    printf("+----------------------------------------------------------------------+\n");
    printf("|  This sample code performs how to receive unknown data length package.|\n");
    printf("|                                                                      |\n");
    printf("|   (1) Please send data to UART0 Rx(PB.12)                            |\n");
    printf("|   (2) UART will receive data by UART Rx RDA and RXTO interrupt.      |\n");
    printf("|   (3) User can modify the Rx Timeout counter RX_TIMEOUT_CNT for      |\n");
    printf("|       different timeout period.                                      |\n");
    printf("|                                                                      |\n");
    printf("|   Description for RX_TIMEOUT_CNT :                                   |\n");
    printf("|   -UART data = 8 bits                                                |\n");
    printf("|   -UART Parity = None                                                |\n");
    printf("|   -RX_TIMEOUT_CNT = 60                                               |\n");
    printf("|     If there is no data comes in 60 baudrate clock,                  |\n");
    printf("|     the UART Rx timeout interrupt flag will be set to 1.             |\n");
    printf("|     RX_TIMEOUT_CNT = 60 = 6 * ( 1 start + 8 data bits + 1 stop bit ) |\n");
    printf("|                         = 6 bytes data transmission times            |\n");
    printf("+----------------------------------------------------------------------+\n\n");

    UART_WAIT_TX_EMPTY(UART0);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32ResetBufferIndex;
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART */
    UART0_Init();

    /* Print description message */
    DescriptionMessage();

    while(1)
    {
        /* Wait to receive UART data */
        while(UART_RX_IDLE(UART0));

        /* Start to received UART data */
        g_bUART_RX_Received_Data_State = eUART_RX_Received_Data_NOT_Finish;
        /* Wait for receiving UART message finished */
        while(g_bUART_RX_Received_Data_State != eUART_RX_Received_Data_Finish);

        printf("\nUART0 Rx Received Data : %s\n",g_au8UART_RX_Buffer);
        printf("UART0 Rx RDA (Fifofull) interrupt times : %d\n",g_u8UART_RDA_Trigger_Cnt);
        printf("UART0 Rx RXTO (Timeout) interrupt times : %d\n",g_u8UART_RXTO_Trigger_Cnt);

        /* Reset SRAM Buffer */
        u32ResetBufferIndex = 0;
        while(g_au8UART_RX_Buffer[u32ResetBufferIndex] != '\0')
            g_au8UART_RX_Buffer[u32ResetBufferIndex++] = '\0';
        /* Reset UART interrupt parameter */
        UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
        g_u8UART_RDA_Trigger_Cnt = 0; // UART RDA interrupt times
        g_u8UART_RXTO_Trigger_Cnt = 0; // UART RXTO interrupt times
    }
}

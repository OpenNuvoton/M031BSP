/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 18/07/16 1:26p $
 * @brief
 *           Transmit and receive data in RS485 mode.
 *           This sample code needs to work with USCI_UART_RS485_Slave.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define RS485_ADDRSS1     0xC0
#define RS485_ADDRSS2     0xA2
#define RS485_ADDRSS3     0xB1
#define RS485_ADDRSS4     0xD3

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_SendAddressByte(uint8_t u8data);
void RS485_SendDataByte(uint8_t *pu8TxBuf, uint32_t u32WriteBytes);
void RS485_9bitModeMaster(void);
void RS485_FunctionTest(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Transmit Control  (Address Byte: Parity Bit =1 , Data Byte:Parity Bit =0)                        */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_SendAddressByte(uint8_t u8data)
{
    UUART_WRITE(UUART0, (0x100 | u8data));
}

void RS485_SendDataByte(uint8_t *pu8TxBuf, uint32_t u32WriteBytes)
{
    uint32_t u32Count;

    for(u32Count = 0; u32Count != u32WriteBytes; u32Count++)
    {
        while(UUART_GET_TX_FULL(UUART0));           /* Wait if Tx is full */
        UUART_WRITE(UUART0, pu8TxBuf[u32Count]);    /* Send UART Data from buffer */
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Transmit Test                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_9bitModeMaster(void)
{
    int32_t i32;
    uint8_t g_u8SendDataGroup1[10] = {0};
    uint8_t g_u8SendDataGroup2[10] = {0};
    uint8_t g_u8SendDataGroup3[10] = {0};
    uint8_t g_u8SendDataGroup4[10] = {0};

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|               RS485 9-bit Master Test                     |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| The function will send different address with 10 data     |\n");
    printf("| bytes to test RS485 9-bit mode. Please connect TX/RX to   |\n");
    printf("| another board and wait its ready to receive.              |\n");
    printf("| Press any key to start...                                 |\n");
    printf("+-----------------------------------------------------------+\n\n");
    getchar();

    /* Set UART line configuration and control signal output inverse */
    UUART0->LINECTL = UUART_WORD_LEN_9 | UUART_LINECTL_LSB_Msk | UUART_LINECTL_CTLOINV_Msk;

    /* Enable RTS auto direction function */
    UUART0->PROTCTL |= UUART_PROTCTL_RTSAUDIREN_Msk;

    /* Prepare data to transmit */
    for(i32 = 0; i32 < 10; i32++)
    {
        g_u8SendDataGroup1[i32] = i32;
        g_u8SendDataGroup2[i32] = i32 + 10;
        g_u8SendDataGroup3[i32] = i32 + 20;
        g_u8SendDataGroup4[i32] = i32 + 30;
    }

    /* Send different address and data for test */
    printf("Send Address %x and data 0~9\n", RS485_ADDRSS1);
    RS485_SendAddressByte(RS485_ADDRSS1);
    RS485_SendDataByte(g_u8SendDataGroup1, 10);

    printf("Send Address %x and data 10~19\n", RS485_ADDRSS2);
    RS485_SendAddressByte(RS485_ADDRSS2);
    RS485_SendDataByte(g_u8SendDataGroup2, 10);

    printf("Send Address %x and data 20~29\n", RS485_ADDRSS3);
    RS485_SendAddressByte(RS485_ADDRSS3);
    RS485_SendDataByte(g_u8SendDataGroup3, 10);

    printf("Send Address %x and data 30~39\n", RS485_ADDRSS4);
    RS485_SendAddressByte(RS485_ADDRSS4);
    RS485_SendDataByte(g_u8SendDataGroup4, 10);

    printf("Transfer Done\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Function Test                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_FunctionTest()
{
    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|      RS485 Function Test IO Setting                       |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  ______                                            _____  |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |Master|--USCI0_DAT1(PA.9) <==> USCI0_DAT0(PA.10) |Slave| |\n");
    printf("| |      |--USCI0_CTL1(PA.8) <==> USCI0_CTL1(PA.8)  |     | |\n");
    printf("| |______|                                          |_____| |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n");


    /*
        The sample code is used to test RS485 9-bit mode and needs
        two Module test board to complete the test.

        Master:
            1.Set RTS auto direction enabled and HW will control RTS pin. CTLOINV is set to '1'.
            2.Master will send four different address with 10 bytes data to test Slave.
            3.Address bytes : the parity bit should be '1'.
            4.Data bytes : the parity bit should be '0'.
            5.RTS pin is low in idle state. When master is sending, RTS pin will be pull high.

        Slave:
            1.Set RTS auto direction enabled and HW will control RTS pin. CTLOINV is set to '1'.
            2.The received byte, parity bit is '1' , is considered "ADDRESS".
            3.The received byte, parity bit is '0' , is considered "DATA".
    */

    RS485_9bitModeMaster();

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

    /* Enable UUART0 clock */
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

    /* Set UUART0 multi-function pins */
    SYS->GPA_MFPH = (SYS->GPA_MFPH & ~(SYS_GPA_MFPH_PA8MFP_Msk | SYS_GPA_MFPH_PA9MFP_Msk | SYS_GPA_MFPH_PA10MFP_Msk)) |
                    (SYS_GPA_MFPH_PA8MFP_USCI0_CTL1 | SYS_GPA_MFPH_PA9MFP_USCI0_DAT1 | SYS_GPA_MFPH_PA10MFP_USCI0_DAT0);

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

void USCI0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset USCI0 */
    SYS->IPRST2 |=  SYS_IPRST2_USCI0RST_Msk;
    SYS->IPRST2 &= ~SYS_IPRST2_USCI0RST_Msk;

    /* Configure USCI0 as UART mode */
    UUART0->CTL = (2 << UUART_CTL_FUNMODE_Pos);                                 /* Set UART function mode */
    UUART0->LINECTL = UUART_WORD_LEN_8 | UUART_LINECTL_LSB_Msk;                 /* Set UART line configuration */
    UUART0->DATIN0 = (2 << UUART_DATIN0_EDGEDET_Pos);                           /* Set falling edge detection */
    UUART0->BRGEN = (51 << UUART_BRGEN_CLKDIV_Pos) | (7 << UUART_BRGEN_DSCNT_Pos); /* Set UART baud rate as 115200bps */
    UUART0->PROTCTL |= UUART_PROTCTL_PROTEN_Msk;                                /* Enable UART protocol */
}

int main()
{
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART0_Init();

    /* Init USCI0 for test */
    USCI0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\nUART Sample Program\n");

    /* USCI UART RS485 sample function */
    RS485_FunctionTest();

    printf("\nUART Sample Program End\n");

    while(1);
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

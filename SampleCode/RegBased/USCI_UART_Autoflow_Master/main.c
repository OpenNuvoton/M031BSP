/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 18/07/16 1:24p $
 * @brief
 *           Transmit and receive data with auto flow control.
 *           This sample code needs to work with USCI_UART_Autoflow_Slave.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define RXBUFSIZE  1024

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_AutoFlow_FunctionTxTest(void);

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
    SYS->GPC_MFPH = (SYS->GPC_MFPH & ~(SYS_GPC_MFPH_PC14MFP_Msk)) | SYS_GPC_MFPH_PC14MFP_USCI0_CTL0;

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

    printf("\nUSCI UART Sample Program\n");

    /* USCI UART auto flow sample master function */
    USCI_AutoFlow_FunctionTxTest();

    while (1);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  AutoFlow Function Tx Test                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_AutoFlow_FunctionTxTest()
{
    uint32_t u32i;

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Pin Configure                                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  ______                                            _____  |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |Master|                                          |Slave| |\n");
    printf("| |    TX|--USCI0_DAT1(PA.9) <=> USCI0_DAT0(PA.10)--|RX   | |\n");
    printf("| |  nCTS|--USCI0_CTL0(PC.14) <=> USCI0_CTL1(PA.8)--|nRTS | |\n");
    printf("| |______|                                          |_____| |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n");

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|       AutoFlow Function Test (Master)                     |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code needs two boards. One is Master and    |\n");
    printf("|    the other is slave. Master will send 1k bytes data     |\n");
    printf("|    to slave. Slave will check if received data is correct |\n");
    printf("|    after getting 1k bytes data.                           |\n");
    printf("|    Press any key to start...                              |\n");
    printf("+-----------------------------------------------------------+\n");
    getchar();

    /* Enable CTS autoflow control */
    UUART0->PROTCTL |= UUART_PROTCTL_CTSAUTOEN_Msk;

    /* Send 1k bytes data */
    for (u32i = 0; u32i < RXBUFSIZE; u32i++)
    {
        /* Send 1 byte data */
        UUART0->TXDAT = (u32i & 0xFF);

        /* Wait if Tx FIFO is full */
        while(UUART0->BUFSTS & UUART_BUFSTS_TXFULL_Msk);
    }

    printf("\n Transmit Done\n");

}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

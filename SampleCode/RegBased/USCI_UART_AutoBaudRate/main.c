/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 18/07/16 1:23p $
 * @brief    Show how to use auto baud rate detection function
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define RXBUFSIZE   1024

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_AutoBaudRate_Test(void);
void USCI_AutoBaudRate_TxTest(void);
void USCI_AutoBaudRate_RxTest(void);

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
    SYS->GPA_MFPH = (SYS->GPA_MFPH & ~(SYS_GPA_MFPH_PA9MFP_Msk | SYS_GPA_MFPH_PA10MFP_Msk)) |
                    (SYS_GPA_MFPH_PA9MFP_USCI0_DAT1 | SYS_GPA_MFPH_PA10MFP_USCI0_DAT0);

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

    /* Init USCI0 */
    USCI0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\nUSCI UART Sample Program\n");

    /* USCI UART auto baud rate sample function */
    USCI_AutoBaudRate_Test();

    printf("\nUSCI UART Sample Program End\n");

    while (1);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Test                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_AutoBaudRate_Test()
{
    uint32_t u32Item;

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Pin Configure                                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  ______                                            _____  |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |Master|                                          |Slave| |\n");
    printf("| |    TX|--USCI0_DAT1(PA.9) <=> USCI0_DAT0(PA.10)--|RX   | |\n");
    printf("| |______|                                          |_____| |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n");

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Auto Baud Rate Function Test                          |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code needs two boards. One is Master and    |\n");
    printf("|    the other is slave.  Master will send input pattern    |\n");
    printf("|    0x1 with different baud rate. It can check if Slave    |\n");
    printf("|    calculates correct baud rate.                          |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Please select Master or Slave test                       |\n");
    printf("|  [0] Master    [1] Slave                                  |\n");
    printf("+-----------------------------------------------------------+\n");
    u32Item = getchar();

    if (u32Item == '0')
        USCI_AutoBaudRate_TxTest();
    else
        USCI_AutoBaudRate_RxTest();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Tx Test                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_AutoBaudRate_TxTest()
{
    uint32_t u32Item;

    do
    {

        printf("\n");
        printf("+-----------------------------------------------------------+\n");
        printf("|     Auto Baud Rate Function Test (Master)                 |\n");
        printf("+-----------------------------------------------------------+\n");
        printf("| [1] baud rate 38400 bps                                   |\n");
        printf("| [2] baud rate 57600 bps                                   |\n");
        printf("| [3] baud rate 115200 bps                                  |\n");
        printf("|                                                           |\n");
        printf("| Select baud rate and master will send 0x55 to slave ...   |\n");
        printf("+-----------------------------------------------------------+\n");
        printf("| Quit                                              - [ESC] |\n");
        printf("+-----------------------------------------------------------+\n\n");
        u32Item = getchar();
        printf("%c\n", u32Item);

        /* Set different baud rate */
        switch (u32Item)
        {
        case '1':
            UUART0->BRGEN = ((124 << UUART_BRGEN_CLKDIV_Pos) | (9 << UUART_BRGEN_DSCNT_Pos));
            break;

        case '2':
            UUART0->BRGEN = ((118 << UUART_BRGEN_CLKDIV_Pos) | (6 << UUART_BRGEN_DSCNT_Pos));
            break;

        default:
            UUART0->BRGEN = ((51 << UUART_BRGEN_CLKDIV_Pos) | (7 << UUART_BRGEN_DSCNT_Pos));
            break;
        }

        /* Send input pattern 0x55 for auto baud rate detection */
        UUART0->TXDAT = 0x55;

    }
    while (u32Item != 27);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Get UUART Baud Rate Function                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t GetUuartBaudrate(UUART_T *uuart)
{
    uint32_t u32PCLKFreq, u32PDSCnt, u32DSCnt, u32ClkDiv, u32PCLK0Div;

    /* Get PCLK frequency */
    u32PCLK0Div = (CLK->PCLKDIV & CLK_PCLKDIV_APB0DIV_Msk) >> CLK_PCLKDIV_APB0DIV_Pos;
    u32PCLKFreq = SystemCoreClock >> u32PCLK0Div;

    /* Get pre-divider counter */
    u32PDSCnt = ((uuart->BRGEN & UUART_BRGEN_PDSCNT_Msk) >> UUART_BRGEN_PDSCNT_Pos);

    /* Get denominator counter */
    u32DSCnt = ((uuart->BRGEN & UUART_BRGEN_DSCNT_Msk) >> UUART_BRGEN_DSCNT_Pos);

    /* Get clock divider */
    u32ClkDiv = ((uuart->BRGEN & UUART_BRGEN_CLKDIV_Msk) >> UUART_BRGEN_CLKDIV_Pos);

    return (u32PCLKFreq / (u32PDSCnt + 1) / (u32DSCnt + 1) / (u32ClkDiv + 1));
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Rx Test                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_AutoBaudRate_RxTest()
{
    /* Set CLKDIV=DSCNT=0x5, Timing Measurement Counter Enable and Timing Measurement Counter Clock Source */
    UUART0->BRGEN = ((0x5 << UUART_BRGEN_CLKDIV_Pos) | (0x5 << UUART_BRGEN_DSCNT_Pos) | (UUART_BRGEN_TMCNTEN_Msk) | (UUART_BRGEN_TMCNTSRC_Msk));

    /* Enable auto baud rate detect function */
    UUART0->PROTCTL |= UUART_PROTCTL_ABREN_Msk;

    printf("\nreceiving input pattern... ");

    /* Wait until auto baud rate detect finished or time-out */
    while (UUART0->PROTCTL & UUART_PROTCTL_ABREN_Msk);

    if(UUART0->PROTSTS & UUART_PROTSTS_ABRDETIF_Msk)
    {
        /* Clear auto baud rate detect finished flag */
        UUART0->PROTSTS = UUART_PROTSTS_ABRDETIF_Msk;
        printf("Baud rate is %dbps.\n", GetUuartBaudrate(UUART0));
    }
    else if(UUART0->PROTSTS & UUART_PROTSTS_ABERRSTS_Msk)
    {
        /* Clear auto baud rate detect time-out flag */
        UUART0->PROTSTS = UUART_PROTSTS_ABERRSTS_Msk;
        printf("Error!\n");
    }
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

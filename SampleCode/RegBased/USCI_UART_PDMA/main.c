/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 18/07/16 1:26p $
 * @brief
 *           This is a USCI_UART PDMA demo and need to connect USCI_UART TX and RX.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PDMA_TEST_LENGTH   128
#define PDMA_TX_CH 0
#define PDMA_RX_CH 1

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8Tx_Buffer[PDMA_TEST_LENGTH] = {0};
uint8_t g_u8Rx_Buffer[PDMA_TEST_LENGTH] = {0};

volatile uint32_t u32IsTestOver = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_PDMATest(void);

void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);

    if (status & PDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
        printf("target abort interrupt !!\n");
        u32IsTestOver = 2;

        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_GET_ABORT_STS(PDMA));
    }
    else if (status & PDMA_INTSTS_TDIF_Msk)     /* done */
    {
        if ((PDMA_GET_TD_STS(PDMA) & (1 << PDMA_TX_CH)) && (PDMA_GET_TD_STS(PDMA) & (1 << PDMA_RX_CH)))
        {
            u32IsTestOver = 1;
            PDMA_CLR_TD_FLAG(PDMA, PDMA_GET_TD_STS(PDMA));
        }
    }
    else if (status & (PDMA_INTSTS_REQTOF0_Msk | PDMA_INTSTS_REQTOF1_Msk))     /* channel timeout */
    {
        printf("timeout interrupt !!\n");
        u32IsTestOver = 3;
        PDMA_CLR_TMOUT_FLAG(PDMA, 0);
        PDMA_CLR_TMOUT_FLAG(PDMA, 1);
    }
    else
        printf("unknown interrupt !!\n");
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

void PDMA_Init(void)
{
    /* Enable PDMA channel */
    PDMA->CHCTL |= ((1 << PDMA_RX_CH) | (1 << PDMA_TX_CH));

    /* UUART Tx PDMA channel configuration */
    PDMA->DSCT[PDMA_TX_CH].CTL =
        (PDMA_TEST_LENGTH - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
        PDMA_WIDTH_8 |  /* Transfer width 8 bits */
        PDMA_DAR_FIX  | /* Fixed destination address */
        PDMA_SAR_INC  | /* Increment source address */
        PDMA_DSCT_CTL_TBINTDIS_Msk  | /* Table interrupt disabled */
        PDMA_REQ_SINGLE  | /* Single request type */
        PDMA_OP_BASIC;     /* Basic mode */
    PDMA->DSCT[PDMA_TX_CH].SA = (uint32_t)(&g_u8Tx_Buffer[0]);     /* Source address */
    PDMA->DSCT[PDMA_TX_CH].DA = (uint32_t)(&(UUART0->TXDAT));  /* Destination address */

    /* Request source selection */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & (~PDMA_REQSEL0_3_REQSRC0_Msk)) | (PDMA_USCI0_TX << PDMA_REQSEL0_3_REQSRC0_Pos);

    /* UUART Rx PDMA channel configuration */
    PDMA->DSCT[PDMA_RX_CH].CTL =
        (PDMA_TEST_LENGTH - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
        PDMA_WIDTH_8 |  /* Transfer width 8 bits */
        PDMA_DAR_INC  | /* Increment destination address */
        PDMA_SAR_FIX  | /* Fixed source address */
        PDMA_DSCT_CTL_TBINTDIS_Msk  | /* Table interrupt disabled */
        PDMA_REQ_SINGLE  | /* Single request type */
        PDMA_OP_BASIC;     /* Basic mode */
    PDMA->DSCT[PDMA_RX_CH].SA = (uint32_t)(&(UUART0->RXDAT));  /* Source address */
    PDMA->DSCT[PDMA_RX_CH].DA = (uint32_t)(&g_u8Rx_Buffer[0]);    /* Destination address */

    /* Request source selection */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & (~PDMA_REQSEL0_3_REQSRC1_Msk)) | (PDMA_USCI0_RX << PDMA_REQSEL0_3_REQSRC1_Pos);

#ifdef ENABLE_PDMA_INTERRUPT
    PDMA->INTEN |= ((1 << PDMA_RX_CH) | (1 << PDMA_TX_CH));
    NVIC_EnableIRQ(PDMA_IRQn);
    u32IsTestOver = 0;
#endif
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

    /* USCI UART sample function */
    USCI_UART_PDMATest();

    while (1);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI UART Function Test                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_PDMATest()
{
    uint32_t i;

    printf("+---------------------------------------------------------------------+\n");
    printf("|  USCI UART PDMA Test                                                |\n");
    printf("+---------------------------------------------------------------------+\n");
    printf("|  Description :                                                      |\n");
    printf("|    The sample code will demo USCI_UART PDMA function                |\n");
    printf("|    Please connect USCI0_UART_TX(PA.9) and USCI0_UART_RX(PA.10) pin. |\n");
    printf("+---------------------------------------------------------------------+\n");
    printf("Please press any key to start test. \n\n");

    for (i = 0; i < PDMA_TEST_LENGTH; i++)
    {
        g_u8Tx_Buffer[i] = i;
        g_u8Rx_Buffer[i] = 0xff;
    }

    /* PDMA reset */
    UUART0->PDMACTL |= UUART_PDMACTL_PDMARST_Msk;

    while (UUART0->PDMACTL & UUART_PDMACTL_PDMARST_Msk);

    PDMA_Init();

    /* Enable UART DMA Tx and Rx */
    UUART0->PDMACTL = UUART_PDMACTL_TXPDMAEN_Msk | UUART_PDMACTL_RXPDMAEN_Msk | UUART_PDMACTL_PDMAEN_Msk;

#ifdef ENABLE_PDMA_INTERRUPT

    while (u32IsTestOver == 0);

    if (u32IsTestOver == 1)
        printf("test done...\n");
    else if (u32IsTestOver == 2)
        printf("target abort...\n");
    else if (u32IsTestOver == 3)
        printf("timeout...\n");

#else

    while ((!(PDMA_GET_TD_STS(PDMA) & (1 << PDMA_TX_CH))) || (!(PDMA_GET_TD_STS(PDMA) & (1 << PDMA_RX_CH))));

    PDMA_CLR_TD_FLAG(PDMA, PDMA_GET_TD_STS(PDMA));
#endif

    for (i = 0; i < PDMA_TEST_LENGTH; i++)
    {
        if (g_u8Rx_Buffer[i] != i)
        {
            printf("\n Receive Data Compare Error !!");

            while (1);
        }

    }

    printf("\nUART PDMA test Pass.\n");

}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

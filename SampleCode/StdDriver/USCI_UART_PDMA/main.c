/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 18/07/16 10:20a $
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
#define PDMA_TX_CH   0
#define PDMA_RX_CH   1

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
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UUART0 clock */
    CLK_EnableModuleClock(USCI0_MODULE);

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

    /* Set UUART0 multi-function pins */
    SYS->GPA_MFPH = (SYS->GPA_MFPH & ~(SYS_GPA_MFPH_PA9MFP_Msk | SYS_GPA_MFPH_PA10MFP_Msk)) |
                    (SYS_GPA_MFPH_PA9MFP_USCI0_DAT1 | SYS_GPA_MFPH_PA10MFP_USCI0_DAT0);

    /* Lock protected registers */
    SYS_LockReg();
}

void USCI0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure USCI0 as UART mode */
    UUART_Open(UUART0, 115200);
}

void PDMA_Init(void)
{
    /* Open PDMA Channel */
    PDMA_Open(PDMA, 1 << PDMA_TX_CH); // Channel 0 for UUART0 TX
    PDMA_Open(PDMA, 1 << PDMA_RX_CH); // Channel 1 for UUART0 RX
    // Select basic mode
    PDMA_SetTransferMode(PDMA, PDMA_TX_CH, PDMA_USCI0_TX, 0, 0);
    PDMA_SetTransferMode(PDMA, PDMA_RX_CH, PDMA_USCI0_RX, 0, 0);
    // Set data width and transfer count
    PDMA_SetTransferCnt(PDMA, PDMA_TX_CH, PDMA_WIDTH_8, PDMA_TEST_LENGTH);
    PDMA_SetTransferCnt(PDMA, PDMA_RX_CH, PDMA_WIDTH_8, PDMA_TEST_LENGTH);
    //Set PDMA Transfer Address
    PDMA_SetTransferAddr(PDMA, PDMA_TX_CH, ((uint32_t)(&g_u8Tx_Buffer[0])), PDMA_SAR_INC, (uint32_t)(&(UUART0->TXDAT)), PDMA_DAR_FIX);
    PDMA_SetTransferAddr(PDMA, PDMA_RX_CH, (uint32_t)(&(UUART0->RXDAT)), PDMA_SAR_FIX, ((uint32_t)(&g_u8Rx_Buffer[0])), PDMA_DAR_INC);
    //Select Single Request
    PDMA_SetBurstType(PDMA, PDMA_TX_CH, PDMA_REQ_SINGLE, 0);
    PDMA_SetBurstType(PDMA, PDMA_RX_CH, PDMA_REQ_SINGLE, 0);

#ifdef ENABLE_PDMA_INTERRUPT
    PDMA_EnableInt(PDMA, PDMA_TX_CH, PDMA_INT_TRANS_DONE);
    PDMA_EnableInt(PDMA, PDMA_RX_CH, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA_IRQn);
    u32IsTestOver = 0;
#endif
}

int main()
{
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

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
    UUART_TRIGGER_RX_PDMA(UUART0);
    UUART_TRIGGER_TX_PDMA(UUART0);

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

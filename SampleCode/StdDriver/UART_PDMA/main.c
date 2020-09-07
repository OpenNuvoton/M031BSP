/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 5 $
 * $Date: 18/07/16 10:24a $
 * @brief
 *           Transmit and receive UART data with PDMA.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define UART_RX_DMA_CH 0
#define UART_TX_DMA_CH 1

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
int32_t UART_TEST_LENGTH = 64;
uint8_t SrcArray[64];
uint8_t DestArray[64];
volatile int32_t IntCnt;
volatile int32_t IsTestOver;
volatile uint32_t g_u32TwoChannelPdmaTest = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);


/*---------------------------------------------------------------------------------------------------------*/
/* Clear buffer funcion                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void ClearBuf(uint32_t u32Addr, uint32_t u32Length, uint8_t u8Pattern)
{
    uint8_t* pu8Ptr;
    uint32_t i;

    pu8Ptr = (uint8_t *)u32Addr;

    for(i = 0; i < u32Length; i++)
    {
        *pu8Ptr++ = u8Pattern;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* Bulid Src Pattern function                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void BuildSrcPattern(uint32_t u32Addr, uint32_t u32Length)
{
    uint32_t i = 0, j, loop;
    uint8_t* pAddr;

    pAddr = (uint8_t *)u32Addr;

    do
    {
        if(u32Length > 256)     /* Pattern from 0 ~ 255 */
            loop = 256;
        else
            loop = u32Length;

        u32Length = u32Length - loop;

        for(j = 0; j < loop; j++)
            *pAddr++ = (uint8_t)(j + i);

        i++;
    }
    while((loop != 0) || (u32Length != 0));
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Tx PDMA Channel Configuration                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_UART_TxTest(void)
{
    /* UART Tx PDMA channel configuration */
    /* Set transfer width (8 bits) and transfer count */
    PDMA_SetTransferCnt(PDMA, UART_TX_DMA_CH, PDMA_WIDTH_8, UART_TEST_LENGTH);

    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA, UART_TX_DMA_CH, (uint32_t)SrcArray, PDMA_SAR_INC, (uint32_t)&UART1->DAT, PDMA_DAR_FIX);

    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA, UART_TX_DMA_CH, PDMA_UART1_TX, FALSE, 0);

    /* Single request type */
    PDMA_SetBurstType(PDMA, UART_TX_DMA_CH, PDMA_REQ_SINGLE, 0);

    /* Disable table interrupt */
    PDMA_DisableInt(PDMA,UART_TX_DMA_CH, PDMA_INT_TEMPTY );
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Rx PDMA Channel Configuration                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_UART_RxTest(void)
{
    /* UART Rx PDMA channel configuration */
    /* Set transfer width (8 bits) and transfer count */
    PDMA_SetTransferCnt(PDMA, UART_RX_DMA_CH, PDMA_WIDTH_8, UART_TEST_LENGTH);

    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA, UART_RX_DMA_CH, (uint32_t)&UART1->DAT, PDMA_SAR_FIX, (uint32_t)DestArray, PDMA_DAR_INC);

    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA, UART_RX_DMA_CH, PDMA_UART1_RX, FALSE, 0);

    /* Single request type */
    PDMA_SetBurstType(PDMA, UART_RX_DMA_CH, PDMA_REQ_SINGLE, 0);

    /* Disable table interrupt */
    PDMA_DisableInt(PDMA,UART_RX_DMA_CH, PDMA_INT_TEMPTY );
}

/*---------------------------------------------------------------------------------------------------------*/
/* PDMA Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_Callback_0(void)
{
    printf("\tTransfer Done %d!\r", ++IntCnt);

    /* Use PDMA to do UART loopback test 10 times */
    if(IntCnt < 10)
    {
        /* UART Tx and Rx PDMA configuration */
        PDMA_UART_TxTest();
        PDMA_UART_RxTest();

        /* Enable UART Tx and Rx PDMA function */
        UART_ENABLE_INT(UART1, (UART_INTEN_RXPDMAEN_Msk | UART_INTEN_TXPDMAEN_Msk));
    }
    else
    {
        /* Test is over */
        IsTestOver = TRUE;
    }
}

void PDMA_Callback_1(void)
{
    int32_t i ;

    printf("\tTransfer Done %d!\t", ++IntCnt);

    /* Show UART Rx data */
    for(i = 0; i < UART_TEST_LENGTH; i++)
        printf(" 0x%x(%c),", inpb(((uint32_t)DestArray + i)), inpb(((uint32_t)DestArray + i)));
    printf("\n");

    /* Use PDMA to do UART Rx test 10 times */
    if(IntCnt < 10)
    {
        /* UART Rx PDMA configuration */
        PDMA_UART_RxTest();

        /* Enable UART Rx PDMA function */
        UART_ENABLE_INT(UART1, UART_INTEN_RXPDMAEN_Msk);
    }
    else
    {
        /* Test is over */
        IsTestOver = TRUE;
    }
}

void PDMA_IRQHandler(void)
{
    /* Get PDMA interrupt status */
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);

    if(status & PDMA_INTSTS_ABTIF_Msk)   /* Target Abort */
    {
        if (PDMA_GET_ABORT_STS(PDMA) & PDMA_ABTSTS_ABTIF2_Msk)
            IsTestOver = 2;

        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_GET_ABORT_STS(PDMA));
    }
    else if(status & PDMA_INTSTS_TDIF_Msk)     /* Transfer Done */
    {
        /* UART Tx PDMA transfer done interrupt flag */
        if(PDMA_GET_TD_STS(PDMA) & (1 << UART_TX_DMA_CH))
        {
            /* Clear PDMA transfer done interrupt flag */
            PDMA_CLR_TD_FLAG(PDMA, (1 << UART_TX_DMA_CH));

            /* Disable UART Tx PDMA function */
            UART_DISABLE_INT(UART1, UART_INTEN_TXPDMAEN_Msk);
        }

        /* UART Rx PDMA transfer done interrupt flag */
        if(PDMA_GET_TD_STS(PDMA) & (1 << UART_RX_DMA_CH))
        {
            /* Clear PDMA transfer done interrupt flag */
            PDMA_CLR_TD_FLAG(PDMA, (1 << UART_RX_DMA_CH));

            /* Disable UART Rx PDMA function */
            UART_DISABLE_INT(UART1, UART_INTEN_RXPDMAEN_Msk);

            /* Handle PDMA transfer done interrupt event */
            if(g_u32TwoChannelPdmaTest == 1)
            {
                PDMA_Callback_0();
            }
            else if(g_u32TwoChannelPdmaTest == 0)
            {
                PDMA_Callback_1();
            }
        }
    }
    else
    {
        printf("unknown interrupt, status=0x%x !!\n", status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART02_IRQHandler(void)
{
    /* Get UART0 Rx data and send the data to UART1 Tx */
    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAIF_Msk))
        UART_WRITE(UART1,UART_READ(UART0));
}

/*---------------------------------------------------------------------------------------------------------*/
/* PDMA Sample Code:                                                                                       */
/*         i32option : ['1'] UART1 TX/RX PDMA Loopback                                                     */
/*                     [Others] UART1 RX PDMA test                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_UART(int32_t i32option)
{
    /* Source data initiation */
    BuildSrcPattern((uint32_t)SrcArray, UART_TEST_LENGTH);
    ClearBuf((uint32_t)DestArray, UART_TEST_LENGTH, 0xFF);

    /* Reset PDMA module */
    SYS_ResetModule(PDMA_RST);

    if(i32option == '1')
    {
        printf("  [Using TWO PDMA channel].\n");
        printf("  This sample code will use PDMA to do UART1 loopback test 10 times.\n");
        printf("  Please connect UART1_RXD(PB.2) <--> UART1_TXD(PB.3) before testing.\n");
        printf("  After connecting PB.2 <--> PB.3, press any key to start transfer.\n");
        g_u32TwoChannelPdmaTest = 1;
        getchar();
    }
    else
    {
        UART_TEST_LENGTH = 2;      /* Test Length */
        printf("  [Using ONE PDMA channel].\n");
        printf("  This sample code will use PDMA to do UART1 Rx test 10 times.\n");
        printf("  Please connect UART1_RXD(PB.2) <--> UART1_TXD(PB.3) before testing.\n");
        printf("  After connecting PB.2 <--> PB.3, press any key to start transfer.\n");
        g_u32TwoChannelPdmaTest = 0;
        getchar();
        printf("  Please input %d bytes to trigger PDMA one time.(Ex: Press 'a''b')\n", UART_TEST_LENGTH);
    }

    if(g_u32TwoChannelPdmaTest == 1)
    {
        /* Enable PDMA channel */
        PDMA_Open(PDMA, (1 << UART_RX_DMA_CH) | (1 << UART_TX_DMA_CH));

        /* UART Tx and Rx PDMA configuration */
        PDMA_UART_TxTest();
        PDMA_UART_RxTest();

        /* Enable PDMA Transfer Done Interrupt */
        PDMA_EnableInt(PDMA, UART_RX_DMA_CH, PDMA_INT_TRANS_DONE);
        PDMA_EnableInt(PDMA, UART_TX_DMA_CH, PDMA_INT_TRANS_DONE);
    }
    else
    {
        /* Enable PDMA channel */
        PDMA_Open(PDMA, (1 << UART_RX_DMA_CH));

        /* UART Rx PDMA configuration */
        PDMA_UART_RxTest();

        /* Enable PDMA Transfer Done Interrupt */
        PDMA_EnableInt(PDMA, UART_RX_DMA_CH, PDMA_INT_TRANS_DONE);
    }

    /* Enable PDMA Transfer Done Interrupt */
    IntCnt = 0;
    IsTestOver = FALSE;
    NVIC_EnableIRQ(PDMA_IRQn);

    /* Enable UART0 RDA interrupt */
    if(g_u32TwoChannelPdmaTest == 0)
    {
        NVIC_EnableIRQ(UART02_IRQn);
        UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk);
    }

    /* Enable UART Tx and Rx PDMA function */
    if(g_u32TwoChannelPdmaTest == 1)
        UART_ENABLE_INT(UART1,UART_INTEN_TXPDMAEN_Msk );
    else
        UART_DISABLE_INT(UART1, UART_INTEN_TXPDMAEN_Msk);

    UART_ENABLE_INT(UART1,UART_INTEN_RXPDMAEN_Msk );

    /* Wait for PDMA operation finish */
    while(IsTestOver == FALSE);

    /* Check PDMA status */
    if(IsTestOver == 2)
        printf("target abort...\n");

    /* Disable UART Tx and Rx PDMA function */
    UART_DISABLE_INT(UART1, (UART_INTEN_TXPDMAEN_Msk | UART_INTEN_RXPDMAEN_Msk));

    /* Disable PDMA channel */
    PDMA_Close(PDMA);

    /* Disable PDMA Interrupt */
    PDMA_DisableInt(PDMA, UART_RX_DMA_CH, PDMA_INT_TRANS_DONE);
    PDMA_DisableInt(PDMA, UART_TX_DMA_CH, PDMA_INT_TRANS_DONE);
    NVIC_DisableIRQ(PDMA_IRQn);

    /* Disable UART0 RDA interrupt */
    UART_DisableInt(UART0, UART_INTEN_RDAIEN_Msk);
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

    /* Set both PCLK0 and PCLK1 as HCLK */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1;

    /* Select IP clock source */
    /* Select UART0 clock source is HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    /* Select UART1 clock source is HIRC */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));

    /* Enable UART0 peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Enable UART1 peripheral clock */
    CLK_EnableModuleClock(UART1_MODULE);
    /* Enable PDMA module clock */
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))    |       \
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set PB multi-function pins for UART1 RXD(PB.2) and TXD(PB.3) */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk))    |       \
                    (SYS_GPB_MFPL_PB2MFP_UART1_RXD | SYS_GPB_MFPL_PB3MFP_UART1_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

void UART1_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART1 */
    SYS_ResetModule(UART1_RST);

    /* Configure UART1 and set UART1 Baudrate */
    UART_Open(UART1, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    uint8_t unItem;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init UART1 for test */
    UART1_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    printf("\nUART PDMA Sample Program");

    /* UART PDMA sample function */
    do
    {
        printf("\n\n");
        printf("+------------------------------------------------------------------------+\n");
        printf("|                      UART PDMA Driver Sample Code                      |\n");
        printf("+------------------------------------------------------------------------+\n");
        printf("| [1] Using TWO PDMA channel to test. < TX1(CH1)-->RX1(CH0) >            |\n");
        printf("| [2] Using ONE PDMA channel to test. < TX1-->RX1(CH0) >                 |\n");
        printf("+------------------------------------------------------------------------+\n");
        unItem = getchar();

        IsTestOver = FALSE;
        if((unItem == '1') || (unItem == '2'))
        {
            PDMA_UART(unItem);
            printf("\n\n  UART PDMA sample code is complete.\n");
        }

    }
    while(unItem != 27);

    while(1);

}

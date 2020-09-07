/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 5 $
 * $Date: 18/07/16 10:28a $
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


#define PLLCTL_SETTING      CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK           72000000

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
        if(u32Length > 256)
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
    PDMA->DSCT[UART_TX_DMA_CH].CTL =
        (UART_TEST_LENGTH - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
        PDMA_WIDTH_8 |  /* Transfer width 8 bits */
        PDMA_DAR_FIX  | /* Fixed destination address */
        PDMA_SAR_INC  | /* Increment source address */
        PDMA_DSCT_CTL_TBINTDIS_Msk  | /* Table interrupt disabled */
        PDMA_REQ_SINGLE  | /* Single request type */
        PDMA_OP_BASIC;     /* Basic mode */
    PDMA->DSCT[UART_TX_DMA_CH].SA = (uint32_t)SrcArray;     /* Source address */
    PDMA->DSCT[UART_TX_DMA_CH].DA = (uint32_t)&UART1->DAT;  /* Destination address */

    /* Request source selection */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & (~PDMA_REQSEL0_3_REQSRC1_Msk)) | (PDMA_UART1_TX << PDMA_REQSEL0_3_REQSRC1_Pos);
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Rx PDMA Channel Configuration                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_UART_RxTest(void)
{
    /* UART Rx PDMA channel configuration */
    PDMA->DSCT[UART_RX_DMA_CH].CTL =
        (UART_TEST_LENGTH - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
        PDMA_WIDTH_8 |  /* Transfer width 8 bits */
        PDMA_DAR_INC  | /* Increment destination address */
        PDMA_SAR_FIX  | /* Fixed source address */
        PDMA_DSCT_CTL_TBINTDIS_Msk  | /* Table interrupt disabled */
        PDMA_REQ_SINGLE  | /* Single request type */
        PDMA_OP_BASIC;     /* Basic mode */
    PDMA->DSCT[UART_RX_DMA_CH].SA = (uint32_t)&UART1->DAT;  /* Source address */
    PDMA->DSCT[UART_RX_DMA_CH].DA = (uint32_t)DestArray;    /* Destination address */

    /* Request source selection */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & (~PDMA_REQSEL0_3_REQSRC0_Msk)) | (PDMA_UART1_RX << PDMA_REQSEL0_3_REQSRC0_Pos);
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
        UART1->INTEN |= (UART_INTEN_RXPDMAEN_Msk | UART_INTEN_TXPDMAEN_Msk);
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
        UART1->INTEN |= UART_INTEN_RXPDMAEN_Msk;
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
    uint32_t u32ABTSTS  = PDMA->ABTSTS;

    if(status & PDMA_INTSTS_ABTIF_Msk)   /* Target Abort */
    {
        IsTestOver = 2;
        PDMA->ABTSTS = u32ABTSTS;
    }
    else if(status & PDMA_INTSTS_TDIF_Msk)     /* Transfer Done */
    {
        /* UART Tx PDMA transfer done interrupt flag */
        if(PDMA_GET_TD_STS(PDMA) & (1 << UART_TX_DMA_CH))
        {
            /* Clear PDMA transfer done interrupt flag */
            PDMA_CLR_TD_FLAG(PDMA, (1 << UART_TX_DMA_CH));

            /* Disable UART Tx PDMA function */
            UART1->INTEN &= ~UART_INTEN_TXPDMAEN_Msk;
        }

        /* UART Rx PDMA transfer done interrupt flag */
        if(PDMA_GET_TD_STS(PDMA) & (1 << UART_RX_DMA_CH))
        {
            /* Clear PDMA transfer done interrupt flag */
            PDMA_CLR_TD_FLAG(PDMA, (1 << UART_RX_DMA_CH));

            /* Disable UART Rx PDMA function */
            UART1->INTEN &= ~UART_INTEN_RXPDMAEN_Msk;

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
        UART1->DAT = UART0->DAT;
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
    SYS->IPRST0 |= SYS_IPRST0_PDMARST_Msk;
    SYS->IPRST0 &= ~SYS_IPRST0_PDMARST_Msk;

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
        PDMA->CHCTL |= ((1 << UART_RX_DMA_CH) | (1 << UART_TX_DMA_CH));

        /* UART Tx and Rx PDMA configuration */
        PDMA_UART_TxTest();
        PDMA_UART_RxTest();

        /* Enable PDMA Transfer Done Interrupt */
        PDMA->INTEN |= ((1 << UART_TX_DMA_CH) | (1 << UART_RX_DMA_CH));
    }
    else
    {
        /* Enable PDMA channel */
        PDMA->CHCTL |= (1 << UART_RX_DMA_CH);

        /* UART Rx PDMA configuration */
        PDMA_UART_RxTest();

        /* Enable PDMA Transfer Done Interrupt */
        PDMA->INTEN |= (1 << UART_RX_DMA_CH);
    }

    /* Enable PDMA Transfer Done Interrupt */
    IntCnt = 0;
    IsTestOver = FALSE;
    NVIC_EnableIRQ(PDMA_IRQn);

    /* Enable UART0 RDA interrupt */
    if(g_u32TwoChannelPdmaTest == 0)
    {
        UART0->INTEN |= UART_INTEN_RDAIEN_Msk;
        NVIC_EnableIRQ(UART02_IRQn);
    }

    /* Enable UART Tx and Rx PDMA function */
    if(g_u32TwoChannelPdmaTest == 1)
        UART1->INTEN |= UART_INTEN_TXPDMAEN_Msk;
    else
        UART1->INTEN &= ~UART_INTEN_TXPDMAEN_Msk;

    UART1->INTEN |= UART_INTEN_RXPDMAEN_Msk;

    /* Wait for PDMA operation finish */
    while(IsTestOver == FALSE);

    /* Check PDMA status */
    if(IsTestOver == 2)
        printf("target abort...\n");

    /* Disable UART Tx and Rx PDMA function */
    UART1->INTEN &= ~(UART_INTEN_TXPDMAEN_Msk | UART_INTEN_RXPDMAEN_Msk);

    /* Disable PDMA channel */
    PDMA->CHCTL = 0;

    /* Disable PDMA Interrupt */
    PDMA->INTEN = 0;
    NVIC_DisableIRQ(PDMA_IRQn);

    /* Disable UART0 RDA interrupt */
    UART0->INTEN &= ~UART_INTEN_RDAIEN_Msk;
    NVIC_DisableIRQ(UART02_IRQn);
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
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);

    /* Set PLL to Power-down mode */
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Enable UART module clock */
    CLK->APBCLK0 |= (CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_UART1CKEN_Msk);

    /* Enable PDMA peripheral clock */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART0SEL_Msk)) | CLK_CLKSEL1_UART0SEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UART0DIV_Msk)) | CLK_CLKDIV0_UART0(1);

    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART1SEL_Msk)) | CLK_CLKSEL1_UART1SEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UART1DIV_Msk)) | CLK_CLKDIV0_UART1(1);
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
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void UART1_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART1 */
    SYS->IPRST1 |=  SYS_IPRST1_UART1RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART1RST_Msk;

    /* Configure UART1 and set UART1 baud rate */
    UART1->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART1->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    uint8_t unItem;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

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

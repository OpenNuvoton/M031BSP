/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 18/07/13 4:20p $
 * @brief    Use PDMA channel 1 to transfer data from memory to memory.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PDMA_CH    1

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t PDMA_TEST_LENGTH = 64;
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t au8SrcArray[256];
uint8_t au8DestArray[256];
#else
__attribute__((aligned(4))) uint8_t au8SrcArray[256];
__attribute__((aligned(4))) uint8_t au8DestArray[256];
#endif
uint32_t volatile g_u32IsTestOver = 0;

/**
 * @brief       DMA IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The DMA default IRQ, declared in startup_M031Series.s.
 */
void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);

    if (status & PDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
        /* Check if channel 1 has abort error */
        if (PDMA_GET_ABORT_STS(PDMA) & PDMA_ABTSTS_ABTIF1_Msk)
            g_u32IsTestOver = 2;

        /* Clear abort flag of channel 1 */
        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_ABTSTS_ABTIF1_Msk);
    }
    else if (status & PDMA_INTSTS_TDIF_Msk)     /* done */
    {
        /* Check transmission of channel 1 has been transfer done */
        if (PDMA_GET_TD_STS(PDMA) & PDMA_TDSTS_TDIF1_Msk)
            g_u32IsTestOver = 1;

        /* Clear transfer done flag of channel 1 */
        PDMA_CLR_TD_FLAG(PDMA, PDMA_TDSTS_TDIF1_Msk);
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

int main()
{
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+------------------------------------------------------+ \n");
    printf("|    PDMA Memory to Memory Driver Sample Code          | \n");
    printf("+------------------------------------------------------+ \n");

    /*------------------------------------------------------------------------------------------------------

                         au8SrcArray                         au8DestArray
                         ---------------------------   -->   ---------------------------
                       /| [0]  | [1]  |  [2] |  [3] |       | [0]  | [1]  |  [2] |  [3] |\
                        |      |      |      |      |       |      |      |      |      |
       PDMA_TEST_LENGTH |            ...            |       |            ...            | PDMA_TEST_LENGTH
                        |      |      |      |      |       |      |      |      |      |
                       \| [60] | [61] | [62] | [63] |       | [60] | [61] | [62] | [63] |/
                         ---------------------------         ---------------------------
                         \                         /         \                         /
                               32bits(one word)                     32bits(one word)

      PDMA transfer configuration:

        Channel = 1
        Operation mode = basic mode
        Request source = PDMA_MEM(memory to memory)
        transfer done and table empty interrupt = enable

        Transfer count = PDMA_TEST_LENGTH
        Transfer width = 32 bits(one word)
        Source address = au8SrcArray
        Source address increment size = 32 bits(one word)
        Destination address = au8DestArray
        Destination address increment size = 32 bits(one word)
        Transfer type = burst transfer

        Total transfer length = PDMA_TEST_LENGTH * 32 bits
    ------------------------------------------------------------------------------------------------------*/

    /* Open Channel 1 */
    PDMA->CHCTL |= (1 << PDMA_CH);

    /* Transfer configuration of Channel 1 */
    PDMA->DSCT[PDMA_CH].CTL = ((PDMA_TEST_LENGTH - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | /* Transfer count is PDMA_TEST_LENGTH */ \
                              PDMA_WIDTH_32 |  /* Transfer width is 32 bits(one word) */ \
                              PDMA_SAR_INC |   /* Source increment size is 32 bits(one word) */ \
                              PDMA_DAR_INC |   /* Destination increment size is 32 bits(one word) */ \
                              PDMA_REQ_BURST | /* Transfer type is burst transfer type */ \
                              PDMA_BURST_4 |   /* Burst size is 4. No effect in single transfer type */ \
                              PDMA_OP_BASIC;   /* Operation mode is basic mode */

    /* Configure source address */
    PDMA->DSCT[PDMA_CH].SA = (uint32_t)au8SrcArray;

    /* Configure destination address */
    PDMA->DSCT[PDMA_CH].DA = (uint32_t)au8DestArray;

    /* Configure PDMA channel 1 as memory to memory transfer */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC1_Msk) | (PDMA_MEM << PDMA_REQSEL0_3_REQSRC1_Pos);

    /* Enable interrupt */
    PDMA->INTEN |= (1 << PDMA_CH);

    /* Enable NVIC for PDMA */
    NVIC_EnableIRQ(PDMA_IRQn);

    g_u32IsTestOver = 0;

    /* Generate a software request to trigger transfer with PDMA channel 1  */
    PDMA->SWREQ = (1 << PDMA_CH);

    /* Waiting for transfer done */
    while (g_u32IsTestOver == 0);

    /* Check transfer result */
    if (g_u32IsTestOver == 1)
        printf("test done...\n");
    else if (g_u32IsTestOver == 2)
        printf("target abort...\n");

    /* Close channel 1 */
    PDMA->CHCTL = 0;

    while (1);

}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

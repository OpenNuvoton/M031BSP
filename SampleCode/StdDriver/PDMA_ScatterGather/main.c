/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 18/07/12 9:32a $
 * @brief    Use PDMA channel 1 to transfer data
 *           from memory to memory by scatter-gather mode.
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
uint8_t au8DestArray0[256];
uint8_t au8DestArray1[256];
#else
__attribute__((aligned(4))) uint8_t au8SrcArray[256];
__attribute__((aligned(4))) uint8_t au8DestArray0[256];
__attribute__((aligned(4))) uint8_t au8DestArray1[256];
#endif

typedef struct dma_desc_t
{
    uint32_t ctl;
    uint32_t src;
    uint32_t dest;
    uint32_t offset;
} DMA_DESC_T;

DMA_DESC_T DMA_DESC[2];

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

    /* Lock protected registers */
    SYS_LockReg();
}

int main()
{
    uint32_t u32Src, u32Dst0, u32Dst1, i;

    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("+-----------------------------------------------------------------------+ \n");
    printf("|    M031 PDMA Memory to Memory Driver Sample Code (Scatter-gather)     | \n");
    printf("+-----------------------------------------------------------------------+ \n");

    u32Src = (uint32_t)au8SrcArray;
    u32Dst0 = (uint32_t)au8DestArray0;
    u32Dst1 = (uint32_t)au8DestArray1;

    for (i = 0; i < sizeof(au8SrcArray) / sizeof(au8SrcArray[0]); i++)
    {
        au8SrcArray[i] = i;
    }

    /* This sample will transfer data by finished two descriptor table in sequence.(descriptor table 1 -> descriptor table 2) */

    /*----------------------------------------------------------------------------------
      PDMA transfer configuration:

        Channel = 1
        Operation mode = scatter-gather mode
        First scatter-gather descriptor table = DMA_DESC[0]
        Request source = PDMA_MEM(memory to memory)

        Transmission flow:
           ------------------------      -----------------------
          |                        |    |                       |
          |  DMA_DESC[0]           | -> |  DMA_DESC[1]          | -> transfer done
          |  (Descriptor table 1)  |    |  (Descriptor table 2) |
          |                        |    |                       |
           ------------------------      -----------------------

    ----------------------------------------------------------------------------------*/

    /* Open Channel 1 */
    PDMA_Open(PDMA, 1 << PDMA_CH);
    /* Enable Scatter Gather mode, assign the first scatter-gather descriptor table is table 1,
       and set transfer mode as memory to memory */
    PDMA_SetTransferMode(PDMA, PDMA_CH, PDMA_MEM, TRUE, (uint32_t)&DMA_DESC[0]);

    /*------------------------------------------------------------------------------------------------------

                         au8SrcArray                         au8DestArray0
                         ---------------------------   -->   ---------------------------
                       /| [0]  | [1]  |  [2] |  [3] |       | [0]  | [1]  |  [2] |  [3] |\
                        |      |      |      |      |       |      |      |      |      |
       PDMA_TEST_LENGTH |            ...            |       |            ...            | PDMA_TEST_LENGTH
                        |      |      |      |      |       |      |      |      |      |
                       \| [60] | [61] | [62] | [63] |       | [60] | [61] | [62] | [63] |/
                         ---------------------------         ---------------------------
                         \                         /         \                         /
                               32bits(one word)                     32bits(one word)

      Descriptor table 1 configuration:

        Operation mode = scatter-gather mode
        Next descriptor table = DMA_DESC[1](Descriptor table 2)
        transfer done and table empty interrupt = disable

        Transfer count = PDMA_TEST_LENGTH
        Transfer width = 32 bits(one word)
        Source address = au8SrcArray
        Source address increment size = 32 bits(one word)
        Destination address = au8DestArray0
        Destination address increment size = 32 bits(one word)
        Transfer type = burst transfer

        Total transfer length = PDMA_TEST_LENGTH * 32 bits
    ------------------------------------------------------------------------------------------------------*/
    DMA_DESC[0].ctl =
        ((PDMA_TEST_LENGTH - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | /* Transfer count is PDMA_TEST_LENGTH */ \
        PDMA_WIDTH_32 |   /* Transfer width is 32 bits(one word) */ \
        PDMA_SAR_INC |    /* Source increment size is 32 bits(one word) */ \
        PDMA_DAR_INC |    /* Destination increment size is 32 bits(one word) */ \
        PDMA_REQ_BURST |  /* Transfer type is burst transfer type */ \
        PDMA_BURST_128 |  /* Burst size is 128. No effect in single transfer type */ \
        PDMA_TBINTDIS_DISABLE |   /* Disable transfer done and table empty interrupt */ \
        PDMA_OP_SCATTER;  /* Operation mode is scatter-gather mode */

    /* Configure source address */
    DMA_DESC[0].src = u32Src;
    /* Configure destination address */
    DMA_DESC[0].dest = u32Dst0;
    /* Configure next descriptor table address */
    DMA_DESC[0].offset = (uint32_t)&DMA_DESC[1] - (PDMA->SCATBA); /* next descriptor table is table 2 */


    /*------------------------------------------------------------------------------------------------------

                         au8DestArray0                       au8DestArray1
                         ---------------------------   -->   ---------------------------
                       /| [0]  | [1]  |  [2] |  [3] |       | [0]  | [1]  |  [2] |  [3] |\
                        |      |      |      |      |       |      |      |      |      |
       PDMA_TEST_LENGTH |            ...            |       |            ...            | PDMA_TEST_LENGTH
                        |      |      |      |      |       |      |      |      |      |
                       \| [60] | [61] | [62] | [63] |       | [60] | [61] | [62] | [63] |/
                         ---------------------------         ---------------------------
                         \                         /         \                         /
                               32bits(one word)                     32bits(one word)

      Descriptor table 2 configuration:

        Operation mode = basic mode
        transfer done and table empty interrupt = enable

        Transfer count = PDMA_TEST_LENGTH
        Transfer width = 32 bits(one word)
        Source address = au8DestArray0
        Source address increment size = 32 bits(one word)
        Destination address = au8DestArray1
        Destination address increment size = 32 bits(one word)
        Transfer type = burst transfer

        Total transfer length = PDMA_TEST_LENGTH * 32 bits
    ------------------------------------------------------------------------------------------------------*/
    DMA_DESC[1].ctl =
        ((PDMA_TEST_LENGTH - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | /* Transfer count is PDMA_TEST_LENGTH */ \
        PDMA_WIDTH_32 |   /* Transfer width is 32 bits(one word) */ \
        PDMA_SAR_INC |    /* Source increment size is 32 bits(one word) */ \
        PDMA_DAR_INC |    /* Destination increment size is 32 bits(one word) */ \
        PDMA_REQ_BURST |  /* Transfer type is burst transfer type */ \
        PDMA_BURST_128 |  /* Burst size is 128. No effect in single transfer type */ \
        PDMA_OP_BASIC;    /* Operation mode is basic mode */

    DMA_DESC[1].src = u32Dst0;
    DMA_DESC[1].dest = u32Dst1;
    DMA_DESC[1].offset = 0; /* No next operation table. No effect in basic mode */


    /* Generate a software request to trigger transfer with PDMA channel 1 */
    PDMA_Trigger(PDMA, PDMA_CH);

    /* Waiting for transfer done */
    while (PDMA_IS_CH_BUSY(PDMA, PDMA_CH));

    for (i = 0; i < sizeof(au8DestArray0) / sizeof(au8DestArray0[0]); i++)
    {
        if (au8DestArray0[i] != au8SrcArray[i])
        {
            printf("Data is not match [%d][Src]0x%x, [Des]0x%x\n", i, au8SrcArray[i], au8DestArray0[i]);
        }

        if (au8DestArray1[i] != au8SrcArray[i])
        {
            printf("Data is not match [%d][Src]0x%x, [Des]0x%x\n", i, au8SrcArray[i], au8DestArray1[i]);
        }
    }

    printf("test done...\n");

    /* Close Channel 1 */
    PDMA_Close(PDMA);

    while (1);

}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

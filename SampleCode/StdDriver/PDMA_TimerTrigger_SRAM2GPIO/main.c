/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 18/05/31 10:00a $
 * @brief    Demonstrate how to use PDMA channel 1 to transfer data from SRAM to GPIO based on 1 kHz timer trigger.
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define dPDMA_TEST_LENGTH 256
#define UART0_Baudrate 115200
#define TIMER1_Freq 1000
#define PDMAchannel 1

uint8_t g_au8SrcArray[dPDMA_TEST_LENGTH];
uint32_t volatile g_u32IsTestOver = 0;//PDMA software flag - 0: Busy/Idle , 1 = Done ,2 = Abort

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

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable TMR0 clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Enable PDMA clock */
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) \
                    | (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

int main()
{
    uint32_t i;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, UART0_Baudrate);

    printf("+----------------------------------------------------------------------------+ \n");
    printf("|    PDMA transfer from RAM to GPIO Port A Sample Code                       | \n");
    printf("|    TIMER trigger : 1 kHz                                                   | \n");
    printf("|    PA.0 outputs 500 Hz signal based SRAM data and TIMER trigger setting    | \n");
    printf("|    PA.1 outputs 250 Hz signal based SRAM data and TIMER trigger setting    | \n");
    printf("|    PA.2 ~ 7 also ouput corresponding signal                                | \n");
    printf("+----------------------------------------------------------------------------+ \n");

    /* Create source pattern for PDMA transfer */
    for(i=0; i<dPDMA_TEST_LENGTH; i++)
    {
        g_au8SrcArray[i]=i;
    }

    /* Set Port A as output mode */
    GPIO_SetMode(PA,0xFFFF,GPIO_MODE_OUTPUT);

    /*------------------------------------------------------------------------------------------------------

        PDMA transfer configuration:

        Channel = 1
        Operation mode = basic mode
        Request source = PDMA_TMR0 (Timer0 trigger)
        transfer done and table empty interrupt = enable

        Transfer count = dPDMA_TEST_LENGTH
        Transfer width = 8 bits(one byte)
        Source address = au8SrcArray
        Source address increment size = 8 bits(one byte)
        Destination address = au8DestArray
        Destination address increment size = Fixed
        Transfer type = single transfer

        Total transfer length = dPDMA_TEST_LENGTH * 8 bits
    ------------------------------------------------------------------------------------------------------*/

    /* Open Channel 1 */
    PDMA_Open(PDMA, 1 << PDMAchannel);
    /* Transfer count is PDMA_TEST_LENGTH, transfer width is 8 bits(one byte) */
    PDMA_SetTransferCnt(PDMA, PDMAchannel, PDMA_WIDTH_8, dPDMA_TEST_LENGTH);
    /* Set source address is au8SrcArray, Source increment size is 8 bits(one byte), destination address is PA->DOUT (no increment)*/
    PDMA_SetTransferAddr(PDMA, PDMAchannel, (uint32_t)g_au8SrcArray, PDMA_SAR_INC, (uint32_t) &(PA->DOUT), PDMA_DAR_FIX);
    /* Request source is timer 0 */
    PDMA_SetTransferMode(PDMA, PDMAchannel, PDMA_TMR0, FALSE,(uint32_t) NULL);
    /* Transfer type is burst transfer and burst size is 4 */
    PDMA_SetBurstType(PDMA, PDMAchannel, PDMA_REQ_SINGLE,(uint32_t) NULL);

    /* Enable interrupt */
    PDMA_EnableInt(PDMA, PDMAchannel, PDMA_INT_TRANS_DONE);
    /* Enable NVIC for PDMA */
    NVIC_EnableIRQ(PDMA_IRQn);
    /* Clear PDMA software flag */
    g_u32IsTestOver = 0;

    // Set timer frequency to 1 kHz
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, TIMER1_Freq);
    // Select Timer 0 as PDMA trigger source
    TIMER_SetTriggerTarget(TIMER0, TIMER_TRG_TO_PDMA);
    // Start Timer 0
    TIMER_Start(TIMER0);

    while(1)
    {
        /* Waiting for transfer done */
        while (g_u32IsTestOver == 0);

        /* Check transfer result */
        if (g_u32IsTestOver == 1)
            printf("Transfer done...\n");
        else if (g_u32IsTestOver == 2)
            printf("Transfer abort...\n");

        /* Clear PDMA software flag */
        g_u32IsTestOver = 0;

        /* Transfer count is PDMA_TEST_LENGTH, transfer width is 8 bits(one byte) */
        PDMA_SetTransferCnt(PDMA, PDMAchannel, PDMA_WIDTH_8, dPDMA_TEST_LENGTH);
        /* Request source is timer 0 */
        PDMA_SetTransferMode(PDMA, PDMAchannel, PDMA_TMR0, FALSE, 0);
    }
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

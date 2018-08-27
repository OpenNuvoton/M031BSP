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
#define TIMER0_Freq 1000
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

    /* Enable HIRC */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) != CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(1);

    /* Enable UART0 peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Enable TIMER peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_TMR0CKEN_Msk ;
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_TMR0SEL_Msk) | CLK_CLKSEL1_TMR0SEL_HIRC;

    /* Enable PDMA clock source */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;



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
/*----------------------------------------------------------------------*/
/* Init UART0                                                           */
/*----------------------------------------------------------------------*/
void UART0_Init(void)
{
    /* Reset UART0 */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

int main()
{
    uint32_t i;
    uint32_t u32Prescale;
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART0_Init();

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
    for(i=0; i<=GPIO_MODE_MODE15_Pos; i+=2)
    {
        PA->MODE |= (PA->MODE&(~GPIO_MODE_MODE0_Pos<<i))| (GPIO_MODE_OUTPUT<<i);
    }

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


    /* Open PDMA Channel 1 based on PDMAchannel setting*/
    PDMA->CHCTL |= (1 << PDMAchannel);

    /* transfer width is half word(16 bit) and transfer count is ADCDatalenght+1 */
    PDMA->DSCT[PDMAchannel].CTL = ((dPDMA_TEST_LENGTH-1) << PDMA_DSCT_CTL_TXCNT_Pos) | /* Transfer count is PDMA_TEST_LENGTH */ \
                                  PDMA_WIDTH_8 |  /* Transfer width is 8 bits(one byte) */ \
                                  PDMA_SAR_INC |   /* Source increment size is 8 bits(one byte)*/ \
                                  PDMA_DAR_FIX |   /* Destination address size is fixed address  */ \
                                  PDMA_REQ_SINGLE | /* Transfer type is Single transfer type */ \
                                  0 |   /*Burst Size: No effect in single transfer type */ \
                                  PDMA_OP_BASIC;   /* Operation mode is basic mode */
    /* Configure source address */
    PDMA->DSCT[PDMAchannel].SA = (uint32_t)g_au8SrcArray;

    /* Configure destination address */
    PDMA->DSCT[PDMAchannel].DA = (uint32_t)&(PA->DOUT);

    /* Configure PDMA channel 1 as request source as TMR0 */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC1_Msk) | (PDMA_TMR0 << PDMA_REQSEL0_3_REQSRC1_Pos);

    /* Enable interrupt */
    PDMA->INTEN |= (1 << PDMAchannel);

    /* Enable NVIC for PDMA */
    NVIC_EnableIRQ(PDMA_IRQn);

    /* Clear PDMA software flag */
    g_u32IsTestOver = 0;



    /* Open Timer0 in period mode and Set timer frequency to 1 kHz*/
    u32Prescale=1;
    TIMER0->CTL = TIMER_PERIODIC_MODE | (u32Prescale-1);
    TIMER0->CMP = (__HIRC/u32Prescale) / TIMER0_Freq;

    // Select Timer 0 as PDMA trigger source
    TIMER0->CTL |= TIMER_TRG_TO_PDMA;
    // Start Timer 0
    TIMER0->CTL |= TIMER_CTL_CNTEN_Msk;

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
        PDMA->DSCT[PDMAchannel].CTL = (PDMA->DSCT[PDMAchannel].CTL &(~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_TXWIDTH_Msk)))\
                                      | (((dPDMA_TEST_LENGTH-1) << PDMA_DSCT_CTL_TXCNT_Pos))  /* Transfer count is PDMA_TEST_LENGTH */ \
                                      |   PDMA_WIDTH_8   /* Transfer width is 8 bits(one byte) */\
                                      | PDMA_OP_BASIC  /* Operation mode is basic mode */;


        /* Request source is timer 0 */
        PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC1_Msk) | (PDMA_TMR0 << PDMA_REQSEL0_3_REQSRC1_Pos);

    }
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

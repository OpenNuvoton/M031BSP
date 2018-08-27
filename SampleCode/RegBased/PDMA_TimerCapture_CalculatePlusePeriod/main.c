/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demostrade timer capture function to measure frequency of external signal through PDMA transfer.
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define dPDMA_TEST_LENGTH 20
#define PDMAchannel 1
#define TIMER0_Freq 2000000
#define TIMER3_Freq 2000

volatile uint32_t g_u32IsTestOver = 0;//PDMA software flag - 0: Busy/Idle , 1 = Done ,2 = Abort
volatile uint32_t g_au32CAPValue[dPDMA_TEST_LENGTH];

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


void TMR1_IRQHandler(void)
{
    if(TIMER_GetCaptureIntFlag(TIMER1) == 1)
    {
        /* Clear Timer1 capture trigger interrupt flag */
        TIMER_ClearCaptureIntFlag(TIMER1);
    }
}

void SYS_Init(void)
{
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

    /* Enable PDMA clock source */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;


    /* Enable TIMER peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_TMR0CKEN_Msk | CLK_APBCLK0_TMR1CKEN_Msk | CLK_APBCLK0_TMR3CKEN_Msk;
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~(CLK_CLKSEL1_TMR0SEL_Msk|CLK_CLKSEL1_TMR1SEL_Msk|CLK_CLKSEL1_TMR3SEL_Msk)) \
                   | CLK_CLKSEL1_TMR0SEL_HIRC|CLK_CLKSEL1_TMR1SEL_HIRC|CLK_CLKSEL1_TMR3SEL_HIRC;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))           \
                    | (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set multi-function pins for Timer0/Timer3 toggle-output pin */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB5MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk)) \
                    | (SYS_GPB_MFPL_PB5MFP_TM0 | SYS_GPB_MFPL_PB2MFP_TM3);

    /* Set multi-function pin for Timer1 external capture pin */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB14MFP_Msk)) \
                    | SYS_GPB_MFPH_PB14MFP_TM1_EXT;

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

int main(void)
{
    uint32_t u32InitCount;
    uint32_t u32CAPDiff;
    uint32_t u32Prescale;
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------+\n");
    printf("|    Timer1 Capture Counter Sample Code    |\n");
    printf("+------------------------------------------+\n\n");

    printf("# Timer0 Settings:\n");
    printf("    - Clock source is HIRC\n");
    printf("    - Time-out frequency is 2 MHz\n");
    printf("    - Toggle-output mode and frequency is 1 MHz\n");
    printf("# Timer3 Settings:\n");
    printf("    - Clock source is HIRC\n");
    printf("    - Time-out frequency is 2 kHz\n");
    printf("    - Toggle-output mode and frequency is 1 kHz\n");
    printf("# Timer1 Settings:\n");
    printf("    - Clock source is HIRC              \n");
    printf("    - Continuous counting mode          \n");
    printf("    - Compared value is 0xFFFFFF        \n");
    printf("    - External capture mode enable      \n");
    printf("# Connect TM3(PB.2) or TM0(PB.5)toggle-output pin to TM1_EXT(PB.14) external capture pin.\n\n");

    /* Open Timer0 in toggle-output mode and toggle-output frequency is 1 MHz*/
    u32Prescale=1;
    TIMER0->CTL = (TIMER0->CTL&(~(TIMER_CTL_OPMODE_Msk|TIMER_CTL_PSC_Msk)))| TIMER_TOGGLE_MODE |(u32Prescale-1);
    TIMER0->CMP = (__HIRC/u32Prescale) / TIMER0_Freq;

    /* Open Timer3 in toggle-output mode and toggle-output frequency is 1 kHz */
    TIMER3->CTL = (TIMER3->CTL&(~(TIMER_CTL_OPMODE_Msk|TIMER_CTL_PSC_Msk)))| TIMER_TOGGLE_MODE |(u32Prescale-1);
    TIMER3->CMP = (__HIRC/u32Prescale) / TIMER3_Freq;

    /* Enable Timer1 external capture function */
    TIMER1->CTL = (TIMER1->CTL&(~(TIMER_CTL_OPMODE_Msk|TIMER_CTL_PSC_Msk)))| TIMER_CONTINUOUS_MODE |(u32Prescale-1);
    TIMER1->CMP = TIMER_CMP_MAX_VALUE;
    TIMER1->EXTCTL = (TIMER1->EXTCTL & ~(TIMER_EXTCTL_CAPFUNCS_Msk | TIMER_EXTCTL_CAPEDGE_Msk)) |
                     TIMER_CAPTURE_FREE_COUNTING_MODE | TIMER_CAPTURE_FALLING_EDGE | TIMER_EXTCTL_CAPEN_Msk;
    TIMER1->CTL = (TIMER1->CTL & ~TIMER_CTL_TRGSSEL_Msk) | TIMER_TRGSRC_CAPTURE_EVENT;
    TIMER1->CTL |= TIMER_CTL_TRGPDMA_Msk;



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

    /* transfer width is 8 bits(one byte) and transfer count is PDMA_TEST_LENGTH */
    PDMA->DSCT[PDMAchannel].CTL = ((dPDMA_TEST_LENGTH-1) << PDMA_DSCT_CTL_TXCNT_Pos) | /* Transfer count is PDMA_TEST_LENGTH */ \
                                  PDMA_WIDTH_32 |    /* Transfer width is 8 bits(one byte) */ \
                                  PDMA_SAR_FIX |    /* Source address size is fixed address */ \
                                  PDMA_DAR_INC |    /* Destination increment size is depended on TXWIDTH selection  */  \
                                  PDMA_REQ_SINGLE | /* Transfer type is Single transfer type */ \
                                  0 |   /*Burst Size: No effect in single transfer type */\
                                  PDMA_OP_BASIC;   /* Operation mode is basic mode */

    /* Set source address is au8SrcArray, Source increment size is 8 bits(one byte), destination address is PA->DOUT (no increment)*/
    /* Configure source address */
    PDMA->DSCT[PDMAchannel].SA = (uint32_t)&(TIMER1->CAP);

    /* Configure destination address */
    PDMA->DSCT[PDMAchannel].DA = (uint32_t)g_au32CAPValue;

    /* Configure PDMA channel 1 as request source as PDMA_TMR1 */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC1_Msk) | (PDMA_TMR1 << PDMA_REQSEL0_3_REQSRC1_Pos);

    /* Enable interrupt */
    PDMA->INTEN |= (1 << PDMAchannel);

    /* Enable NVIC for PDMA */
    NVIC_EnableIRQ(PDMA_IRQn);

    /* Start Timer0, Timer3 and Timer1 counting */
    TIMER0->CTL |= TIMER_CTL_CNTEN_Msk;
    TIMER3->CTL |= TIMER_CTL_CNTEN_Msk;
    TIMER1->CTL |= TIMER_CTL_CNTEN_Msk;

    /* Waiting for transfer done */
    while (g_u32IsTestOver == 0)
    {
        TIMER_ClearCaptureIntFlag(TIMER1);//clear capture flag for new capture value.
    }

    /* Check transfer result */
    if (g_u32IsTestOver == 1)
        printf("PDMA trasnfer done...\n");
    else if (g_u32IsTestOver == 2)
        printf("PDMA trasnfer abort...\n");

    printf("# Period between two falling edge captured event is showed as following.\n");
    printf("    [%2d]: %4d. (1st captured value)\n", 0, g_au32CAPValue[0]);

    for(u32InitCount=1; u32InitCount<dPDMA_TEST_LENGTH; u32InitCount++)
    {
        u32CAPDiff = g_au32CAPValue[u32InitCount] - g_au32CAPValue[u32InitCount - 1];
        printf("    [%2d]: %4d. Diff: %d. Freq: %d Hz\n",u32InitCount, g_au32CAPValue[u32InitCount], u32CAPDiff, SystemCoreClock/u32CAPDiff);
    }

    /* Stop Timer0, Timer1 and Timer3 counting */
    TIMER0->CTL &= ~TIMER_CTL_CNTEN_Msk;
    TIMER1->CTL &= ~TIMER_CTL_CNTEN_Msk;
    TIMER3->CTL &= ~TIMER_CTL_CNTEN_Msk;
    while(1);

}

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

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable PDMA clock */
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Enable TIMER peripheral clock */
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_EnableModuleClock(TMR3_MODULE);

    /* Set TIMER clock source as HIRC */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_HIRC, 0);

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
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

int main(void)
{
    uint32_t u32InitCount;
    uint32_t u32CAPDiff;

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
    TIMER_Open(TIMER0, TIMER_TOGGLE_MODE, TIMER0_Freq);

    /* Open Timer3 in toggle-output mode and toggle-output frequency is 1 kHz */
    TIMER_Open(TIMER3, TIMER_TOGGLE_MODE, TIMER3_Freq);

    /* Enable Timer1 external capture function */
    TIMER_Open(TIMER1, TIMER_CONTINUOUS_MODE, 1);
    TIMER_SET_PRESCALE_VALUE(TIMER1, 0);
    TIMER_SET_CMP_VALUE(TIMER1, 0xFFFFFF);
    TIMER_EnableCapture(TIMER1, TIMER_CAPTURE_FREE_COUNTING_MODE, TIMER_CAPTURE_FALLING_EDGE);
    TIMER_SetTriggerSource(TIMER1, TIMER_TRGSRC_CAPTURE_EVENT);
    TIMER_SetTriggerTarget(TIMER1, TIMER_TRG_TO_PDMA);

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
    PDMA_SetTransferCnt(PDMA, PDMAchannel, PDMA_WIDTH_32, dPDMA_TEST_LENGTH);
    /* Set source address is au8SrcArray, Source increment size is 8 bits(one byte), destination address is PA->DOUT (no increment)*/
    PDMA_SetTransferAddr(PDMA, PDMAchannel, (uint32_t) &(TIMER1->CAP), PDMA_SAR_FIX, (uint32_t)g_au32CAPValue, PDMA_DAR_INC);
    /* Request source is timer 1 */
    PDMA_SetTransferMode(PDMA, PDMAchannel, PDMA_TMR1, FALSE,(uint32_t) NULL);
    /* Transfer type is burst transfer and burst size is 4 */
    PDMA_SetBurstType(PDMA, PDMAchannel, PDMA_REQ_SINGLE,(uint32_t) NULL);

    /* Enable interrupt */
    PDMA_EnableInt(PDMA, PDMAchannel, PDMA_INT_TRANS_DONE);
    /* Enable NVIC for PDMA */
    NVIC_EnableIRQ(PDMA_IRQn);

    /* Start Timer0, Timer3 and Timer1 counting */
    TIMER_Start(TIMER0);
    TIMER_Start(TIMER3);
    TIMER_Start(TIMER1);

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
    TIMER_Stop(TIMER0);
    TIMER_Stop(TIMER1);
    TIMER_Stop(TIMER3);
    while(1);

}

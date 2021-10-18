/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 9 $
 * $Date: 18/07/05 4:58p $
 * @brief    Capture the PWM0 Channel 0 waveform by PWM0 Channel 2, and use PDMA to transfer captured data.
 *           Frequency of PWM Channel 0 is 1 MHz to test maximum input frequency
 *           for PWM Capture function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#define PDMAchannel 0
/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define PLL_CLOCK       96000000

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define Transfer_Count 4
uint16_t g_au16Count[Transfer_Count];
volatile uint32_t g_u32IsTestOver = 0;


/**
 * @brief       PDMA IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle PDMA interrupt event
 */
void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);

    if(status & 0x1)    /* abort */
    {
        if(PDMA_GET_ABORT_STS(PDMA) & 0x1)
            g_u32IsTestOver = 2;
        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_ABTSTS_ABTIF0_Msk);
    }
    else if(status & 0x2)      /* done */
    {
        if(PDMA_GET_TD_STS(PDMA) & 0x1)
            g_u32IsTestOver = 1;
        PDMA_CLR_TD_FLAG(PDMA, PDMA_TDSTS_TDIF0_Msk);
    }
    else
        printf("unknown interrupt !!\n");
}

/*--------------------------------------------------------------------------------------*/
/* Capture function to calculate the input waveform information                         */
/* g_au16Count[4] : Keep the internal counter value when input signal rising / falling  */
/*               happens                                                                */
/*                                                                                      */
/* time    A    B     C     D                                                           */
/*           ___   ___   ___   ___   ___   ___   ___   ___                              */
/*      ____|   |_|   |_|   |_|   |_|   |_|   |_|   |_|   |_____                        */
/* index              0 1   2 3                                                         */
/*                                                                                      */
/* The capture internal counter down count from 0xFFFF, and reload to 0xFFFF after      */
/* input signal falling happens (Time B/C/D)                                            */
/*--------------------------------------------------------------------------------------*/
void CalPeriodTime(PWM_T *PWM, uint32_t u32Ch)
{
    uint16_t u16HighPeriod, u16LowPeriod, u16TotalPeriod;
    uint32_t u32TimeOutCount;

    /* setup timeout */
    u32TimeOutCount = SystemCoreClock;

    /* Wait PDMA interrupt (g_u32IsTestOver will be set at IRQ_Handler function) */
    while(g_u32IsTestOver == 0)
    {
        if(u32TimeOutCount == 0)
        {
            printf("\nSomething is wrong, please check if pin connection is correct. \n");
            while(1);
        }
        u32TimeOutCount--;
    }

    u16HighPeriod = g_au16Count[1] - g_au16Count[2] + 1;

    u16LowPeriod = 0xFFFF - g_au16Count[1];

    u16TotalPeriod = 0xFFFF - g_au16Count[2] + 1;

    printf("\nHigh Period = %d ns, Low Period = %d ns , Total Period = %d ns.\n",
           u16HighPeriod*1000/48, u16LowPeriod*1000/48, u16TotalPeriod*1000/48);
    printf("Frequency = %d Hz, Duty = %d %%.\n\n",
           48000000/u16TotalPeriod, u16HighPeriod*100/u16TotalPeriod );
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) != CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(1);

    /* Set core clock as PLL_CLOCK from PLL (no PLL in rev. B & C) */
    //CLK->PLLCTL = PLLCTL_SETTING;

    /* Waiting for PLL clock ready */
    //while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));
    //CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_PLL;


    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART0SEL_Msk)) | CLK_CLKSEL1_UART0SEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UART0DIV_Msk)) | CLK_CLKDIV0_UART0(1);

    /* Enable UART0 peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;


    /*---------------------------------------------------------------------------------------------------------*/
    /* PWM clock frequency configuration                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Select HCLK clock source as PLL and and HCLK clock divider as 2 */
    // CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(2);

    /* PWM clock frequency can be set equal or double to HCLK by choosing case 1 or case 2 */
    /* case 1.PWM clock frequency is set equal to HCLK: select PWM module clock source as PCLK */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_PWM0SEL_Msk) | CLK_CLKSEL2_PWM0SEL_PCLK0;

    /* case 2.PWM clock frequency is set double to HCLK: select PWM module clock source as PLL */
    //CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_PWM0SEL_Msk) | CLK_CLKSEL2_PWM0SEL_PLL;
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable PDMA module clock */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;

    /* Enable PWM0 module clock */
    CLK->APBCLK1 |= CLK_APBCLK1_PWM0CKEN_Msk;

    /* Reset PWM0 module */
    SYS->IPRST2 |= SYS_IPRST2_PWM0RST_Msk;
    SYS->IPRST2 &= ~SYS_IPRST2_PWM0RST_Msk;

    /* Reset PDMA module */
    SYS->IPRST0 |= SYS_IPRST0_PDMARST_Msk;
    SYS->IPRST0 &= ~SYS_IPRST0_PDMARST_Msk;

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set PB multi-function pins for PWM0 Channel 0 and 2 */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB5MFP_Msk)) |
                    SYS_GPB_MFPL_PB5MFP_PWM0_CH0;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB3MFP_Msk)) |
                    SYS_GPB_MFPL_PB3MFP_PWM0_CH2;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint8_t u8Option;

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("PWM0 clock is from %s\n", (CLK->CLKSEL2 & CLK_CLKSEL2_PWM0SEL_Msk) ? "CPU" : "PLL");
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          PWM Capture Sample Code                       |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use PWM0 channel 2 to capture the signal from PWM0 channel 0.\n");
    printf("  And the captured data is transferred by PDMA channel 0.\n");
    printf("  PWM0 channel 0 is 1 MHz, duty is 50 %%.\n");
    printf("  PWM0 channel 2 counter period is about 20.8(1/48,000,000) ns\n");
    printf("  I/O configuration:\n");
    printf("    PWM0 channel 2(PB.3) <--> PWM0 channel 0(PB.5)\n\n");
    printf("Use PWM0 Channel 2(PB.3) to capture the PWM0 Channel 0(PB.5) Waveform\n");

    /*--------------------------------------------------------------------------------------*/
    /* Set the PWM0 Channel 0 as PWM output function.                                       */
    /*--------------------------------------------------------------------------------------*/

    /* Assume PWM output frequency is 1 MHz and duty ratio is 50%, user can calculate PWM settings by follows.
         duty ratio = (CMR+1)/(CNR+1)
         cycle time = CNR+1
         High level = CMR
         PWM clock source frequency from HIRC is 48,000,000
         (CNR+1) = PWM clock source frequency/prescaler/PWM output frequency
                         = 48,000,000/1/48 = 1,000,000
         (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
         CNR = 47
         duty ratio = 50% ==> CMR/(CNR+1) = 50%
         CMR = 24
         Prescale value is 0 : prescaler = 1
    */

    /* PWM0 channel 0 frequency prescaler to 1 */
    PWM_SET_PRESCALER(PWM0, 0, 1-1);

    /* PWM0 channel 0 frequency period to 47 */
    PWM_SET_CNR(PWM0, 0, 47);

    /* PWM0 channel 0 frequency comparator to 24 */
    PWM_SET_CMR(PWM0, 0, 24);

    /* PWM0 channel 0 is edge-aligned and down counter type */
    PWM0->CTL1 = (PWM0->CTL1 & ~PWM_CTL1_CNTTYPE0_Msk) | PWM_DOWN_COUNTER;

    /* ZeroLevel: Low, CmpUpLevel: nothing, PeriodLevel: nothing, CmpDownLevel: High */
    PWM_SET_OUTPUT_LEVEL(PWM0, PWM_CH_0_MASK, PWM_OUTPUT_LOW, PWM_OUTPUT_HIGH, PWM_OUTPUT_NOTHING, PWM_OUTPUT_HIGH);

    /* Enable PWM Output path for PWM0 channel 0 */
    PWM0->POEN |= PWM_CH_0_MASK;

    /* Enable Timer for PWM0 channel 0 */
    PWM0->CNTEN |= PWM_CH_0_MASK;

    /*--------------------------------------------------------------------------------------*/
    /* Configure PDMA peripheral mode form PWM to memory                                    */
    /*--------------------------------------------------------------------------------------*/
    /* Open Channel 0 */
    PDMA->CHCTL |= (1 << PDMAchannel);

    /* Configure source address */
    PDMA->DSCT[PDMAchannel].SA = (uint32_t)&PWM0->PDMACAP2_3;

    /* Configure destination address */
    PDMA->DSCT[PDMAchannel].DA = (uint32_t)&g_au16Count[0];

    /* Configure PDMA channel 0 as request source as PDWM channel 2 */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC0_Pos) | (PDMA_PWM0_P2_RX << PDMA_REQSEL0_3_REQSRC0_Pos);

    /* Set source address as PWM capture channel PDMA register(no increment) and destination address as g_au16Count array(increment) */
    PDMA->DSCT[PDMAchannel].CTL = ((Transfer_Count-1)<< PDMA_DSCT_CTL_TXCNT_Pos) | /* Transfer count is PDMA_TEST_LENGTH */ \
                                  PDMA_WIDTH_16   |  /* Transfer width is 16 bits(half word) */ \
                                  PDMA_SAR_FIX    |  /* Source address size is fixed address */ \
                                  PDMA_DAR_INC    |  /* Destination increment size is depended on TXWIDTH selection  */ \
                                  PDMA_REQ_SINGLE |  /* Transfer type is Single transfer type */ \
                                  0               |  /* Burst Size: No effect in single transfer type */ \
                                  PDMA_OP_BASIC;   /* Operation mode is basic mode */


    /* Enable interrupt */
    PDMA->INTEN |= (1 << PDMAchannel);

    /* Enable NVIC for PDMA */
    NVIC_EnableIRQ(PDMA_IRQn);

    /*--------------------------------------------------------------------------------------*/
    /* Set the PWM0 channel 2 for capture function                                          */
    /*--------------------------------------------------------------------------------------*/
    /* (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
         CNR = 0xFFFF
         (Note: In capture mode, user should set CNR to 0xFFFF to increase capture frequency range.)

         Capture unit time = 1/Capture clock source frequency/prescaler
         20.8ns = 1/48,000,000/1
    */

    /* Set PWM0 channel 2 capture configuration */
    /*Set counter as down count*/
    PWM0->CTL1 = (PWM0->CTL1 & ~PWM_CTL1_CNTTYPE2_Msk) | (PWM_DOWN_COUNTER << PWM_CTL1_CNTTYPE2_Pos);

    /*Set PWM0 channel 2 Timer period*/
    PWM_SET_CNR(PWM0, 2, 0xFFFF);

    /* Enable falling capture reload */
    PWM0->CAPCTL |= PWM_CAPCTL_FCRLDEN2_Msk;

    while(1)
    {
        g_u32IsTestOver = 0;

        printf("[0] Trigger PWM0 Channel 2 capture function\n");
        printf("[Other] Exit \n");
        printf("Please input key:");
        u8Option = getchar();

        if(u8Option != '0')
        {
            printf("Exit\n");
            break;

        }

        /*--------------------------------------------------------------------------------------*/
        /* Configure PDMA transfer count and enable PDMA function                               */
        /*--------------------------------------------------------------------------------------*/
        /* Select PDMA request source as PWM RX(PWM0 channel 2 should be PWM0 pair 2) */
        PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC0_Pos) | (PDMA_PWM0_P2_RX << PDMA_REQSEL0_3_REQSRC0_Pos);

        /* Transfer width is half word(16 bit) and transfer count is 4 */
        PDMA->DSCT[PDMAchannel].CTL = (PDMA->DSCT[PDMAchannel].CTL &(~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_TXWIDTH_Msk))) |  \
                                      (Transfer_Count-1) << PDMA_DSCT_CTL_TXCNT_Pos |  \
                                      PDMA_WIDTH_16 | /* Transfer width is 16 bits(half word) */ \
                                      PDMA_OP_BASIC;  /* Operation mode is basic mode */

        /* Enable Capture Function for PWM0 channel 2 */
        PWM0->CAPCTL |= PWM_CAPCTL_CAPEN2_Msk;
        PWM0->CAPINEN |= PWM_CAPINEN_CAPINEN2_Msk;

        /* Enable Timer for PWM0 channel 2 */
        PWM0->CNTEN |= PWM_CNTEN_CNTEN2_Msk;

        /* Enable PDMA for PWM0 channel 2 capture function, and set capture order as falling first, */
        /* And select capture mode as both rising and falling to do PDMA transfer. */
        PWM0->PDMACTL = (PWM0->PDMACTL&(~(PWM_PDMACTL_CHSEL2_3_Msk | PWM_PDMACTL_CAPORD2_3_Msk | PWM_PDMACTL_CHEN2_3_Msk))) |\
                        (PWM_CAPTURE_PDMA_RISING_FALLING_LATCH <<(PWM_PDMACTL_CAPMOD2_3_Pos-1)) | /*select capture mode as both rising and falling to do PDMA transfer. */\
                        PWM_PDMACTL_CHEN2_3_Msk;

        /* Capture the Input Waveform Data */
        CalPeriodTime(PWM0, 2);

        /* Set loaded value as 0 for PWM0 channel 2 */
        PWM0->PERIOD[2] = 0;

        /* Clear Capture Interrupt flag for PWM0 channel 2 */
        PWM0->CAPIF = PWM_CAPIF_CFLIF2_Msk;

        /* Disable PDMA for PWM0 channel 2 capture function */
        PWM0->PDMACTL &= ~PWM_PDMACTL_CHEN2_3_Msk;

    }

    /*---------------------------------------------------------------------------------------------------------*/
    /* Stop PWM0 channel 2 (Recommended procedure method 1)                                                    */
    /* Set PWM Timer loaded value(Period) as 0. When PWM internal counter(CNT) reaches to 0, disable PWM Timer */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set loaded value as 0 for PWM0 channel 2 */
    PWM0->PERIOD[2] = 0;

    /* Wait until PWM0 channel 2 current counter reach to 0 */
    while((PWM0->CNT[2] & PWM_CNT_CNT_Msk) != 0);

    /* Disable Timer for PWM0 channel 2 */
    PWM0->CNTEN &= ~PWM_CNTEN_CNTEN2_Msk;

    /* Stop PWM counter */
    PWM0->PERIOD[0] = 0;

    /* Wait until PWM0 channel 0 current counter reach to 0 */
    while((PWM0->CNT[0] & PWM_CNT_CNT_Msk) != 0);

    /* Disable output of PWM0 channel 0 */
    PWM0->CNTEN &= ~PWM_CNTEN_CNTEN0_Msk;

    PDMA->CHCTL = 0UL;

    while(1);
}

/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to transfer 1.822 Msps conversion data from ADC to SRAM by PDMA
 *           when ADC clock source is selected to HXT that is connected to 32 Mhz crystal.
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define ADCDatalenght 16
#define ADCextendSampling 0
#define PDMAchannel 1

volatile uint32_t g_u32ADCounter = 0;
volatile uint32_t g_u32IsTestOver = 0;//PDMA software flag - 0: Busy/Idle , 1 = Done ,2 = Abort
volatile uint32_t g_u32ResultSum = 0;
volatile uint32_t g_u32ADChannel = 0;
int16_t  g_i32ConversionData[ADCDatalenght+1] = {0};


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HXT (4~32 MHz) */
    CLK->PWRCTL |= (CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HIRC, HXT clock ready */
    while((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) != CLK_STATUS_HIRCSTB_Msk);
    while((CLK->STATUS & CLK_STATUS_HXTSTB_Msk) != CLK_STATUS_HXTSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(1);

    /* Switch UART0 clock source to HIRC */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART0SEL_Msk) | CLK_CLKSEL1_UART0SEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_UART0DIV_Msk) | CLK_CLKDIV0_UART0(1);

    /* Enable UART0 peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* ADC clock source is HXT 12MHz, set divider to 8, ADC clock is 12/8 MHz */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_ADCSEL_Msk) | CLK_CLKSEL2_ADCSEL_HXT;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_ADCDIV_Msk) | CLK_CLKDIV0_ADC(1);

    /* Enable ADC module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_ADCCKEN_Msk;

    /* Enable PDMA clock source */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH&~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) \
                    | (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Configure the GPB0 - GPB3 ADC analog input pins.  */
    SYS->GPB_MFPL = (SYS->GPB_MFPL &~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk)) \
                    | (SYS_GPB_MFPL_PB0MFP_ADC0_CH0 | SYS_GPB_MFPL_PB1MFP_ADC0_CH1) | (SYS_GPB_MFPL_PB2MFP_ADC0_CH2 | SYS_GPB_MFPL_PB3MFP_ADC0_CH3);

    /* Set PB.0 ~ PB.3 to input mode */
    PB->MODE &= ~(GPIO_MODE_MODE0_Msk | GPIO_MODE_MODE1_Msk | GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Disable the PB0 ~ PB3 digital input path to avoid the leakage current. */
    PB->DINOFF |= ((BIT0|BIT1|BIT2|BIT3)<<GPIO_DINOFF_DINOFF0_Pos);

    /* Lock protected registers */
    SYS_LockReg();
}

void PDMA_Init()
{
    /* Configure PDMA peripheral mode form ADC to memory */
    /* Open PDMA Channel 1 based on PDMAchannel setting*/
    PDMA->CHCTL |= (1 << PDMAchannel);

    /* transfer width is half word(16 bit) and transfer count is ADCDatalenght+1 */
    PDMA->DSCT[PDMAchannel].CTL = ((ADCDatalenght) << PDMA_DSCT_CTL_TXCNT_Pos) | /* Transfer count is PDMA_TEST_LENGTH */ \
                                  PDMA_WIDTH_16 |  /* Transfer width is 16 bits(half word) */ \
                                  PDMA_SAR_FIX |   /* Source increment size is 32 bits(one word) */ \
                                  PDMA_DAR_INC |   /* Destination increment size is 32 bits(one word) */ \
                                  PDMA_REQ_SINGLE | /* Transfer type is Single transfer type */ \
                                  0 |   /* No effect in single transfer type */ \
                                  PDMA_OP_BASIC;   /* Operation mode is basic mode */


    /* Configure source address */
    PDMA->DSCT[PDMAchannel].SA = (uint32_t)&ADC->ADPDMA;

    /* Configure destination address */
    PDMA->DSCT[PDMAchannel].DA = (uint32_t)g_i32ConversionData;

    /* Configure PDMA channel 1 as request source as ADC RX */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC1_Msk) | (PDMA_ADC_RX << PDMA_REQSEL0_3_REQSRC1_Pos);

    /* Enable interrupt */
    PDMA->INTEN |= (1 << PDMAchannel);

    /* Enable NVIC for PDMA */
    NVIC_EnableIRQ(PDMA_IRQn);

}

void ReloadPDMA()
{
    /* transfer width is half word(16 bit) and transfer count is ADCDatalenght+1 */
    PDMA->DSCT[PDMAchannel].CTL = ((ADCDatalenght) << PDMA_DSCT_CTL_TXCNT_Pos) | /* Transfer count is PDMA_TEST_LENGTH */ \
                                  PDMA_WIDTH_16 |  /* Transfer width is 32 bits(one word) */ \
                                  PDMA_SAR_FIX |   /* Source increment size is 32 bits(one word) */ \
                                  PDMA_DAR_INC |   /* Destination increment size is 32 bits(one word) */ \
                                  PDMA_REQ_SINGLE | /* Transfer type is burst transfer type */ \
                                  0 |   /* No effect in single transfer type */ \
                                  PDMA_OP_BASIC;   /* Operation mode is basic mode */

    /* Select PDMA request source as ADC RX */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC1_Msk) | (PDMA_ADC_RX << PDMA_REQSEL0_3_REQSRC1_Pos);
}

void ADC_FunctionTest()
{
    uint8_t  u8Option;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|   High speed ADC conversion rate with PDMA transfer                  |\n");
    printf("|   ADC clock source -> HXT = 32MHz                                    |\n");
    printf("|   ADC extended sampling time = 0                                     |\n");
    printf("|   ADC conversion time = 17 + ADC extended sampling time = 17         |\n");
    printf("|   ADC conversion rate = 32 MHz / 17 = 1.882MSPS                      |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("\nIn this test, software will get 16 conversion results from the specified channel.\n");
    printf("\nEnable ADC Power and then calibrate ADC.\n");


    /* Enable ADC converter */
    ADC->ADCR |= ADC_ADCR_ADEN_Msk;

    /* Do calibration for ADC to decrease the effect of electrical random noise. */
    ADC->ADCALSTSR |= ADC_ADCALSTSR_CALIF_Msk;  /* Clear Calibration Finish Interrupt Flag */
    ADC->ADCALR |= ADC_ADCALR_CALEN_Msk;        /* Enable Calibration function */
    ADC_START_CONV(ADC);                        /* Start to calibration */
    while((ADC->ADCALSTSR & ADC_ADCALSTSR_CALIF_Msk) != ADC_ADCALSTSR_CALIF_Msk);   /* Wait calibration finish */

    /*Wait for ADC internal power ready*/
    CLK_SysTickDelay(10000);

    /* Set input mode as single-end, and CONTINUOUS mode*/
    ADC->ADCR = (ADC->ADCR & ~(ADC_ADCR_DIFFEN_Msk | ADC_ADCR_ADMD_Msk)) |
                (ADC_ADCR_DIFFEN_SINGLE_END | ADC_ADCR_ADMD_CONTINUOUS);

    /* Set extend sampling time based on external resistor value.*/
    ADC->ESMPCTL = (ADC->ESMPCTL & ~ADC_ESMPCTL_EXTSMPT_Msk) |
                   (ADCextendSampling << ADC_ESMPCTL_EXTSMPT_Pos);

    while(1)
    {
        /* reload PDMA configuration for next transmission */
        ReloadPDMA();

        printf("\nSelect input ADC channel from 0 to 3:\n");
        printf("  [0] Single end input channel 0\n");
        printf("  [1] Single end input channel 1\n");
        printf("  [2] Single end input channel 2\n");
        printf("  [3] Single end input channel 3\n");
        printf("  Other keys: exit ADC + PDMA test\n");
        printf("  Please input key.\n");
        u8Option = getchar();
        if(u8Option == '0')
            g_u32ADChannel = 0;
        else if(u8Option == '1')
            g_u32ADChannel = 1;
        else if(u8Option == '2')
            g_u32ADChannel = 2;
        else if(u8Option == '3')
            g_u32ADChannel = 3;
        else
            return ;

        /* Select ADC input channel */
        ADC->ADCHER = (ADC->ADCHER & ~ADC_ADCHER_CHEN_Msk) |(0x1 << g_u32ADChannel);

        /* ADC enable PDMA transfer */
        ADC_ENABLE_PDMA(ADC);

        /* Start ADC conversion */
        ADC_START_CONV(ADC);

        /* Wait PDMA interrupt (g_u32IsTestOver will be set at IRQ_Handler function) */
        while(g_u32IsTestOver == 0);

        /* Check transfer result */
        if (g_u32IsTestOver == 1)
            printf("PDMA trasnfer done...\n");
        else if (g_u32IsTestOver == 2)
            printf("PDMA trasnfer abort...\n");

        /* Clear g_u32IsTestOver software flag */
        g_u32IsTestOver = 0;

        /* Stop ADC conversion */
        ADC_STOP_CONV(ADC);

        /* Disable PDMA function of ADC */
        ADC_DISABLE_PDMA(ADC);

        /* Calculate average and print ADC result except first sampling data result that belongs to preivous channel*/
        g_u32ResultSum = 0;
        printf("\nConversion result of channel %d:\n",g_u32ADChannel);
        for(g_u32ADCounter = 1; (g_u32ADCounter) < (ADCDatalenght+1); g_u32ADCounter++)
        {
            g_u32ResultSum += g_i32ConversionData[g_u32ADCounter];
            printf("%2d:0x%X (%d)\n", g_u32ADCounter, g_i32ConversionData[g_u32ADCounter], g_i32ConversionData[g_u32ADCounter]);
        }
        g_u32ResultSum /= (ADCDatalenght);
        printf("Average Result : 0x%X (%d)\n",g_u32ResultSum, g_u32ResultSum);
    }
}

void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);

    if(status & PDMA_INTSTS_ABTIF_Msk)    /* abort */
    {
        if(PDMA_GET_ABORT_STS(PDMA) & PDMA_ABTSTS_ABTIF1_Msk)
            g_u32IsTestOver = 2;
        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_ABTSTS_ABTIF1_Msk);
    }
    else if(status & PDMA_INTSTS_TDIF_Msk)      /* done */
    {
        if(PDMA_GET_TD_STS(PDMA) & PDMA_TDSTS_TDIF1_Msk)
            g_u32IsTestOver = 1;
        PDMA_CLR_TD_FLAG(PDMA, PDMA_TDSTS_TDIF1_Msk);
    }
    else
        printf("unknown PDMA interrupt !!\n");
}

/*----------------------------------------------------------------------*/
/* Init UART0                                                           */
/*----------------------------------------------------------------------*/
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

int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init PDMA for ADC */
    PDMA_Init();

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* ADC function test */
    ADC_FunctionTest();

    /* Disable ADC IP clock */
    CLK->APBCLK0 &= ~(CLK_APBCLK0_ADCCKEN_Msk);

    /* Disable PDMA IP clock */
    CLK->AHBCLK &= ~(CLK_AHBCLK_PDMACKEN_Msk);

    /* Disable PDMA Interrupt */
    NVIC_DisableIRQ(PDMA_IRQn);

    printf("Exit ADC sample code\n");

    while(1);
}

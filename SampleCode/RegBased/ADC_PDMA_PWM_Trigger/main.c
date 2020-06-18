/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to trigger ADC by PWM and transfer conversion data by PDMA.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag, g_u32COVNUMFlag = 0;
volatile uint32_t g_u32IsTestOver = 0;
int16_t  g_i32ConversionData[6] = {0};
uint32_t g_u32SampleModuleNum = 0;


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

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Switch UART0 clock source to HIRC */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART0SEL_Msk) | CLK_CLKSEL1_UART0SEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_UART0DIV_Msk) | CLK_CLKDIV0_UART0(1);

    /* Enable UART0 and ADC peripheral clock */
    CLK->APBCLK0 |= (CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_ADCCKEN_Msk);

    /* Enable PWM0 module clock */
    CLK->APBCLK1 |= CLK_APBCLK1_PWM0CKEN_Msk;

    /* Select PWM0 module clock source as PCLK0 */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_PWM0SEL_Msk) | CLK_CLKSEL2_PWM0SEL_PCLK0;

    /* ADC clock source is PCLK1, set divider to 1 */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & ~CLK_CLKSEL2_ADCSEL_Msk) | CLK_CLKSEL2_ADCSEL_PCLK1;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_ADCDIV_Msk) | CLK_CLKDIV0_ADC(1);

    /* Enable PDMA clock source */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set PB.2 ~ PB.3 to input mode */
    PB->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);
    /* Configure the PB.2 - PB.3 ADC analog input pins. */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk)) |
                    (SYS_GPB_MFPL_PB2MFP_ADC0_CH2 | SYS_GPB_MFPL_PB3MFP_ADC0_CH3);
    /* Disable the GPB2 digital input path to avoid the leakage current. */
    PB->DINOFF |= ((BIT2|BIT3)<<GPIO_DINOFF_DINOFF0_Pos);

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


void PWM0_Init()
{
    /* Set PWM0 timer clock prescaler */
    *(__IO uint32_t *) (&(PWM0->CLKPSC[0])) = 0;

    /* Set up counter type */
    PWM0->CTL1 &= ~PWM_CTL1_CNTTYPE0_Msk;

    /* Set PWM0 timer duty */
    PWM0->CMPDAT[0] = 108;

    /* Set PWM0 timer period */
    PWM0->PERIOD[0] = 216;

    /* PWM period point trigger ADC enable */
    PWM0->ADCTS0 = (PWM0->ADCTS0 & ~(PWM_ADCTS0_TRGSEL0_Msk)) |
                   (PWM_ADCTS0_TRGEN0_Msk | PWM_TRIGGER_ADC_EVEN_PERIOD_POINT);

    /* Set output level at zero, compare up, period(center) and compare down of specified channel */
    {
        int i;
        for(i = 0; i < 6; i++)
        {
            if((BIT0) & (1 << i))
            {
                PWM0->WGCTL0 = ((PWM0->WGCTL0 & ~(3UL << (i << 1))) | (PWM_OUTPUT_HIGH << (i << 1)));
                PWM0->WGCTL0 = ((PWM0->WGCTL0 & ~(3UL << (PWM_WGCTL0_PRDPCTL0_Pos + (i << 1)))) | (PWM_OUTPUT_NOTHING << (PWM_WGCTL0_PRDPCTL0_Pos + (i << 1))));
                PWM0->WGCTL1 = ((PWM0->WGCTL1 & ~(3UL << (i << 1))) | (PWM_OUTPUT_LOW << (i << 1)));
                PWM0->WGCTL1 = ((PWM0->WGCTL1 & ~(3UL << (PWM_WGCTL1_CMPDCTL0_Pos + (i << 1)))) | (PWM_OUTPUT_NOTHING << (PWM_WGCTL1_CMPDCTL0_Pos + (i << 1))));
            }
        }
    }

    /* Enable output of PWM0 channel 0 */
    PWM0->POEN |= BIT0;
}

void PDMA_Init()
{
    /* Configure PDMA peripheral mode form EADC to memory */
    /* Open Channel 1 */
    PDMA->CHCTL |= BIT1;

    /* transfer width is half word(16 bit) and transfer count is 6 */
    PDMA->DSCT[1].CTL = (PDMA->DSCT[1].CTL & ~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_TXWIDTH_Msk)) |
                        (PDMA_WIDTH_16 | ((6 - 1UL) << PDMA_DSCT_CTL_TXCNT_Pos));

    /* Set source address as ADC data register(no increment) and destination address as g_i32ConversionData array(increment) */
    PDMA->DSCT[1].SA = (uint32_t)&ADC->ADDR[2];
    PDMA->DSCT[1].DA = (uint32_t)g_i32ConversionData;
    PDMA->DSCT[1].CTL = (PDMA->DSCT[1].CTL & ~(PDMA_DSCT_CTL_SAINC_Msk | PDMA_DSCT_CTL_DAINC_Msk)) |
                        (PDMA_SAR_FIX | PDMA_DAR_INC);

    /* Select PDMA request source as ADC RX */
    /* Set PDMA as single request type for ADC */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC1_Msk) | (PDMA_ADC_RX << PDMA_REQSEL0_3_REQSRC1_Pos);
    PDMA->DSCT[1].CTL = (PDMA->DSCT[1].CTL & ~(PDMA_DSCT_CTL_OPMODE_Msk | PDMA_DSCT_CTL_TXTYPE_Msk | PDMA_DSCT_CTL_BURSIZE_Msk)) |
                        (PDMA_OP_BASIC | PDMA_REQ_SINGLE | PDMA_BURST_4);

    PDMA->INTEN |= (1ul << 1);

    NVIC_EnableIRQ(PDMA_IRQn);
}

void ReloadPDMA()
{
    /* transfer width is half word(16 bit) and transfer count is 6 */
    PDMA->DSCT[1].CTL = (PDMA->DSCT[1].CTL & ~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_TXWIDTH_Msk)) |
                        (PDMA_WIDTH_16 | ((6 - 1UL) << PDMA_DSCT_CTL_TXCNT_Pos));

    /* Select PDMA request source as ADC RX */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC1_Msk) | (PDMA_ADC_RX << PDMA_REQSEL0_3_REQSRC1_Pos);
    PDMA->DSCT[1].CTL = (PDMA->DSCT[1].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | PDMA_OP_BASIC;
}

void ADC_FunctionTest()
{
    uint8_t  u8Option;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|   PWM trigger mode and transfer ADC conversion data by PDMA test     |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("\nIn this test, software will get 6 conversion result from the specified channel.\n");

    /* Enable ADC converter */
    ADC->ADCR |= ADC_ADCR_ADEN_Msk;

    /* Do calibration for ADC to decrease the effect of electrical random noise. */
    ADC->ADCALSTSR |= ADC_ADCALSTSR_CALIF_Msk;  /* Clear Calibration Finish Interrupt Flag */
    ADC->ADCALR |= ADC_ADCALR_CALEN_Msk;        /* Enable Calibration function */
    ADC_START_CONV(ADC);                        /* Start to calibration */
    while((ADC->ADCALSTSR & ADC_ADCALSTSR_CALIF_Msk) != ADC_ADCALSTSR_CALIF_Msk);   /* Wait calibration finish */

    while(1)
    {
        /* reload PDMA configuration for next transmission */
        ReloadPDMA();

        printf("Select input mode:\n");
        printf("  [1] Single end input (channel 2 only)\n");
        printf("  [2] Differential input (channel pair 1 only(channel 2 and 3))\n");
        printf("  Other keys: exit single mode test\n");
        u8Option = getchar();
        if(u8Option == '1')
        {
            /* Set input mode as single-end, Single mode, and select channel 2 */
            /* Configure the sample module and enable PWM0 trigger source */
            /* ADC enable PDMA transfer */
            ADC->ADCR = (ADC->ADCR & ~(ADC_ADCR_DIFFEN_Msk | ADC_ADCR_ADMD_Msk | ADC_ADCR_TRGS_Msk | ADC_ADCR_TRGCOND_Msk | ADC_ADCR_TRGEN_Msk)) |
                        (ADC_ADCR_DIFFEN_SINGLE_END | ADC_ADCR_ADMD_SINGLE | ADC_ADCR_TRGS_PWM | ADC_ADCR_TRGEN_Msk | ADC_ADCR_PTEN_Msk);
            ADC->ADCHER = (ADC->ADCHER & ~ADC_ADCHER_CHEN_Msk) | (BIT2);

            printf("Conversion result of channel 2:\n");

            /* Enable PWM0 channel 0 counter */
            PWM0->CNTEN |= PWM_CH_0_MASK;   /* PWM0 channel 0 counter start running. */

            while(1)
            {
                /* Wait PDMA interrupt (g_u32IsTestOver will be set at IRQ_Handler function) */
                while(g_u32IsTestOver == 0);
                break;
            }
            g_u32IsTestOver = 0;

            /* Disable PWM0 channel 0 counter */
            PWM0->CNTEN &= ~BIT0;   /* PWM0 counter stop running. */

            for(g_u32COVNUMFlag = 0; (g_u32COVNUMFlag) < 6; g_u32COVNUMFlag++)
                printf("                                0x%X (%d)\n", g_i32ConversionData[g_u32COVNUMFlag], g_i32ConversionData[g_u32COVNUMFlag]);
        }
        else if(u8Option == '2')
        {
            /* Set input mode as differential, Single mode, and select channel 2 */
            /* Configure the sample module and enable PWM0 trigger source */
            /* ADC enable PDMA transfer */
            ADC->ADCR = (ADC->ADCR & ~(ADC_ADCR_DIFFEN_Msk | ADC_ADCR_ADMD_Msk | ADC_ADCR_TRGS_Msk | ADC_ADCR_TRGCOND_Msk | ADC_ADCR_TRGEN_Msk)) |
                        (ADC_ADCR_DIFFEN_DIFFERENTIAL | ADC_ADCR_ADMD_SINGLE | ADC_ADCR_TRGS_PWM | ADC_ADCR_TRGEN_Msk | ADC_ADCR_PTEN_Msk);
            ADC->ADCHER = (ADC->ADCHER & ~ADC_ADCHER_CHEN_Msk) | (BIT2);

            printf("Conversion result of channel 2:\n");

            /* Enable PWM0 channel 0 counter */
            PWM0->CNTEN |= PWM_CH_0_MASK;   /* PWM0 channel 0 counter start running. */

            while(1)
            {
                /* Wait PDMA interrupt (g_u32IsTestOver will be set at IRQ_Handler function) */
                while(g_u32IsTestOver == 0);
                break;
            }
            g_u32IsTestOver = 0;

            /* Disable EPWM0 channel 0 counter */
            PWM0->CNTEN &= ~BIT0;   /* PWM0 counter stop running. */

            for(g_u32COVNUMFlag = 0; (g_u32COVNUMFlag) < 6; g_u32COVNUMFlag++)
                printf("                                0x%X (%d)\n", g_i32ConversionData[g_u32COVNUMFlag], g_i32ConversionData[g_u32COVNUMFlag]);
        }
        else
            return ;
    }
}

void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA->INTSTS;

    if(status & PDMA_INTSTS_ABTIF_Msk)    /* abort */
    {
        if(PDMA->ABTSTS & PDMA_ABTSTS_ABTIF1_Msk)
            g_u32IsTestOver = 2;
        PDMA->ABTSTS = PDMA_ABTSTS_ABTIF1_Msk;
    }
    else if(status & PDMA_INTSTS_TDIF_Msk)      /* done */
    {
        if(PDMA->TDSTS & PDMA_TDSTS_TDIF1_Msk)
            g_u32IsTestOver = 1;
        PDMA->TDSTS = PDMA_TDSTS_TDIF1_Msk;
    }
    else
        printf("unknown PDMA interrupt !!\n");
}


int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init PWM for ADC */
    PWM0_Init();

    /* Init PDMA for ADC */
    PDMA_Init();

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* ADC function test */
    ADC_FunctionTest();

    /* Disable ADC IP clock */
    CLK->APBCLK0 &= ~(CLK_APBCLK0_ADCCKEN_Msk);

    /* Disable PWM0 IP clock */
    CLK->APBCLK1 &= ~(CLK_APBCLK1_PWM0CKEN_Msk);

    /* Disable PDMA clock source */
    CLK->AHBCLK &= ~(CLK_AHBCLK_PDMACKEN_Msk);

    /* Disable PDMA Interrupt */
    NVIC_DisableIRQ(PDMA_IRQn);

    printf("Exit ADC sample code\n");

    while(1);
}

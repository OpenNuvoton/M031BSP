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
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable PWM0 module clock */
    CLK_EnableModuleClock(PWM0_MODULE);

    /* Select PWM0 module clock source as PCLK0 */
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL2_PWM0SEL_PCLK0, 0);

    /* Enable ADC module clock */
    CLK_EnableModuleClock(ADC_MODULE);

    /* ADC clock source is PCLK1, set divider to 1 */
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL2_ADCSEL_PCLK1, CLK_CLKDIV0_ADC(1));

    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set PB.2 - PB.3 to input mode */
    GPIO_SetMode(PB, BIT2|BIT3, GPIO_MODE_INPUT);
    /* Configure the PB.2 - PB.3 ADC analog input pins.  */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk)) |
                    (SYS_GPB_MFPL_PB2MFP_ADC0_CH2 | SYS_GPB_MFPL_PB3MFP_ADC0_CH3);
    /* Disable the PB.2 - PB.3 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT2|BIT3);

    /* Lock protected registers */
    SYS_LockReg();
}

void PWM0_Init()
{
    /* Set PWM0 timer clock prescaler */
    PWM_SET_PRESCALER(PWM0, 0, 0);

    /* Set up counter type */
    PWM0->CTL1 &= ~PWM_CTL1_CNTTYPE0_Msk;

    /* Set PWM0 timer duty */
    PWM_SET_CMR(PWM0, 0, 108);

    /* Set PWM0 timer period */
    PWM_SET_CNR(PWM0, 0, 216);

    /* PWM period point trigger ADC enable */
    PWM_EnableADCTrigger(PWM0, 0, PWM_TRIGGER_ADC_EVEN_PERIOD_POINT);

    /* Set output level at zero, compare up, period(center) and compare down of specified channel */
    PWM_SET_OUTPUT_LEVEL(PWM0, BIT0, PWM_OUTPUT_HIGH, PWM_OUTPUT_LOW, PWM_OUTPUT_NOTHING, PWM_OUTPUT_NOTHING);

    /* Enable output of PWM0 channel 0 */
    PWM_EnableOutput(PWM0, BIT0);
}

void PDMA_Init()
{
    /* Configure PDMA peripheral mode form ADC to memory */
    /* Open Channel 1 */
    PDMA_Open(PDMA, BIT1);

    /* transfer width is half word(16 bit) and transfer count is 6 */
    PDMA_SetTransferCnt(PDMA, 1, PDMA_WIDTH_16, 6);

    /* Set source address as ADC PDMA Current Transfer Data register (no increment) and destination address as g_i32ConversionData array (increment) */
    PDMA_SetTransferAddr(PDMA, 1, (uint32_t)&ADC->ADPDMA, PDMA_SAR_FIX, (uint32_t)g_i32ConversionData, PDMA_DAR_INC);

    /* Select PDMA request source as ADC RX */
    PDMA_SetTransferMode(PDMA, 1, PDMA_ADC_RX, FALSE, 0);

    /* Set PDMA as single request type for ADC */
    PDMA_SetBurstType(PDMA, 1, PDMA_REQ_SINGLE, PDMA_BURST_4);

    PDMA_EnableInt(PDMA, 1, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA_IRQn);
}

void ReloadPDMA()
{
    /* transfer width is half word(16 bit) and transfer count is 6 */
    PDMA_SetTransferCnt(PDMA, 1, PDMA_WIDTH_16, 6);

    /* Select PDMA request source as ADC RX */
    PDMA_SetTransferMode(PDMA, 1, PDMA_ADC_RX, FALSE, 0);
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
    ADC_POWER_ON(ADC);

    while(1)
    {
        /* reload PDMA configuration for next transmission */
        ReloadPDMA();

        printf("Select input mode:\n");
        printf("  [1] Single end input (channel 2 only)\n");
        printf("  [2] Differential input (channel pair 1 only (channel 2 and 3))\n");
        printf("  Other keys: exit single mode test\n");
        u8Option = getchar();
        if(u8Option == '1')
        {
            /* Set input mode as single-end, Single mode, and select channel 2 */
            ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE, BIT2);

            /* Configure the sample module and enable PWM0 trigger source */
            ADC_EnableHWTrigger(ADC, ADC_ADCR_TRGS_PWM, 0);

            /* ADC enable PDMA transfer */
            ADC_ENABLE_PDMA(ADC);

            printf("Conversion result of channel 2:\n");

            /* Enable PWM0 channel 0 counter */
            PWM_Start(PWM0, PWM_CH_0_MASK);

            while(1)
            {
                /* Wait PDMA interrupt (g_u32IsTestOver will be set at IRQ_Handler function) */
                while(g_u32IsTestOver == 0);
                break;
            }
            g_u32IsTestOver = 0;

            /* Disable PWM0 channel 0 counter */
            PWM_ForceStop(PWM0, BIT0);  /* PWM0 counter stop running. */

            for(g_u32COVNUMFlag = 0; (g_u32COVNUMFlag) < 6; g_u32COVNUMFlag++)
                printf("                                0x%X (%d)\n", g_i32ConversionData[g_u32COVNUMFlag], g_i32ConversionData[g_u32COVNUMFlag]);
        }
        else if(u8Option == '2')
        {
            /* Set input mode as differential, Single mode, and select channel 2 */
            ADC_Open(ADC, ADC_ADCR_DIFFEN_DIFFERENTIAL, ADC_ADCR_ADMD_SINGLE, BIT2);

            /* Configure the sample module and enable PWM0 trigger source */
            ADC_EnableHWTrigger(ADC, ADC_ADCR_TRGS_PWM, 0);

            /* ADC enable PDMA transfer */
            ADC_ENABLE_PDMA(ADC);

            printf("Conversion result of channel 2:\n");

            /* Enable PWM0 channel 0 counter */
            PWM_Start(PWM0, PWM_CH_0_MASK);

            while(1)
            {
                /* Wait PDMA interrupt (g_u32IsTestOver will be set at IRQ_Handler function) */
                while(g_u32IsTestOver == 0);
                break;
            }
            g_u32IsTestOver = 0;

            /* Disable EPWM0 channel 0 counter */
            PWM_ForceStop(PWM0, BIT0);  /* PWM0 counter stop running. */

            for(g_u32COVNUMFlag = 0; (g_u32COVNUMFlag) < 6; g_u32COVNUMFlag++)
                printf("                                0x%X (%d)\n", g_i32ConversionData[g_u32COVNUMFlag], g_i32ConversionData[g_u32COVNUMFlag]);
        }
        else
            return ;
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
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
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
    CLK_DisableModuleClock(ADC_MODULE);

    /* Disable PWM0 IP clock */
    CLK_DisableModuleClock(PWM0_MODULE);

    /* Disable PDMA clock source */
    CLK_DisableModuleClock(PDMA_MODULE);

    /* Disable PDMA Interrupt */
    NVIC_DisableIRQ(PDMA_IRQn);

    printf("Exit ADC sample code\n");

    while(1);
}

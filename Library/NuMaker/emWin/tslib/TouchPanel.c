/**************************************************************************//**
 * @file     TouchPanel.c
 * @version  V1.00
 * @brief    Perform A/D Conversion with ADC single mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include "NuMicro.h"

#include "TouchPanel.h"

static volatile    uint32_t    g_u32AdcIntFlag_TP;

void ADC_IRQHandler(void)
{
    /* Clear the A/D ADINT1 interrupt flag */
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

    g_u32AdcIntFlag_TP = 1;

}

int Init_TouchPanel(void)
{
    /* Enable peripheral clock */
    CLK_EnableModuleClock(ADC_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL2_ADCSEL_PCLK1, CLK_CLKDIV0_ADC(1));

    return 1;
}
#if 0
static void _BubbleSort(uint16_t arr[], int len)
{
    int i, j, temp;
    for(i = 0; i < len - 1; ++i)
        for(j = 0; j < len - 1 - i; ++j)
            if(arr[j] > arr[j + 1])
            {
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
}
#endif
static uint16_t _Api_Get_TP_X(void)
{
    uint32_t x_adc_in;
//    uint16_t nNormalizationFactor;
//    static uint16_t nXBorderMin = Init_MinBorderCaliValue_X;
//    static uint16_t nXBorderMax = Init_MaxBorderCaliValue_X;
//    static uint8_t nMaxBorderCaliCnt = 0;
//    static uint8_t nMinBorderCaliCnt = 0;
//    static uint16_t nMaxBorderCaliBuffer[5] = {0};
//    static uint16_t nMinBorderCaliBuffer[5] = {0};
    /* Init ADC for TP */
    /* Power on ADC module */
    ADC_POWER_ON(ADC);

    /* Set input mode as single-end and enable the A/D converter */
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE, BIT7);

    /*=== Get X from ADC input ===*/
    GPIO_SetMode(PB, BIT4, GPIO_MODE_OUTPUT);   // XR
    GPIO_SetMode(PB, BIT5, GPIO_MODE_INPUT);   // YD
    GPIO_SetMode(PB, BIT6, GPIO_MODE_OUTPUT);  // XL
    PB4 = 1; //XR High
    PB6 = 0; //XL Low

    /* Configure the GPB7 ADC analog input pins.  */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB4MFP_Msk);    // Disable ADC CH4
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB7MFP_Msk);    // Enable ADC CH7
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB7MFP_ADC0_CH7;  //YU sample

    /* Disable the GPB7 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT7);            //YU

    /* Power on ADC module */
//    ADC_POWER_ON(ADC);

    /* Enable the sample module 1 interrupt.  */
    ADC_EnableInt(ADC, ADC_ADF_INT);                //Enable sample module A/D ADINT1 interrupt.
    NVIC_EnableIRQ(ADC_IRQn);

    /* Clear the A/D ADINT1 interrupt flag for safe */
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

    /* Reset the ADC interrupt indicator and trigger sample module 1 to start A/D conversion */
    g_u32AdcIntFlag_TP = 0;
    ADC_START_CONV(ADC);

    /* Wait ADC interrupt (g_u32AdcIntFlag_TP will be set at IRQ_Handler function) */
    while(g_u32AdcIntFlag_TP == 0);
    x_adc_in = ADC_GET_CONVERSION_DATA(ADC, 7);

    ADC_Close(ADC);
#if 0
    //Normalization
    if(x_adc_in <= ReportThreshold)
    {
        //Dymanic Border Calibration
        if(x_adc_in > nXBorderMax)
        {
            nMaxBorderCaliBuffer[nMaxBorderCaliCnt] = x_adc_in;
            nMaxBorderCaliCnt++;
            if(nMaxBorderCaliCnt == 5)
            {
                _BubbleSort(nMaxBorderCaliBuffer, 5);
                nMaxBorderCaliCnt = 0;
                nXBorderMax = (nMaxBorderCaliBuffer[0] + nMaxBorderCaliBuffer[1]) / 2;
            }
        }
        if(x_adc_in < nXBorderMin)
        {
            nMinBorderCaliBuffer[nMinBorderCaliCnt] = x_adc_in;
            nMinBorderCaliCnt++;
            if(nMinBorderCaliCnt == 5)
            {
                _BubbleSort(nMinBorderCaliBuffer, 5);
                nMinBorderCaliCnt = 0;
                nXBorderMin = (nMinBorderCaliBuffer[3] + nMinBorderCaliBuffer[4]) / 2;
            }
        }
        x_adc_in -= nXBorderMin;
        nNormalizationFactor = (nXBorderMax - nXBorderMin) * 1000 / LCD_Resolution_X;
        x_adc_in *= 1000;
        x_adc_in /= nNormalizationFactor;
    }
#endif
    return x_adc_in;
}

/*-----------------------------------------------*/
// Get Y Position from Touch Panel (ADC input)
//
/*-----------------------------------------------*/
static uint16_t _Api_Get_TP_Y(void)
{
    uint32_t y_adc_in;
//    uint16_t nNormalizationFactor;
//    static uint16_t nYBorderMin = Init_MinBorderCaliValue_Y;
//    static uint16_t nYBorderMax = Init_MaxBorderCaliValue_Y;
//    static uint8_t nMaxBorderCaliCnt = 0;
//    static uint8_t nMinBorderCaliCnt = 0;
//    static uint16_t nMaxBorderCaliBuffer[5] = {0};
//    static uint16_t nMinBorderCaliBuffer[5] = {0};
    /* Init ADC for TP */
    /* Power on ADC module */
    ADC_POWER_ON(ADC);

    /* Set input mode as single-end and enable the A/D converter */
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE, BIT4);

    /*=== Get Y from ADC input ===*/
    GPIO_SetMode(PB, BIT7, GPIO_MODE_OUTPUT);   // YU
    GPIO_SetMode(PB, BIT5, GPIO_MODE_OUTPUT);  // YD
    GPIO_SetMode(PB, BIT6, GPIO_MODE_INPUT);   // XL
    PB7 = 1;
    PB5 = 0;

    /* Configure the GPB4 ADC analog input pins.  */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB7MFP_Msk);    // Disable ADC CH7
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB4MFP_Msk);    // Enable ADC CH4
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB4MFP_ADC0_CH4;  //XR

    /* Disable the GPB4 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT4);            //XR

    /* Power on ADC module */
//    ADC_POWER_ON(ADC);

    /* Enable the sample module 1 interrupt.  */
    ADC_EnableInt(ADC, ADC_ADF_INT);    //Enable sample module A/D ADINT1 interrupt.
    NVIC_EnableIRQ(ADC_IRQn);

    /* Clear the A/D ADINT1 interrupt flag for safe */
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

    /* Reset the ADC interrupt indicator and trigger sample module 2 to start A/D conversion */
    g_u32AdcIntFlag_TP = 0;
    ADC_START_CONV(ADC);

    /* Wait ADC interrupt (g_u32AdcIntFlag_TP will be set at IRQ_Handler function) */
    while(g_u32AdcIntFlag_TP == 0);
    y_adc_in = ADC_GET_CONVERSION_DATA(ADC, 4);
    ADC_Close(ADC);
#if 0
    /*=== Calculate the Y ===*/
    if(y_adc_in <= ReportThreshold)
    {
        //Dymanic Border Calibration
        if(y_adc_in > nYBorderMax)
        {
            nMaxBorderCaliBuffer[nMaxBorderCaliCnt] = y_adc_in;
            nMaxBorderCaliCnt++;
            if(nMaxBorderCaliCnt == 5)
            {
                _BubbleSort(nMaxBorderCaliBuffer, 5);
                nMaxBorderCaliCnt = 0;
                nYBorderMax = (nMaxBorderCaliBuffer[0] + nMaxBorderCaliBuffer[1]) / 2;
            }
        }
        if(y_adc_in < nYBorderMin)
        {
            nMinBorderCaliBuffer[nMinBorderCaliCnt] = y_adc_in;
            nMinBorderCaliCnt++;
            if(nMinBorderCaliCnt == 5)
            {
                _BubbleSort(nMinBorderCaliBuffer, 5);
                nMinBorderCaliCnt = 0;
                nYBorderMin = (nMinBorderCaliBuffer[3] + nMinBorderCaliBuffer[4]) / 2;
            }
        }
        y_adc_in -= nYBorderMin;
        nNormalizationFactor = (nYBorderMax - nYBorderMin) * 1000 / LCD_Resolution_Y;
        y_adc_in *= 1000;
        y_adc_in /= nNormalizationFactor;
    }
#endif
    return y_adc_in;
}

int Read_TouchPanel(int *x, int *y)
{
    *x = _Api_Get_TP_X();
    *y = _Api_Get_TP_Y();
    if(((*x & 0x0F00) == 0x0F00) || ((*y & 0x0F00) == 0x0F00))
        return 0;
    else
        return 1;
}

int Uninit_TouchPanel(void)
{
    return 1;
}

int Check_TouchPanel(void)
{
    return 0;   //Pen up;
}


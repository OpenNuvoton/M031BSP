/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief
 *           Demonstrate how to implement a USB audio class device.
 *           NAU8822 is used in this sample code to play the audio data from Host.
 *           It also supports to record data from NAU8822 to Host.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "usbd_audio.h"

#define TRIM_INIT           (SYS_BASE+0x118)

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    #if CRYSTAL_LESS
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    #else
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk|CLK_PWRCTL_HXTEN_Msk);
    #endif
    /* Waiting for HIRC clock ready */
    #if CRYSTAL_LESS
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    #else
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk|CLK_STATUS_HXTSTB_Msk);
    #endif

    /* Switch HCLK clock source to HIRC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));
    #if (CRYSTAL_LESS==0)
    CLK_SetCoreClock(48000000);
    #endif

    /* Select HIRC as the clock source of UART0 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    
    #if CRYSTAL_LESS
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_PCLK0, 0);    
    #else
    CLK_SetModuleClock(USBD_MODULE,CLK_CLKSEL0_USBDSEL_PLL,CLK_CLKDIV0_USB(2));    
    #endif

    /* Select PCLK1 as the clock source of SPI0 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable SPI0 peripheral clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Enable USB clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /* Enable I2C0 clock */
#ifdef OPT_I2C0
    CLK_EnableModuleClock(I2C0_MODULE);
#else
    CLK_EnableModuleClock(I2C1_MODULE);
#endif

    SystemCoreClockUpdate();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) | 
        (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

#ifdef OPT_I2C0
    /* Set PC multi-function pins for I2C0 */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC1MFP_I2C0_SCL | SYS_GPC_MFPL_PC0MFP_I2C0_SDA)) | 
        (SYS_GPC_MFPL_PC1MFP_I2C0_SCL | SYS_GPC_MFPL_PC0MFP_I2C0_SDA);
#else
    /* Set PB multi-function pins for I2C1 */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB1MFP_I2C1_SCL | SYS_GPB_MFPL_PB0MFP_I2C1_SDA)) | 
        (SYS_GPB_MFPL_PB1MFP_I2C1_SCL | SYS_GPB_MFPL_PB0MFP_I2C1_SDA);
#endif

#ifdef OPT_I2C0
    /* Setup SPI0 multi-function pins */
    /* PB.0 is SPI0_I2SMCLK, 		PA.3 is SPI0_SS (I2S_LRCLK)FS
       PA.2 is SPI0_CLK (I2S_BCLK), PA.1 is SPI0_MISO (I2S_DI)ADC
       PA.0 is SPI0_MOSI (I2S_DO)DAC 
    */

    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA3MFP_Msk |
                     SYS_GPA_MFPL_PA2MFP_Msk |
                     SYS_GPA_MFPL_PA1MFP_Msk |
                     SYS_GPA_MFPL_PA0MFP_Msk)) |
                    (SYS_GPA_MFPL_PA3MFP_SPI0_SS |
                     SYS_GPA_MFPL_PA2MFP_SPI0_CLK |
                     SYS_GPA_MFPL_PA1MFP_SPI0_MISO |
                     SYS_GPA_MFPL_PA0MFP_SPI0_MOSI);

    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB11MFP_Msk)) | SYS_GPB_MFPH_PB11MFP_SPI0_I2SMCLK;

#else
    /* Setup SPI0 multi-function pins */
    /* PA.4 is SPI0_I2SMCLK, 		PA.3 is SPI0_SS (I2S_LRCLK)
       PA.2 is SPI0_CLK (I2S_BCLK), PA.1 is SPI0_MISO (I2S_DI)
       PA.0 is SPI0_MOSI (I2S_DO)
     */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA4MFP_Msk | 
                     SYS_GPA_MFPL_PA3MFP_Msk |
                     SYS_GPA_MFPL_PA2MFP_Msk |
                     SYS_GPA_MFPL_PA1MFP_Msk |
                     SYS_GPA_MFPL_PA0MFP_Msk)) |
                    (SYS_GPA_MFPL_PA4MFP_SPI0_I2SMCLK |
                     SYS_GPA_MFPL_PA3MFP_SPI0_SS |
                     SYS_GPA_MFPL_PA2MFP_SPI0_CLK |
                     SYS_GPA_MFPL_PA1MFP_SPI0_MISO |
                     SYS_GPA_MFPL_PA0MFP_SPI0_MOSI);
#endif
}

void UART0_Init(void)
{
    /* Reset IP */
    SYS_ResetModule(UART0_RST);
    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/* Init I2C interface */
#ifdef OPT_I2C0
void I2C0_Init(void)
{
    /* Open I2C0 and set clock to 100k */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));
}
#else
void I2C1_Init(void)
{
    /* Open I2C1 and set clock to 100k */
    I2C_Open(I2C1, 100000);

    /* Get I2C3 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C1));
}
#endif

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
#if CRYSTAL_LESS
    uint32_t u32TrimInit;
#endif
    /* Unlock Protected Regsiter */
    SYS_UnlockReg();

    /* Initial system & multi-function */
    SYS_Init();

    /* Initial UART0 for debug message */
    UART0_Init();

#ifdef __HID__
    GPIO_Init();
#endif

    printf("\n");
    printf("+-------------------------------------------------------+\n");
    printf("|          NuMicro USB Audio CODEC Sample Code          |\n");
    printf("+-------------------------------------------------------+\n");
    printf("Please uninstall the UAC device if you use different target or other UAC demo code before\n");

    /* Init I2C1 to access NAU8822 */
#ifdef OPT_I2C0
    I2C0_Init();
#else
    I2C1_Init();
#endif

#ifdef OPT_I2S_SLAVE_MODE
    /* Set I2S in slave mode and let NAU8822 provide the exact WS/BCLK */
#ifdef INPUT_IS_LIN
    SPII2S_Open(SPI0, SPII2S_MODE_SLAVE, SAMPLING_RATE, SPII2S_DATABIT_16, SPII2S_STEREO, SPII2S_FORMAT_I2S);
#else
    SPII2S_Open(SPI0, SPII2S_MODE_SLAVE, SAMPLING_RATE, SPII2S_DATABIT_16, SPII2S_STEREO, SPII2S_FORMAT_I2S);
#endif
#else
#ifdef INPUT_IS_LIN
    SPII2S_Open(SPI0, SPII2S_MODE_MASTER, SAMPLING_RATE, SPII2S_DATABIT_16, SPII2S_STEREO, SPII2S_FORMAT_I2S);
#else
    SPII2S_Open(SPI0, SPII2S_MODE_MASTER, SAMPLING_RATE, SPII2S_DATABIT_16, SPII2S_STEREO, SPII2S_FORMAT_I2S);
#endif
#endif  /* OPT_I2S_SLAVE_MODE */

    SPII2S_SetFIFO(SPI0, 2, 2);

    /* Initialize NAU8822 codec */
    NAU8822_Setup();

    /* Set MCLK and enable MCLK (MCLK can provide clock source to NAU882) */
    SPII2S_EnableMCLK(SPI0, 12000000);

#ifndef INPUT_IS_LIN
    SPII2S_SET_MONO_RX_CHANNEL(SPI0, SPII2S_MONO_LEFT);       /* NAU8822 will store data in left channel */
#endif

    /* Start I2S play iteration */
    SPII2S_EnableInt(SPI0, SPII2S_FIFO_TXTH_INT_MASK | SPII2S_FIFO_RXTH_INT_MASK);

    USBD_Open(&gsInfo, UAC_ClassRequest, (SET_INTERFACE_REQ)UAC_SetInterface);

    /* Endpoint configuration */
    UAC_Init();

    USBD_Start();
    
#if CRYSTAL_LESS
    /* Backup init trim */
    u32TrimInit = M32(TRIM_INIT);

    /* Waiting for USB bus stable */
    USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);

    while((USBD_GET_INT_FLAG() & USBD_INTSTS_SOFIF_Msk) == 0);

    /* Enable USB crystal-less - Set reference clock from USB SOF packet & Enable HIRC auto trim function */
    SYS->HIRCTRIMCTL |= (SYS_HIRCTRIMCTL_REFCKSEL_Msk | 0x1);
#endif

    NVIC_EnableIRQ(USBD_IRQn);
    NVIC_EnableIRQ(SPI0_IRQn);

    /* SPI (I2S) interrupt has higher frequency then USBD interrupt.
       Therefore, we need to set SPI (I2S) with higher priority to avoid
       SPI (I2S) interrupt pending too long time when USBD interrupt happen. */
    NVIC_SetPriority(USBD_IRQn, 3);
    NVIC_SetPriority(SPI0_IRQn, 2);

#if CRYSTAL_LESS
    /* Give a dummy target frequency here. Will over write prescale and compare value with macro */
    TIMER_Open(TIMER0, TIMER_ONESHOT_MODE, 100);

    /* Update prescale and compare value. Calculate average frequency every 100 events */
    TIMER_SET_PRESCALE_VALUE(TIMER0, 0);

    TIMER_SET_CMP_VALUE(TIMER0,11);

    TIMER0->EXTCTL|=TIMER_EXTCTL_ECNTSSEL_Msk;

    /* Update Timer 1 prescale value. So Timer 0 clock is 24MHz */
    TIMER_SET_PRESCALE_VALUE(TIMER1, 0);

    /* We need capture interrupt */
    NVIC_EnableIRQ(TMR1_IRQn); 

    TIMER_EnableFreqCounter(TIMER0, 0, 0, TRUE);
#endif

    while (SYS->PDID) 
    {
        uint8_t ch;
        uint32_t u32Reg, u32Data;
        extern int32_t kbhit(void);
#ifndef __FEEDBACK__
        #if CRYSTAL_LESS
        AdjFreq1();
        #else
        /* Adjust codec sampling rate to synch with USB. The adjustment range is +-0.005% */
        AdjFreq();
        #endif
#endif
#ifdef __HID__
        HID_UpdateHidData();
#endif

        /* Set audio volume according USB volume control settings */
        VolumnControl();

        SamplingControl();

        /* User can change audio codec settings by I2C at run-time if necessary */
        if (!kbhit())
        {
            printf("\nEnter codec setting:\n");
            /* Get Register number */
            ch = getchar();
            u32Reg = ch - '0';
            ch = getchar();
            u32Reg = u32Reg * 10 + (ch - '0');
            printf("%d\n", u32Reg);

            /* Get data */
            ch = getchar();
            u32Data = (ch >= '0' && ch <= '9') ? ch - '0' : ch - 'a' + 10;
            ch = getchar();
            u32Data = u32Data * 16 + ((ch >= '0' && ch <= '9') ? ch - '0' : ch - 'a' + 10);
            ch = getchar();
            u32Data = u32Data * 16 + ((ch >= '0' && ch <= '9') ? ch - '0' : ch - 'a' + 10);
            printf("%03x\n", u32Data);
            I2C_WriteNAU8822(u32Reg,  u32Data);
        }

#if CRYSTAL_LESS
        /* Re-start crystal-less when any error found */
        if (SYS->HIRCTRIMSTS & (SYS_HIRCTRIMSTS_TFAILIF_Msk | SYS_HIRCTRIMSTS_CLKERIF_Msk))
        {
            SYS->HIRCTRIMSTS = SYS_HIRCTRIMSTS_TFAILIF_Msk | SYS_HIRCTRIMSTS_CLKERIF_Msk;

            /* Init TRIM */
            M32(TRIM_INIT) = u32TrimInit;

            /* Waiting for USB bus stable */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SOFIF_Msk);
            while((USBD_GET_INT_FLAG() & USBD_INTSTS_SOFIF_Msk) == 0);

            /* Re-enable crystal-less - Set reference clock from USB SOF packet & Enable HIRC auto trim function */
            SYS->HIRCTRIMCTL |= (SYS_HIRCTRIMCTL_REFCKSEL_Msk | 0x1);
            //printf("USB trim fail. Just retry. SYS->HIRCTRIMSTS = 0x%x, SYS->HIRCTRIMCTL = 0x%x\n", SYS->HIRCTRIMSTS, SYS->HIRCTRIMCTL);
        }
#endif
    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/


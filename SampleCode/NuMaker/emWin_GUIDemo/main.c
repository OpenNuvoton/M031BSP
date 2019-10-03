/****************************************************************************
 * @file     main.c
 * @version  V3.0
 * @brief    To utilize emWin library to demonstrate  widgets feature.
 *
 * @note
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>

#include "NuMicro.h"

#include "TouchPanel.h"

#include "GUI.h"

extern volatile GUI_TIMER_TIME OS_TimeMS;

volatile int g_enable_Touch;

extern int ts_writefile(void);
extern int ts_readfile(void);
extern void ts_init(void);
int ts_calibrate(int xsize, int ysize);
void ts_test(int xsize, int ysize);

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/
/*********************************************************************
*
*       _SYS_Init
*/
void _SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(USCI0_MODULE);
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set USCI_SPI0 multi-function pins */
    SYS->GPA_MFPH = (SYS->GPA_MFPH & ~(SYS_GPA_MFPH_PA9MFP_Msk | SYS_GPA_MFPH_PA10MFP_Msk | SYS_GPA_MFPH_PA11MFP_Msk)) |
                    (SYS_GPA_MFPH_PA9MFP_USCI0_DAT1 | SYS_GPA_MFPH_PA10MFP_USCI0_DAT0 | SYS_GPA_MFPH_PA11MFP_USCI0_CLK);
    SYS->GPC_MFPH = (SYS->GPC_MFPH & ~(SYS_GPC_MFPH_PC14MFP_Msk)) | (SYS_GPC_MFPH_PC14MFP_USCI0_CTL0);

    /* Lock protected registers */
    SYS_LockReg();
}

/*********************************************************************
*
*       TMR0_IRQHandler
*/
void TMR0_IRQHandler(void)
{
    OS_TimeMS++;
#if GUI_SUPPORT_TOUCH
    if(OS_TimeMS % 10 == 0)
    {
        if(g_enable_Touch == 1)
        {
            GUI_TOUCH_Exec();
        }
    }
#endif
    TIMER_ClearIntFlag(TIMER0);
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       main
*/
int main(void)
{
    //
    // Init System, IP clock and multi-function I/O
    //
    _SYS_Init();
    //
    // Init UART to 115200-8n1 for print message
    //
    UART_Open(UART0, 115200);

    g_enable_Touch = 0;
    //
    // Enable Timer0 clock and select Timer0 clock source
    //
    CLK_EnableModuleClock(TMR0_MODULE);
//    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);
    //
    // Initial Timer0 to periodic mode with 1000Hz
    //
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000);
    //
    // Enable Timer0 interrupt
    //
    TIMER_EnableInt(TIMER0);
    NVIC_SetPriority(TMR0_IRQn, 1);
    NVIC_EnableIRQ(TMR0_IRQn);
    //
    // Start Timer0
    //
    TIMER_Start(TIMER0);
    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

#if GUI_SUPPORT_TOUCH
    GUI_Init();

    Init_TouchPanel();

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

#if 0 // Use default touch screen parameters
    ts_init();
#else // Get touch screen parameters
    /* SPI flash marker offset (__DEMO_TSFILE_ADDR__ + 0x1C) */
    /* Please note that ts_calibrate() needs more cstack size or may encouter Hard_Fault_Handler() */
    /* increase ctack size from 0x800 to 0x1000 in emWin_SimpleDemo.icf (IAR only) */
    /* decrease GUI_NUMBYTES from 14KB to 8KB in GUIConf.c */
    /* Please note that the SRAM size */
    if (FMC_Read(__DEMO_TSFILE_ADDR__ + 0x1C) != 0x55AAA55A)
    {
        FMC_ENABLE_AP_UPDATE();
        ts_calibrate(__DEMO_TS_WIDTH__, __DEMO_TS_HEIGHT__);
        // Erase page
        FMC_Erase(__DEMO_TSFILE_ADDR__);
        ts_writefile();
        FMC_DISABLE_AP_UPDATE();
    }
    else
    {
        ts_readfile();
    }
#endif
    
    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    g_enable_Touch = 1;

//    ts_test(__DEMO_TS_WIDTH__, __DEMO_TS_HEIGHT__);
#endif

    g_enable_Touch = 1;

    //
    // Start application
    //
    MainTask();
    while(1);
}

/*************************** End of file ****************************/

/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 12 $
 * $Date: 18/07/26 11:39a $
 * @brief    Demonstrate how to transfer data between USB device and PC through USB HID interface.
 *           A windows tool is also included in this sample code to connect with a USB device.
 *
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "hid_transfer.h"

#define CRYSTAL_LESS        0
#define TRIM_INIT           (SYS_BASE+0x118)

extern uint8_t volatile g_u8Suspend;
int IsDebugFifoEmpty(void);

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Switch USB clock source to HIRC & USB Clock = HIRC / 1 */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL0_USBDSEL_HIRC, CLK_CLKDIV0_USB(1));

    /* Enable USB clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))
                    |(SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

void PowerDown()
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    printf("Enter power down ...\n");
    while(!IsDebugFifoEmpty());

    /* Wakeup Enable */
    USBD_ENABLE_INT(USBD_INTEN_WKEN_Msk);

    CLK_PowerDown();

    /* Clear PWR_DOWN_EN if it is not clear by itself */
    if(CLK->PWRCTL & CLK_PWRCTL_PDEN_Msk)
        CLK->PWRCTL ^= CLK_PWRCTL_PDEN_Msk;

    printf("device wakeup!\n");

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
#if CRYSTAL_LESS
    uint32_t u32TrimInit;
#endif

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("\n");
    printf("+--------------------------------------------------------+\n");
    printf("|      NuMicro USB HID Transfer via Control Transfer     |\n");
    printf("+--------------------------------------------------------+\n");
    printf("Windows tool will Read and Write one pair of reports(periodic exchanges of reports).\n");
    USBD_Open(&gsInfo, HID_ClassRequest, NULL);

    /* Endpoint configuration */
    HID_Init();

    NVIC_EnableIRQ(USBD_IRQn);

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

    while (1)
    {
        /* Enter power down when USB suspend */
        if(g_u8Suspend)
            PowerDown();

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


/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

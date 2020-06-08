/******************************************************************************
* @file     main.c
* @version  V3.00
* $Revision: 7 $
* $Date: 18/07/16 3:49p $
* @brief    Demonstrate how to use LXT to trim HIRC
*
* @note
* SPDX-License-Identifier: Apache-2.0
* Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/**
 * @brief       HIRC Trim IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The CLKDIRC_IRQHandler default IRQ
 */
void CKFAIL_IRQHandler()
{
    if (SYS->HIRCTRIMSTS & SYS_HIRCTRIMSTS_TFAILIF_Msk)
    {
        /* Get Trim Failure Interrupt */
        /* Display HIRC trim status */
        printf("HIRC Trim Failure Interrupt\n");
        /* Clear Trim Failure Interrupt */
        SYS->HIRCTRIMSTS = SYS_HIRCTRIMSTS_TFAILIF_Msk;
    }

    if (SYS->HIRCTRIMSTS & SYS_HIRCTRIMSTS_CLKERIF_Msk)
    {
        /* Get Clock Error Interrupt */
        /* Display HIRC trim status */
        printf("Clock Error Interrupt\n");
        /* Clear Clock Error Interrupt */
        SYS->HIRCTRIMSTS = SYS_HIRCTRIMSTS_CLKERIF_Msk;
    }

}

/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable Internal High speed RC oscillator (HIRC) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Enable External Low speed crystal (LXT) */
    CLK->PWRCTL |= CLK_PWRCTL_LXTEN_Msk;

    /* Waiting for External Low speed clock ready */
    while((CLK->STATUS & CLK_STATUS_LXTSTB_Msk) != CLK_STATUS_LXTSTB_Msk);

    /* Waiting for Internal High speed RC clock ready */
    while((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) != CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk ) | CLK_CLKSEL0_HCLKSEL_HIRC ;

    /* Switch UART0 clock source to XTAL */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART0SEL_Msk) | CLK_CLKSEL1_UART0SEL_HIRC;

    /* Enable UART0 clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk ;

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

void TrimHIRC()
{
    /*  Enable IRC Trim, set HIRC clock and enable interrupt */
    SYS->HIRCTRIMIEN |= (SYS_HIRCTRIMIEN_CLKEIEN_Msk | SYS_HIRCTRIMIEN_TFALIEN_Msk);
    SYS->HIRCTRIMCTL = (SYS->HIRCTRIMCTL & ~SYS_HIRCTRIMCTL_FREQSEL_Msk) | 0x1;

    /* Waiting for HIRC Frequency Lock */
    SysTick->LOAD = 2000 * CyclesPerUs;
    SysTick->VAL  = (0x00);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Waiting for down-count to zero */
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);

    /* Disable SysTick counter */
    SysTick->CTRL = 0;

    /* Get HIRC Frequency Lock */
    while (1)
    {
        if (SYS->HIRCTRIMSTS & SYS_HIRCTRIMSTS_FREQLOCK_Msk)
        {
            printf("HIRC Frequency Lock\n");
            SYS->HIRCTRIMSTS = SYS_HIRCTRIMSTS_FREQLOCK_Msk;     /* Clear Trim Lock */
            break;
        }
    }
}

int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O
    In the end of SYS_Init() will issue SYS_LockReg()
    to lock protected register. If user want to write
    protected register, please issue SYS_UnlockReg()
    to unlock protected register if necessary */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    /* Reset UART0 */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

    /* Enable Interrupt */
    NVIC_EnableIRQ(CKFAIL_IRQn);

    /* Trim HIRC to 12MHz */
    TrimHIRC();

    /* Disable IRC Trim */
    SYS->HIRCTRIMCTL = 0;
    printf("Disable HIRC Trim\n");

    while (1);
}


/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/

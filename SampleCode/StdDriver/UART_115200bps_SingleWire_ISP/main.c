/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief
 *           The UART ISP LDROM firmware to update APROM through Single wire (PB.12).
 *           UART setting : Baudrate 115200 bps and 8-N-1.
 *           Config boot option must be set to "boot in LDROM".
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "uart_transfer.h"
#include "ISP_USER.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Init system clock and UART single wire function                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Wait for HIRC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);

    /* Set PLL to Power-down mode */
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Enable UART module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART0SEL_Msk)) | CLK_CLKSEL1_UART0SEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UART0DIV_Msk)) | CLK_CLKDIV0_UART0(1);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* The RX pin needs to qasi-high for single-wire */
    PB->MODE = (PB->MODE & ~GPIO_MODE_MODE0_Msk) | (GPIO_MODE_QUASI << GPIO_MODE_MODE12_Pos);

    /* Set PB multi-function pins for UART0 RXD=PB.12 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk)) | (SYS_GPB_MFPH_PB12MFP_UART0_RXD);
}
#define UART_RX_IDEL(uart) (((uart)->FIFOSTS & UART_FIFOSTS_RXIDLE_Msk )>> UART_FIFOSTS_RXIDLE_Pos)
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();
    /* Init UART to 115200 bps, 8bit data length, none parity check, one stop bit */
    UART_Init();

    /* Checking if flash page size matches with target chip's */
    if( (GET_CHIP_SERIES_NUM == CHIP_SERIES_NUM_I) || (GET_CHIP_SERIES_NUM == CHIP_SERIES_NUM_G) )
    {
        if(FMC_FLASH_PAGE_SIZE != 2048)
        {
            /* FMC_FLASH_PAGE_SIZE is different from target device */
            /* Please enable the compiler option PAGE_SIZE_2048 in fmc.h */
            while(SYS->PDID);
        }
    }
    else
    {
        if(FMC_FLASH_PAGE_SIZE != 512)
        {
            /* FMC_FLASH_PAGE_SIZE is different from target device */
            /* Please disable the compiler option PAGE_SIZE_2048 in fmc.h */
            while(SYS->PDID);
        }
    }

    //Enable uart single wire funciton
    UART0->FUNCSEL = ((UART0->FUNCSEL & (~UART_FUNCSEL_FUNCSEL_Msk)) | UART_FUNCSEL_SINGLE_WIRE);
    //Enable ISP clock
    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;
    //Enable ISPEN and APROM
    FMC->ISPCTL |= (FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk);
    //Get APROM size
    g_apromSize = GetApromSize();
    //Get Dataflash address and size
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);
    //Setting Systick time out count
    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL   = (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    while (1)
    {
        //check bufhead is ready
        if ((bufhead >= 4) || (bUartDataReady == TRUE))
        {
            uint32_t lcmd;
            //Get uart buffer for command
            lcmd = inpw(uart_rcvbuf);
            //check cmd is connect command
            if (lcmd == CMD_CONNECT)
            {
                //command is ready goto ISP flow
                goto _ISP;
            }
            else
            {
                //reset buffer count
                bUartDataReady = FALSE;
                bufhead = 0;
            }
        }

        //Systick count timeout, then goto APROM
        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
        {
            goto _APROM;
        }
    }

_ISP:

    while (1)
    {
        //check uart_rcvbuf is ready, and parser command
        if (bUartDataReady == TRUE)
        {
            //clear bUartDataReady flag
            bUartDataReady = FALSE;
            //Parser command and execution
            ParseCmd(uart_rcvbuf, 64);
            //wait rx bus is idel.
            while (!UART_RX_IDEL(UART0)) {};
            //return respond
            PutString();
        }
    }

_APROM:
    //software reset, jumper to APROM
    SYS->RSTSTS = (SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk);
    FMC->ISPCTL &= ~(FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_BS_Msk);
    SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);

    /* Trap the CPU */
    while (1);
}

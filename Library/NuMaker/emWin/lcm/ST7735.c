/**************************************************************************//**
 * @file     ST7735.c
 * @version  V1.00
 * @brief    Display controller configuration.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stddef.h>
#include <stdio.h>

#include "GUI.h"
#include "GUIDRV_FlexColor.h"

#include "NuMicro.h"
#ifdef __DEMO_160x128__
#include "TouchPanel.h"

#include "lcm.h"

//
// Hardware related
//
#define SPI_LCD_PORT  USPI0

#define GPIO_SPI_SS PA8
#define GPIOPORT_SPI_SS PA
#define PINMASK_SPI_SS BIT8

#define GPIO_LCM_DC PB2
#define GPIOPORT_LCM_DC PB
#define PINMASK_LCM_DC BIT2

#define GPIO_LCM_RESET PB3
#define GPIOPORT_LCM_RESET PB
#define PINMASK_LCM_RESET BIT3

#define SPI_CS_SET    GPIO_SPI_SS = 1
#define SPI_CS_CLR    GPIO_SPI_SS = 0

#define LCM_DC_SET    GPIO_LCM_DC = 1
#define LCM_DC_CLR    GPIO_LCM_DC = 0

#define LCM_RESET_SET GPIO_LCM_RESET = 1
#define LCM_RESET_CLR GPIO_LCM_RESET = 0

/*********************************************************************
*
*       _Read1
*/
U8 _Read1(void)
{
#if 1
    /* FIXME if panel supports read back feature */
    return 0;
#else
    LCM_DC_SET;
    SPI_CS_CLR;
    USPI_WRITE_TX(SPI_LCD_PORT, 0x00);
    USPI_READ_RX(SPI_LCD_PORT);
    SPI_CS_SET;
    return (USPI_READ_RX(SPI_LCD_PORT));
#endif
}

/*********************************************************************
*
*       _ReadM1
*/
void _ReadM1(U8 * pData, int NumItems)
{
#if 1
    /* FIXME if panel supports read back feature */
#else
    LCM_DC_SET;
    SPI_CS_CLR;
    while(NumItems--)
    {
        USPI_WRITE_TX(SPI_LCD_PORT, 0x00);
        while(SPI_IS_BUSY(SPI_LCD_PORT));
        *pData++ = USPI_READ_RX(SPI_LCD_PORT);
    }
    SPI_CS_SET;
#endif
}

/*********************************************************************
*
*       _Write0
*/
void _Write0(U8 Cmd)
{
    LCM_DC_CLR;

    SPI_CS_CLR;

    while(USPI_GET_TX_FULL_FLAG(SPI_LCD_PORT));
    USPI_WRITE_TX(SPI_LCD_PORT, Cmd);
    while(USPI_IS_BUSY(SPI_LCD_PORT));

    SPI_CS_SET;
}

/*********************************************************************
*
*       _Write1
*/
void _Write1(U8 Data)
{
    LCM_DC_SET;

    SPI_CS_CLR;

    while(USPI_GET_TX_FULL_FLAG(SPI_LCD_PORT));
    USPI_WRITE_TX(SPI_LCD_PORT, Data);
    while(USPI_IS_BUSY(SPI_LCD_PORT));

    SPI_CS_SET;
}

/*********************************************************************
*
*       _WriteM1
*/
void _WriteM1(U8 * pData, int NumItems)
{
    LCM_DC_SET;

    SPI_CS_CLR;

    while(NumItems--)
    {
        while(USPI_GET_TX_FULL_FLAG(SPI_LCD_PORT));
        USPI_WRITE_TX(SPI_LCD_PORT, *pData++);
        while (USPI_IS_BUSY(SPI_LCD_PORT));
    }

    SPI_CS_SET;
}

static void _Open_SPI(void)
{
    GPIO_SetMode(GPIOPORT_LCM_DC, PINMASK_LCM_DC, GPIO_MODE_OUTPUT);
    GPIO_SetMode(GPIOPORT_LCM_RESET, PINMASK_LCM_RESET, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PA, BIT6, GPIO_MODE_OUTPUT);
    GPIO_SetMode(GPIOPORT_SPI_SS, PINMASK_SPI_SS, GPIO_MODE_OUTPUT); //cs pin for gpiod

    /* Setup USPI0 multi-function pins */
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA9MFP_Msk);
    SYS->GPA_MFPH |=  (SYS_GPA_MFPH_PA9MFP_USCI0_DAT1);

    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA11MFP_Msk       | SYS_GPA_MFPH_PA10MFP_Msk);
    SYS->GPA_MFPH |=  (SYS_GPA_MFPH_PA11MFP_USCI0_CLK | SYS_GPA_MFPH_PA10MFP_USCI0_DAT0);

    /* Enable USCI0 peripheral clock */
    CLK_EnableModuleClock(USCI0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI_SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure USCI_SPI0 */
    /* Configure USCI_SPI0 as a master, USCI_SPI clock rate 48MHz,
       clock idle low, 8-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    USPI_Open(USPI0, USPI_MASTER, USPI_MODE_0, 8, 48000000);
    /* Disable the automatic hardware slave selection function. */
    USPI_DisableAutoSS(SPI_LCD_PORT);
}

/*********************************************************************
*
*       _InitController
*
* Purpose:
*   Initializes the display controller
*/
void _InitController(void) {
    int i;
    static uint8_t s_InitOnce = 0;

    if(s_InitOnce == 0)
        s_InitOnce = 1;
    else
        return;

    _Open_SPI();

    LCM_RESET_SET;
    LCM_RESET_CLR;

    GUI_X_Delay(100);

    LCM_RESET_SET;

    GUI_X_Delay(120);
    _Write0(0x11);    //Exit Sleep
    GUI_X_Delay(120);
//------------------------------------------------------------------//
//-------------------Software Reset---------------------------------//

    _Write0(0xB1);
    _Write1(0x01);
    _Write1(0x2C);
    _Write1(0x2D);

    _Write0(0xB2);
    _Write1(0x01);
    _Write1(0x2C);
    _Write1(0x2D);

    _Write0(0xB3);
    _Write1(0x01);
    _Write1(0x2C);
    _Write1(0x2D);

    _Write1(0x01);
    _Write1(0x2C);
    _Write1(0x2D);


    _Write0(0xB4);  //Column inversion
    _Write1(0x07);
    //ST7735R Power Sequence
    _Write0(0xC0);
    _Write1(0xA2);
    _Write1(0x02);
    _Write1(0x84);
    _Write1(0xC1);
    _Write1(0xC5);
    _Write0(0xC2);
    _Write1(0x0A);
    _Write1(0x00);

    _Write0(0xC3);
    _Write1(0x8A);
    _Write1(0x2A);
    _Write1(0xC4);
    _Write1(0x8A);
    _Write1(0xEE);

    _Write0(0xC5); //VCOM
    _Write1(0x0E);

    _Write0(0x36); //MX, MY, RGB mode
    _Write1(0xC0);
    _Write1(0xC8); //??C8 ??08 A8

    //ST7735R Gamma Sequence
    _Write0(0xe0);
    _Write1(0x0f);
    _Write1(0x1a);
    _Write1(0x0f);
    _Write1(0x18);
    _Write1(0x2f);
    _Write1(0x28);
    _Write1(0x20);
    _Write1(0x22);
    _Write1(0x1f);
    _Write1(0x1b);
    _Write1(0x23);
    _Write1(0x37);
    _Write1(0x00);
    _Write1(0x07);
    _Write1(0x02);
    _Write1(0x10);

    _Write0(0xe1);
    _Write1(0x0f);
    _Write1(0x1b);
    _Write1(0x0f);
    _Write1(0x17);
    _Write1(0x33);
    _Write1(0x2c);
    _Write1(0x29);
    _Write1(0x2e);
    _Write1(0x30);
    _Write1(0x30);
    _Write1(0x39);
    _Write1(0x3f);
    _Write1(0x00);
    _Write1(0x07);
    _Write1(0x03);
    _Write1(0x10);

    _Write0(0x2a);
    _Write1(0x02);
    _Write1(0x00+2);
    _Write1(0x02);
    _Write1(0x7F+2);

    _Write0(0x2b);
    _Write1(0x01);
    _Write1(0x00+1);
    _Write1(0x01);
    _Write1(0x9F+1);

    _Write0(0xF0); //Enable test command
    _Write1(0x01);
    _Write0(0xF6); //Disable ram power save mode
    _Write1(0x00);

    _Write0(0x3A); //65k mode
    _Write1(0x05);

    _Write0(0x2c);
    for(i=0; i<0x5000; i++)
    {
        _Write1(0x00>>8);
        _Write1(0x00);
    }

    _Write0(0x29);    //Display on
}
#endif

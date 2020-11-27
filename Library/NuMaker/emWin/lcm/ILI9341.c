/**************************************************************************//**
 * @file     ILI9341.c
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
#ifdef __DEMO_320x240__
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

#define ILI9341_LED     PA6

#define USPI_MASTER_TX_DMA_CH  0

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

    /* Set transfer count */
    PDMA_SET_TRANS_CNT(PDMA, USPI_MASTER_TX_DMA_CH, NumItems);
    /* Set source address */
    PDMA_SET_SRC_ADDR(PDMA, USPI_MASTER_TX_DMA_CH, (uint32_t)pData++);
    /* Set basic mode */
    PDMA->DSCT[USPI_MASTER_TX_DMA_CH].CTL = (PDMA->DSCT[USPI_MASTER_TX_DMA_CH].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | PDMA_OP_BASIC;

    SPI_CS_CLR;

    /* Enable SPI master's PDMA transfer function */
    USPI_TRIGGER_TX_PDMA(SPI_LCD_PORT);

    /* Check the PDMA transfer done flag */
    while((PDMA_GET_TD_STS(PDMA) & (1 << USPI_MASTER_TX_DMA_CH)) == 0);

    /* Clear the PDMA transfer done flag */
    PDMA_CLR_TD_FLAG(PDMA, (1 << USPI_MASTER_TX_DMA_CH));

    /* Wait TX finish */
    while(USPI_IS_BUSY(SPI_LCD_PORT));
	//
	// Finish PDMA gracefully
	//
    USPI_DISABLE_TX_PDMA(SPI_LCD_PORT);

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

static void _Open_PDMA(void)
{
    /* Reset PDMA module */
    SYS_ResetModule(PDMA_RST);

    /* Enable PDMA channel */
    PDMA_Open(PDMA, (1 << USPI_MASTER_TX_DMA_CH));

    /* Set transfer width (8 bits) */
    PDMA->DSCT[USPI_MASTER_TX_DMA_CH].CTL = (PDMA->DSCT[USPI_MASTER_TX_DMA_CH].CTL & ~PDMA_DSCT_CTL_TXWIDTH_Msk) | PDMA_WIDTH_8;
    /* Set destination address */
    PDMA_SET_DST_ADDR(PDMA, USPI_MASTER_TX_DMA_CH, (uint32_t)&SPI_LCD_PORT->TXDAT);
    /* Set source/destination attributes */
    PDMA->DSCT[USPI_MASTER_TX_DMA_CH].CTL = (PDMA->DSCT[USPI_MASTER_TX_DMA_CH].CTL & ~(PDMA_DSCT_CTL_SAINC_Msk | PDMA_DSCT_CTL_DAINC_Msk)) | (PDMA_SAR_INC | PDMA_DAR_FIX);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA, USPI_MASTER_TX_DMA_CH, PDMA_USCI0_TX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA, USPI_MASTER_TX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA->DSCT[USPI_MASTER_TX_DMA_CH].CTL |= PDMA_TBINTDIS_DISABLE;
}

/*********************************************************************
*
*       _InitController
*
* Purpose:
*   Initializes the display controller
*/
void _InitController(void)
{
    static uint8_t s_InitOnce = 0;

    if(s_InitOnce == 0)
        s_InitOnce = 1;
    else
        return;

    _Open_SPI();
    _Open_PDMA();

    /* Configure DC/RESET/LED pins */
    GPIO_LCM_DC = 0;
    GPIO_LCM_RESET = 0;
    ILI9341_LED = 0;

    /* Configure LCD */
    GPIO_LCM_DC = 1;

    GPIO_LCM_RESET = 0;
    GUI_X_Delay(20);

    GPIO_LCM_RESET = 1;
    GUI_X_Delay(40);

    //************* Start Initial Sequence **********//

    _Write0(0xCF);
    _Write1(0x00);
    _Write1(0xD9);
    _Write1(0X30);

    _Write0(0xED);
    _Write1(0x64);
    _Write1(0x03);
    _Write1(0X12);
    _Write1(0X81);

    _Write0(0xE8);
    _Write1(0x85);
    _Write1(0x10);
    _Write1(0x78);

    _Write0(0xCB);
    _Write1(0x39);
    _Write1(0x2C);
    _Write1(0x00);
    _Write1(0x34);
    _Write1(0x02);

    _Write0(0xF7);
    _Write1(0x20);

    _Write0(0xEA);
    _Write1(0x00);
    _Write1(0x00);

    _Write0(0xC0);    //Power control
    _Write1(0x21);   //VRH[5:0]

    _Write0(0xC1);    //Power control
    _Write1(0x12);   //SAP[2:0];BT[3:0]

    _Write0(0xC5);    //VCM control
    _Write1(0x32);
    _Write1(0x3C);

    _Write0(0xC7);    //VCM control2
    _Write1(0XC1);

    _Write0(0x36);    // Memory Access Control
    _Write1(0xe8);

    _Write0(0x3A);
    _Write1(0x55);

    _Write0(0xB1);
    _Write1(0x00);
    _Write1(0x18);

    _Write0(0xB6);    // Display Function Control
    _Write1(0x0A);
    _Write1(0xA2);

    _Write0(0xF2);    // 3Gamma Function Disable
    _Write1(0x00);

    _Write0(0x26);    //Gamma curve selected
    _Write1(0x01);

    _Write0(0xE0);    //Set Gamma
    _Write1(0x0F);
    _Write1(0x20);
    _Write1(0x1E);
    _Write1(0x09);
    _Write1(0x12);
    _Write1(0x0B);
    _Write1(0x50);
    _Write1(0XBA);
    _Write1(0x44);
    _Write1(0x09);
    _Write1(0x14);
    _Write1(0x05);
    _Write1(0x23);
    _Write1(0x21);
    _Write1(0x00);

    _Write0(0XE1);    //Set Gamma
    _Write1(0x00);
    _Write1(0x19);
    _Write1(0x19);
    _Write1(0x00);
    _Write1(0x12);
    _Write1(0x07);
    _Write1(0x2D);
    _Write1(0x28);
    _Write1(0x3F);
    _Write1(0x02);
    _Write1(0x0A);
    _Write1(0x08);
    _Write1(0x25);
    _Write1(0x2D);
    _Write1(0x0F);

    _Write0(0x11);    //Exit Sleep
    GUI_X_Delay(120);
    _Write0(0x29);    //Display on

    ILI9341_LED = 1;
}
#endif

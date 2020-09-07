/**************************************************************************//**
 * @file     TouchPanel.h
 * @version  V1.00
 * @brief    Perform A/D Conversion with ADC single mode header file.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __M032TOUCHPANEL_H__
#define __M032TOUCHPANEL_H__

#define __DEMO_TSFILE_ADDR__    0x00030000 /* SPI flash 192KB address */

#ifdef __DEMO_160x128__
#define __DEMO_TS_WIDTH__       160
#define __DEMO_TS_HEIGHT__      128
#else
#define __DEMO_TS_WIDTH__       320
#define __DEMO_TS_HEIGHT__      240
#endif

int Init_TouchPanel(void);
int Read_TouchPanel(int *x, int *y);
int Uninit_TouchPanel(void);
int Check_TouchPanel(void);
#endif

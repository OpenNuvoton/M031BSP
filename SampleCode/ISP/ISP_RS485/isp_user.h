/**************************************************************************//**
 * @file     isp_user.h
 * @brief    ISP Command header file
 * @version  0x34
 * @date     14, June, 2017
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2017-2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef ISP_USER_H
#define ISP_USER_H

#define FW_VERSION 0x34

#include "fmc_user.h"
#include <string.h>

#define CMD_UPDATE_APROM      0x000000A0
#define CMD_UPDATE_CONFIG     0x000000A1
#define CMD_READ_CONFIG       0x000000A2
#define CMD_ERASE_ALL         0x000000A3
#define CMD_SYNC_PACKNO       0x000000A4
#define CMD_GET_FWVER         0x000000A6
#define CMD_RUN_APROM         0x000000AB
#define CMD_RUN_LDROM         0x000000AC
#define CMD_RESET             0x000000AD
#define CMD_CONNECT           0x000000AE
#define CMD_GET_DEVICEID      0x000000B1
#define CMD_UPDATE_DATAFLASH  0x000000C3
#define CMD_RESEND_PACKET     0x000000FF

#define V6M_AIRCR_VECTKEY_DATA    0x05FA0000UL
#define V6M_AIRCR_SYSRESETREQ     0x00000004UL

// targetdev.c
extern void GetDataFlashInfo(uint32_t *addr, uint32_t *size);
extern uint32_t GetApromSize(void);

// isp_user.c
extern int ParseCmd(unsigned char *buffer, uint8_t len);
extern uint32_t g_apromSize, g_dataFlashAddr, g_dataFlashSize;

#ifdef __ICCARM__
#pragma data_alignment=4
extern uint8_t response_buff[64];
extern uint8_t usb_rcvbuf[];
#else
extern uint8_t response_buff[64] __attribute__((aligned (4)));
extern uint8_t usb_rcvbuf[] __attribute__((aligned (4)));
#endif

extern volatile uint8_t bISPDataReady;



#endif  // #ifndef ISP_USER_H

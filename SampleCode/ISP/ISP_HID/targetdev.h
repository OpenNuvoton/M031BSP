/***************************************************************************//**
 * @file     targetdev.h
 * @brief    ISP support function header file
 * @version  0x32
 * @date     14, June, 2017
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2017-2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "NuMicro.h"
#include "isp_user.h"

#define DetectPin                   PB12

/* rename for uart_transfer.c */
#define UART_N                          UART0
#define UART_N_IRQHandler       UART02_IRQHandler
#define UART_N_IRQn                 UART02_IRQn

#define CONFIG_SIZE 8 // in bytes

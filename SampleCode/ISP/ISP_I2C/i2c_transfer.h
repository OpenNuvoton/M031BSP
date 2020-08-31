/******************************************************************************
 * @file     i2c_transfer.h
 * @brief    I2C ISP slave header file
 * @version  1.0.0
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __I2C_TRANS_H__
#define __I2C_TRANS_H__
#include <stdint.h>

extern volatile uint8_t bI2cDataReady;
extern uint8_t i2c_rcvbuf[];

/*-------------------------------------------------------------*/
void I2C_Init(void);

#endif  /* __I2C_TRANS_H__ */

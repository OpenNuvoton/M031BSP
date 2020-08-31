/******************************************************************************
 * @file     spi_transfer.h
 * @brief    SPI ISP slave header file
 * @version  1.0.0
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __SPI_TRANS_H__
#define __SPI_TRANS_H__
#include <stdint.h>

extern volatile uint8_t bSpiDataReady;
extern uint32_t spi_rcvbuf[];

/*-------------------------------------------------------------*/
void SPI_Init(void);

#endif  /* __SPI_TRANS_H__ */

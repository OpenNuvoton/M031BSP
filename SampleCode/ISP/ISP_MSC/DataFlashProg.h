/******************************************************************************
 * @file     DataFlashProg.h
 * @brief    M031 series data flash programming driver header
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __DATA_FLASH_PROG_H__
#define __DATA_FLASH_PROG_H__

#define BUFFER_PAGE_SIZE    512

#define BYTE_PER_SEC        512
#define RSVD_SEC_CNT        2
#define ROOT_ENT_CNT        32
#define NUM_FAT             2
#define FAT_SZ              7
#define DATA_SEC_ADDR       ((RSVD_SEC_CNT + FAT_SZ * NUM_FAT + ROOT_ENT_CNT) * BYTE_PER_SEC)

#endif  /* __DATA_FLASH_PROG_H__ */

/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/

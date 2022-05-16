/******************************************************************************
 * @file     DataFlashProg.h
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 18/04/03 10:46a $
 * @brief    Data flash programming driver header
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __DATA_FLASH_PROG_H__
#define __DATA_FLASH_PROG_H__

#define MASS_STORAGE_OFFSET       0x00008000 /* To avoid the code to write APROM */
#define DATA_FLASH_STORAGE_SIZE   (32*1024)  /* Configure the DATA FLASH storage size. To pass USB-IF MSC Test, it needs > 64KB */
/* Windows will consume about 20KB for file system formating. */
#define BUFFER_PAGE_SIZE          512

#define FLASH_PAGE_SIZE           FMC_FLASH_PAGE_SIZE

#endif  /* __DATA_FLASH_PROG_H__ */

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

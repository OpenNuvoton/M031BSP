/******************************************************************************
 * @file     DataFlashProg.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 18/04/03 10:46a $
 * @brief    M031 Series Data Flash Access API
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

/*---------------------------------------------------------------------------------------------------------*/
/* Includes of system headers                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "DataFlashProg.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

uint32_t g_sectorBuf[FLASH_PAGE_SIZE/4];

uint32_t FMC_ReadPage(uint32_t u32StartAddr, uint32_t * u32Buf)
{
    uint32_t i;

    for (i = 0; i < FLASH_PAGE_SIZE/4; i++)
        u32Buf[i] = FMC_Read(u32StartAddr + i*4);

    return 0;
}

void DataFlashRead(uint32_t addr, uint32_t size, uint32_t buffer)
{
    /* This is low level read function of USB Mass Storage */
    int32_t len;

    /* Modify the address to MASS_STORAGE_OFFSET */
    addr += MASS_STORAGE_OFFSET;

    len = (int32_t)size;

    SYS_UnlockReg();
    FMC_Open();

    while(len >= FLASH_PAGE_SIZE)
    {
        FMC_ReadPage(addr, (uint32_t *)buffer);
        addr   += FLASH_PAGE_SIZE;
        buffer += FLASH_PAGE_SIZE;
        len  -= FLASH_PAGE_SIZE;
    }

    FMC_Close();
    SYS_LockReg();
}


uint32_t FMC_ProgramPage(uint32_t u32StartAddr, uint32_t * u32Buf)
{
    uint32_t i;

    for (i = 0; i < FLASH_PAGE_SIZE/4; i++)
    {
        FMC_Write(u32StartAddr + i*4, u32Buf[i]);
    }

    return 0;
}


void DataFlashWrite(uint32_t addr, uint32_t size, uint32_t buffer)
{
    /* This is low level write function of USB Mass Storage */
    int32_t len, i, offset;
    uint32_t *pu32;
    uint32_t alignAddr;

    /* Modify the address to MASS_STORAGE_OFFSET */
    addr += MASS_STORAGE_OFFSET;

    len = (int32_t)size;

    SYS_UnlockReg();
    FMC_Open();

    if ( len == FLASH_PAGE_SIZE && ((addr & (FLASH_PAGE_SIZE-1)) == 0) )
    {
        FMC_Erase(addr);

        while (len >= FLASH_PAGE_SIZE)
        {
            FMC_ProgramPage(addr, (uint32_t *) buffer);
            len    -= FLASH_PAGE_SIZE;
            buffer += FLASH_PAGE_SIZE;
            addr   += FLASH_PAGE_SIZE;
        }
    }
    else
    {
        do
        {
            alignAddr = addr & 0x1FE00;

            /* Get the sector offset*/
            offset = ( addr & (FLASH_PAGE_SIZE-1) );

            if ( offset || (size < FLASH_PAGE_SIZE) )
            {
                /* Non 4k alignment. Note: It needs to avoid add MASS_STORAGE_OFFSET twice. */
                DataFlashRead(alignAddr - MASS_STORAGE_OFFSET, FLASH_PAGE_SIZE, (uint32_t)&g_sectorBuf[0]);

            }

            /* Update the data */
            pu32 = (uint32_t *)buffer;
            len = FLASH_PAGE_SIZE - offset;
            if (size < len)
                len = size;

            for (i=0; i<len/4; i++)
            {
                g_sectorBuf[offset/4 + i] = pu32[i];
            }

            FMC_Erase(alignAddr);

            for(i=0; i<16; i++)
            {
                FMC_ProgramPage(alignAddr + (i << 8), (uint32_t *) g_sectorBuf + (i << 8));
            }

            size -= len;
            addr += len;
            buffer += len;

        }
        while (size > 0);
    }

    FMC_Close();
    SYS_LockReg();
}


/******************************************************************************//**
 * @file     DataFlashProg.c
 * @version  V1.00
 * @brief    M031 Series Data Flash Access API
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*---------------------------------------------------------------------------------------------------------*/
/* Includes of system headers                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#include "M031Series_User.h"
#include "DataFlashProg.h"
#include "massstorage.h"

static uint8_t g_u8LockBit = 1;

extern uint32_t g_u32u32StorageSize;

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

uint8_t u8FormatData[62] =
{
    0xEB, 0x3C, 0x90, 0x4D, 0x53, 0x44, 0x4F, 0x53,
    0x35, 0x2E, 0x30, 0x00, 0x02, 0x01, 0x02, 0x00,
    0x02, 0x00, 0x02, 0x80, 0x00, 0xF8, 0x07, 0x00,
    0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0xB9,
    0xC1, 0xAA, 0x42, 0x4E, 0x4F, 0x20, 0x4E, 0x41,
    0x4D, 0x45, 0x20, 0x20, 0x20, 0x20, 0x46, 0x41,
    0x54, 0x31, 0x32, 0x20, 0x20, 0x20
};

uint8_t u8RootDirData[92] =
{
    0x42, 0x20, 0x00, 0x49, 0x00, 0x6E, 0x00, 0x66,
    0x00, 0x6F, 0x00, 0x0F, 0x00, 0x72, 0x72, 0x00,
    0x6D, 0x00, 0x61, 0x00, 0x74, 0x00, 0x69, 0x00,
    0x6F, 0x00, 0x00, 0x00, 0x6E, 0x00, 0x00, 0x00,
    0x01, 0x53, 0x00, 0x79, 0x00, 0x73, 0x00, 0x74,
    0x00, 0x65, 0x00, 0x0F, 0x00, 0x72, 0x6D, 0x00,
    0x20, 0x00, 0x56, 0x00, 0x6F, 0x00, 0x6C, 0x00,
    0x75, 0x00, 0x00, 0x00, 0x6D, 0x00, 0x65, 0x00,
    0x53, 0x59, 0x53, 0x54, 0x45, 0x4D, 0x7E, 0x31,
    0x20, 0x20, 0x20, 0x16, 0x00, 0x99, 0x0D, 0x5C,
    0x6D, 0x43, 0x6D, 0x43, 0x00, 0x00, 0x0E, 0x5C,
    0x6D, 0x43, 0x02, 0x00
};


void DataFlashRead(uint32_t u32Addr, uint32_t * u32Buf)
{
    USBD_MemReset((uint8_t *)u32Buf, STORAGE_BUFFER_SIZE);

    if (u32Addr == 0x00000000)
    {
        uint32_t u32Sectors = g_u32u32StorageSize / UDC_SECTOR_SIZE;

        u8FormatData[19] = u32Sectors & 0xFF;
        u8FormatData[20] = (uint8_t)(u32Sectors >> 8);

        USBD_MemCopy((uint8_t *)u32Buf, u8FormatData, sizeof(u8FormatData));
        u32Buf[STORAGE_BUFFER_SIZE/4-1] = 0xAA550000;
    }
    else if ((u32Addr == (RSVD_SEC_CNT * BYTE_PER_SEC)) || (u32Addr == ((RSVD_SEC_CNT + FAT_SZ) * BYTE_PER_SEC)))
    {
        u32Buf[0] = 0x00FFFFF8;
    }
    else if (u32Addr == ((RSVD_SEC_CNT + FAT_SZ * NUM_FAT) * BYTE_PER_SEC)) /* root dir */
    {
        USBD_MemCopy((uint8_t *)u32Buf, u8RootDirData, sizeof(u8RootDirData));
    }
}


void FMC_ISP(uint32_t u32Cmd, uint32_t u32Addr, uint32_t u32Data)
{
    FMC->ISPCMD = u32Cmd;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = u32Data;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    //__ISB();
    //while(FMC->ISPTRG);
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }
    
    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {    
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        while(1);
    }
}


uint32_t FMC_Init(void)
{
    uint32_t u32APSize;

#if 0
    FMC_ISP(FMC_ISPCMD_READ_DID, 0, 0);

    u32ROMSEL = (FMC->ISPDAT >> 4) & 0x0F;

    u32APSize = (1 << (u32ROMSEL + 9));

    if (u32ROMSEL < 8)
        u32APSize += 0x2000;
#endif
    u32APSize=512*1024;
    FMC_ISP(FMC_ISPCMD_READ, FMC_CONFIG_BASE, 0);

    g_u8LockBit = (FMC->ISPDAT & 0x02);

    return u32APSize;
}


void DataFlashProgramPage(uint32_t u32StartAddr, uint32_t * u32Buf)
{
    uint32_t i;

    for(i = 0; i < STORAGE_BUFFER_SIZE / 4; i++)
    {
        FMC_ISP(FMC_ISPCMD_PROGRAM, u32StartAddr + i * 4, u32Buf[i]);
    }
}


void DataFlashWrite(uint32_t u32Addr, uint32_t * u32Buf)
{
    /* This is low level write function of USB Mass Storage */
    if ((u32Addr >= DATA_SEC_ADDR) && (u32Addr < g_u32u32StorageSize))
    {
        u32Addr -= DATA_SEC_ADDR;

        if (g_u8LockBit)
        {
            if ((u32Addr & (FMC_FLASH_PAGE_SIZE - 1)) == 0)
                FMC_ISP(FMC_ISPCMD_PAGE_ERASE, u32Addr, 0);
        }
        /* For Security Lock */
        else if (u32Addr == 0)
        {
            uint32_t i;

            for (i = 0; i < (g_u32u32StorageSize - DATA_SEC_ADDR); i += FMC_FLASH_PAGE_SIZE)
            {
                FMC_ISP(FMC_ISPCMD_PAGE_ERASE, i, 0);
            }
        }

        DataFlashProgramPage(u32Addr, u32Buf);
    }
}



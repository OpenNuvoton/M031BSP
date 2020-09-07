/***************************************************************************//**
 * @file     targetdev.c
 * @brief    ISP support function source file
 * @version  0x32
 * @date     14, June, 2017
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "NuMicro.h"
#include "ISP_USER.h"
/*---------------------------------------------------------------------------------------------------------*/
/*  Get APROM size                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t GetApromSize()
{
	unsigned int size = 0x4000, data;
    int result;

    do
    {
        result = FMC_Read_User(size, &data);
        if(result < 0)
        {
            return size;
        }
        else
            size *= 2;
    }
    while(1);
}
#include "ISP_USER.h"
/*---------------------------------------------------------------------------------------------------------*/
/*  Get DataFlash start address and size                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void GetDataFlashInfo(uint32_t *addr, uint32_t *size)
{
	unsigned int uData;

    *size = 0;

    FMC_Read_User(Config0, &uData);
    if((uData&0x01) == 0)   //DFEN enable
    {
        FMC_Read_User(Config1, &uData);
        if(uData > g_apromSize || (uData & 0x1FF))//avoid config1 value from error
            uData = g_apromSize;

        *addr = uData;
        *size = g_apromSize - uData;
    }
    else
    {
        *addr = g_apromSize;
        *size = 0;
    }
}


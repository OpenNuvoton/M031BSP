/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Implement a code and execute in SRAM to program embedded Flash.
 *           (Support KEIL MDK/IAR Embedded Workbench/Eclipse GCC Only)
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define APROM_TEST_BASE             0x3000
#define TEST_PATTERN                0x5A5A5A5A

/**
 * @brief      CPU access flash code on SRAM
 *
 * @retval      0  Access successful
 * @return     -1  Access fail.
 *
 * @detail     If the low level APIs such as FMC_Erase(), FMC_Write() and FMC_Read()
 *             also will be located on SRAM. Modification fmc.c to place these APIs
 *             with section name - "fasecode" like the example code.
 *
 */
#if defined ( __CC_ARM )
#pragma arm section code = "fastcode"
int32_t FlashAccess_OnSRAM(void)

#elif defined ( __ICCARM__ )
int32_t FlashAccess_OnSRAM(void) @ "fastcode"

#elif defined ( __GNUC__ )
#pragma GCC push_options
#pragma GCC optimize ("O0")
__attribute__ ((used, section("fastcode"))) int32_t FlashAccess_OnSRAM(void)

#else
int32_t FlashAccess_OnSRAM(void)
	
#endif
{
    uint32_t u32Data, u32RData;
    uint32_t u32Addr = APROM_TEST_BASE;
    uint32_t u32Cnt;

    for (u32Cnt = 0; u32Cnt < 0x1000; u32Cnt += 4)
    {
        /* Erase Demo */
        if( ((u32Addr+u32Cnt)%FMC_FLASH_PAGE_SIZE) == 0)
        {
            FMC_Erase(u32Addr+u32Cnt);
            if (g_FMC_i32ErrCode != 0)
            {
                printf("FMC_Erase failed!\n");
                return -1;
            }
        }
        /* Write Demo */
        u32Data = u32Cnt + 0x12345678;
        FMC_Write(u32Addr + u32Cnt, u32Data);
        if (g_FMC_i32ErrCode != 0)
        {
            printf("FMC_Write failed!\n");
            return -1;
        }
        if ((u32Cnt & 0xf) == 0)
            printf(".");

        /* Read Demo */
        u32RData = FMC_Read(u32Addr + u32Cnt);

        if (g_FMC_i32ErrCode != 0)
        {
            printf("FMC_Read failed!\n");
            return -1;
        }
        if (u32Data != u32RData)
        {
            printf("[Read/Write FAIL]\n");
            return -1;
        }
    }
    printf("\nISP function run at SRAM finished\n");
    return 0;
}
#if defined ( __CC_ARM )
#pragma arm section

#elif defined ( __GNUC__ )
#pragma GCC pop_options

#endif
/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

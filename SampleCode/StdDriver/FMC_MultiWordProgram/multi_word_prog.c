/****************************************************************************//**
 * @file     multi_word_prog.c
 * @version  V0.10
 * @brief    This sample run on SRAM to show FMC multi word program function.
 *
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>

#include "NuMicro.h"

#define PAGE_SIZE       0x800
uint32_t    page_buff[PAGE_SIZE / 4];

#if defined ( __CC_ARM )
#pragma arm section code="fastcode"
int32_t multi_word_program(void)

#elif defined ( __ICCARM__ )
int32_t multi_word_program(void) @ "fastcode"

#elif defined ( __GNUC__ )
#pragma GCC push_options
#pragma GCC optimize ("O0")
__attribute__ ((used, section("fastcode"))) int32_t multi_word_program(void)

#else
int32_t multi_word_program(void)
#endif
{
    uint32_t i, addr, maddr;

    for(addr = 0x10000; addr < 0x20000; addr += FMC_FLASH_PAGE_SIZE)
    {
        printf("Multiword program APROM page 0x%x =>\n", addr);

        if(FMC_Erase(addr) < 0)
        {
            printf("    Erase failed!!\n");
            goto err_out;
        }

        printf("    Program...\n");

        for(maddr = addr; maddr < addr + FMC_FLASH_PAGE_SIZE; maddr += FMC_MULTI_WORD_PROG_LEN)
        {
            /* Prepare test pattern */
            for(i = 0; i < FMC_MULTI_WORD_PROG_LEN; i += 4)
                page_buff[i / 4] = maddr + i;

            i = FMC_WriteMultiple(maddr, page_buff, FMC_MULTI_WORD_PROG_LEN);
            if(i <= 0)
            {
                printf("FMC_WriteMultiple failed: %d\n", i);
                goto err_out;
            }
            printf("programmed length = %d\n", i);

        }
        printf("    [OK]\n");

        printf("    Verify...");

        for(i = 0; i < FMC_FLASH_PAGE_SIZE; i += 4)
            page_buff[i / 4] = addr + i;

        for(i = 0; i < FMC_FLASH_PAGE_SIZE; i += 4)
        {
            if(FMC_Read(addr + i) != page_buff[i / 4])
            {
                printf("\n[FAILED] Data mismatch at address 0x%x, expect: 0x%x, read: 0x%x!\n", addr + i, page_buff[i / 4], FMC_Read(addr + i));
                goto err_out;
            }
            if (g_FMC_i32ErrCode != 0)
            {
                printf("FMC_Read address 0x%x failed!\n", addr+i);
                goto err_out;
            }
        }
        printf("[OK]\n");
    }
    return 0;
err_out:
    return -1;
}

#if defined ( __CC_ARM )
#pragma arm section
#elif defined ( __GNUC__ )
#pragma GCC pop_options
#endif

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

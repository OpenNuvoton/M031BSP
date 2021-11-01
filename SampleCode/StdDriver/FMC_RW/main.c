/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 13 $
 * $Date: 18/07/18 3:19p $
 * @brief    Show FMC read flash IDs, erase, read, and write functions.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define APROM_TEST_BASE             0x3000
#define DATA_FLASH_TEST_BASE        0x3000
#define DATA_FLASH_TEST_END         0x4000      /* 16KB */
#define TEST_PATTERN                0x5A5A5A5A

uint32_t APROM_TEST_END  = 0x00004000UL;        /* 16KB */
uint32_t LDROM_TEST_SIZE = 0x00000800UL;        /*  2KB */
uint32_t LDROM_TEST_END  = 0x00100800UL;

int IsDebugFifoEmpty(void);

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))
                    |(SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);


    /* Lock protected registers */
    SYS_LockReg();
}

static int  set_data_flash_base(uint32_t u32DFBA)
{
    uint32_t   au32Config[2];

    /* Read User Configuration 0 & 1 */
    if (FMC_ReadConfig(au32Config, 2) < 0)
    {
        printf("\nRead User Config failed!\n");       /* Error message */
        return -1;                     /* failed to read User Configuration */
    }

    /* Check if Data Flash is enabled (CONFIG0[0]) and is expected address (CONFIG1) */
    if ((!(au32Config[0] & 0x1)) && (au32Config[1] == u32DFBA))
        return 0;                      /* no need to modify User Configuration */

    FMC_ENABLE_CFG_UPDATE();           /* Enable User Configuration update. */

    /* Erase User Configuration */
    FMC_Erase(FMC_CONFIG_BASE);

    if(u32DFBA == 0xFFFFFFFF)
        au32Config[0] |=  0x1;         /* CONFIG0[0] = 1 (Disabled) */
    else
        au32Config[0] &= ~0x1;         /* CONFIG0[0] = 0 (Enabled) */

    au32Config[1] = u32DFBA;

    /* Update User Configuration settings. */
    if (FMC_WriteConfig(au32Config, 2) < 0)
        return -1;

    FMC_DISABLE_CFG_UPDATE();

    if(u32DFBA == 0xFFFFFFFF)
        printf("\nDisable Data Flash\n");
    else
        printf("\nSet Data Flash base as 0x%x.\n", u32DFBA);

    /* To check if all the debug messages are finished */
    while(!IsDebugFifoEmpty());

    if(u32DFBA != 0xFFFFFFFF)
    {
        /* Perform chip reset to make new User Config take effect */
        SYS->IPRST0 = SYS_IPRST0_CHIPRST_Msk;
    }
    return 0;
}


void run_crc32_checksum()
{
    uint32_t    chksum;

    chksum = FMC_GetChkSum(FMC_APROM_BASE, DATA_FLASH_TEST_BASE - FMC_APROM_BASE);

    printf("  APROM CRC32 checksum .................. [0x%08x]\n", chksum);

    chksum = FMC_GetChkSum(DATA_FLASH_TEST_BASE, APROM_TEST_END - DATA_FLASH_TEST_BASE);

    printf("  Data Flash CRC32 checksum ............. [0x%08x]\n", chksum);

    chksum = FMC_GetChkSum(FMC_LDROM_BASE, LDROM_TEST_SIZE);

    printf("  LDROM CRC32 checksum .................. [0x%08x]\n", chksum);

    chksum = FMC_GetChkSum(FMC_SPROM_BASE, FMC_SPROM_SIZE);

    printf("  SPROM CRC32 checksum .................. [0x%08x]\n", chksum);
}


int32_t fill_data_pattern(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t u32Addr;                  /* flash address */

    /* Fill flash range from u32StartAddr to u32EndAddr with data word u32Pattern. */
    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        if (FMC_Write(u32Addr, u32Pattern) != 0)          /* Program flash */
        {
            printf("FMC_Write address 0x%x failed!\n", u32Addr);
            return -1;
        }
    }
    return 0;
}


int32_t  verify_data(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t    u32Addr;               /* flash address */
    uint32_t    u32data;               /* flash data    */

    /* Verify if each data word from flash u32StartAddr to u32EndAddr be u32Pattern.  */
    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        u32data = FMC_Read(u32Addr);   /* Read a flash word from address u32Addr. */

        if (g_FMC_i32ErrCode != 0)
        {
            printf("FMC_Read address 0x%x failed!\n", u32Addr);
            return -1;
        }

        if (u32data != u32Pattern)     /* Verify if data matched. */
        {
            printf("\nFMC_Read data verify failed at address 0x%x, read=0x%x, expect=0x%x\n", u32Addr, u32data, u32Pattern);
            return -1;                 /* data verify failed */
        }
    }
    return 0;                          /* data verify OK */
}


int32_t  flash_test(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t    u32Addr;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += FMC_FLASH_PAGE_SIZE)
    {
        printf("    Flash test address: 0x%x    \r", u32Addr);

        /* Erase page */
        if (FMC_Erase(u32Addr) != 0)
        {
            printf("FMC_Erase address 0x%x failed!\n", u32Addr);
            return -1;
        }

        /* Verify if page contents are all 0xFFFFFFFF */
        if (verify_data(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, 0xFFFFFFFF) < 0)
        {
            printf("\nPage 0x%x erase verify failed!\n", u32Addr);   /* error message */
            return -1;                 /* Erase verify failed */
        }

        /* Write test pattern to fill the whole page */
        if (fill_data_pattern(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, u32Pattern) < 0)
        {
            printf("Failed to write page 0x%x!\n", u32Addr);
            return -1;
        }

        /* Verify if page contents are all equal to test pattern */
        if (verify_data(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, u32Pattern) < 0)
        {
            printf("\nData verify failed!\n ");                      /* error message */
            return -1;                 /* Program verify failed */
        }

        /* Erase page */
        if (FMC_Erase(u32Addr) != 0)
        {
            printf("FMC_Erase address 0x%x failed!\n", u32Addr);
            return -1;
        }

        /* Verify if page contents are all 0xFFFFFFFF after erased. */
        if (verify_data(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, 0xFFFFFFFF) < 0)
        {
            printf("\nPage 0x%x erase verify failed!\n", u32Addr);   /* error message */
            return -1;                 /* Erase verify failed */
        }
    }
    printf("\r    Flash Test Passed.          \n");                  /* information message */
    return 0;      /* flash test passed */
}


int main()
{
    uint32_t    i, u32Data;

    /* Unlock protected registers to operate FMC ISP function */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /* Checking if target device supports the feature */
    if( (GET_CHIP_SERIES_NUM == CHIP_SERIES_NUM_I) || (GET_CHIP_SERIES_NUM == CHIP_SERIES_NUM_G) )
    {
        /* Checking if flash size matches with target device */
        if(FMC_FLASH_PAGE_SIZE != 2048)
        {
            /* FMC_FLASH_PAGE_SIZE is different from target device's */
            printf("Please enable the compiler option - PAGE_SIZE_2048 in fmc.h\n");
            while(SYS->PDID);
        }
    }
    else
    {
        if(FMC_FLASH_PAGE_SIZE != 512)
        {
            /* FMC_FLASH_PAGE_SIZE is different from target device's */
            printf("Please disable the compiler option - PAGE_SIZE_2048 in fmc.h\n");
            while(SYS->PDID);
        }
    }

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|           M031 FMC Sample Code         |\n");
    printf("+----------------------------------------+\n");

    SYS_UnlockReg();

    /* Checking if flash page size matches with target chip's */
    if( (GET_CHIP_SERIES_NUM == CHIP_SERIES_NUM_I) || (GET_CHIP_SERIES_NUM == CHIP_SERIES_NUM_G) )
    {
        if(FMC_FLASH_PAGE_SIZE != 2048)
        {
            /* FMC_FLASH_PAGE_SIZE is different from target device */
            /* Please enable the compiler option PAGE_SIZE_2048 in fmc.h */
            printf("FMC_FLASH_PAGE_SIZE is different from target device\n");
            printf("Please enable the compiler option PAGE_SIZE_2048 in fmc.h\n");
            while(SYS->PDID);
        }
    }
    else
    {
        if(FMC_FLASH_PAGE_SIZE != 512)
        {
            /* FMC_FLASH_PAGE_SIZE is different from target device */
            /* Please disable the compiler option PAGE_SIZE_2048 in fmc.h */    
            printf("FMC_FLASH_PAGE_SIZE is different from target device\n");
            printf("Please disable the compiler option PAGE_SIZE_2048 in fmc.h\n");
            while(SYS->PDID);
        }
    }

    /* Enable FMC ISP function */
    FMC_Open();

    /* Enable Data Flash and set base address. */
    if (set_data_flash_base(DATA_FLASH_TEST_BASE) < 0)
    {
        printf("Failed to set Data Flash base address!\n");
        goto lexit;
    }

    /* Read BS */
    printf("  Boot Mode ............................. ");
    if (FMC_GetBootSource() == 0)
        printf("[APROM]\n");
    else
    {
        printf("[LDROM]\n");
        printf("  WARNING: The driver sample code must execute in AP mode!\n");
        goto lexit;
    }

    u32Data = FMC_ReadCID();           /* Get company ID, should be 0xDA. */
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_ReadCID failed!\n");
        goto lexit;
    }
    printf("  Company ID ............................ [0x%08x]\n", u32Data);   /* information message */

    u32Data = FMC_ReadPID();           /* Get product ID. */
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_ReadPID failed!\n");
        goto lexit;
    }
    printf("  Product ID ............................ [0x%08x]\n", u32Data);   /* information message */

    for (i = 0; i < 3; i++)            /* Get 96-bits UID. */
    {
        u32Data = FMC_ReadUID(i);
        if (g_FMC_i32ErrCode != 0)
        {
            printf("FMC_ReadUID %d failed!\n", i);
            goto lexit;
        }
        printf("  Unique ID %d ........................... [0x%08x]\n", i, u32Data);  /* information message */
    }

    for (i = 0; i < 4; i++)            /* Get 4 words UCID. */
    {
        u32Data = FMC_ReadUCID(i);
        if (g_FMC_i32ErrCode != 0)
        {
            printf("FMC_ReadUCID %d failed!\n", i);
            goto lexit;
        }
        printf("  Unique Customer ID %d .................. [0x%08x]\n", i, u32Data);  /* information message */
    }

    /* Read User Configuration */
    printf("  User Config 0 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE));
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_Read(FMC_CONFIG_BASE) failed!\n");
        goto lexit;
    }
    /* Read User Configuration CONFIG1 */
    printf("  User Config 1 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE+4));
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_Read(FMC_CONFIG_BASE+4) failed!\n");
        goto lexit;
    }

    /* Read Data Flash base address */
    u32Data = FMC_ReadDataFlashBaseAddr();
    printf("  Data Flash Base Address ............... [0x%08x]\n", u32Data);

    run_crc32_checksum();

    printf("\n\nLDROM test =>\n");
    FMC_ENABLE_LD_UPDATE();
    if (flash_test(FMC_LDROM_BASE, LDROM_TEST_END, TEST_PATTERN) < 0)
    {
        printf("\n\nLDROM test failed!\n");
        goto lexit;
    }
    FMC_DISABLE_LD_UPDATE();

    printf("\n\nSPROM test =>\n");
    FMC_ENABLE_SP_UPDATE();
    if (flash_test(FMC_SPROM_BASE, FMC_SPROM_BASE + FMC_SPROM_SIZE - 4, TEST_PATTERN) < 0)
    {
        printf("\n\nSPROM test failed!\n");
        goto lexit;
    }
    FMC_DISABLE_SP_UPDATE();

    printf("\n\nAPROM test =>\n");     /* information message */

    FMC_ENABLE_AP_UPDATE();            /* Enable APROM update. */

    /* Execute flash program/verify test on APROM. */
    if (flash_test(APROM_TEST_BASE, DATA_FLASH_TEST_BASE, TEST_PATTERN) < 0)
    {
        printf("\n\nAPROM test failed!\n");        /* debug message */
        goto lexit;                                /* LDROM test failed. Program aborted. */
    }

    FMC_DISABLE_AP_UPDATE();           /* Disable APROM update. */

    printf("\n\nData Flash test =>\n");            /* information message */

    /* Execute flash program/verify test on Data Flash. */
    if (flash_test(DATA_FLASH_TEST_BASE, DATA_FLASH_TEST_END, TEST_PATTERN) < 0)
    {
        printf("\n\nData flash read/write test failed!\n");
        goto lexit;                    /* flash test failed */
    }

    run_crc32_checksum();

lexit:
    /* Disable Data flash */
    set_data_flash_base(0xFFFFFFFF);

    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nFMC Sample Code Completed.\n");

    while (1);
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

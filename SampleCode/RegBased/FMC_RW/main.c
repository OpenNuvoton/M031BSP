/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 12 $
 * $Date: 18/07/18 3:21p $
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
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) != CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk ) | CLK_CLKSEL0_HCLKSEL_HIRC ;

    /* Enable UART0 clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk ;

    /* Switch UART0 clock source to HIRC */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART0SEL_Msk) | CLK_CLKSEL1_UART0SEL_HIRC;

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
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = FMC_CONFIG_BASE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }
    au32Config[0] = FMC->ISPDAT;

    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = FMC_CONFIG_BASE+4UL;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }
    au32Config[1] = FMC->ISPDAT;

    /* Check if Data Flash is enabled (CONFIG0[0]) and is expected address (CONFIG1) */
    if ((!(au32Config[0] & 0x1)) && (au32Config[1] == u32DFBA))
        return 0;

    /* Update User Configuration settings. */
    FMC->ISPCTL |=  FMC_ISPCTL_CFGUEN_Msk;

    FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
    FMC->ISPADDR = FMC_CONFIG_BASE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }
    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;

    if(u32DFBA == 0xFFFFFFFF)
        au32Config[0] |=  0x1;         /* CONFIG0[0] = 1 (Disabled) */
    else
        au32Config[0] &= ~0x1;         /* CONFIG0[0] = 0 (Enabled) */

    au32Config[1] = u32DFBA;

    FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
    FMC->ISPADDR = FMC_CONFIG_BASE;
    FMC->ISPDAT = au32Config[0];
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }

    FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
    FMC->ISPADDR = FMC_CONFIG_BASE+4UL;
    FMC->ISPDAT = au32Config[1];
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }

    FMC->ISPCTL &= ~FMC_ISPCTL_CFGUEN_Msk;

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

    FMC->ISPCMD  = FMC_ISPCMD_RUN_CKS;
    FMC->ISPADDR = FMC_APROM_BASE;
    FMC->ISPDAT  = DATA_FLASH_TEST_BASE-FMC_APROM_BASE;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk) { }

    FMC->ISPCMD = FMC_ISPCMD_READ_CKS;
    FMC->ISPADDR  = FMC_APROM_BASE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk) { }

    chksum = FMC->ISPDAT;

    printf("  APROM CRC32 checksum .................. [0x%08x]\n", chksum);

    FMC->ISPCMD  = FMC_ISPCMD_RUN_CKS;
    FMC->ISPADDR = DATA_FLASH_TEST_BASE;
    FMC->ISPDAT  = APROM_TEST_END - DATA_FLASH_TEST_BASE;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk) { }

    FMC->ISPCMD = FMC_ISPCMD_READ_CKS;
    FMC->ISPADDR    = DATA_FLASH_TEST_BASE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk) { }

    chksum = FMC->ISPDAT;

    printf("  Data Flash CRC32 checksum ............. [0x%08x]\n", chksum);

    FMC->ISPCMD  = FMC_ISPCMD_RUN_CKS;
    FMC->ISPADDR = FMC_LDROM_BASE;
    FMC->ISPDAT  = LDROM_TEST_SIZE;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk) { }

    FMC->ISPCMD = FMC_ISPCMD_READ_CKS;
    FMC->ISPADDR    = FMC_LDROM_BASE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk) { }

    chksum = FMC->ISPDAT;

    printf("  LDROM CRC32 checksum .................. [0x%08x]\n", chksum);

    FMC->ISPCMD  = FMC_ISPCMD_RUN_CKS;
    FMC->ISPADDR = FMC_SPROM_BASE;
    FMC->ISPDAT  = FMC_SPROM_SIZE;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk) { }

    FMC->ISPCMD = FMC_ISPCMD_READ_CKS;
    FMC->ISPADDR    = FMC_SPROM_BASE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk) { }

    chksum = FMC->ISPDAT;

    printf("  SPROM CRC32 checksum .................. [0x%08x]\n", chksum);
}


int32_t fill_data_pattern(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t u32Addr;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
        FMC->ISPADDR = u32Addr;
        FMC->ISPDAT = u32Pattern;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }
    }
    return 0;
}


int32_t  verify_data(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t    u32Addr;
    uint32_t    u32data;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        FMC->ISPCMD = FMC_ISPCMD_READ;
        FMC->ISPADDR = u32Addr;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }

        u32data = FMC->ISPDAT;

        if (u32data != u32Pattern)
        {
            printf("\nFMC_Read data verify failed at address 0x%x, read=0x%x, expect=0x%x\n", u32Addr, u32data, u32Pattern);
            return -1;
        }
    }
    return 0;
}


int32_t  flash_test(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t    u32Addr;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += FMC_FLASH_PAGE_SIZE)
    {
        printf("    Flash test address: 0x%x    \r", u32Addr);

        // Erase page
        if (u32Addr == FMC_SPROM_BASE)
        {
            FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
            FMC->ISPADDR = FMC_SPROM_BASE;
            FMC->ISPDAT = 0x0055AA03UL;
            FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

            while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }

            if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
                FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        }
        else
        {
            FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
            FMC->ISPADDR = u32Addr;
            FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

            while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }

            if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
                FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        }

        // Verify if page contents are all 0xFFFFFFFF
        if (verify_data(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, 0xFFFFFFFF) < 0)
        {
            printf("\nPage 0x%x erase verify failed!\n", u32Addr);
            return -1;
        }

        // Write test pattern to fill the whole page
        if (fill_data_pattern(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, u32Pattern) < 0)
        {
            printf("Failed to write page 0x%x!\n", u32Addr);
            return -1;
        }

        // Verify if page contents are all equal to test pattern
        if (verify_data(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, u32Pattern) < 0)
        {
            printf("\nData verify failed!\n ");
            return -1;
        }

        if (u32Addr == FMC_SPROM_BASE)
        {
            FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
            FMC->ISPADDR = FMC_SPROM_BASE;
            FMC->ISPDAT = 0x0055AA03UL;
            FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

            while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }

            if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
                FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        }
        else
        {
            FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
            FMC->ISPADDR = u32Addr;
            FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

            while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }

            if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
                FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        }

        // Verify if page contents are all 0xFFFFFFFF
        if (verify_data(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, 0xFFFFFFFF) < 0)
        {
            printf("\nPage 0x%x erase verify failed!\n", u32Addr);
            return -1;
        }
    }
    printf("\r    Flash Test Passed.          \n");
    return 0;
}


int main()
{
    uint32_t    i, u32Data;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    /* Reset UART0 */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

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
    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk;

    /* Enable Data Flash and set base address. */
    if (set_data_flash_base(DATA_FLASH_TEST_BASE) < 0)
    {
        printf("Failed to set Data Flash base address!\n");
        goto lexit;
    }

    /* Read BS */
    printf("  Boot Mode ............................. ");
    if ((FMC->ISPCTL & FMC_ISPCTL_BS_Msk) == 0)
        printf("[APROM]\n");
    else
    {
        printf("[LDROM]\n");
        printf("  WARNING: The driver sample code must execute in AP mode!\n");
        goto lexit;
    }

    /* FMC_ReadCID */
    FMC->ISPCMD = FMC_ISPCMD_READ_CID;           /* Set ISP Command Code */
    FMC->ISPADDR = 0x0u;                         /* Must keep 0x0 when read CID */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;          /* Trigger to start ISP procedure */
#if ISBEN
    __ISB();
#endif                                           /* To make sure ISP/CPU be Synchronized */
    while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) {} /* Waiting for ISP Done */

    u32Data = FMC->ISPDAT;

    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    /* FMC_ReadPID */
    FMC->ISPCMD = FMC_ISPCMD_READ_DID;          /* Set ISP Command Code */
    FMC->ISPADDR = 0x04u;                       /* Must keep 0x4 when read PID */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;         /* Trigger to start ISP procedure */
#if ISBEN
    __ISB();
#endif                                          /* To make sure ISP/CPU be Synchronized */
    while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) {} /* Waiting for ISP Done */

    u32Data = FMC->ISPDAT;

    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    for (i = 0; i < 3; i++)
    {
        /* FMC_ReadUID */
        FMC->ISPCMD = FMC_ISPCMD_READ_UID;
        FMC->ISPADDR = ((uint32_t)i << 2u);
        FMC->ISPDAT = 0u;
        FMC->ISPTRG = 0x1u;
#if ISBEN
        __ISB();
#endif
        while(FMC->ISPTRG) {}

        u32Data = FMC->ISPDAT;
        printf("  Unique ID %d ........................... [0x%08x]\n", i, u32Data);
    }

    for (i = 0; i < 4; i++)
    {
        /* FMC_ReadUCID */
        FMC->ISPCMD = FMC_ISPCMD_READ_UID;            /* Set ISP Command Code */
        FMC->ISPADDR = (0x04u * i) + 0x10u;    /* The UCID is at offset 0x10 with word alignment. */
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;           /* Trigger to start ISP procedure */
#if ISBEN
        __ISB();
#endif                                            /* To make sure ISP/CPU be Synchronized */
        while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) {}  /* Waiting for ISP Done */

        u32Data = FMC->ISPDAT;

        printf("  Unique Customer ID %d .................. [0x%08x]\n", i, u32Data);
    }
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = FMC_CONFIG_BASE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }
    /* Read User Configuration */
    printf("  User Config 0 ......................... [0x%08x]\n", FMC->ISPDAT);
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = FMC_CONFIG_BASE+4;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }
    printf("  User Config 1 ......................... [0x%08x]\n", FMC->ISPDAT);

    /* Read Data Flash base address */
    u32Data = FMC->DFBA;
    printf("  Data Flash Base Address ............... [0x%08x]\n", u32Data);

    run_crc32_checksum();

    printf("\n\nLDROM test =>\n");
    FMC->ISPCTL |=  FMC_ISPCTL_LDUEN_Msk;
    if (flash_test(FMC_LDROM_BASE, LDROM_TEST_END, TEST_PATTERN) < 0)
    {
        printf("\n\nLDROM test failed!\n");
        goto lexit;
    }
    FMC->ISPCTL &= ~FMC_ISPCTL_LDUEN_Msk;

    printf("\n\nSPROM test =>\n");
    FMC->ISPCTL |=  FMC_ISPCTL_SPUEN_Msk;
    if (flash_test(FMC_SPROM_BASE, FMC_SPROM_BASE + FMC_SPROM_SIZE - 4, TEST_PATTERN) < 0)
    {
        printf("\n\nSPROM test failed!\n");
        goto lexit;
    }
    FMC->ISPCTL &= ~FMC_ISPCTL_SPUEN_Msk;

    printf("\n\nAPROM test =>\n");
    FMC->ISPCTL |=  FMC_ISPCTL_APUEN_Msk;
    if (flash_test(APROM_TEST_BASE, DATA_FLASH_TEST_BASE, TEST_PATTERN) < 0)
    {
        printf("\n\nAPROM test failed!\n");
        goto lexit;
    }
    FMC->ISPCTL &= ~FMC_ISPCTL_APUEN_Msk;

    printf("\n\nData Flash test =>\n");
    if (flash_test(DATA_FLASH_TEST_BASE, DATA_FLASH_TEST_END, TEST_PATTERN) < 0)
    {
        printf("\n\nData flash read/write test failed!\n");
        goto lexit;
    }

    run_crc32_checksum();

lexit:
    /* Disable Data flash */
    set_data_flash_base(0xFFFFFFFF);

    /* Disable FMC ISP function */
    FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk;

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nFMC Sample Code Completed.\n");

    while (1);
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

/******************************************************************************
 * @file     APROM_main.c
 * @version  V1.00
 * $Revision: 9 $
 * $Date: 18/07/13 3:58p $
 * @brief    This sample code includes LDROM image (fmc_ld_iap)
 *           and APROM image (fmc_ap_main).
 *           It shows how to branch between APROM and LDROM. To run
 *           this sample code, the boot mode must be "Boot from APROM
 *           with IAP".
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>

#include "NuMicro.h"

typedef void (FUNC_PTR)(void);

extern uint32_t  loaderImage1Base, loaderImage1Limit;

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

/**
  * @brief    Check User Configuration CONFIG0 bit 6 IAP boot setting. If it's not boot with IAP
  *           mode, modify it and execute a chip reset to make it take effect.
  * @return   Is boot with IAP mode or not.
  * @retval   0   Success.
  * @retval   -1  Failed on reading or programming User Configuration.
  */
static int  set_IAP_boot_mode(void)
{
    uint32_t  au32Config[2];

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

    if (au32Config[0] & 0x40)
    {
        FMC->ISPCTL |=  FMC_ISPCTL_CFGUEN_Msk;
        au32Config[0] &= ~0x40;

        FMC->ISPCTL |=  FMC_ISPCTL_CFGUEN_Msk;

        FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
        FMC->ISPADDR = FMC_CONFIG_BASE;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }
        if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
            FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;

        FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
        FMC->ISPADDR = FMC_CONFIG_BASE+4UL;
        FMC->ISPDAT = au32Config[1];
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }

        FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
        FMC->ISPADDR = FMC_CONFIG_BASE;
        FMC->ISPDAT = au32Config[0];
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }

        FMC->ISPCTL &= ~FMC_ISPCTL_CFGUEN_Msk;

        // Perform chip reset to make new User Config take effect
        SYS->IPRST0 = SYS_IPRST0_CHIPRST_Msk;
    }
    return 0;
}

/**
  * @brief    Load an image to specified flash address. The flash area must have been enabled by
  *           caller. For example, if caller want to program an image to LDROM, FMC_ENABLE_LD_UPDATE()
  *           must be called prior to calling this function.
  * @return   Image is successfully programmed or not.
  * @retval   0   Success.
  * @retval   -1  Program/verify failed.
  */
static int  load_image_to_flash(uint32_t image_base, uint32_t image_limit, uint32_t flash_addr, uint32_t max_size)
{
    uint32_t   i, j, u32Data, u32ImageSize, *pu32Loader;

    u32ImageSize = max_size;

    printf("Program image to flash address 0x%x...", flash_addr);

    /*
     * program the whole image to specified flash area
     */
    pu32Loader = (uint32_t *)image_base;
    for (i = 0; i < u32ImageSize; i += FMC_FLASH_PAGE_SIZE)
    {
        FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
        FMC->ISPADDR = flash_addr + i;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }

        if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
            FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;

        for (j = 0; j < FMC_FLASH_PAGE_SIZE; j += 4)
        {
            FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
            FMC->ISPADDR = flash_addr + i + j;
            FMC->ISPDAT = pu32Loader[(i + j) / 4];
            FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
            while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }
        }
    }
    printf("OK.\nVerify ...");

    /* Verify loader */
    for (i = 0; i < u32ImageSize; i += FMC_FLASH_PAGE_SIZE)
    {
        for (j = 0; j < FMC_FLASH_PAGE_SIZE; j += 4)
        {
            FMC->ISPCMD = FMC_ISPCMD_READ;
            FMC->ISPADDR = flash_addr + i + j;
            FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
            while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }

            u32Data = FMC->ISPDAT;
            if (u32Data != pu32Loader[(i+j)/4])
            {
                printf("data mismatch on 0x%x, [0x%x], [0x%x]\n", flash_addr + i + j, u32Data, pu32Loader[(i+j)/4]);
                return -1;
            }

            if (i + j >= u32ImageSize)
                break;
        }
    }
    printf("OK.\n");
    return 0;
}


int main()
{
    uint8_t     u8Item;
    uint32_t    u32Data;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;


    printf("\r\n\n\n");
    printf("+----------------------------------------+\n");
    printf("|        M031 FMC IAP Sample Code        |\n");
    printf("|              [APROM code]              |\n");
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

    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk;

    /*
     *  Check if User Configuration CBS is boot with IAP mode.
     *  If not, modify it.
     */
    if (set_IAP_boot_mode() < 0)
    {
        printf("Failed to set IAP boot mode!\n");
        goto lexit;
    }

    /* Read BS */
    printf("  Boot Mode ............................. ");
    if ((FMC->ISPCTL & FMC_ISPCTL_BS_Msk) == 0)
        printf("[APROM]\n");
    else
    {
        printf("[LDROM]\n");
        printf("  WARNING: The sample code must execute in APROM!\n");
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

    /* Read User Configuration */
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = FMC_CONFIG_BASE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }
    printf("  User Config 0 ......................... [0x%08x]\n", FMC->ISPDAT);
    /* Read User Configuration CONFIG1 */
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = FMC_CONFIG_BASE+4;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }
    printf("  User Config 1 ......................... [0x%08x]\n", FMC->ISPDAT);

    do
    {
        printf("\n\n\n");
        printf("+----------------------------------------+\n");
        printf("|               Select                   |\n");
        printf("+----------------------------------------+\n");
        printf("| [0] Load IAP code to LDROM             |\n");
        printf("| [1] Run IAP program (in LDROM)         |\n");
        printf("+----------------------------------------+\n");
        printf("Please select...");
        u8Item = getchar();
        printf("%c\n", u8Item);

        switch (u8Item)
        {
        case '0':
            /* Enable LDROM update capability */
            FMC->ISPCTL |=  FMC_ISPCTL_LDUEN_Msk;
            /*
             *  The binary image of LDROM code is embedded in this sample.
             *  load_image_to_flash() will program this LDROM code to LDROM.
             */
            if (load_image_to_flash((uint32_t)&loaderImage1Base, (uint32_t)&loaderImage1Limit,
                                    FMC_LDROM_BASE, (uint32_t)&loaderImage1Limit - (uint32_t)&loaderImage1Base) != 0)
            {
                printf("Load image to LDROM failed!\n");
                goto lexit;            /* Load LDROM code failed. Program aborted. */
            }
            /* Disable LDROM update capability */
            FMC->ISPCTL &= ~FMC_ISPCTL_LDUEN_Msk;
            break;

        case '1':
            printf("\n\nChange VECMAP and branch to LDROM...\n");
            printf("LDROM code SP = 0x%x\n", *(uint32_t *)(FMC_LDROM_BASE));
            printf("LDROM code ResetHandler = 0x%x\n", *(uint32_t *)(FMC_LDROM_BASE+4));
            /* To check if all the debug messages are finished */
            while((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) == 0);

            /*  NOTE!
             *     Before change VECMAP, user MUST disable all interrupts.
             *     The following code CANNOT locate in address 0x0 ~ 0x200.
             */

            /* FMC_SetVectorPageAddr(FMC_LDROM_BASE) */
            FMC->ISPCMD = FMC_ISPCMD_VECMAP;
            FMC->ISPADDR = FMC_LDROM_BASE;
            FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
            while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;

            /* Software reset to boot to LDROM */
            __DSB();                                                          /* Ensure all outstanding memory accesses included
                                                                       buffered write are completed before reset */
            SCB->AIRCR  = ((0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
                 SCB_AIRCR_SYSRESETREQ_Msk);
            __DSB();                                                          /* Ensure completion of memory access */

            for(;;)                                                           /* wait until reset */
            {
                __NOP();
            }

        default :
            /* invalid selection */
            continue;
        }
    }
    while (1);


lexit:

    /* Disable FMC ISP function */

    FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk;

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nFMC Sample Code Completed.\n");

    while (1);
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

/******************************************************************************
 * @file     APROM_main.c
 * @version  V1.00
 * $Revision: 13 $
 * $Date: 18/07/16 11:44a $
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

void UART_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure UART and set UART Baudrate */
    UART_Open(UART0, 115200);
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

    if (FMC_ReadConfig(au32Config, 2) < 0)
    {
        printf("\nRead User Config failed!\n");
        return -1;
    }

    /*
        CONFIG0[7:6]
        00 = Boot from LDROM with IAP mode.
        01 = Boot from LDROM without IAP mode.
        10 = Boot from APROM with IAP mode.
        11 = Boot from APROM without IAP mode.
    */
    if (au32Config[0] & 0x40)          /* Check if it's boot from APROM/LDROM with IAP. */
    {
        FMC_ENABLE_CFG_UPDATE();       /* Enable User Configuration update. */
        au32Config[0] &= ~0x40;        /* Select IAP boot mode. */

        if (FMC_WriteConfig(au32Config, 2) != 0) /* Update User Configuration CONFIG0 and CONFIG1. */
        {
            printf("FMC_WriteConfig failed!\n");
            return -1;
        }
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

    u32ImageSize = max_size;           /* Give the maximum size of programmable flash area. */

    printf("Program image to flash address 0x%x...", flash_addr);    /* information message */

    /*
     * program the whole image to specified flash area
     */
    pu32Loader = (uint32_t *)image_base;
    for (i = 0; i < u32ImageSize; i += FMC_FLASH_PAGE_SIZE)
    {
        if (FMC_Erase(flash_addr + i) != 0)    /* erase a flash page */
        {
            printf("FMC_Erase address 0x%x failed!\n", flash_addr + i);
            return -1;
        }
        for (j = 0; j < FMC_FLASH_PAGE_SIZE; j += 4)                 /* program image to this flash page */
        {
            if (FMC_Write(flash_addr + i + j, pu32Loader[(i + j) / 4]) != 0)
            {
                printf("FMC_Write address 0x%x failed!\n", flash_addr + i + j);
                return -1;
            }
        }
    }
    printf("OK.\nVerify ...");

    /* Verify loader */
    for (i = 0; i < u32ImageSize; i += FMC_FLASH_PAGE_SIZE)
    {
        for (j = 0; j < FMC_FLASH_PAGE_SIZE; j += 4)
        {
            u32Data = FMC_Read(flash_addr + i + j);        /* read a word from flash memory */
            if (g_FMC_i32ErrCode != 0)
            {
                printf("FMC_Read address 0x%x failed!\n", flash_addr + i + j);
                return -1;
            }

            if (u32Data != pu32Loader[(i+j)/4])            /* check if the word read from flash be matched with original image */
            {
                printf("data mismatch on 0x%x, [0x%x], [0x%x]\n", flash_addr + i + j, u32Data, pu32Loader[(i+j)/4]);
                return -1;             /* image program failed */
            }

            if (i + j >= u32ImageSize) /* check if it reach the end of image */
                break;
        }
    }
    printf("OK.\n");
    return 0;                          /* success */
}


int main()
{
    uint8_t     u8Item;
    uint32_t    u32Data;

    SYS_Init();
    UART_Init();

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

    printf("\r\n\n\n");
    printf("+----------------------------------------+\n");
    printf("|        M031 FMC IAP Sample Code        |\n");
    printf("|              [APROM code]              |\n");
    printf("+----------------------------------------+\n");

    /* Unlock protected registers to operate FMC ISP function */
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

    FMC_Open();                        /* Enable FMC ISP function */

    /*
     *  Check if User Configuration CBS is boot with IAP mode.
     *  If not, modify it.
     */
    if (set_IAP_boot_mode() < 0)
    {
        printf("Failed to set IAP boot mode!\n");
        goto lexit;                    /* Failed to set IAP boot mode. Program aborted. */
    }

    /* Read BS */
    printf("  Boot Mode ............................. ");
    if (FMC_GetBootSource() == 0)      /* Get boot source */
        printf("[APROM]\n");           /* Is booting from APROM */
    else
    {
        printf("[LDROM]\n");           /* Is booting from LDROM */
        printf("  WARNING: The sample code must execute in APROM!\n");
        goto lexit;                    /* This sample program must execute in APROM. Program aborted. */
    }

    u32Data = FMC_ReadCID();           /* get company ID */
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_ReadCID failed!\n");
        goto lexit;
    }
    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    u32Data = FMC_ReadPID();           /* get product ID */
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_ReadPID failed!\n");
        goto lexit;
    }
    printf("  Product ID ............................ [0x%08x]\n", u32Data);

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
        u8Item = getchar();            /* block waiting to receive any one character from UART0 */
        printf("%c\n", u8Item);        /* print out the selected item */

        switch (u8Item)
        {
        case '0':
            FMC_ENABLE_LD_UPDATE();    /* Enable LDROM update capability */
            /*
             *  The binary image of LDROM code is embedded in this sample.
             *  load_image_to_flash() will program this LDROM code to LDROM.
             */
            if (load_image_to_flash( (uint32_t)&loaderImage1Base, (uint32_t)&loaderImage1Limit,
                                     FMC_LDROM_BASE, (uint32_t)&loaderImage1Limit - (uint32_t)&loaderImage1Base) != 0)
            {
                printf("Load image to LDROM failed!\n");
                goto lexit;            /* Load LDROM code failed. Program aborted. */
            }
            FMC_DISABLE_LD_UPDATE();   /* Disable LDROM update capability */
            break;

        case '1':
            printf("\n\nChange VECMAP and branch to LDROM...\n");
            printf("LDROM code ResetHandler = 0x%x\n", *(uint32_t *)(FMC_LDROM_BASE+4));
            /* To check if all the debug messages are finished */
            while(IsDebugFifoEmpty() == 0);
            /*  NOTE!
             *     Before change VECMAP, user MUST disable all interrupts.
             *     The following code CANNOT locate in address 0x0 ~ 0x200.
             */

            FMC_SetVectorPageAddr(FMC_LDROM_BASE);

            if (g_FMC_i32ErrCode != 0)
            {
                printf("FMC_SetVectorPageAddr failed!\n");
                return -1;
            }
            /* Software reset to boot to LDROM */
            NVIC_SystemReset();
            break;

        default :
            continue;                  /* invalid selection */
        }
    }
    while (1);


lexit:

    FMC_Close();                       /* Disable FMC ISP function */

    SYS_LockReg();                     /* Lock protected registers */

    printf("\nFMC Sample Code Completed.\n");

    while (1);
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

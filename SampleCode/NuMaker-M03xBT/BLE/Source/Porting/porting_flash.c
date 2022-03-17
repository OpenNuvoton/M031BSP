/*----------------------------------------------------------------------------*/
/* This file implement MCU peripherals like: Flash control for BLE            */
/*----------------------------------------------------------------------------*/
#include <stdio.h>
#include "NuMicro.h"
#include "porting_flash.h"

uint32_t flash_fota_bank_addr = 0;              /* The start address of FOTA bank partition */
uint32_t flash_fota_info_addr = 0;              /* The start address of FOTA info partition */
uint32_t flash_data_partition_addr = 0;         /* The start address of data flash partition */


/******************************************************************************
 * Public Functions
 ******************************************************************************/
/** This function is used to set the base address of flash partitions.
 *  Must call this function when enable FOTA or Data Flash feature.
 *
 * @param[in] u8ShowMsg : Show the partition information or not.
 * @return none
 */
void setBLE_FlashPartitionsBA(uint8_t u8ShowMsg)
{
    uint32_t total_flash_size;

    if (GET_CHIP_SERIES_NUM == CHIP_SERIES_NUM_D)
    {
        total_flash_size = 0x00010000;      /* 64 KB */
    }
    else if (GET_CHIP_SERIES_NUM == CHIP_SERIES_NUM_E)
    {
        total_flash_size = 0x00020000;      /* 128 KB */
    }
    else if (GET_CHIP_SERIES_NUM == CHIP_SERIES_NUM_G)
    {
        total_flash_size = 0x00040000;      /* 256 KB */
    }
    else if (GET_CHIP_SERIES_NUM == CHIP_SERIES_NUM_I)
    {
        total_flash_size = 0x00080000;      /* 512 KB */
    }
    else
    {
#if !defined(_BOOT_LOADER_)
        printf("Do not support the chipset for BLE !\n");
#endif
        while (1);
    }

    /* The data flash partition must be the most top partition */
    flash_data_partition_addr = total_flash_size - SIZE_OF_DATA_FLASH;

    if (u8ShowMsg)
    {
#if !defined(_BOOT_LOADER_)
        printf("Set the base address of Flash partitions...\n");
        printf("Image Bank      : %08X ~ %08X\n", 0, BONDING_INFORMATION_ADDRESS - 1);
        printf("Bonding Info    : %08X ~ %08X\n", BONDING_INFORMATION_ADDRESS, BONDING_INFORMATION_ADDRESS + SIZE_OF_BONDING_INFORMATION - 1);
#endif
    }

    /* 64KB flash size does not support FOTA feature */
    if (GET_CHIP_SERIES_NUM != CHIP_SERIES_NUM_D)
    {
        /* The FOTA info partition is close to and under data flash partition */
        flash_fota_info_addr = total_flash_size - SIZE_OF_DATA_FLASH - SIZE_OF_FOTA_INFO;
        /* The FOTA bank partition is close to and under FOTA info partition */
        flash_fota_bank_addr = flash_fota_info_addr - SIZE_OF_FOTA_BANK;

        if (u8ShowMsg)
        {
#if !defined(_BOOT_LOADER_)
            printf("FOTA Bank       : %08X ~ %08X\n", flash_fota_bank_addr, flash_fota_bank_addr + SIZE_OF_FOTA_BANK - 1);
            printf("FOTA Info       : %08X ~ %08X\n", flash_fota_info_addr, flash_fota_info_addr + SIZE_OF_FOTA_INFO - 1);
#endif
        }

        /* Check the partition boundary */
        if (BONDING_INFORMATION_ADDRESS + SIZE_OF_BONDING_INFORMATION > flash_fota_bank_addr)
        {
#if !defined(_BOOT_LOADER_)
            printf("Bonding partition is conflict with FOTA bank partition !\n");
#endif
            while (1);
        }
    }

    if (u8ShowMsg)
    {
#if !defined(_BOOT_LOADER_)
        printf("Data Flash      : %08X ~ %08X\n", flash_data_partition_addr, flash_data_partition_addr + SIZE_OF_DATA_FLASH - 1);
#endif
    }

    /* Check the partition boundary */
    if (BONDING_INFORMATION_ADDRESS + SIZE_OF_BONDING_INFORMATION > flash_data_partition_addr)
    {
#if !defined(_BOOT_LOADER_)
        printf("Bonding partition is conflict with data flash partition !\n");
#endif
        while (1);
    }
}

/** This function is used to flash program for BLE.
 *
 * @param[in] u32Addr : Address of the flash location to be programmed.
 * @param[in] u32Data : The data to be programmed.
 * @return none
 */
void setBLE_FlashProgram(uint32_t u32Addr, uint32_t u32Data)
{
    __disable_irq();
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable FMC ISP function */
    FMC_Open();

    FMC_ENABLE_AP_UPDATE();
    FMC_Write(u32Addr, u32Data);
#if (_CHIP_SELECTION_ == _CHIP_M032BT)
    /* Invalidate cache after FMC_Write() to fix ISP issue */
    FMC->FTCTL |= BIT9;
    while ((FMC->FTCTL & BIT9) == BIT9);
#endif
    FMC_DISABLE_AP_UPDATE();

    /* Disable FMC ISP function */
    FMC_Close();
    /* Lock protected registers */
    SYS_LockReg();
    __enable_irq();
}

/** This function is used to flash erase for BLE.
 *
 * @param[in] u32Addr : Address of the flash page to be erased..
 * @return  Page erase success or not.
 * @retval  0:  Success
 * @retval -1:  Erase failed
 */
int32_t setBLE_FlashErase(uint32_t u32Addr)
{
    int32_t EraseStatus = 0;

    __disable_irq();

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable FMC ISP function */
    FMC_Open();

    FMC_ENABLE_AP_UPDATE();
    EraseStatus = FMC_Erase(u32Addr);
    FMC_DISABLE_AP_UPDATE();

    /* Disable FMC ISP function */
    FMC_Close();
    /* Lock protected registers */
    SYS_LockReg();

    __enable_irq();

    return EraseStatus;
}


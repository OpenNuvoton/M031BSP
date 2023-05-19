/**************************************************************************//**
 * @file     data_flash.c
 * @version  V0.1
 * $Revision: 01 $
 * @brief
 *           Support functions to control data flash
 * @note
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "porting_flash.h"
#include "BleAppSetting.h"

#if (BLE_AUTO_CONNECT == ENABLE_DEF)
#define HOST_ID_SIZE        (8)                     // Need be 8-bytes alignment

extern uint32_t flash_data_partition_addr;          // The start address of data flash partition


/* Add record to data flash */
void add_record_to_data_flash(uint8_t input_addr[])
{
    uint32_t u32Addr;
    uint32_t u32Data;
    uint16_t offset;
    uint8_t i;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

    /* The second page of the data flash */
    u32Addr = flash_data_partition_addr + FLASH_PAGE_SIZE;

    /* Find the first blank area */
    for (offset = 0; offset <= (FLASH_PAGE_SIZE - HOST_ID_SIZE); offset += HOST_ID_SIZE)
    {
        u32Data = FMC_Read(u32Addr + offset);
        if (u32Data == 0xFFFFFFFF)
            break;
    }

    if (offset <= (FLASH_PAGE_SIZE - HOST_ID_SIZE))
    {
        u32Addr += offset;

        /* Enable APROM erase/program */
        FMC_ENABLE_AP_UPDATE();

        /* Host ID */
        u32Data = 0;
        for (i = 0; i < 4; i++)
            u32Data += input_addr[i] << (i * 8);
        FMC_Write(u32Addr, u32Data);
#if (_CHIP_SELECTION_ == _CHIP_M032BT)
        /* Invalidate cache after FMC_Write() to fix ISP issue */
        FMC->FTCTL |= BIT9;
        while ((FMC->FTCTL & BIT9) == BIT9);
#endif

        u32Data = input_addr[4] + (input_addr[5] << 8);
        FMC_Write(u32Addr + 4, u32Data);
#if (_CHIP_SELECTION_ == _CHIP_M032BT)
        /* Invalidate cache after FMC_Write() to fix ISP issue */
        FMC->FTCTL |= BIT9;
        while ((FMC->FTCTL & BIT9) == BIT9);
#endif

        /* Disable APROM erase/program */
        FMC_DISABLE_AP_UPDATE();
    }
    else
    {
        printf("ERROR! The data flash page is full.\n");
    }

    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();
}


/* Check current records in data flash */
int check_record_in_data_flash(uint8_t input_addr[])
{
    uint8_t paired_addr[6];
    uint32_t u32Addr;
    uint32_t u32Data;
    uint16_t offset;
    uint8_t i, j;
    int ret;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

    /* The second page of the data flash */
    u32Addr = flash_data_partition_addr + FLASH_PAGE_SIZE;

    ret = -1;
    /* Compare the records in flash */
    for (offset = 0; offset <= (FLASH_PAGE_SIZE - HOST_ID_SIZE); offset += HOST_ID_SIZE)
    {
        u32Data = FMC_Read(u32Addr + offset);

        /* Check the flash is blank or not */
        if (u32Data == 0xFFFFFFFF)
        {
            break;
        }

        for (i = 0; i < 6; i += 4)
        {
            for (j = 0; j < 4; j++)
                paired_addr[i + j] = u32Data >> (j * 8);

            u32Data = FMC_Read(u32Addr + offset + 4);
        }

        if (memcmp(&input_addr[0], &paired_addr[0], 6) == 0)
        {
            ret = 0;
            break;
        }
    }

    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    return ret;
}


/* Write default settings to data flash */
void initial_data_flash(void)
{
    uint32_t u32Addr;

    /* Enable APROM erase/program */
    FMC_ENABLE_AP_UPDATE();

    /* Erase the page of device address records */
    u32Addr = flash_data_partition_addr + FLASH_PAGE_SIZE;
    FMC_Erase(u32Addr);

    /* Disable APROM erase/program */
    FMC_DISABLE_AP_UPDATE();
}


/* Set data flash partition */
int set_data_flash(void)
{
    uint32_t au32Config[2];

    if (!flash_data_partition_addr)
    {
        /* Set the base address of flash partitions */
        setBLE_FlashPartitionsBA(0);
    }

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

    /* Read User Configuration 0 & 1 */
    if (FMC_ReadConfig(au32Config, 2) < 0)
    {
        printf("Read FMC User Config failed!\n");
        return -1;
    }

    /* Check if Data Flash is enabled (CONFIG0[0]) and is expected address (CONFIG1) */
    if ((!(au32Config[0] & 0x1)) && (au32Config[1] == flash_data_partition_addr))
    {
        /* Disable FMC ISP function */
        FMC_Close();

        /* Lock protected registers */
        SYS_LockReg();

        return 0;
    }
    else
    {
        FMC_ENABLE_CFG_UPDATE();

        /* CONFIG0[0] = 0 (Enabled) / 1 (Disabled) */
        au32Config[0] &= ~0x1;
        au32Config[1] = flash_data_partition_addr;

        /* Update User Configuration settings. */
        if (FMC_WriteConfig(au32Config, 2) < 0)
        {
            printf("Write FMC User Config failed!\n");
            return -1;
        }
        printf("Set Data Flash base as 0x%x.\n", flash_data_partition_addr);
    }

    initial_data_flash();

    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(UART0);

    /* Perform chip reset to make new User Config take effect */
    SYS_ResetChip();

    return 1;
}
#endif


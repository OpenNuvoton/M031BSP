/**************************************************************************//**
 * @file     atcmd_misc.c
 * @version  V0.1
 * $Revision: 01 $
 * @brief
 *           Support AT command to control BLE
 * @note
 *
 ******************************************************************************/
#include <stdio.h>
#include "porting_flash.h"
#include "atcmd.h"

#define ATCMD_SETTING_SIZE     (24 + 4 + 4)         // Need be 4-bytes alignment

extern uint32_t flash_data_partition_addr;          // The start address of data flash partition


/* Write current settings to data flash */
void atcmd_update_data_flash(void)
{
    uint32_t u32Addr;
    uint32_t u32Data;
    uint16_t offset;
    uint8_t i, j;

    /* The second page of the data flash */
    u32Addr = flash_data_partition_addr + FLASH_PAGE_SIZE;

    /* Find the first blank area */
    for (offset = 0; offset <= (FLASH_PAGE_SIZE - ATCMD_SETTING_SIZE); offset += ATCMD_SETTING_SIZE)
    {
        u32Data = FMC_Read(u32Addr + offset);
        if (u32Data == 0xFFFFFFFF)
            break;
    }

    if (offset <= (FLASH_PAGE_SIZE - ATCMD_SETTING_SIZE))
    {
        u32Addr += offset;

        /* Enable APROM erase/program */
        FMC_ENABLE_AP_UPDATE();

        /* Scan name */
        for (i = 0; i < sizeof(ble_scan_name); i += 4)
        {
            u32Data = 0;
            for (j = 0; j < 4; j++)
                u32Data += ble_scan_name[i + j] << (j * 8);
            FMC_Write(u32Addr, u32Data);
#if (_CHIP_SELECTION_ == _CHIP_M032BT)
            /* Invalidate cache after FMC_Write() to fix ISP issue */
            FMC->FTCTL |= BIT9;
            while ((FMC->FTCTL & BIT9) == BIT9);
#endif
            u32Addr += 4;
        }

        /* UART */
        u32Data = ble_uart;
        FMC_Write(u32Addr, u32Data);
#if (_CHIP_SELECTION_ == _CHIP_M032BT)
        FMC->FTCTL |= BIT9;
        while ((FMC->FTCTL & BIT9) == BIT9);
#endif

        /* TX power */
        u32Addr += 4;
        u32Data = 0;
        for (i = 0; i < 3; i++)
            u32Data += ble_tx_power[i] << (i * 8);
        FMC_Write(u32Addr, u32Data);
#if (_CHIP_SELECTION_ == _CHIP_M032BT)
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
}


/* Load current settings from data flash */
int atcmd_load_data_flash(void)
{
    uint32_t u32Addr;
    uint32_t u32Data;
    uint16_t offset;
    uint8_t i, j;

    /* The second page of the data flash */
    u32Addr = flash_data_partition_addr + FLASH_PAGE_SIZE;

    /* Find the last non-blank area */
    for (offset = 0; offset <= (FLASH_PAGE_SIZE - ATCMD_SETTING_SIZE); offset += ATCMD_SETTING_SIZE)
    {
        u32Data = FMC_Read(u32Addr + offset);

        /* Check the flash is blank or not */
        if (u32Data == 0xFFFFFFFF)
            break;
    }

    /* Load settings fail if data flash is empty */
    if (!offset)
        return -1;

    u32Addr += offset - ATCMD_SETTING_SIZE;

    /* Scan name */
    for (i = 0; i < sizeof(ble_scan_name); i += 4)
    {
        u32Data = FMC_Read(u32Addr);
        for (j = 0; j < 4; j++)
            ble_scan_name[i + j] = u32Data >> (j * 8);
        u32Addr += 4;
    }

    /* UART */
    u32Data = FMC_Read(u32Addr);
    ble_uart = u32Data;

    /* TX power */
    u32Addr += 4;
    u32Data = FMC_Read(u32Addr);
    for (i = 0; i < 3; i++)
        ble_tx_power[i] = u32Data >> (i * 8);

    /* If there are more than one setting */
    if (offset > ATCMD_SETTING_SIZE)
    {
        /* Erase page and keep the last record */
        FMC_Erase(flash_data_partition_addr + FLASH_PAGE_SIZE);
        atcmd_update_data_flash();
    }

    return 0;
}


/* Write default settings to data flash */
void atcmd_initial_data_flash(void)
{
    uint32_t u32Addr;
    uint32_t u32Data;
    uint8_t i, j;

    /* Enable APROM erase/program */
    FMC_ENABLE_AP_UPDATE();

    /* Erase the page of ATCMD settings */
    u32Addr = flash_data_partition_addr + FLASH_PAGE_SIZE;
    FMC_Erase(u32Addr);

    /* Scan name */
    for (i = 0; i < sizeof(ble_scan_name_def); i += 4)
    {
        u32Data = 0;
        for (j = 0; j < 4; j++)
            u32Data += ble_scan_name_def[i + j] << (j * 8);
        FMC_Write(u32Addr, u32Data);
#if (_CHIP_SELECTION_ == _CHIP_M032BT)
        /* Invalidate cache after FMC_Write() to fix ISP issue */
        FMC->FTCTL |= BIT9;
        while ((FMC->FTCTL & BIT9) == BIT9);
#endif
        u32Addr += 4;
    }

    /* UART */
    u32Data = ble_uart_def;
    FMC_Write(u32Addr, u32Data);
#if (_CHIP_SELECTION_ == _CHIP_M032BT)
    FMC->FTCTL |= BIT9;
    while ((FMC->FTCTL & BIT9) == BIT9);
#endif

    /* TX power */
    u32Addr += 4;
    u32Data = 0;
    for (i = 0; i < 3; i++)
        u32Data += ble_tx_power_def[i] << (i * 8);
    FMC_Write(u32Addr, u32Data);
#if (_CHIP_SELECTION_ == _CHIP_M032BT)
    FMC->FTCTL |= BIT9;
    while ((FMC->FTCTL & BIT9) == BIT9);
#endif

    /* Disable APROM erase/program */
    FMC_DISABLE_AP_UPDATE();
}


/* Set data flash partition */
int atcmd_set_data_flash(void)
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
        /* Try to load settings from data flash */
        if (atcmd_load_data_flash() >= 0)
        {
            /* Disable FMC ISP function */
            FMC_Close();

            return 0;
        }
        else
        {
            printf("Load data flash failed! Re-initial the data flash.\n");
        }
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

    atcmd_initial_data_flash();

    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(UART0);

    /* Perform chip reset to make new User Config take effect */
    SYS_ResetChip();

    return 1;
}


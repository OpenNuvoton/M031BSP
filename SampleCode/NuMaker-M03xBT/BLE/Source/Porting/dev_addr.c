/**************************************************************************//**
 * @file     addr.c
 * @version  V0.1
 * $Revision: 01 $
 * @brief
 *           Setting the BLE device address.
 * @note
 *
 ******************************************************************************/
#include <stdio.h>
#include "ble_cmd.h"
#include "porting_flash.h"
#include "mcu_definition.h"

extern uint32_t flash_data_partition_addr;          // The start address of data flash partition

/* Set BLE device address from data flash */
int BleAddr_Form_Flash(BLE_Addr_Param *bleAddrParam)
{
    uint32_t au32Config[2];
    uint32_t u32Addr;
    uint32_t u32Data1;
    uint32_t u32Data2;
    int32_t i32ValidAddr = -1;
    uint8_t i;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

    /* Read User Configuration 0 & 1 */
    if (FMC_ReadConfig(au32Config, 2) < 0)
    {
        printf("Read FMC User Config failed!\n");
    }
    /* Check if Data Flash is enabled (CONFIG0[0]) and is expected address (CONFIG1) */
    else if ((!(au32Config[0] & 0x1)) && (au32Config[1] == flash_data_partition_addr))
    {
        /* The first page of data flash */
        u32Addr = flash_data_partition_addr;

        /* BLE device address */
        u32Data1 = FMC_Read(u32Addr);
        u32Data2 = FMC_Read(u32Addr + 4);

        /* Check the device address is vaild or not (46-bits are not all zero or one) */
        if ((u32Data1 == 0x00000000) && ((u32Data2 & 0xFF3F) == 0x0000))
            ;
        else if ((u32Data1 == 0xFFFFFFFF) && ((u32Data2 & 0xFF3F) == 0xFF3F))
            ;
        else
        {
            /* Load device address from flash */
            for (i = 0; i < 4; i++)
            {
                bleAddrParam->addr[i] = (uint8_t)(u32Data1 >> (i * 8));
            }

            for (i = 0; i < 2; i++)
            {
                bleAddrParam->addr[i + 4] = (uint8_t)(u32Data2 >> (i * 8));
            }
            i32ValidAddr = 0;
        }
    }

    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    return i32ValidAddr;
}


/* Set BLE device address from UID */
int BleAddr_Form_UID(BLE_Addr_Param *bleAddrParam)
{
    uint32_t u32UID[3];
    uint8_t i;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

    /* BLE device address */
    for (i = 0; i < 3; i ++)
    {
        u32UID[i] = FMC_ReadUID(i);
    }

    /* Decide 46-bits address from 96-bits UID */
    bleAddrParam->addr[0] = (uint8_t)((u32UID[0] & 0x0FF0) >> 4);
    bleAddrParam->addr[1] = (uint8_t)(u32UID[1] & 0x00FF);
    bleAddrParam->addr[2] = (uint8_t)((u32UID[1] & 0xFF00) >> 8);
    bleAddrParam->addr[3] = (uint8_t)(u32UID[2] & 0x00FF);
    bleAddrParam->addr[4] = (uint8_t)((u32UID[2] & 0xFF00) >> 8);
    bleAddrParam->addr[5] = (uint8_t)(((u32UID[2] & 0x30000) >> 12) + ((u32UID[0] & 0xF000) >> 12));

    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    return 0;
}


/* Set BLE device address */
BleStackStatus Set_BleAddr(void)
{
    BLE_Addr_Param bleAddrParam;
    BleStackStatus status;
    uint8_t i;

    if (BleAddr_Form_Flash(&bleAddrParam) >= 0)
    {
        printf("BLE device address from flash: ");
    }
    else
    {
        BleAddr_Form_UID(&bleAddrParam);
        printf("BLE device address from UID: ");
    }

    /* Set BLE device address as random address */
    bleAddrParam.addrType = RANDOM_ADDR;
    bleAddrParam.addr[5] |= 0xC0;
    status = setBLE_BleDeviceAddr(&bleAddrParam);
    if (status == BLESTACK_STATUS_SUCCESS)
    {
        getBLE_BleDeviceAddr(&bleAddrParam);
        printf("%02x", bleAddrParam.addr[5]);
        for (i = 0; i < 5; i++)
        {
            printf(":%02x", bleAddrParam.addr[4 - i]);
        }
        printf("\r\n");
    }
    else
    {
        printf("setBLE_BleDeviceAddr() returns fail %d\n", status);
    }

    return status;
}


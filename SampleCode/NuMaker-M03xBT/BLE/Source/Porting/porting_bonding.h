#ifndef __PORTING_BONDING_H__
#define __PORTING_BONDING_H__

#include "ble_msgblock.h"
/**************************************************************************
 * Definitions
 **************************************************************************/
/**
 * @defgroup portingBondingDef BLE Bonding Porting Related
 * @{
 * @ingroup porting
 * @details The detail settings for BLE Bonding.
 **************************************************************************/

/**< Total size of BLE bonding information array.*/

#define SIZE_OF_KEY_BLK                                128                                              /**< Size of bond key information.*/
#define SIZE_OF_DATA_BLK                               128                                              /**< Size of data information.*/
#define SIZE_OF_INFO_BLK                               (SIZE_OF_KEY_BLK+SIZE_OF_DATA_BLK)               /**< Size of BLE bonding information block.*/
#define SIZE_OF_INFO_BLK_FOR_POWER_OF_2                8                                                /**< SIZE_OF_INFO_BLK 256 = 2^8. */

#define NUM_OF_TOTAL_INFO_BLK                          (SIZE_OF_BONDING_INFORMATION/SIZE_OF_INFO_BLK)   /**< Total number of inforamtion size of bound information block.*/
#define NUM_OF_FLASH_PAGE_FOR_BONDING_INFO_BLK         (SIZE_OF_BONDING_INFORMATION/FLASH_PAGE_SIZE)    /**< Total number of flash size of bound information block.*/
#define NUM_OF_INFO_BLK_ONE_PAGE                       (FLASH_PAGE_SIZE/SIZE_OF_INFO_BLK)               /**< Amount of information bound using flash page.*/

#define TAB_FLASH_BONDING_INFO_BLK                     SIZE_OF_INFO_BLK                                 /**< Tab Size of BLE bonding information block.*/

#define ERASE_THRESHOLD                                (NUM_OF_TOTAL_INFO_BLK-1-NUM_HOST_LINK)          /**< Can be set 1 ~ (NUM_OF_TOTAL_INFO_BLK-1-NUM_HOST_LINK) */

#endif  //(ifndef __PORTING_BONDING_H__)

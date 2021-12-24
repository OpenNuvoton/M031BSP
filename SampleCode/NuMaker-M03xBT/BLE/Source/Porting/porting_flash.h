#ifndef __PORTING_FLASH_H__
#define __PORTING_FLASH_H__

#include <stdint.h>
#include "mcu_definition.h"

/**************************************************************************
* Flash Map Definition
**************************************************************************/
/**
 * @defgroup mcuFlashGeneralDef MCU Flash Porting Definition
 * @{
 * @ingroup mcuPortingDef
 * @details MCU flash porting related definition.
 * @}
 * @defgroup mcuFlashDef MCU Flash Detail Porting Definition
 * @{
 * @ingroup mcuFlashGeneralDef
 * @details MCU flash porting related definition.
 * @}
 **************************************************************************/

/** Flash default value definition.
 * @ingroup mcuFlashDef
*/
#define FLASH_DEFAULT_VALUE                           0xFF

/**
 * @ingroup mcuFlashDef
 * @defgroup flashPageSizeDef Flash Page Size
 * @{
 * @brief Defined supported flash Page size.
 */
#if (_CHIP_SELECTION_ == _CHIP_M031BT)
#define FLASH_PAGE_SIZE                               512   /**< Size of Flash Page Type.*/
#elif (_CHIP_SELECTION_ == _CHIP_M032BT)
#define FLASH_PAGE_SIZE                               2048  /**< Size of Flash Page Type.*/
#endif

/**
 * @ingroup mcuFlashDef
 * @defgroup flashProgramSizeDef Flash Programming Size
 * @{
 * @brief Defined supported flash programming size.
 */
#define FLASH_PROGRAM_SIZE_1BYTE                      1     /**< Programming 1 byte at once  */
#define FLASH_PROGRAM_SIZE_2BYTE                      2     /**< Programming 2 bytes at once */
#define FLASH_PROGRAM_SIZE_4BYTE                      4     /**< Programming 4 bytes at once */
/** @} */

/** Selected a flash programming size at once from @ref flashProgramSizeDef "Flash programming size definition list".
 * @attention Please update the definition of @ref FLASH_PROGRAM_SIZE_FOR_POWER_OF_2 if change the definition of @ref FLASH_PROGRAM_SIZE.
*/
#define FLASH_PROGRAM_SIZE                            FLASH_PROGRAM_SIZE_4BYTE


/**
 * @defgroup flashProgramSizeExponentDef The Exponent of Flash Programming Size Definition
 * @{
 * @ingroup flashProgramSizeDef
 * @brief Defined supported the exponent of flash programming size.
 */
#define FLASH_PROGRAM_SIZE_FOR_EXPONENT_0             0     /**< @ref FLASH_PROGRAM_SIZE = 2^exponent = 2^0 = 1 */
#define FLASH_PROGRAM_SIZE_FOR_EXPONENT_1             1     /**< @ref FLASH_PROGRAM_SIZE = 2^exponent = 2^1 = 2 */
#define FLASH_PROGRAM_SIZE_FOR_EXPONENT_2             2     /**< @ref FLASH_PROGRAM_SIZE = 2^exponent = 2^2 = 4 */
/** @} */

/** Selected the exponent of flash programming size from @ref flashProgramSizeExponentDef "The exponent of flash programming size definition list".
 * @attention The value of 2^ @ref FLASH_PROGRAM_SIZE_FOR_POWER_OF_2 shall be equal to @ref FLASH_PROGRAM_SIZE.
*/
#define FLASH_PROGRAM_SIZE_FOR_POWER_OF_2             FLASH_PROGRAM_SIZE_FOR_EXPONENT_2


/** Selected the exponent of flash page size from @ref flashPageSizeDef "The exponent of flash page size definition list".
 * @attention The value of 2^ @ref FLASH_PAGE_SIZE_FOR_POWER_OF_2 shall be equal to @ref FLASH_PAGE_SIZE.
*/
#if (_CHIP_SELECTION_ == _CHIP_M031BT)
#define FLASH_PAGE_SIZE_FOR_POWER_OF_2                9     /**< FLASH_PAGE 512 = 2^9 */
#elif (_CHIP_SELECTION_ == _CHIP_M032BT)
#define FLASH_PAGE_SIZE_FOR_POWER_OF_2                11    /**< FLASH_PAGE 2048 = 2^11 */
#endif
/** @} */


/**************************************************************************/
/**
 * @defgroup mcuFlashMapDef MCU Flash Map Definition
 * @{
 * @ingroup mcuFlashGeneralDef
 * @details MCU flash map porting definition.
 * @}
 **************************************************************************/

/**
 * @defgroup mcuFlashMapBondingDef MCU Flash Map (BLE Bonding) Porting Definition
 * @{
 * @ingroup mcuFlashMapDef
 * @details MCU flash map for BLE bonding porting related definition.
 * @attention Do not modify the setting of @ref SIZE_OF_BONDING_INFORMATION and @ref SIZE_OF_BONDING_DATA.
 */
#if (_CHIP_SELECTION_ == _CHIP_M031BT)
#define BONDING_INFORMATION_ADDRESS                   0x0000F000            /**< Address of bonding information block.*/

#elif (_CHIP_SELECTION_ == _CHIP_M032BT)
#define BONDING_INFORMATION_ADDRESS                   0x0001E000            /**< Address of bonding information block.*/
#endif

#define SIZE_OF_BONDING_INFORMATION                   FLASH_PAGE_SIZE * 4   /**< Total size of BLE bonding information array.*/
/** @} */

/**
 * @defgroup mcuFlashMapFotaDef MCU Flash Map (BLE FOTA) Porting Definition
 * @{
 * @ingroup mcuFlashMapDef
 * @details MCU flash map for BLE FOTA porting related definition.
 * @attention Do not modify the setting of @ref SIZE_OF_FOTA_BANK and @ref SIZE_OF_FOTA_INFO.
 */
#define SIZE_OF_FOTA_BANK                             BONDING_INFORMATION_ADDRESS   /**< Total Size for saving FOTA update data.*/
#define SIZE_OF_FOTA_INFO                             FLASH_PAGE_SIZE               /**< Total Size for saving FOTA update information, please reference structure "ota_information_t" (fota.c) to get the page content.*/
/** @} */

/**
 * @defgroup mcuFlashMapBondingDef MCU Flash Map (Data Flash) Porting Definition
 * @{
 * @ingroup mcuFlashMapDef
 * @details MCU flash map for data flash related definition.
 * There are three pages in data flash. The 1st page is used for BLE device address,
 * 2nd and 3rd pages are reserved for user data (ex. AT command settings use 2nd page).
 */
#define SIZE_OF_DATA_FLASH                            FLASH_PAGE_SIZE * 3           /**< Total Size for saving FOTA update information, please reference structure "ota_information_t" (fota.c) to get the page content.*/


/**************************************************************************
* Functions
**************************************************************************/
/** @defgroup mcuFlashFunc MCU Flash Function Definition
 * @{
 * @details Here shows the BLE MCU flash program and erase functions.
 * @ingroup mcuFlashDef
 * @}
 **************************************************************************/

/** This function is used to set the base address of flash partitions.
 *  Must call this function when enable FOTA or Data Flash feature.
 *
 * @ingroup mcuFlashFunc
 *
 * @param[in] u8ShowMsg : Show the partition information or not.
 * @return none
 */
void setBLE_FlashPartitionsBA(uint8_t u8ShowMsg);



/** This function is used to flash program for BLE.
 *
 * @ingroup mcuFlashFunc
 *
 * @param[in] u32Addr : Address of the flash location to be programmed.
 * @param[in] u32Data : The data to be programmed.
 * @return none
 */
void setBLE_FlashProgram(uint32_t u32Addr, uint32_t u32Data);



/** This function is used to flash erase for BLE.
 *
 * @ingroup mcuFlashFunc
 *
 * @param[in] u32Addr : Address of the flash page to be erased..
 * @return  Page erase success or not.
 * @retval  0:  Success
 * @retval -1:  Erase failed
 */
int32_t setBLE_FlashErase(uint32_t u32Addr);



#endif  //__PORTING_FLASH_H__

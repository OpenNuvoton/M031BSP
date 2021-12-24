/**************************************************************************//**
* @file       ble_host.h
* @brief      Provide the declarations that for BLE Host and Profile subsystem needed.
*
* @defgroup host BLE Host and Profile Subsystem Definition
* @{
* @details Provide the declarations that for BLE Host subsystem needed. (ble_host.h)
* @}
*****************************************************************************/

#ifndef _BLE_HOST_H_
#define _BLE_HOST_H_

#include <stdint.h>
#include "ble_gap.h"
#include "ble_uuid.h"
#include "ble_att_gatt.h"
#include "ble_stack_status.h"


/**************************************************************************
 * Definitions
 **************************************************************************/

/** @ingroup host
*/
#define SIZE_ARRAY_ROW(a)             (sizeof((a))/sizeof((a[0])))        /**< The size of the array.*/

/** @ingroup host
*/
#define SIZE_STRING(a)                (sizeof((a))/sizeof((a[0])) - 1)    /**< The size of the string.*/

/** @ingroup host
*/
#define BLE_HOSTID_RESERVED           0xFF                                /**< Reserved BLE host id.*/



/** REF_SIZE_LE_HOST_PARA for checking host parameter size in application layer and host layer.
 * @ingroup host
 * @attention Do NOT modify this definition.
*/
#define REF_SIZE_LE_HOST_PARA          168



/**************************************************************************
 * Structures
 **************************************************************************/
/** @brief BLE Services GATT Event Parameter Definition
*/
typedef struct BLE_Event_AttParam
{
    uint8_t         hostId;                       /**< Host id.*/
    uint8_t         gattRole;                     /**< GATT role.*/
    uint16_t        hdlNum;                       /**< Characteristic value handle.*/
    uint8_t         cmdAccess;                    /**< Defined command identification.*/
    uint8_t         *data;                        /**< BLE attribute event data.*/
    uint16_t        length;                       /**< The length of BLE attribute event data.*/
} BLE_Event_AttParam;


/**
 * @defgroup host_att BLE Profile Subsystem Structure Definition
 * @{
 * @ingroup host
 * @details The definition of BLE profile subsystem structure.
 */

/** @brief BLE Attribute Handle Parameter Definition
*/
typedef struct ATTRIBUTE_BLE_Hdl_Para
{
    uint16_t  numHDL;                              /**< Attribute handle number.*/
    uint8_t   propertyValue;                       /**< characteristic property value.*/
    union
    {
        uint8_t  cfgClientCharc;                  /**< Client characteristic configuration descriptor value.*/
        uint8_t  cfgServerCharc;                  /**< Server characteristic configuration descriptor value*/
    } value;                                      /**< Client/Server characteristic configuration descriptor value.*/
} ATTRIBUTE_BLE_Hdl_Para;


/** @brief BLE Attribute Database Mapping By Host ID
*/
typedef struct ATTR_DB_Mapping_by_ID
{
    ATTRIBUTE_BLE_Hdl_Para *mapClientDB;          /**< Mapping to client attribute database.*/
    ATTRIBUTE_BLE_Hdl_Para *mapServerDB;          /**< Mapping to server attribute database.*/
} ATTR_DB_Mapping_by_ID;


/** @brief BLE Attribute Database Mapping By Host ID Size
*/
typedef struct ATTR_DB_Mapping_by_ID_size
{
    uint16_t  sizeMapClientDB;                   /**< Mapping to size of client attribute database.*/
    uint16_t  sizeMapServerDB;                   /**< Mapping to size of server attribute database.*/
} ATTR_DB_Mapping_by_ID_size;


/** @brief BLE Service Declaration With An Attribute.
*/
typedef struct ATTRIBUTE_BLE
{
    void    *uuidType;                            /**< Attribute type which defined by a UUID, an UUID is used to identify every attribute type. */
    void    *uuidValue;                           /**< Attribute Value shall be the 16-bit Bluetooth UUID or 128-bit UUID for the service/ characteristic, known as the service/ characteristic UUID.*/
    uint16_t  lenValue;                           /**< The length of attribute value.*/
    uint8_t   propertyValue;                      /**< Characteristic properties.*/
    uint8_t   permFormatDB;                       /**< UUID format and characteristic permission.*/
    void    (*attCallFunc)(BLE_Event_AttParam *AttParam); /**< Register callback function. */
} ATTRIBUTE_BLE;


/** @brief BLE Pre-defined Services Definition
*/
typedef struct ATTR_DB_Role_by_ID
{
    const ATTRIBUTE_BLE *const *clientDB;         /**< The table of GATT database describes the pre-defined attributes. */
    const ATTRIBUTE_BLE *const *serverDB;         /**< The table of GATT database describes the server and attributes contained on the server. */
} ATTR_DB_Role_by_ID;

/** @} */

#endif // _BLE_HOST_H_


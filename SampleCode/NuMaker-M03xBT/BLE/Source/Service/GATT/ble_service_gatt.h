#ifndef _BLE_SERVICE_GATT_H_
#define _BLE_SERVICE_GATT_H_

/**************************************************************************//**
 * @file  ble_service_gatt.h
 * @brief Provide the Definition of GATT Service.
*****************************************************************************/

#include "ble_service_base.h"
#include "ble_host.h"
#include "ble_cmd.h"


/**************************************************************************
 * GATT Definitions
 **************************************************************************/
/** @defgroup serviceGATT_def BLE GATT Service Definitions
* @{
* @ingroup service_def
* @details Here shows the definitions of the GATT service.
* @}
**************************************************************************/

/**
 * @ingroup serviceGATT_def
 * @defgroup serviceGATT_UUIDDef BLE GATT UUID Definitions
 * @{
 * @details Here shows the definitions of the BLE GATT UUID Definitions.
*/
extern const uint16_t ATTR_UUID_GATT_PRIMARY_SERVICE[];        /**< GATT service UUID. */
extern const uint16_t ATTR_UUID_GATT_CHARC_SERVICE_CHANGED[];  /**< GATT characteristic service changed UUID. */
/** @} */

/**
 * @defgroup serviceGATT_dataDef BLE GATT Value Definitions
 * @{
 * @ingroup serviceGATT_def
 * @details Here shows the value definitions of the GATT service.
 * @}
*/

/** GATT Characteristic Service Changed Value Definition.
 * @ingroup serviceGATT_dataDef
*/
extern const uint16_t ATTR_VALUE_GATT_SERVICE_CHANGED[];


/**
 * @defgroup serviceGATT_ServiceChardef BLE GATT Service and Characteristic Definitions
 * @{
 * @ingroup serviceGATT_def
 * @details Here shows the definitions of the GATT service and characteristic.
 * @}
*/

/**
 * @ingroup serviceGATT_ServiceChardef
 * @{
*/
extern const ATTRIBUTE_BLE ATT_GATT_PRIMARY_SERVICE;                              /**< GATT primary Service. */
extern const ATTRIBUTE_BLE ATT_GATT_CHARACTERISTIC_SERVICE_CHANGED;               /**< GATT characteristic service changed. */
extern const ATTRIBUTE_BLE ATT_GATT_SERVICE_CHANGED;                              /**< GATT service changed value. */
extern const ATTRIBUTE_BLE ATT_GATT_CLIENT_CHARC_CONFIGURATION_SERVICE_CHANGED;   /**< GATT client characteristic configuration descriptor. */
/** @} */


/** GATT Service Definition
 * @ingroup serviceGATT_ServiceChardef
*/
#define ATT_GATT_SERVICE                                        \
    &ATT_GATT_PRIMARY_SERVICE,                                  \
    &ATT_GATT_CHARACTERISTIC_SERVICE_CHANGED,                   \
    &ATT_GATT_SERVICE_CHANGED,                                  \
    &ATT_GATT_CLIENT_CHARC_CONFIGURATION_SERVICE_CHANGED,       \

/**************************************************************************
 * GATT Application Definitions
 **************************************************************************/
/** @defgroup appGATT_def BLE GATT Application Definitions
 * @{
 * @ingroup serviceGATT_def
 * @details Here shows the definitions of the GATT service for application.
 * @}
**************************************************************************/

/**
 * @ingroup appGATT_def
 * @defgroup appGATT_eventDef BLE GATT Service and Characteristic Definitions
 * @{
 * @details Here shows the event definitions of the GATT service.
*/
#define BLESERVICE_GATT_SERVICE_CHANGED_INDICATE_EVENT               0x01   /**< GATT characteristic service cahnged indicate event. */
#define BLESERVICE_GATT_SERVICE_CHANGED_CCCD_WRITE_EVENT             0x02   /**< GATT characteristic service cahnged cccd write event. */
#define BLESERVICE_GATT_SERVICE_CHANGED_CCCD_READ_EVENT              0x03   /**< GATT characteristic service cahnged cccd read event. */
#define BLESERVICE_GATT_SERVICE_CHANGED_CCCD_WRITE_RSP_EVENT         0x04   /**< GATT characteristic service cahnged cccd write response event. */
/** @} */


/**
 * @ingroup appGATT_def
 * @defgroup appGATT_structureDef BLE GATT Structure Definitions
 * @{
 * @details Here shows the structure definitions of the GATT service.
 * @}
*/

/** GATT Handles Definition
 * @ingroup appGATT_structureDef
*/
typedef struct BLEATT_GATT_Handles
{
    uint16_t hdl_service_changed;           /**< Handle of GATT service changed value. */
    uint16_t hdl_service_changed_cccd;      /**< Handle of service changed client characteristic configuration descriptor. */
} BLEATT_GATT_Handles;


/** GATT Data Definition
 * @ingroup appGATT_structureDef
 * @note User defined.
*/
typedef struct BLEATT_GATT_Data
{
    uint16_t  service_changed_cccd;         /**< GATT service changed cccd value */
} BLEATT_GATT_Data;



/** GATT Application Data Structure Definition
 * @ingroup appGATT_structureDef
*/
typedef struct BLEATT_GATT_Info
{
    BleGattRole                 role;       /**< BLE GATT role. */
    BLEATT_GATT_Handles         handles;    /**< GATT attribute handles */
    BLEATT_GATT_Data            data;       /**< GATT attribute data */
} BLEATT_GATT_Info;


/**
 * @ingroup appGATT_def
 * @defgroup appGATT_App BLE GATT Definitions for Application
 * @{
 * @details Here shows the definitions of the GATT for application uses.
 * @}
*/

/** BleGATT_EventCallBack
 *
 * @ingroup appGATT_App
 *
 * @note This callback receives the GATT service events. Each of these events can be associated with parameters.
 *
 */
typedef void (*BleGATT_EventCallBack)(uint8_t hostId, uint8_t cmdAccess, uint8_t *data, uint16_t length);


/** GATT Service Initialization
 *
 * @ingroup appGATT_App
 *
 * @param[in] hostId : the link's host id.
 * @param[in] gattRole : @ref BleGattRole "BLE GATT role".
 * @param[in] info : a pointer to GATT information.
 * @param[in] callback : a pointer to a callback function that receive the service events.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED : Registered services buffer full.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus setGATT_ServiceInit(uint8_t hostId, BleGattRole gattRole, BLEATT_GATT_Info *info, BleGATT_EventCallBack callback);


/** Get GATT Service Handle Numbers
 *
 * @ingroup appGATT_App
 *
 * @attention
 *            - role = <b> @ref BLE_GATT_ROLE_CLIENT </b> \n
 *              MUST call this API to get service information after received @ref BLECMD_EVENT_ATT_DATABASE_PARSING_FINISHED  \n
 *            - role = <b> @ref BLE_GATT_ROLE_SERVER </b> \n
 *              MUST call this API to get service information before connection established. \n
 *
 * @param[in] hostId : the link's host id.
 * @param[out] info : a pointer to GATT information
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus getGATT_ServiceHandles(uint8_t hostId, BLEATT_GATT_Info *info);


#endif //_BLE_SERVICE_GATT_H_

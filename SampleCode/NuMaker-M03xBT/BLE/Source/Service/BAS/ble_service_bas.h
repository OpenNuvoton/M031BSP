#ifndef _BLE_SERVICE_BAS_H_
#define _BLE_SERVICE_BAS_H_

/**************************************************************************//**
 * @file  ble_service_bas.h
 * @brief Provide the Definition of BAS (Battery Service).
*****************************************************************************/

#include "ble_service_base.h"
#include "ble_host.h"
#include "ble_cmd.h"


/**************************************************************************
 * BAS Definitions
 **************************************************************************/
/** @defgroup serviceBAS_def BLE BAS Definitions
 * @{
 * @ingroup service_def
 * @details Here shows the definitions of the BAS service.
 * @}
**************************************************************************/

/**
 * @ingroup serviceBAS_def
 * @defgroup serviceBAS_UUIDDef BLE BAS UUID Definitions
 * @{
 * @details Here shows the definitions of the BLE BAS UUID Definitions.
*/
extern const uint16_t ATTR_UUID_BAS_PRIMARY_SERVICE[];        /**< BAS service UUID. */
extern const uint16_t ATTR_UUID_BAS_CHARC_BATTERY_LEVEL[];    /**< BAS characteristic BATTERY_LEVEL UUID. */
/** @} */

/**
 * @defgroup serviceBAS_ServiceChardef BLE BAS Service and Characteristic Definitions
 * @{
 * @ingroup serviceBAS_def
 * @details Here shows the definitions of the BAS service and characteristic.
 * @}
*/

/**
 * @ingroup serviceBAS_ServiceChardef
 * @{
*/
extern const ATTRIBUTE_BLE ATT_BAS_PRIMARY_SERVICE;                             /**< BAS primary service. */
extern const ATTRIBUTE_BLE ATT_BAS_CHARACTERISTIC_BATTERY_LEVEL;                /**< BAS characteristic BATTERY_LEVEL. */
extern const ATTRIBUTE_BLE ATT_BAS_BATTERY_LEVEL;                               /**< BAS BATTERY_LEVEL value. */
extern const ATTRIBUTE_BLE ATT_BAS_BATTERY_LEVEL_CLIENT_CHARC_CONFIGURATION;    /**< BAS BATTERY_LEVEL client characteristic configuration descriptor. */
/** @} */


/** BAS Service Definition
 * @ingroup serviceBAS_ServiceChardef
*/
#define ATT_BAS_SERVICE                                 \
    &ATT_BAS_PRIMARY_SERVICE,                           \
    &ATT_BAS_CHARACTERISTIC_BATTERY_LEVEL,              \
    &ATT_BAS_BATTERY_LEVEL,                             \
    &ATT_BAS_BATTERY_LEVEL_CLIENT_CHARC_CONFIGURATION,  \

/**************************************************************************
 * BAS Application Definitions
 **************************************************************************/
/** @defgroup appBAS_def BLE BAS Application Definitions
 * @{
 * @ingroup serviceBAS_def
 * @details Here shows the definitions of the BAS service for application.
 * @}
**************************************************************************/

/**
 * @ingroup appBAS_def
 * @defgroup appBAS_eventDef BLE BAS Service and Characteristic Definitions
 * @{
 * @details Here shows the event definitions of the BAS service.
*/
#define BLESERVICE_BAS_LEVEL_NOTIFY_EVENT          0x01   /**< Battery level notification event. */
#define BLESERVICE_BAS_LEVEL_READ_EVENT            0x02   /**< Battery level read event. */
#define BLESERVICE_BAS_LEVEL_CCCD_WRITE_EVENT      0x03   /**< Battery level cccd write event. */
#define BLESERVICE_BAS_LEVEL_CCCD_WRITE_RSP_EVENT  0x04   /**< Battery level cccd write response event. */
#define BLESERVICE_BAS_LEVEL_CCCD_READ_EVENT       0x05   /**< Battery level cccd read event. */
/** @} */


/**
 * @ingroup appBAS_def
 * @defgroup appBAS_structureDef BLE BAS Structure Definitions
 * @{
 * @details Here shows the structure definitions of the BAS service.
 * @}
*/

/** BAS Handles Definition
 * @ingroup appBAS_structureDef
*/
typedef struct BLEATT_BAS_Handles
{
    uint16_t hdl_battery_level;             /**< Handle of battery level value. */
    uint16_t hdl_battery_level_cccd;        /**< Handle of battery level client characteristic configuration descriptor. */
} BLEATT_BAS_Handles;


/** BAS Data Definition
 * @ingroup appBAS_structureDef
 * @note User defined.
*/
typedef struct BLEATT_BAS_Data
{
    uint8_t   battery_level;                /**< Battery level value */
    uint16_t  battery_level_cccd;           /**< Battery level cccd value */
} BLEATT_BAS_Data;


/** BAS Application Data Structure Definition
 * @ingroup appBAS_structureDef
*/
typedef struct BLEATT_BAS_Info
{
    BleGattRole             role;           /**< BLE GATT role */
    BLEATT_BAS_Handles      handles;        /**< BAS attribute handles */
    BLEATT_BAS_Data         data;           /**< BAS attribute data */
} BLEATT_BAS_Info;



/**
 * @ingroup appBAS_def
 * @defgroup appBAS_App BLE BAS Definitions for Application
 * @{
 * @details Here shows the definitions of the BAS service for application uses.
 * @}
*/

/** BleBAS_EventCallBack
 * @ingroup appBAS_App
 * @note This callback receives the BAS events. Each of these events can be associated with parameters.
*/
typedef void (*BleBAS_EventCallBack)(uint8_t hostId, uint8_t cmdAccess, uint8_t *data, uint16_t length);



/** BAS Initialization
 *
 * @ingroup appBAS_App
 *
 * @param[in] hostId : the link's host id.
 * @param[in] gattRole : @ref BleGattRole "BLE GATT role".
 * @param[in] info : a pointer to BAS information.
 * @param[in] callback : a pointer to a callback function that receive the service events.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED : Registered services buffer full.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus setBAS_ServiceInit(uint8_t hostId, BleGattRole gattRole, BLEATT_BAS_Info *info, BleBAS_EventCallBack callback);



/** Get BAS Handle Numbers
 *
 * @ingroup appBAS_App
 *
 * @attention
 *            - role = <b> @ref BLE_GATT_ROLE_CLIENT </b> \n
 *              MUST call this API to get service information after received @ref BLECMD_EVENT_ATT_DATABASE_PARSING_FINISHED  \n
 *            - role = <b> @ref BLE_GATT_ROLE_SERVER </b> \n
 *              MUST call this API to get service information before connection established. \n
 *
 * @param[in] hostId : the link's host id.
 * @param[out] info : a pointer to BAS information
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus getBAS_ServiceHandles(uint8_t hostId, BLEATT_BAS_Info *info);


#endif //_BLE_SERVICE_BAS_H_

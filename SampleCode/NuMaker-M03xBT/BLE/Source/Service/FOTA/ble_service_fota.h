#ifndef _BLE_SERVICE_FOTA_H_
#define _BLE_SERVICE_FOTA_H_

/**************************************************************************//**
 * @file  ble_service_fota.h
 * @brief Provide the Definition of FOTA Service (BLE Firmware on the air Service).
*****************************************************************************/

#include "ble_service_base.h"
#include "ble_host.h"
#include "ble_cmd.h"


/**************************************************************************
 * FOTA Definitions
 **************************************************************************/
/** @defgroup serviceFOTA_def BLE FOTA Definitions
 * @{
 * @ingroup service_def
 * @details Here shows the definitions of the FOTA service.
 * @}
**************************************************************************/

/**
 * @ingroup serviceFOTA_def
 * @defgroup serviceFOTA_UUIDDef BLE FOTA UUID Definitions
 * @{
 * @details Here shows the definitions of the BLE FOTA UUID Definitions.
*/
extern const uint16_t ATTR_UUID_FOTA_PRIMARY_SERVICE[];    /**< FOTA service UUID. */
extern const uint16_t ATTR_UUID_FOTA_CHARC_DATA[];         /**< FOTA characteristic DATA UUID. */
extern const uint16_t ATTR_UUID_FOTA_CHARC_COMMAND[];      /**< FOTA characteristic COMMAND UUID. */
/** @} */

/**
 * @defgroup serviceFOTA_ServiceChardef BLE FOTA Service and Characteristic Definitions
 * @{
 * @ingroup serviceFOTA_def
 * @details Here shows the definitions of the FOTA service and characteristic.
 * @}
*/

/**
 * @ingroup serviceFOTA_ServiceChardef
 * @{
*/
extern const ATTRIBUTE_BLE ATT_FOTA_PRIMARY_SERVICE;                       /**< FOTA primary service. */
extern const ATTRIBUTE_BLE ATT_FOTA_CHARACTERISTIC_DATA;                   /**< FOTA characteristic DATA. */
extern const ATTRIBUTE_BLE ATT_FOTA_DATA;                                  /**< FOTA DATA value. */
extern const ATTRIBUTE_BLE ATT_FOTA_DATA_CLIENT_CHARC_CONFIGURATION;       /**< FOTA DATA client characteristic configuration descriptor. */
extern const ATTRIBUTE_BLE ATT_FOTA_DATA_USER_DESCRIPTION;                 /**< FOTA DATA user description. */
extern const ATTRIBUTE_BLE ATT_FOTA_CHARACTERISTIC_COMMAND;                /**< FOTA characteristic COMMAND. */
extern const ATTRIBUTE_BLE ATT_FOTA_COMMAND;                               /**< FOTA COMMAND value. */
extern const ATTRIBUTE_BLE ATT_FOTA_COMMAND_CLIENT_CHARC_CONFIGURATION;    /**< FOTA COMMAND client characteristic configuration descriptor. */
extern const ATTRIBUTE_BLE ATT_FOTA_COMMAND_USER_DESCRIPTION;              /**< FOTA COMMAND user description. */
/** @} */


/** FOTA Service Definition
 * @ingroup serviceFOTA_ServiceChardef
*/
#define ATT_FOTA_SERVICE                                      \
    &ATT_FOTA_PRIMARY_SERVICE,                                \
    &ATT_FOTA_CHARACTERISTIC_DATA,                            \
    &ATT_FOTA_DATA,                                           \
    &ATT_FOTA_DATA_CLIENT_CHARC_CONFIGURATION,                \
    &ATT_FOTA_DATA_USER_DESCRIPTION,                          \
    &ATT_FOTA_CHARACTERISTIC_COMMAND,                         \
    &ATT_FOTA_COMMAND,                                        \
    &ATT_FOTA_COMMAND_CLIENT_CHARC_CONFIGURATION,             \
    &ATT_FOTA_COMMAND_USER_DESCRIPTION,

/**************************************************************************
 * FOTA Application Definitions
 **************************************************************************/
/** @defgroup appFOTA_def BLE FOTA Application Definitions
 * @{
 * @ingroup serviceFOTA_def
 * @details Here shows the definitions of the FOTA service for application.
 * @}
**************************************************************************/

/**
 * @ingroup appFOTA_def
 * @defgroup appFOTA_eventDef BLE FOTA Service and Characteristic Definitions
 * @{
 * @details Here shows the event definitions of the FOTA service.
*/
#define BLESERVICE_FOTA_DATA_WRITE_EVENT                      0x01   /**< Firmware on the air data write event. */
#define BLESERVICE_FOTA_DATA_CCCD_WRITE_EVENT                 0x02   /**< Firmware on the air data cccd write event. */
#define BLESERVICE_FOTA_DATA_CCCD_READ_EVENT                  0x03   /**< Firmware on the air data cccd read event. */
#define BLESERVICE_FOTA_COMMAND_WRITE_EVENT                   0x04   /**< Firmware on the air command notification event. */
#define BLESERVICE_FOTA_COMMAND_CCCD_WRITE_EVENT              0x05   /**< Firmware on the air command cccd write event. */
#define BLESERVICE_FOTA_COMMAND_CCCD_READ_EVENT               0x06   /**< Firmware on the air command cccd read event. */
/** @} */


/**
 * @ingroup appFOTA_def
 * @defgroup appFOTA_structureDef BLE FOTA Structure Definitions
 * @{
 * @details Here shows the structure definitions of the FOTA service.
 * @}
*/

/** FOTA Handles Definition
 * @ingroup appFOTA_structureDef
*/
typedef struct BLEATT_FOTA_Handles
{
    uint16_t hdl_fota_data;                       /**< Handle of Firmware on the air data. */
    uint16_t hdl_fota_data_cccd;                  /**< Handle of Firmware on the air data client characteristic configuration descriptor. */
    uint16_t hdl_fota_data_user_description;      /**< Handle of Firmware on the air data user description. */
    uint16_t hdl_fota_command;                    /**< Handle of Firmware on the air command. */
    uint16_t hdl_fota_command_cccd;               /**< Handle of Firmware on the air command client characteristic configuration descriptor. */
    uint16_t hdl_fota_command_user_description;   /**< Handle of Firmware on the air command user description. */
} BLEATT_FOTA_Handles;


/** FOTA Data Definition
 * @ingroup appFOTA_structureDef
 * @note User defined.
*/
typedef struct BLEATT_FOTA_Data
{
    uint16_t  fota_data_cccd;                /**< Firmware on the air data cccd value. */
    uint16_t  fota_command_cccd;             /**< Firmware on the air command cccd value */
} BLEATT_FOTA_Data;


/** FOTA Application Data Structure Definition
 * @ingroup appFOTA_structureDef
*/
typedef struct BLEATT_FOTA_Info
{
    BleGattRole             role;            /**< BLE GATT role. */
    BLEATT_FOTA_Handles     handles;         /**< FOTA attribute handles */
    BLEATT_FOTA_Data        data;            /**< FOTA attribute data */
} BLEATT_FOTA_Info;


/**
 * @ingroup appFOTA_def
 * @defgroup appFOTA_App BLE FOTA Definitions for Application
 * @{
 * @details Here shows the definitions of the FOTA service for application uses.
 * @}
*/

/** BleFOTA_EventCallBack
 * @ingroup appFOTA_App
 * @note This callback receives the FOTA events. Each of these events can be associated with parameters.
*/
typedef void (*BleFOTA_EventCallBack)(uint8_t hostId, uint8_t cmdAccess, uint8_t *data, uint16_t length);



/** FOTA Service Initialization
 *
 * @ingroup appFOTA_App
 *
 * @param[in] hostId : the link's host id.
 * @param[in] gattRole : @ref BleGattRole "BLE GATT role".
 * @param[in] info : a pointer to FOTA information.
 * @param[in] callback : a pointer to a callback function that receive the service events.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED : Registered services buffer full.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus setFOTA_ServiceInit(uint8_t hostId, BleGattRole gattRole, BLEATT_FOTA_Info *info, BleFOTA_EventCallBack callback);



/** Get FOTA Service Handle Numbers
 *
 * @ingroup appFOTA_App
 *
 * @attention
 *            - role = <b> @ref BLE_GATT_ROLE_CLIENT </b> \n
 *              MUST call this API to get service information after received @ref BLECMD_EVENT_ATT_DATABASE_PARSING_FINISHED  \n
 *            - role = <b> @ref BLE_GATT_ROLE_SERVER </b> \n
 *              MUST call this API to get service information before connection established. \n
 *
 * @param[in] hostId : the link's host id.
 * @param[out] info : a pointer to FOTA information
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus getFOTA_ServiceHandles(uint8_t hostId, BLEATT_FOTA_Info *info);


#endif //_BLE_SERVICE_FOTA_H_

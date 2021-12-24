#ifndef _BLE_SERVICE_HRS_H_
#define _BLE_SERVICE_HRS_H_

/**************************************************************************//**
 * @file  ble_service_hrs.h
 * @brief Provide the Definition of HRS (Heart Rate Service).
*****************************************************************************/

#include "ble_service_base.h"
#include "ble_host.h"
#include "ble_cmd.h"


/**************************************************************************
 * HRS Definitions
 **************************************************************************/
/** @defgroup serviceHRS_def BLE HRS Definitions
 * @{
 * @ingroup service_def
 * @details Here shows the definitions of the HRS service.
 * @}
**************************************************************************/

/**
 * @ingroup serviceHRS_def
 * @defgroup serviceHRS_UUIDDef BLE HRS UUID Definitions
 * @{
 * @details Here shows the definitions of the BLE HRS UUID Definitions.
*/
extern const uint16_t ATTR_UUID_HRS_PRIMARY_SERVICE[];                  /**< Heart Rate Service UUID. */
extern const uint16_t ATTR_UUID_HRS_CHARC_HEART_RATE_MEASUREMENT[];     /**< Heart Rate measurement characteristic UUID. */
extern const uint16_t ATTR_UUID_HRS_CHARC_BODY_SENSOR_LOCATION[];       /**< Heart Rate body sensor location characteristic UUID. */
/** @} */

/**
 * @defgroup serviceHRS_ServiceChardef BLE HRS Service and Characteristic Definitions
 * @{
 * @ingroup serviceHRS_def
 * @details Here shows the definitions of the HRS service and characteristic.
 * @}
*/

/**
 * @ingroup serviceHRS_ServiceChardef
 * @{
*/
extern const ATTRIBUTE_BLE ATT_HRS_PRIMARY_SERVICE;                                 /**< HRS primary service. */
extern const ATTRIBUTE_BLE ATT_HRS_CHARACTERISTIC_HEART_RATE_MEASUREMENT;           /**< HRS characteristic heart Rate measurement. */
extern const ATTRIBUTE_BLE ATT_HRS_HEART_RATE_MEASUREMENT;                          /**< HRS heart Rate measurement value. */
extern const ATTRIBUTE_BLE ATT_HRS_MEASUREMENT_NOTIFY_CLIENT_CHARC_CONFIGURATION;   /**< HRS heart Rate measurement client characteristic configuration descriptor. */
extern const ATTRIBUTE_BLE ATT_HRS_CHARACTERISTIC_BODY_SENSOR_LOCATION;             /**< HRS characteristic body sensor location. */
extern const ATTRIBUTE_BLE ATT_HRS_BODY_SENSOR_LOCATION;                            /**< HRS body sensor location value. */
/** @} */


/** HRS Service Definition
 * @ingroup serviceHRS_ServiceChardef
*/
#define ATT_HRS_SERVICE                                      \
    &ATT_HRS_PRIMARY_SERVICE,                                \
    &ATT_HRS_CHARACTERISTIC_HEART_RATE_MEASUREMENT,          \
    &ATT_HRS_HEART_RATE_MEASUREMENT,                         \
    &ATT_HRS_MEASUREMENT_NOTIFY_CLIENT_CHARC_CONFIGURATION,  \
    &ATT_HRS_CHARACTERISTIC_BODY_SENSOR_LOCATION,            \
    &ATT_HRS_BODY_SENSOR_LOCATION,                           \

/**************************************************************************
 * HRS Application Definitions
 **************************************************************************/
/** @defgroup appHRS_def BLE HRS Application Definitions
 * @{
 * @ingroup serviceHRS_def
 * @details Here shows the definitions of the HRS service for application.
 * @}
**************************************************************************/

/**
 * @ingroup appHRS_def
 * @defgroup appHRS_eventDef BLE HRS Service and Characteristic Definitions
 * @{
 * @details Here shows the event definitions of the HRS service.
*/
#define BLESERVICE_HRS_MEASUREMENT_NOTIFY_EVENT                  0x01   /**< Heart rate measurement notification event. */
#define BLESERVICE_HRS_MEASUREMENT_CCCD_WRITE_RSP_EVENT          0x02   /**< Heart rate measurement cccd write response event. */
#define BLESERVICE_HRS_MEASUREMENT_CCCD_READ_RSP_EVENT           0x03   /**< Heart rate measurement cccd read event. */
#define BLESERVICE_HRS_BODY_SENSOR_LOCATION_READ_EVENT           0x04   /**< Heart rate body sensor location read event. */
/** @} */


/**
 * @ingroup appHRS_def
 * @defgroup appHRS_structureDef BLE HRS Structure Definitions
 * @{
 * @details Here shows the structure definitions of the HRS service.
 * @}
*/

/** HRS Handles Definition
 * @ingroup appHRS_structureDef
*/
typedef struct BLEATT_HRS_Handles
{
    uint16_t hdl_heart_rate_measurement;            /**< Handle of Heart rate measurement. */
    uint16_t hdl_heart_rate_measurement_cccd;       /**< Handle of Heart rate measurement client characteristic configuration descriptor. */
    uint16_t hdl_body_sensor_location;              /**< Handle of Body sensor location. */
} BLEATT_HRS_Handles;


/** HRS Data Definition
 * @ingroup appHRS_structureDef
 * @note User defined.
*/
typedef struct BLEATT_HRS_Data
{
    uint8_t   heart_rate_measurement[4];            /**< Heart rate measurement value: [0]: HRS Flag; [1]: Heart Rate Data [3][2]: Heart Rate RR-Interval */
    uint16_t  heart_rate_measurement_cccd;          /**< Heart rate measurement cccd value */
    uint8_t   body_sensor_location;                 /**< Body sensor location value */
} BLEATT_HRS_Data;


/** HRS Application Data Structure Definition
 * @ingroup appHRS_structureDef
*/
typedef struct BLEATT_HRS_Info
{
    BleGattRole             role;                   /**< BLE GATT role */
    BLEATT_HRS_Handles      handles;                /**< HRS attribute handles */
    BLEATT_HRS_Data         data;                   /**< HRS attribute data */
} BLEATT_HRS_Info;


/**
 * @ingroup appHRS_def
 * @defgroup appHRS_App BLE HRS Definitions for Application
 * @{
 * @details Here shows the definitions of the HRS service for application uses.
 * @}
*/

/** BleHRS_EventCallBack
 * @ingroup appHRS_App
 * @note This callback receives the HRS events. Each of these events can be associated with parameters.
*/
typedef void (*BleHRS_EventCallBack)(uint8_t hostId, uint8_t cmdAccess, uint8_t *data, uint16_t length);


/** HRS Initialization
 *
 * @ingroup appHRS_App
 *
 * @param[in] hostId : the link's host id.
 * @param[in] gattRole : @ref BleGattRole "BLE GATT role".
 * @param[in] info : a pointer to HRS information.
 * @param[in] callback : a pointer to a callback function that receive the service events.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED : Registered services buffer full.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus setHRS_ServiceInit(uint8_t hostId, BleGattRole gattRole, BLEATT_HRS_Info *info, BleHRS_EventCallBack callback);


/** Get HRS Handle Numbers
 *
 * @ingroup appHRS_App
 *
 * @attention
 *            - role = <b> @ref BLE_GATT_ROLE_CLIENT </b> \n
 *              MUST call this API to get service information after received @ref BLECMD_EVENT_ATT_DATABASE_PARSING_FINISHED  \n
 *            - role = <b> @ref BLE_GATT_ROLE_SERVER </b> \n
 *              MUST call this API to get service information before connection established. \n
 *
 * @param[in] hostId : the link's host id.
 * @param[out] info : a pointer to HRS information
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus getHRS_ServiceHandles(uint8_t hostId, BLEATT_HRS_Info *info);



#endif //_BLE_SERVICE_HRS_H_

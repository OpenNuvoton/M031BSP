#ifndef _BLE_SERVICE_GAP_H_
#define _BLE_SERVICE_GAP_H_

/**************************************************************************//**
 * @file  ble_service_gap.h
 * @brief Provide the Definition of GAP Service.
*****************************************************************************/

#include <stdint.h>
#include "ble_service_base.h"
#include "ble_host.h"
#include "ble_uuid.h"
#include "ble_cmd.h"


/**************************************************************************
 * GAP Definitions
 **************************************************************************/
/** @defgroup serviceGAP_def BLE GAP Service Definitions
* @{
* @ingroup service_def
* @details Here shows the definitions of the GAP service.
* @}
**************************************************************************/

/**
 * @ingroup serviceGAP_def
 * @defgroup serviceGAP_UUIDDef BLE GAP UUID Definitions
 * @{
 * @details Here shows the definitions of the BLE GAP UUID Definitions.
*/
extern const uint16_t ATTR_UUID_GAP_PRIMARY_SERVICE[];                                    /**< GAP service UUID. */
extern const uint16_t ATTR_UUID_GAP_CHARC_DEVICE_NAME[];                                  /**< GAP characteristic device name UUID. */
extern const uint16_t ATTR_UUID_GAP_CHARC_APPEARANCE[];                                   /**< GAP characteristic appearance UUID. */
extern const uint16_t ATTR_UUID_GAP_CHARC_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS[];   /**< GAP characteristic peripheral preferred connection parameters UUID. */
/** @} */

/**
 * @defgroup serviceGAP_ServiceChardef BLE GAP Service and Characteristic Definitions
 * @{
 * @ingroup serviceGAP_def
 * @details Here shows the definitions of the GAP service and characteristic.
 * @}
*/

/**
 * @ingroup serviceGAP_ServiceChardef
 * @{
*/
extern const ATTRIBUTE_BLE ATT_GAP_PRIMARY_SERVICE;                                           /**< GAP primary Service. */
extern const ATTRIBUTE_BLE ATT_GAP_CHARACTERISTIC_DEVICE_NAME;                                /**< GAP characteristic device name.*/
extern const ATTRIBUTE_BLE ATT_GAP_DEVICE_NAME;                                               /**< GAP device name value.*/
extern const ATTRIBUTE_BLE ATT_GAP_CHARACTERISTIC_APPEARANCE;                                 /**< GAP characteristic appearance.*/
extern const ATTRIBUTE_BLE ATT_GAP_APPEARANCE;                                                /**< GAP appearance value.*/
extern const ATTRIBUTE_BLE ATT_GAP_CHARACTERISTIC_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS; /**< GAP characteristic peripheral preferred connection parameters.*/
extern const ATTRIBUTE_BLE ATT_GAP_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS;                /**< GAP peripheral preferred connection parameters value.*/
/** @} */


/** GAP Service Definition
 * @ingroup serviceGAP_ServiceChardef
*/
#define ATT_GAP_SERVICE                                                   \
    &ATT_GAP_PRIMARY_SERVICE,                                             \
    &ATT_GAP_CHARACTERISTIC_DEVICE_NAME,                                  \
    &ATT_GAP_DEVICE_NAME,                                                 \
    &ATT_GAP_CHARACTERISTIC_APPEARANCE,                                   \
    &ATT_GAP_APPEARANCE,                                                  \
    &ATT_GAP_CHARACTERISTIC_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS,   \
    &ATT_GAP_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS,                  \


/**************************************************************************
 * GAP Application Definitions
 **************************************************************************/
/** @defgroup appGAP_def BLE GAP Application Definitions
* @{
* @ingroup serviceGAP_def
* @details Here shows the definitions of the GAP service for application.
* @}
**************************************************************************/
/**
 * @ingroup appGAP_def
 * @{
 */
#define GAP_DEVICE_NAME_LENGH_MAX                           248  /**< Maximum length of GAP server device name. */
#define GAP_DEVICE_NAME_LENGH                               30   /**< Default length of GAP server device name. */
/** @} */

/**
 * @ingroup appGAP_def
 * @defgroup appGAP_eventDef BLE GAP Service and Characteristic Definitions
 * @{
 * @details Here shows the event definitions of the GAP service.
*/
#define BLESERVICE_GAP_DEVICE_NAME_READ_EVENT               0x01   /**< GAP characteristic GAP device name read event. */
#define BLESERVICE_GAP_APPEARANCE_READ_EVENT                0x02   /**< GAP characteristic GAP appearance read event. */
#define BLESERVICE_GAP_PERIPHERAL_CONN_PARAM_READ_EVENT     0x03   /**< GAP characteristic GAP peripheral preferred connection parameters read event. */


/**
 * @ingroup appGAP_def
 * @defgroup appGAP_structureDef BLE GAP Structure Definitions
 * @{
 * @details Here shows the structure definitions of the GAP service.
 * @}
*/

/** GAP Handles Definition
 * @ingroup appGAP_structureDef
*/
typedef struct BLEATT_GAP_Handles
{
    uint16_t          hdl_device_name;                         /**< Handle of GAP device name. */
    uint16_t          hdl_appearance;                          /**< Handle of GAP appearance. */
    uint16_t          hdl_peripheral_preferred_connParam;      /**< Handle of GAP peripheral preferred connection parameters. */
} BLEATT_GAP_Handles;


/** GAP Data Definition
 * @ingroup appGAP_structureDef
 * @note User defined.
*/
typedef struct BLEATT_GAP_Data
{
    uint8_t            device_name[GAP_DEVICE_NAME_LENGH];       /**< GAP device name. */
    uint8_t            device_name_len;                          /**< GAP the length of device name. */
    uint16_t           appearance;                               /**< GAP appearance. */
    BLE_Conn_Param     peripheral_conn_param;                    /**< GAP peripheral preferred connection parameter. */
} BLEATT_GAP_Data;


/** GAP Application Data Structure Definition
 * @ingroup appGAP_structureDef
*/
typedef struct BLEATT_GAP_Info
{
    BleGattRole              role;       /**< BLE GAP role. */
    BLEATT_GAP_Handles       handles;    /**< GAP attribute handles. */
} BLEATT_GAP_Info;


/**
 * @ingroup appGAP_def
 * @defgroup appGAP_App BLE GAP Definitions for Application
 * @{
 * @details Here shows the definitions of the GAP service for application uses.
 * @}
*/

/** BleGAP_EventCallBack
 * @ingroup appGAP_App
 * @note This callback receives the GAP service events. Each of these events can be associated with parameters.  *
 */
typedef void (*BleGAP_EventCallBack)(uint8_t hostId, uint8_t cmdAccess, uint8_t *data, uint16_t length);



/** Set GAP Device Name
 *
 * @ingroup appGAP_def
 *
 * @param[in] name : a pointer to the device name.
 * @param[in] length : the length of the device name.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus setGAP_DeviceName(uint8_t *name, uint8_t length);


/** Set GAP Appearance
 *
 * @ingroup appGAP_def
 *
 *
 * @param[in] appearance : @ref bleGapAppearance.
*/
void setGAP_Appearance(uint16_t appearance);



/** Set GAP Peripheral Preferred Connection Parameters
 *
 * @ingroup appGAP_def
 *
 * @param[in] connParam : a pointer to the preferred peripheral connection parameters.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus setGAP_PeripheralConnectionParam(BLE_Conn_Param *connParam);



/** GAP Service Initialization
 *
 * @ingroup appGAP_def
 *
 * @attention There is only one instance of GAP shall be exposed on a device (if role is @ref BLE_GATT_ROLE_SERVER). \n
 *            Callback shall be ignored if role is @ref BLE_GATT_ROLE_SERVER).
 *
 * @param[in] hostId : the link's host id.
 * @param[in] gattRole : @ref BleGattRole "BLE GATT role".
 * @param[in] info : a pointer to GAP information.
 * @param[in] callback : a pointer to a callback function that receive the service events.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED : Registered services buffer full.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus setGAP_ServiceInit(uint8_t hostId, BleGattRole gattRole, BLEATT_GAP_Info *info, BleGAP_EventCallBack callback);


/** Get GAP Service Handle Numbers
 *
 * @ingroup appGAP_def
 *
 * @attention - role = @ref BLE_GATT_ROLE_CLIENT: \n
 *              MUST call this API to get service information after received @ref BLECMD_EVENT_ATT_DATABASE_PARSING_FINISHED  \n
 *            - role = @ref BLE_GATT_ROLE_SERVER: \n
 *              MUST call this API to get service information before connection established. \n
 *
 * @param[in] hostId : the link's host id.
 * @param[out] info : a pointer to GAP information
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus getGAP_ServiceHandles(uint8_t hostId, BLEATT_GAP_Info *info);



/** Get data from server by reading request (Client ONLY)
 *
 * @ingroup appGAP_def
 *
 * @param[in] hostId : the link's host id
 * @param[in] hdlNum : handle number of the data you want to read.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_ERR_DB_PARSING_IN_PROGRESS : parsing database process has NOT finished.
 * @retval BLESTACK_STATUS_ERR_INVALID_HANDLE : Invalid attribute handle.
 * @retval BLESTACK_STATUS_ERR_BUSY : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus getGAP_ClientDataRead(uint8_t hostId, uint16_t hdlNum);


#endif //_BLE_SERVICE_GAP_H_


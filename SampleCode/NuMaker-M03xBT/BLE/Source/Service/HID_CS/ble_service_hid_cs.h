#ifndef _BLE_SERVICE_HID_CS_H_
#define _BLE_SERVICE_HID_CS_H_

/**************************************************************************//**
 * @file  ble_service_hid_cs.h
 * @brief Provide the Definition of HID_CS Service.
*****************************************************************************/

#include "ble_service_base.h"
#include "ble_host.h"
#include "ble_cmd.h"


/**************************************************************************
 * HID Definitions
 **************************************************************************/
/** @defgroup serviceHidCs_def BLE HID Service Definitions
 * @{
 * @ingroup service_def
* @details Here shows the definitions of the BLE HID (Human Interface Device) service.
 * @}
**************************************************************************/

/**
 * @ingroup serviceHidCs_def
 * @defgroup serviceHidCs_UUIDDef BLE HID Service UUID Definitions
 * @{
 * @details Here shows the definitions of the BLE HID service UUID definitions.
*/
extern const uint16_t ATTR_UUID_HID_PRIMARY_SERVICE[];                      /**< HID service UUID. */
extern const uint16_t ATTR_UUID_HID_CHARC_INFORMATION[];                    /**< HID information characteristic UUID. */
extern const uint16_t ATTR_UUID_HID_CHARC_CONTROL_POINT[];                  /**< HID control point characteristic UUID. */
extern const uint16_t ATTR_UUID_HID_CHARC_REPORT_MAP[];                     /**< HID report map characteristic UUID. */
extern const uint16_t ATTR_UUID_HID_CHARC_PROTOCOL_MODE[];                  /**< HID protocol mode characteristic UUID. */
extern const uint16_t ATTR_UUID_HID_CHARC_REPORT[];                         /**< HID report characteristic UUID. */
/** @} */

/**
 * @defgroup serviceHidCs_ServiceChardef BLE HID Service and Characteristic Definitions
 * @{
 * @ingroup serviceHidCs_def
 * @details Here shows the definitions of the BLE HID service and characteristic.
 * @}
*/

/**
 * @ingroup serviceHidCs_ServiceChardef
 * @{
*/
extern const ATTRIBUTE_BLE ATT_HID_PRIMARY_SERVICE;                                              /**< HID primary service. */
extern const ATTRIBUTE_BLE ATT_HID_CHARACTERISTIC_INFORMATION;                                   /**< HID characteristic information. */
extern const ATTRIBUTE_BLE ATT_HID_INFORMATION;                                                  /**< HID characteristic information value. */
extern const ATTRIBUTE_BLE ATT_HID_CHARACTERISTIC_CONTROL_POINT;                                 /**< HID characteristic control point. */
extern const ATTRIBUTE_BLE ATT_HID_CONTROL_POINT;                                                /**< HID characteristic control point value. */
extern const ATTRIBUTE_BLE ATT_HID_CHARACTERISTIC_PROTOCOL_MODE;                                 /**< HID characteristic protocol mode. */
extern const ATTRIBUTE_BLE ATT_HID_PROTOCOL_MODE;                                                /**< HID characteristic protocol mode value. */
extern const ATTRIBUTE_BLE ATT_HID_CHARACTERISTIC_REPORT_MAP;                                    /**< HID characteristic report map. */
extern const ATTRIBUTE_BLE ATT_HID_REPORT_MAP;                                                   /**< HID characteristic report map value. */

extern const ATTRIBUTE_BLE ATT_HID_CHARACTERISTIC_CONSUMER_INPUT_REPORT;                         /**< HID characteristic consumer intput report. */
extern const ATTRIBUTE_BLE ATT_HID_CONSUMER_INPUT_REPORT;                                        /**< HID characteristic consumer intput report value. */
extern const ATTRIBUTE_BLE ATT_HID_CONSUMER_INPUT_NOTIFY_CLIENT_CHARC_CONFIGURATION;             /**< HID characteristic consumer intput report client characteristic configuration descriptor. */
extern const ATTRIBUTE_BLE ATT_HID_CONSUMER_INPUT_REPORT_REFERENCE;                              /**< HID characteristic consumer intput report reference value. */
/** @} */


/** HID Service Definition
 * @ingroup serviceHidCs_ServiceChardef
*/
#define ATT_HID_SERVICE                                             \
    &ATT_HID_PRIMARY_SERVICE,                                       \
    &ATT_HID_CHARACTERISTIC_INFORMATION,                            \
    &ATT_HID_INFORMATION,                                           \
    &ATT_HID_CHARACTERISTIC_CONTROL_POINT,                          \
    &ATT_HID_CONTROL_POINT,                                         \
    &ATT_HID_CHARACTERISTIC_PROTOCOL_MODE,                          \
    &ATT_HID_PROTOCOL_MODE,                                         \
    &ATT_HID_CHARACTERISTIC_REPORT_MAP,                             \
    &ATT_HID_REPORT_MAP,                                            \
    &ATT_HID_CHARACTERISTIC_CONSUMER_INPUT_REPORT,                  \
    &ATT_HID_CONSUMER_INPUT_REPORT,                                 \
    &ATT_HID_CONSUMER_INPUT_NOTIFY_CLIENT_CHARC_CONFIGURATION,      \
    &ATT_HID_CONSUMER_INPUT_REPORT_REFERENCE,                       \

/**************************************************************************
 * HID Application Definitions
 **************************************************************************/
/** @defgroup appHidCs_def BLE HID Service Application Definitions
 * @{
 * @ingroup serviceHidCs_def
 * @details Here shows the definitions of the BLE HID service for application.
 * @}
**************************************************************************/

/**
 * @ingroup appHidCs_def
 * @defgroup appHidCs_eventDef BLE HID Service and Characteristic Definitions
 * @{
 * @details Here shows the event definitions of the BLE HID service.
*/
#define BLESERVICE_HID_CONTROL_POINT_WRITE_EVENT                        0x01   /**< HID control point write event. */
#define BLESERVICE_HID_PROTOCOL_MODE_READ_EVENT                         0x02   /**< HID protocol mode read event. */
#define BLESERVICE_HID_PROTOCOL_MODE_WRITE_EVENT                        0x03   /**< HID protocol mode write event. */
#define BLESERVICE_HID_CONSUMER_INPUT_REPORT_READ_EVENT                 0x04   /**< HID consumer input report read event. */
#define BLESERVICE_HID_CONSUMER_INPUT_REPORT_WRITE_EVENT                0x05   /**< HID consumer input report read event. */
#define BLESERVICE_HID_CONSUMER_INPUT_REPORT_NOTIFY_EVENT               0x06   /**< HID consumer input report read event. */
#define BLESERVICE_HID_CONSUMER_INPUT_REPORT_CCCD_WRITE_EVENT           0x07   /**< HID consumer input report cccd write event. */
#define BLESERVICE_HID_CONSUMER_INPUT_REPORT_CCCD_READ_EVENT            0x08   /**< HID consumer input report cccd write event. */
/** @} */


/**
 * @ingroup appHidCs_def
 * @defgroup appHidCs_structureDef BLE HID Structure Definitions
 * @{
 * @details Here shows the structure definitions of the HID service.
 * @}
*/

/** HID Handles Definition
 * @ingroup appHidCs_structureDef
*/
typedef struct BLEATT_HID_Handles
{
    uint16_t hdl_hid_information;
    uint16_t hdl_hid_control_point;
    uint16_t hdl_hid_protocol_mode;
    uint16_t hdl_hid_report_map;
    uint16_t hdl_hid_consumer_input_report;
    uint16_t hdl_hid_consumer_input_report_cccd;
    uint16_t hdl_hid_consumer_input_report_reference;
} BLEATT_HID_Handles;


/** HID Service Data Definition
 * @ingroup appHidCs_structureDef
*/
typedef struct BLEATT_HID_Data
{
    uint8_t  hid_control_point;
    uint8_t  hid_protocol_mode;
    uint8_t  hid_consumer_input_report[2];
    uint16_t hid_consumer_input_report_cccd;
} BLEATT_HID_Data;


/** HID Application Data Structure Definition
 * @ingroup appHidCs_structureDef
*/
typedef struct BLEATT_HID_Info
{
    BleGattRole             role;                   /**< BLE GATT role */
    BLEATT_HID_Handles      handles;                /**< HID attribute handles */
    BLEATT_HID_Data         data;                   /**< HID attribute data */
} BLEATT_HID_Info;


/**
 * @ingroup appHidCs_def
 * @defgroup appHID_App BLE HID Definitions for Application
 * @{
 * @details Here shows the definitions of the HID service for application uses.
 * @}
*/

/** BleHID_EventCallBack
 * @ingroup appHID_App
 * @note This callback receives the HID service events. Each of these events can be associated with parameters.  *
 */
typedef void (*BleHID_EventCallBack)(uint8_t hostId, uint8_t cmdAccess, uint8_t *data, uint16_t length);



/** HID Service Initialization
 *
 * @ingroup appHID_App
 *
 * @param[in] hostId : the link's host id.
 * @param[in] gattRole : @ref BleGattRole "BLE GATT role".
 * @param[in] info : a pointer to HID information.
 * @param[in] callback : a pointer to a callback function that receive the service events.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED : Registered services buffer full.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus setHID_ServiceInit(uint8_t hostId, BleGattRole gattRole, BLEATT_HID_Info *info, BleHID_EventCallBack callback);



/** Get HID Service Handle Numbers
 *
 * @ingroup appHID_App
 *
 * @attention
 *            - role = <b> @ref BLE_GATT_ROLE_CLIENT </b> \n
 *              MUST call this API to get service information after received @ref BLECMD_EVENT_ATT_DATABASE_PARSING_FINISHED  \n
 *            - role = <b> @ref BLE_GATT_ROLE_SERVER </b> \n
 *              MUST call this API to get service information before connection established. \n
 *
 * @param[in] hostId : the link's host id.
 * @param[out] info : a pointer to HID information
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus getHID_ServiceHandles(uint8_t hostId, BLEATT_HID_Info *info);


#endif //_BLE_SERVICE_HID_CS_H_

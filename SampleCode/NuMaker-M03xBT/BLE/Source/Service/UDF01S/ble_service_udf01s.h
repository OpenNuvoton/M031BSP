#ifndef _BLE_SERVICE_UDF01S_H_
#define _BLE_SERVICE_UDF01S_H_

/**************************************************************************//**
 * @file  ble_service_udf01s.h
 * @brief Provide the Definition of UDF01S Service (User Defined 01 Service).
*****************************************************************************/

#include <stdint.h>
#include "ble_service_base.h"
#include "ble_host.h"
#include "ble_cmd.h"


/**************************************************************************
 * UDF01S Definitions
 **************************************************************************/
/** @defgroup serviceUDF01S_def BLE UDF01S Definitions
 * @{
 * @ingroup service_def
 * @details Here shows the definitions of the UDF01S service.
 * @}
**************************************************************************/

/**
 * @ingroup serviceUDF01S_def
 * @defgroup serviceUDF01S_UUIDDef BLE UDF01S UUID Definitions
 * @{
 * @details Here shows the definitions of the BLE UDF01S UUID Definitions.
*/
extern const uint16_t ATTR_UUID_UDF01S_PRIMARY_SERVICE[];  /**< UDF01S service UUID. */
extern const uint16_t ATTR_UUID_UDF01S_CHARC_UDATR01[];    /**< UDF01S characteristic UDATR01 UUID. */
extern const uint16_t ATTR_UUID_UDF01S_CHARC_UDATN01[];    /**< UDF01S characteristic UDATN01 UUID. */
extern const uint16_t ATTR_UUID_UDF01S_CHARC_UDATRW01[];   /**< UDF01S characteristic UDATRW01 UUID. */
/** @} */

/**
 * @defgroup serviceUDF01S_ServiceChardef BLE UDF01S Service and Characteristic Definitions
 * @{
 * @ingroup serviceUDF01S_def
 * @details Here shows the definitions of the UDF01S service and characteristic.
 * @}
*/

/**
 * @ingroup serviceUDF01S_ServiceChardef
 * @{
*/
extern const ATTRIBUTE_BLE ATT_UDF01S_PRIMARY_SERVICE;                    /**< UDF01S primary Service. */
extern const ATTRIBUTE_BLE ATT_UDF01S_CHARACTERISTIC_UDATR01;             /**< UDF01S characteristic UDATR01. */
extern const ATTRIBUTE_BLE ATT_UDF01S_UDATR01;                            /**< UDF01S UDATR01 value. */
extern const ATTRIBUTE_BLE ATT_UDF01S_CHARACTERISTIC_UDATN01;             /**< UDF01S characteristic UDATN01. */
extern const ATTRIBUTE_BLE ATT_UDF01S_UDATN01;                            /**< UDF01S UDATN01 value. */
extern const ATTRIBUTE_BLE ATT_UDF01S_UDATN01_CLIENT_CHARC_CONFIGURATION; /**< UDF01S Client Characteristic Configuration Descriptor. */
extern const ATTRIBUTE_BLE ATT_UDF01S_CHARACTERISTIC_UDATRW01;            /**< UDF01S characteristic UDATRW01. */
extern const ATTRIBUTE_BLE ATT_UDF01S_UDATRW01;                           /**< UDF01S UDATRW01 value. */
/** @} */


/** UDF01S Service Definition
 * @ingroup serviceUDF01S_ServiceChardef
*/
#define ATT_UDF01S_SERVICE                                          \
    &ATT_UDF01S_PRIMARY_SERVICE,                                    \
    &ATT_UDF01S_CHARACTERISTIC_UDATR01,                             \
    &ATT_UDF01S_UDATR01,                                            \
    &ATT_UDF01S_CHARACTERISTIC_UDATN01,                             \
    &ATT_UDF01S_UDATN01,                                            \
    &ATT_UDF01S_UDATN01_CLIENT_CHARC_CONFIGURATION,                 \
    &ATT_UDF01S_CHARACTERISTIC_UDATRW01,                            \
    &ATT_UDF01S_UDATRW01,                                           \

/**************************************************************************
 * UDF01S Application Definitions
 **************************************************************************/
/** @defgroup appUDF01S_def BLE UDF01S Application Definitions
 * @{
 * @ingroup serviceUDF01S_def
 * @details Here shows the definitions of the UDF01S service for application.
 * @}
**************************************************************************/

/**
 * @ingroup appUDF01S_def
 * @defgroup appUDF01S_eventDef BLE UDF01S Service and Characteristic Definitions
 * @{
 * @details Here shows the event definitions of the UDF01S service.
*/
#define BLESERVICE_UDF01S_UDATR01_READ_EVENT               0x01   /**< UDF01S characteristic UDATR01 read event. */
#define BLESERVICE_UDF01S_UDATR01_READRSP_EVENT            0x02   /**< UDF01S characteristic UDATR01 read response event. */
#define BLESERVICE_UDF01S_UDATN01_NOTIFY_EVENT             0x03   /**< UDF01S characteristic UDATN01 notification event. */
#define BLESERVICE_UDF01S_UDATN01_CCCD_READ_RSP_EVENT      0x04   /**< UDF01S characteristic UDATN01 cccd read response event. */
#define BLESERVICE_UDF01S_UDATN01_CCCD_WRITE_RSP_EVENT     0x05   /**< UDF01S characteristic UDATN01 cccd write response event. */
#define BLESERVICE_UDF01S_UDATRW01_READ_EVENT              0x06   /**< UDF01S characteristic UDATRW01 read event. */
#define BLESERVICE_UDF01S_UDATRW01_READRSP_EVENT           0x07   /**< UDF01S characteristic UDATRW01 read response event. */
#define BLESERVICE_UDF01S_UDATRW01_WRITE_EVENT             0x08   /**< UDF01S characteristic UDATRW01 write event. */
#define BLESERVICE_UDF01S_UDATRW01_WRITE_RSP_EVENT         0x09   /**< UDF01S characteristic UDATRW01 write response event. */
/** @} */


/**
 * @ingroup appUDF01S_def
 * @defgroup appUDF01S_structureDef BLE UDF01S Structure Definitions
 * @{
 * @details Here shows the structure definitions of the UDF01S service.
 * @}
*/

/** UDF01S Handles Definition
 * @ingroup appUDF01S_structureDef
*/
typedef struct BLEATT_UDF01S_Handles
{
    uint16_t hdl_udatr01;           /**< Handle of UDATR01. */
    uint16_t hdl_udatn01;           /**< Handle of UDATN01. */
    uint16_t hdl_udatn01_cccd;      /**< Handle of UDATN01 client characteristic configuration descriptor. */
    uint16_t hdl_udatrw01;          /**< Handle of UDATRW01. */
} BLEATT_UDF01S_Handles;



/** UDF01S Data Definition
 * @ingroup appUDF01S_structureDef
 * @note User defined.
*/
typedef struct BLEATT_UDF01S_Data
{
    uint16_t  udatn01_cccd;          /**< UDATN01 cccd value */
} BLEATT_UDF01S_Data;


/** UDF01S Application Data Structure Definition
 * @ingroup appUDF01S_structureDef
*/
typedef struct BLEATT_UDF01S_Info
{
    BleGattRole                 role;       /**< BLE GATT role. */
    BLEATT_UDF01S_Handles       handles;    /**< UDF01S attribute handles. */
    BLEATT_UDF01S_Data          data;       /**< UDF01S attribute data */
} BLEATT_UDF01S_Info;


/**
 * @ingroup appUDF01S_def
 * @defgroup appUDF01S_App BLE UDF01S Definitions for Application
 * @{
 * @details Here shows the definitions of the UDF01S service for application uses.
 * @}
*/

/** BleUDF01S_EventCallBack
 * @ingroup appUDF01S_App
 * @note This callback receives the UDF01 service events. Each of these events can be associated with parameters.  *
 */
typedef void (*BleUDF01S_EventCallBack)(uint8_t hostId, uint8_t cmdAccess, uint8_t *data, uint16_t length);


/** UDF01 Service Initialization
 *
 * @ingroup appUDF01S_App
 *
 * @param[in] hostId : the link's host id.
 * @param[in] gattRole : @ref BleGattRole "BLE GATT role".
 * @param[in] info : a pointer to UDF01S information.
 * @param[in] callback : a pointer to a callback function that receive the service events.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED : Registered services buffer full.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus setUDF01S_ServiceInit(uint8_t hostId, BleGattRole gattRole, BLEATT_UDF01S_Info *info, BleUDF01S_EventCallBack callback);


/** Get UDF01 Service Handle Numbers
 *
 * @ingroup appUDF01S_App
 *
 * @attention
 *            - role = <b> @ref BLE_GATT_ROLE_CLIENT </b> \n
 *              MUST call this API to get service information after received @ref BLECMD_EVENT_ATT_DATABASE_PARSING_FINISHED  \n
 *            - role = <b> @ref BLE_GATT_ROLE_SERVER </b> \n
 *              MUST call this API to get service information before connection established. \n
 *
 * @param[in] hostId : the link's host id.
 * @param[out] info : a pointer to UDF01S information.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus getUDF01S_ServiceHandles(uint8_t hostId, BLEATT_UDF01S_Info *info);


/** Send Data to Server (Client ONLY)
 *
 * @ingroup appUDF01S_App
 *
 * @param[in] hostId : the link's host id.
 * @param[in] writeType : BLE GATT characteristic write type
 * @param[in] writeNumHdl : handle number of write characteristic.
 * @param[in] data : a pointer to data to send
 * @param[in] length : length of  the data
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_ERR_DB_PARSING_IN_PROGRESS : parsing database process has NOT finished.
 * @retval BLESTACK_STATUS_ERR_INVALID_HANDLE : Invalid attribute handle.
 * @retval BLESTACK_STATUS_ERR_BUSY : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus setUDF01S_ClientDataSend(uint8_t hostId, BleGattWrite writeType, uint16_t writeNumHdl, uint8_t *data, uint8_t length);


/** Get data from server by reading request (Client ONLY)
 *
 * @ingroup appUDF01S_App
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
BleStackStatus getUDF01S_ClientDataRead(uint8_t hostId, uint16_t hdlNum);



/** Send Data to Client (Server ONLY)
 *
 * @ingroup appUDF01S_App
 *
 * @param[in] hostId : the link's host id.
 * @param[in] cccd : BLE GATT characteristic cccd value to indicated notification or indication
 * @param[in] writeNumHdl : handle number of write characteristic.
 * @param[in] data : a pointer to data to send
 * @param[in] length : length of  the data
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_ERR_DB_PARSING_IN_PROGRESS : parsing database process has NOT finished.
 * @retval BLESTACK_STATUS_ERR_INVALID_HANDLE : Invalid attribute handle.
 * @retval BLESTACK_STATUS_ERR_BUSY : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus setUDF01S_ServerDataSend(uint8_t hostId, uint16_t cccd, uint16_t writeNumHdl, uint8_t *data, uint8_t length);

#endif //_BLE_SERVICE_UDF01S_H_

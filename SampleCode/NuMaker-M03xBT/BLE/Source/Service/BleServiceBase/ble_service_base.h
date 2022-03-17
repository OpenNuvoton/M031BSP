#ifndef _BLE_SERVICE_BASE_H_
#define _BLE_SERVICE_BASE_H_

/**************************************************************************//**
 * @file  ble_service_base.h
 * @brief Provide the Based Definition of BLE Profile.
*****************************************************************************/

#include <stdint.h>
#include "ble_uuid.h"
#include "ble_host.h"
#include "ble_cmd.h"
#include "processors_compiler.h"

/**************************************************************************
* BLE Profile Based Definitions
**************************************************************************/
/**
 * @defgroup service_allDef BLE Services
 * @{
 * @details This file shows the BLE defined-services definitions and functions.
 * @defgroup service_basedDef BLE Service Based Definitions
 * @{
 * @details Here shows the definitions of the defined service based definitions.
 * @}
 * @defgroup service_def BLE Defined-Services
 * @{
 * @details Here shows the definitions of the defined services.
 * @}
 * @}
**************************************************************************/



/** Marcro retrun if input callback function is  equal to NULL.
 *
 * @ingroup service_basedDef
 * @param[in] callback BLE service callback function.
 */
#define SERVICE_CALLBACK_NULL_CHECK(callback)        \
    if (callback == NULL)                            \
    {                                                \
        return;                                      \
    }



/** Length of @ref Charac_Presentation_Format for decoded data.
 * @ingroup service_basedDef
*/
#define BLE_CHARAC_PRESENTATION_FORMAT_LEN               7


/**
 * @ingroup service_basedDef
 * @defgroup service_basedUUIDDef BLE Based UUID Definitions
 * @{
 * @details Here shows the definitions of the BLE based UUID Definitions.
*/
extern const uint16_t ATTR_UUID_INVALID[];                            /**< Invalid UUID. */
extern const uint16_t ATTR_UUID_TYPE_PRIMARY_SERVICE[];               /**< Primary Service UUID. */
extern const uint16_t ATTR_UUID_TYPE_SECONDARY_SERVICE[];             /**< Secondary Service UUID. */
extern const uint16_t ATTR_UUID_TYPE_INCLUDE[];                       /**< Included Service UUID. */
extern const uint16_t ATTR_UUID_TYPE_CHARACTERISTIC[];                /**< Characteristic UUID. */
extern const uint16_t ATTR_UUID_TYPE_CHARC_EXTENDED_PROPERTIES[];     /**< Characteristic extended properties UUID. */
extern const uint16_t ATTR_UUID_TYPE_CHARC_USER_DESCRIPTION[];        /**< Characteristic user description UUID. */
extern const uint16_t ATTR_UUID_TYPE_CLIENT_CHARC_CONFIGURATION[];    /**< Client characteristic configuration descriptor UUID. */
extern const uint16_t ATTR_UUID_TYPE_SERVER_CHARC_CONFIGURATION[];    /**< Client characteristic configuration UUID. */
extern const uint16_t ATTR_UUID_TYPE_CHARC_PRESENTATION_FORMAT[];     /**< Characteristic presentation format UUID. */
extern const uint16_t ATTR_UUID_TYPE_CHARC_AGGREGATE_FORMAT[];        /**< Characteristic aggregate format UUID. */
extern const uint16_t ATTR_UUID_TYPE_VALID_RANGE[];                   /**< Valid range UUID. */
extern const uint16_t ATTR_UUID_TYPE_EXTERNAL_REPORT_REFERENCE[];     /**< External report reference UUID. */
extern const uint16_t ATTR_UUID_TYPE_REPORT_REFERENCE[];              /**< Report reference UUID. */
/** @} */


/**
 * @ingroup service_basedDef
 * @defgroup service_nullDef BLE Based Definitions
 * @{
 * @details Here shows the definitions of the BLE based Definitions.
 * @}
*/

/** NULL access function for service defined uses.
 * @ingroup service_nullDef
*/
void ATTR_NULL_Access(BLE_Event_AttParam *AttParam);



/** NULL definition for service defined uses.
 * @ingroup service_nullDef
*/
extern const ATTRIBUTE_BLE ATT_NULL_INVALID;



/**
 * @ingroup service_basedDef
 * @defgroup appBase_generalDef BLE Based General Structures
 * @{
 * @details Here shows the general definitions of structure.
*/


/** @brief Attribute Error Response Structure Definition.
 * @ingroup appBase_generalDef
 * @note Attribute error response data structure for handling data in @ref OPCODE_ATT_ERROR_RESPONSE.
 * @{
*/
struct Att_Error_Rsp_Data
{
    uint8_t       request_opcode; /**< The request that generated this error response. @ref bleAttOpCode.*/
    uint16_t      att_handler;    /**< The attribute handle that generated this error response. */
    uint8_t       error_code;     /**< The reason why the request has generated an error response. @ref bleAttErrorCode. */
} __PACKED;

/** @brief Attribute Error Response Structure Definition. */
typedef struct Att_Error_Rsp_Data Att_Error_Rsp_Data;
/** @} */


/** @brief String Format Structure Definition.
 * @ingroup appBase_generalDef
*/
typedef struct String_Format
{
    uint8_t     *str;         /**< String data. */
    uint8_t     length;       /**< String length. */
} String_Format;



/** @brief Characteristic Presentation Format Structure Definition.
 * @ingroup appBase_generalDef
*/
typedef struct Charac_Presentation_Format
{
    uint8_t     format;       /**< Format of the value of this characteristic. */
    uint8_t     exponent;     /**< Exponent field to determine how the value of this characteristic is further formatted. */
    uint16_t    unit;         /**< The unit of this characteristic. */
    uint8_t     name_space;   /**< The name space of the description. */
    uint16_t    description;  /**< The description of this characteristic as defined in a higher layer profile. */
} Charac_Presentation_Format;



/** @brief Service Basic Information Structure Definition.
 * @ingroup appBase_generalDef
*/
typedef struct Service_Basic_Info
{
    uint8_t       index;      /**< Service information index. */
    uint8_t       hostId;     /**< Host id. */
    BleGattRole   role;       /**< GATT role. */
} Service_Basic_Info;


/** @} */


/**
 * @ingroup service_basedDef
 * @defgroup appBase_App BLE Based Functions
 * @{
 * @details Here shows the definitions of functions.
 * @}
*/

/** Query Index from Service Registered Index Mapping Array
 *
 * @ingroup appBase_App
 *
 * @param[in] hostId : the link's host id.
 * @param[in] gattRole : @ref BleGattRole "BLE GATT role".
 * @param[in] maxCount : maximum number of this registered service.
 * @param[in] info : a pointer to basic service information.
 * @param[out] index : a pointer to the index.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : error host id.
 * @retval BLESTACK_STATUS_SUCCESS  : setting success.
*/
BleStackStatus queryIndexByHostIdGattRole(uint8_t hostId, BleGattRole gattRole, uint8_t maxCount, Service_Basic_Info *info, uint8_t *index);



/** BLE Service Basic Initialization
 *
 * @ingroup appBase_App
 *
 * @param[in] hostId : the link's host id.
 * @param[in] gattRole : @ref BleGattRole "BLE GATT role".
 * @param[in] maxCount : maximum number of this registered service.
 * @param[out] info : a pointer to service basic information.
 * @param[out] serviceIndex : a pointer to the index.
 * @param[out] serviceCount : a pointer to the total count of this registered service.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : error host id.
 * @retval BLESTACK_STATUS_SUCCESS  : setting success.
*/
BleStackStatus setBLE_ServiceBasicInit(uint8_t hostId, BleGattRole gattRole, uint8_t maxCount, Service_Basic_Info *info, uint8_t *serviceIndex, uint8_t *serviceCount);



/** BLE Set Handle CCCD Write Request From the Client
 *
 * @ingroup appBase_App
 *
 * @note The server defined cccd value will be updated to the attribute data of write request from client.
 * @note This function supports for GATT role @ref BLE_GATT_ROLE_SERVER only.
 *
 * @param[in]  wqData        : a pointer to the attribute data of write request.
 * @param[in]  wqDataLength  : the length of the attribute data of write request.
 * @param[out] cccd          : a pointer to the server defined cccd which will be updated.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM                : Invalid length of the attribute data of write request.
 * @retval BLESTACK_STATUS_SUCCESS                          : Setting success.
 */
BleStackStatus setBLEGATT_HandleCCCDWriteRequest(uint8_t *wqData, uint16_t wqDataLength, uint16_t *cccd);


/** BLE Set Handle CCCD Read Request From the Client
 *
 * @ingroup appBase_App
 *
 * @note This function will send Read Response to the client.
 * @note This function supports for GATT role @ref BLE_GATT_ROLE_SERVER only.
 *
 * @param[in] hostId        : the link's host id.
 * @param[in] cccdHandleNum : cccd attribute handle number.
 * @param[in] cccdValue     : cccd attribute data.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID          : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED           : Invalid GATT role, only supports for server.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE           : Invalid BLE state, usually happens in there is no connection established with the host id.
 * @retval BLESTACK_STATUS_ERR_DB_PARSING_IN_PROGRESS  : Services and characteristics discovering have not completed.
 * @retval BLESTACK_STATUS_ERR_INVALID_HANDLE          : Invalid handle.
 * @retval BLESTACK_STATUS_ERR_BUSY                    : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS                     : Setting success.
 */
BleStackStatus setBLEGATT_HandleCCCDReadRequest(uint8_t hostId, uint16_t cccdHandleNum, uint16_t cccdValue);


/** BLE Set Handle CCCD Read By Type Request From the Client
 *
 * @ingroup appBase_App
 *
 * @note This function will send Read By Type Response to the client.
 * @note This function supports for GATT role @ref BLE_GATT_ROLE_SERVER only.
 *
 * @param[in] hostId        : the link's host id.
 * @param[in] cccdHandleNum : cccd attribute handle number.
 * @param[in] cccdValue     : cccd attribute data.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID          : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED           : Invalid GATT role, only supports for server.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE           : Invalid BLE state, usually happens in there is no connection established with the host id.
 * @retval BLESTACK_STATUS_ERR_DB_PARSING_IN_PROGRESS  : Services and characteristics discovering have not completed.
 * @retval BLESTACK_STATUS_ERR_INVALID_HANDLE          : Invalid handle.
 * @retval BLESTACK_STATUS_ERR_BUSY                    : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS                     : Setting success.
 */
BleStackStatus setBLEGATT_HandleCCCDReadByTypeRequest(uint8_t hostId, uint16_t cccdHandleNum, uint16_t cccdValue);


/** BLE Set Automatically Handle CCCD Read Request or Read By Type Request From the Client
 *
 * @ingroup appBase_App
 *
 * @note This function will automatically send Read Response or Read By Type Response to the client.
 * @note This function supports for GATT role @ref BLE_GATT_ROLE_SERVER only.
 *
 * @param[in] hostId        : the link's host id.
 * @param[in] opcode        : @ref OPCODE_ATT_READ_REQUEST or @ref OPCODE_ATT_FIND_BY_TYPE_VALUE_REQUEST
 * @param[in] cccdHandleNum : cccd attribute handle number.
 * @param[in] cccdValue     : cccd attribute data.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID          : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED           : Invalid GATT role, only supports for server.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE           : Invalid BLE state, usually happens in there is no connection established with the host id.
 * @retval BLESTACK_STATUS_ERR_DB_PARSING_IN_PROGRESS  : Services and characteristics discovering have not completed.
 * @retval BLESTACK_STATUS_ERR_INVALID_HANDLE          : Invalid handle.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM           : Invalid parameter, please check the input opcode.
 * @retval BLESTACK_STATUS_ERR_BUSY                    : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS                     : Setting success.
 */
BleStackStatus setBLEGATT_HandleCCCDGeneralReadRequest(uint8_t hostId, uint8_t opcode, uint16_t cccdHandleNum, uint16_t cccdValue);


/** Handle BLE Service Read Blob Request
 *
 * @ingroup appBase_App
 *
 * @param[in] attParam : a pointer to attribute parameter.
 * @param[out] readData : a pointer to "read" data.
 * @param[out] reaDataLength : the length of "read" data.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE  : Invalid BLE state, usually happens in there is no connection established with the host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_HANDLE : Invalid handle.
 * @retval BLESTACK_STATUS_ERR_BUSY           : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS            : Setting success.
*/
BleStackStatus setBLEGATT_HandleReadBlobRequest(BLE_Event_AttParam *attParam, uint8_t *readData, uint8_t reaDataLength);


#endif // _BLE_SERVICE_BASE_H_

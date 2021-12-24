/************************************************************************
 *
 * File Name  : BLE_SERVICE_BASE.c
 * Description: This file contains the based definitions and functions of BLE profile
 *
 *
 ************************************************************************/
#include "ble_service_base.h"
#include "BleAppSetting.h"


/**************************************************************************
 * BLE Profile Based UUID Definitions
 **************************************************************************/
const uint16_t ATTR_UUID_INVALID[] =
{
    0,
};


/** Primary Service Descriptors UUID.
 * @note 16-bits UUID
 * @note UUID: 2800
*/
const uint16_t ATTR_UUID_TYPE_PRIMARY_SERVICE[] =
{
    GATT_DECL_PRIMARY_SERVICE,
};


/** Secondary Service Descriptors UUID.
 * @note 16-bits UUID
 * @note UUID: 2801
*/
const uint16_t ATTR_UUID_TYPE_SECONDARY_SERVICE[] =
{
    GATT_DECL_SECONDARY_SERVICE,
};


/** Included Service Descriptors UUID.
 * @note 16-bits UUID
 * @note UUID: 2802
*/
const uint16_t ATTR_UUID_TYPE_INCLUDE[] =
{
    GATT_DECL_INCLUDE,
};


/** Characteristic Descriptors UUID.
 * @note 16-bits UUID
 * @note UUID: 2803
*/
const uint16_t ATTR_UUID_TYPE_CHARACTERISTIC[] =
{
    GATT_DECL_CHARACTERISTIC,
};


/** Characteristic Extended Properties Descriptor UUID.
 * @note 16-bits UUID
 * @note UUID: 2900
*/
const uint16_t ATTR_UUID_TYPE_CHARC_EXTENDED_PROPERTIES[] =
{
    GATT_DESC_CHARC_EXTENDED_PROPERTIES,
};


/** Characteristic User Description Descriptor UUID.
 * @note 16-bits UUID
 * @note UUID: 2901
*/
const uint16_t ATTR_UUID_TYPE_CHARC_USER_DESCRIPTION[] =
{
    GATT_DESC_CHARC_USER_DESCRIPTION,
};


/** Client Characteristic Configuration Descriptor UUID.
 * @note 16-bits UUID
 * @note UUID: 2902
*/
const uint16_t ATTR_UUID_TYPE_CLIENT_CHARC_CONFIGURATION[] =
{
    GATT_DESC_CLIENT_CHARC_CONFIGURATION,
};


/** Server Characteristic Configuration Descriptor UUID.
 * @note 16-bits UUID
 * @note UUID: 2903
*/
const uint16_t ATTR_UUID_TYPE_SERVER_CHARC_CONFIGURATION[] =
{
    GATT_DESC_SERVER_CHARC_CONFIGURATION,
};


/** Characteristic Presentation Format Descriptor UUID.
 * @note 16-bits UUID
 * @note UUID: 2904
*/
const uint16_t ATTR_UUID_TYPE_CHARC_PRESENTATION_FORMAT[] =
{
    GATT_DESC_CHARC_PRESENTATION_FORMAT,
};


/** Characteristic Aggregate Format Descriptor UUID.
 * @note 16-bits UUID
 * @note UUID: 2905
*/
const uint16_t ATTR_UUID_TYPE_CHARC_AGGREGATE_FORMAT[] =
{
    GATT_DESC_CHARC_AGGREGATE_FORMAT,
};

/** Valid Range Descriptor UUID.
 * @note 16-bits UUID
 * @note UUID: 2906
*/
const uint16_t ATTR_UUID_TYPE_VALID_RANGE[] =
{
    GATT_DESC_VALID_RANGE,
};


/** Characteristic Aggregate Format Descriptor UUID.
 * @note 16-bits UUID
 * @note UUID: 2907
*/
const uint16_t ATTR_UUID_TYPE_EXTERNAL_REPORT_REFERENCE[] =
{
    GATT_DESC_EXTERNAL_REPORT_REFERENCE,
};


/** Report Reference Descriptor UUID.
 * @note 16-bits UUID
 * @note UUID: 2908
*/
const uint16_t ATTR_UUID_TYPE_REPORT_REFERENCE[] =
{
    GATT_DESC_REPORT_REFERENCE,
};



/**************************************************************************
 * BLE Profile NULL Access Definition
 **************************************************************************/
void ATTR_NULL_Access(BLE_Event_AttParam *AttParam)
{
}


/**************************************************************************
 * BLE Profile NULL Definition
 **************************************************************************/
#define ATTR_TEMP_DEF        \
    (void *)0,          \
    (void *)0,          \
        0,              \
        0,              \
        0,              \
    ATTR_NULL_Access,   \


const ATTRIBUTE_BLE ATT_NULL_INVALID =
{
    ATTR_TEMP_DEF
};



/**************************************************************************
 * BLE Profile Public Functions
 **************************************************************************/


/** Query Index from Service Registered Index Mapping Array
 *
 * @param[in] hostId : the link's host id.
 * @param[in] gattRole : @ref BleGattRole "BLE GATT role".
 * @param[in] maxCount : maximum number of this registered service.
 * @param[in] info : a pointer to basic service information.
 * @param[out] index : a pointer to the index.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus queryIndexByHostIdGattRole(uint8_t hostId, BleGattRole gattRole, uint8_t maxCount, Service_Basic_Info *info, uint8_t *index)
{
    uint8_t i;
    for (i = 0; i < maxCount; i++)
    {
        if ((info[i].hostId == hostId) && (info[i].role == gattRole))
        {
            *index = i;
            return BLESTACK_STATUS_SUCCESS;
        }
    }

    return BLESTACK_STATUS_ERR_INVALID_PARAM;
}



/** BLE Service Basic Initialization
 *
 * @param[in] hostId : the link's host id.
 * @param[in] gattRole : @ref BleGattRole "BLE GATT role".
 * @param[in] maxCount : maximum number of this registered service.
 * @param[out] info : a pointer to service basic information.
 * @param[out] serviceIndex : a pointer to the index.
 * @param[out] serviceCount : a pointer to the total count of this registered service.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_SUCCESS            : Setting success.
*/
BleStackStatus setBLE_ServiceBasicInit(uint8_t hostId, BleGattRole gattRole, uint8_t maxCount, Service_Basic_Info *info, uint8_t *serviceIndex, uint8_t *serviceCount)
{
    // check host id
    if ( (hostId == BLE_HOSTID_RESERVED) ||
            (hostId >= BLE_SUPPORT_NUM_CONN_MAX))
    {
        return BLESTACK_STATUS_ERR_INVALID_HOSTID;
    }

    if ((serviceCount != 0) && (queryIndexByHostIdGattRole(hostId, gattRole, (*serviceCount), info, serviceIndex) == BLESTACK_STATUS_SUCCESS))
    {
        // already exist
    }
    else
    {
        // check service supported maximum count
        if (*serviceCount >=  maxCount)
        {
            return BLESTACK_STATUS_ERR_NOT_SUPPORTED;
        }

        // new link
        *serviceIndex = *serviceCount;

        // count++ for next link
        (*serviceCount)++;
    }

    info[*serviceIndex].index  = *serviceIndex;
    info[*serviceIndex].role   = gattRole;
    info[*serviceIndex].hostId = hostId;

    return BLESTACK_STATUS_SUCCESS;
}


/** BLE Set Handle CCCD Write Request From the Client
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
BleStackStatus setBLEGATT_HandleCCCDWriteRequest(uint8_t *wqData, uint16_t wqDataLength, uint16_t *cccd)
{
    uint16_t cccdValue;

    if (wqDataLength != 2)
    {
        return BLESTACK_STATUS_ERR_INVALID_PARAM;
    }

    cccdValue = ((uint16_t)wqData[0] | ((uint16_t)wqData[1] << 8));

    if ( (cccdValue != BLEGATT_CCCD_NONE) &&
            (cccdValue != BLEGATT_CCCD_NOTIFICATION) &&
            (cccdValue != BLEGATT_CCCD_INDICATION) &&
            (cccdValue != BLEGATT_CCCD_NOTIFY_INDICATE))
    {
        return BLESTACK_STATUS_ERR_INVALID_PARAM;
    }

    *cccd = cccdValue;

    return BLESTACK_STATUS_SUCCESS;
}


/** BLE Set Handle CCCD Read Request From the Client
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
BleStackStatus setBLEGATT_HandleCCCDReadRequest(uint8_t hostId, uint16_t cccdHandleNum, uint16_t cccdValue)
{
    BleStackStatus status;
    uint8_t cccd[2];

    // check cccdValue is valid or not
    if ( (cccdValue != BLEGATT_CCCD_NONE) &&
            (cccdValue != BLEGATT_CCCD_NOTIFICATION) &&
            (cccdValue != BLEGATT_CCCD_INDICATION) &&
            (cccdValue != BLEGATT_CCCD_NOTIFY_INDICATE))
    {
        return BLESTACK_STATUS_ERR_INVALID_PARAM;
    }

    cccd[0] = (uint8_t)(cccdValue & 0xFF);
    cccd[1] = (uint8_t)((cccdValue >> 8) & 0xFF);

    status = setBLEGATT_ReadRsp(hostId, cccdHandleNum, (uint8_t *)cccd, 2);

    return status;
}

/** BLE Set Handle CCCD Read By Type Request From the Client
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
BleStackStatus setBLEGATT_HandleCCCDReadByTypeRequest(uint8_t hostId, uint16_t cccdHandleNum, uint16_t cccdValue)
{
    BleStackStatus status;
    uint8_t cccd[2];

    // check cccdValue is valid or not
    if ( (cccdValue != BLEGATT_CCCD_NONE) &&
            (cccdValue != BLEGATT_CCCD_NOTIFICATION) &&
            (cccdValue != BLEGATT_CCCD_INDICATION) &&
            (cccdValue != BLEGATT_CCCD_NOTIFY_INDICATE))
    {
        return BLESTACK_STATUS_ERR_INVALID_PARAM;
    }

    cccd[0] = (uint8_t)(cccdValue & 0xFF);
    cccd[1] = (uint8_t)((cccdValue >> 8) & 0xFF);

    status = setBLEGATT_ReadByTypeRsp(hostId, cccdHandleNum, (uint8_t *)cccd, 2);

    return status;
}


/** BLE Set Automatically Handle CCCD Read Request or Read By Type Request From the Client
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
BleStackStatus setBLEGATT_HandleCCCDGeneralReadRequest(uint8_t hostId, uint8_t opcode, uint16_t cccdHandleNum, uint16_t cccdValue)
{
    switch (opcode)
    {
    case  OPCODE_ATT_FIND_BY_TYPE_VALUE_REQUEST:
        return setBLEGATT_HandleCCCDReadByTypeRequest(hostId, cccdHandleNum, cccdValue);
    case  OPCODE_ATT_READ_REQUEST:
        return setBLEGATT_HandleCCCDReadRequest(hostId, cccdHandleNum, cccdValue);

    default:
        break;
    }

    return BLESTACK_STATUS_ERR_INVALID_PARAM;
}


/** Handle BLE Service Read Blob Request
 *
 * @param[in] attParam : a pointer to attribute parameter.
 * @param[out] readData : a pointer to "read" data.
 * @param[out] reaDataLength : the length of "read" data.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Rrror host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE  : Invalid BLE state, usually happens in there is no connection established with the host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_HANDLE : Invalid handle.
 * @retval BLESTACK_STATUS_ERR_BUSY           : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS            : Setting success.
*/
BleStackStatus setBLEGATT_HandleReadBlobRequest(BLE_Event_AttParam *attParam, uint8_t *readData, uint8_t reaDataLength)
{
    BleAttErrorRsp errRsp = ERR_CODE_ATT_NO_ERROR;
    BleStackStatus status;
    uint16_t offset;
    uint16_t mtuSize;

    // get current MTU size
    status = getBLEGATT_MtuSize(attParam->hostId, &mtuSize);
    if (status != BLESTACK_STATUS_SUCCESS)
    {
        return status;
    }

    // calculate offset
    offset = (((uint16_t)attParam->data[1] << 8) | attParam->data[0]);

    // check offset + length
    if ((offset != 0) && (reaDataLength <= (mtuSize - 1)))
    {
        errRsp = ERR_CODE_ATT_ATTRIBUTE_NOT_LONG;
    }
    else if (offset >= reaDataLength)
    {
        errRsp = ERR_CODE_ATT_INVALID_OFFSET;
    }

    if (errRsp != ERR_CODE_ATT_NO_ERROR)
    {
        // send error rsp
        status = setBLEGATT_ErrorRsp(attParam->hostId, attParam->hdlNum, OPCODE_ATT_READ_BLOB_REQUEST, errRsp);
        if (status != BLESTACK_STATUS_SUCCESS)
        {
            return status;
        }
    }
    else
    {
        // send read blob rsp
        status = setBLEGATT_ReadBlobRsp(attParam->hostId, attParam->hdlNum, (readData + offset), (reaDataLength - offset));
        if (status != BLESTACK_STATUS_SUCCESS)
        {
            return status;
        }
    }

    return BLESTACK_STATUS_SUCCESS;
}



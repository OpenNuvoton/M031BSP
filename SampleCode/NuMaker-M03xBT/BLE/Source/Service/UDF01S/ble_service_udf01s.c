/************************************************************************
 *
 * File Name  : BLE_SERVICE_UDF01S.c
 * Description: This file contains the definitions and functions of BLE UDF01S
 *
 *
 ************************************************************************/
#include "ble_service_udf01s.h"
#include "ble_profile.h"

/** ATTR_VALUE_UDF01S_General_Access
 * @note This callback receives the UDF01S Service events.  \n
 *  Each of these events can be associated with parameters.
 */
void ATTR_VALUE_UDF01S_General_Access(BLE_Event_AttParam *attParam);

/**************************************************************************
 * UDF01S Service UUID Definitions
 **************************************************************************/

/** UDF01S Service UUID.
 * @note 128-bits UUID
 * @note UUID: 00112233445566778899AABBCCDDEEFF
*/
const uint16_t ATTR_UUID_UDF01S_PRIMARY_SERVICE[] =
{
    0xEEFF,  0xCCDD,
    0xAABB,  0x8899,
    0x6677,  0x4455,
    0x2233,  0x0011,
};

/** UDF01S characteristic UDATR01 UUID.
 * @note 128-bits UUID
 * @note UUID: 101112131415161718191A1B1C1D1E1F
*/
const uint16_t ATTR_UUID_UDF01S_CHARC_UDATR01[] =
{
    0x1E1F, 0x1C1D,
    0x1A1B, 0x1819,
    0x1617, 0x1415,
    0x1213, 0x1011,
};

/** UDF01S characteristic UDATN01 UUID.
 * @note 128-bits UUID
 * @note UUID: 303132333435363738393A3B3C3D3E3F
*/
const uint16_t ATTR_UUID_UDF01S_CHARC_UDATN01[] =
{
    0x3E3F, 0x3C3D,
    0x3A3B, 0x3839,
    0x3637, 0x3435,
    0x3233, 0x3031,
};

/** UDF01S characteristic UDATRW01 UUID.
 * @note 128-bits UUID
 * @note UUID: 505152535455565758595A5B5C5D5E5F
*/
const uint16_t ATTR_UUID_UDF01S_CHARC_UDATRW01[] =
{
    0x5E5F, 0x5C5D,
    0x5A5B, 0x5859,
    0x5657, 0x5455,
    0x5253, 0x5051,
};


/**************************************************************************
 * UDF01S Service Value Definitions
 **************************************************************************/

/** UDF01S characteristic UDATR01 Value.
 * @note Return the "Read data" when client send "Read Request".
*/
#define ATTR_VALUE_UDF01S_UDATR01     "UDF01S UDATR01 Data"


/**************************************************************************
 * UDF01S Service/ Characteristic Definitions
 **************************************************************************/

const ATTRIBUTE_BLE ATT_UDF01S_PRIMARY_SERVICE =
{
    (void *)ATTR_UUID_TYPE_PRIMARY_SERVICE,
    (void *)ATTR_UUID_UDF01S_PRIMARY_SERVICE,
    sizeof(ATTR_UUID_UDF01S_PRIMARY_SERVICE),
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_NULL_Access,                       //This function should be set to ATTR_NULL_Access when lenValue or uuidValue is a null value.
};

const ATTRIBUTE_BLE ATT_UDF01S_CHARACTERISTIC_UDATR01 =
{
    (void *)ATTR_UUID_TYPE_CHARACTERISTIC,
    (void *)ATTR_UUID_UDF01S_CHARC_UDATR01,
    sizeof(ATTR_UUID_UDF01S_CHARC_UDATR01),
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_NULL_Access,                       //This function should be set to ATTR_NULL_Access when lenValue or uuidValue is a null value.
};

const ATTRIBUTE_BLE ATT_UDF01S_UDATR01 =
{
    (void *)ATTR_UUID_UDF01S_CHARC_UDATR01,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        //ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_VALUE_UDF01S_General_Access,       //registered callback function
};

const ATTRIBUTE_BLE ATT_UDF01S_CHARACTERISTIC_UDATN01 =
{
    (void *)ATTR_UUID_TYPE_CHARACTERISTIC,
    (void *)ATTR_UUID_UDF01S_CHARC_UDATN01,
    sizeof(ATTR_UUID_UDF01S_CHARC_UDATN01),
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_NULL_Access,                       //This function should be set to ATTR_NULL_Access when lenValue or uuidValue is a null value.
};

const ATTRIBUTE_BLE ATT_UDF01S_UDATN01 =
{
    (void *)ATTR_UUID_UDF01S_CHARC_UDATN01,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        //GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        //ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_VALUE_UDF01S_General_Access,       //registered callback function
};

const ATTRIBUTE_BLE ATT_UDF01S_UDATN01_CLIENT_CHARC_CONFIGURATION =
{
    (void *)ATTR_UUID_TYPE_CLIENT_CHARC_CONFIGURATION,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_VALUE_UDF01S_General_Access,       //registered callback function
};

const ATTRIBUTE_BLE ATT_UDF01S_CHARACTERISTIC_UDATRW01 =
{
    (void *)ATTR_UUID_TYPE_CHARACTERISTIC,
    (void *)ATTR_UUID_UDF01S_CHARC_UDATRW01,
    sizeof(ATTR_UUID_UDF01S_CHARC_UDATRW01),
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_NULL_Access,                       //This function should be set to ATTR_NULL_Access when lenValue or uuidValue is a null value.
};

const ATTRIBUTE_BLE ATT_UDF01S_UDATRW01 =
{
    (void *)ATTR_UUID_UDF01S_CHARC_UDATRW01,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        //ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_VALUE_UDF01S_General_Access,       //registered callback function
};


/**************************************************************************
 * BLE Service << UDF01S >> Local Variable
 **************************************************************************/
#ifndef MAX_NUM_CONN_UDF01S
// check MAX_NUM_CONN_UDF01S if defined or set to default 1.
#define MAX_NUM_CONN_UDF01S       1
#endif


// Service basic information
Service_Basic_Info                udf01sBasicInfo[MAX_NUM_CONN_UDF01S];

// UDF01S information
BLEATT_UDF01S_Info                *udf01sInfo[MAX_NUM_CONN_UDF01S];

// UDF01S callback function
BleUDF01S_EventCallBack           udf01sCallback[MAX_NUM_CONN_UDF01S];

// UDF01S registered total count
uint8_t udf01s_count = 0;


/**************************************************************************
 * BLE Service << UDF01S >> Public Function
 **************************************************************************/
/** UDF01 Service Initialization
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
BleStackStatus setUDF01S_ServiceInit(uint8_t hostId, BleGattRole gattRole, BLEATT_UDF01S_Info *info, BleUDF01S_EventCallBack callback)
{
    BleStackStatus status;
    uint8_t config_index;

    if (info == NULL)
    {
        return BLESTACK_STATUS_ERR_INVALID_PARAM;
    }

    // init service basic information and get "config_index" & "udf01s_count"
    status = setBLE_ServiceBasicInit(hostId, gattRole, MAX_NUM_CONN_UDF01S, udf01sBasicInfo, &config_index, &udf01s_count);
    BLESTACK_STATUS_CHECK(status);

    // Set service role
    info->role = gattRole;

    // Set UDF01S data
    udf01sInfo[config_index] = info;

    // Register UDF01S callback function
    udf01sCallback[config_index] = callback;

    // Get handles at initialization if role is set to BLE_GATT_ROLE_SERVER
    if (gattRole == BLE_GATT_ROLE_SERVER)
    {
        status = getUDF01S_ServiceHandles(hostId, udf01sInfo[config_index]);
        BLESTACK_STATUS_CHECK(status);
    }

    return BLESTACK_STATUS_SUCCESS;
}



/** Get UDF01 Service Handle Numbers
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
BleStackStatus getUDF01S_ServiceHandles(uint8_t hostId, BLEATT_UDF01S_Info *info)
{
    BleStackStatus status;

    // Get UDF01S handles
    status = getBLEGATT_HandleNumAddr(hostId, info->role, (ATTRIBUTE_BLE *)&ATT_UDF01S_PRIMARY_SERVICE, (void *)&info->handles);
    BLESTACK_STATUS_CHECK(status);

    return BLESTACK_STATUS_SUCCESS;
}


/** Send Data to Server (Client ONLY)
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
BleStackStatus setUDF01S_ClientDataSend(uint8_t hostId, BleGattWrite writeType, uint16_t writeNumHdl, uint8_t *data, uint8_t length)
{
    BleStackStatus status;
    uint16_t mtuSize;

    // get current MTU size
    status = getBLEGATT_MtuSize(hostId, &mtuSize);
    BLESTACK_STATUS_CHECK(status);

    // check data length <= Current MTU size
    if (length > (mtuSize - 3)) // 3 bytes header
    {
        return BLESTACK_STATUS_ERR_INVALID_PARAM;
    }

    if (writeType == BLEGATT_WRITE)
    {
        // write request
        status = setBLEGATT_Write(hostId, writeNumHdl, data, length);
    }
    else if (writeType == BLEGATT_WRITE_WITHOUT_RSP)
    {
        // write command
        status = setBLEGATT_WriteWithoutRsp(hostId, writeNumHdl, data, length);
    }
    else
    {
        return BLESTACK_STATUS_ERR_INVALID_PARAM;
    }

    return status;
}



/** Send Data to Client (Server ONLY)
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
BleStackStatus setUDF01S_ServerDataSend(uint8_t hostId, uint16_t cccd, uint16_t writeNumHdl, uint8_t *data, uint8_t length)
{
    BleStackStatus status;
    uint16_t mtuSize;

    // get current MTU size
    status = getBLEGATT_MtuSize(hostId, &mtuSize);

    if (status != BLESTACK_STATUS_SUCCESS)
    {
        return status;
    }

    // check data length <= Current MTU size
    if (length > (mtuSize - 3)) // 3 bytes header
    {
        return BLESTACK_STATUS_ERR_INVALID_PARAM;
    }

    if (cccd == BLEGATT_CCCD_INDICATION)
    {
        // write request
        status = setBLEGATT_Indication(hostId, writeNumHdl, data, length);
    }
    else if (cccd == BLEGATT_CCCD_NOTIFICATION)
    {
        // write command
        status = setBLEGATT_Notification(hostId, writeNumHdl, data, length);
    }
    else
    {
        return BLESTACK_STATUS_ERR_INVALID_PARAM;
    }

    return status;
}



/** Get data from server by reading request (Client ONLY)
 *
 * @param[in] hostId : the link's host id
 * @param[in] hdlNum : handle numnber of the data you want to read.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_ERR_DB_PARSING_IN_PROGRESS : parsing database process has NOT finished.
 * @retval BLESTACK_STATUS_ERR_INVALID_HANDLE : Invalid attribute handle.
 * @retval BLESTACK_STATUS_ERR_BUSY : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus getUDF01S_ClientDataRead(uint8_t hostId, uint16_t hdlNum)
{
    BleStackStatus status;

    status = setBLEGATT_ReadCharacteristicValue(hostId, hdlNum);

    return status;
}

/**************************************************************************
 * BLE Service << UDF01S >> General Callback Function
 **************************************************************************/
static void bleUDF01S_PostEvent(BLE_Event_AttParam *attParam, BleUDF01S_EventCallBack *callback)
{
    // check callback is null or not
    SERVICE_CALLBACK_NULL_CHECK(*callback);

    // post event to user
    (*callback)(attParam->hostId, attParam->cmdAccess, attParam->data, attParam->length);
}


// handle UDF01S client GATT event
static void handle_UDF01S_client(uint8_t index, BLE_Event_AttParam *attParam)
{
    switch (attParam->cmdAccess)
    {
    case OPCODE_ATT_WRITE_RESPONSE:
        if (attParam->hdlNum == udf01sInfo[index]->handles.hdl_udatn01_cccd)
        {
            // received write response from server -> cccd configure completed
            attParam->cmdAccess = BLESERVICE_UDF01S_UDATN01_CCCD_WRITE_RSP_EVENT;
            bleUDF01S_PostEvent(attParam, &udf01sCallback[index]);
        }
        else
        {
            // received write response from server
            attParam->cmdAccess = BLESERVICE_UDF01S_UDATRW01_WRITE_RSP_EVENT;
            bleUDF01S_PostEvent(attParam, &udf01sCallback[index]);
        }
        break;

    case OPCODE_ATT_READ_RESPONSE:
        if (attParam->hdlNum == udf01sInfo[index]->handles.hdl_udatn01_cccd)
        {
            // received read response (cccd value) from server
            attParam->cmdAccess = BLESERVICE_UDF01S_UDATN01_CCCD_READ_RSP_EVENT;
            bleUDF01S_PostEvent(attParam, &udf01sCallback[index]);
        }
        break;

    case OPCODE_ATT_HANDLE_VALUE_NOTIFICATION:
        if (attParam->hdlNum == udf01sInfo[index]->handles.hdl_udatn01)
        {
            // received notification from server
            attParam->cmdAccess = BLESERVICE_UDF01S_UDATN01_NOTIFY_EVENT;
            bleUDF01S_PostEvent(attParam, &udf01sCallback[index]);
        }
        break;

    case OPCODE_ATT_HANDLE_VALUE_INDICATION:
        break;

    default:
        break;
    }
}


// handle UDF01S server GATT event
static void handle_UDF01S_server(uint8_t index, BLE_Event_AttParam *attParam)
{
    switch (attParam->cmdAccess)
    {
    case OPCODE_ATT_WRITE_COMMAND:
        if (attParam->hdlNum == udf01sInfo[index]->handles.hdl_udatrw01)
        {
            // received write command from client -> post to user
            attParam->cmdAccess = BLESERVICE_UDF01S_UDATRW01_WRITE_EVENT;
            bleUDF01S_PostEvent(attParam, &udf01sCallback[index]);
        }
        break;

    case OPCODE_ATT_WRITE_REQUEST:
        if (attParam->hdlNum == udf01sInfo[index]->handles.hdl_udatn01_cccd)
        {
            // received write request (cccd value) from client -> update server defined cccd value
            setBLEGATT_HandleCCCDWriteRequest(attParam->data, attParam->length, &udf01sInfo[index]->data.udatn01_cccd);
        }
        else if (attParam->hdlNum == udf01sInfo[index]->handles.hdl_udatrw01)
        {
            // received write request from client -> post to user
            attParam->cmdAccess = BLESERVICE_UDF01S_UDATRW01_WRITE_EVENT;
            bleUDF01S_PostEvent(attParam, &udf01sCallback[index]);
        }
        break;

    case OPCODE_ATT_READ_BY_TYPE_REQUEST:
    case OPCODE_ATT_READ_REQUEST:
        if (attParam->hdlNum == udf01sInfo[index]->handles.hdl_udatn01_cccd)
        {
            // received read or read by type request from client -> send read or read by type response
            setBLEGATT_HandleCCCDGeneralReadRequest(attParam->hostId,
                                                    attParam->cmdAccess,
                                                    udf01sInfo[index]->handles.hdl_udatn01_cccd,
                                                    udf01sInfo[index]->data.udatn01_cccd);
        }
        else if (attParam->hdlNum == udf01sInfo[index]->handles.hdl_udatr01)
        {
            // received read or read by type request from client -> send read or read by type rsp with data back to client
            setBLEGATT_GeneralReadRsp(attParam->hostId, attParam->hdlNum, (uint8_t *)ATTR_VALUE_UDF01S_UDATR01, (sizeof(ATTR_VALUE_UDF01S_UDATR01) - 1));
        }
        else if (attParam->hdlNum == udf01sInfo[index]->handles.hdl_udatrw01)
        {
            // received read or read by type request from client -> post to user to prepare read data back to client
            attParam->cmdAccess = BLESERVICE_UDF01S_UDATRW01_READ_EVENT;
            bleUDF01S_PostEvent(attParam, &udf01sCallback[index]);
        }
        break;

    default:
        break;
    }
}


void ATTR_VALUE_UDF01S_General_Access(BLE_Event_AttParam *attParam)
{
    uint8_t index;

    if (queryIndexByHostIdGattRole(attParam->hostId, attParam->gattRole, MAX_NUM_CONN_UDF01S, udf01sBasicInfo, &index) != BLESTACK_STATUS_SUCCESS)
    {
        // Host id has not registered so there is no callback function -> do nothing
        return;
    }

    if (attParam->gattRole == BLE_GATT_ROLE_CLIENT)
    {
        // handle UDF01S client GATT event
        handle_UDF01S_client(index, attParam);
    }

    if (attParam->gattRole == BLE_GATT_ROLE_SERVER)
    {
        // handle UDF01S server GATT event
        handle_UDF01S_server(index, attParam);
    }
}



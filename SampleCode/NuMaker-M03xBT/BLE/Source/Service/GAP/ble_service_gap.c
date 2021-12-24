/************************************************************************
 *
 * File Name  : BLE_SERVICE_GAP.c
 * Description: This file contains the definitions and functions of BLE GAP Service
 *
 *
 ************************************************************************/
#include "ble_service_gap.h"
#include "ble_att_gatt.h"
#include "ble_profile.h"

/** ATTR_VALUE_GAP_General_Access
 * @note This callback receives the GAP Service events.  \n
 *  Each of these events can be associated with parameters.
 */
void ATTR_VALUE_GAP_General_Access(BLE_Event_AttParam *attParam);

/**************************************************************************
 * GAP(Server) Service Value Definitions
 **************************************************************************/
/** GAP characteristic service Value.
 * @note Return the "Read data" when client send "Read Request".
*/
#define GAP_DEVICE_NAME_DEFAULT             "BLE_DEMO"
#define GAP_DEVICE_NAME_DEFAULT_LEN         (SIZE_STRING(GAP_DEVICE_NAME_DEFAULT))

// Set GAP Server Default Data.
BLEATT_GAP_Data GAP_Data =
{
    GAP_DEVICE_NAME_DEFAULT,          // GAP device name
    GAP_DEVICE_NAME_DEFAULT_LEN,      // GAP device name length
    BLE_APPEARANCE_UNKNOWN,           // GAP appearance
    {
        0x0006,                       // minimum connection interval
        0x0008,                       // maximum connection interval
        0x0000,                       // slave latency
        0x0258,                       // Connection supervision timeout
    },
};


/**************************************************************************
 * GAP Service UUID Definitions
 **************************************************************************/

/** GAP Service UUID.
 * @note 16-bits UUID
 * @note UUID: 1800
*/
const uint16_t ATTR_UUID_GAP_PRIMARY_SERVICE[] =
{
    GATT_SERVICES_GENERIC_ACCESS,
};


/** GAP Characteristic Device Name UUID.
 * @note 16-bits UUID
 * @note UUID: 2A00
*/
const uint16_t ATTR_UUID_GAP_CHARC_DEVICE_NAME[] =
{
    GATT_SPEC_CHARC_DEVICE_NAME,
};


/** GAP Characteristic Appearance UUID.
 * @note 16-bits UUID
 * @note UUID: 2A01
*/
const uint16_t ATTR_UUID_GAP_CHARC_APPEARANCE[] =
{
    GATT_SPEC_CHARC_APPEARANCE,
};


/** GAP Characteristic Peripheral Preferred Connection Parameters UUID.
 * @note 16-bits UUID
 * @note UUID: 2A04
*/
const uint16_t ATTR_UUID_GAP_CHARC_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS[] =
{
    GATT_SPEC_CHARC_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS,
};


/**************************************************************************
 * GAP Service Value Definitions
 **************************************************************************/


/**************************************************************************
 * GAP Service/ Characteristic Definitions
 **************************************************************************/

const ATTRIBUTE_BLE ATT_GAP_PRIMARY_SERVICE =
{
    (void *)ATTR_UUID_TYPE_PRIMARY_SERVICE,
    (void *)ATTR_UUID_GAP_PRIMARY_SERVICE,
    sizeof(ATTR_UUID_GAP_PRIMARY_SERVICE),
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

const ATTRIBUTE_BLE ATT_GAP_CHARACTERISTIC_DEVICE_NAME =
{
    (void *)ATTR_UUID_TYPE_CHARACTERISTIC,
    (void *)ATTR_UUID_GAP_CHARC_DEVICE_NAME,
    sizeof(ATTR_UUID_GAP_CHARC_DEVICE_NAME),
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

const ATTRIBUTE_BLE ATT_GAP_DEVICE_NAME =
{
    (void *)ATTR_UUID_GAP_CHARC_DEVICE_NAME,
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
    ATTR_VALUE_GAP_General_Access,       //registered callback function
};

const ATTRIBUTE_BLE ATT_GAP_CHARACTERISTIC_APPEARANCE =
{
    (void *)ATTR_UUID_TYPE_CHARACTERISTIC,
    (void *)ATTR_UUID_GAP_CHARC_APPEARANCE,
    sizeof(ATTR_UUID_GAP_CHARC_APPEARANCE),
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

const ATTRIBUTE_BLE ATT_GAP_APPEARANCE =
{
    (void *)ATTR_UUID_GAP_CHARC_APPEARANCE,
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
    ATTR_VALUE_GAP_General_Access,       //registered callback function
};

const ATTRIBUTE_BLE ATT_GAP_CHARACTERISTIC_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS =
{
    (void *)ATTR_UUID_TYPE_CHARACTERISTIC,
    (void *)ATTR_UUID_GAP_CHARC_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS,
    sizeof(ATTR_UUID_GAP_CHARC_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS),
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

const ATTRIBUTE_BLE ATT_GAP_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS =
{
    (void *)ATTR_UUID_GAP_CHARC_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS,
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
    ATTR_VALUE_GAP_General_Access,       //registered callback function
};


/**************************************************************************
 * BLE Service << GAP >> Local Variable
 **************************************************************************/
#ifndef MAX_NUM_CONN_GAP
// check MAX_NUM_CONN_GAP if defined or set to default 1.
#define MAX_NUM_CONN_GAP                  1
#endif


// Length of appearance for decoded data to send read response to client.
#define BLE_GAP_APPEARANCE_LEN            2

// Length of preferred peripheral connection parameters for decoded data to send read response to client.
#define BLE_GAP_PREFERRED_CONNPARAM_LEN   8


// Service basic information
Service_Basic_Info                gapBasicInfo[MAX_NUM_CONN_GAP];

// GAP information
BLEATT_GAP_Info                   *gapInfo[MAX_NUM_CONN_GAP];

// GAP callback function
BleGAP_EventCallBack              gapCallback[MAX_NUM_CONN_GAP];

// GAP registered total count
uint8_t gap_count = 0;

// GAP decoded buffer
uint8_t                           gapDecodedBuffer[8];

/**************************************************************************
 * BLE Service << GAP >> Public Function
 **************************************************************************/

/** Set GAP Device Name
 *
 * @param[in] name : a pointer to the device name.
 * @param[in] length : the length of the device name.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus setGAP_DeviceName(uint8_t *name, uint8_t length)
{
    uint8_t i;

    if ((length > GAP_DEVICE_NAME_LENGH) ||
            (length > GAP_DEVICE_NAME_LENGH_MAX))
    {
        return BLESTACK_STATUS_ERR_INVALID_PARAM;
    }

    for (i = 0; i < length; i++)
    {
        GAP_Data.device_name[i] = name[i];
    }
    GAP_Data.device_name_len = length;

    return BLESTACK_STATUS_SUCCESS;
}


/** Set GAP Appearance
 * @param[in] appearance : @ref bleGapAppearance.
*/
void setGAP_Appearance(uint16_t appearance)
{
    GAP_Data.appearance = appearance;
}


/** Set GAP Peripheral Preferred Connection Parameters
 * @param[in] connParam : a pointer to the preferred peripheral connection parameters.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus setGAP_PeripheralConnectionParam(BLE_Conn_Param *connParam)
{
    if ( (connParam->connIntervalMin < CONN_INTERVAL_MIN || connParam->connIntervalMin > CONN_INTERVAL_MAX) ||
            (connParam->connIntervalMax < CONN_INTERVAL_MIN || connParam->connIntervalMax > CONN_INTERVAL_MAX) ||
            (connParam->connIntervalMin > connParam->connIntervalMax) ||
            (connParam->connLatency > CONN_LATENCY_MAX) ||
            (connParam->connSupervisionTimeout < CONN_SUBTIMEOUT_MIN || connParam->connSupervisionTimeout > CONN_SUBTIMEOUT_MAX) ||
            ((connParam->connSupervisionTimeout * 4) < ((1 + connParam->connLatency) * connParam->connIntervalMax )))
    {
        return BLESTACK_STATUS_ERR_INVALID_PARAM;
    }

    GAP_Data.peripheral_conn_param = *connParam;

    return BLESTACK_STATUS_SUCCESS;
}


/** GAP Service Initialization
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
BleStackStatus setGAP_ServiceInit(uint8_t hostId, BleGattRole gattRole, BLEATT_GAP_Info *info, BleGAP_EventCallBack callback)
{
    BleStackStatus status;
    uint8_t config_index;

    if (info == NULL)
    {
        return BLESTACK_STATUS_ERR_INVALID_PARAM;
    }

    // init service client basic information and get "config_index" & "gap_count"
    status = setBLE_ServiceBasicInit(hostId, gattRole, MAX_NUM_CONN_GAP, gapBasicInfo, &config_index, &gap_count);
    BLESTACK_STATUS_CHECK(status);

    // Set service role
    info->role = gattRole;

    // Set GAP client information
    gapInfo[config_index] = info;

    // Register GAP client callback function
    gapCallback[config_index] = callback;

    if (gattRole == BLE_GATT_ROLE_SERVER)
    {
        // get server handles
        status = getGAP_ServiceHandles(hostId, gapInfo[config_index]);
        BLESTACK_STATUS_CHECK(status);
    }

    return BLESTACK_STATUS_SUCCESS;
}


/** Get GAP Service Handle Numbers
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
BleStackStatus getGAP_ServiceHandles(uint8_t hostId, BLEATT_GAP_Info *info)
{
    BleStackStatus status;

    // Get GAP handles
    status = getBLEGATT_HandleNumAddr(hostId, info->role, (ATTRIBUTE_BLE *)&ATT_GAP_PRIMARY_SERVICE, (void *)&info->handles);
    BLESTACK_STATUS_CHECK(status);

    return BLESTACK_STATUS_SUCCESS;
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
BleStackStatus getGAP_ClientDataRead(uint8_t hostId, uint16_t hdlNum)
{
    BleStackStatus status;

    status = setBLEGATT_ReadCharacteristicValue(hostId, hdlNum);

    return status;
}


/**************************************************************************
 * BLE Service << GAP >> General Callback Function
 **************************************************************************/
static void bleGAP_PostEvent(BLE_Event_AttParam *attParam, BleGAP_EventCallBack *callback)
{
    // check callback is null or not
    SERVICE_CALLBACK_NULL_CHECK(*callback);

    // post event to user
    (*callback)(attParam->hostId, attParam->cmdAccess, attParam->data, attParam->length);
}


// handle GAP client GATT event
static void handle_GAP_client(uint8_t index, BLE_Event_AttParam *attParam)
{
    switch (attParam->cmdAccess)
    {
    case OPCODE_ATT_READ_RESPONSE:
    {
        // received read response from server -> post to user
        if (attParam->hdlNum == gapInfo[index]->handles.hdl_device_name)
        {
            attParam->cmdAccess = BLESERVICE_GAP_DEVICE_NAME_READ_EVENT;
            bleGAP_PostEvent(attParam, &gapCallback[index]);
        }
        else if (attParam->hdlNum == gapInfo[index]->handles.hdl_appearance)
        {
            attParam->cmdAccess = BLESERVICE_GAP_APPEARANCE_READ_EVENT;
            bleGAP_PostEvent(attParam, &gapCallback[index]);
        }
        else if (attParam->hdlNum == gapInfo[index]->handles.hdl_peripheral_preferred_connParam)
        {
            attParam->cmdAccess = BLESERVICE_GAP_PERIPHERAL_CONN_PARAM_READ_EVENT;
            bleGAP_PostEvent(attParam, &gapCallback[index]);
        }
    }
    break;

    default:
        break;
    }
}


static void handle_GAP_server(uint8_t index, BLE_Event_AttParam *attParam)
{
    switch (attParam->cmdAccess)
    {
    case OPCODE_ATT_READ_BY_TYPE_REQUEST:
    case OPCODE_ATT_READ_REQUEST:
    {
        // received read request from client -> send read rsp with data back to client
        if (attParam->hdlNum == gapInfo[index]->handles.hdl_device_name)
        {
            setBLEGATT_GeneralReadRsp(attParam->hostId, attParam->hdlNum, GAP_Data.device_name, GAP_Data.device_name_len);
        }
        else if (attParam->hdlNum == gapInfo[index]->handles.hdl_appearance)
        {
            // decoded uint16_t to uint8_t array
            gapDecodedBuffer[0] = (uint8_t)(GAP_Data.appearance & 0xFF);
            gapDecodedBuffer[1] = (uint8_t)((GAP_Data.appearance >> 8) & 0xFF);
            setBLEGATT_GeneralReadRsp(attParam->hostId, attParam->hdlNum, (uint8_t *)gapDecodedBuffer, BLE_GAP_APPEARANCE_LEN);
        }
        else if (attParam->hdlNum == gapInfo[index]->handles.hdl_peripheral_preferred_connParam)
        {
            gapDecodedBuffer[0] = (uint8_t) ((GAP_Data.peripheral_conn_param.connIntervalMin & 0x00FF) >> 0);
            gapDecodedBuffer[1] = (uint8_t) ((GAP_Data.peripheral_conn_param.connIntervalMin & 0xFF00) >> 8);

            gapDecodedBuffer[2] = (uint8_t) ((GAP_Data.peripheral_conn_param.connIntervalMax & 0x00FF) >> 0);
            gapDecodedBuffer[3] = (uint8_t) ((GAP_Data.peripheral_conn_param.connIntervalMax & 0xFF00) >> 8);

            gapDecodedBuffer[4] = (uint8_t) ((GAP_Data.peripheral_conn_param.connLatency & 0x00FF) >> 0);
            gapDecodedBuffer[5] = (uint8_t) ((GAP_Data.peripheral_conn_param.connLatency & 0xFF00) >> 8);

            gapDecodedBuffer[6] = (uint8_t) ((GAP_Data.peripheral_conn_param.connSupervisionTimeout & 0x00FF) >> 0);
            gapDecodedBuffer[7] = (uint8_t) ((GAP_Data.peripheral_conn_param.connSupervisionTimeout & 0xFF00) >> 8);

            // received read or read by type request from client -> send read or read by type rsp with data back to client
            setBLEGATT_GeneralReadRsp(attParam->hostId, attParam->hdlNum, gapDecodedBuffer, BLE_GAP_PREFERRED_CONNPARAM_LEN);
        }
    }
    break;

    case OPCODE_ATT_READ_BLOB_REQUEST:
    {
        if (attParam->hdlNum == gapInfo[index]->handles.hdl_device_name)
        {
            setBLEGATT_HandleReadBlobRequest(attParam, GAP_Data.device_name, GAP_Data.device_name_len);
        }
    }
    break;

    default:
        break;
    }
}

void ATTR_VALUE_GAP_General_Access(BLE_Event_AttParam *attParam)
{
    uint8_t index;

    if (queryIndexByHostIdGattRole(attParam->hostId, attParam->gattRole, MAX_NUM_CONN_GAP, gapBasicInfo, &index) != BLESTACK_STATUS_SUCCESS)
    {
        // Host id has not registered so there is no callback function -> do nothing
        return;
    }

    if (attParam->gattRole == BLE_GATT_ROLE_CLIENT)
    {
        // handle GAP client GATT event
        handle_GAP_client(index, attParam);
    }

    if (attParam->gattRole == BLE_GATT_ROLE_SERVER)
    {
        // handle GAP server GATT event
        handle_GAP_server(index, attParam);
    }
}


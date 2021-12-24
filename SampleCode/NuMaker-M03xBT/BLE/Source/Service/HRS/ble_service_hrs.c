/************************************************************************
 *
 * File Name  : BLE_SERVICE_HRS.c
 * Description: This file contains the definitions and functions of BLE HRS
 *
 *
 ************************************************************************/
#include "ble_service_hrs.h"
#include "ble_profile.h"

/** ATTR_VALUE_HRS_General_Access
 * @note This callback receives the HRS Service events.  \n
 *  Each of these events can be associated with parameters.
 */
void ATTR_VALUE_HRS_General_Access(BLE_Event_AttParam *attParam);

/**************************************************************************
 * HRS Service UUID Definitions
 **************************************************************************/

/** HRS Service UUID.
 * @note 16-bits UUID
 * @note UUID: 180D
*/
const uint16_t ATTR_UUID_HRS_PRIMARY_SERVICE[] =
{
    GATT_SERVICES_HEART_RATE,
};

/** Heart Rate measurement characteristic UUID.
 * @note 16-bits UUID
 * @note UUID: 2A37
*/
const uint16_t ATTR_UUID_HRS_CHARC_HEART_RATE_MEASUREMENT[] =
{
    GATT_SPEC_CHARC_HEART_RATE_MEASUREMENT,
};

/** Body sensor location characteristic UUID.
 * @note 16-bits UUID
 * @note UUID: 2A38
*/
const uint16_t ATTR_UUID_HRS_CHARC_BODY_SENSOR_LOCATION[] =
{
    GATT_SPEC_CHARC_BODY_SENSOR_LOCATION,
};


/**************************************************************************
 * HRS Service Value Definitions
 **************************************************************************/


/**************************************************************************
 * HRS Service/ Characteristic Definitions
 **************************************************************************/

const ATTRIBUTE_BLE ATT_HRS_PRIMARY_SERVICE =
{
    (void *)ATTR_UUID_TYPE_PRIMARY_SERVICE,
    (void *)ATTR_UUID_HRS_PRIMARY_SERVICE,
    sizeof(ATTR_UUID_HRS_PRIMARY_SERVICE),
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

const ATTRIBUTE_BLE ATT_HRS_CHARACTERISTIC_HEART_RATE_MEASUREMENT =
{
    (void *)ATTR_UUID_TYPE_CHARACTERISTIC,
    (void *)ATTR_UUID_HRS_CHARC_HEART_RATE_MEASUREMENT,
    sizeof(ATTR_UUID_HRS_CHARC_HEART_RATE_MEASUREMENT),
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

const ATTRIBUTE_BLE ATT_HRS_HEART_RATE_MEASUREMENT =
{
    (void *)ATTR_UUID_HRS_CHARC_HEART_RATE_MEASUREMENT,
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
    ATTR_VALUE_HRS_General_Access,    //ATTR_VALUE_HERAT_RATE_General_Access,
};

const ATTRIBUTE_BLE ATT_HRS_MEASUREMENT_NOTIFY_CLIENT_CHARC_CONFIGURATION =
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
    ATTR_VALUE_HRS_General_Access,       //registered callback function
};

const ATTRIBUTE_BLE ATT_HRS_CHARACTERISTIC_BODY_SENSOR_LOCATION =
{
    (void *)ATTR_UUID_TYPE_CHARACTERISTIC,
    (void *)ATTR_UUID_HRS_CHARC_BODY_SENSOR_LOCATION,
    sizeof(ATTR_UUID_HRS_CHARC_BODY_SENSOR_LOCATION),
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

const ATTRIBUTE_BLE ATT_HRS_BODY_SENSOR_LOCATION =
{
    (void *)ATTR_UUID_HRS_CHARC_BODY_SENSOR_LOCATION,
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
    ATTR_VALUE_HRS_General_Access,       //registered callback function
};


/**************************************************************************
 * BLE Service << HRS >> Local Variable
 **************************************************************************/
#ifndef MAX_NUM_CONN_HRS
// check MAX_NUM_CONN_HRS if defined or set to default 1.
#define MAX_NUM_CONN_HRS       1
#endif


// Service basic information
Service_Basic_Info     hrsBasicInfo[MAX_NUM_CONN_HRS];

// HRS information
BLEATT_HRS_Info        *hrsInfo[MAX_NUM_CONN_HRS];

// HRS callback function
BleHRS_EventCallBack   hrsCallback[MAX_NUM_CONN_HRS];

// HRS registered total count
uint8_t hrs_count = 0;


/**************************************************************************
 * BLE Service << HRS >> Public Function
 **************************************************************************/

/** HRS Initialization
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
BleStackStatus setHRS_ServiceInit(uint8_t hostId, BleGattRole gattRole, BLEATT_HRS_Info *info, BleHRS_EventCallBack callback)
{
    BleStackStatus status;
    uint8_t config_index;

    if (info == NULL)
    {
        return BLESTACK_STATUS_ERR_INVALID_PARAM;
    }

    // init service basic information and get "config_index" & "hrs_count"
    status = setBLE_ServiceBasicInit(hostId, gattRole, MAX_NUM_CONN_HRS, hrsBasicInfo, &config_index, &hrs_count);
    BLESTACK_STATUS_CHECK(status);

    // Set service role
    info->role = gattRole;

    // Set HRS data
    hrsInfo[config_index] = info;

    // Register HRS callback function
    hrsCallback[config_index] = callback;

    // Get handles at initialization if role is set to BLE_GATT_ROLE_SERVER
    if (gattRole == BLE_GATT_ROLE_SERVER)
    {
        status = getHRS_ServiceHandles(hostId, hrsInfo[config_index]);
        BLESTACK_STATUS_CHECK(status);
    }

    return BLESTACK_STATUS_SUCCESS;
}


/** Get HRS Handle Numbers
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
BleStackStatus getHRS_ServiceHandles(uint8_t hostId, BLEATT_HRS_Info *info)
{
    BleStackStatus status;

    // Get HRS handles
    status = getBLEGATT_HandleNumAddr(hostId, info->role, (ATTRIBUTE_BLE *)&ATT_HRS_PRIMARY_SERVICE, (void *)&info->handles);
    BLESTACK_STATUS_CHECK(status);

    return BLESTACK_STATUS_SUCCESS;
}


/**************************************************************************
 * BLE Service << HRS >> General Callback Function
 **************************************************************************/
static void bleHRS_PostEvent(BLE_Event_AttParam *attParam, BleHRS_EventCallBack *callback)
{
    // check callback is null or not
    SERVICE_CALLBACK_NULL_CHECK(*callback);

    // post event to user
    (*callback)(attParam->hostId, attParam->cmdAccess, attParam->data, attParam->length);
}

// handle HRS client GATT event
static void handle_HRS_client(uint8_t index, BLE_Event_AttParam *attParam)
{
    switch (attParam->cmdAccess)
    {
    case OPCODE_ATT_WRITE_RESPONSE:
        if (attParam->hdlNum == hrsInfo[index]->handles.hdl_heart_rate_measurement_cccd)
        {
            // received write response from server -> cccd configure completed
            attParam->cmdAccess = BLESERVICE_HRS_MEASUREMENT_CCCD_WRITE_RSP_EVENT;
            bleHRS_PostEvent(attParam, &hrsCallback[index]);
        }
        break;

    case OPCODE_ATT_READ_RESPONSE:
        if (attParam->hdlNum == hrsInfo[index]->handles.hdl_heart_rate_measurement_cccd)
        {
            // received read response (cccd value) from server
            attParam->cmdAccess = BLESERVICE_HRS_MEASUREMENT_CCCD_READ_RSP_EVENT;
            bleHRS_PostEvent(attParam, &hrsCallback[index]);
        }
        break;

    case OPCODE_ATT_HANDLE_VALUE_NOTIFICATION:
        if (attParam->hdlNum == hrsInfo[index]->handles.hdl_heart_rate_measurement)
        {
            // received notification from server
            attParam->cmdAccess = BLESERVICE_HRS_MEASUREMENT_NOTIFY_EVENT;
            bleHRS_PostEvent(attParam, &hrsCallback[index]);
        }
        break;

    default:
        break;
    }
}

// handle HRS server GATT event
static void handle_HRS_server(uint8_t index, BLE_Event_AttParam *attParam)
{
    switch (attParam->cmdAccess)
    {
    case OPCODE_ATT_WRITE_REQUEST:
        if (attParam->hdlNum == hrsInfo[index]->handles.hdl_heart_rate_measurement_cccd)
        {
            // update server defined cccd value
            setBLEGATT_HandleCCCDWriteRequest(attParam->data, attParam->length, &hrsInfo[index]->data.heart_rate_measurement_cccd);
        }
        break;

    case OPCODE_ATT_READ_BY_TYPE_REQUEST:
    case OPCODE_ATT_READ_REQUEST:
        if (attParam->hdlNum == hrsInfo[index]->handles.hdl_heart_rate_measurement_cccd)
        {
            // received read or read by type request from client -> send read or read by type response
            setBLEGATT_HandleCCCDGeneralReadRequest(attParam->hostId,
                                                    attParam->cmdAccess,
                                                    hrsInfo[index]->handles.hdl_heart_rate_measurement_cccd,
                                                    hrsInfo[index]->data.heart_rate_measurement_cccd);
        }
        else if (attParam->hdlNum == hrsInfo[index]->handles.hdl_body_sensor_location)
        {
            // received read or read by type request from client -> send read or read by type rsp with data back to client
            setBLEGATT_GeneralReadRsp(attParam->hostId,
                                      hrsInfo[index]->handles.hdl_body_sensor_location,
                                      (uint8_t *)&hrsInfo[index]->data.body_sensor_location,
                                      1);
        }
        break;

    default:
        break;
    }
}


void ATTR_VALUE_HRS_General_Access(BLE_Event_AttParam *attParam)
{
    uint8_t index;

    if (queryIndexByHostIdGattRole(attParam->hostId, attParam->gattRole, MAX_NUM_CONN_HRS, hrsBasicInfo, &index) != BLESTACK_STATUS_SUCCESS)
    {
        // Host id has not registered so there is no callback function -> do nothing
        return;
    }

    if (attParam->gattRole == BLE_GATT_ROLE_CLIENT)
    {
        // handle HRS client GATT event
        handle_HRS_client(index, attParam);
    }

    if (attParam->gattRole == BLE_GATT_ROLE_SERVER)
    {
        // handle HRS server GATT event
        handle_HRS_server(index, attParam);
    }
}

/************************************************************************
 *
 * File Name  : BLE_SERVICE_BAS.c
 * Description: This file contains the definitions and functions of BLE BAS
 *
 *
 ************************************************************************/
#include "ble_service_bas.h"
#include "ble_profile.h"


/** ATTR_VALUE_BAS_General_Access
 * @note This callback receives the BAS Service events.  \n
 *  Each of these events can be associated with parameters.
 */
void ATTR_VALUE_BAS_General_Access(BLE_Event_AttParam *attParam);


/**************************************************************************
 * BAS Service UUID Definitions
 **************************************************************************/

/** BAS Service UUID.
 * @note 16-bits UUID
 * @note UUID: 180F
*/
const uint16_t ATTR_UUID_BAS_PRIMARY_SERVICE[] =
{
    GATT_SERVICES_BATTERY_SERVICE,
};

/** BAS characteristic BATTERY_LEVEL UUID.
 * @note 16-bits UUID
 * @note UUID: 2A19
*/
const uint16_t ATTR_UUID_BAS_CHARC_BATTERY_LEVEL[] =
{
    GATT_SPEC_CHARC_BATTERY_LEVEL,
};


/**************************************************************************
 * BAS Service Value Definitions
 **************************************************************************/


/**************************************************************************
 * BAS Service/ Characteristic Definitions
 **************************************************************************/

const ATTRIBUTE_BLE ATT_BAS_PRIMARY_SERVICE =
{
    (void *)ATTR_UUID_TYPE_PRIMARY_SERVICE,
    (void *)ATTR_UUID_BAS_PRIMARY_SERVICE,
    sizeof(ATTR_UUID_BAS_PRIMARY_SERVICE),
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

const ATTRIBUTE_BLE ATT_BAS_CHARACTERISTIC_BATTERY_LEVEL =
{
    (void *)ATTR_UUID_TYPE_CHARACTERISTIC,
    (void *)ATTR_UUID_BAS_CHARC_BATTERY_LEVEL,
    sizeof(ATTR_UUID_BAS_CHARC_BATTERY_LEVEL),
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

const ATTRIBUTE_BLE ATT_BAS_BATTERY_LEVEL =
{
    (void *)ATTR_UUID_BAS_CHARC_BATTERY_LEVEL,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
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
    ATTR_VALUE_BAS_General_Access,       //registered callback function
};

const ATTRIBUTE_BLE ATT_BAS_BATTERY_LEVEL_CLIENT_CHARC_CONFIGURATION =
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
    ATTR_VALUE_BAS_General_Access,       //registered callback function
};

/**************************************************************************
 * BLE Service << BAS >> Local Variable
 **************************************************************************/
#ifndef MAX_NUM_CONN_BAS
// check MAX_NUM_CONN_BAS if defined or set to default 1.
#define MAX_NUM_CONN_BAS       1
#endif

// Service basic information
Service_Basic_Info     basBasicInfo[MAX_NUM_CONN_BAS];

// BAS information
BLEATT_BAS_Info        *basInfo[MAX_NUM_CONN_BAS];

// BAS callback function
BleBAS_EventCallBack   basCallback[MAX_NUM_CONN_BAS];

// BAS registered total count
uint8_t bas_count = 0;


/**************************************************************************
 * BLE Service << BAS >> Public Function
 **************************************************************************/

/** BAS Initialization
 *
 * @param[in] hostId : the link's host id.
 * @param[in] gattRole : @ref BleGattRole "BLE GATT role".
 * @param[in] info : a pointer to BAS information.
 * @param[in] callback : a pointer to a callback function that receive the service events.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED : Registered services buffer full.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus setBAS_ServiceInit(uint8_t hostId, BleGattRole gattRole, BLEATT_BAS_Info *info, BleBAS_EventCallBack callback)
{
    BleStackStatus status;
    uint8_t config_index;

    if (info == NULL)
    {
        return BLESTACK_STATUS_ERR_INVALID_PARAM;
    }

    // init service basic information and get "config_index" & "bas_count"
    status = setBLE_ServiceBasicInit(hostId, gattRole, MAX_NUM_CONN_BAS, basBasicInfo, &config_index, &bas_count);
    BLESTACK_STATUS_CHECK(status);

    // Set service role
    info->role = gattRole;

    // Set BAS data
    basInfo[config_index] = info;

    // Register BAS callback function
    basCallback[config_index] = callback;

    // Get handles at initialization if role is set to BLE_GATT_ROLE_SERVER
    if (gattRole == BLE_GATT_ROLE_SERVER)
    {
        status = getBAS_ServiceHandles(hostId, basInfo[config_index]);
        BLESTACK_STATUS_CHECK(status);
    }

    return BLESTACK_STATUS_SUCCESS;
}


/** Get BAS Handle Numbers
 *
 * @attention
 *            - role = <b> @ref BLE_GATT_ROLE_CLIENT </b> \n
 *              MUST call this API to get service information after received @ref BLECMD_EVENT_ATT_DATABASE_PARSING_FINISHED  \n
 *            - role = <b> @ref BLE_GATT_ROLE_SERVER </b> \n
 *              MUST call this API to get service information before connection established. \n
 *
 * @param[in] hostId : the link's host id.
 * @param[out] info : a pointer to BAS information
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus getBAS_ServiceHandles(uint8_t hostId, BLEATT_BAS_Info *info)
{
    BleStackStatus status;

    // Get BAS handles
    status = getBLEGATT_HandleNumAddr(hostId, info->role, (ATTRIBUTE_BLE *)&ATT_BAS_PRIMARY_SERVICE, (void *)&info->handles);
    BLESTACK_STATUS_CHECK(status);

    return BLESTACK_STATUS_SUCCESS;
}

/**************************************************************************
 * BLE Service << BAS >> General Callback Function
 **************************************************************************/
static void bleBAS_PostEvent(BLE_Event_AttParam *attParam, BleBAS_EventCallBack *callback)
{
    // check callback is null or not
    SERVICE_CALLBACK_NULL_CHECK(*callback);

    // post event to user
    (*callback)(attParam->hostId, attParam->cmdAccess, attParam->data, attParam->length);
}

// handle UDF01S client GATT event
static void handle_BAS_client(uint8_t index, BLE_Event_AttParam *attParam)
{
    switch (attParam->cmdAccess)
    {
    case OPCODE_ATT_WRITE_RESPONSE:
    case OPCODE_ATT_RESTORE_BOND_DATA_COMMAND:
        if (attParam->hdlNum == basInfo[index]->handles.hdl_battery_level_cccd)
        {
            // received write response from server -> cccd configure completed
            attParam->cmdAccess = BLESERVICE_BAS_LEVEL_CCCD_WRITE_RSP_EVENT;
            bleBAS_PostEvent(attParam, &basCallback[index]);
        }
        break;

    case OPCODE_ATT_READ_RESPONSE:
        if (attParam->hdlNum == basInfo[index]->handles.hdl_battery_level_cccd)
        {
            // received write response from server -> cccd configure completed
            attParam->cmdAccess = BLESERVICE_BAS_LEVEL_CCCD_READ_EVENT;
            bleBAS_PostEvent(attParam, &basCallback[index]);
        }
        else if (attParam->hdlNum == basInfo[index]->handles.hdl_battery_level)
        {
            // received write response from server -> cccd configure completed
            attParam->cmdAccess = BLESERVICE_BAS_LEVEL_READ_EVENT;
            bleBAS_PostEvent(attParam, &basCallback[index]);
        }
        break;

    case OPCODE_ATT_HANDLE_VALUE_NOTIFICATION:
        // received notification from server -> post to user
        attParam->cmdAccess = BLESERVICE_BAS_LEVEL_NOTIFY_EVENT;
        bleBAS_PostEvent(attParam, &basCallback[index]);
        break;

    default:
        break;
    }
}

// handle UDF01S server GATT event
static void handle_BAS_server(uint8_t index, BLE_Event_AttParam *attParam)
{
    switch (attParam->cmdAccess)
    {
    case OPCODE_ATT_READ_BY_TYPE_REQUEST:
    case OPCODE_ATT_READ_REQUEST:
        if (attParam->hdlNum == basInfo[index]->handles.hdl_battery_level)
        {
            // received read or read by type request from client -> post to user
            attParam->cmdAccess = BLESERVICE_BAS_LEVEL_READ_EVENT;
            bleBAS_PostEvent(attParam, &basCallback[index]);
        }
        else if (attParam->hdlNum == basInfo[index]->handles.hdl_battery_level_cccd)
        {
            // received read or read by type request from client -> send read or read by type response
            setBLEGATT_HandleCCCDGeneralReadRequest(attParam->hostId,
                                                    attParam->cmdAccess,
                                                    basInfo[index]->handles.hdl_battery_level_cccd,
                                                    basInfo[index]->data.battery_level_cccd);
        }
        break;

    case OPCODE_ATT_WRITE_REQUEST:
    case OPCODE_ATT_RESTORE_BOND_DATA_COMMAND:
        if (attParam->hdlNum == basInfo[index]->handles.hdl_battery_level_cccd)
        {
            // update server defined cccd value
            setBLEGATT_HandleCCCDWriteRequest(attParam->data, attParam->length, &basInfo[index]->data.battery_level_cccd);
        }
        break;

    default:
        break;
    }
}

void ATTR_VALUE_BAS_General_Access(BLE_Event_AttParam *attParam)
{
    uint8_t index;

    if (queryIndexByHostIdGattRole(attParam->hostId, attParam->gattRole, MAX_NUM_CONN_BAS, basBasicInfo, &index) != BLESTACK_STATUS_SUCCESS)
    {
        // Host id has not registered so there is no callback function -> do nothing
        return;
    }
    if (attParam->gattRole == BLE_GATT_ROLE_CLIENT)
    {
        // handle BAS client GATT event
        handle_BAS_client(index, attParam);
    }

    if (attParam->gattRole == BLE_GATT_ROLE_SERVER)
    {
        // handle BAS server GATT event
        handle_BAS_server(index, attParam);
    }
}


/************************************************************************
 *
 * File Name  : BLE_SERVICE_GATT.c
 * Description: This file contains the definitions and functions of BLE GATT Service
 *
 *
 ************************************************************************/
#include "ble_service_gatt.h"
#include "ble_att_gatt.h"
#include "ble_profile.h"

/** ATTR_VALUE_GATT_General_Access
 * @note This callback receives the GATT Service events.  \n
 *  Each of these events can be associated with parameters.
 */
void ATTR_VALUE_GATT_General_Access(BLE_Event_AttParam *attParam);

/**************************************************************************
 * GATT Service UUID Definitions
 **************************************************************************/

/** GATT Service UUID.
 * @note 16-bits UUID
 * @note UUID: 1801
*/
const uint16_t ATTR_UUID_GATT_PRIMARY_SERVICE[] =
{
    GATT_SERVICES_GENERIC_ATTRIBUTE,
};


/** GATT Characteristic Service Changed UUID.
 * @note 16-bits UUID
 * @note UUID: 2A05
*/
const uint16_t ATTR_UUID_GATT_CHARC_SERVICE_CHANGED[] =
{
    GATT_SPEC_CHARC_SERVICE_CHANGED,
};


/**************************************************************************
 * GATT Service Value Definitions
 **************************************************************************/

/** GATT characteristic Service Changed Value which indicates the range of attribute handles.
*/
const uint16_t ATTR_VALUE_GATT_SERVICE_CHANGED[] =
{
    0x0001,                                         //Start of Affected Attribute Handle Range
    0xFFFF,                                         //End of Affected Attribute Handle Range
};

/**************************************************************************
 * GATT Service/ Characteristic Definitions
 **************************************************************************/

const ATTRIBUTE_BLE ATT_GATT_PRIMARY_SERVICE =
{
    (void *)ATTR_UUID_TYPE_PRIMARY_SERVICE,
    (void *)ATTR_UUID_GATT_PRIMARY_SERVICE,
    sizeof(ATTR_UUID_GATT_PRIMARY_SERVICE),
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

const ATTRIBUTE_BLE ATT_GATT_CHARACTERISTIC_SERVICE_CHANGED =
{
    (void *)ATTR_UUID_TYPE_CHARACTERISTIC,
    (void *)ATTR_UUID_GATT_CHARC_SERVICE_CHANGED,
    sizeof(ATTR_UUID_GATT_CHARC_SERVICE_CHANGED),
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

const ATTRIBUTE_BLE ATT_GATT_SERVICE_CHANGED =
{
    (void *)ATTR_UUID_GATT_CHARC_SERVICE_CHANGED,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        //GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        GATT_DECLARATIONS_PROPERTIES_INDICATE |
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
    ATTR_VALUE_GATT_General_Access,       //registered callback function
};


const ATTRIBUTE_BLE ATT_GATT_CLIENT_CHARC_CONFIGURATION_SERVICE_CHANGED =
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
    ATTR_VALUE_GATT_General_Access,     //registered callback function
};


/**************************************************************************
 * BLE Service << GATT >> Local Variable
 **************************************************************************/
#ifndef MAX_NUM_CONN_GATT
// check MAX_NUM_CONN_GATT if defined or set to default 1.
#define MAX_NUM_CONN_GATT       1
#endif


// Service basic information
Service_Basic_Info      gattBasicInfo[MAX_NUM_CONN_GATT];

// GATT information
BLEATT_GATT_Info        *gattInfo[MAX_NUM_CONN_GATT];

// GATT callback function
BleGATT_EventCallBack   gattCallback[MAX_NUM_CONN_GATT];

// GATT registered total count
uint8_t gatt_count = 0;


/**************************************************************************
 * BLE Service << GATT >> Public Function
 **************************************************************************/
/** GATT Service Initialization
 *
 * @param[in] hostId : the link's host id.
 * @param[in] gattRole : @ref BleGattRole "BLE GATT role".
 * @param[in] info : a pointer to GATT information.
 * @param[in] callback : a pointer to a callback function that receive the service events.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED : Registered services buffer full.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus setGATT_ServiceInit(uint8_t hostId, BleGattRole gattRole, BLEATT_GATT_Info *info, BleGATT_EventCallBack callback)
{
    BleStackStatus status;
    uint8_t config_index;

    if (info == NULL)
    {
        return BLESTACK_STATUS_ERR_INVALID_PARAM;
    }

    // init service basic information and get "config_index" & "gatt_count"
    status = setBLE_ServiceBasicInit(hostId, gattRole, MAX_NUM_CONN_GATT, gattBasicInfo, &config_index, &gatt_count);
    BLESTACK_STATUS_CHECK(status);

    // Set service role
    info->role = gattRole;

    // Set GATT data
    gattInfo[config_index] = info;

    // Register GATT callback function
    gattCallback[config_index] = callback;

    // Get handles at initialization if role is set to BLE_GATT_ROLE_SERVER
    if (gattRole == BLE_GATT_ROLE_SERVER)
    {
        status = getGATT_ServiceHandles(hostId, gattInfo[config_index]);
        BLESTACK_STATUS_CHECK(status);
    }

    return BLESTACK_STATUS_SUCCESS;
}



/** Get GATT Service Handle Numbers
 *
 * @attention
 *            - role = <b> @ref BLE_GATT_ROLE_CLIENT </b> \n
 *              MUST call this API to get service information after received @ref BLECMD_EVENT_ATT_DATABASE_PARSING_FINISHED  \n
 *            - role = <b> @ref BLE_GATT_ROLE_SERVER </b> \n
 *              MUST call this API to get service information before connection established. \n
 *
 * @param[in] hostId : the link's host id.
 * @param[out] info : a pointer to GATT information
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus getGATT_ServiceHandles(uint8_t hostId, BLEATT_GATT_Info *info)
{
    BleStackStatus status;

    // Get GATT handles
    status = getBLEGATT_HandleNumAddr(hostId, info->role, (ATTRIBUTE_BLE *)&ATT_GATT_PRIMARY_SERVICE, (void *)&info->handles);
    BLESTACK_STATUS_CHECK(status);

    return BLESTACK_STATUS_SUCCESS;
}


/**************************************************************************
 * BLE Service << GATT >> General Callback Function
 **************************************************************************/
static void bleGATT_PostEvent(BLE_Event_AttParam *attParam, BleGATT_EventCallBack *callback)
{
    // check callback is null or not
    SERVICE_CALLBACK_NULL_CHECK(*callback);

    // post event to user
    (*callback)(attParam->hostId, attParam->cmdAccess, attParam->data, attParam->length);
}

// handle GATT client GATT event
static void handle_GATT_client(uint8_t index, BLE_Event_AttParam *attParam)
{
    switch (attParam->cmdAccess)
    {
    case OPCODE_ATT_WRITE_RESPONSE:
        if (attParam->hdlNum == gattInfo[index]->handles.hdl_service_changed_cccd)
        {
            // received write response from server -> cccd configure completed
            attParam->cmdAccess = BLESERVICE_GATT_SERVICE_CHANGED_CCCD_WRITE_RSP_EVENT;
            bleGATT_PostEvent(attParam, &gattCallback[index]);
        }
        break;

    case OPCODE_ATT_READ_RESPONSE:
        if (attParam->hdlNum == gattInfo[index]->handles.hdl_service_changed_cccd)
        {
            // received read response (cccd value) from server
            attParam->cmdAccess = BLESERVICE_GATT_SERVICE_CHANGED_CCCD_READ_EVENT;
            bleGATT_PostEvent(attParam, &gattCallback[index]);
        }
        break;

    case OPCODE_ATT_HANDLE_VALUE_INDICATION:
        // received indication from server
        if (attParam->hdlNum == gattInfo[index]->handles.hdl_service_changed)
        {
            // re-do service parsing here
            setBLEGATT_ReparseAttDatabase(attParam->hostId);
            return;
        }
        break;

    default:
        break;
    }
}


// handle GATT server GATT event
static void handle_GATT_server(uint8_t index, BLE_Event_AttParam *attParam)
{
    switch (attParam->cmdAccess)
    {
    case OPCODE_ATT_WRITE_REQUEST:
        if (attParam->hdlNum == gattInfo[index]->handles.hdl_service_changed_cccd)
        {
            // received write request (cccd value) from client -> update server defined cccd value
            setBLEGATT_HandleCCCDWriteRequest(attParam->data, attParam->length, &gattInfo[index]->data.service_changed_cccd);
        }
        break;

    case OPCODE_ATT_READ_BY_TYPE_REQUEST:
    case OPCODE_ATT_READ_REQUEST:
        if (attParam->hdlNum == gattInfo[index]->handles.hdl_service_changed_cccd)
        {
            // received read or read by type request from client -> send read or read by type response
            setBLEGATT_HandleCCCDGeneralReadRequest(attParam->hostId,
                                                    attParam->cmdAccess,
                                                    gattInfo[index]->handles.hdl_service_changed_cccd,
                                                    gattInfo[index]->data.service_changed_cccd);
        }
        break;

    default:
        break;
    }
}

void ATTR_VALUE_GATT_General_Access(BLE_Event_AttParam *attParam)
{
    uint8_t index;

    if (queryIndexByHostIdGattRole(attParam->hostId, attParam->gattRole, MAX_NUM_CONN_GATT, gattBasicInfo, &index) != BLESTACK_STATUS_SUCCESS)
    {
        // Host id has not registered so there is no callback function -> do nothing
        return;
    }

    if (attParam->gattRole == BLE_GATT_ROLE_CLIENT)
    {
        // handle GATT client GATT event
        handle_GATT_client(index, attParam);
    }

    if (attParam->gattRole == BLE_GATT_ROLE_SERVER)
    {
        // handle GATT server GATT event
        handle_GATT_server(index, attParam);
    }
}

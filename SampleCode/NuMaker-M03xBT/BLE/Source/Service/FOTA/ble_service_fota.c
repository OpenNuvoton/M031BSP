/************************************************************************
 *
 * File Name  : BLE_SERVICE_FOTA.c
 * Description: This file contains the definitions and functions of BLE FOTA
 *
 *
 ************************************************************************/
#include "ble_service_fota.h"
#include "ble_profile.h"

/** ATTR_VALUE_FOTA_General_Access
 * @note This callback receives the FOTA Service events.  \n
 *  Each of these events can be associated with parameters.
 */
void ATTR_VALUE_FOTA_General_Access(BLE_Event_AttParam *attParam);

/**************************************************************************
 * FOTA Service UUID Definitions
 **************************************************************************/

/** FOTA Service UUID.
 * @note 128-bits UUID
 * @note UUID: 09102132435465768798a9bacbdcedfe
*/
const uint16_t ATTR_UUID_FOTA_PRIMARY_SERVICE[] =
{
    0xedfe, 0xcbdc,
    0xa9ba, 0x8798,
    0x6576, 0x4354,
    0x2132, 0x0910,
};

/** FOTA characteristic DATA UUID.
 * @note 128-bits UUID
 * @note UUID: 01112131415161718191a1b1c1d1e1f1
*/
const uint16_t ATTR_UUID_FOTA_CHARC_DATA[] =
{
    0xe1f1, 0xc1d1,
    0xa1b1, 0x8191,
    0x6171, 0x4151,
    0x2131, 0x0111,
};

/** FOTA characteristic COMMAND UUID.
 * @note 128-bits UUID
 * @note UUID: 02122232425262728292a2b2c2d2e2f2
*/
const uint16_t ATTR_UUID_FOTA_CHARC_COMMAND[] =
{
    0xe2f2, 0xc2d2,
    0xa2b2, 0x8292,
    0x6272, 0x4252,
    0x2232, 0x0212,
};


/**************************************************************************
 * FOTA Service Value Definitions
 **************************************************************************/

/** DATA User Description Definition.
 * @note Return the "description value" when central send "Read Request".
*/
#define ATTR_VALUE_FOTA_DATA_USER_DESCRIPTION     "FOTA_DATA"

/** COMMAND User Description Definition.
 * @note Return the "description value" when central send "Read Request".
*/
#define ATTR_VALUE_FOTA_COMMAND_USER_DESCRIPTION  "FOTA_CMD"


/**************************************************************************
 * FOTA Service/ Characteristic Definitions
 **************************************************************************/

const ATTRIBUTE_BLE ATT_FOTA_PRIMARY_SERVICE =
{
    (void *)ATTR_UUID_TYPE_PRIMARY_SERVICE,
    (void *)ATTR_UUID_FOTA_PRIMARY_SERVICE,
    sizeof(ATTR_UUID_FOTA_PRIMARY_SERVICE),
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

const ATTRIBUTE_BLE ATT_FOTA_CHARACTERISTIC_DATA =
{
    (void *)ATTR_UUID_TYPE_CHARACTERISTIC,
    (void *)ATTR_UUID_FOTA_CHARC_DATA,
    sizeof(ATTR_UUID_FOTA_CHARC_DATA),
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

const ATTRIBUTE_BLE ATT_FOTA_DATA =
{
    (void *)ATTR_UUID_FOTA_CHARC_DATA,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        //GATT_DECLARATIONS_PROPERTIES_READ |
        GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
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
    ATTR_VALUE_FOTA_General_Access,       //registered callback function
};

const ATTRIBUTE_BLE ATT_FOTA_DATA_CLIENT_CHARC_CONFIGURATION =
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
    ATTR_VALUE_FOTA_General_Access,       //registered callback function
};

const ATTRIBUTE_BLE ATT_FOTA_DATA_USER_DESCRIPTION =
{
    (void *)ATTR_UUID_TYPE_CHARC_USER_DESCRIPTION,
    (void *)ATTR_VALUE_FOTA_DATA_USER_DESCRIPTION,
    sizeof(ATTR_VALUE_FOTA_DATA_USER_DESCRIPTION),
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
    ATTR_VALUE_FOTA_General_Access,       //registered callback function
};

const ATTRIBUTE_BLE ATT_FOTA_CHARACTERISTIC_COMMAND =
{
    (void *)ATTR_UUID_TYPE_CHARACTERISTIC,
    (void *)ATTR_UUID_FOTA_CHARC_COMMAND,
    sizeof(ATTR_UUID_FOTA_CHARC_COMMAND),
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

const ATTRIBUTE_BLE ATT_FOTA_COMMAND =
{
    (void *)ATTR_UUID_FOTA_CHARC_COMMAND,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        GATT_DECLARATIONS_PROPERTIES_INDICATE |
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
    ATTR_VALUE_FOTA_General_Access,       //registered callback function
};

const ATTRIBUTE_BLE ATT_FOTA_COMMAND_CLIENT_CHARC_CONFIGURATION =
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
    ATTR_VALUE_FOTA_General_Access,       //registered callback function
};

const ATTRIBUTE_BLE ATT_FOTA_COMMAND_USER_DESCRIPTION =
{
    (void *)ATTR_UUID_TYPE_CHARC_USER_DESCRIPTION,
    (void *)ATTR_VALUE_FOTA_COMMAND_USER_DESCRIPTION,
    sizeof(ATTR_VALUE_FOTA_COMMAND_USER_DESCRIPTION),
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
    ATTR_VALUE_FOTA_General_Access,       //registered callback function
};


/**************************************************************************
 * BLE Service << FOTA >> Local Variable
 **************************************************************************/
#ifndef MAX_NUM_CONN_FOTA
// check MAX_NUM_CONN_FOTA if defined or set to default 1.
#define MAX_NUM_CONN_FOTA       1
#endif


// Service basic information
Service_Basic_Info              fotaBasicInfo[MAX_NUM_CONN_FOTA];

// FOTA information
BLEATT_FOTA_Info                *fotaInfo[MAX_NUM_CONN_FOTA];

// FOTA callback function
BleFOTA_EventCallBack           fotaCallback[MAX_NUM_CONN_FOTA];

// FOTA registered total count
uint8_t fota_count = 0;

/**************************************************************************
 * BLE Service << FOTA >> Public Function
 **************************************************************************/
/** FOTA Service Initialization
 *
 * @param[in] hostId : the link's host id.
 * @param[in] gattRole : @ref BleGattRole "BLE GATT role".
 * @param[in] info : a pointer to FOTA information.
 * @param[in] callback : a pointer to a callback function that receive the service events.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED : Registered services buffer full.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus setFOTA_ServiceInit(uint8_t hostId, BleGattRole gattRole, BLEATT_FOTA_Info *info, BleFOTA_EventCallBack callback)
{
    BleStackStatus status;
    uint8_t config_index;

    if (info == NULL)
    {
        return BLESTACK_STATUS_ERR_INVALID_PARAM;
    }

    // init service basic information and get "config_index" & "fota_count"
    status = setBLE_ServiceBasicInit(hostId, gattRole, MAX_NUM_CONN_FOTA, fotaBasicInfo, &config_index, &fota_count);
    BLESTACK_STATUS_CHECK(status);

    // Set service role
    info->role = gattRole;

    // Set FOTA data
    fotaInfo[config_index] = info;

    // Register FOTA callback function
    fotaCallback[config_index] = callback;

    // Get handles at initialization if role is set to BLE_GATT_ROLE_SERVER
    if (gattRole == BLE_GATT_ROLE_SERVER)
    {
        status = getFOTA_ServiceHandles(hostId, fotaInfo[config_index]);
        BLESTACK_STATUS_CHECK(status);
    }

    return BLESTACK_STATUS_SUCCESS;
}


/** Get FOTA Service Handle Numbers
 *
 * @attention
 *            - role = <b> @ref BLE_GATT_ROLE_CLIENT </b> \n
 *              MUST call this API to get service information after received @ref BLECMD_EVENT_ATT_DATABASE_PARSING_FINISHED  \n
 *            - role = <b> @ref BLE_GATT_ROLE_SERVER </b> \n
 *              MUST call this API to get service information before connection established. \n
 *
 * @param[in] hostId : the link's host id.
 * @param[out] info : a pointer to FOTA information
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus getFOTA_ServiceHandles(uint8_t hostId, BLEATT_FOTA_Info *info)
{
    BleStackStatus status;

    // Get FOTA handles
    status = getBLEGATT_HandleNumAddr(hostId, info->role, (ATTRIBUTE_BLE *)&ATT_FOTA_PRIMARY_SERVICE, (void *)&info->handles);
    BLESTACK_STATUS_CHECK(status);

    return BLESTACK_STATUS_SUCCESS;
}


/**************************************************************************
 * BLE Service << FOTA >> General Callback Function
 **************************************************************************/
static void bleFOTA_PostEvent(BLE_Event_AttParam *attParam, BleFOTA_EventCallBack *callback)
{
    // check callback is null or not
    SERVICE_CALLBACK_NULL_CHECK(*callback);

    // post event to user
    (*callback)(attParam->hostId, attParam->cmdAccess, attParam->data, attParam->length);
}

// handle FOTA client GATT event
static void handle_FOTA_client(uint8_t index, BLE_Event_AttParam *attParam)
{
    // To be implemented
}


// handle FOTA server GATT event
static void handle_FOTA_server(uint8_t index, BLE_Event_AttParam *attParam)
{
    // Here only deal with cccd related process, others are post to user.c
    switch (attParam->cmdAccess)
    {
    case OPCODE_ATT_WRITE_REQUEST:
        if (attParam->hdlNum == fotaInfo[index]->handles.hdl_fota_data_cccd)
        {
            // update server defined cccd value
            setBLEGATT_HandleCCCDWriteRequest(attParam->data, attParam->length, &fotaInfo[index]->data.fota_data_cccd);
        }
        else if (attParam->hdlNum == fotaInfo[index]->handles.hdl_fota_command_cccd)
        {
            // update server defined cccd value
            setBLEGATT_HandleCCCDWriteRequest(attParam->data, attParam->length, &fotaInfo[index]->data.fota_command_cccd);
        }
        else if (attParam->hdlNum == fotaInfo[index]->handles.hdl_fota_command)
        {
            // received write request from client -> post to user
            attParam->cmdAccess = BLESERVICE_FOTA_COMMAND_WRITE_EVENT;
            bleFOTA_PostEvent(attParam, &fotaCallback[index]);
        }
        break;

    case OPCODE_ATT_READ_BY_TYPE_REQUEST:
    case OPCODE_ATT_READ_REQUEST:
        if (attParam->hdlNum == fotaInfo[index]->handles.hdl_fota_data_cccd)
        {
            // send cccd Read or Read By Type Response
            setBLEGATT_HandleCCCDGeneralReadRequest(attParam->hostId,
                                                    attParam->cmdAccess,
                                                    fotaInfo[index]->handles.hdl_fota_data_cccd,
                                                    fotaInfo[index]->data.fota_data_cccd);
        }
        else if (attParam->hdlNum == fotaInfo[index]->handles.hdl_fota_command_cccd)
        {
            // send cccd Read or Read By Type Response
            setBLEGATT_HandleCCCDGeneralReadRequest(attParam->hostId,
                                                    attParam->cmdAccess,
                                                    fotaInfo[index]->handles.hdl_fota_command_cccd,
                                                    fotaInfo[index]->data.fota_command_cccd);
        }
        break;

    case OPCODE_ATT_WRITE_COMMAND:
        if (attParam->hdlNum == fotaInfo[index]->handles.hdl_fota_data)
        {
            // received write command from client -> post to user
            attParam->cmdAccess = BLESERVICE_FOTA_DATA_WRITE_EVENT;
            bleFOTA_PostEvent(attParam, &fotaCallback[index]);
        }
        break;

    default:
        break;
    }
}


void ATTR_VALUE_FOTA_General_Access(BLE_Event_AttParam *attParam)
{
    uint8_t index = 0;

    if (queryIndexByHostIdGattRole(attParam->hostId, attParam->gattRole, MAX_NUM_CONN_FOTA, fotaBasicInfo, &index) != BLESTACK_STATUS_SUCCESS)
    {
        // Host id has not registered so there is no callback function -> do nothing
        return;
    }

    if (attParam->gattRole == BLE_GATT_ROLE_CLIENT)
    {
        // handle FOTA client GATT event
        handle_FOTA_client(index, attParam);
    }

    if (attParam->gattRole == BLE_GATT_ROLE_SERVER)
    {
        // handle FOTA server GATT event
        handle_FOTA_server(index, attParam);
    }
}


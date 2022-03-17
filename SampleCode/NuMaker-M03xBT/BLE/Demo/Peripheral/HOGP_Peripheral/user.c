/**************************************************************************//**
 * @file     user.c
 * @version  V0.9
 * $Revision: 01 $
 * @brief
 *           Demonstrate how to use LIB pre-build function to implement BLE
 *           HOGP application.
 *           This demo includes keyboard, mouse, volume control
 * @note
 *
 ******************************************************************************/
#include <stdio.h>
#include "BleAppSetting.h"
#include "ble_cmd.h"
#include "ble_event.h"
#include "ble_host.h"
#include "ble_profile.h"
#include "stdlib.h"

/**************************************************************************
* Application Definitions
**************************************************************************/
// LINK please refer to "ble_profile_def.c" --> ATT_DB_MAPPING
#define CONN_HOGP_LINK_HOSTID           0       // host Id start from 0

// uint16 convert to uint8 high byte and low byte
#define U16_HIGHBYTE(x)                 (uint8_t)((x >> 8) & 0xFF)
#define U16_LOWBYTE(x)                  (uint8_t)(x & 0xFF)

// Advertising device name & GAP device name
#define DEVICE_NAME             'N', 'u', 'v', 'o', 't', 'o', 'n', '_', 'H', 'O', 'G', 'P'

// Advertising parameters
#define APP_ADV_INTERVAL_MIN            160U    // 160*0.625ms=100ms
#define APP_ADV_INTERVAL_MAX            160U    // 160*0.625ms=100ms

// GAP device name
const uint8_t DEVICE_NAME_STR[] = {DEVICE_NAME};


const uint8_t HID_RPT_CS_KEY_DEMO[][2] =
{
    {0xE9, 0x00,},  //vol+
    {0xEA, 0x00,},  //vol-
    {0xE2, 0x00,},  //Mute
    {0xB0, 0x00,},  //Play
    {0xB1, 0x00,},  //Pause
    {0xB3, 0x00,},  //Fast forward
    {0xB4, 0x00,},  //Rewind
    {0xB5, 0x00,},  //Scan next track
    {0xB6, 0x00,},  //Scan previous track
    {0xB7, 0x00,},  //Stop
    {0xB8, 0x00,},  //Eject
    {0x8A, 0x01,},  //Email reader
    {0x96, 0x01,},  //Internet browser
    {0x9E, 0x01,},  //Terminal lock/screensaver
    {0xC6, 0x01,},  //Research/search browser
    {0x2D, 0x02,},  //Zoom in
};

#define STATE_HID_REPORT_CS_INITIAL             0
#define STATE_HID_REPORT_CS_DATA_UPD            0x01

#define STATE_HID_REPORT_KB_INITIAL             0
#define STATE_HID_REPORT_KB_DATA_UPD            0x01

#define HDL_HIDS_REPORT_TAB_CSKEY_L             0
#define HDL_HIDS_REPORT_TAB_CSKEY_H             1

#define HDL_HIDS_REPORT_TAB_KEY_L_R             0
#define HDL_HIDS_REPORT_TAB_DIR_L_R_L           1
#define HDL_HIDS_REPORT_TAB_DIR_L_R_H           2
#define HDL_HIDS_REPORT_TAB_DIR_U_D_L           3
#define HDL_HIDS_REPORT_TAB_DIR_U_D_H           4
#define HDL_HIDS_REPORT_TAB_ROL_U_D             5
#define HDL_HIDS_REPORT_TAB_ROL_L_R_L           6
#define HDL_HIDS_REPORT_TAB_ROL_L_R_H           7

#define HDL_HIDS_REPORT_TAB_KEY_CTRL            0
#define HDL_HIDS_REPORT_TAB_KEY_DATA0           2
#define HDL_HIDS_REPORT_TAB_KEY_DATA1           3
#define HDL_HIDS_REPORT_TAB_KEY_DATA2           4
#define HDL_HIDS_REPORT_TAB_KEY_DATA3           5
#define HDL_HIDS_REPORT_TAB_KEY_DATA4           6
#define HDL_HIDS_REPORT_TAB_KEY_DATA5           7

/**************************************************************************
* Application Variables
**************************************************************************/
uint8_t hogp_data_transmit_enable    = 0;         // 1: transmit hogp data enabled

uint8_t HID_report_MS_key_temp;         //mouse control value. It is a counter, also use to control keyboard/consumer behavior in this demo

uint8_t STATE_HID_reportCS;             //consumer state
uint8_t HID_report_CS_key_temp;         //consumer control value. Here use to control volume

uint8_t STATE_HID_reportKB;             //keyboard state
uint8_t HID_report_KB_key_temp;         //keyboard control value

#if (IOCAPABILITY_SETTING == KEYBOARD_ONLY)
uint8_t ble_passKeyConfirmedState = 0;  //wait to 1 to set scanned Passkey.
#endif

/**************************************************************************
* Function Prototype Declarations
**************************************************************************/
BleStackStatus Ble_AdvStart(uint8_t hostId);
void BleService_GATT_DataInit(BLEATT_GATT_Data *data);
void BleService_HID_DataInit(BLEATT_HID_Data *data);
void BleService_BAS_DataInit(BLEATT_BAS_Data *data);
static void BleEvent_Callback(BleCmdEvent event, void *param);
static void BleService_HID_Link0Handler(uint8_t hostId, uint8_t cmdAccess, uint8_t *data, uint16_t length);
static void BleService_BAS_Link0Handler(uint8_t hostId, uint8_t cmdAccess, uint8_t *data, uint16_t length);

/**************************************************************************
 * Function
 **************************************************************************/
void handle_AppLink0_HOGP(void)
{
    BleStackStatus status;

    if (bleProfile_link0_info.bleState == STATE_BLE_STANDBY)
    {
        // reset preferred MTU size
        setBLEGATT_PreferredMtuSize(CONN_HOGP_LINK_HOSTID, DEFAULT_MTU);

        // reset service data
        BleService_GATT_DataInit(&(bleProfile_link0_info.serviceGATT_info_s.data));
        BleService_HID_DataInit(&bleProfile_link0_info.serviceHID_info_s.data);
        BleService_BAS_DataInit(&bleProfile_link0_info.serviceBAS_info_s.data);

        // reset variables
        STATE_HID_reportKB = STATE_HID_REPORT_KB_INITIAL;
        STATE_HID_reportCS = STATE_HID_REPORT_CS_INITIAL;
        HID_report_KB_key_temp = 0;
        HID_report_CS_key_temp = 0;
        HID_report_MS_key_temp = 0x80;

        // enable advertisement
        status = Ble_AdvStart(CONN_HOGP_LINK_HOSTID);
        if (status == BLESTACK_STATUS_SUCCESS)
        {
            bleProfile_link0_info.bleState = STATE_BLE_ADVERTISING;
        }
    }
    else if (bleProfile_link0_info.bleState == STATE_BLE_CONNECTION)
    {
#if (IOCAPABILITY_SETTING == KEYBOARD_ONLY)
        if (ble_passKeyConfirmedState == 1)
        {
#if defined (__ICCARM__)
            char passkey[8];
            uint32_t passkey_a;
            char ch;
            uint8_t i = 0;

            ble_passKeyConfirmedState = 0;

            scanf("%c", &ch);
            passkey[i] = ch;
            i++;
            while((ch != 'r') && (ch != '\n') && (i < 6))   //6: Decimal representation of the six digits.
            {
                scanf("%c",&ch);
                passkey[i] = ch;
                i++;
            }
            sscanf(passkey, "%06d", &passkey_a);
            printf("BLE_PAIRING_KEY = %06d\n", passkey_a);                                // show the passkey
            setBLE_PairingPassKey(bleProfile_link0_info.hostId, (uint32_t)passkey_a);     // set scanned passkey
#else
            uint32_t passkey;

            ble_passKeyConfirmedState = 0;

            scanf("%d", &passkey);                                                      // wait for passkey entered
            printf("BLE_PAIRING_KEY = %06d\n", passkey);                                // show the passkey
            setBLE_PairingPassKey(bleProfile_link0_info.hostId, (uint32_t)passkey);     // set scanned passkey
#endif
        }
#endif
        if (hogp_data_transmit_enable == 1)
        {
            hogp_data_transmit_enable = 0;

            if ((HID_report_MS_key_temp & 0x3F) != 0x3F)    //(counter value!=0x3F or 0x7F or 0xBF or 0xFF)
            {
                if (HID_report_MS_key_temp < 0x80)
                {
                    if (bleProfile_link0_info.serviceHID_info_s.data.hid_mouse_input_report_cccd != 0)
                    {
                        if (HID_report_MS_key_temp <= 0x1F)    //counter 0~0x1F, mouse move right-down
                        {
                            bleProfile_link0_info.serviceHID_info_s.data.hid_mouse_input_report[HDL_HIDS_REPORT_TAB_DIR_L_R_L] = 0x05;    // right
                            bleProfile_link0_info.serviceHID_info_s.data.hid_mouse_input_report[HDL_HIDS_REPORT_TAB_DIR_L_R_H] = 0x00;
                            bleProfile_link0_info.serviceHID_info_s.data.hid_mouse_input_report[HDL_HIDS_REPORT_TAB_DIR_U_D_L] = 0x05;    // down
                            bleProfile_link0_info.serviceHID_info_s.data.hid_mouse_input_report[HDL_HIDS_REPORT_TAB_DIR_U_D_H] = 0x00;
                        }
                        else if (HID_report_MS_key_temp <= 0x3F)    //counter 0x20~0x3F, mouse move left-down
                        {
                            bleProfile_link0_info.serviceHID_info_s.data.hid_mouse_input_report[HDL_HIDS_REPORT_TAB_DIR_L_R_L] = 0xFA;    // left
                            bleProfile_link0_info.serviceHID_info_s.data.hid_mouse_input_report[HDL_HIDS_REPORT_TAB_DIR_L_R_H] = 0xFF;
                            bleProfile_link0_info.serviceHID_info_s.data.hid_mouse_input_report[HDL_HIDS_REPORT_TAB_DIR_U_D_L] = 0x05;    // down
                            bleProfile_link0_info.serviceHID_info_s.data.hid_mouse_input_report[HDL_HIDS_REPORT_TAB_DIR_U_D_H] = 0x00;
                        }
                        else if (HID_report_MS_key_temp <= 0x5F)    //counter 0x40~0x5F, mouse move left-up
                        {
                            bleProfile_link0_info.serviceHID_info_s.data.hid_mouse_input_report[HDL_HIDS_REPORT_TAB_DIR_L_R_L] = 0xFA;    // left
                            bleProfile_link0_info.serviceHID_info_s.data.hid_mouse_input_report[HDL_HIDS_REPORT_TAB_DIR_L_R_H] = 0xFF;
                            bleProfile_link0_info.serviceHID_info_s.data.hid_mouse_input_report[HDL_HIDS_REPORT_TAB_DIR_U_D_L] = 0xFA;    // up
                            bleProfile_link0_info.serviceHID_info_s.data.hid_mouse_input_report[HDL_HIDS_REPORT_TAB_DIR_U_D_H] = 0xFF;
                        }
                        else if (HID_report_MS_key_temp <= 0x7F)    //counter 0x60~0x7F, mouse move right-up
                        {
                            bleProfile_link0_info.serviceHID_info_s.data.hid_mouse_input_report[HDL_HIDS_REPORT_TAB_DIR_L_R_L] = 0x05;    // right
                            bleProfile_link0_info.serviceHID_info_s.data.hid_mouse_input_report[HDL_HIDS_REPORT_TAB_DIR_L_R_H] = 0x00;
                            bleProfile_link0_info.serviceHID_info_s.data.hid_mouse_input_report[HDL_HIDS_REPORT_TAB_DIR_U_D_L] = 0xFA;    // up
                            bleProfile_link0_info.serviceHID_info_s.data.hid_mouse_input_report[HDL_HIDS_REPORT_TAB_DIR_U_D_H] = 0xFF;
                        }

                        if (setBLEGATT_Notification(bleProfile_link0_info.hostId, bleProfile_link0_info.serviceHID_info_s.handles.hdl_hid_mouse_input_report, bleProfile_link0_info.serviceHID_info_s.data.hid_mouse_input_report, sizeof(bleProfile_link0_info.serviceHID_info_s.data.hid_mouse_input_report)) == BLESTACK_STATUS_SUCCESS)
                        {
                            HID_report_MS_key_temp++;    //counter++
                        }
                    }
                    else
                    {
                        HID_report_MS_key_temp++;    //counter++
                    }
                }
                else
                {
                    HID_report_MS_key_temp++;    //counter++
                }
            }
            else    //(counter vlaue==0x3F or 0x7F or 0xBF or 0xFF)
            {
                if ((HID_report_MS_key_temp == 0x3F) || (HID_report_MS_key_temp == 0xBF))    //control keyboard when counter=0x3F, 0xBF
                {
                    if (bleProfile_link0_info.serviceHID_info_s.data.hid_keyboard_intput_report_cccd != 0)
                    {
                        if ((STATE_HID_reportKB & STATE_HID_REPORT_KB_DATA_UPD) == 0)   //check keyboard report status
                        {
                            if ((HID_report_KB_key_temp <= 0x04) || (HID_report_KB_key_temp >= 0x27))
                            {
                                HID_report_KB_key_temp = 0x04;    //0x04 mean 'a'; 0x27 mean '9'; see USB HID spec.
                            }
                            bleProfile_link0_info.serviceHID_info_s.data.hid_keyboard_intput_report[HDL_HIDS_REPORT_TAB_KEY_DATA0] = HID_report_KB_key_temp;    // repeat keyCode: 'a' 'b' ~ 'z' ~ '1' '2'  ~ '9'
                            if (setBLEGATT_Notification(bleProfile_link0_info.hostId, bleProfile_link0_info.serviceHID_info_s.handles.hdl_hid_keyboard_intput_report, bleProfile_link0_info.serviceHID_info_s.data.hid_keyboard_intput_report, sizeof(bleProfile_link0_info.serviceHID_info_s.data.hid_keyboard_intput_report)) == BLESTACK_STATUS_SUCCESS)
                            {
                                STATE_HID_reportKB |= STATE_HID_REPORT_KB_DATA_UPD;
                                HID_report_KB_key_temp++;    //keyboard keycode
                            }
                        }
                        else    //release key
                        {
                            bleProfile_link0_info.serviceHID_info_s.data.hid_keyboard_intput_report[HDL_HIDS_REPORT_TAB_KEY_DATA0] = 0x00;    // release key
                            if (setBLEGATT_Notification(bleProfile_link0_info.hostId, bleProfile_link0_info.serviceHID_info_s.handles.hdl_hid_keyboard_intput_report, bleProfile_link0_info.serviceHID_info_s.data.hid_keyboard_intput_report, sizeof(bleProfile_link0_info.serviceHID_info_s.data.hid_keyboard_intput_report)) == BLESTACK_STATUS_SUCCESS)
                            {
                                STATE_HID_reportKB &= ~STATE_HID_REPORT_KB_DATA_UPD;
                                HID_report_MS_key_temp++;
                            }
                        }
                    }
                    else
                    {
                        HID_report_MS_key_temp++;
                    }
                }
                if ((HID_report_MS_key_temp == 0x7F) || (HID_report_MS_key_temp == 0xFF))    //control volume when counter=0x7F, 0xFF
                {
                    if (bleProfile_link0_info.serviceHID_info_s.data.hid_consumer_input_report_cccd != 0)
                    {
                        if ((STATE_HID_reportCS & STATE_HID_REPORT_CS_DATA_UPD) == 0)    // check consumer report status
                        {
                            if ((HID_report_CS_key_temp & 0x01) == 0x01)
                            {
                                bleProfile_link0_info.serviceHID_info_s.data.hid_consumer_input_report[0] = HID_RPT_CS_KEY_DEMO[0][0];    // vol+
                                bleProfile_link0_info.serviceHID_info_s.data.hid_consumer_input_report[1] = HID_RPT_CS_KEY_DEMO[0][1];
                            }
                            else
                            {
                                bleProfile_link0_info.serviceHID_info_s.data.hid_consumer_input_report[0] = HID_RPT_CS_KEY_DEMO[1][0];    // vol-
                                bleProfile_link0_info.serviceHID_info_s.data.hid_consumer_input_report[1] = HID_RPT_CS_KEY_DEMO[1][1];
                            }
                            if (setBLEGATT_Notification(bleProfile_link0_info.hostId, bleProfile_link0_info.serviceHID_info_s.handles.hdl_hid_consumer_input_report, bleProfile_link0_info.serviceHID_info_s.data.hid_consumer_input_report, sizeof(bleProfile_link0_info.serviceHID_info_s.data.hid_consumer_input_report)) == BLESTACK_STATUS_SUCCESS)
                            {
                                STATE_HID_reportCS |= STATE_HID_REPORT_CS_DATA_UPD;
                                HID_report_CS_key_temp++;    // just counter for send another consumer data
                            }
                        }
                        else    //release key
                        {
                            bleProfile_link0_info.serviceHID_info_s.data.hid_consumer_input_report[0] = 0x00;    // release key
                            bleProfile_link0_info.serviceHID_info_s.data.hid_consumer_input_report[1] = 0x00;
                            if (setBLEGATT_Notification(bleProfile_link0_info.hostId, bleProfile_link0_info.serviceHID_info_s.handles.hdl_hid_consumer_input_report, bleProfile_link0_info.serviceHID_info_s.data.hid_consumer_input_report, sizeof(bleProfile_link0_info.serviceHID_info_s.data.hid_consumer_input_report)) == BLESTACK_STATUS_SUCCESS)
                            {
                                STATE_HID_reportCS &= ~STATE_HID_REPORT_CS_DATA_UPD;
                                HID_report_MS_key_temp++;
                            }
                        }
                    }
                    else
                    {
                        HID_report_MS_key_temp++;
                    }
                }
            }
        }
    }
}

BleStackStatus Ble_AdvInit(void)
{
    BleStackStatus status;
    BLE_Adv_Param advParam;
    const uint8_t SCANRSP_LENGTH    = (2) + sizeof(DEVICE_NAME_STR); //  1 byte data type + 1 byte ad length
    const uint8_t SCANRSP_ADLENGTH  = (1) + sizeof(DEVICE_NAME_STR); //  1 byte data type

    //Adv data format. Set according to user select profile. See Bluetooth Spec. Ver5.0 [Vol 3], Part C, Section 11
    uint8_t advData[] =
    {
        0x0B,  //total length
        GAP_AD_TYPE_LENGTH_2, GAP_AD_TYPE_FLAGS, BLE_GAP_FLAGS_GENERAL_DISCOVERABLE_MODE,
        GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_SERVICE_MORE_16B_UUID, U16_LOWBYTE(GATT_SERVICES_HUMAN_INTERFACE_DEVICE), U16_HIGHBYTE(GATT_SERVICES_HUMAN_INTERFACE_DEVICE),  //0x1812: GATT_SERVICES_HUMAN_INTERFACE_DEVICE
        GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, U16_LOWBYTE(BLE_APPEARANCE_GENERIC_HID), U16_HIGHBYTE(BLE_APPEARANCE_GENERIC_HID),                           //0x03C0: 960 -> Human Interface Device (HID), HID Generic
    };

    //Scan response data format. See Bluetooth Spec. Ver5.0 [Vol 3], Part C, Section 11
    uint8_t advScanRspData[] =
    {
        SCANRSP_LENGTH,                     // Length
        SCANRSP_ADLENGTH,                   // AD length
        GAP_AD_TYPE_LOCAL_NAME_COMPLETE,    // AD data type
        DEVICE_NAME,                        // the name is shown on scan list
    };

    // Adv. parameter init
    advParam.advType = ADV_TYPE_ADV_IND;
    advParam.advIntervalMin = APP_ADV_INTERVAL_MIN;
    advParam.advIntervalMax = APP_ADV_INTERVAL_MAX;
    advParam.advChannelMap = ADV_CHANNEL_ALL;
    advParam.advFilterPolicy = ADV_FILTER_POLICY_ACCEPT_ALL;
    status = setBLE_AdvParam(&advParam);
    BLESTACK_STATUS_CHECK(status);

    // Adv. data init
    status = setBLE_AdvData((uint8_t *)advData, sizeof(advData));
    BLESTACK_STATUS_CHECK(status);

    // Adv. scan rsp data init
    status = setBLE_ScanRspData((uint8_t *)advScanRspData, sizeof(advScanRspData));
    BLESTACK_STATUS_CHECK(status);

    return BLESTACK_STATUS_SUCCESS;
}


BleStackStatus Ble_AdvStart(uint8_t hostId)
{
    BleStackStatus status;

    status = Ble_AdvInit();
    BLESTACK_STATUS_CHECK(status);

    status = setBLE_AdvEnable(hostId);
    BLESTACK_STATUS_CHECK(status);

    return BLESTACK_STATUS_SUCCESS;
}


void BleService_GATT_DataInit(BLEATT_GATT_Data *data)
{
    data->service_changed_cccd = 0;
}

void BleService_HID_DataInit(BLEATT_HID_Data *data)
{
    data->hid_control_point = 0;
    data->hid_protocol_mode = 1;

    data->hid_boot_keyboard_intput_report_cccd = 0x00;
    data->hid_keyboard_intput_report_cccd = 0x00;
    data->hid_boot_mouse_input_report_cccd = 0x00;
    data->hid_mouse_input_report_cccd = 0x00;
    data->hid_consumer_input_report_cccd = 0x00;
}

void BleService_BAS_DataInit(BLEATT_BAS_Data *data)
{
    data->battery_level = 100;
    data->battery_level_cccd  = 0;
}


BleStackStatus BleApp_ProfileInit(void)
{
    BleStackStatus status;

    //------------------------------------------------------------------------
    // init LINK0 GAP/ DIS/ UDF01S services parameter and register callback function
    //------------------------------------------------------------------------
    bleProfile_link0_info.hostId = CONN_HOGP_LINK_HOSTID;
    bleProfile_link0_info.bleState = STATE_BLE_STANDBY;
    bleProfile_link0_info.subState = 0x00;

    // GAP (Server) Related
    // -------------------------------------
    status = setGAP_ServiceInit(CONN_HOGP_LINK_HOSTID, BLE_GATT_ROLE_SERVER,  &(bleProfile_link0_info.serviceGAP_info_s), NULL);
    BLESTACK_STATUS_CHECK(status);

    // set GAP device name
    status = setGAP_DeviceName((uint8_t *)DEVICE_NAME_STR, sizeof(DEVICE_NAME_STR));
    BLESTACK_STATUS_CHECK(status);

    // set GAP appearance
    setGAP_Appearance(BLE_APPEARANCE_GENERIC_HID);

    // GATT (Server) Related
    // -------------------------------------
    status = setGATT_ServiceInit(CONN_HOGP_LINK_HOSTID, BLE_GATT_ROLE_SERVER, &(bleProfile_link0_info.serviceGATT_info_s), NULL);
    BLESTACK_STATUS_CHECK(status);

    // DIS (Server) Related
    // -------------------------------------
    status = setDIS_ServiceInit(CONN_HOGP_LINK_HOSTID, BLE_GATT_ROLE_SERVER, &(bleProfile_link0_info.serviceDIS_info_s), NULL);
    BLESTACK_STATUS_CHECK(status);

    // HID (Server) Related
    // -------------------------------------
    status = setHID_ServiceInit(CONN_HOGP_LINK_HOSTID, BLE_GATT_ROLE_SERVER, &(bleProfile_link0_info.serviceHID_info_s), BleService_HID_Link0Handler);
    BLESTACK_STATUS_CHECK(status);

    // BAS (Server) Related
    // -------------------------------------
    status = setBAS_ServiceInit(CONN_HOGP_LINK_HOSTID, BLE_GATT_ROLE_SERVER, &(bleProfile_link0_info.serviceBAS_info_s), BleService_BAS_Link0Handler);
    BLESTACK_STATUS_CHECK(status);

    return BLESTACK_STATUS_SUCCESS;
}


BleStackStatus BleApp_Init(void)
{
    BleStackStatus status = BLESTACK_STATUS_SUCCESS;
    BLE_IOCaps_Param ioCaps;

    /* set company Id*/
    setBLE_CompanyId(((uint16_t)BLE_COMPANY_ID_H << 8) | BLE_COMPANY_ID_L);

    /* register command event callback function */
    setBLE_RegisterBleEvent(BleEvent_Callback);

    /* set BLE IO Capabilities */
    ioCaps.ioCapsParam = IOCAPABILITY_SETTING;
    setBLE_IOCapabilities(&ioCaps);

    /* initial profiles */
    status = BleApp_ProfileInit();
    if (status != BLESTACK_STATUS_SUCCESS)
    {
        printf("Error init profiles.\n");
    }

    return status;
}

void BleApp_Main(void)
{
    // Handle Link 0 - HOGP Peripheral
    handle_AppLink0_HOGP();
} //end of BleApp_Main()


/* BLE Callback Function */
static void BleEvent_Callback(BleCmdEvent event, void *param)
{
    switch (event)
    {
    case BLECMD_EVENT_ADV_COMPLETE:
        printf("Advertising...\n");
        break;

    case BLECMD_EVENT_CONN_COMPLETE:
    {
        BLE_Event_ConnParam *connParam = (BLE_Event_ConnParam *)param;

        // set connection state
        if (connParam->hostId == CONN_HOGP_LINK_HOSTID)
        {
            bleProfile_link0_info.bleState = STATE_BLE_CONNECTION;
        }
        printf("Status=%d, ID=%d, Connected to %02x:%02x:%02x:%02x:%02x:%02x\n",
               connParam->status,
               connParam->hostId,
               connParam->peerAddr.addr[5],
               connParam->peerAddr.addr[4],
               connParam->peerAddr.addr[3],
               connParam->peerAddr.addr[2],
               connParam->peerAddr.addr[1],
               connParam->peerAddr.addr[0]);
    }
    break;

    case BLECMD_EVENT_DISCONN_COMPLETE:
    {
        BLE_Event_DisconnParam *disconnParam = (BLE_Event_DisconnParam *)param;

        // reset state
        if (disconnParam->hostId == CONN_HOGP_LINK_HOSTID)
        {
            bleProfile_link0_info.bleState = STATE_BLE_STANDBY;
        }
        printf("Disconnected, ID:%d, Reason:0x%X\n", disconnParam->hostId, disconnParam->disconnectReason);
    }
    break;

    case BLECMD_EVENT_CONN_UPDATE_COMPLETE:
    {
        BLE_Event_ConnUpdateParam *connUpdateParam  = (BLE_Event_ConnUpdateParam *)param;

        printf("Connection updated\n");
        printf("Status: %d, ", connUpdateParam->status);
        printf("ID: %d, ", connUpdateParam->hostId);
        printf("Interval: %d, ", connUpdateParam->connInterval);
        printf("Latency: %d, ", connUpdateParam->connLatency);
        printf("Supervision Timeout: %d\n", connUpdateParam->connSupervisionTimeout);
    }
    break;

    case BLECMD_EVENT_PHY_UPDATE_COMPLETE:
    {
        BLE_Event_PhyUpdateParam *phy = (BLE_Event_PhyUpdateParam *)param;
        printf("PHY updated, status: %d, ID: %d, TX PHY: %d, RX PHY: %d\n", phy->status, phy->hostId, phy->phy.txPhy, phy->phy.rxPhy);
    }
    break;

    case BLECMD_EVENT_STK_GEN_METHOD:
    {
        BLE_Event_StkGenMethodParam *passkey_method = (BLE_Event_StkGenMethodParam *)param;
        if (passkey_method->stkGenMethod == PASSKEY_ENTRY)
        {
            // I/O Capability is keyboard
            // Start scanning user-entered passkey.
        }
        else if (passkey_method->stkGenMethod == PASSKEY_DISPLAY)
        {
            // I/O Capability is display
            // Generate a 6-digit random code and display it for pairing.
            printf("BLE_PAIRING_KEY = %d\n", (uint32_t)BLE_PAIRING_KEY);
        }
    }
    break;

    case BLECMD_EVENT_PASSKEY_CONFIRM:
    {
        //enter a scanned Passkey or use a randomly generaated passkey.
#if (IOCAPABILITY_SETTING == DISPLAY_ONLY)
        BLE_Event_PassKeyConfirmParam *event_param = (BLE_Event_PassKeyConfirmParam *)param;
        setBLE_PairingPassKey(event_param->hostId, (uint32_t)BLE_PAIRING_KEY);

#elif (IOCAPABILITY_SETTING == KEYBOARD_ONLY)
        ble_passKeyConfirmedState = 1;
        printf("Please enter passkey...\n");
#endif
    }
    break;

    case BLECMD_EVENT_AUTH_STATUS:
    {
        BLE_Event_AuthStatusParam *auth_result = (BLE_Event_AuthStatusParam *)param;

        // restore cccd data from bonding information
        if (auth_result->status == AUTH_SUCCESS)
        {
            getBLEGATT_RestoreCCCDValueFromBond(CONN_HOGP_LINK_HOSTID);
        }

        printf("AUTH Report, ID:%d , Status:%d\n", auth_result->hostId, auth_result->status);
    }
    break;

    default:
        break;
    }
}


static void BleService_HID_Link0Handler(uint8_t hostId, uint8_t cmdAccess, uint8_t *data, uint16_t length)
{
    uint16_t i;

    switch (cmdAccess)
    {
    case BLESERVICE_HID_PROTOCOL_MODE_READ_EVENT:
        setBLEGATT_GeneralReadRsp(bleProfile_link0_info.hostId,
                                  bleProfile_link0_info.serviceHID_info_s.handles.hdl_hid_protocol_mode,
                                  &bleProfile_link0_info.serviceHID_info_s.data.hid_protocol_mode,
                                  sizeof(bleProfile_link0_info.serviceHID_info_s.data.hid_protocol_mode));
        break;

    case BLESERVICE_HID_BOOT_KEYBOARD_INPUT_REPORT_READ_EVENT:
        setBLEGATT_GeneralReadRsp(bleProfile_link0_info.hostId,
                                  bleProfile_link0_info.serviceHID_info_s.handles.hdl_hid_boot_keyboard_intput_report,
                                  &bleProfile_link0_info.serviceHID_info_s.data.hid_boot_keyboard_intput_report[0],
                                  sizeof(bleProfile_link0_info.serviceHID_info_s.data.hid_boot_keyboard_intput_report));
        break;

    case BLESERVICE_HID_BOOT_KEYBOARD_OUTPUT_REPORT_READ_EVENT:
        setBLEGATT_GeneralReadRsp(bleProfile_link0_info.hostId,
                                  bleProfile_link0_info.serviceHID_info_s.handles.hdl_hid_boot_keyboard_output_report,
                                  &bleProfile_link0_info.serviceHID_info_s.data.hid_boot_keyboard_output_report[0],
                                  sizeof(bleProfile_link0_info.serviceHID_info_s.data.hid_boot_keyboard_output_report));
        break;

    case BLESERVICE_HID_KEYBOARD_INPUT_REPORT_READ_EVENT:
        setBLEGATT_GeneralReadRsp(bleProfile_link0_info.hostId,
                                  bleProfile_link0_info.serviceHID_info_s.handles.hdl_hid_keyboard_output_report,
                                  &bleProfile_link0_info.serviceHID_info_s.data.hid_keyboard_output_report[0],
                                  sizeof(bleProfile_link0_info.serviceHID_info_s.data.hid_keyboard_output_report));
        break;

    case BLESERVICE_HID_KEYBOARD_OUTPUT_REPORT_READ_EVENT:
        setBLEGATT_GeneralReadRsp(bleProfile_link0_info.hostId,
                                  bleProfile_link0_info.serviceHID_info_s.handles.hdl_hid_keyboard_output_report,
                                  &bleProfile_link0_info.serviceHID_info_s.data.hid_keyboard_output_report[0],
                                  sizeof(bleProfile_link0_info.serviceHID_info_s.data.hid_keyboard_output_report));
        break;

    case BLESERVICE_HID_BOOT_MOUSE_INPUT_REPORT_READ_EVENT:
        setBLEGATT_GeneralReadRsp(bleProfile_link0_info.hostId,
                                  bleProfile_link0_info.serviceHID_info_s.handles.hdl_hid_boot_mouse_input_report,
                                  &bleProfile_link0_info.serviceHID_info_s.data.hid_boot_mouse_input_report[0],
                                  sizeof(bleProfile_link0_info.serviceHID_info_s.data.hid_boot_mouse_input_report));
        break;

    case BLESERVICE_HID_MOUSE_INPUT_REPORT_READ_EVENT:
        setBLEGATT_GeneralReadRsp(bleProfile_link0_info.hostId,
                                  bleProfile_link0_info.serviceHID_info_s.handles.hdl_hid_mouse_input_report,
                                  &bleProfile_link0_info.serviceHID_info_s.data.hid_mouse_input_report[0],
                                  sizeof(bleProfile_link0_info.serviceHID_info_s.data.hid_mouse_input_report));
        break;

    case BLESERVICE_HID_CONSUMER_INPUT_REPORT_READ_EVENT:
        setBLEGATT_GeneralReadRsp(bleProfile_link0_info.hostId,
                                  bleProfile_link0_info.serviceHID_info_s.handles.hdl_hid_consumer_input_report,
                                  &bleProfile_link0_info.serviceHID_info_s.data.hid_consumer_input_report[0],
                                  sizeof(bleProfile_link0_info.serviceHID_info_s.data.hid_consumer_input_report));
        break;

    case BLESERVICE_HID_BOOT_KEYBOARD_INPUT_REPORT_WRITE_EVENT:
        for (i = 0; i < length; i++)
        {
            *(&bleProfile_link0_info.serviceHID_info_s.data.hid_boot_keyboard_intput_report[0] + i) = *(data + i);
        }
        break;

    case BLESERVICE_HID_BOOT_KEYBOARD_OUTPUT_REPORT_WRITE_EVENT:
        for (i = 0; i < length; i++)
        {
            *(&bleProfile_link0_info.serviceHID_info_s.data.hid_boot_keyboard_output_report[0] + i) = *(data + i);
        }
        break;

    case BLESERVICE_HID_KEYBOARD_INPUT_REPORT_WRITE_EVENT:
        for (i = 0; i < length; i++)
        {
            *(&bleProfile_link0_info.serviceHID_info_s.data.hid_keyboard_intput_report[0] + i) = *(data + i);
        }
        break;

    case BLESERVICE_HID_KEYBOARD_OUTPUT_REPORT_WRITE_EVENT:
        for (i = 0; i < length; i++)
        {
            *(&bleProfile_link0_info.serviceHID_info_s.data.hid_keyboard_output_report[0] + i) = *(data + i);
        }
        break;

    case BLESERVICE_HID_BOOT_MOUSE_INPUT_REPORT_WRITE_EVENT:
        for (i = 0; i < length; i++)
        {
            *(&bleProfile_link0_info.serviceHID_info_s.data.hid_boot_mouse_input_report[0] + i) = *(data + i);
        }
        break;

    case BLESERVICE_HID_MOUSE_INPUT_REPORT_WRITE_EVENT:
        for (i = 0; i < length; i++)
        {
            *(&bleProfile_link0_info.serviceHID_info_s.data.hid_mouse_input_report[0] + i) = *(data + i);
        }
        break;

    case BLESERVICE_HID_CONSUMER_INPUT_REPORT_WRITE_EVENT:
        for (i = 0; i < length; i++)
        {
            *(&bleProfile_link0_info.serviceHID_info_s.data.hid_consumer_input_report[0] + i) = *(data + i);
        }
        break;

    case BLESERVICE_HID_CONTROL_POINT_WRITE_EVNET:
        bleProfile_link0_info.serviceHID_info_s.data.hid_control_point = *data;
        break;

    case BLESERVICE_HID_PROTOCOL_MODE_WRITE_EVENT:
        bleProfile_link0_info.serviceHID_info_s.data.hid_protocol_mode = *data;
        break;

    default:
        break;
    }
}

static void BleService_BAS_Link0Handler(uint8_t hostId, uint8_t cmdAccess, uint8_t *data, uint16_t length)
{
    switch (cmdAccess)
    {
    case BLESERVICE_BAS_LEVEL_READ_EVENT:
        setBLEGATT_GeneralReadRsp(bleProfile_link0_info.hostId,
                                  bleProfile_link0_info.serviceBAS_info_s.handles.hdl_battery_level,
                                  &bleProfile_link0_info.serviceBAS_info_s.data.battery_level,
                                  sizeof(bleProfile_link0_info.serviceBAS_info_s.data.battery_level));
        break;

    default:
        break;
    }
}

void TMR0_IRQHandler(void)
{
    if (TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);
    }

    // transmit HOGP data
    hogp_data_transmit_enable = 1;
}


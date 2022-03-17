/**************************************************************************//**
 * @file     user.c
 * @version  V0.9
 * $Revision: 01 $
 * @brief
 *           Demonstrate how to use LIB pre-build function to start and stop a BLE
 *           connection.
 * @note
 *
 ******************************************************************************/
#include <stdio.h>
#include "BleAppSetting.h"
#include "ble_cmd.h"
#include "ble_event.h"
#include "ble_host.h"
#include "ble_profile.h"
#include "porting_misc.h"
#include "fota.h"

/**************************************************************************
* Application Definitions
**************************************************************************/
// LINK please refer to "ble_profile_def.c" --> ATT_DB_MAPPING
#define CONN_HRS_LINK_HOSTID            0       // host Id start from 0

// uint16 convert to uint8 high byte and low byte
#define U16_HIGHBYTE(x)                 (uint8_t)((x >> 8) & 0xFF)
#define U16_LOWBYTE(x)                  (uint8_t)(x & 0xFF)

// Advertising device name
#define DEVICE_NAME                     'N', 'u', 'v', 'o', 't', 'o', 'n', '_', 'O', 'T', 'A'

// Advertising parameters
#define APP_ADV_INTERVAL_MIN            160U    // 160*0.625ms=100ms
#define APP_ADV_INTERVAL_MAX            160U    // 160*0.625ms=100ms

// GAP device name
const uint8_t DEVICE_NAME_STR[] = {DEVICE_NAME};


/**************************************************************************
 * Variable
 **************************************************************************/
uint8_t hrs_data_transmit_enable    = 0;         // 1: transmit hrs data enabled
uint8_t fota_timer_expired          = 0;         // 1: expired

/**************************************************************************
 * Function Prototype Declaration
 **************************************************************************/
BleStackStatus Ble_AdvStart(uint8_t hostId);
void BleService_GATT_DataInit(BLEATT_GATT_Data *data);
void BleService_HRS_DataInit(BLEATT_HRS_Data *data);
void BleService_FOTA_DataInit(BLEATT_FOTA_Data *data);
static void BleEvent_Callback(BleCmdEvent event, void *param);
static void BleService_FOTALink0Handler(uint8_t hostId, uint8_t cmdAccess, uint8_t *data, uint16_t length);

/**************************************************************************
 * Function
 **************************************************************************/

void handle_AppLink0_FOTAHRSP(void)
{
    BleStackStatus status;

    if (bleProfile_link0_info.bleState == STATE_BLE_STANDBY)
    {
        // reset preferred MTU size
        setBLEGATT_PreferredMtuSize(CONN_HRS_LINK_HOSTID, DEFAULT_MTU);

        // reset service data
        BleService_GATT_DataInit(&(bleProfile_link0_info.serviceGATT_info_s.data));
        BleService_HRS_DataInit(&bleProfile_link0_info.serviceHRS_info_s.data);
        BleService_FOTA_DataInit(&(bleProfile_link0_info.serviceFOTA_info_s.data));

        // enable advertisement
        status = Ble_AdvStart(CONN_HRS_LINK_HOSTID);
        if (status == BLESTACK_STATUS_SUCCESS)
        {
            bleProfile_link0_info.bleState = STATE_BLE_ADVERTISING;
        }
    }
    else if (bleProfile_link0_info.bleState == STATE_BLE_CONNECTION)
    {
        // HRS
        if ((hrs_data_transmit_enable == 1)  && (bleProfile_link0_info.serviceHRS_info_s.data.heart_rate_measurement_cccd != 0 ))
        {
            hrs_data_transmit_enable = 0;

            // send heart rate measurement value to client
            if ((bleProfile_link0_info.serviceHRS_info_s.data.heart_rate_measurement[0] & BIT1) == 0)    //initial is 0x14 & 0x02 = 0, toggle "device detected" / "device not detected" information
            {
                bleProfile_link0_info.serviceHRS_info_s.data.heart_rate_measurement[0] |= BIT1;          //set Sensor Contact Status bit
            }
            else
            {
                bleProfile_link0_info.serviceHRS_info_s.data.heart_rate_measurement[0] &= ~BIT1;         //clear Sensor Contact Status bit
            }

            if (setBLEGATT_Notification(bleProfile_link0_info.hostId,
                                        bleProfile_link0_info.serviceHRS_info_s.handles.hdl_heart_rate_measurement,
                                        bleProfile_link0_info.serviceHRS_info_s.data.heart_rate_measurement,
                                        sizeof(bleProfile_link0_info.serviceHRS_info_s.data.heart_rate_measurement) / sizeof(bleProfile_link0_info.serviceHRS_info_s.data.heart_rate_measurement[0])
                                       ) == BLESTACK_STATUS_SUCCESS)
            {
                bleProfile_link0_info.serviceHRS_info_s.data.heart_rate_measurement[1]++;  //+1, Heart Rate Data. Here just a simulation, increase 1 about every second
                bleProfile_link0_info.serviceHRS_info_s.data.heart_rate_measurement[2]++;  //+1, Heart Rate RR-Interval
            }
        }

        // FOTA
        if (fota_timer_expired == 1)
        {
            fota_timer_expired = 0;

            // handle FOTA timer expired event
            BleFota_TimerExpiryHandler();
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
        0x0B, //total length
        GAP_AD_TYPE_LENGTH_2, GAP_AD_TYPE_FLAGS, BLE_GAP_FLAGS_LIMITED_DISCOVERABLE_MODE,                                                         //LE Limit Discoverable Mode, Bluetooth Spec. Ver5.0 [Vol 3], Part C, Section 11
        GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_SERVICE_MORE_16B_UUID, U16_LOWBYTE(GATT_SERVICES_HEART_RATE), U16_HIGHBYTE(GATT_SERVICES_HEART_RATE),   // GATT_SERVICES_HEART_RATE
        GAP_AD_TYPE_LENGTH_3, GAP_AD_TYPE_APPEARANCE, U16_LOWBYTE(BLE_APPEARANCE_GENERIC_HEART_RATE_SENSOR), U16_HIGHBYTE(BLE_APPEARANCE_GENERIC_HEART_RATE_SENSOR),                                                                                 //0x0340: 832 -> Generic Heart rate Sensor
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

void BleService_HRS_DataInit(BLEATT_HRS_Data *data)
{
    data->body_sensor_location = 0x02;
    data->heart_rate_measurement_cccd = 0;

    //[0]: HRS Flag; [1]: Heart Rate Data [2][3]: Heart Rate RR-Interval
    data->heart_rate_measurement[0] = 0x14; // HRS Flag
    data->heart_rate_measurement[1] = 0;    // Heart Rate Data
    data->heart_rate_measurement[2] = 0;    // Heart Rate RR-Interval
    data->heart_rate_measurement[3] = 0;    // Heart Rate RR-Interval
}

void BleService_FOTA_DataInit(BLEATT_FOTA_Data *data)
{
    data->fota_command_cccd = 0;
    data->fota_data_cccd = 0;
}

BleStackStatus BleApp_ProfileInit(void)
{
    BleStackStatus status;

    //---------------------------------------------------------------------------------------
    // init LINK0 GAP/ GATT/ DIS/ HRS/ FOTA services parameter and register callback function
    //---------------------------------------------------------------------------------------
    bleProfile_link0_info.hostId = CONN_HRS_LINK_HOSTID;
    bleProfile_link0_info.bleState = STATE_BLE_STANDBY;
    bleProfile_link0_info.subState = 0x00;

    // GAP (Server) Related
    // -------------------------------------
    status = setGAP_ServiceInit(CONN_HRS_LINK_HOSTID, BLE_GATT_ROLE_SERVER, &(bleProfile_link0_info.serviceGAP_info_s), NULL);
    BLESTACK_STATUS_CHECK(status);

    // set GAP device name
    status = setGAP_DeviceName((uint8_t *)DEVICE_NAME_STR, sizeof(DEVICE_NAME_STR));
    BLESTACK_STATUS_CHECK(status);

    // set GAP appearance
    setGAP_Appearance(BLE_APPEARANCE_GENERIC_HEART_RATE_SENSOR);

    // GATT (Server) Related
    // -------------------------------------
    status = setGATT_ServiceInit(CONN_HRS_LINK_HOSTID, BLE_GATT_ROLE_SERVER, &(bleProfile_link0_info.serviceGATT_info_s), NULL);
    BLESTACK_STATUS_CHECK(status);

    // DIS (Server) Related
    // -------------------------------------
    status = setDIS_ServiceInit(CONN_HRS_LINK_HOSTID, BLE_GATT_ROLE_SERVER, &(bleProfile_link0_info.serviceDIS_info_s), NULL);
    BLESTACK_STATUS_CHECK(status);

    // HRS (Server) Related
    // -------------------------------------
    status = setHRS_ServiceInit(CONN_HRS_LINK_HOSTID, BLE_GATT_ROLE_SERVER, &(bleProfile_link0_info.serviceHRS_info_s), NULL);
    BLESTACK_STATUS_CHECK(status);

    // FOTA (Server) Related
    // -------------------------------------
    status = setFOTA_ServiceInit(CONN_HRS_LINK_HOSTID, BLE_GATT_ROLE_SERVER, &(bleProfile_link0_info.serviceFOTA_info_s), BleService_FOTALink0Handler);
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

    /* initial FOTA */
    BleFota_Init();

    /* set BLE IO Capabilities */
    ioCaps.ioCapsParam = DISPLAY_ONLY;
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
    // Handle Link 0 - FOTA+HRS Peripheral
    handle_AppLink0_FOTAHRSP();
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
        if (connParam->hostId == CONN_HRS_LINK_HOSTID)
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
        if (disconnParam->hostId == CONN_HRS_LINK_HOSTID)
        {
            bleProfile_link0_info.bleState = STATE_BLE_STANDBY;
        }
        printf("Disconnected, ID:%d, Reason:0x%X\n", disconnParam->hostId, disconnParam->disconnectReason);

        // FOTA disconnect
        BleFota_Disconnect();
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

    case BLECMD_EVENT_PHY_READ_COMPLETE:
    {
        BLE_Event_PhyParam *phy = (BLE_Event_PhyParam *)param;
        printf("PHY read, ID:%d, TX:%d, RX:%d\n", phy->hostId, phy->phy.txPhy, phy->phy.rxPhy);
    }
    break;

    default:
        break;
    }
}

static void BleService_FOTALink0Handler(uint8_t hostId, uint8_t cmdAccess, uint8_t *data, uint16_t length)
{
    switch (cmdAccess)
    {
    case BLESERVICE_FOTA_DATA_WRITE_EVENT:
    {
        BleFota_Data(length, data);
    }
    break;

    case BLESERVICE_FOTA_COMMAND_WRITE_EVENT:
    {
        BleFota_Cmd(length, data);
    }
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

    // send heart rate measurement value
    hrs_data_transmit_enable = 1;

    // FOTA timer tick and check if timer is expired
    if (BleFota_TimerTick() == EXPIRED)
    {
        fota_timer_expired = 1;
    }
}


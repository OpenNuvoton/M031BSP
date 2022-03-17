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
#include <string.h>
#include "BleAppSetting.h"
#include "ble_cmd.h"
#include "ble_event.h"
#include "ble_host.h"
#include "ble_profile.h"
#include "porting_misc.h"

/**************************************************************************
* Application Definitions
**************************************************************************/
// LINK please refer to "ble_profile_def.c" --> ATT_DB_MAPPING
#define CONN_TRSPC_LINK_HOSTID          0       // host Id start from 0
#define CONN_HRSP_LINK_HOSTID           1

// uint16 convert to uint8 high byte and low byte
#define U16_HIGHBYTE(x)                 (uint8_t)((x >> 8) & 0xFF)
#define U16_LOWBYTE(x)                  (uint8_t)(x & 0xFF)

// Advertising device name
#define DEVICE_NAME                     'N', 'u', 'v', 'o', 't', 'o', 'n', '_', 'H', 'R', 'S'

// Target peer device name
#define TARGET_DEVICE_NAME              (uint8_t *)"Nuvoton_UART"

//Scan parameters
#define SCAN_TYPE                       SCAN_TYPE_ACTIVE
#define SCAN_FILTER                     SCAN_FILTER_POLICY_ACCEPT_ALL
#define SCAN_WINDOW                     10U    //10*0.625ms=6.25ms
#define SCAN_INTERVAL                   10U    //10*0.625ms=6.25ms

//Initial connection parameters
#define APP_CONN_INTERVAL_MIN           38U    //38*1.25ms=47.5ms
#define APP_CONN_INTERVAL_MAX           42U    //42*1.25ms=52.5ms
#define CONN_SUPERVISION_TIMEOUT        60U    //60*10ms=600ms
#define CONN_SLAVE_LATENCY              0U

// Advertising parameters
#define APP_ADV_INTERVAL_MIN            160U    // 160*0.625ms=100ms
#define APP_ADV_INTERVAL_MAX            160U    // 160*0.625ms=100ms

// TRSP setting
#define BLE_MTU_SIZE_MIN                23


// Process mode type definition
typedef enum _Process_Mode
{
    TRSP_PROCESS_GENERAL,               // handle a GATT commands and general control.
    TRSP_PROCESS_IN_PARSING_FINISHED,   // handle multiple GATT commands in "BLECMD_EVENT_ATT_DATABASE_PARSING_FINISHED" event.
} Process_Mode;

// GAP device name
const uint8_t DEVICE_NAME_STR[] = {DEVICE_NAME};


/**************************************************************************
 * Variable
 **************************************************************************/
// Target Peer Device Address
BLE_Addr_Param  targetAddr;

// HRS
uint8_t hrs_data_transmit_enable      = 0;         // 1: transmit hrs data enabled

// TRSP Related variables
uint8_t TRSPX_mtu                     = (BLE_MTU_SIZE_MIN - 3);    // 3 bytes for header, default to set to the minimum, will be updated after MTU exchanged
uint8_t notifyPending                 = 0;                         // 1: indicates that there is data pending, still send old data and skip new data from UART.
uint8_t tx_data_transmit_enable       = 0;                         // 1: indicates there are data to be transmitted
uint8_t txDataLength                  = 0;                         // transmitted data length
uint8_t txDataBuffer[DEFAULT_MTU];                                 // transmitted data buffer

/**************************************************************************
 * Extern Function
 **************************************************************************/
extern void UART_TX_Send(uint32_t len, uint8_t *ptr);         //show data on UART

/**************************************************************************
 * Function Prototype Declaration
 **************************************************************************/
void BleService_GATT_DataInit(BLEATT_GATT_Data *data);
void BleService_HRS_DataInit(BLEATT_HRS_Data *data);
void BleService_UDF01S_DataInit(BLEATT_UDF01S_Data *data);
BleStackStatus Ble_AdvStart(uint8_t hostId);
BleStackStatus Ble_ScanEnable(void);
BleStackStatus Ble_ConnectionCreate(uint8_t hostId, BLE_Addr_Param *peerAddrParam);
static void BleEvent_Callback(BleCmdEvent event, void *param);
static void BleService_UDF01SLink0Handler(uint8_t hostId, uint8_t cmdAccess, uint8_t *data, uint16_t length);
static void BleService_DISLink0Handler(uint8_t hostId, uint8_t cmdAccess, uint8_t *data, uint16_t length);

/**************************************************************************
 * Function
 **************************************************************************/

//Send out RF data
void trspx_send(uint8_t *data, uint16_t len)
{
    int i = 0;

    tx_data_transmit_enable = 1;

    /* if there is data pending, still send old data and skip new data */
    if (notifyPending == 0)
    {
        txDataLength = len;

        //put UART data in txDataBuffer
        for (i = 0; i < len; i++)
        {
            txDataBuffer[i] = data[i];
        }
    }
}

uint8_t targetDevice_nameCheck(uint8_t *name, uint8_t nameLen)
{
    // check device name
    if (((nameLen == strlen((char *)TARGET_DEVICE_NAME)) && (strncmp((char *)name, (char *)TARGET_DEVICE_NAME, nameLen) == 0)))
    {
        return TRUE_DEF;
    }
    return FALSE_DEF;
}



BleStackStatus Ble_TRSPC_MassCmdProcess(BLEProfile_Link0_Info *linkInfo)
{
    // after received "BLECMD_EVENT_ATT_DATABASE_PARSING_FINISHED" event, do
    // 0. MTU exchange
    // 1. data length update
    // 2. set UDF01S cccd value
    // 3. set GATT cccd value
    // 4. read DIS manufacturer name from server
    // 5. read DIS firmware revision from server

    static uint8_t cmd_index = 0;
    static uint8_t runningHostId = BLE_HOSTID_RESERVED;
    BleStackStatus status;

    if ((runningHostId != BLE_HOSTID_RESERVED) && (runningHostId != linkInfo->hostId))
    {
        return BLESTACK_STATUS_ERR_BUSY;
    }

    switch (cmd_index)
    {
    case 0:
        // start running the process
        runningHostId = linkInfo->hostId;

        // send MTU exchange to server
        status = setBLEGATT_ExchangeMtuRequest(linkInfo->hostId, DEFAULT_MTU);
        if (status == BLESTACK_STATUS_ERR_INVALID_CMD)
        {
            // MTU exchange command can be issued only one time, go to next command.
            cmd_index++;
        }
        break;

    case 1:
    {
        BLE_DataLength_Param param;
        param.rxMaxOctets = DEFAULT_MTU + 4; // 4 bytes header
        param.txMaxOctets = DEFAULT_MTU + 4; // 4 bytes header

        // send data length updated to server
        status = setBLE_DataLength(linkInfo->hostId, &param);
    }
    break;

    case 2:
        // set UDF01S cccd
        linkInfo->serviceUDF01S_info_c.data.udatn01_cccd = BLEGATT_CCCD_NOTIFICATION;

        // send config UDF01S cccd to server to enable to receive notifications from server
        status = setBLEGATT_ConfigCCCD(linkInfo->hostId,
                                       linkInfo->serviceUDF01S_info_c.handles.hdl_udatn01_cccd,
                                       linkInfo->serviceUDF01S_info_c.data.udatn01_cccd);
        break;

    case 3:
        // set GATT cccd
        linkInfo->serviceGATT_info_c.data.service_changed_cccd = BLEGATT_CCCD_INDICATION;

        // send config GATT cccd to server to enable to receive indication from server
        status = setBLEGATT_ConfigCCCD(linkInfo->hostId,
                                       linkInfo->serviceGATT_info_c.handles.hdl_service_changed_cccd,
                                       linkInfo->serviceGATT_info_c.data.service_changed_cccd);
        break;

    case 4:
        // get DIS manufacturer name from server
        status = getDIS_ClientDataRead(linkInfo->hostId, linkInfo->serviceDIS_info_c.handles.hdl_manufacturer_name_string);
        break;

    case 5:
        // get DIS firmware revision from server
        status = getDIS_ClientDataRead(linkInfo->hostId, linkInfo->serviceDIS_info_c.handles.hdl_firmware_revision_string);
        break;

    default:
        // Mass process commands are done.
        cmd_index = 0;
        runningHostId = BLE_HOSTID_RESERVED;
        return BLESTACK_STATUS_SUCCESS;
    }

    // if command status is BLESTACK_STATUS_SUCCESS then go to do next command (index++)
    if (status == BLESTACK_STATUS_SUCCESS)
    {
        cmd_index++;
    }

    return BLESTACK_STATUS_ERR_BUSY;
}


void handle_AppLink0_TRSPC(void)
{
    BleStackStatus status;

    if (bleProfile_link0_info.bleState == STATE_BLE_STANDBY)
    {
        // reset service data
        BleService_GATT_DataInit(&bleProfile_link0_info.serviceGATT_info_c.data);
        BleService_UDF01S_DataInit(&bleProfile_link0_info.serviceUDF01S_info_c.data);

        // reset sub-state
        bleProfile_link0_info.subState = TRSP_PROCESS_GENERAL;

        // enable scan
        status = Ble_ScanEnable();
        if (status == BLESTACK_STATUS_SUCCESS)
        {
            bleProfile_link0_info.bleState = STATE_BLE_SCANNING;
        }
    }
    else if (bleProfile_link0_info.bleState == STATE_BLE_INITIATING)
    {
        // create connection
        bleProfile_link0_info.bleState = STATE_BLE_CONN_ESTABLISHING;
        status = Ble_ConnectionCreate(CONN_TRSPC_LINK_HOSTID, &targetAddr);

        if (status != BLESTACK_STATUS_SUCCESS)
        {
            bleProfile_link0_info.bleState = STATE_BLE_STANDBY;
            printf("Create BLE connection failed: %d\n", status);
        }
    }
    else if (bleProfile_link0_info.bleState == STATE_BLE_CONNECTION)
    {
        if (bleProfile_link0_info.subState == TRSP_PROCESS_GENERAL)
        {
            if ((tx_data_transmit_enable == 1) || (notifyPending == 1))
            {
                tx_data_transmit_enable = 0;

                // Send out data by RF
                status = setUDF01S_ClientDataSend(bleProfile_link0_info.hostId,
                                                  BLE_TRSP_WRITE,
                                                  bleProfile_link0_info.serviceUDF01S_info_c.handles.hdl_udatrw01,
                                                  txDataBuffer,
                                                  txDataLength);
                if ( status == BLESTACK_STATUS_SUCCESS)
                {
                    notifyPending = 0;
                }
                else
                {
                    notifyPending = 1;
                }
            }
        }
        else // TRSP_PROCESS_IN_PARSING_FINISHED
        {
            if (Ble_TRSPC_MassCmdProcess(&bleProfile_link0_info) == BLESTACK_STATUS_SUCCESS)
            {
                bleProfile_link0_info.subState = TRSP_PROCESS_GENERAL;  // mass commands process done and back to TRSP_PROCESS_GENERAL
            }
        }
    }
}


void handle_AppLink1_HRSP(void)
{
    BleStackStatus status;

    if (bleProfile_link1_info.bleState == STATE_BLE_STANDBY)
    {
        // reset preferred MTU size
        setBLEGATT_PreferredMtuSize(CONN_HRSP_LINK_HOSTID, DEFAULT_MTU);

        // reset service data
        BleService_GATT_DataInit(&(bleProfile_link1_info.serviceGATT_info_s.data));
        BleService_HRS_DataInit(&bleProfile_link1_info.serviceHRS_info_s.data);

        // enable advertisement
        status = Ble_AdvStart(CONN_HRSP_LINK_HOSTID);
        if (status == BLESTACK_STATUS_SUCCESS)
        {
            bleProfile_link1_info.bleState = STATE_BLE_ADVERTISING;
        }
    }
    else if (bleProfile_link1_info.bleState == STATE_BLE_CONNECTION)
    {
        if ((hrs_data_transmit_enable == 1) && (bleProfile_link1_info.serviceHRS_info_s.data.heart_rate_measurement_cccd != 0))
        {
            hrs_data_transmit_enable = 0;

            // send heart rate measurement value to client
            if ((bleProfile_link1_info.serviceHRS_info_s.data.heart_rate_measurement[0] & BIT1) == 0)    //initial is 0x14 & 0x02 = 0, toggle "device detected" / "device not detected" information
            {
                bleProfile_link1_info.serviceHRS_info_s.data.heart_rate_measurement[0] |= BIT1;          //set Sensor Contact Status bit
            }
            else
            {
                bleProfile_link1_info.serviceHRS_info_s.data.heart_rate_measurement[0] &= ~BIT1;         //clear Sensor Contact Status bit
            }

            if (setBLEGATT_Notification(bleProfile_link1_info.hostId,
                                        bleProfile_link1_info.serviceHRS_info_s.handles.hdl_heart_rate_measurement,
                                        bleProfile_link1_info.serviceHRS_info_s.data.heart_rate_measurement,
                                        sizeof(bleProfile_link1_info.serviceHRS_info_s.data.heart_rate_measurement) / sizeof(bleProfile_link1_info.serviceHRS_info_s.data.heart_rate_measurement[0])
                                       ) == BLESTACK_STATUS_SUCCESS)
            {
                bleProfile_link1_info.serviceHRS_info_s.data.heart_rate_measurement[1]++;  //+1, Heart Rate Data. Here just a simulation, increase 1 about every second
                bleProfile_link1_info.serviceHRS_info_s.data.heart_rate_measurement[2]++;  //+1, Heart Rate RR-Interval
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


BleStackStatus Ble_ScanInit(void)
{
    BleStackStatus status;
    BLE_Scan_Param scanParam;

    scanParam.scanType = SCAN_TYPE;
    scanParam.scanInterval = SCAN_INTERVAL;
    scanParam.scanWindow = SCAN_WINDOW;
    scanParam.scanFilterPolicy = SCAN_FILTER;
    status = setBLE_ScanParam(&scanParam);

    return status;
}


BleStackStatus Ble_ScanEnable(void)
{
    BleStackStatus status;

    status = Ble_ScanInit();
    BLESTACK_STATUS_CHECK(status);

    status = setBLE_ScanEnable();
    BLESTACK_STATUS_CHECK(status);

    return BLESTACK_STATUS_SUCCESS;
}


BleStackStatus Ble_ConnectionCreate(uint8_t hostId, BLE_Addr_Param *peerAddrParam)
{
    BleStackStatus status;
    BLE_Conn_Param connParam;

    // connection parameters
    connParam.connIntervalMin = APP_CONN_INTERVAL_MIN;
    connParam.connIntervalMax = APP_CONN_INTERVAL_MAX;
    connParam.connLatency = CONN_SLAVE_LATENCY;
    connParam.connSupervisionTimeout = CONN_SUPERVISION_TIMEOUT;

    // create connection
    status = setBLE_ConnCreateWithSetScanParam(hostId, peerAddrParam, &connParam);

    return status;
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

void BleService_UDF01S_DataInit(BLEATT_UDF01S_Data *data)
{
    data->udatn01_cccd = 0;
}

BleStackStatus BleApp_ProfileInit(void)
{
    BleStackStatus status;

    //------------------------------------------------------------------------
    // init LINK0 Central GAP/ DIS/ UDF01S services parameter and register callback function
    //------------------------------------------------------------------------
    bleProfile_link0_info.hostId = CONN_TRSPC_LINK_HOSTID;
    bleProfile_link0_info.bleState = STATE_BLE_STANDBY;
    bleProfile_link0_info.subState = TRSP_PROCESS_GENERAL;

    // GAP (Client) Related
    // -------------------------------------
    status = setGAP_ServiceInit(CONN_TRSPC_LINK_HOSTID, BLE_GATT_ROLE_CLIENT, &(bleProfile_link0_info.serviceGAP_info_c), NULL);
    BLESTACK_STATUS_CHECK(status);

    // GATT (Client) Related
    // -------------------------------------
    status = setGATT_ServiceInit(CONN_TRSPC_LINK_HOSTID, BLE_GATT_ROLE_CLIENT, &(bleProfile_link0_info.serviceGATT_info_c), NULL);
    BLESTACK_STATUS_CHECK(status);

    // DIS (Client) Related
    // -------------------------------------
    status = setDIS_ServiceInit(CONN_TRSPC_LINK_HOSTID, BLE_GATT_ROLE_CLIENT, &(bleProfile_link0_info.serviceDIS_info_c), BleService_DISLink0Handler);
    BLESTACK_STATUS_CHECK(status);

    // UDF01S (Client) Related
    // -------------------------------------
    status = setUDF01S_ServiceInit(CONN_TRSPC_LINK_HOSTID, BLE_GATT_ROLE_CLIENT, &(bleProfile_link0_info.serviceUDF01S_info_c), BleService_UDF01SLink0Handler);
    BLESTACK_STATUS_CHECK(status);

    //------------------------------------------------------------------------
    // init LINK1 Peripheral GAP/ DIS/ HRS services parameter and register callback function
    //------------------------------------------------------------------------
    bleProfile_link1_info.hostId = CONN_HRSP_LINK_HOSTID;
    bleProfile_link1_info.bleState = STATE_BLE_STANDBY;
    bleProfile_link1_info.subState = 0x00;

    // GAP (Server) Related
    // -------------------------------------
    status = setGAP_ServiceInit(CONN_HRSP_LINK_HOSTID, BLE_GATT_ROLE_SERVER, &(bleProfile_link1_info.serviceGAP_info_s), NULL);
    BLESTACK_STATUS_CHECK(status);

    // set GAP device name
    status = setGAP_DeviceName((uint8_t *)DEVICE_NAME_STR, sizeof(DEVICE_NAME_STR));
    BLESTACK_STATUS_CHECK(status);

    // set GAP appearance
    setGAP_Appearance(BLE_APPEARANCE_GENERIC_HEART_RATE_SENSOR);

    // GATT (Server) Related
    // -------------------------------------
    status = setGATT_ServiceInit(CONN_HRSP_LINK_HOSTID, BLE_GATT_ROLE_SERVER, &(bleProfile_link1_info.serviceGATT_info_s), NULL);
    BLESTACK_STATUS_CHECK(status);

    // DIS (Server) Related
    // -------------------------------------
    status = setDIS_ServiceInit(CONN_HRSP_LINK_HOSTID, BLE_GATT_ROLE_SERVER, &(bleProfile_link1_info.serviceDIS_info_s), NULL);
    BLESTACK_STATUS_CHECK(status);

    // HRS (Server) Related
    // -------------------------------------
    status = setHRS_ServiceInit(CONN_HRSP_LINK_HOSTID, BLE_GATT_ROLE_SERVER, &(bleProfile_link1_info.serviceHRS_info_s), NULL);
    BLESTACK_STATUS_CHECK(status);

    return BLESTACK_STATUS_SUCCESS;
}


BleStackStatus BleApp_Init(void)
{
    BleStackStatus status = BLESTACK_STATUS_SUCCESS;

    /* set company Id*/
    setBLE_CompanyId(((uint16_t)BLE_COMPANY_ID_H << 8) | BLE_COMPANY_ID_L);

    /* register command event callback function */
    setBLE_RegisterBleEvent(BleEvent_Callback);

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
    // Handle Link0 - TRSP Central
    handle_AppLink0_TRSPC();

    // Handle Link 1 - HRS Peripheral
    handle_AppLink1_HRSP();
} //end of BleApp_Main()


/* BLE Callback Function */
static void BleEvent_Callback(BleCmdEvent event, void *param)
{
    static uint8_t scan_enable = DISABLE_DEF;

    switch (event)
    {
    case BLECMD_EVENT_ADV_COMPLETE:
        printf("Advertising...\n");
        break;

    case BLECMD_EVENT_SCAN_COMPLETE:
        scan_enable = (scan_enable == DISABLE_DEF) ? ENABLE_DEF : DISABLE_DEF;
        if (scan_enable == ENABLE_DEF)
        {
            printf("Scanning...\n");
        }
        else
        {
            // stop scanning -> create connection
            bleProfile_link0_info.bleState = STATE_BLE_INITIATING;
            printf("Connecting...\n");
        }
        break;

    case BLECMD_EVENT_SCAN_REPORT:
    {
        BLE_Event_ScanReportParam *scanRepParam = (BLE_Event_ScanReportParam *)param;
        uint8_t nameStr[31];
        uint8_t nameLen;

        // deviceName
        if ((getBLE_GapAdDataByAdType(scanRepParam, GAP_AD_TYPE_LOCAL_NAME_COMPLETE, nameStr, &nameLen) == BLESTACK_STATUS_SUCCESS) ||
            (getBLE_GapAdDataByAdType(scanRepParam, GAP_AD_TYPE_LOCAL_NAME_SHORTENED, nameStr, &nameLen) == BLESTACK_STATUS_SUCCESS))
        {
            nameStr[nameLen] = '\0';
            printf("Found [Name:%s] [Address: %02x:%02x:%02x:%02x:%02x:%02x]\n",
                   nameStr,
                   scanRepParam->rptPeerAddr.addr[5], scanRepParam->rptPeerAddr.addr[4],
                   scanRepParam->rptPeerAddr.addr[3], scanRepParam->rptPeerAddr.addr[2],
                   scanRepParam->rptPeerAddr.addr[1], scanRepParam->rptPeerAddr.addr[0]);


            if (targetDevice_nameCheck(nameStr, nameLen) == TRUE_DEF)
            {
                printf("Found device and stop scanning...\n");

                // targetAddr
                targetAddr.addrType = scanRepParam->rptPeerAddr.addrType;
                memcpy(targetAddr.addr, scanRepParam->rptPeerAddr.addr, SIZE_BLE_ADDR);

                // disable scan first then create connection
                setBLE_ScanDisable();
            }
        }
    }
    break;

    case BLECMD_EVENT_CONN_COMPLETE:
    {
        BLE_Event_ConnParam *connParam = (BLE_Event_ConnParam *)param;

        // set connection state
        if (connParam->hostId == bleProfile_link0_info.hostId)
        {
            bleProfile_link0_info.bleState = STATE_BLE_CONNECTION;
        }
        else
        {
            bleProfile_link1_info.bleState = STATE_BLE_CONNECTION;
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
        if (disconnParam->hostId == bleProfile_link0_info.hostId)
        {
            bleProfile_link0_info.bleState = STATE_BLE_STANDBY;
        }
        else
        {
            bleProfile_link1_info.bleState = STATE_BLE_STANDBY;
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

    case BLECMD_EVENT_PHY_READ_COMPLETE:
    {
        BLE_Event_PhyParam *phy = (BLE_Event_PhyParam *)param;
        printf("PHY read, ID:%d, TX:%d, RX:%d\n", phy->hostId, phy->phy.txPhy, phy->phy.rxPhy);
    }
    break;

    case BLECMD_EVENT_EXCHANGE_MTU_SIZE:
    {
        BLE_Event_MtuParam *mtuParam = (BLE_Event_MtuParam *)param;
        TRSPX_mtu = ((mtuParam->mtuSize) > 247) ? 244 : (mtuParam->mtuSize - 3); // update to real mtu size, 3 bytes header
        printf("Exchange MTU, ID:%d, size: %d\n", mtuParam->hostId, mtuParam->mtuSize);
    }
    break;

    case BLECMD_EVENT_DATA_LENGTH_UPDATE:
    {
        BLE_Event_DataLengthParam *result = (BLE_Event_DataLengthParam *)param;
        printf("Data Length, Tx=%d, Rx=%d \n", result->dataLenparam.txMaxOctets, result->dataLenparam.rxMaxOctets);
    }
    break;

    case BLECMD_EVENT_ATT_DATABASE_PARSING_FINISHED:
    {
        BLE_Event_AttDbParsed_Param *dbParsingParam = (BLE_Event_AttDbParsed_Param *)param;

        if (dbParsingParam->hostId == CONN_TRSPC_LINK_HOSTID) // LINK 0
        {
            // Get LINK 0 all service handles and related information
            if (getBLELink0_ServiceHandles(CONN_TRSPC_LINK_HOSTID, &bleProfile_link0_info) == BLESTACK_STATUS_SUCCESS)
            {
                // start to do multiple GATT commands first (MTU exchange, configure cccd and get DIS data...)
                bleProfile_link0_info.subState = TRSP_PROCESS_IN_PARSING_FINISHED;
            }
        }
    }
    break;

    default:
        break;
    }
}


static void BleService_UDF01SLink0Handler(uint8_t hostId, uint8_t cmdAccess, uint8_t *data, uint16_t length)
{
    switch (cmdAccess)
    {
    case BLESERVICE_UDF01S_UDATN01_CCCD_WRITE_RSP_EVENT:
        // Ready to receive notification from slave
        printf("cccd write completed!\n");
        break;

    case BLESERVICE_UDF01S_UDATRW01_WRITE_RSP_EVENT:
        break;

    case BLESERVICE_UDF01S_UDATN01_NOTIFY_EVENT:
        // Received data from slave
        UART_TX_Send(length, data);
        break;

    default:
        break;
    }
}


static void BleService_DISLink0Handler(uint8_t hostId, uint8_t cmdAccess, uint8_t *data, uint16_t length)
{
    switch (cmdAccess)
    {
    case BLESERVICE_DIS_MANU_NAME_RR_EVENT:
        printf("Manu Name:");
        UART_TX_Send(length, data);
        break;

    case BLESERVICE_DIS_FIRMWARE_REVISION_RR_EVENT:
        printf("Firmware Rev:");
        UART_TX_Send(length, data);
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
}


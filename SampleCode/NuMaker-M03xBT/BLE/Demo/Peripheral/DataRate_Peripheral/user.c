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
#include <stdbool.h>
#include "BleAppSetting.h"
#include "rf_phy.h"
#include "ble_cmd.h"
#include "ble_event.h"
#include "ble_host.h"
#include "ble_profile.h"
#include "porting_misc.h"

/**************************************************************************
* Macro
**************************************************************************/
#define CHECK_STR(data,target_str) (strncmp((char *)(data), (target_str), sizeof(target_str) - 1) == 0)


/**************************************************************************
* Application Definitions
**************************************************************************/
// LINK please refer to "ble_profile_def.c" --> ATT_DB_MAPPING
#define CONN_DATARATE_LINK_HOSTID       0       // host Id start from 0

// Advertising device name
#define DEVICE_NAME                     'N', 'u', 'v', 'o', 't', 'o', 'n', '_', 'D', 'a', 't', 'a', 'R', 'a', 't', 'e'

// Advertising parameters
#define APP_ADV_INTERVAL_MIN            160U    // 160*0.625ms=100ms
#define APP_ADV_INTERVAL_MAX            160U    // 160*0.625ms=100ms

// GAP device name
const uint8_t DEVICE_NAME_STR[] = {DEVICE_NAME};

// Data rate test total length
#define DATARATE_TEST_LENGTH            1048712


// Data rate cmd identification
#define SET_PARAM_STR                   "set_param"         // Receive from central device, set data rate parameters for the test
#define GET_PARAM_STR                   "get_param"         // Receive from central device, send current data rate parameters for the central device
#define PRX_TEST_STR                    "pRxtest"           // Receive from central device, test C->P data rate
#define PTX_TEST_STR                    "pTxtest"           // Receive from central device, test P->C data rate
#define CANCEL_TEST_STR                 "canceltest"        // Receive from central device, cancel the test case

// Data rate parameter set status
#define STATUS_SET_PARAM                0

// Data rate type definition
typedef enum _RunningMode
{
    DATARATE_MODE_IDLE,
    DATARATE_MODE_RX,
    DATARATE_MODE_TX,
    DATARATE_MODE_SET_PARAM,
    DATARATE_MODE_GET_PARAM
} RunningMode;

typedef struct _DataRateParam
{
    uint8_t      phy;
    uint8_t      packageDataLen;
    uint16_t     connIntervalMin;
    uint16_t     connIntervalMax;
} DataRateParam;

typedef struct _RecordParam
{
    uint8_t      phy;
    uint8_t      packageDataLen;
    uint8_t      mtuSize;
    uint16_t     connInterval;
    uint16_t     connLatency;
    uint16_t     connSupervisionTimeout;
} RecordParam;


/**************************************************************************
 * Variable
**************************************************************************/
RecordParam current_param;                          // record the current param
RunningMode currentMode = DATARATE_MODE_IDLE;       // Initial DataRate mode
DataRateParam dataRateParam;                        // record the setting Data Rate Parameters

uint8_t txDataBuffer[DEFAULT_MTU];                  // Transmit data buffer

uint32_t dataRateTestLen = DATARATE_TEST_LENGTH;    // Total test data length
uint32_t receivedTotalDataLength = 0;               // Current received data length
uint32_t transmittedTotalDataLength = 0;            // Current transmitted data length
uint32_t tmr0cnt = 0;                               // Timer 0 Counter

/**************************************************************************
 * Function Prototype Declaration
 **************************************************************************/
BleStackStatus Ble_AdvStart(uint8_t hostId);
void BleService_UDF01S_DataInit(BLEATT_UDF01S_Data *data);
void BleService_GATT_DataInit(BLEATT_GATT_Data *data);
void BleApp_TxTestRun(void);
void BleApp_DataRateParamSet(void);
void handle_UDF01S_ReadParam(uint8_t hostId, uint16_t hdlNum);
void handle_UDF01S_WriteCommand(uint8_t length, uint8_t *data);
static void BleEvent_Callback(BleCmdEvent event, void *param);
static void BleService_UDF01SLink0Handler(uint8_t hostId, uint8_t cmdAccess, uint8_t *data, uint16_t length);

/**************************************************************************
 * Function
 **************************************************************************/

void BleApp_TimerEnable(uint32_t u32Freq)
{
    /* Enable TIMER module clock */
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    /* Open Timer0 in periodic mode, enable interrupt and 1 interrupt tick per second */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, u32Freq);
    TIMER_EnableInt(TIMER0);

    /* Enable Timer0 NVIC */
    NVIC_SetPriority(TMR0_IRQn, 1);
    NVIC_EnableIRQ(TMR0_IRQn);

    /* Start Timer0 counting */
    TIMER_Start(TIMER0);
}


void BleApp_TimerDisable(void)
{
    TIMER_Close(TIMER0);
}


// notify data to the app through characteristic UDATN01 in service UDF01S
void BleApp_DataRateParamNotifyToClient(uint8_t *data, uint16_t length)
{
    static BleStackStatus status;
    status = setBLEGATT_Notification(bleProfile_link0_info.hostId,
                                     bleProfile_link0_info.serviceUDF01S_info_s.handles.hdl_udatn01,
                                     data,
                                     length);
    if (status != BLESTACK_STATUS_SUCCESS)
    {
        printf("Ble notification failed: %d\n", status);
    }
}

void BleApp_TxTestRun(void)
{
    BleStackStatus status = BLESTACK_STATUS_FREE;
    uint32_t dataLen = current_param.packageDataLen;

    do
    {
        if ((transmittedTotalDataLength + current_param.packageDataLen) > dataRateTestLen)
        {
            dataLen = dataRateTestLen - transmittedTotalDataLength;
        }

        // Send out data by RF
        /* Be careful that Central device should enable NOTIFY, then Peripheral device can start sending notification data */
        status = setBLEGATT_Notification(bleProfile_link0_info.hostId,
                                         bleProfile_link0_info.serviceUDF01S_info_s.handles.hdl_udatn01,
                                         txDataBuffer,
                                         dataLen);

        if (status == BLESTACK_STATUS_SUCCESS)
        {
            transmittedTotalDataLength += dataLen;

            if (transmittedTotalDataLength >= dataRateTestLen)
            {
                printf("Stop TX\n");
                currentMode = DATARATE_MODE_IDLE;
                return;
            }
        }
    }
    while (status == BLESTACK_STATUS_SUCCESS);
}


void BleApp_DataRateParamSet(void)
{
    static BleStackStatus phy_status = BLESTACK_STATUS_SUCCESS;
    static BleStackStatus con_status = BLESTACK_STATUS_SUCCESS;
    static BleStackStatus mtu_status = BLESTACK_STATUS_SUCCESS;
    static BLE_Phy_Param blePhy;
    static BLE_Conn_Param connParam;
    static uint8_t notifyData[100];
    static uint32_t notifyDataLen;

    if (dataRateParam.phy != 0)
    {
        //set phy
        blePhy.txPhy = dataRateParam.phy;
        blePhy.rxPhy = dataRateParam.phy;
        phy_status = setBLE_Phy(bleProfile_link0_info.hostId, &blePhy);
        if (phy_status != BLESTACK_STATUS_SUCCESS)
        {
            printf("Set phy failed: %d\n", phy_status);
        }
    }


    //set TX data size
    if (dataRateParam.packageDataLen != 0)
    {
        if (dataRateParam.packageDataLen > (current_param.mtuSize - 3))
        {
            mtu_status = BLESTACK_STATUS_ERR_INVALID_PARAM;
        }
        else
        {
            current_param.packageDataLen = dataRateParam.packageDataLen;
            mtu_status = BLESTACK_STATUS_SUCCESS;
        }
    }

    if (dataRateParam.connIntervalMax != 0)
    {
        //set con interval
        connParam.connIntervalMax = dataRateParam.connIntervalMax;
        connParam.connIntervalMin = dataRateParam.connIntervalMin;
        connParam.connLatency = 0;
        connParam.connSupervisionTimeout = 600;

        con_status = setBLE_ConnUpdate(bleProfile_link0_info.hostId, &connParam);
        if (con_status != BLESTACK_STATUS_SUCCESS)
        {
            printf("Set connection update failed: %d\n", con_status);
        }
    }

    notifyDataLen = sprintf((char *)notifyData, "%d,%d,%d,%d",
                            STATUS_SET_PARAM, phy_status, mtu_status, con_status);
    BleApp_DataRateParamNotifyToClient(notifyData, notifyDataLen);

    currentMode = DATARATE_MODE_IDLE;
}


void handle_UDF01S_ReadParam(uint8_t hostId, uint16_t hdlNum)
{
    static BleStackStatus status;
    static char readData[100];
    static int readDataLen;

    readDataLen = sprintf((char *)readData, "%d,%d,%d,%d,%d,%d",
                          current_param.phy,
                          current_param.packageDataLen,
                          current_param.mtuSize,
                          current_param.connInterval,
                          current_param.connLatency,
                          current_param.connSupervisionTimeout);
    status = setBLEGATT_GeneralReadRsp(hostId, hdlNum, (uint8_t *)readData, readDataLen);

    if (status != BLESTACK_STATUS_SUCCESS)
    {
        printf("Read RSP error: 0x%02x\n", status);
    }
}

void handle_UDF01S_WriteCommand(uint8_t length, uint8_t *data)
{
    uint32_t dataLen;

    if (CHECK_STR(data, CANCEL_TEST_STR))
    {
        /* Cancel the test */
        currentMode = DATARATE_MODE_IDLE;
        BleApp_TimerDisable();

        printf("Cancel Test.\n");
    }

    if (currentMode == DATARATE_MODE_IDLE)
    {
        if (CHECK_STR(data, PRX_TEST_STR))
        {
            /* Start device RX test */
            currentMode = DATARATE_MODE_RX;
            receivedTotalDataLength = 0;

            BleApp_TimerEnable(1000); // 1ms timer
            tmr0cnt = 0;

            printf("Start RX...\n");

            /* Total test length follows the test string */
            data[length] = 0;
            sscanf((char *)(data + strlen(PRX_TEST_STR)), "%d", &dataLen);
            dataRateTestLen = dataLen;
            printf("dataRateTestLen = %d\n", dataRateTestLen);
        }
        else if (CHECK_STR(data, PTX_TEST_STR))
        {
            /* Start device TX test */
            currentMode = DATARATE_MODE_TX;
            transmittedTotalDataLength = 0;
            printf("Start TX...\n");

            /* Total test length follows the test string */
            data[length] = 0;
            sscanf((char *)(data + strlen(PTX_TEST_STR)), "%d", &dataLen);
            dataRateTestLen = dataLen;
            printf("dataRateTestLen = %d\n", dataRateTestLen);
        }
        else if (CHECK_STR(data, SET_PARAM_STR))
        {
            int state;

            /* Get Data Rate Parameters */
#if defined (__GNUC__)
            char *p_str;
            uint16_t data_array[4];
            uint8_t i;

            i = 0;
            p_str = strtok((char *)data+10, ",");
            while(p_str != NULL)
            {
                state = sscanf((char *)p_str, "%hhu", (unsigned char *)&data_array[i]);
                if (state == -1)
                {
                    printf("the params of set_param cmd is wrong!\n");
                }
                i++;
                p_str = strtok(NULL, ",");
            }
            dataRateParam.phy = data_array[0];
            dataRateParam.packageDataLen = data_array[1];
            dataRateParam.connIntervalMin = data_array[2];
            dataRateParam.connIntervalMax = data_array[3];
#else
            data[length] = 0;
            state = sscanf((char *)data, "set_param=%hhu,%hhu,%hu,%hu",
                           &(dataRateParam.phy),
                           &(dataRateParam.packageDataLen),
                           &(dataRateParam.connIntervalMin),
                           &(dataRateParam.connIntervalMax));
            if (state == -1)
            {
                printf("the params of set_param cmd is wrong!\n");
            }
#endif
            currentMode = DATARATE_MODE_SET_PARAM;
        }
        else if (CHECK_STR(data, GET_PARAM_STR))
        {
            currentMode = DATARATE_MODE_GET_PARAM;
        }
    }
    else if (currentMode == DATARATE_MODE_RX)
    {
        receivedTotalDataLength += length;

        /* RX test done */
        if (receivedTotalDataLength >= dataRateTestLen)
        {
            double throughput;

            BleApp_TimerDisable();
            printf("Stop RX\n");
            currentMode = DATARATE_MODE_IDLE;

            printf("Total Rx Received Time: %d ms\n", tmr0cnt);
            printf("Total Rx Received %d Bytes\n", dataRateTestLen);
            throughput = (double)(dataRateTestLen << 3) / (double)tmr0cnt;
            printf("Rx Through: %.3f bps\n",  throughput * 1000);
        }
    }
}


void handle_AppLink0_DataRate(void)
{
    BleStackStatus status;
    static uint8_t notifyData[100];
    static uint32_t notifyDataLen;

    if (bleProfile_link0_info.bleState == STATE_BLE_STANDBY)
    {
        // reset preferred MTU size
        setBLEGATT_PreferredMtuSize(CONN_DATARATE_LINK_HOSTID, DEFAULT_MTU);

        // reset service data
        BleService_GATT_DataInit(&(bleProfile_link0_info.serviceGATT_info_s.data));
        BleService_UDF01S_DataInit(&(bleProfile_link0_info.serviceUDF01S_info_s.data));

        // enable advertisement
        status = Ble_AdvStart(CONN_DATARATE_LINK_HOSTID);
        if (status == BLESTACK_STATUS_SUCCESS)
        {
            bleProfile_link0_info.bleState = STATE_BLE_ADVERTISING;
        }
    }
    else if (bleProfile_link0_info.bleState == STATE_BLE_CONNECTION)
    {
        if (currentMode == DATARATE_MODE_SET_PARAM)
        {
            BleApp_DataRateParamSet();
        }
        else if (currentMode == DATARATE_MODE_GET_PARAM)
        {
            currentMode = DATARATE_MODE_IDLE;
            notifyDataLen = sprintf((char *)notifyData, "%x,%x,%x,%x,%x,%x",
                                    current_param.phy,
                                    current_param.mtuSize,
                                    current_param.packageDataLen,
                                    current_param.connInterval,
                                    current_param.connLatency,
                                    current_param.connSupervisionTimeout);
            BleApp_DataRateParamNotifyToClient(notifyData, notifyDataLen);
        }
        else if (currentMode == DATARATE_MODE_TX)
        {
            BleApp_TxTestRun();
        }
    }
}

void BleService_GATT_DataInit(BLEATT_GATT_Data *data)
{
    data->service_changed_cccd = 0;
}

void BleService_UDF01S_DataInit(BLEATT_UDF01S_Data *data)
{
    data->udatn01_cccd = 0;
}

BleStackStatus BleApp_ProfileInit(void)
{
    BleStackStatus status;

    //------------------------------------------------------------------------
    // init LINK0 GAP / DIS / UDF01S services parameter and register callback function
    //------------------------------------------------------------------------
    bleProfile_link0_info.hostId = CONN_DATARATE_LINK_HOSTID;
    bleProfile_link0_info.bleState = STATE_BLE_STANDBY;
    bleProfile_link0_info.subState = 0x00;

    // GAP (Server) Related
    // -------------------------------------
    status = setGAP_ServiceInit(CONN_DATARATE_LINK_HOSTID, BLE_GATT_ROLE_SERVER, &(bleProfile_link0_info.serviceGAP_info_s), NULL);
    BLESTACK_STATUS_CHECK(status);

    // set GAP device name
    status = setGAP_DeviceName((uint8_t *)DEVICE_NAME_STR, sizeof(DEVICE_NAME_STR));
    BLESTACK_STATUS_CHECK(status);

    // GATT (Server) Related
    // -------------------------------------
    status = setGATT_ServiceInit(CONN_DATARATE_LINK_HOSTID, BLE_GATT_ROLE_SERVER, &(bleProfile_link0_info.serviceGATT_info_s), NULL);
    BLESTACK_STATUS_CHECK(status);

    // DIS (Server) Related
    // -------------------------------------
    status = setDIS_ServiceInit(CONN_DATARATE_LINK_HOSTID, BLE_GATT_ROLE_SERVER, &(bleProfile_link0_info.serviceDIS_info_s), NULL);
    BLESTACK_STATUS_CHECK(status);

    // UDF01S (Server) Related
    // -------------------------------------
    status = setUDF01S_ServiceInit(CONN_DATARATE_LINK_HOSTID, BLE_GATT_ROLE_SERVER, &(bleProfile_link0_info.serviceUDF01S_info_s), BleService_UDF01SLink0Handler);
    BLESTACK_STATUS_CHECK(status);

    return BLESTACK_STATUS_SUCCESS;
}

BleStackStatus BleApp_Init(void)
{
    BleStackStatus status = BLESTACK_STATUS_SUCCESS;
    int i = 0;

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

    /* Generate TX data */
    for (i = 0; i < DEFAULT_MTU; i++)
    {
        txDataBuffer[i] = i;
    }

    return status;
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
        0x03,   // Length
        GAP_AD_TYPE_LENGTH_2, GAP_AD_TYPE_FLAGS, BLE_GAP_FLAGS_LIMITED_DISCOVERABLE_MODE,   //LE Limit Discoverable Mode, Bluetooth Spec. Ver5.0 [Vol 3], Part C, Section 11
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
    advParam.advType = ADV_TYPE_ADV_IND;                        //ADV Type. Default is ADV_IND;
    advParam.advIntervalMin = APP_ADV_INTERVAL_MIN;             //ADV interval minimum. Default 100ms;
    advParam.advIntervalMax = APP_ADV_INTERVAL_MAX;             //ADV interval maximum. Default 100ms;
    advParam.advChannelMap = ADV_CHANNEL_ALL;                   //ADV channel. Default is all(ch37,38,39);
    advParam.advFilterPolicy = ADV_FILTER_POLICY_ACCEPT_ALL;    //ADV filter policy. Default accept all devices;
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

void BleApp_Main(void)
{
    // Handle Link0 - Data Rate Peripheral
    handle_AppLink0_DataRate();
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
        if (connParam->hostId == CONN_DATARATE_LINK_HOSTID)
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

        if (connParam->status == 0)
        {
            current_param.connInterval = connParam->connInterval;
            current_param.connLatency = connParam->connLatency;
            current_param.connSupervisionTimeout = connParam->connSupervisionTimeout;
            current_param.mtuSize = 23;
            current_param.phy = 1;
            current_param.packageDataLen = (current_param.mtuSize - 3);
        }
    }
    break;

    case BLECMD_EVENT_DISCONN_COMPLETE:
    {
        BLE_Event_DisconnParam *disconnParam = (BLE_Event_DisconnParam *)param;

        // reset state
        if (disconnParam->hostId == CONN_DATARATE_LINK_HOSTID)
        {
            bleProfile_link0_info.bleState = STATE_BLE_STANDBY;
        }
        printf("Disconnected, ID:%d, Reason:0x%X\n", disconnParam->hostId, disconnParam->disconnectReason);
    }
    break;

    case BLECMD_EVENT_CONN_UPDATE_COMPLETE:
    {
        BleStackStatus status;
        BLE_Event_ConnUpdateParam *connUpdateParam  = (BLE_Event_ConnUpdateParam *)param;
        BLE_Conn_Param connParam;

        printf("Connection updated\n");
        printf("Status: %d, ", connUpdateParam->status);
        printf("ID: %d, ", connUpdateParam->hostId);
        printf("Interval: %d, ", connUpdateParam->connInterval);
        printf("Latency: %d, ", connUpdateParam->connLatency);
        printf("Supervision Timeout: %d\n", connUpdateParam->connSupervisionTimeout);

        if (connUpdateParam->status == 0)
        {
            current_param.connInterval = connUpdateParam->connInterval;
            current_param.connLatency = connUpdateParam->connLatency;
            current_param.connSupervisionTimeout = connUpdateParam->connSupervisionTimeout;

            // set connection latency to 0 to confirm the high performance
            if (connUpdateParam->connLatency != 0)
            {
                connParam.connIntervalMin = current_param.connInterval;
                connParam.connIntervalMax = current_param.connInterval;
                connParam.connLatency = 0;
                connParam.connSupervisionTimeout = current_param.connSupervisionTimeout;
                status = setBLE_ConnUpdate(connUpdateParam->hostId, &connParam);
                if (status != BLESTACK_STATUS_SUCCESS)
                {
                    printf("Set connection latency to 0 failed: %d\n", status);
                }
            }
        }
    }
    break;

    case BLECMD_EVENT_PHY_UPDATE_COMPLETE:
    {
        BLE_Event_PhyUpdateParam *phy = (BLE_Event_PhyUpdateParam *)param;
        printf("PHY updated, status: %d, ID: %d, TX PHY: %d, RX PHY: %d\n", phy->status, phy->hostId, phy->phy.txPhy, phy->phy.rxPhy);

        if (phy->status == 0)
        {
            current_param.phy = phy->phy.txPhy;
        }

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
        uint8_t mtuSize = mtuParam->mtuSize;

        printf("Exchange MTU, ID:%d, size: %d\n", mtuParam->hostId, mtuSize);

        current_param.mtuSize = ((mtuSize) > 247) ? 247 : mtuSize;
        current_param.packageDataLen = current_param.mtuSize - 3;
    }
    break;

    case BLECMD_EVENT_STK_GEN_METHOD:
        break;

    case BLECMD_EVENT_PASSKEY_CONFIRM:
        break;

    case BLECMD_EVENT_AUTH_STATUS:
    {
        BLE_Event_AuthStatusParam *auth_result = (BLE_Event_AuthStatusParam *)param;
        printf("AUTH Report, ID:%d , Status:%d\n", auth_result->hostId, auth_result->status);
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
    case BLESERVICE_UDF01S_UDATRW01_WRITE_EVENT:
        // receive data from client --> parsing string to do related test
        handle_UDF01S_WriteCommand(length, data);
        break;

    case BLESERVICE_UDF01S_UDATRW01_READ_EVENT:
        // send current parameters via read response
        handle_UDF01S_ReadParam(hostId, bleProfile_link0_info.serviceUDF01S_info_s.handles.hdl_udatrw01);
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

        /* Increase counter. */
        tmr0cnt++;
    }
}


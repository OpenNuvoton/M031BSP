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
#include "porting_rfpower.h"
#include "atcmd.h"

/**************************************************************************
* Application Definitions
**************************************************************************/
// LINK please refer to "ble_profile_def.c" --> ATT_DB_MAPPING
#define CONN_TRSP_LINK_HOSTID           0       // host Id start from 0

// Advertising device name
#define DEVICE_NAME                    'N', 'u', 'v', 'o', 't', 'o', 'n', '_', 'A', 'T', 'C', 'M', 'D'

// Advertising parameters
#define APP_ADV_INTERVAL_MIN            160U    // 160*0.625ms=100ms
#define APP_ADV_INTERVAL_MAX            160U    // 160*0.625ms=100ms

// GAP device name
const uint8_t DEVICE_NAME_STR[] = {DEVICE_NAME};

// TRSP setting
#define BLE_MTU_SIZE_MIN                23

/**************************************************************************
 * Variable
 **************************************************************************/
uint8_t TRSPX_mtu                     = (BLE_MTU_SIZE_MIN - 3);    // 3 bytes for header, default to set to the minimum, will be updated after MTU exchanged
uint8_t notifyPending                 = 0;                         // 1: indicates that there is data pending, still send old data and skip new data from UART.
uint8_t tx_data_transmit_enable       = 0;                         // 1: indicates there are data to be transmitted
uint8_t txDataLength                  = 0;                         // transmitted data length
uint8_t txDataBuffer[DEFAULT_MTU];                                 // transmitted data buffer

BLE_Event_ConnUpdateParam ble_conn_update;
uint16_t ble_adv_interval = ADV_INTERVAL_MIN;
uint8_t ble_phy = 1;                                               // bandwidth of the BLE phy
uint8_t standby_reset = 1;                                         // flag of reset BLE data in standby mode

/**************************************************************************
 * Extern Function
 **************************************************************************/
extern void UART_TX_Send(uint32_t len, uint8_t *ptr);         //show data on UART

/**************************************************************************
 * Function Prototype Declaration
 **************************************************************************/
BleStackStatus Ble_AdvStart(uint8_t hostId);
void BleService_GATT_DataInit(BLEATT_GATT_Data *data);
void BleService_UDF01S_DataInit(BLEATT_UDF01S_Data *data);
static void BleEvent_Callback(BleCmdEvent event, void *param);
static void BleService_UDF01SLink0Handler(uint8_t hostId, uint8_t cmdAccess, uint8_t *data, uint16_t length);

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


void handle_AppLink0_TRSPP(void)
{
    BleStackStatus status;

    if (bleProfile_link0_info.bleState == STATE_BLE_STANDBY)
    {
        if (standby_reset == 1)
        {
            standby_reset = 0;

            // reset preferred MTU size
            setBLEGATT_PreferredMtuSize(CONN_TRSP_LINK_HOSTID, DEFAULT_MTU);

            // reset service data
            BleService_GATT_DataInit(&(bleProfile_link0_info.serviceGATT_info_s.data));
            BleService_UDF01S_DataInit(&(bleProfile_link0_info.serviceUDF01S_info_s.data));
        }
    }
    else if (bleProfile_link0_info.bleState == STATE_BLE_CONNECTION)
    {
        if ((tx_data_transmit_enable == 1) || (notifyPending == 1))
        {
            tx_data_transmit_enable = 0;

            // Send out data by RF
            /* Be careful that the client should enable NOTIFY, then the server can start sending notification data */
            status = setUDF01S_ServerDataSend(bleProfile_link0_info.hostId,
                                              bleProfile_link0_info.serviceUDF01S_info_s.data.udatn01_cccd,
                                              bleProfile_link0_info.serviceUDF01S_info_s.handles.hdl_udatn01,
                                              txDataBuffer,
                                              txDataLength);
            if (status == BLESTACK_STATUS_SUCCESS)
            {
                notifyPending = 0;
            }
            else
            {
                notifyPending = 1;
            }
        }
    }
}

BleStackStatus setBLE_AdvInterval(uint16_t interval_min, uint16_t interval_max)
{
    BleStackStatus status;
    BLE_Adv_Param advParam;

    // Adv. parameter init
    advParam.advType = ADV_TYPE_ADV_IND;                        //ADV Type. Default is ADV_IND;
    advParam.advIntervalMin = interval_min;                     //ADV interval minimum. Default 100ms;
    advParam.advIntervalMax = interval_max;                     //ADV interval maximum. Default 100ms;
    advParam.advChannelMap = ADV_CHANNEL_ALL;                   //ADV channel. Default is all(ch37,38,39);
    advParam.advFilterPolicy = ADV_FILTER_POLICY_ACCEPT_ALL;    //ADV filter policy. Default accept all devices;

    status = setBLE_AdvParam(&advParam);
    debug_printf("setBLE_AdvParam() = %d\n", status);
    if (status == BLESTACK_STATUS_SUCCESS)
        ble_adv_interval = interval_min;

    return status;
}

BleStackStatus setBLE_ScanName(char *name)
{
    BleStackStatus status;
    uint32_t name_size;

    name_size = strlen(name);
    if (name_size > 20)
    {
        debug_printf("Device name size is too large!!\n");
        status = BLESTACK_STATUS_ERR_INVALID_PARAM;
    }
    else
    {
        memset(ble_scan_name, 0x0, sizeof(ble_scan_name));
        ble_scan_name[0] = name_size + 2;                                       //Length: 1byte;
        ble_scan_name[1] = name_size + 1;                                       //AD length,
        ble_scan_name[2] = GAP_AD_TYPE_LOCAL_NAME_COMPLETE;                     //AD Data: 1st byte
        strcpy((char *)&ble_scan_name[3], name);                                //AD Data: other bytes
        status = setBLE_ScanRspData((uint8_t *)ble_scan_name, name_size+3);     //ble_scan_name can be modified by user
        debug_printf("setBLE_ScanRspData() = %d\n", status);
    }

    return status;
}

void Ble_Slave_Init(void)
{
    BleMode ble_mode;
    uint8_t i;

    //Adv data format. Set according to user select profile. See Bluetooth Spec. Ver5.0 [Vol 3], Part C, Section 11
    uint8_t advData[] =
    {
        0x03,   // Length
        GAP_AD_TYPE_LENGTH_2, GAP_AD_TYPE_FLAGS, BLE_GAP_FLAGS_LIMITED_DISCOVERABLE_MODE,   //LE Limit Discoverable Mode, Bluetooth Spec. Ver5.0 [Vol 3], Part C, Section 11
    };

    //Set advertising interval
    setBLE_AdvInterval(ble_adv_interval, ble_adv_interval);

    //Set advertising data format
    setBLE_AdvData((uint8_t *)advData, sizeof(advData));

    //Set scan response name
    setBLE_ScanRspData((uint8_t *)ble_scan_name, strlen((char *)ble_scan_name));

    /* Set TX power */
    for (i = 0; i < 3; i++)
    {
        if (ble_tx_power[i] != ble_tx_power_def[i])
        {
            if (i == 0)
                ble_mode = STATE_BLE_ADVERTISING;
            else if (i == 1)
                ble_mode = STATE_BLE_SCANNING;
            else
                ble_mode = STATE_BLE_INITIATING;

            setBLE_TxPower_Wrap(ble_tx_power[i], ble_mode);
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
    // init LINK0 GAP/ DIS/ UDF01S services parameter and register callback function
    //------------------------------------------------------------------------
    bleProfile_link0_info.hostId = CONN_TRSP_LINK_HOSTID;
    bleProfile_link0_info.bleState = STATE_BLE_STANDBY;
    bleProfile_link0_info.subState = 0x00;

    // GAP (Server) Related
    // -------------------------------------
    status = setGAP_ServiceInit(CONN_TRSP_LINK_HOSTID, BLE_GATT_ROLE_SERVER, &(bleProfile_link0_info.serviceGAP_info_s), NULL);
    BLESTACK_STATUS_CHECK(status);

    // set GAP device name
    status = setGAP_DeviceName((uint8_t *)DEVICE_NAME_STR, sizeof(DEVICE_NAME_STR));
    BLESTACK_STATUS_CHECK(status);

    // GATT (Server) Related
    // -------------------------------------
    status = setGATT_ServiceInit(CONN_TRSP_LINK_HOSTID, BLE_GATT_ROLE_SERVER, &(bleProfile_link0_info.serviceGATT_info_s), NULL);
    BLESTACK_STATUS_CHECK(status);

    // DIS (Server) Related
    // -------------------------------------
    status = setDIS_ServiceInit(CONN_TRSP_LINK_HOSTID, BLE_GATT_ROLE_SERVER, &(bleProfile_link0_info.serviceDIS_info_s), NULL);
    BLESTACK_STATUS_CHECK(status);

    // UDF01S (Server) Related
    // -------------------------------------
    status = setUDF01S_ServiceInit(CONN_TRSP_LINK_HOSTID, BLE_GATT_ROLE_SERVER, &(bleProfile_link0_info.serviceUDF01S_info_s), BleService_UDF01SLink0Handler);
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
    else
    {
        /* Set BLE settings */
        Ble_Slave_Init();
    }

    return status;
}

void BleApp_Main(void)
{
    // Handle Link0 - TRSP Peripheral
    handle_AppLink0_TRSPP();
} //end of BleApp_Main()


/* BLE Callback Function */
static void BleEvent_Callback(BleCmdEvent event, void *param)
{
    BleStackStatus status = BLESTACK_STATUS_FREE;

    switch (event)
    {
    case BLECMD_EVENT_ADV_COMPLETE:
    {
        if (bleProfile_link0_info.bleState == STATE_BLE_ADVERTISING)
        {
            standby_reset = 1;
            debug_printf("Start advertising...\n");
        }
        else if (bleProfile_link0_info.bleState == STATE_BLE_STANDBY)
        {
            debug_printf("Stop advertising...\n");
        }
        status = BLESTACK_STATUS_SUCCESS;
    }
    break;

    case BLECMD_EVENT_CONN_COMPLETE:
    {
        BLE_Event_ConnParam *connParam = (BLE_Event_ConnParam *)param;

        // set connection state
        if (connParam->hostId == CONN_TRSP_LINK_HOSTID)
        {
            bleProfile_link0_info.bleState = STATE_BLE_CONNECTION;
        }
        ble_conn_update.hostId = connParam->hostId;
        ble_conn_update.connInterval = connParam->connInterval;
        ble_conn_update.connLatency = connParam->connLatency;
        ble_conn_update.connSupervisionTimeout = connParam->connSupervisionTimeout;
        debug_printf("Status=%d, ID=%d, Connected to %02x:%02x:%02x:%02x:%02x:%02x\n",
                     connParam->status,
                     connParam->hostId,
                     connParam->peerAddr.addr[5],
                     connParam->peerAddr.addr[4],
                     connParam->peerAddr.addr[3],
                     connParam->peerAddr.addr[2],
                     connParam->peerAddr.addr[1],
                     connParam->peerAddr.addr[0]);
        if (connParam->status == 0)
            printf("CONNECTED\n");
    }
    break;

    case BLECMD_EVENT_DISCONN_COMPLETE:
    {
        BLE_Event_DisconnParam *disconnParam = (BLE_Event_DisconnParam *)param;

        debug_printf("Disconnected, ID:%d, Reason:0x%X\n", disconnParam->hostId, disconnParam->disconnectReason);
        printf("DISCONNECTED\n");

        /* Return to advertising after disconnect */
        if (disconnParam->hostId == CONN_TRSP_LINK_HOSTID)
        {
            status = setBLE_AdvEnable(disconnParam->hostId);
            debug_printf("setBLE_AdvEnable(%d) = %d\n", disconnParam->hostId, status);
            if (status == BLESTACK_STATUS_SUCCESS)
            {
                bleProfile_link0_info.bleState = STATE_BLE_ADVERTISING;
            }
            else
            {
                bleProfile_link0_info.bleState = STATE_BLE_STANDBY;
            }
        }
    }
    break;

    case BLECMD_EVENT_CONN_UPDATE_COMPLETE:
    {
        BLE_Event_ConnUpdateParam *connUpdateParam  = (BLE_Event_ConnUpdateParam *)param;

        debug_printf("Connection updated\n");
        debug_printf("Status: %d, ", connUpdateParam->status);
        debug_printf("ID: %d, ", connUpdateParam->hostId);
        debug_printf("Interval: %d, ", connUpdateParam->connInterval);
        debug_printf("Latency: %d, ", connUpdateParam->connLatency);
        debug_printf("Supervision Timeout: %d\n", connUpdateParam->connSupervisionTimeout);
        if (connUpdateParam->status == CMD_SUCCESS)
        {
            ble_conn_update.connInterval = connUpdateParam->connInterval;
            ble_conn_update.connLatency = connUpdateParam->connLatency;
            ble_conn_update.connSupervisionTimeout = connUpdateParam->connSupervisionTimeout;
            status = BLESTACK_STATUS_SUCCESS;
        }
        else
        {
            status = BLESTACK_STATUS_ERR_BUSY;
        }
    }
    break;

    case BLECMD_EVENT_PHY_UPDATE_COMPLETE:
    {
        BLE_Event_PhyUpdateParam *phy = (BLE_Event_PhyUpdateParam *)param;
        debug_printf("PHY updated, status: %d, ID: %d, TX PHY: %d, RX PHY: %d\n", phy->status, phy->hostId, phy->phy.txPhy, phy->phy.rxPhy);
        if (phy->status == BLESTACK_STATUS_SUCCESS)
        {
            ble_phy = phy->phy.txPhy;
            status = BLESTACK_STATUS_SUCCESS;
        }
        else
        {
            status = BLESTACK_STATUS_ERR_BUSY;
        }
    }
    break;

    case BLECMD_EVENT_PHY_READ_COMPLETE:
    {
        BLE_Event_PhyParam *phy = (BLE_Event_PhyParam *)param;
        debug_printf("PHY read, ID:%d, TX:%d, RX:%d\n", phy->hostId, phy->phy.txPhy, phy->phy.rxPhy);
        ble_phy = phy->phy.txPhy;
        printf("+PHY:%d\r\n", ble_phy);
    }
    break;

    case BLECMD_EVENT_EXCHANGE_MTU_SIZE:
    {
        BLE_Event_MtuParam *mtuParam = (BLE_Event_MtuParam *)param;
        TRSPX_mtu = ((mtuParam->mtuSize) > 247) ? 244 : (mtuParam->mtuSize - 3); // update to real mtu size, 3 bytes header
        debug_printf("Exchange MTU, ID:%d, size: %d\n", mtuParam->hostId, mtuParam->mtuSize);
    }
    break;

    default:
        break;
    }

    /* AT command is finished when receive event */
    if (atcmd_state == AT_STATE_WAIT)
    {
        if (status == BLESTACK_STATUS_SUCCESS)
            printf("OK\r\n");
        else if (status == BLESTACK_STATUS_ERR_BUSY)
            printf("ERROR\r\n");

        atcmd_state = AT_STATE_FREE;
    }
}


static void BleService_UDF01SLink0Handler(uint8_t hostId, uint8_t cmdAccess, uint8_t *data, uint16_t length)
{
    switch (cmdAccess)
    {
    case BLESERVICE_UDF01S_UDATRW01_WRITE_EVENT:
    {
        //show received RF data on UART
        UART_TX_Send(length, data);
    }
    break;

    case BLESERVICE_UDF01S_UDATRW01_READ_EVENT:
    {
        //send read rsp with const read data to client
        uint8_t readData[] = "UDATRW01 data";
        setBLEGATT_GeneralReadRsp(hostId, bleProfile_link0_info.serviceUDF01S_info_s.handles.hdl_udatrw01, (uint8_t *)readData, (SIZE_STRING(readData)));
    }
    break;

    default:
        break;
    }
}


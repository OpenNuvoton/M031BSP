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

// TRSP setting
#define BLE_MTU_SIZE_MIN                23


// Process mode type definition
typedef enum _Process_Mode
{
    TRSP_PROCESS_GENERAL,               // handle a GATT commands and general control.
    TRSP_PROCESS_IN_PARSING_FINISHED,   // handle multiple GATT commands in "BLECMD_EVENT_ATT_DATABASE_PARSING_FINISHED" event.
} Process_Mode;

/**************************************************************************
 * Variable
 **************************************************************************/
// Active scan host ID
int8_t  activeScanHostId = -1;

// Target Peer Device Address
BLE_Addr_Param  targetAddr;

// TRSP Related variables
uint8_t TRSPX_mtu       = (BLE_MTU_SIZE_MIN - 3);                  // 3 bytes for header, default to set to the minimum, will be updated after MTU exchanged
uint8_t notifyPending[BLE_SUPPORT_NUM_CONN_MAX];                   // 1: indicates that there is data pending, still send old data and skip new data from UART.
uint8_t tx_data_transmit_enable[BLE_SUPPORT_NUM_CONN_MAX];         // 1: indicates there are data to be transmitted
uint8_t txDataLength[BLE_SUPPORT_NUM_CONN_MAX];                    // transmitted data length
uint8_t txDataBuffer[BLE_SUPPORT_NUM_CONN_MAX][DEFAULT_MTU];       // transmitted data buffer

/**************************************************************************
 * Extern Function
 **************************************************************************/
extern void UART_TX_Send(uint32_t len, uint8_t *ptr);         //show data on UART

/**************************************************************************
 * Function Prototype Declaration
 **************************************************************************/
void BleService_GATT_DataInit(BLEATT_GATT_Data *data);
void BleService_UDF01S_DataInit(BLEATT_UDF01S_Data *data);
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
    int i, j;

    for (i = 0; i < BLE_SUPPORT_NUM_CONN_MAX; i++)
    {
        if (bleProfile_link0_info[i].bleState == STATE_BLE_CONNECTION)
        {
            tx_data_transmit_enable[i] = 1;

            /* if there is data pending, still send old data and skip new data */
            if (notifyPending[i] == 0)
            {
                txDataLength[i] = len;

                //put UART data in txDataBuffer
                for (j = 0; j < len; j++)
                {
                    txDataBuffer[i][j] = data[j];
                }
            }
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


void handle_AppLink0_TRSPC(BLEProfile_Link0_Info *linkInfo)
{
    BleStackStatus status;

    if (linkInfo->bleState == STATE_BLE_STANDBY)
    {
        if (activeScanHostId == -1)
        {
            // reset service data
            BleService_GATT_DataInit(&linkInfo->serviceGATT_info_c.data);
            BleService_UDF01S_DataInit(&linkInfo->serviceUDF01S_info_c.data);

            // reset sub-state
            linkInfo->subState = TRSP_PROCESS_GENERAL;

            // enable scan
            status = Ble_ScanEnable();
            if (status == BLESTACK_STATUS_SUCCESS)
            {
                linkInfo->bleState = STATE_BLE_SCANNING;
            }

            activeScanHostId = linkInfo->hostId;
        }
    }
    else if (linkInfo->bleState == STATE_BLE_INITIATING)
    {
        // create connection
        linkInfo->bleState = STATE_BLE_CONN_ESTABLISHING;
        status = Ble_ConnectionCreate(linkInfo->hostId, &targetAddr);

        if (status != BLESTACK_STATUS_SUCCESS)
        {
            linkInfo->bleState = STATE_BLE_STANDBY;
            printf("Create BLE connection failed: %d\n", status);
        }
    }
    else if (linkInfo->bleState == STATE_BLE_CONNECTION)
    {
        if (linkInfo->subState == TRSP_PROCESS_GENERAL)
        {
            if ((tx_data_transmit_enable[linkInfo->hostId] == 1) || (notifyPending[linkInfo->hostId] == 1))
            {
                tx_data_transmit_enable[linkInfo->hostId] = 0;

                // Send out data by RF
                status = setUDF01S_ClientDataSend(linkInfo->hostId,
                                                  BLE_TRSP_WRITE,
                                                  linkInfo->serviceUDF01S_info_c.handles.hdl_udatrw01,
                                                  txDataBuffer[linkInfo->hostId],
                                                  txDataLength[linkInfo->hostId]);
                if ( status == BLESTACK_STATUS_SUCCESS)
                {
                    notifyPending[linkInfo->hostId] = 0;
                }
                else
                {
                    notifyPending[linkInfo->hostId] = 1;
                }
            }
        }
        else // TRSP_PROCESS_IN_PARSING_FINISHED
        {
            if (Ble_TRSPC_MassCmdProcess(linkInfo) == BLESTACK_STATUS_SUCCESS)
            {
                linkInfo->subState = TRSP_PROCESS_GENERAL;  // mass commands process done and back to TRSP_PROCESS_GENERAL
            }
        }
    }
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

void BleService_UDF01S_DataInit(BLEATT_UDF01S_Data *data)
{
    data->udatn01_cccd = 0;
}

BleStackStatus BleApp_ProfileInit(void)
{
    BleStackStatus status;
    uint8_t i;

    for (i = 0; i < BLE_SUPPORT_NUM_CONN_MAX; i++)
    {
        //------------------------------------------------------------------------
        // init LINK0 GAP/ DIS/ UDF01S services parameter and register callback function
        //------------------------------------------------------------------------
        bleProfile_link0_info[i].hostId = i;
        bleProfile_link0_info[i].bleState = STATE_BLE_STANDBY;
        bleProfile_link0_info[i].subState = TRSP_PROCESS_GENERAL;

        // GAP (Client) Related
        // -------------------------------------
        status = setGAP_ServiceInit(i, BLE_GATT_ROLE_CLIENT, &(bleProfile_link0_info[i].serviceGAP_info_c), NULL);
        BLESTACK_STATUS_CHECK(status);

        // GATT (Client) Related
        // -------------------------------------
        status = setGATT_ServiceInit(i, BLE_GATT_ROLE_CLIENT, &(bleProfile_link0_info[i].serviceGATT_info_c), NULL);
        BLESTACK_STATUS_CHECK(status);

        // DIS (Client) Related
        // -------------------------------------
        status = setDIS_ServiceInit(i, BLE_GATT_ROLE_CLIENT, &(bleProfile_link0_info[i].serviceDIS_info_c), BleService_DISLink0Handler);
        BLESTACK_STATUS_CHECK(status);

        // UDF01S (Client) Related
        // -------------------------------------
        status = setUDF01S_ServiceInit(i, BLE_GATT_ROLE_CLIENT, &(bleProfile_link0_info[i].serviceUDF01S_info_c), BleService_UDF01SLink0Handler);
        BLESTACK_STATUS_CHECK(status);
    }

    return BLESTACK_STATUS_SUCCESS;
}


BleStackStatus BleApp_Init(void)
{
    BleStackStatus status = BLESTACK_STATUS_SUCCESS;
    uint8_t i;

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

    for (i = 0; i < BLE_SUPPORT_NUM_CONN_MAX; i++)
    {
        tx_data_transmit_enable[i] = 0;
        notifyPending[i] = 0;
    }

    return status;
}

void BleApp_Main(void)
{
    uint8_t i;

    for (i = 0; i < BLE_SUPPORT_NUM_CONN_MAX; i++)
    {
        // Handle Link0 - TRSP Central
        handle_AppLink0_TRSPC(&bleProfile_link0_info[i]);
    }
} //end of BleApp_Main()


/* BLE Callback Function */
static void BleEvent_Callback(BleCmdEvent event, void *param)
{
    static uint8_t scan_enable = DISABLE_DEF;

    switch (event)
    {
    case BLECMD_EVENT_SCAN_COMPLETE:
        scan_enable = (scan_enable == DISABLE_DEF) ? ENABLE_DEF : DISABLE_DEF;
        if (scan_enable == ENABLE_DEF)
        {
            printf("[%d] Scanning...\n", activeScanHostId);
        }
        else
        {
            // stop scanning -> create connection
            bleProfile_link0_info[activeScanHostId].bleState = STATE_BLE_INITIATING;
            printf("[%d] Connecting...\n", activeScanHostId);
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

            if (targetDevice_nameCheck(nameStr, nameLen) == TRUE_DEF)
            {
                printf("[%d] Found [Name:%s] [Address: %02x:%02x:%02x:%02x:%02x:%02x]\n",
                       activeScanHostId,
                       nameStr,
                       scanRepParam->rptPeerAddr.addr[5], scanRepParam->rptPeerAddr.addr[4],
                       scanRepParam->rptPeerAddr.addr[3], scanRepParam->rptPeerAddr.addr[2],
                       scanRepParam->rptPeerAddr.addr[1], scanRepParam->rptPeerAddr.addr[0]);
                printf("[%d] Stop scanning...\n", activeScanHostId);

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
        bleProfile_link0_info[connParam->hostId].bleState = STATE_BLE_CONNECTION;

        printf("[%d] Status=%d, Connected to %02x:%02x:%02x:%02x:%02x:%02x\n",
               connParam->hostId,
               connParam->status,
               connParam->peerAddr.addr[5],
               connParam->peerAddr.addr[4],
               connParam->peerAddr.addr[3],
               connParam->peerAddr.addr[2],
               connParam->peerAddr.addr[1],
               connParam->peerAddr.addr[0]);

        // previous scan and initial actions are done, we can start new scan action
        activeScanHostId = -1;
    }
    break;

    case BLECMD_EVENT_DISCONN_COMPLETE:
    {
        BLE_Event_DisconnParam *disconnParam = (BLE_Event_DisconnParam *)param;

        // reset state
        bleProfile_link0_info[disconnParam->hostId].bleState = STATE_BLE_STANDBY;

        printf("[%d] Disconnected, Reason:0x%X\n", disconnParam->hostId, disconnParam->disconnectReason);
    }
    break;

    case BLECMD_EVENT_CONN_UPDATE_COMPLETE:
    {
        BLE_Event_ConnUpdateParam *connUpdateParam  = (BLE_Event_ConnUpdateParam *)param;

        printf("[%d] Connection updated\n", connUpdateParam->hostId);
        printf("Status: %d, ", connUpdateParam->status);
        printf("Interval: %d, ", connUpdateParam->connInterval);
        printf("Latency: %d, ", connUpdateParam->connLatency);
        printf("Supervision Timeout: %d\n", connUpdateParam->connSupervisionTimeout);
    }
    break;

    case BLECMD_EVENT_PHY_UPDATE_COMPLETE:
    {
        BLE_Event_PhyUpdateParam *phy = (BLE_Event_PhyUpdateParam *)param;
        printf("[%d] PHY updated, status: %d, TX PHY: %d, RX PHY: %d\n", phy->hostId, phy->status, phy->phy.txPhy, phy->phy.rxPhy);
    }
    break;

    case BLECMD_EVENT_PHY_READ_COMPLETE:
    {
        BLE_Event_PhyParam *phy = (BLE_Event_PhyParam *)param;
        printf("[%d] PHY read, TX:%d, RX:%d\n", phy->hostId, phy->phy.txPhy, phy->phy.rxPhy);
    }
    break;

    case BLECMD_EVENT_EXCHANGE_MTU_SIZE:
    {
        BLE_Event_MtuParam *mtuParam = (BLE_Event_MtuParam *)param;
        TRSPX_mtu = ((mtuParam->mtuSize) > 247) ? 244 : (mtuParam->mtuSize - 3); // update to real mtu size, 3 bytes header
        printf("[%d] Exchange MTU, size: %d\n", mtuParam->hostId, mtuParam->mtuSize);
    }
    break;

    case BLECMD_EVENT_DATA_LENGTH_UPDATE:
    {
        BLE_Event_DataLengthParam *result = (BLE_Event_DataLengthParam *)param;
        printf("[%d] Data Length, Tx=%d, Rx=%d \n", result->hostId, result->dataLenparam.txMaxOctets, result->dataLenparam.rxMaxOctets);
    }
    break;

    case BLECMD_EVENT_ATT_DATABASE_PARSING_FINISHED:
    {
        BLE_Event_AttDbParsed_Param *dbParsingParam = (BLE_Event_AttDbParsed_Param *)param;

        // Get LINK all service handles and related information
        if (getBLELink0_ServiceHandles(dbParsingParam->hostId, &bleProfile_link0_info[dbParsingParam->hostId]) == BLESTACK_STATUS_SUCCESS)
        {
            // start to do multiple GATT commands first (MTU exchange, configure cccd and get DIS data...)
            bleProfile_link0_info[dbParsingParam->hostId].subState = TRSP_PROCESS_IN_PARSING_FINISHED;
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
        printf("[%d] cccd write completed!\n", hostId);
        break;

    case BLESERVICE_UDF01S_UDATRW01_WRITE_RSP_EVENT:
        break;

    case BLESERVICE_UDF01S_UDATN01_NOTIFY_EVENT:
        // Received data from slave
        printf("[%d] ", hostId);
        UART_TX_Send(length, data);
        break;

    case BLESERVICE_UDF01S_UDATN01_CCCD_READ_RSP_EVENT:
        // Received data from slave
        printf("[%d] cccd = 0x%02x%02x\n", hostId, data[1], data[0]);
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
        printf("[%d] Manu Name:", hostId);
        UART_TX_Send(length, data);
        break;

    case BLESERVICE_DIS_FIRMWARE_REVISION_RR_EVENT:
        printf("[%d] Firmware Rev:", hostId);
        UART_TX_Send(length, data);
        break;

    default:
        break;
    }
}


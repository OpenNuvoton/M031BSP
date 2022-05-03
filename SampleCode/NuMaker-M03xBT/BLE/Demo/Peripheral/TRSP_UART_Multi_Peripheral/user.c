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
// Active advertising host ID
int8_t  activeAdvHostId = -1;

// Advertising device name
#define DEVICE_NAME                    'N', 'u', 'v', 'o', 't', 'o', 'n', '_', 'U', 'A', 'R', 'T'

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


void handle_AppLink0_TRSPP(BLEProfile_Link0_Info *linkInfo)
{
    BleStackStatus status;

    if (linkInfo->bleState == STATE_BLE_STANDBY)
    {
        if (activeAdvHostId == -1)
        {
            // reset preferred MTU size
            setBLEGATT_PreferredMtuSize(linkInfo->hostId, DEFAULT_MTU);

            // reset service data
            BleService_GATT_DataInit(&(linkInfo->serviceGATT_info_s.data));
            BleService_UDF01S_DataInit(&(linkInfo->serviceUDF01S_info_s.data));

            // enable advertisement
            status = Ble_AdvStart(linkInfo->hostId);
            if (status == BLESTACK_STATUS_SUCCESS)
            {
                linkInfo->bleState = STATE_BLE_ADVERTISING;
            }

            activeAdvHostId = linkInfo->hostId;
        }
    }
    else if (linkInfo->bleState == STATE_BLE_CONNECTION)
    {
        if ((tx_data_transmit_enable[linkInfo->hostId] == 1) || (notifyPending[linkInfo->hostId] == 1))
        {
            tx_data_transmit_enable[linkInfo->hostId] = 0;

            // Send out data by RF
            /* Be careful that the client should enable NOTIFY, then the server can start sending notification data */
            status = setUDF01S_ServerDataSend(linkInfo->hostId,
                                              linkInfo->serviceUDF01S_info_s.data.udatn01_cccd,
                                              linkInfo->serviceUDF01S_info_s.handles.hdl_udatn01,
                                              txDataBuffer[linkInfo->hostId],
                                              txDataLength[linkInfo->hostId]);
            if (status == BLESTACK_STATUS_SUCCESS)
            {
                notifyPending[linkInfo->hostId] = 0;
            }
            else
            {
                notifyPending[linkInfo->hostId] = 1;
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
        bleProfile_link0_info[i].subState = 0x00;

        // GAP (Server) Related
        // -------------------------------------
        status = setGAP_ServiceInit(i, BLE_GATT_ROLE_SERVER, &(bleProfile_link0_info[i].serviceGAP_info_s), NULL);
        BLESTACK_STATUS_CHECK(status);

        // set GAP device name
        status = setGAP_DeviceName((uint8_t *)DEVICE_NAME_STR, sizeof(DEVICE_NAME_STR));
        BLESTACK_STATUS_CHECK(status);

        // GATT (Server) Related
        // -------------------------------------
        status = setGATT_ServiceInit(i, BLE_GATT_ROLE_SERVER, &(bleProfile_link0_info[i].serviceGATT_info_s), NULL);
        BLESTACK_STATUS_CHECK(status);

        // DIS (Server) Related
        // -------------------------------------
        status = setDIS_ServiceInit(i, BLE_GATT_ROLE_SERVER, &(bleProfile_link0_info[i].serviceDIS_info_s), NULL);
        BLESTACK_STATUS_CHECK(status);

        // UDF01S (Server) Related
        // -------------------------------------
        status = setUDF01S_ServiceInit(i, BLE_GATT_ROLE_SERVER, &(bleProfile_link0_info[i].serviceUDF01S_info_s), BleService_UDF01SLink0Handler);
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
        // Handle Link0 - TRSP Peripheral
        handle_AppLink0_TRSPP(&bleProfile_link0_info[i]);
    }
} //end of BleApp_Main()


/* BLE Callback Function */
static void BleEvent_Callback(BleCmdEvent event, void *param)
{
    switch (event)
    {
    case BLECMD_EVENT_ADV_COMPLETE:
    {
        if (bleProfile_link0_info[activeAdvHostId].bleState == STATE_BLE_ADVERTISING)
        {
            printf("[%d] Start advertising...\n", activeAdvHostId);
        }
        else
        {
            printf("[%d] Stop advertising...\n", activeAdvHostId);
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

        // previous advertising action is done, we can start new adv action
        activeAdvHostId = -1;
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

    default:
        break;
    }
}


static void BleService_UDF01SLink0Handler(uint8_t hostId, uint8_t cmdAccess, uint8_t *data, uint16_t length)
{
    switch (cmdAccess)
    {
    case BLESERVICE_UDF01S_UDATRW01_WRITE_EVENT:
    {
        //show received RF data on UART
        printf("[%d] ", hostId);
        UART_TX_Send(length, data);
    }
    break;

    case BLESERVICE_UDF01S_UDATRW01_READ_EVENT:
    {
        //send read rsp with const read data to client
        uint8_t readData[] = "UDATRW01 data";
        setBLEGATT_GeneralReadRsp(hostId, bleProfile_link0_info[hostId].serviceUDF01S_info_s.handles.hdl_udatrw01, (uint8_t *)readData, (SIZE_STRING(readData)));
    }
    break;

    default:
        break;
    }
}


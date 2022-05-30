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
#include "porting_misc.h"

/**************************************************************************
* Application Definitions
**************************************************************************/
// Scan Parameters
#define SCAN_TYPE                       SCAN_TYPE_ACTIVE
#define SCAN_FILTER                     SCAN_FILTER_POLICY_ACCEPT_ALL
#define SCAN_WINDOW                     18U     // 18*0.625ms=11.25ms
#define SCAN_INTERVAL                   160U    // 160*0.625ms=100ms


/**************************************************************************
 * Variable
 **************************************************************************/
uint8_t ble_state = STATE_BLE_STANDBY;


/**************************************************************************
 * Function Prototype Declaration
 **************************************************************************/
BleStackStatus Ble_ScanStart(void);
static void BleEvent_Callback(BleCmdEvent event, void *param);

/**************************************************************************
 * Function
 **************************************************************************/

void handle_App_NoConnect(void)
{
    BleStackStatus status;

    if (ble_state == STATE_BLE_STANDBY)
    {
        // enable scanning
        status = Ble_ScanStart();
        if (status == BLESTACK_STATUS_SUCCESS)
        {
            ble_state = STATE_BLE_SCANNING;
        }
    }
    else if (ble_state == STATE_BLE_SCANNING)
    {

    }
}

BleStackStatus Ble_ScanInit(void)
{
    BleStackStatus status;
    BLE_Scan_Param scanParam;

    // Scan parameter init
    scanParam.scanType = SCAN_TYPE;
    scanParam.scanInterval = SCAN_INTERVAL;
    scanParam.scanWindow = SCAN_WINDOW;
    scanParam.scanFilterPolicy = SCAN_FILTER;
    status = setBLE_ScanParam(&scanParam);

    return status;
}


BleStackStatus Ble_ScanStart(void)
{
    BleStackStatus status;

    status = Ble_ScanInit();
    BLESTACK_STATUS_CHECK(status);

    status = setBLE_ScanEnable();
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

    return status;
}

void BleApp_Main(void)
{
    // Handle App for No Connect
    handle_App_NoConnect();
} //end of BleApp_Main()


/* BLE Callback Function */
static void BleEvent_Callback(BleCmdEvent event, void *param)
{
    switch (event)
    {
    case BLECMD_EVENT_SCAN_COMPLETE:
        break;

    case BLECMD_EVENT_SCAN_REPORT:
    {
        BLE_Event_ScanReportParam *scanRepParam = (BLE_Event_ScanReportParam *)param;

#if 0
        printf("Address: %02x:%02x:%02x:%02x:%02x:%02x\n",
               scanRepParam->rptPeerAddr.addr[5], scanRepParam->rptPeerAddr.addr[4],
               scanRepParam->rptPeerAddr.addr[3], scanRepParam->rptPeerAddr.addr[2],
               scanRepParam->rptPeerAddr.addr[1], scanRepParam->rptPeerAddr.addr[0]);
        printf("rptType = 0x%02x\n", scanRepParam->rptType);
        printf("rptData = 0x%02x, 0x%02x, 0x%02x\n",
               scanRepParam->rptData[0],
               scanRepParam->rptData[7],
               scanRepParam->rptData[8]);
        printf("RSSI:%d\n", scanRepParam->rptRssi);
#endif

        /* Find the Non-Connectable undirected advertising */
        if (scanRepParam->rptType == ADV_TYPE_ADV_NONCONN_IND)
        {
            if ((scanRepParam->rptData[5] == 0x4C) && \
                (scanRepParam->rptData[6] == 0x00) && \
                (scanRepParam->rptData[7] == 0x02))
            {
                printf("Found an iBeacon device\n");

                // device address
                printf("\tAddress: %02x:%02x:%02x:%02x:%02x:%02x\n",
                       scanRepParam->rptPeerAddr.addr[5], scanRepParam->rptPeerAddr.addr[4],
                       scanRepParam->rptPeerAddr.addr[3], scanRepParam->rptPeerAddr.addr[2],
                       scanRepParam->rptPeerAddr.addr[1], scanRepParam->rptPeerAddr.addr[0]);
                // received data
                printf("\tRSSI: %d\n", scanRepParam->rptRssi);
            }
            else if ((scanRepParam->rptData[7] == 0x01) && \
                (scanRepParam->rptData[8] == 0x02) && \
                (scanRepParam->rptData[14] == 0x08))
            {
                printf("Found a Custom Beacon device\n");

                // device address
                printf("\tAddress: %02x:%02x:%02x:%02x:%02x:%02x\n",
                       scanRepParam->rptPeerAddr.addr[5], scanRepParam->rptPeerAddr.addr[4],
                       scanRepParam->rptPeerAddr.addr[3], scanRepParam->rptPeerAddr.addr[2],
                       scanRepParam->rptPeerAddr.addr[1], scanRepParam->rptPeerAddr.addr[0]);
                // received data
                printf("\tRSSI: %d\n", scanRepParam->rptRssi);
                printf("\tVariable 1: %d\n", scanRepParam->rptData[5]);
                printf("\tVariable 2: %d\n", scanRepParam->rptData[6]);
            }
        }
    }
    break;

    default:
        break;
    }
}


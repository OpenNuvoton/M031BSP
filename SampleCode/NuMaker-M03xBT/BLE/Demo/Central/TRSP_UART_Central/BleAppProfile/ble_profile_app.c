/************************************************************************
 *
 * File Name  : ble_profile_app.c
 * Description: This file contains the definitions and functions of BLE profiles for application.
 *
 *******************************************************************/
#include <stdio.h>
#include "ble_profile.h"


/**************************************************************************
 * Profile Application Public Definitions and Variables
 **************************************************************************/

/* Link0 profile information */
BLEProfile_Link0_Info        bleProfile_link0_info;


/**************************************************************************
 * Profile Application GENERAL Public Functions
 **************************************************************************/

/** Get BLE LINK0 Service All Handles
 *
 * @attention MUST call this API to get service handles after received @ref BLECMD_EVENT_ATT_DATABASE_PARSING_FINISHED if role is client
 *
 * @param[in] hostId : the link's host id.
 * @param[out] attInfo : a pointer to INK0 attribute information
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus getBLELink0_ServiceHandles(uint8_t hostId, BLEProfile_Link0_Info *attInfo)
{
    BleStackStatus status;

    attInfo->hostId = hostId;

    // Get GAP handles
    status = getGAP_ServiceHandles(hostId, (void *)&attInfo->serviceGAP_info_c);
    BLESTACK_STATUS_CHECK(status);

    // Get GATT handles
    status = getGATT_ServiceHandles(hostId, (void *)&attInfo->serviceGATT_info_c);
    BLESTACK_STATUS_CHECK(status);

    // Get DIS handles
    status = getDIS_ServiceHandles(hostId, (void *)&attInfo->serviceDIS_info_c);
    BLESTACK_STATUS_CHECK(status);

    // Get UDF01S handles
    status = getUDF01S_ServiceHandles(hostId, (void *)&attInfo->serviceUDF01S_info_c);
    BLESTACK_STATUS_CHECK(status);

    return BLESTACK_STATUS_SUCCESS;
}


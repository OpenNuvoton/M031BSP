/**************************************************************************//**
* @file       ble_profile.h
* @brief      Provide the declarations that for BLE Profile subsystem needed.
*
*****************************************************************************/

#ifndef _BLE_PROFILE_H_
#define _BLE_PROFILE_H_

#include <stdint.h>
#include "BleAppSetting.h"
#include "ble_host.h"
#include "ble_service_base.h"
#include "ble_service_gap.h"
#include "ble_service_gatt.h"
#include "ble_service_dis.h"
#include "ble_service_udf01s.h"
#include "ble_service_hrs.h"


/**************************************************************************
 * Profile Application GENERAL Public Definitions and Functions
 **************************************************************************/
/** @defgroup profileAppGeneralFunc Profile Application General Definitions and Functions
* @{
* @details Here shows the general definitions of the application profile.
* @}
**************************************************************************/
/** Define the maximum number of BLE GAP service link. */
#define MAX_NUM_CONN_GAP                   2

/** Define the maximum number of BLE GATT service link. */
#define MAX_NUM_CONN_GATT                  2

/** Define the maximum number of BLE DIS service link. */
#define MAX_NUM_CONN_DIS                   2

/** Define the maximum number of BLE UDF01 service link. */
#define MAX_NUM_CONN_UDF01S                1

/** Define the maximum number of BLE HRS service link. */
#define MAX_NUM_CONN_HRS                   1

/** Extern maximum Number of Host Connection Link Definition. */
extern const uint8_t MAX_NUM_CONN_HOST;


/**************************************************************************
 * Profile Application LINK HOST ID = 0 Public Definitions and Functions
 **************************************************************************/

/** BLE Application Link 0 Profile Attribute Information Structure.
*/
typedef struct BLEProfile_Link0_Info
{
    uint8_t              hostId;                  /**< Host id. */
    BleMode              bleState;                /**< Current BLE mode. */
    uint8_t              subState;                /**< Current link substate. */
    BLEATT_GAP_Info      serviceGAP_info_c;       /**< GAP information (client). */
    BLEATT_GATT_Info     serviceGATT_info_c;      /**< GATT information (client). */
    BLEATT_DIS_Info      serviceDIS_info_c;       /**< Device Information Service information (client). */
    BLEATT_UDF01S_Info   serviceUDF01S_info_c;    /**< UDF01 Service information (client). */
} BLEProfile_Link0_Info;


/** BLE Application Link 0 Profile Attribute Information Structure.
*/
typedef struct BLEProfile_Link1_Info
{
    uint8_t              hostId;                  /**< Host id. */
    BleMode              bleState;                /**< Current BLE mode. */
    uint8_t              subState;                /**< Current link substate. */
    BLEATT_GAP_Info      serviceGAP_info_s;       /**< GAP information (server). */
    BLEATT_GATT_Info     serviceGATT_info_s;      /**< GATT information (server). */
    BLEATT_DIS_Info      serviceDIS_info_s;       /**< Device Information Service information (server). */
    BLEATT_HRS_Info      serviceHRS_info_s;       /**< Heart Rate Service information (server). */
} BLEProfile_Link1_Info;



/** Extern BLE Application Link 0 Profile Attribute Information Initialization.
*/
extern BLEProfile_Link0_Info bleProfile_link0_info;

/** Extern BLE Link1 HRS service information
*/
extern BLEProfile_Link1_Info bleProfile_link1_info;


/** Get BLE LINK0 Service Information
 *
 * @attention MUST call this API to get service information after received @ref BLECMD_EVENT_ATT_DATABASE_PARSING_FINISHED
 *
 * @param[in] hostId : the link's host id.
 * @param[out] attInfo : a pointer to INK0 attribute information
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus getBLELink0_ServiceHandles(uint8_t hostId, BLEProfile_Link0_Info *attInfo);

#endif // _BLE_PROFILE_H_


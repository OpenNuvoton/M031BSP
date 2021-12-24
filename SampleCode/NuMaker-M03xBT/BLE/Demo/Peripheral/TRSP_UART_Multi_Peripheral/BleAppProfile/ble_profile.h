/**************************************************************************//**
* @file       ble_profile.h
* @brief      Provide the declarations that for BLE Profile subsystem needed.
*
*****************************************************************************/

#ifndef _BLE_PROFILE_H_
#define _BLE_PROFILE_H_

#include <stdint.h>
#include "ble_host.h"
#include "ble_service_base.h"
#include "ble_service_gap.h"
#include "ble_service_gatt.h"
#include "ble_service_dis.h"
#include "ble_service_udf01s.h"


/**************************************************************************
 * Profile Application GENERAL Public Definitions and Functions
 **************************************************************************/
/** @defgroup profileAppGeneralFunc Profile Application General Definitions and Functions
* @{
* @details Here shows the general definitions of the application profile.
* @}
**************************************************************************/
/** Define the maximum number of BLE GAP service link. */
#define MAX_NUM_CONN_GAP            3

/** Define the maximum number of BLE GATT service link. */
#define MAX_NUM_CONN_GATT           3

/** Define the maximum number of BLE DIS service link. */
#define MAX_NUM_CONN_DIS            3

/** Define the maximum number of BLE UDF01 service link. */
#define MAX_NUM_CONN_UDF01S         3


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
    BLEATT_GAP_Info      serviceGAP_info_s;       /**< GAP Service information (server). */
    BLEATT_GATT_Info     serviceGATT_info_s;      /**< GATT Service information (server). */
    BLEATT_DIS_Info      serviceDIS_info_s;       /**< DIS Service information (server). */
    BLEATT_UDF01S_Info   serviceUDF01S_info_s;    /**< UDF01 Service information (server). */
} BLEProfile_Link0_Info;



/** Extern BLE Application Link 0 Profile Attribute Information Initialization.
*/
extern BLEProfile_Link0_Info bleProfile_link0_info[MAX_NUM_CONN_UDF01S];



#endif // _BLE_PROFILE_H_


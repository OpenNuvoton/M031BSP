#ifndef _BLEAPPSETTING_H_
#define _BLEAPPSETTING_H_

#include "mcu_definition.h"
#include "porting_misc.h"

/**************************************************************************
 * Application Setting
 **************************************************************************/
// Define the maximum number of BLE connection link.
#define BLE_SUPPORT_NUM_CONN_MAX    1

// BLE stack supports device bonding if defined ENABLE_DEF.
#define BLE_SUPPORT_BOND            DISABLE_DEF

// BLE stack supports large MTU size if define ENABLE_DEF.
#define BLE_SUPPORT_MTU_LARGE       DISABLE_DEF

// BLE stack run in DTM mode.
#define BLE_DTM_ENABLE              DISABLE_DEF

// Set MTU size
#if (BLE_SUPPORT_MTU_LARGE == ENABLE_DEF)
#define DEFAULT_MTU                 247
#else
#define DEFAULT_MTU                 23
#endif

// Set device BLE company ID
#define BLE_COMPANY_ID_L            0xCC
#define BLE_COMPANY_ID_H            0x0A


#endif

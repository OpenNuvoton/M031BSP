/**************************************************************************//**
 * @file       ble_rftone.h
 * @brief      This file contains the functions of Direct Test Mode (DTM) function for MP.
 *
 * @defgroup ble_commonDtm BLE DTM Function For MP
 * @{
 * @ingroup ble_cmd_comAPI
 * @details This file shows the BLE DTM functions for MP. (ble_rftone.h)
 * @}
*****************************************************************************/
#ifndef __BLE_MPTEST_H__
#define __BLE_MPTEST_H__

#include <stdio.h>
#include <stdint.h>
#include "ble_stack_status.h"


/** Function for Enable Continuous TX for MP Test.
 *
 * @ingroup ble_commonDtm
 *
 * @note This function is defined in "ble_rftone.h"
*/
BleStackStatus setRF_MPTestContTx(uint8_t channel);

#endif // __BLE_MPTEST_H__

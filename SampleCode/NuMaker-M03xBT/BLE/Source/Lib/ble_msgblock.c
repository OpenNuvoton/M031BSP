/*----------------------------------------------------------------------------*/
/* This file defined Message Block Related */
/*----------------------------------------------------------------------------*/
#include <stdint.h>
#include "ble_msgblock.h"
#include "BleAppSetting.h"

/**************************************************************************
* Definition for BLE Stack
**************************************************************************/
/**
 * @note The total message blocks are split into two regions (L1 and L2) in BLE stack.
 *
 **************************************************************************/
/** Each LL link reserves 1 message block for L1. */
#define RESERVED_MBLK_LL_LINK     (NUM_LL_LINK * 1)

/** Define message block number of Link Layer connection link for BLE stack reference. */
const uint8_t MAX_MBLK_RSV_LL_ROLE = NUM_LL_LINK;

/** Define total message block number to L2 for BLE stack reference. */
const uint8_t MAX_MBLK_RSV_L2 = MIN_MSGBLOCK_LL;

/** Define total message block number to L1 for BLE stack reference. */
const uint8_t MAX_MBLK_RSV_L1 = (MIN_MSGBLOCK_LL_ADV + MIN_MSGBLOCK_LL_SCAN + MIN_MSGBLOCK_LL_INIT + RESERVED_MBLK_LL_LINK);

/** Define maximum number of message block for BLE stack reference. */
const uint16_t MAX_MBLK_NO = MAX_MBLK_NO_;

/** Define message block array for BLE stack. */
uint32_t MBLKData[MAX_MBLK_NO_][REF_SIZE_MBLK >> 2];

/** Define maximum number of BLE connection link for BLE stack. */
uint8_t const MAX_CONN_NO_APP = BLE_SUPPORT_NUM_CONN_MAX;



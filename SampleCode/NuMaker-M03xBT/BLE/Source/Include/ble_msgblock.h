/**************************************************************************//**
* @file       ble_msgblock.h
* @brief      This file defined Message Block Related.
*
*****************************************************************************/

#ifndef _BLE_MSGBLCOK_H_
#define _BLE_MSGBLCOK_H_

#include "BleAppSetting.h"


// Check if defined "BLE_SUPPORT_NUM_CONN_MAX" or show error message
#ifndef BLE_SUPPORT_NUM_CONN_MAX
#error "Undefine BLE_SUPPORT_NUM_CONN_MAX."
#endif

// Check if defined "BLE_SUPPORT_MTU_LARGE" or show error message
#ifndef BLE_SUPPORT_MTU_LARGE
#error "Undefine BLE_SUPPORT_MTU_LARGE."
#endif

/**************************************************************************
* Definition
**************************************************************************/
/**
 * @attention 1. The value of the definitions which are defined in this file shall NOT be modified except for @ref NUM_LL_LINK and @ref MIN_MSGBLOCK_HOST_MORE_DATE. \n
 *            2. Host Layer default is set to (MTU = 251), can be set to Host Layer (MTU = 23) to reserved more RAM.
 *            3. @ref NUM_LL_LINK can be less than or equal to @ref NUM_HOST_LINK, default is set to the same setting.
 **************************************************************************/

/* The size of a message block(Bytes).*/
#define REF_SIZE_MBLK                   56

/*------------------------------------------------------------------------------
 * Link Layer
 *------------------------------------------------------------------------------*/
/* Define maximum connection links for Link Layer. */
#define NUM_LL_LINK                     BLE_SUPPORT_NUM_CONN_MAX

/* Define the minimum message block number for LL supports advertisement. */
#define MIN_MSGBLOCK_LL_ADV             2

/* Define the minimum message block number for LL supports scan. */
#define MIN_MSGBLOCK_LL_SCAN            2

/* Define the minimum message block number for LL supports initiator. */
#define MIN_MSGBLOCK_LL_INIT            1

/* Define the minimum message block number for LL supports a connection link. */
#define MIN_MSGBLOCK_LL_CONN_LINK       5

/* Define minimum message block number for Link layer.
 * @note Shall be supported advertisement, scan, initiator and connections.
*/
#define MIN_MSGBLOCK_LL                 (MIN_MSGBLOCK_LL_ADV + MIN_MSGBLOCK_LL_SCAN + MIN_MSGBLOCK_LL_INIT) + \
                                        (NUM_LL_LINK * MIN_MSGBLOCK_LL_CONN_LINK)


/*------------------------------------------------------------------------------
 * Host Layer
 *------------------------------------------------------------------------------*/
/* Define maximum connection links for Host Layer. */
#define NUM_HOST_LINK                   BLE_SUPPORT_NUM_CONN_MAX

/* Define the minimum message block number for Host supports a extra "more data(MD)". */
#define MIN_MSGBLOCK_HOST_MORE_DATA     3


#if (BLE_SUPPORT_MTU_LARGE == ENABLE_DEF) // MTU = 247

/* Define the minimum message block number for Host supports a connection link. */
#define MIN_MSGBLOCK_HOST_CONN_LINK     7

#else // MTU = 23

/* Define the minimum message block number for Host supports a connection link. */
#define MIN_MSGBLOCK_HOST_CONN_LINK     2

#endif // BLE_SUPPORT_MTU_LARGE


/* Define the minimum message block number for Host handle commands. */
#define MIN_MSGBLOCK_COMMAND            6

/* Define the minimum message block number for Host layer. */
#define MIN_MSGBLOCK_HOST               ((NUM_HOST_LINK * MIN_MSGBLOCK_HOST_CONN_LINK * MIN_MSGBLOCK_HOST_MORE_DATA) + MIN_MSGBLOCK_COMMAND)

/*------------------------------------------------------------------------------
 * Total Number of Message Blocks
 *------------------------------------------------------------------------------*/
/** Calculate the maximum number of message block. */
#define MAX_MBLK_NO_                    (MIN_MSGBLOCK_LL + MIN_MSGBLOCK_HOST)

#endif //_BLE_MSGBLCOK_H_


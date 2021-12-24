/**************************************************************************//**
* @file       ble_stack_status.h
* @brief      This file contains the functions of HOST to HCI interface.
*
*
* @defgroup ble_common BLE Common
* @{
* @details This file shows the common BLE definitions and functions. (ble_cmd.h, ble_event.h)
* @}
*****************************************************************************/

#ifndef _BLE_STACK_STATUS_H_
#define _BLE_STACK_STATUS_H_

#include "mcu_definition.h"

/**
 * @ingroup ble_stack_definition
 * @defgroup ble_Stack_Status BLE Stack Error Code Definition
 * @{
 * @details BLE Stack error code definition.
 * @name BleStackStatus
 * @brief Define different BleStackStatus type.
 * @{
 */
typedef uint8_t BleStackStatus;

/** Successful command */
#define BLESTACK_STATUS_SUCCESS                           0x00

/** Stack state is free indicates that LL and Host task is NOT running. */
#define BLESTACK_STATUS_FREE                              0x01

/** Stack state busy. */
#define BLESTACK_STATUS_ERR_BUSY                          0x02

/** Invalid parameter. */
#define BLESTACK_STATUS_ERR_INVALID_PARAM                 0x03

/** Invalid state. */
#define BLESTACK_STATUS_ERR_INVALID_STATE                 0x04

/** Invalid Host ID */
#define BLESTACK_STATUS_ERR_INVALID_HOSTID                0x05

/** Invalid count of Host Links. (MAX_NUM_CONN_HOST shall be less than or equal to BLE_SUPPORT_NUM_CONN_MAX) */
#define BLESTACK_STATUS_ERR_INVALID_HOSTCOUNT             0x06

/** Invalid command. */
#define BLESTACK_STATUS_ERR_INVALID_CMD                   0x07

/** Invalid BLE handle. */
#define BLESTACK_STATUS_ERR_INVALID_HANDLE                0x08

/** Command timer busy */
#define BLESTACK_STATUS_ERR_TIMER_BUSY                    0x09

/** Command feature not supported */
#define BLESTACK_STATUS_ERR_NOT_SUPPORTED                 0x0A

/** Host peripheral database parsing is still in progress. */
#define BLESTACK_STATUS_ERR_DB_PARSING_IN_PROGRESS        0x0B

/** The other mandatory procedure is still in progress. */
#define BLESTACK_STATUS_ERR_OTHER_PROCEDURE_IN_PROGRESS   0x0C

/** Detecting a sequential protocol violation. Usually happens in there is an another GATT request already in progress please wait and retry.*/
#define BLESTACK_STATUS_ERR_SEQUENTIAL_PROTOCOL_VIOLATION 0x0D

/** Profile client configuration disable */
#define BLESTACK_STATUS_ERR_CLIENT_CONFIGURATION_DISABLE  0x0E

/** @} */
/** @} */



/** Marcro return BLE stack status if input status is not equal to BLESTACK_STATUS_SUCCESS.
 *
 * @param[in] status BLE stack status.
 */
#define BLESTACK_STATUS_CHECK(status)                \
    if (status != BLESTACK_STATUS_SUCCESS)           \
    {                                                \
        return status;                               \
    }                                                \



#endif // _BLE_STACK_STATUS_H_

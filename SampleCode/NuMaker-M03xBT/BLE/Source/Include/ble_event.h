/**************************************************************************//**
* @file       ble_event.h
* @brief      This file contains the functions of HCI to HOST interface.
*
* @defgroup   ble_event BLE Event Definition
* @{
* @ingroup ble_common
* @details This file handles the events from HCI to HOST.
* @}
*****************************************************************************/

#ifndef _BLE_EVENT_H_
#define _BLE_EVENT_H_


#include "ble_cmd.h"



/** BleEventCallBack
 * @ingroup ble_event
 * @note This callback receives the @ref ble_cmd_cmdEvent "BleCmdEvent" events.  \n
 *  Each of these events can be associated with parameters.
 *
 */
typedef void (*BleEventCallBack)(BleCmdEvent event,
                                 void *param);



/** Register a callback function to receive BleCmdEvent
 * @ingroup ble_event
 * @param[in] callback : a pointer to a callback function that receive the command events.
 */
void setBLE_RegisterBleEvent(BleEventCallBack callback);


#endif // _BLE_EVENT_H_


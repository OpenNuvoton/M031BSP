/*************************************************************************//**
 * @file       ble_hci_cmd.h
 * @brief      This file contains the functions of HCI command.
 *
 * @defgroup ble_hci_cmd BLE HCI COMMAND
 * @{
 * @details  Provides the declaration that needed for hci command. (ble_hci_cmd.h).
 * @}
*****************************************************************************/
#ifndef _BLE_HCI_CMD_H_
#define _BLE_HCI_CMD_H_

/**************************************************************************
 * Functions
 **************************************************************************/
/** @defgroup ble_hci_cmdFunc BLE HCI Command Function
 * @{
 * @ingroup ble_hci_cmd
 * @details  HCI Command Functions.
 * @}
 */

/** BLE HCI command Function
 *
 * @remark <b>[BLELIB_HCI_CONTROL]</b>
 *
 * @param[in]   pHCIcmd: the HCI command data.
 * @param[in]   length : the HCI command length.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED    : Command not supported.
 * @retval BLESTACK_STATUS_ERR_BUSY             : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS              : Setting success.
 */
BleStackStatus setBLE_HCICommand(uint8_t *pHCIcmd, uint16_t length);



#endif  //(#ifndef _BLE_HCI_CMD_H_)

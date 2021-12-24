/**************************************************************************//**
* @file       fota.h
* @brief      This file contains the functions of FOTA.
*
*****************************************************************************/
#ifndef _FOTA_H_
#define _FOTA_H_


typedef enum _FotaTimerState
{
    RUNNING,
    EXPIRED,
} FotaTimerState;



/** Ble FOTA Timer tick
 *
 *
 * @note       This function should be called once every second, \n
 *             so that the fota related timers can be normally operation. \n
 *
 * @retval RUNNING : FOTA timer is running.
 * @retval EXPIRED : FOTA timer is expired.
 */
FotaTimerState BleFota_TimerTick(void);


/** Ble FOTA handle FOTA timer expired event
 *
 *
 * @note       This function should be called when FOTA timer is expired.
 *
 * @return none
 */
void BleFota_TimerExpiryHandler(void);


/** Ble FOTA parameters initialization
 *
 *
 * @return none
 */
void BleFota_Init(void);


/** Ble FOTA command processing
 *
 *
 * @param[in] length : command length.
 * @param[in] data : command payload.
 *
 * @note       First byte of command payload contains command ID and each command ID may contain different information behind.
 *             OTA_CMD_QUERY : Get device current system information. \n
 *             OTA_CMD_START : Start FW upgrade, this command contains new FW length and CRC. \n
 *             OTA_CMD_ERASE : Terminated the connection and erasing legacy FW and informations. \n
 *             OTA_CMD_APPLY : Apply the new FW if receiving FW length and CRC matched with OTA_CMD_START. \n
 *
 * @return none
 */
void BleFota_Cmd(uint8_t length, uint8_t *data);


/** Ble FOTA data processing
 *
 *
 * @param[in] length : data length.
 * @param[in] data : data payload.
 *
 * @note       First 4 bytes of data payload is data header which contains the FOTA data address (3 bytes) and length (1 byte),
 *             if there were invalid data header, send notification to response it. \n
 *
 * @return none
 */
void BleFota_Data(uint8_t length, uint8_t *data);



/** The actions related to FOTA after complete the disconnection.
 *
 * @note       perform the action by FwUpgradeState. \n
 *             OTA_STATE_COMPLETE : System reboot for bootloader to check new FW. \n
 *             OTA_STATE_ERASING : Erase bank FW and bank information and waiting for reconnection. \n
 *
 * @return none
 */
void BleFota_Disconnect(void);

#endif // _FOTA_H_


/**************************************************************************//**
 * @file       ble_rf_dtm.h
 * @brief      This file provides the declaration that RF IC control needed for DTM.
 *
 * @defgroup ble_dtm_phy BLE RF PHY for DTM
 * @{
 * @ingroup ble_dtm
 * @details  Provides the declaration that RF IC control needed for DTM. (ble_rf_dtm.h).
 * @}
*****************************************************************************/
#ifndef _BLE_RF_DTM_H__
#define _BLE_RF_DTM_H__


/**************************************************************************
 * Definitions
 **************************************************************************/
/**
 * @defgroup ble_dtm_txrx_def BLE DTM TX/ RX Mode Definition
 * @{
 * @ingroup ble_dtm_phy
 */
#define BLEDTM_TX_MODE                  0    /**< Constant defining TX mode for radio during dtm test. */
#define BLEDTM_RX_MODE                  1    /**< Constant defining RX mode for radio during dtm test. */
/** @} */

/**
 * @defgroup ble_dtm_sleep_def BLE DTM Sleep/ Deep Sleep Mode Definition
 * @{
 * @ingroup ble_dtm_phy
 */
#define BLEDTM_SLEEP_MODE               0    /**< Sleep mode. */
#define BLEDTM_DEEP_SLEEP_MODE          1    /**< Deep sleep mode. */
/** @} */

/**************************************************************************
 * Functions
 **************************************************************************/

/** BLE DTM RF PHY Reset
 *
 * @ingroup ble_dtm_phy
 *
 * @return none
*/
void BleDTM_phy_reset(void);


/** BLE DTM Set TX Power
 *
 * @ingroup ble_dtm_phy
 *
 * @param[in] power_index : requested TX power level in dBm, enter the index of TX power register table. \n
 *                          The TX power register table please refer to "porting_rfpower.c".
 *
 * @return  TX power set result success or not.
 * @retval  0:  Success
 * @retval  1:  FAIL
 */
uint8_t BleDTM_txPower_set(uint8_t power_index);


/** BLE DTM Set TX test payload
 *
 * @ingroup ble_dtm_phy
 *
 * @param[in] length: the length of TX test payload.
 *
 * @return none
*/
void BleDTM_testPayload_set(uint32_t length);


/** BLE DTM Enable TX Test
 *
 * @ingroup ble_dtm_phy
 *
 * @param[in] headerSrcAddr: a pointer to header buffer.
 *
 * @return none
*/
void BleDTM_txTest_enable(uint8_t *headerSrcAddr);


/** BLE DTM Set Channel
 *
 * @ingroup ble_dtm_phy
 *
 * @param[in] channel: physical channel.
 *
 * @return none
*/
void BleDTM_channel_set(uint8_t channel);


/** BLE DTM Manual Set TX/ RX.
 *
 * @ingroup ble_dtm_phy
 *
 * @return none
*/
void BleDTM_TR_ManualSet(void);


/** BLE DTM Enter Sleep Mode
 *
 * @ingroup ble_dtm_phy
 *
 * @param[in] mode: sleep mode is @ref BLEDTM_SLEEP_MODE or @ref BLEDTM_DEEP_SLEEP_MODE.
 *
 * @return none
*/
void BleDTM_sleepMode_set(uint8_t mode);


/** BLE DTM Check Interrupt State if is RX State or Not
 *
 * @ingroup ble_dtm_phy
 *
 * @return  Checking result YES or not.
 * @retval  0:  NO
 * @retval  1:  YES
 */
uint8_t BleDTM_intRxState_check(void);


/** BLE DTM Get CRC Result
 *
 * @ingroup ble_dtm_phy
 *
 * @return  CRC result success or not.
 * @retval  0:  FAILED
 * @retval  1:  SUCCESS
 */
uint8_t BleDTM_crc_checkResult_get(void);


/** BLE DTM Preparing the RF Radio.
 *
 * @ingroup ble_dtm_phy
 *
 * @note At start of each test: Turn off RF, clear interrupt flags of RF, initialize the radio at given RF channel.
 *
 * @param[in] mode : @ref BLEDTM_TX_MODE or @ref BLEDTM_RX_MODE.
 * @param[in] packet_type : packet type.
 * @param[in] radio_mode : @ref BLEDTM_RADIO_PHY_1M or @ref BLEDTM_RADIO_PHY_2M.
 * @param[in] phys_ch : physical channel.
 *
 * @return none
*/
void BleDTM_radio_prepare(uint8_t rx, uint32_t packet_type, uint8_t radio_mode, uint32_t phys_ch);


/** BLE DTM Get RSSI Value
 *
 * @ingroup ble_dtm_phy
 *
 * @return  RSSI value.
 */
int8_t BleDTM_rssi_get(void);


#endif // _BLE_RF_DTM_H__


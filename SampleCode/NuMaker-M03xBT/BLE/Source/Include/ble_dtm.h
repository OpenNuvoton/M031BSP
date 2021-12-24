/**************************************************************************//**
 * @file       ble_dtm.h
 * @brief      This file contains the functions of Direct Test Mode (DTM).
 *
 * @defgroup ble_dtm BLE DTM
 * @{
 * @details  Provides the declaration that needed for DTM. (ble_dtm.h).
 * @}
*****************************************************************************/
#ifndef BLE_DTM_H__
#define BLE_DTM_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble_rf_dtm.h"
#include "porting_dtm.h"
#include "porting_misc.h"

/**************************************************************************
 * Definitions
 **************************************************************************/
/** @defgroup ble_dtmDef BLE DTM Definition
 * @{
 * @ingroup ble_dtm
 * @details  DTM Definitions.
 */

/**
 *@brief BLE DTM Command for Lab Test Tool.
 */
#define ENABLE_LAB_TEST_TOOL_SUPPORTS                ENABLE_DEF         /**<  @ref ENABLE_DEF or @ref DISABLE_DEF */


/**
 *@brief BLE DTM Command Definition.
 */
#define LE_TEST_SETUP                                         0         /**< DTM command: Set PHY or modulation, configure upper two bits of length,
                                                                             request matrix of supported features or request max values of parameters. */
#define LE_RECEIVER_TEST                                      1         /**< DTM command: Start receive test. */
#define LE_TRANSMITTER_TEST                                   2         /**< DTM command: Start transmission test. */
#define LE_TEST_END                                           3         /**< DTM command: End test and send packet report. */

/**
 *@brief BLE DTM "Test Setup" Command Parameter Definition.
 */
#define LE_TEST_SETUP_RESET                                   0         /**< DTM command parameter: Stop TX/RX, reset the packet length upper bits and set the PHY to 1Mbit. */
#define LE_TEST_SETUP_SET_UPPER                               1         /**< DTM command parameter: Set the upper two bits of the length field. */
#define LE_TEST_SETUP_SET_PHY                                 2         /**< DTM command parameter: Select the PHY to be used for packets. */
#define LE_TEST_SETUP_SELECT_MODULATION                       3         /**< DTM command parameter: Select standard or stable modulation index. Stable modulation index is not supported. */
#define LE_TEST_SETUP_READ_SUPPORTED                          4         /**< DTM command parameter: Read the supported test case features. */
#define LE_TEST_SETUP_READ_MAX                                5         /**< DTM command parameter: Read the max supported time and length for packets. */


/**
 *@brief BLE DTM "Test Setup" Command Parameter - PHY Definition.
 */
#define LE_PHY_1M                                             1         /**< DTM command parameter: Set PHY for future packets to use 1MBit PHY. */
#define LE_PHY_2M                                             2         /**< DTM command parameter: Set PHY for future packets to use 2MBit PHY. */
#define LE_PHY_LE_CODED_S8                                    3         /**< DTM command parameter: Set PHY for future packets to use coded PHY with S=8. */
#define LE_PHY_LE_CODED_S2                                    4         /**< DTM command parameter: Set PHY for future packets to use coded PHY with S=2 */


/**
 *@brief BLE DTM Packet Reporting Event Definition.
 */
#define LE_PACKET_REPORTING_EVENT                        0x8000         /**< DTM Packet reporting event, returned by the device to the tester. */


/**
 *@brief BLE DTM Status Event Definition.
 */
#define LE_TEST_STATUS_EVENT_SUCCESS                     0x0000         /**< DTM event status success. */
#define LE_TEST_STATUS_EVENT_ERROR                       0x0001         /**< DTM event status error. */


/**
 *@brief BLE DTM Packet Type Definition.
 */
#define DTM_PKT_PRBS9                                      0x00         /**< Bit pattern PRBS9. */
#define DTM_PKT_0X0F                                       0x01         /**< Bit pattern 11110000 (LSB is the leftmost bit). */
#define DTM_PKT_0X55                                       0x02         /**< Bit pattern 10101010 (LSB is the leftmost bit). */
#define DTM_PKT_0XFF                                       0x03         /**< Bit pattern 11111111 (Used only for coded PHY). */



#if (ENABLE_LAB_TEST_TOOL_SUPPORTS == ENABLE_DEF)
/**
 *@brief BLE DTMVendor Specific Command Definition.
 */
#define DTM_VENDORSPECIFIC_TX_RANDOM_CH_DISABLE               1         /**< TX Random Channel disable. */
#define DTM_VENDORSPECIFIC_TX_RANDOM_CH_ENABLE                2         /**< TX Random CH enable. */
#define DTM_VENDORSPECIFIC_TX_PACKAGE_NUMBER_UNLIMITED        3         /**< TX package number unlimited. */
#define DTM_VENDORSPECIFIC_TX_PACKAGE_NUMBER_BY_USER          4         /**< TX package number by user. */
#define DTM_VENDORSPECIFIC_TX_POWER_SELECT                    5         /**< TX power select 0,4,8 and 10dBm. */
#define DTM_VENDORSPECIFIC_RX_GET_RSSI                        6         /**< RX need print RSSI value. */
#define DTM_VENDORSPECIFIC_TX_POWER_SELECT_MORE_OPTION        7         /**< TX power select 0.5dB Step. */
#define DTM_VENDORSPECIFIC_GO_SLEEP_MODE                     20         /**< Phy enter sleep mode. */
#define DTM_VENDORSPECIFIC_GO_DEEP_SLEEP_MODE                21         /**< Phy enter deep sleep mode. */
#define DTM_VENDORSPECIFIC_GO_TXRF_CONTINUOUS_MODE           22         /**< TX output single tone mode. */
#define DTM_VENDORSPECIFIC_GO_TXRF_DTM_MODE                  23         /**< TX output DTM normal mode. */
#endif // (ENABLE_LAB_TEST_TOOL_SUPPORTS == ENABLE_DEF) 


/**
 *@brief Return codes from bledtm_cmd().
 */
typedef uint8_t BleDtmStatus;
#define BLEDTM_SUCCESS                                     0x00         /**< Indicate that the DTM function completed with success. */
#define BLEDTM_ERROR_ILLEGAL_CHANNEL                       0x01         /**< Physical channel number must be in the range 0..39. */
#define BLEDTM_ERROR_INVALID_STATE                         0x02         /**< Sequencing error: Command is not valid now. */
#define BLEDTM_ERROR_ILLEGAL_LENGTH                        0x03         /**< Payload size must be in the range 0..37. */
#define BLEDTM_ERROR_ILLEGAL_CONFIGURATION                 0x04         /**< Parameter out of range (legal range is function dependent). */
#define BLEDTM_ERROR_UNINITIALIZED                         0x05         /**< DTM module has not been initialized by the application. */


/** @} */

/**************************************************************************
 * Functions
 **************************************************************************/
/** @defgroup ble_dtmFunc BLE DTM Function
 * @{
 * @ingroup ble_dtm
 * @details  DTM Functions.
 * @}
 */

/** Function for handle BLE stack interrupt.
 *
 * @ingroup ble_dtmFunc
 *
*/
void BleDTM_Isr(void);


/**Function for initializing DTM module.
 *
 * @ingroup ble_dtmFunc
 *
 * @retval BLEDTM_ERROR_ILLEGAL_CONFIGURATION : Invalid configurations.
 * @retval BLEDTM_SUCCESS                     : Setting success.
*/
BleDtmStatus BleDTM_init(void);


/**Function for handle radio control process.
 *
 * @ingroup ble_dtmFunc
 *
 */
void BleDTM_radio_process(void);


/** Function for get current timer counter.
 *
 * @ingroup ble_dtmFunc
 *
 * @return  timer counter.
 */
uint32_t BleDTM_timerCount_update(void);


/** Function for splitting UART command bit fields into separate command parameters for the DTM library.
 *
 * @ingroup ble_dtmFunc
 *
 * @param[in]   command : The packed UART command.
 *
 * @return      result status.
 */
uint32_t BleDTM_cmd_process(uint16_t command);


/** Function for reading the result of a DTM command.
 *
 * @ingroup ble_dtmFunc
 *
 * @param[out] p_dtm_event : Pointer to buffer for 16 bit event code according to DTM standard.
 *
 * @return  Checking result.
 * @retval  0:  no event since last call.
 * @retval  1:  new event.
 */
uint8_t BleDTM_event_get(uint32_t *dtm_event);



#if (ENABLE_LAB_TEST_TOOL_SUPPORTS == ENABLE_DEF)
/**@brief Function for Get the RSSI flag.
 *
 * @ingroup ble_dtmFunc
 *
 * @return  Checking result.
 * @retval  0:  no No RSSI value provided.
 * @retval  1:  the last RSSI value will be provided after BleDTM_EventGet.
*/
uint8_t BleDTM_gui_rssiFlag_get(void);

#endif


#endif // BLE_DTM_H__

/** @} */

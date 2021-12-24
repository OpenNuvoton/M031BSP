/**************************************************************************//**
* @file       ble_cmd.h
* @brief      This file contains the functions of HOST to HCI interface.
*
*
* @defgroup ble_common BLE Common
* @{
* @details This file shows the common BLE definitions and functions. (ble_cmd.h, ble_event.h)
* @}
*****************************************************************************/

#ifndef _BLE_CMD_H_
#define _BLE_CMD_H_

#include "ble_stack_status.h"
#include "ble_host.h"

/**************************************************************************
 * Definitions
 **************************************************************************/
/** @defgroup ble_cmd_definition BLE Command Definition
 * @{
 * @details Here shows the definitions in ble_cmd.h.
 * @ingroup ble_common
 * @}
 * @defgroup adv_definition BLE Advertising Definition
 * @{
 * @details Here shows the advertising related definition.
 * @ingroup ble_cmd_definition
 * @}
 * @defgroup scan_definition BLE Scan Definition
 * @{
 * @ingroup ble_cmd_definition
 * @details Here shows the scan related definition.
 * @}
 * @defgroup conn_definition BLE Connection Parameter Definition
 * @{
 * @ingroup ble_cmd_definition
 * @details Here shows the connection parameter related definition.
 * @}
 **************************************************************************/
/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_cmdEvent BLE Command Event Definition
 * @{
 * @details BLE Command event definition.
 * @name BleCmdEvent
 * @brief Define the different events that could be received by the @ref BleEventCallBack.
 * @{
 * @note The event parameter shall be ignored if the event status is not equal to @ref BLESTACK_STATUS_SUCCESS.
 */
typedef uint8_t BleCmdEvent;

/** Null event.*/
#define BLECMD_EVENT_NULL                                 0x00

/** Event indicates the advertisement is enabled or disabled.
 * @note The parameter field indicates the advertisement information though the @ref BLE_Event_AdvCompleteParam
*/
#define BLECMD_EVENT_ADV_COMPLETE                         0x01

/** Event indicates that receives the advertisement packets.
 * @note The parameter field indicates the received data though the @ref BLE_Event_ScanReportParam
*/
#define BLECMD_EVENT_SCAN_REPORT                          0x02

/** Event indicates the BLE connection is established.
 * @note The parameter field indicates the connection information though the @ref BLE_Event_ConnParam
*/
#define BLECMD_EVENT_CONN_COMPLETE                        0x03

/** Event indicates the BLE connection has been canceled.
 * @note The parameter field is always NULL
*/
#define BLECMD_EVENT_CONN_CANCEL_COMPLETE                 0x04


/** Event indicates the connection parameter update response can be retrieved.
 * @note The parameter field indicates the connection information though the
 * @ref ble_cmd_connUpdateStatus \n
 * @attention The event can be ignored, just wait for @ref BLECMD_EVENT_CONN_UPDATE_COMPLETE event
*/
#define BLECMD_EVENT_CONN_PARAMETER_UPDATE_RSP            0x05


/** Event indicates the BLE connection has been updated.
 * @note The parameter field indicates the connection information though the @ref BLE_Event_ConnUpdateParam
*/
#define BLECMD_EVENT_CONN_UPDATE_COMPLETE                 0x06

/** Event indicates the BLE connection has been disconnected.
 * @note The parameter field indicates the connection information though the @ref BLE_Event_DisconnParam
*/
#define BLECMD_EVENT_DISCONN_COMPLETE                     0x07

/** Event indicates that the Link Layer PHY update procedure is complete.
 * @note The parameter field indicates the Link Layer PHY information though the @ref BLE_Event_PhyUpdateParam
*/
#define BLECMD_EVENT_PHY_UPDATE_COMPLETE                  0x08

/** Event indicates the current Link Layer PHY can be retrieved.
 * @note The parameter field indicates the Link Layer PHY information though the @ref BLE_Event_PhyParam
*/
#define BLECMD_EVENT_PHY_READ_COMPLETE                    0x09

/** Event indicates the RSSI value can be retrieved.
 * @note The parameter field indicates the RSSI value though the @ref BLE_Event_RssiParam
*/
#define BLECMD_EVENT_READ_RSSI_COMPLETE                   0x0A

/** Event indicates the security STK generator method can be retrieved.
 * @note The parameter field indicates the security STK generator information though the @ref BLE_Event_StkGenMethodParam
*/
#define BLECMD_EVENT_STK_GEN_METHOD                       0x0B


/** Event indicates the passkey entry can be retrieved.
 * @note The parameter field indicates the passkey entry information though the @ref BLE_Event_PassKeyConfirmParam
*/
#define BLECMD_EVENT_PASSKEY_CONFIRM                      0x0C

/** Event indicates the authentication status can be retrieved.
 * @note The parameter field indicates the authentication status though the @ref BLE_Event_AuthStatusParam
*/
#define BLECMD_EVENT_AUTH_STATUS                          0x0D

/** Event indicates the scan is enabled or disabled. @note The parameter field is always NULL
*/
#define BLECMD_EVENT_SCAN_COMPLETE                        0x0E


/** Event indicates the connection is creating.
 * @note The parameter field indicates the connection information though the @ref BLE_Event_CreateConnParam
*/
#define BLECMD_EVENT_CREATE_CONNECTION                    0x0F

/** Event indicates the MTU size can be retrieved.
 * @note The parameter field indicates the MTU size though the @ref BLE_Event_MtuParam
*/
#define BLECMD_EVENT_EXCHANGE_MTU_SIZE                    0x10


/** Event indicates the updated data length can be retrieved.
 * @note The parameter field indicates the updated data length though the @ref BLE_Event_DataLengthParam
*/
#define BLECMD_EVENT_DATA_LENGTH_UPDATE                   0x11

/** Event indicates the host attribute database has been parsed.
 * @note The parameter field indicates the updated data length though the @ref BLE_Event_AttDbParsedParam
*/
#define BLECMD_EVENT_ATT_DATABASE_PARSING_FINISHED        0x12

/** @} */
/** @} */



/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_disconnReason BLE Disconnect Reason Definition
 * @{
 * @details BLE disconnect reason definition.
 * @name BleDisConnReason
 * @brief Define the BLE disconnect reason.
 * @{
 */
typedef uint8_t BleDisConnReason;
#define BLEDISCONNREASON_AUTHENTICATION_FAILURE                                       (0x05u)   /**< Authentication failure. */
#define BLEDISCONNREASON_PIN_OR_KEY_MISSING                                           (0x06u)   /**< Encryption key is missing or not saved on the remote device. */
#define BLEDISCONNREASON_CONNECTION_TIMEOUT                                           (0x08u)   /**< Connection timeout. */
#define BLEDISCONNREASON_REMOTE_USER_TERMINATED_CONNECTION                            (0x13u)   /**< Disconnection requested by the remote device users. */
#define BLEDISCONNREASON_REMOTE_DEVICE_TERMINATED_CONNECTION_DUE_TO_LOW_RESOURCES     (0x14u)   /**< Disconnection by the remote device due to low resources. */
#define BLEDISCONNREASON_REMOTE_DEVICE_TERMINATED_CONNECTION_DUE_TO_POWER_OFF         (0x15u)   /**< Disconnection by the remote device due to power off. */
#define BLEDISCONNREASON_CONNECTION_TERMINATED_BY_LOCAL_HOST                          (0x16u)   /**< Disconnection by the local device host stack. */
#define BLEDISCONNREASON_UNSUPPORTED_REMOTE_FEATURE_UNSUPPORTED_LMP_FEATURE           (0x1Au)   /**< Unsupported remote feature. */
#define BLEDISCONNREASON_LMP_RESPONSE_TIMEOUT_LL_RESPONSE_TIMEOUT                     (0x22u)   /**< LMP response timeout. */
#define BLEDISCONNREASON_PAIRING_WITH_UNIT_KEY_NOT_SUPPORTED                          (0x29u)   /**< Pairing with unit key unsupported. */
#define BLEDISCONNREASON_UNACCEPTABLE_CONNECTION_INTERVAL                             (0x3Bu)   /**< Connection interval unacceptable. */
#define BLEDISCONNREASON_CONNECTION_TERMINATED_DUE_TO_MIC_FAILURE                     (0x3Du)   /**< Connection terminated due to MIC failure. */
#define BLEDISCONNREASON_CONNECTION_FAILED_TO_BE_ESTABLISHED                          (0x3Eu)   /**< Connection failed to be established. */
/** @} */
/** @} */


/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_gattRole BLE GATT Role Definition
 * @{
 * @details BLE GATT role.
 * @name BleGATTRole
 * @brief Define BLE GATT role.
 * @{
 */
typedef uint8_t BleGattRole;

/** GATT client is reading or writing data from or to the GATT server. */
#define BLE_GATT_ROLE_CLIENT                  (0x00u)

/** GATT server contains the characteristic database that is being read or written by a GATT client. */
#define BLE_GATT_ROLE_SERVER                  (0x01u)
/** @} */
/** @} */


/**
 * @defgroup ble_cmd_advChannelMap BLE Advertising Channel Definition
 * @{
 * @ingroup adv_definition
 * @details BLE advertising channel definition.
 * @name BleAdvChannelMap
 * @brief Define BLE advertising channel.
 * @{
 */
typedef uint8_t BleAdvChannelMap;

/** Channel 37 */
#define ADV_CHANNEL_37         0x01

/** Channel 38 */
#define ADV_CHANNEL_38         0x02

/** Channel 39 */
#define ADV_CHANNEL_39         0x04

/** Channel 37 and Channel 38 */
#define ADV_CHANNEL_37_38      (ADV_CHANNEL_37 | ADV_CHANNEL_38)

/** Channel 37 and Channel 39 */
#define ADV_CHANNEL_37_39      (ADV_CHANNEL_37 | ADV_CHANNEL_39)

/** Channel 38 and Channel 39 */
#define ADV_CHANNEL_38_39      (ADV_CHANNEL_38 | ADV_CHANNEL_39)

/** Channel 37 and Channel 38 and channel 39 */
#define ADV_CHANNEL_ALL        (ADV_CHANNEL_37 | ADV_CHANNEL_38 | ADV_CHANNEL_39)
/** @} */
/** @} */


/**
 * @defgroup ble_cmd_advFilter BLE Advertising Filter Policy Definition
 * @{
 * @ingroup adv_definition
 * @details BLE advertising filter policy definition.
 * @name BleAdvFilterPolicy
 * @brief Define BLE advertising filter policy.
 * @{
 */
typedef uint8_t BleAdvFilterPolicy;

/** Filter scan requests and connect requests from any device */
#define ADV_FILTER_POLICY_ACCEPT_ALL               0x00

/** Filter scan requests with whitelist,connect requests with from any . */
#define ADV_FILTER_POLICY_ACCEPT_SCAN_REQ_WL       0x01

/** Filter scan requests from any, connect requests with whitelist. */
#define ADV_FILTER_POLICY_ACCEPT_CONN_REQ_WL       0x02

/** Filter both scan and connect requests with whitelist. */
#define ADV_FILTER_POLICY_ACCEPT_SCAN_CONN_REQ_WL  0x03
/** @} */
/** @} */



/**
 * @defgroup ble_cmd_advInterval BLE Advertising Interval Minimum and Maximum Definition
 * @{
 * @ingroup adv_definition
 * @details BLE advertising interval range definition.
 * @name BleAdvInterval
 * @brief  Define BLE advertising interval minimum and maximum.
 * @{
 */
typedef uint8_t BleAdvIntervalRange;

/** The unit of advertising interval is 0.625ms \n
 * Minimum advertising interval is 20ms = (0x20 * 0.625) ms
*/
#define ADV_INTERVAL_MIN              0x0020

/** The unit of advertising interval is 0.625ms \n
 * Maximum advertising interval is 10.24s = (0x4000 * 0.625) ms
*/
#define ADV_INTERVAL_MAX              0x4000
/** @} */
/** @} */


/**
 * @defgroup ble_cmd_advType BLE Advertising Type Definition
 * @{
 * @ingroup adv_definition
 * @details BLE advertising type definition.
 * @name BleAdvType
 * @brief  Define advertising type.
 * @{
 */
typedef uint8_t BleAdvType;
#define ADV_TYPE_ADV_IND              0x00  /**< Connectable and scannable undirected advertising. */
#define ADV_TYPE_ADV_DIRECT_IND       0x01  /**< Connectable directed advertising. */
#define ADV_TYPE_SCAN_IND             0x02  /**< Scanable undirected advertising. */
#define ADV_TYPE_ADV_NONCONN_IND      0x03  /**< Non-Connectable undirected advertising. */
#define ADV_TYPE_SCAN_RSP             0x04  /**< Scan Response. */
/** @} */
/** @} */



/**
 * @defgroup ble_cmd_scanType BLE Scan Type Definition
 * @{
 * @ingroup scan_definition
 * @details BLE scan type definition.
 * @name BleScanType
 * @brief  Define BLE scan type.
 * @{
 */
typedef uint8_t BleScanType;
#define SCAN_TYPE_PASSIVE             0x00  /**< Passive scanning. */
#define SCAN_TYPE_ACTIVE              0x01  /**< Active scanning. */
/** @} */
/** @} */



/**
 * @defgroup ble_cmd_scanInterval BLE Scan Interval Minimum and Maximum Definition
 * @{
 * @ingroup scan_definition
 * @details BLE scan Interval range definition.
 * @name BleScanInterval
 * @brief  Define BLE scan interval minimum and maximum.
 * @{
 */
typedef uint8_t BleScanIntervalRange;

/** The unit of scan interval is 0.625ms \n
 * Minimum scan interval is 2.5ms = (0x0004 * 0.625) ms
*/
#define SCAN_INTERVAL_MIN             0x0004

/** The unit of scan interval is 0.625ms \n
 * Maximum scan interval is 10.24s = (0x4000 * 0.625) ms
*/
#define SCAN_INTERVAL_MAX             0x4000
/** @} */
/** @} */


/**
 * @defgroup ble_cmd_scanWindow BLE Scan Window Minimum and Maximum Definition
 * @{
 * @ingroup scan_definition
 * @details BLE scan window range definition.
 * @name BleScanWindow
 * @brief  Define BLE Scan window minimum and maximum.
 * @{
 */
typedef uint8_t BleScanWindowRange;

/** The unit of scan window is 0.625ms \n
 * Minimum scan window is 2.5ms = (0x0004 * 0.625) ms
*/
#define SCAN_WINDOW_MIN               0x0004

/** The unit of scan window is 0.625ms \n
 * Maximum scan window is 10.24s = (0x4000 * 0.625) ms
*/
#define SCAN_WINDOW_MAX               0x4000
/** @} */
/** @} */


/**
 * @defgroup ble_cmd_scanFilterPolicy BLE Scan Filter Policy Definition
 * @{
 * @ingroup scan_definition
 * @details BLE scan filter policy definition.
 * @name BleScanFilterPolicy
 * @brief  Define BLE scan filter policy.
 * @{
 */
typedef uint8_t BleScanFilterPolicy;

/** Filter advertisement packets except directed advertising and scan response from any device */
#define SCAN_FILTER_POLICY_ACCEPT_ALL               0x00

/** Filter advertisement from the device which is in the white list . */
#define ADV_FILTER_POLICY_ACCEPT_WL                 0x01
/** @} */
/** @} */


/**
 * @defgroup ble_cmd_connInterval BLE Connection Interval Definition
 * @{
 * @ingroup conn_definition
 * @details BLE connection interval range definition.
 * @name BleConnInt
 * @brief  Define BLE connection interval minimum and maximum.
 * @{
 */
typedef uint8_t BleConnectionIntervalRange;

/** The unit of connection interval is 1.25ms \n
 * Minimum connection interval is 7.5ms = (0x0006 * 1.25) ms
*/
#define CONN_INTERVAL_MIN             0x0006

/** The unit of connection interval is 1.25ms \n
 * Maximum connection interval is 4s = (0x0C80 * 1.25) ms
*/
#define CONN_INTERVAL_MAX             0x0C80
/** @} */
/** @} */


/**
 * @defgroup ble_cmd_connLatency BLE Connection Latency Definition
 * @{
 * @ingroup conn_definition
 * @details BLE connection latency range definition.
 * @name BleConnLatency
 * @brief  Define BLE connection latency minimum and maximum.
 * @{
 */
typedef uint8_t BleConnectionLatencyRange;

/** Minimum connection latency is 0 */
#define CONN_LATENCY_MIN              0

/** Maximum connection latency is 0x01F3 */
#define CONN_LATENCY_MAX              0x01F3
/** @} */
/** @} */


/**
 * @defgroup ble_cmd_connSupervisionTimeout BLE Connection Supervision Timeout Range Definition
 * @{
 * @ingroup conn_definition
 * @details BLE connection supervision timeout range definition.
 * @name BleConnSupTimeout
 * @brief  Define BLE connection supervision timeout minimum and maximum.
 * @{
 */
typedef uint8_t BleConnectionSupTimeoutRange;

/** The unit of connection supervision timeout is 10ms \n
 * Minimum connection supervision timeout is 100ms = (0x000A * 10) ms
*/
#define CONN_SUBTIMEOUT_MIN           0x000A

/** The unit of connection supervision timeout is 10ms \n
 * Maximum connection supervision timeout is 32s = (0x0C80 * 10) ms
*/
#define CONN_SUBTIMEOUT_MAX           0x0C80
/** @} */
/** @} */


/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_addrType BLE Address Type Definition
 * @{
 * @details BLE device address type.
 * @name BleAddrType
 * @brief Define different BLE address types.
 * @{
 */
typedef uint8_t BleAddrType;
#define PUBLIC_ADDR                   0x00  /**< Public device address.  */
#define RANDOM_ADDR                   0x01  /**< Random device address.  */
#define PUBLIC_IDENTITY_ADDR          0x02  /**< Public identity address (corresponds to resolved private address). */
#define RANDOM_IDENTITY_ADDR          0x03  /**< Random (static) identity address (corresponds to resolved private address). */

/** @} */
/** @} */


/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_blePhy BLE PHY Definition
 * @{
 * @details BLE PHY definition.
 * @name BlePhy
 * @brief  Define BLE PHY.
 * @{
 */
typedef uint8_t BlePhy;
#define BLE_PHY_1M                    0x01  /**< The transmitter PHY is LE 1M.*/
#define BLE_PHY_2M                    0x02  /**< The transmitter PHY is LE 2M.*/
/** @} */
/** @} */



/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_bleState BLE State Definition
 * @{
 * @details BLE state definition.
 * @name BleState
 * @brief  Define BLE State.
 * @{
 */
typedef uint8_t BleMode;
#define STATE_BLE_STANDBY             0x00  /**< Standby mode.                  */
#define STATE_BLE_ADVERTISING         0x01  /**< Advertising mode.              */
#define STATE_BLE_CONNECTION          0x02  /**< Connection mode.               */
#define STATE_BLE_SCANNING            0x03  /**< Scanning mode.                 */
#define STATE_BLE_INITIATING          0x04  /**< Initialating mode.             */
#define STATE_BLE_CONN_ESTABLISHING   0x05  /**< Connection establishing mode.  */
/** @} */
/** @} */



/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_STK_GenMethod BLE STK Generator Method Definition
 * @{
 * @details BLE STK Generator method definition.
 * @name BleSTK_GenMethod
 * @brief  Define BLE STK generator methods.
 * @{
 */
typedef uint8_t BleSTK_GenMethod;
#define PASSKEY_ENTRY                 0x01  /**< Entry only.   */
#define PASSKEY_DISPLAY               0x02  /**< Display only. */
/** @} */
/** @} */


/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_connUpdateStatus BLE Connection Update Status Definition
 * @{
 * @details BLE connection update status definition.
 * @name BleConnUpdateStatus
 * @brief  Define BLE connection update status.
 * @{
 */
typedef uint8_t BleConnUpdateStatus;
#define CMD_SUCCESS                   0x00  /**< Success.  */
#define CMD_REJECTED                  0x01  /**< Rejected. */
#define CMD_TIMEOUT                   0x02  /**< Timeout.  */
/** @} */
/** @} */


/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_Auth_Status BLE Authentication Status Definition
 * @{
 * @details BLE authentication status definition.
 * @name BleAuthenticationStatus
 * @brief  Define BLE authentication status.
 * @{
 */
typedef uint8_t BleAuthStatus;
#define AUTH_SUCCESS                                 0x00  /**< Success.  */
#define AUTH_ERR_PASSKEY_ENTRY_FAILED                0x01  /**< Passkey entry failed.  */
#define AUTH_ERR_OOB_NOT_AVAILABLE                   0x02  /**< Out of Band key not available.  */
#define AUTH_ERR_AUTHENTICATION_REQUIREMENTS         0x03  /**< Error authentication requirement.  */
#define AUTH_ERR_CONFIRM_VALUE_FAILED                0x04  /**< Confirm value failed.  */
#define AUTH_ERR_PAIRING_NOT_SUPPORTED               0x05  /**< Pairing not supported.  */
#define AUTH_ERR_ENCRYPTION_KEY_SIZE                 0x06  /**< Encryption key size.  */
#define AUTH_ERR_COMMAND_NOT_SUPPORTED               0x07  /**< Command not supported.  */
#define AUTH_ERR_UNSPECIFIED_REASON                  0x08  /**< Unspecified reason.  */
#define AUTH_ERR_REPEATED_ATTEMPTS                   0x09  /**< Too little time has elapsed since last attempt.  */
#define AUTH_ERR_INVALID_PARAMETERS                  0x0A  /**< Invalid parameters.  */
#define AUTH_ERR_DHKEY_CHECK_FAILED                  0x0B  /**< DHKey check failed.  */
#define AUTH_ERR_NUMERIC_COMPARISON_FAILED           0x0C  /**< Numeric comparison failed.  */
#define AUTH_ERR_BR_EDR_PAIRING_IN_PROGRESS          0x0D  /**< BR/EDR paring in progress.  */
#define AUTH_ERR_CROSS_TRANS_KEY_GEN_NOT_ALLOWED     0x0E  /**< BR/EDR Link Key cannot be used for LE keys.  */
#define AUTH_ERR_TIMEOUT                             0x0F  /**< Procedure time out.  */
#define AUTH_ERR_PIN_OR_KEY_MISSING                  0x10  /**< Bonding PIN / KEY missing.  */
/** @} */
/** @} */



/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_PhyUpdateStatus BLE Phy Update Status Definition
 * @{
 * @details BLE Phy update status definition.
 * @name BlePhyUpdateStatus
 * @brief  Define BLE Phy update status.
 * @{
 */
typedef uint8_t BlePhyUpdateStatus;
#define PHY_UPDATE_SUCCESS                           0x00  /**< Success.  */
#define PHY_UPDATE_REMOTE_FEATURE_UNSUPPORTED        0x1A  /**< Command cancel due to remote feature unsupported.  */
#define PHY_UDDATE_TRANSACTION_COLLISION             0x2A  /**< Command cancel due to transaction collision. */

/** @} */
/** @} */



/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_Adv_Complete_Event_Status BLE Advertising Complete Event Status Definition
 * @{
 * @details BLE advertising complete event status definition.
 * @name BleAdvCompleteStatus
 * @brief  Define BLE advertising complete event status.
 * @{
 */
typedef uint8_t BleAdvCompleteStatus;
#define ADV_ENABLE_CMD_SUCCESS                       0x00  /**< Success.  */
#define CMD_REJECTED_DUE_TO_LIMITED_RESOURCES        0x0D  /**< Rejected. */

/** @} */
/** @} */



/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_IOCaps BLE IO Capabilities Definition
 * @{
 * @details BLE IO capabilities definition.
 * @name BleIOCaps
 * @brief  Define BLE IO capabilities.
 * @{
 */
typedef uint8_t IOCaps;
#define DISPLAY_ONLY                                 0x00  /**< Display only.              */
#define DISPLAY_YESNO                                0x01  /**< Display and Yes/No entry.  */
#define KEYBOARD_ONLY                                0x02  /**< Keyboard only.             */
#define NOINPUT_NOOUTPUT                             0x03  /**< No IO capabilities.        */
#define KEYBOARD_DISPLAY                             0x04  /**< Keyboard and display.      */
/** @} */
/** @} */


/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_BondingFlags BLE Bonding Flags Definition
 * @{
 * @details BLE bonding flags definition.
 * @name BondingFlags
 * @brief  Define BLE bonding flags.
 * @{
 */
typedef uint8_t BondingFlags;
#define NO_BONDING                                   0x00  /**< NO Bonding.  */
#define BONDING                                      0x01  /**< Bonding.     */
/** @} */
/** @} */

/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_Conn_status BLE Connection Event Status Definition
 * @{
 * @details BLE connection event status definition.
 * @name BleConnStatus
 * @brief  Define BLE connection event status.
 * @{
 */
typedef uint8_t BleConnStatus;
#define COMMAND_SUCCESS                              0x00  /**< Success.  */
#define COMMAND_DISALLOWED                           0x0C  /**< Disallowed. */
#define COMMAND_REJECTED_DUE_TO_LIMITED_RESOURCES    0x0D  /**< Rejected due to the limited resources. */
#define COMMAND_ADVERTISING_TIMEOUT                  0x3C  /**< Advertising timeout. */

/** @} */
/** @} */



/**
 * @defgroup ble_cccd BLE GATT Client Characteristic Configuration Value Definition
 * @{
 * @details BLE GATT client characteristic configuration value definition.
 * @ingroup ble_cmd_definition
 * @{
 */
typedef uint16_t BleGattCCCDValue;
#define BLEGATT_CCCD_NONE                            0x0000  /**< Disabled notification/ indication.  */
#define BLEGATT_CCCD_NOTIFICATION                    0x0001  /**< The Characteristic shall be notified.  */
#define BLEGATT_CCCD_INDICATION                      0x0002  /**< The Characteristic shall be indicated.  */
#define BLEGATT_CCCD_NOTIFY_INDICATE                 0x0003  /**< The Characteristic shall be both notified and indicated. */
/** @} */
/** @} */



/**
 * @defgroup bleCharWrite BLE GATT Characteristic Value Write Definition
 * @{
 * @details There are five sub-procedures that can be used to write a Characteristic Value.
 * @ingroup ble_cmd_definition
 * @{
 */
typedef uint8_t BleGattWrite;
#define BLEGATT_WRITE                                0x00  /**< GATT write (Write request).  */
#define BLEGATT_WRITE_WITHOUT_RSP                    0x01  /**< GATT write without response (Write command). */
/** @} */
/** @} */


/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_connRole BLE Connection Role Definition
 * @{
 * @details BLE connection role.
 * @name BleConnRole
 * @brief Define BLE connection role.
 * @{
 */
typedef uint8_t BleConnRole;
#define BLEROLE_MASTER                      (0x00u) /**< Master role. */
#define BLEROLE_SLAVE                       (0x01u) /**< Slave role. */
/** @} */
/** @} */



/**************************************************************************
 * Structures
 **************************************************************************/
/** @defgroup ble_cmd_structure BLE Structure Definition
 * @{
 * @details Here shows the structures in ble_cmd.h.
 * @ingroup ble_common
 **************************************************************************/

/**
 * @brief BLE address.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Addr_Param
{
    BleAddrType addrType;             /**< @ref ble_cmd_addrType "BLE address type".  */
    uint8_t     addr[SIZE_BLE_ADDR];  /**< BLE address 48-bit in LSB.  */

} BLE_Addr_Param;


/** @brief BLE advertising parameter.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Adv_Param
{
    /**
     * @ref ble_cmd_advType "BLE advertising type"
     */
    BleAdvType          advType;

    /** Minimum advertising interval. \n
      *  @note Advertising interval = value * 0.625ms \n
      *        Advertising interval range = @ref ADV_INTERVAL_MIN ~ @ref ADV_INTERVAL_MAX
      */
    uint16_t            advIntervalMin;

    /** Maximum advertising  interval.
      *  @note Advertising interval = value * 0.625ms \n
      *        Advertising interval range = @ref ADV_INTERVAL_MIN ~ @ref ADV_INTERVAL_MAX
      */
    uint16_t            advIntervalMax;


    /** The BLE address and BLE address type for directed advertising
     * @note MUST set advDirectAddrParam if BLE advertising type is set to @ref ADV_TYPE_ADV_DIRECT_IND.
    */
    BLE_Addr_Param      advDirectAddrParam;

    /**
     * @ref ble_cmd_advChannelMap "Advertising channel map"
     */
    BleAdvChannelMap    advChannelMap;

    /**
     * @ref ble_cmd_advFilter "Advertising filter policy"
     */
    BleAdvFilterPolicy  advFilterPolicy;
} BLE_Adv_Param;



/** @brief BLE scan parameter.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Scan_Param
{
    /**
     * @ref ble_cmd_scanType "BLE scan type"
     */
    BleScanType           scanType;

    /** Scan interval.
     *  @note Scan interval = value * 0.625ms \n
     *        Scan interval range = @ref SCAN_INTERVAL_MIN ~ @ref SCAN_INTERVAL_MAX
     */
    uint16_t              scanInterval;

    /** Scan window.
     *  @note Scan window = value * 0.625ms \n
     *        Scan window range = @ref SCAN_WINDOW_MIN  ~ @ref SCAN_WINDOW_MAX
     */
    uint16_t              scanWindow;

    /**
     * @ref ble_cmd_scanFilterPolicy "BLE scan filter policy"
     */
    BleScanFilterPolicy   scanFilterPolicy;

} BLE_Scan_Param;



/** @brief BLE connection parameter.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Conn_Param
{
    /** Minimum connection interval.
      *  @note Connection interval = value * 1.25ms \n
      *        Connection interval range = @ref CONN_INTERVAL_MIN ~ @ref CONN_INTERVAL_MAX
      */
    uint16_t connIntervalMin;

    /** Maximum connection interval.
    *  @note Connection interval = value * 1.25ms \n
    *        Connection interval range = @ref CONN_INTERVAL_MIN ~ @ref CONN_INTERVAL_MAX
    */
    uint16_t connIntervalMax;

    /** Slave latency for the connection in number of connection event.
     *  @note Latency range = @ref CONN_LATENCY_MIN  to @ref CONN_LATENCY_MAX
     */
    uint16_t connLatency;

    /** Connection supervision timeout.
     *  @note Supervision timeout = value * 10ms \n
     *        Supervision timeout range = @ref CONN_SUBTIMEOUT_MIN  ~ @ref CONN_SUBTIMEOUT_MAX
     */
    uint16_t connSupervisionTimeout;

} BLE_Conn_Param;



/** @brief BLE IO Capabilities.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_IOCaps_Param
{
    IOCaps          ioCapsParam;              /**< @ref ble_cmd_IOCaps "IO capabilities". */

} BLE_IOCaps_Param;



/** @brief BLE Bonding Flags.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_BondingFlags_Param
{
    BondingFlags     bondingFlags;            /**< @ref ble_cmd_BondingFlags "Bonding flags". */

} BLE_BondingFlags_Param;




/** @brief BLE Data Length.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_DataLength_Param
{
    /** Preferred maximum number of payload octets that the Controller for reception in a single Link Layer packet on this connection.*/
    uint16_t          txMaxOctets;

    /** Preferred maximum number of payload octets that the Controller for transmission in a single Link Layer packet on this connection.*/
    uint16_t          rxMaxOctets;

} BLE_DataLength_Param;


/**
 * @brief BLE Phy Parameters.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Phy_Param
{
    BlePhy             txPhy;                 /**< Tx Phy : @ref BLE_PHY_1M or @ref BLE_PHY_2M */
    BlePhy             rxPhy;                 /**< Rx Phy : @ref BLE_PHY_1M or @ref BLE_PHY_2M  */
} BLE_Phy_Param;


/** @brief BLE Event Attribute Database Parsed Result Parameter.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_AttDbParsed_Param
{
    uint8_t         hostId;                   /**< Host id. */
    uint8_t         result;                   /**< The result of parsing attribute database operation. */
} BLE_Event_AttDbParsed_Param;


/**
 * @brief BLE Event Advertising Completed Parameter.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_AdvCompleteParam
{
    BleAdvCompleteStatus  status;             /**< @ref ble_cmd_Adv_Complete_Event_Status "BLE advertising complete event status." */

} BLE_Event_AdvCompleteParam;



/**
 * @brief BLE Event Connection Parameter.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_ConnParam
{
    BleConnStatus      status;                   /**< @ref ble_cmd_Conn_status "BLE connection event status". */
    uint8_t            hostId;                   /**< Host id. */
    BleConnRole        connRole;                 /**< @ref ble_cmd_connRole "Connection role" */
    BLE_Addr_Param     peerAddr;                 /**< Remote device address type and address. */
    uint16_t           connInterval;             /**< The current link connection interval value. */
    uint16_t           connLatency;              /**< The current link connection latency value. */
    uint16_t           connSupervisionTimeout;   /**< The current link connection supervision timeout value. */

} BLE_Event_ConnParam;


/**
 * @brief BLE Event Connection Update Parameter.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_ConnUpdateParam
{
    BleConnUpdateStatus status;                   /**< @ref ble_cmd_connUpdateStatus "BLE connection update event status." */
    uint8_t             hostId;                   /**< Host id. */
    uint16_t            connInterval;             /**< The current link connection interval value. */
    uint16_t            connLatency;              /**< The current link connection latency value. */
    uint16_t            connSupervisionTimeout;   /**< The current link connection supervision timeout value. */

} BLE_Event_ConnUpdateParam;



/**
 * @brief BLE Event Create Connection Parameter.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_CreateConnParam
{
    BleConnStatus      status;                /**< @ref ble_cmd_Conn_status "BLE connection create event status". */

} BLE_Event_CreateConnParam;


/**
 * @brief BLE Event Scan Report Parameter.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_ScanReportParam
{
    BleAdvType      rptType;                  /**< @ref ble_cmd_addrType "BLE advertising type". */
    BLE_Addr_Param  rptPeerAddr;              /**< Remote device address type and address. */
    uint8_t         rptDataLength;            /**< Received advertising or scan response data length. */
    uint8_t         rptData[31];              /**< Received advertising or scan response data. */
    int8_t          rptRssi;                  /**< Received Signal Strength Indication in dBm. */

} BLE_Event_ScanReportParam;


/**
 * @brief BLE Event Disconnection Parameter.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_DisconnParam
{
    uint8_t            hostId;                /**< Host id. */
    BleDisConnReason   disconnectReason;      /**< @ref ble_cmd_disconnReason "Disconnection reason". */

} BLE_Event_DisconnParam;



/** @brief BLE Event RSSI Parameters.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_RssiParam
{
    uint8_t           hostId;                 /**< Host id. */
    int8_t            rssi;                   /**< RSSI value. */
} BLE_Event_RssiParam;



/**
 * @brief BLE Event STK Generator Method Parameter.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_StkGenMethodParam
{
    uint8_t            hostId;                /**< Host id. */
    BleSTK_GenMethod   stkGenMethod;          /**< @ref ble_cmd_STK_GenMethod "STK generate method." */

} BLE_Event_StkGenMethodParam;



/**
 * @brief BLE Event PassKey Confirmation Parameter.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_PassKeyConfirmParam
{
    uint8_t            hostId;                /**< Host id. */
} BLE_Event_PassKeyConfirmParam;



/**
 * @brief BLE Event Authentication Status Parameter.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_AuthStatusParam
{
    uint8_t            hostId;                /**< Host id. */
    BleAuthStatus      status;                /**< @ref ble_cmd_Auth_Status "BLE authentication event status." */
} BLE_Event_AuthStatusParam;


/**
 * @brief BLE Event Phy Parameter.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_PhyParam
{
    uint8_t            hostId;                /**< Host id. */
    BLE_Phy_Param      phy;                   /**< Tx/ Rx Phy. */

} BLE_Event_PhyParam;


/**
 * @brief BLE Event Phy Update Complete Parameter.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_PhyUpdateParam
{
    BlePhyUpdateStatus     status;            /**< @ref ble_cmd_PhyUpdateStatus "BLE Phy update event status". */
    uint8_t                hostId;            /**< Host id. */
    BLE_Phy_Param          phy;               /**< Tx/Rx Phy. */
} BLE_Event_PhyUpdateParam;




/**
 * @brief BLE Event MTU Parameter.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_MtuParam
{
    uint8_t      hostId;                      /**< Host id. */
    uint16_t     mtuSize;                     /**< MTU size */

} BLE_Event_MtuParam;


/**
 * @brief BLE Event Data Length Update Parameter.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_DataLengthParam
{
    uint8_t                 hostId;           /**< Host id. */
    BLE_DataLength_Param    dataLenparam;     /**< Data length parameter. */

} BLE_Event_DataLengthParam;


/**
 * @brief BLE Event Host Attribute Database Parsed Result Parameter.
 * @ingroup ble_cmd_structure
*/
typedef struct BLE_Event_AttDbParsedParam
{
    BLE_Event_AttDbParsed_Param     attDbParsedResultparam;     /**< Parameter shows the result of parsing attribute database operation. */

} BLE_Event_AttDbParsedParam;

/** @} */


/**************************************************************************
 * Functions
 **************************************************************************/
/** @defgroup ble_cmd_comAPI BLE Command Function
 * @{
 * @details Here shows the function in ble_cmd.h. \n\n
 *          <b>BLE stacks:
 *          - [BLELIB_NOCONN]: advertisement + scan (not support BLE connection).
 *          - [BLELIB_P]     : peripheral (+ server) only.
 *          - [BLELIB_C]     : Central (+ client) only.
 *          - [BLELIB_PC]    : peripheral and Central (Full function). </b>
 * @see @ref ble_BLEAPI
 * @ingroup ble_common
 * @}
 * @defgroup ble_BLEAPI BLE API Comparison
 * @{
 * @ingroup ble_cmd_comAPI
 * @}
 * @defgroup ble_cmd_function BLE Command Function
 * @{
 * @ingroup ble_cmd_comAPI
 * @}
 **************************************************************************/

/** Set BLE Stack Initialization
 *
 * This function is used to initial BLE stack.
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_NOCONN], [BLELIB_P], [BLELIB_C], [BLELIB_PC] </b>
 *
 * @attention MUST initial BLE stack after @ref setRF_Init \n
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE     : RF Phy has NOT initialized.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTCOUNT : Can ONLY be set while in standby state.
 * @retval BLESTACK_STATUS_SUCCESS               : Setting success.
 */
BleStackStatus setBLE_BleStackInit(void);


/** Set BLE Stack Handle Interrupt ISR
 *
 * This function is used to handle BLE stack interrupt.
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_NOCONN], [BLELIB_P], [BLELIB_C], [BLELIB_PC] </b>
 *
 */
void setBLE_BleStackGpio_Isr(void);


/** Set BLE Device Address and Device Address Type
 *
 * This function is used to set BLE Local Device Address and Device Address Type.
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_NOCONN], [BLELIB_P], [BLELIB_C], [BLELIB_PC] </b>
 *
 * @attention MUST set BLE Device Address and Address Type after @ref setRF_Init and @ref setBLE_BleStackInit \n
 *
 * @attention  If the address/ address type change, MUST re-init @ref setBLE_AdvParam "advertisement parameter" or @ref setBLE_ScanParam "scan parameter" or
 *            @ref setBLE_ConnCreate "create connection" to use the updated address/ address type.
 *
 * @note      BLE Address Type : MUST be either @ref PUBLIC_ADDR or @ref RANDOM_ADDR \n
 *            If BLE Address Type is set to @ref RANDOM_ADDR (static random address),
 *            the two most significant bits of the address shall be equal to 1. \n
 *
 * @note      BLE Address : Little Endian and the length is @ref SIZE_BLE_ADDR \n
 *            If Device BLE Address is set to "01:02:03:04:05:06", addrParam->addr[0] = 0x06
 *
 * @param[in] addrParam : a pointer to local device BLE address and BLE address type \n
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : BLE stack has not initialized or can ONLY be set while in standby state.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS           : Setting success.
 */
BleStackStatus setBLE_BleDeviceAddr(BLE_Addr_Param *addrParam);


/** Get BLE Device Address and Device Address Type
 *
 *  This function is used to get BLE Local Device Address and Device Address Type.
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_NOCONN], [BLELIB_P], [BLELIB_C], [BLELIB_PC] </b>
 *
 * @note       BLE Address Type : @ref PUBLIC_ADDR or @ref RANDOM_ADDR \n
 *             BLE Address : Little Endian and the length is @ref SIZE_BLE_ADDR \n
 *             If Device BLE Address is set to "01:02:03:04:05:06", addrParam->addr[0] = 0x06
 *
 * @param[out] addrParam : a pointer to local device BLE address and BLE address type. \n
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : BLE stack has not initialized.
 * @retval BLESTACK_STATUS_SUCCESS           : Setting success.
 */
BleStackStatus getBLE_BleDeviceAddr(BLE_Addr_Param *addrParam);


/** Set BLE Advertising Parameter
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_NOCONN], [BLELIB_P], [BLELIB_PC] </b>
 *
 * @attention MUST set advDirectAddrParam if BLE advertising type is set to @ref ADV_TYPE_ADV_DIRECT_IND.
 *
 * @note    Advertising interval Min. and Max. : @ref ADV_INTERVAL_MIN to @ref ADV_INTERVAL_MAX \n
 *          Advertising interval Min. shall be less than or equal to advertising interval Max.
 *
 * @param[in] advParam : a pointer to advertising parameter
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : BLE stack has not initialized or can ONLY be set while the device is not in advertisement state.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_ERR_BUSY          : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS           : Setting success.
*/
BleStackStatus setBLE_AdvParam(BLE_Adv_Param *advParam);


/** Set BLE Advertising Data
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_NOCONN], [BLELIB_P], [BLELIB_PC]</b>
 *
 * @note BLE advertising data length shall be less than or equal to 31 bytes, the input advData length shall be less than or equal to 32 bytes.
 * @note AD Type please refer to @ref bleGapAdvType
 * @verbatim
   ||--------------------------------------------------------------------------------------------------------------------------------------||
   ||                                         <length octets = (1+alength) octets>                                                         ||
   ||                                                     advData                                                                          ||
   ||--------------------------------------------------------------------------------------------------------------------------------------||
   ||      <1 octet>            ||           <alength octets = (1+slength_1)+...+(1+slength_N)  octets>                                    ||
   ||BLE advertising data length||                        BLE advertising data                                                             ||
   ||--------------------------------------------------------------------------------------------------------------------------------------||
   ||                           ||              AD Structure 1                 ||  ........ ||               AD Structure N                ||
   ||--------------------------------------------------------------------------------------------------------------------------------------||
   ||                           ||<1 octet>||      <slength_1 octets>          ||           ||<1 octet>||    <slength_N octets>            ||
   ||                           ||slength_1||              Data                ||  ........ ||slength_N||              Data                ||
   ||--------------------------------------------------------------------------------------------------------------------------------------||
   ||                           ||         ||<1 octets>||<(slength_1-1) octets>||           ||         ||<1 octets>||<(slength_N-1) octets>||
   ||                           ||         || AD Type  ||     AD Data          ||  ........ ||         || AD Type  ||     AD Data          ||
   ||--------------------------------------------------------------------------------------------------------------------------------------||
   @endverbatim
 * @param[in] advData : a pointer to advertising data.
 * @param[in] length : length of advData.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : BLE stack has not initialized.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_ERR_BUSY          : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS           : Setting success.
*/
BleStackStatus setBLE_AdvData(uint8_t *advData, uint8_t length);


/** Set BLE Scan Response Data
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_NOCONN], [BLELIB_P], [BLELIB_PC] </b>
 *
 * @note    BLE scan response data length shall be less than or equal to 31 bytes, the input scanRspData length shall be less than or equal to 32 bytes.
 *
 * @note AD Type please refer to @ref bleGapAdvType
 * @verbatim
   ||----------------------------------------------------------------------------------------------------------------------------------------||
   ||                                         <length octets = (1+alength) octets>                                                           ||
   ||                                                     scanRspData                                                                        ||
   ||----------------------------------------------------------------------------------------------------------------------------------------||
   ||      <1 octet>              ||               <alength octets = (1+slength_1)+...+(1+slength_N)  octets>                                ||
   ||BLE scan response data length||                            BLE scan response data                                                       ||
   ||----------------------------------------------------------------------------------------------------------------------------------------||
   ||                             ||              AD Structure 1                 ||  ........ ||               AD Structure N                ||
   ||----------------------------------------------------------------------------------------------------------------------------------------||
   ||                             ||<1 octet>||      <slength_1 octets>          ||           ||<1 octet>||    <slength_N octets>            ||
   ||                             ||slength_1||              Data                ||  ........ ||slength_N||              Data                ||
   ||----------------------------------------------------------------------------------------------------------------------------------------||
   ||                             ||         ||<1 octets>||<(slength_1-1) octets>||           ||         ||<1 octets>||<(slength_N-1) octets>||
   ||                             ||         || AD Type  ||     AD Data          ||  ........ ||         || AD Type  ||     AD Data          ||
   ||----------------------------------------------------------------------------------------------------------------------------------------||
   @endverbatim
 *
 * @param[in] scanRspData : a pointer to scan response data.
 * @param[in] length : BLE scan response data.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : BLE stack has not initialized.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_ERR_BUSY          : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS           : Setting success.
*/
BleStackStatus setBLE_ScanRspData(uint8_t *scanRspData, uint8_t length);


/** Set BLE Start Advertising
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_NOCONN], [BLELIB_P], [BLELIB_PC]</b>
 *
 * @par BLE Event
 * Wait for @ref BLECMD_EVENT_ADV_COMPLETE event which indicates the advertisement has been started.
 *
 * @param[in] hostId : the link's host ID or set to @ref BLE_HOSTID_RESERVED to enable @ref ADV_TYPE_ADV_NONCONN_IND or @ref ADV_TYPE_SCAN_IND advertisement.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED  : Command not supported or only @ref ADV_TYPE_ADV_NONCONN_IND or @ref ADV_TYPE_SCAN_IND advertisement can be enabled.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE  : Invalid state usually happens in
 *                                               - BLE stack has not initialized or \n
 *                                               - there is a connection established with the host id or \n
 *                                               - already in advertisement mode.
 * @retval BLESTACK_STATUS_ERR_BUSY           : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS            : Setting success.
 */
BleStackStatus setBLE_AdvEnable(uint8_t hostId);


/** Set BLE Stop Advertising
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_NOCONN], [BLELIB_P], [BLELIB_PC]</b>
 *
 * @par BLE Event
 * Wait for @ref BLECMD_EVENT_ADV_COMPLETE event which indicates the advertisement has been stopped.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED  : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE  : BLE stack has not initialized or can ONLY be set while in advertising state.
 * @retval BLESTACK_STATUS_ERR_BUSY           : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS            : Setting success.
 */
BleStackStatus setBLE_AdvDisable(void);



/** Set BLE Scanning Parameter
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_NOCONN], [BLELIB_C], [BLELIB_PC]</b>
 *
 * @note Scan window can only be less than or equal to the scan interval.
 *
 * @param[in] scanParam : a pointer to scanning parameter.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED  : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE  : BLE stack has not initialized can ONLY be set while the device is not in scanning state.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM  : Invalid parameter.
 * @retval BLESTACK_STATUS_ERR_BUSY           : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS            : Setting success.
 */
BleStackStatus setBLE_ScanParam(BLE_Scan_Param *scanParam);



/** Set BLE Start Scanning
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_NOCONN], [BLELIB_C], [BLELIB_PC]</b>
 *
 * @par BLE Event
 * Wait for @ref BLECMD_EVENT_SCAN_COMPLETE event which indicates the scan has been started. \n
 * Wait for @ref BLECMD_EVENT_SCAN_REPORT event to receive scanned devices information.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : BLE stack has not initialized or already in scanning mode.
 * @retval BLESTACK_STATUS_ERR_BUSY          : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS           : Setting success.
 */
BleStackStatus setBLE_ScanEnable(void);



/** Set BLE Stop Scanning
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_NOCONN], [BLELIB_C], [BLELIB_PC]</b>
 *
 * @par BLE Event
 * Wait for @ref BLECMD_EVENT_SCAN_COMPLETE event which indicates the scan has been stopped.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : BLE stack has not initialized or can ONLY be set while in scanning state.
 * @retval BLESTACK_STATUS_ERR_BUSY          : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS           : Setting success.
 */
BleStackStatus setBLE_ScanDisable(void);



/** Get BLE Advertising Parsing Data By Advertising Data Type
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_NOCONN], [BLELIB_C], [BLELIB_PC]</b>
 *
 * @param[in]  rptData         : scanned advertising data to parse.
 * @param[in]  parsingAdType   : parsing advertising type.
 * @param[out] parsingData     : a pointer to the parsing data.
 * @param[out] parsingDataLen  : a pointer to the length of parsing data.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED  : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM  : Invalid parameter which means error advertising data or there is no data of requested advertising data type.
 * @retval BLESTACK_STATUS_SUCCESS            : Setting success.
 */
BleStackStatus getBLE_GapAdDataByAdType(BLE_Event_ScanReportParam *rptData, BLE_GAP_AD_TYPE parsingAdType, uint8_t *parsingData, uint8_t *parsingDataLen);


/** Set BLE Create Connection
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_C], [BLELIB_PC]</b>
 *
 * @par BLE Event
 * Wait for @ref BLECMD_EVENT_CONN_COMPLETE event which indicates the connection establishing process has been started. \n
 * @attention - The connection process will be stopped until the connection is established or @ref setBLE_ConnCancel is called.
 *
 * @param[in] hostId        : the link's host id.
 * @param[in] peerAddrParam : a pointer to the target peer device address and address type.
 * @param[in] scanParam     : a pointer to scan parameter.
 * @param[in] connParam     : a pointer to connection parameter.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID   : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE    : Invalid state usually happens in
 *                                                - BLE stack has not initialized or \n
 *                                                - there is a connection established with the host id or \n
 *                                                - there is another initialing process in on-going.
 * @retval BLESTACK_STATUS_ERR_BUSY             : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS              : Setting success.
 */
BleStackStatus setBLE_ConnCreate(uint8_t hostId, BLE_Addr_Param *peerAddrParam, BLE_Scan_Param *scanParam, BLE_Conn_Param *connParam);



/** Set BLE Create Connection with the setting of scan parameters by @ref setBLE_ScanParam
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_C], [BLELIB_PC]</b>
 *
 * @par BLE Event
 * Wait for @ref BLECMD_EVENT_CONN_COMPLETE event which indicates the connection establishing process has been started. \n
 * @attention The connection process will be stopped until the connection is established or @ref setBLE_ConnCancel is called.
 *
 * @note it calls @ref setBLE_ConnCreate with the setting of scan parameters by @ref setBLE_ScanParam otherwise the deafult scan parameters are:
 *       - scan type    : @ref SCAN_TYPE_ACTIVE
 *       - scan interval: 10 (10*0.625ms=6.25ms)
 *       - scan window  : 10 10*0.625ms=6.25ms
 *       - scan filter  : @ref SCAN_FILTER_POLICY_ACCEPT_ALL
 *
 * @param[in] hostId        : the link's host id.
 * @param[in] peerAddrParam : a pointer to the target peer device address and address type.
 * @param[in] connParam     : a pointer to connection parameter.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED  : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE  : Invalid state usually happens in
 *                                               - BLE stack has not initialized or \n
 *                                               - there is a connection established with the host id or \n
 *                                               - there is another initialing process in on-going.
 * @retval BLESTACK_STATUS_ERR_BUSY           : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS            : Setting success.
 */
BleStackStatus setBLE_ConnCreateWithSetScanParam(uint8_t hostId, BLE_Addr_Param *peerAddrParam, BLE_Conn_Param *connParam);


/** Set BLE Cancel the On-going Connection Procedure
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_C], [BLELIB_PC]</b>
 *
 * @par BLE Event
 * Wait for @ref BLECMD_EVENT_CONN_CANCEL_COMPLETE event which indicates the connection has been canceled.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : BLE stack has not initialized or can ONLY be set while the connection is establishing.
 * @retval BLESTACK_STATUS_ERR_BUSY          : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS           : Setting success.
 */
BleStackStatus setBLE_ConnCancel(void);


/** Set BLE Terminate the Connection
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_P], [BLELIB_C], [BLELIB_PC]</b>
 *
 * @par BLE Event
 * Wait for @ref BLECMD_EVENT_DISCONN_COMPLETE event which indicates the connection has been terminated.
 *
 * @param[in] HostId : the link's host id.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED  : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE  : BLE stack has not initialized or there is no connection established with the host id.
 * @retval BLESTACK_STATUS_ERR_BUSY           : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS            : Setting success.
 */
BleStackStatus setBLE_Disconnect(uint8_t hostId);


/** BLE Update Connection Parameter
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_P], [BLELIB_C], [BLELIB_PC]</b>
 *
 * @par BLE Event
 * Wait for @ref BLECMD_EVENT_CONN_UPDATE_COMPLETE event which indicates the connection parameter has been updated.
 *
 * @note    Conn Interval Min/Max : @ref CONN_INTERVAL_MIN - @ref CONN_INTERVAL_MAX
 * @note    Conn Latency : @ref CONN_LATENCY_MIN - @ref CONN_LATENCY_MAX
 * @note    Conn Svision Timeout : @ref CONN_SUBTIMEOUT_MIN - @ref CONN_SUBTIMEOUT_MAX \n
 *          The Svision Timeout in milliseconds shall be larger than (1 + connLatency) * CONN_INTERVAL_MAX * 2
 *          , where CONN_INTERVAL_MAX is given in milliseconds.
 *
 * @param[in] connId : the link's Host id.
 * @param[in] connParam : a pointer to connection parameter.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED  : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE  : BLE stack has not initialized or there is no connection established with the host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM  : Invalid parameter.
 * @retval BLESTACK_STATUS_ERR_TIMER_BUSY     : The connection parameter update timer is busy. Meaning there are still outstanding connection parameter update events.
 * @retval BLESTACK_STATUS_ERR_BUSY           : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS            : Setting success.
 */
BleStackStatus setBLE_ConnUpdate(uint8_t hostId, BLE_Conn_Param *connParam);



/** Set BLE TX Power
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_NOCONN], [BLELIB_P], [BLELIB_C], [BLELIB_PC]</b>
 *
 * @attention
 * - Set advertising transmit power MUST before enable advertising.
 * - Set scan transmit power MUST before enable scan.
 * - Set initiator and connection transmit power MUST before connection creating.
 *
 * @note Default TX power is set to the index "0" of "TXPOWER_TABLE" which is defined in "rf_power_definition.c".
 *
 * @par Usage:
 *      If requested advertising TX power is +4 dBm please call API likes
 *      - setBLE_TxPower(TXPOWER_4DBM,STATE_BLE_ADVERTISING);  // "TXPOWER_4DBM" defined in "rf_power_definition.h"
 *
 * @param[in] power_index : requested TX power level in dBm, enter the index of TX power register table. \n
 *                          The TX power register table please refer to "porting_rfpower.c".
 * @param[in] bleMode : BLE mode.
 *                 - @ref STATE_BLE_ADVERTISING :  set advertising and connection transmit power for peripheral role.
 *                 - @ref STATE_BLE_SCANNING    :  set scan transmit power.
 *                 - @ref STATE_BLE_INITIATING  :  set initiator and connection transmit power.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE    : BLE stack has not initialized or in invalid BLE state for central role.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM    : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS              : Setting success.
 */
BleStackStatus setBLE_TxPower(uint8_t power_index, BleMode bleMode);


/** Set BLE TX/ RX PHY
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_P], [BLELIB_C], [BLELIB_PC]</b>
 *
 * @par BLE Event
 * Wait for @ref BLECMD_EVENT_PHY_UPDATE_COMPLETE event which indicates the BLE PHY is updated.
 *
 * @note      The setting of TX PHY must equal to RX PHY.
 *
 * @param[in] hostId : the link's host id.
 * @param[in] blePhy : a pointer to requested @ref BLE_Phy_Param "blePhy".
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED  : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE  : BLE stack has not initialized or there is no connection established with the host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM  : Invalid parameter.
 * @retval BLESTACK_STATUS_ERR_BUSY           : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS            : Setting success.
 */
BleStackStatus setBLE_Phy(uint8_t hostId, BLE_Phy_Param *blePhy);



/** Get BLE TX/ RX PHY
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_P], [BLELIB_C], [BLELIB_PC]</b>
 *
 * @par BLE Event
 * Wait for @ref BLECMD_EVENT_PHY_READ_COMPLETE event to get TX/ RX PHY.
 *
 * @param[in] hostId : the link's host id.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED   : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID  : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE   : BLE stack has not initialized or there is no connection established with the host id.
 * @retval BLESTACK_STATUS_ERR_BUSY            : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS             : Setting success.
 */
BleStackStatus getBLE_Phy(uint8_t hostId);




/** Get BLE RSSI Value
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_P], [BLELIB_C], [BLELIB_PC]</b>
 *
 * @par BLE Event
 * Wait for @ref BLECMD_EVENT_READ_RSSI_COMPLETE event to get last RSSI value.
 *
 * @param[in] hostId : the link's host id.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED   : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID  : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE    : BLE stack has not initialized or there is no connection established with the host id.
 * @retval BLESTACK_STATUS_ERR_BUSY            : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS             : Setting success.
 */
BleStackStatus getBLE_RssiValue(uint8_t hostId);




/** BLE Security Request
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_P], [BLELIB_C], [BLELIB_PC]</b>
 *
 * @par BLE Event
 * Wait for @ref BLECMD_EVENT_AUTH_STATUS event which indicates the authentication status can be retrieved. \n
 *
 * @param[in] hostId     : the link's host id.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED  : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE  : Invalid BLE state, usually happens in \n
 *                                               - BLE stack has not initialized or \n
 *                                               - there is no connection established with the host id \n
 *                                               - the link is already encrypted \n
 *                                               - there is the same procedure has been processed.
 * @retval BLESTACK_STATUS_ERR_TIMER_BUSY     : The Security command timer is busy. Meaning there are still unfinished security events.
 * @retval BLESTACK_STATUS_ERR_BUSY           : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS            : Setting success.
 */
BleStackStatus setBLE_SecurityRequest(uint8_t hostId);


/** Set BLE Pairing PassKey Value
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_P], [BLELIB_C], [BLELIB_PC]</b>
 *
 * Set PassKey value to do pairing process.
 *
 * @param[in] hostId     : the link's host id.
 * @param[in] hexPasskey : passkey for 6-digit code.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED  : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE  : BLE stack has not initialized or there is no connection established with the host id.
 * @retval BLESTACK_STATUS_ERR_BUSY           : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS            : Setting success.
 */
BleStackStatus setBLE_PairingPassKey(uint8_t hostId, uint32_t hexPasskey);




/** Set BLE IO Capabilities
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_P], [BLELIB_C], [BLELIB_PC]</b>
 *
 * @note Shall be issued while in standby state or return @ref BLESTACK_STATUS_ERR_INVALID_STATE.
 *
 * @param[in] ioCapsParam     : a pointer to IO capabilities parameter.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : Invalid state usually happens in BLE stack has not initialized or BLE state is not in standby mode.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS           : Setting success.
 */
BleStackStatus setBLE_IOCapabilities(BLE_IOCaps_Param *ioCapsParam);




/** Set BLE Bonding Flags
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_P], [BLELIB_C], [BLELIB_PC]</b>
 *
 * @attention   if @ref ble_cmd_gattRole "GATT role" is @ref BLE_GATT_ROLE_CLIENT : \n
 *                - Bonding is not supported if the peer device address type is "private device address". \n
 *
 * @note Shall be issued while in standby state or return @ref BLESTACK_STATUS_ERR_INVALID_STATE.
 *
 * @param[in] bondingParam   : a pointer to bonding flags.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : Invalid state usually happens in BLE stack has not initialized or BLE state is not in standby mode.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
 */
BleStackStatus setBLE_BondingFlags(BLE_BondingFlags_Param *bondingParam);



/** Set BLE Data Length Update
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_P], [BLELIB_C], [BLELIB_PC]</b>
 *
 * @par BLE Event
 * Wait for @ref BLECMD_EVENT_DATA_LENGTH_UPDATE event to get updated BLE data length.
 *
 * @note The range of data length is 27~251 bytes.
 *
 * @param[in] hostId       : the link's host id.
 * @param[in] dataLenParam : a pointer to data length parameters.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED   : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID  : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE    : BLE stack has not initialized or there is no connection established with the host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM   : Invalid parameter.
 * @retval BLESTACK_STATUS_ERR_BUSY            : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS             : Setting success.
 */
BleStackStatus setBLE_DataLength(uint8_t hostId, BLE_DataLength_Param *dataLenParam);



/** Set BLE Company ID.
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_P], [BLELIB_C], [BLELIB_PC]</b>
 *
 * @param[in] companyId : company id.
 *
 * @retval BLESTACK_STATUS_SUCCESS  : Setting Completed.
 */
BleStackStatus setBLE_CompanyId(uint16_t companyId);


/**************************************************************************
 * GATT Functions
 **************************************************************************/
/** Get BLE GATT MTU Size
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_P], [BLELIB_C], [BLELIB_PC]</b>
 *
 * @param[in]   hostId  : the link's host id.
 * @param[out]  mtuSize : MTU size.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED   : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID  : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE   : BLE stack has not initialized.
 * @retval BLESTACK_STATUS_SUCCESS             : Setting success.
 */
BleStackStatus getBLEGATT_MtuSize(uint8_t hostId, uint16_t *mtuSize);


/** Get BLE GATT Handle Numbers
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_P], [BLELIB_C], [BLELIB_PC]</b>
 *
 * @param[in]   hostId           : the link's host id.
 * @param[in]   gattRole         : @ref BleGattRole "GATT role"
 * @param[in]   attributeElement : a pointer to service attribute element.
 * @param[out]  handleNumAddr    : a point to handle number table.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED   : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID  : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE   : BLE stack has not initialized.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM   : Invalid parameters.
 * @retval BLESTACK_STATUS_SUCCESS             : Setting success.
 */
BleStackStatus getBLEGATT_HandleNumAddr(uint8_t hostId, BleGattRole gattRole, ATTRIBUTE_BLE *attributeElement, void *handleNumAddr);



/** BLE GATT Get Characteristic Values from Bonded space
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_P], [BLELIB_C], [BLELIB_PC]</b>
 *
 * @attention The link shall be in connection and encryption/ authentication mode or return error code.
 *
 * @param[in]   hostId    : the link's host id.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED           : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID          : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_DB_PARSING_IN_PROGRESS  : Service/ characteristic discovering or authentication/ authorization have not completed.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE           : BLE stack has not initialized or there is no connection established with the host id.
 * @retval BLESTACK_STATUS_SUCCESS                     : Setting success.
 */
BleStackStatus getBLEGATT_RestoreCCCDValueFromBond(uint8_t hostId);


/**************************************************************************
 * GATT Client Functions
 **************************************************************************/
/** BLE Re-parse attribute database
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_C], [BLELIB_PC]</b>
 *
 * @par BLE Event
 * Wait for @ref BLECMD_EVENT_ATT_DATABASE_PARSING_FINISHED event to get the result of the process. \n
 *
 * @note This function supports for GATT role @ref BLE_GATT_ROLE_CLIENT only.
 *
 * @param[in]   hostId: the link's host id.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED            : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID           : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE            : BLE stack has not initialized or there is no connection established with the host id.
 * @retval BLESTACK_STATUS_ERR_DB_PARSING_IN_PROGRESS   : Already in Attribute database parsing process.
 * @retval BLESTACK_STATUS_SUCCESS                      : Setting success.
 */
BleStackStatus setBLEGATT_ReparseAttDatabase(uint8_t hostId);


/** Set BLE Exchange MTU Request
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_C], [BLELIB_PC]</b>
 *
 * @par BLE Event
 * Wait for @ref BLECMD_EVENT_EXCHANGE_MTU_SIZE event to get exchanged MTU size. \n
 *
 * @attention This request shall only be sent once during a connection.
 *
 * @note The range of input "clientRxMtu" is 23 ~ 247 bytes.
 * @note This function supports for GATT role @ref BLE_GATT_ROLE_CLIENT only.
 *
 * @param[in] hostId        : the link's host id.
 * @param[in] clientRxMtu   : client maximum receive MTU size.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED            : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID           : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_DB_PARSING_IN_PROGRESS   : Service/ characteristic discovering or authentication/ authorization have not completed.
 * @retval BLESTACK_STATUS_ERR_INVALID_CMD              : Invalid command, usually happens in requesting the command over 1 times.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE            : BLE stack has not initialized or there is no connection established with the host id.
 * @retval BLESTACK_STATUS_ERR_BUSY                     : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS                      : Setting success.
 */
BleStackStatus setBLEGATT_ExchangeMtuRequest(uint8_t hostId, uint16_t clientRxMtu);


/** BLE GATT Read Characteristic Value (Read Request)
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_C], [BLELIB_PC]</b>
 *
 * @par BLE Event
 * Wait for @ref OPCODE_ATT_READ_RESPONSE event if registered service/characteristic callback function.
 *
 * @note Read a Characteristic Value from a server when the client knows the Characteristic Value Handle.
 * @note This function supports for GATT role @ref BLE_GATT_ROLE_CLIENT only.
 *
 * @param[in]   hostId    : the link's host id.
 * @param[in]   hdlNum    : characteristic value handle.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED                  : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID                 : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED                  : Invalid GATT role or no reading attribute definition, only supports for client.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE                  : BLE stack has not initialized or there is no connection established with the host id.
 * @retval BLESTACK_STATUS_ERR_DB_PARSING_IN_PROGRESS         : Service/ characteristic discovering or authentication/ authorization have not completed.
 * @retval BLESTACK_STATUS_ERR_INVALID_HANDLE                 : Invalid handle.
 * @retval BLESTACK_STATUS_ERR_SEQUENTIAL_PROTOCOL_VIOLATION  : Another GATT request already in progress please wait and retry.
 * @retval BLESTACK_STATUS_ERR_BUSY                           : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS                            : Setting success.
 */
BleStackStatus setBLEGATT_ReadCharacteristicValue(uint8_t hostId, uint16_t hdlNum);


/** BLE GATT Read Long Characteristic Value (Read Blob Request)
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_C], [BLELIB_PC]</b>
 *
 * @par BLE Event
 * Wait for @ref OPCODE_ATT_READ_BLOB_RESPONSE event if registered service/characteristic callback function.
 *
 * @note read part of the value of an attribute at a given offset from a server when the client knows the Characteristic Value Handle.
 * @note This function supports for GATT role @ref BLE_GATT_ROLE_CLIENT only.
 *
 * @param[in]   hostId    : the link's host id.
 * @param[in]   hdlNum    : characteristic value handle.
 * @param[in]   hdlNum    : characteristic value handle.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED                  : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID                 : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED                  : Invalid GATT role or no reading attribute definition, only supports for client.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE                  : BLE stack has not initialized or there is no connection established with the host id.
 * @retval BLESTACK_STATUS_ERR_DB_PARSING_IN_PROGRESS         : Service/ characteristic discovering or authentication/ authorization have not completed.
 * @retval BLESTACK_STATUS_ERR_INVALID_HANDLE                 : Invalid handle.
 * @retval BLESTACK_STATUS_ERR_SEQUENTIAL_PROTOCOL_VIOLATION  : Another GATT request already in progress please wait and retry.
 * @retval BLESTACK_STATUS_ERR_BUSY                           : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS                            : Setting success.
 */
BleStackStatus setBLEGATT_ReadLongCharacteristicValue(uint8_t hostId, uint16_t hdlNum, uint16_t Offset);


/** BLE GATT Configure CCCD(Client Characteristic Configuration Descriptor)
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_C], [BLELIB_PC]</b>
 *
 * @par BLE Event
 * Wait for @ref OPCODE_ATT_WRITE_RESPONSE event if registered service/characteristic callback function.
 *
 * @note This function supports for GATT role @ref BLE_GATT_ROLE_CLIENT only.
 *
 * @param[in]   hostId    : the link's host id.
 * @param[in]   hdlNum    : cccd handle number.
 * @param[in]   cccdVal   : @ref BleGattCCCDValue "CCCD value"
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED                  : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID                 : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM                  : Invalid parameter.
 * @retval BLESTACK_STATUS_ERR_DB_PARSING_IN_PROGRESS         : Service/ characteristic discovering or authentication/ authorization have not completed.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE                  : BLE stack has not initialized or there is no connection established with the host id.
 * @retval BLESTACK_STATUS_ERR_SEQUENTIAL_PROTOCOL_VIOLATION  : Another GATT request already in progress please wait and retry.
 * @retval BLESTACK_STATUS_ERR_INVALID_HANDLE                 : Invalid handle.
 * @retval BLESTACK_STATUS_ERR_BUSY                           : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS                            : Setting success.
 */
BleStackStatus setBLEGATT_ConfigCCCD(uint8_t hostId, uint16_t hdlNum, BleGattCCCDValue cccdVal);



/** BLE GATT Write A Characteristic Value to A Server (Write Request)
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_C], [BLELIB_PC]</b>
 *
 * @par BLE Event
 * Wait for @ref OPCODE_ATT_WRITE_RESPONSE event if registered service/characteristic callback function.
 *
 * @note The Write Request is used to request the server to write the value of an attribute and acknowledge that this has been achieved in a Write Response. \n
 * A Write Response shall be sent by the server if the write of the Characteristic Value succeeded.
 * @note This function supports for GATT role @ref BLE_GATT_ROLE_CLIENT only.
 *
 * @param[in]   hostId    : the link's host id.
 * @param[in]   hdlNum    : characteristic value handle.
 * @param[in]   data      : data to transmit.
 * @param[in]   length    : the length of data.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED                  : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID                 : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE                  : BLE stack has not initialized or there is no connection established with the host id.
 * @retval BLESTACK_STATUS_ERR_DB_PARSING_IN_PROGRESS         : Service/ characteristic discovering or authentication/ authorization have not completed.
 * @retval BLESTACK_STATUS_ERR_INVALID_HANDLE                 : Invalid handle.
 * @retval BLESTACK_STATUS_ERR_SEQUENTIAL_PROTOCOL_VIOLATION  : Another GATT request already in progress please wait and retry.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM                  : Invalid parameter, please check the maximum length of data.
 * @retval BLESTACK_STATUS_ERR_BUSY                           : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS                            : Setting success.
 */
BleStackStatus setBLEGATT_Write(uint8_t hostId, uint16_t hdlNum, uint8_t *data, uint16_t length);



/** BLE GATT Write A Characteristic Value to A Server (Write Without Response Request)
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_C], [BLELIB_PC]</b>
 *
 * @note Write a Characteristic Value to a server when the client knows the Characteristic Value Handle \n
 *       and the client does not need an acknowledgment that the write was successfully performed.
 * @note This function supports for GATT role @ref BLE_GATT_ROLE_CLIENT only.
 *
 * @param[in]   hostId    : the link's host id.
 * @param[in]   hdlNum    : characteristic value handle.
 * @param[in]   data      : data to transmit.
 * @param[in]   length    : the length of data.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED                  : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID                 : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE                  : BLE stack has not initialized or there is no connection established with the host id.
 * @retval BLESTACK_STATUS_ERR_DB_PARSING_IN_PROGRESS         : Service/ characteristic discovering or authentication/ authorization have not completed.
 * @retval BLESTACK_STATUS_ERR_INVALID_HANDLE                 : Invalid handle.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM                  : Invalid parameter, please check the maximum length of data.
 * @retval BLESTACK_STATUS_ERR_BUSY                           : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS                            : Setting success.
 */
BleStackStatus setBLEGATT_WriteWithoutRsp(uint8_t hostId, uint16_t hdlNum, uint8_t *data, uint16_t length);



/** BLE GATT Prepare Write A Characteristic Value to A Server Data Queue (Prepare Write Request)
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_C], [BLELIB_PC]</b>
 *
 * @par BLE Event
 * Wait for @ref OPCODE_ATT_PREPARE_WRITE_RESPONSE event if registered service/characteristic callback function.
 *
 * @note The Prepare Write Request is used to request the server to prepare to write the value of an attribute. The server will respond to this request with a Prepare Write Response, so that the client can verify that the value was received correctly. \n
 * A Prepare Write Response shall be sent by the server if the write of the Data Queue succeeded.
 * @note This function supports for GATT role @ref BLE_GATT_ROLE_CLIENT only.
 *
 * @param[in]   hostId    : the link's host id.
 * @param[in]   hdlNum    : characteristic value handle.
 * @param[in]   data      : data to transmit (with a 16bit Value Offset at beginning). \n
 *                          i.e. offset = 0x01, data = [0x0A,0x0B]
 *                               data = [0x01,0x00,0x0A,0x0B]
 * @param[in]   length    : the length of data (include the 16bit Value Offset, 2 byte).
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED                  : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID                 : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE                  : BLE stack has not initialized or there is no connection established with the host id.
 * @retval BLESTACK_STATUS_ERR_DB_PARSING_IN_PROGRESS         : Service/ characteristic discovering or authentication/ authorization have not completed.
 * @retval BLESTACK_STATUS_ERR_INVALID_HANDLE                 : Invalid handle.
 * @retval BLESTACK_STATUS_ERR_SEQUENTIAL_PROTOCOL_VIOLATION  : Another GATT request already in progress please wait and retry.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM                  : Invalid parameter, please check the maximum length of data.
 * @retval BLESTACK_STATUS_ERR_BUSY                           : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS                            : Setting success.
 */
BleStackStatus setBLEGATT_PrepareWrite(uint8_t hostId, uint16_t hdlNum, uint8_t *data, uint16_t length);



/** BLE GATT Execute Write A Characteristic Value to A Server Data Queue (Execute Write Request)
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_C], [BLELIB_PC]</b>
 *
 * @par BLE Event
 * Wait for @ref OPCODE_ATT_EXECUTE_WRITE_RESPONSE event if registered service/characteristic callback function.
 *
 * @note The Execute Write Request is used to request the server to write or cancel the write of all the prepared values currently held in the prepare queue from this client. \n
 * The Execute Write Response shall be sent after the attributes are written with the Data Queue succeeded.
 *
 * @note This function supports for GATT role @ref BLE_GATT_ROLE_CLIENT only.
 *
 * @param[in]   hostId    : the link's host id.
 * @param[in]   hdlNum    : characteristic value handle.
 * @param[in]   flag      : 0x00: all pending prepare write values shall be discarded for this client, \n
 *                          0x01: writ values that were queued by the previous prepare write requests.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED                  : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID                 : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE                  : BLE stack has not initialized or there is no connection established with the host id.
 * @retval BLESTACK_STATUS_ERR_DB_PARSING_IN_PROGRESS         : Service/ characteristic discovering or authentication/ authorization have not completed.
 * @retval BLESTACK_STATUS_ERR_INVALID_HANDLE                 : Invalid handle.
 * @retval BLESTACK_STATUS_ERR_SEQUENTIAL_PROTOCOL_VIOLATION  : Another GATT request already in progress please wait and retry.
 * @retval BLESTACK_STATUS_ERR_BUSY                           : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS                            : Setting success.
 */
BleStackStatus setBLEGATT_ExecuteWrite(uint8_t hostId, uint16_t hdlNum, uint8_t flag);

/**************************************************************************
 * GATT Server Functions
 **************************************************************************/
/** Set BLE Preferred MTU Size
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_P], [BLELIB_PC]</b>
 *
 * @attention The default BLE Preferred MTU Size is set to 23.
 *
 * @note The range of input "preferredRxMtu" is 23 ~ 247 bytes.
 * @note This function supports for GATT role @ref BLE_GATT_ROLE_SERVER only.
 *
 * @param[in] hostId          : the link's host id.
 * @param[in] preferredRxMtu  : preferred Rx MTU size.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED  : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE  : BLE stack has not initialized or can ONLY be set while in connection state.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM  : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS            : Setting success.
 */
BleStackStatus setBLEGATT_PreferredMtuSize(uint8_t hostId, uint16_t preferredRxMtu);


/** BLE GATT Set Characteristic Read By Type Response
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_P], [BLELIB_PC]</b>
 *
 * @note The read by type response is sent in reply to a received Read By Type Request and contains the handle number and value of the attribute that has been read.
 * @note This function supports for GATT role @ref BLE_GATT_ROLE_SERVER only.
 *
 * @param[in]   hostId    : the link's host id.
 * @param[in]   hdlNum    : characteristic value handle.
 * @param[in]   data      : data to set.
 * @param[in]   length    : the length of data.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED           : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID          : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_DB_PARSING_IN_PROGRESS  : Service/ characteristic discovering or authentication/ authorization have not completed.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE           : BLE stack has not initialized or there is no connection established with the host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_HANDLE          : Invalid handle.
 * @retval BLESTACK_STATUS_ERR_BUSY                    : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS                     : Setting success.
 */
BleStackStatus setBLEGATT_ReadByTypeRsp(uint8_t hostId, uint16_t hdlNum, uint8_t *data, uint16_t length);


/** BLE GATT Set Characteristic Read Response
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_P], [BLELIB_PC]</b>
 *
 * @note The read response is sent in reply to a received Read Request and contains the value of the attribute that has been read.
 * @note This function supports for GATT role @ref BLE_GATT_ROLE_SERVER only.
 *
 * @param[in]   hostId    : the link's host id.
 * @param[in]   hdlNum    : characteristic value handle.
 * @param[in]   data      : data to set.
 * @param[in]   length    : the length of data.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED           : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID          : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_DB_PARSING_IN_PROGRESS  : Service/ characteristic discovering or authentication/ authorization have not completed.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE           : BLE stack has not initialized or there is no connection established with the host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_HANDLE          : Invalid handle.
 * @retval BLESTACK_STATUS_ERR_BUSY                    : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS                     : Setting success.
 */
BleStackStatus setBLEGATT_ReadRsp(uint8_t hostId, uint16_t hdlNum, uint8_t *data, uint16_t length);



/** BLE GATT Automatically Set Characteristic Read Response or Read By Type Response
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_P], [BLELIB_PC]</b>
 *
 * @note This function supports for GATT role @ref BLE_GATT_ROLE_SERVER only.
 *
 * @param[in]   hostId    : the link's host id.
 * @param[in]   hdlNum    : characteristic value handle.
 * @param[in]   data      : data to set.
 * @param[in]   length    : the length of data.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED           : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID          : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_DB_PARSING_IN_PROGRESS  : Service/ characteristic discovering or authentication/ authorization have not completed.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE           : BLE stack has not initialized or there is no connection established with the host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_HANDLE          : Invalid handle.
 * @retval BLESTACK_STATUS_ERR_BUSY                    : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS                     : Setting success.
 */
BleStackStatus setBLEGATT_GeneralReadRsp(uint8_t hostId, uint16_t hdlNum, uint8_t *data, uint16_t length);


/** BLE GATT Set Characteristic Read Blob Response
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_P], [BLELIB_PC]</b>
 *
 * @note The read blob response is sent in reply to a received Read Blob Request and contains the value of the attribute that has been read.
 * @note This function supports for GATT role @ref BLE_GATT_ROLE_SERVER only.
 *
 * @param[in]   hostId    : the link's host id.
 * @param[in]   hdlNum    : characteristic value handle.
 * @param[in]   data      : data to set.
 * @param[in]   length    : the length of data.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED           : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID          : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_DB_PARSING_IN_PROGRESS  : Service/ characteristic discovering or authentication/ authorization have not completed.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE           : BLE stack has not initialized or there is no connection established with the host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_HANDLE          : Invalid handle.
 * @retval BLESTACK_STATUS_ERR_BUSY                    : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS                     : Setting success.
 */
BleStackStatus setBLEGATT_ReadBlobRsp(uint8_t hostId, uint16_t hdlNum, uint8_t *data, uint16_t length);


/** BLE GATT Indicate the Attribute Value
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_P], [BLELIB_PC]</b>
 *
 * @par BLE Event
 * Wait for @ref OPCODE_ATT_HANDlE_VAlUE_CONFIRMATION event if registered service/characteristic callback function.
 *
 * @note When a server is configured to indicate a Characteristic Value to a client and expects the indication was successfully received.
 * @note This function supports for GATT role @ref BLE_GATT_ROLE_SERVER only.
 *
 * @param[in]   hostId    : the link's host id.
 * @param[in]   hdlNum    : characteristic value handle.
 * @param[in]   data      : data to transmit.
 * @param[in]   length    : the length of data.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED                  : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID                 : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE                  : BLE stack has not initialized or there is no connection established with the host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_HANDLE                 : Invalid handle.
 * @retval BLESTACK_STATUS_ERR_DB_PARSING_IN_PROGRESS         : Service/ characteristic discovering or authentication/ authorization have not completed.
 * @retval BLESTACK_STATUS_ERR_SEQUENTIAL_PROTOCOL_VIOLATION  : Another GATT request already in progress please wait and retry.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM                  : Invalid parameter, please check the maximum length of data.
 * @retval BLESTACK_STATUS_ERR_BUSY                           : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS                            : Setting success.
 */
BleStackStatus setBLEGATT_Indication(uint8_t hostId, uint16_t hdlNum, uint8_t *data, uint16_t length);



/** BLE GATT Notify the Attribute Value
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_P], [BLELIB_PC]</b>
 *
 * @note When a server is configured to notify a Characteristic Value to a client without the acknowledgment that the notification was successfully received.
 * @note This function supports for GATT role @ref BLE_GATT_ROLE_SERVER only.
 *
 * @param[in]   hostId    : the link's host id.
 * @param[in]   hdlNum    : characteristic value handle.
 * @param[in]   data      : data to transmit.
 * @param[in]   length    : the length of data.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED                  : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID                 : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE                  : BLE stack has not initialized or there is no connection established with the host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_HANDLE                 : Invalid handle.
 * @retval BLESTACK_STATUS_ERR_DB_PARSING_IN_PROGRESS         : Service/ characteristic discovering or authentication/ authorization have not completed.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM                  : Invalid parameter, please check the maximum length of data.
 * @retval BLESTACK_STATUS_ERR_BUSY                           : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS                            : Setting success.
 */
BleStackStatus setBLEGATT_Notification(uint8_t hostId, uint16_t hdlNum, uint8_t *data, uint16_t length);


/** BLE GATT Set Error Response
 *
 * @ingroup ble_cmd_function
 *
 * @remark <b>[BLELIB_P], [BLELIB_PC]</b>
 *
 * @note The Error Response is used to state that a given request cannot be performed, and to provide the reason.
 *
 * @param[in] hostId  : the link's host id.
 * @param[in] hdlNum  : attribute handle in error.
 * @param[in] opcode  : attribute opcode in error.
 * @param[in] errRsp  : error code.
 *
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED           : Command not supported.
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID          : Invalid host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE           : BLE stack has not initialized or there is no connection established with the host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_HANDLE          : Invalid handle.
 * @retval BLESTACK_STATUS_ERR_DB_PARSING_IN_PROGRESS  : Service/ characteristic discovering or authentication/ authorization have not completed.
 * @retval BLESTACK_STATUS_ERR_BUSY                    : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS                     : Setting success.
 */
BleStackStatus setBLEGATT_ErrorRsp(uint8_t hostId, uint16_t hdlNum, BleAttOpcode opcode, BleAttErrorRsp errRsp);

#endif // _BLE_CMD_H_


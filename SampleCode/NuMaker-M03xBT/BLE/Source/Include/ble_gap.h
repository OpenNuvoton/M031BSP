/**************************************************************************//**
* @file       ble_gap.h
* @brief      Provide the Definition of BLE Generic Access Profile (GAP).
*
*
* @defgroup ble_gap BLE GAP Definition
* @{
* @details  Common definitions for the Generic Access Profile (GAP). (ble_gap.h).
* @}
*****************************************************************************/

#ifndef _BLE_GAP_H_
#define _BLE_GAP_H_


/** @brief BLE Address Length.
 * @ingroup ble_gap
*/
#define SIZE_BLE_ADDR                                                           6


/**
 * @defgroup bleGapAdvTypeLen BLE Gap Data Type Pre-defined Length
 * @{
 * @ingroup ble_gap
 * @details  BLE Gap data type pre-defined length.
 */
#define GAP_AD_TYPE_LENGTH_0                                                    0
#define GAP_AD_TYPE_LENGTH_1                                                    1
#define GAP_AD_TYPE_LENGTH_2                                                    2
#define GAP_AD_TYPE_LENGTH_3                                                    3
#define GAP_AD_TYPE_LENGTH_4                                                    4
#define GAP_AD_TYPE_LENGTH_5                                                    5
#define GAP_AD_TYPE_LENGTH_6                                                    6
#define GAP_AD_TYPE_LENGTH_7                                                    7
#define GAP_AD_TYPE_LENGTH_8                                                    8
#define GAP_AD_TYPE_LENGTH_9                                                    9
#define GAP_AD_TYPE_LENGTH_10                                                   10
#define GAP_AD_TYPE_LENGTH_11                                                   11
#define GAP_AD_TYPE_LENGTH_12                                                   12
#define GAP_AD_TYPE_LENGTH_13                                                   13
#define GAP_AD_TYPE_LENGTH_14                                                   14
#define GAP_AD_TYPE_LENGTH_15                                                   15
/** @} */


/**
 * @defgroup bleGapAdvType BLE Gap Data Type Formats
 * @{
 * @ingroup ble_gap
 * @details  BLE Gap data type format.
 */
//Bluetooth Spec. Ver4.0 [Vol 3] page 401~405 of 656
//https://www.bluetooth.org/en-us/specification/assigned-numbers/generic-access-profile

typedef uint8_t BLE_GAP_AD_TYPE;
#define GAP_AD_TYPE_FLAGS                                                       0x01  /**< @ref bleGapFlags "Flags for discoverability." */
#define GAP_AD_TYPE_SERVICE_MORE_16B_UUID                                       0x02  /**< Partial list of 16 bit service UUIDs. */
#define GAP_AD_TYPE_SERVICE_CPLT_16B_UUID                                       0x03  /**< Complete list of 16 bit service UUIDs. */
#define GAP_AD_TYPE_SERVICE_MORE_32B_UUID                                       0x04  /**< Partial list of 32 bit service UUIDs. */
#define GAP_AD_TYPE_SERVICE_CPLT_32B_UUID                                       0x05  /**< Complete list of 32 bit service UUIDs. */
#define GAP_AD_TYPE_SERVICE_MORE_128B_UUID                                      0x06  /**< Partial list of 128 bit service UUIDs. */
#define GAP_AD_TYPE_SERVICE_CPLT_128B_UUID                                      0x07  /**< Complete list of 128 bit service UUIDs. */
#define GAP_AD_TYPE_LOCAL_NAME_SHORTENED                                        0x08  /**< Short local device name. */
#define GAP_AD_TYPE_LOCAL_NAME_COMPLETE                                         0x09  /**< Complete local device name. */
#define GAP_AD_TYPE_TX_POWER_LEVEL                                              0x0A  /**< Transmit power level, 1byte : 0xXX:-127 to +127dBm */
#define GAP_AD_TYPE_SIMPLE_PAIRING_OPT_OOB_CLASS_OF_DEVICE                      0x0D  /**< Class of device. */
#define GAP_AD_TYPE_SIMPLE_PAIRING_OPT_OOB_HASH_C                               0x0E  /**< Simple pairing hash C. */
#define GAP_AD_TYPE_SIMPLE_PAIRING_OPT_OOB_RANDOMIZER_R                         0x0F  /**< Simple pairing randomizer R. */
#define GAP_AD_TYPE_SECURITY_MANAGER_TK_VALUE                                   0x10  /**< Security manager TK value. */
#define GAP_AD_TYPE_SECURITY_MANAGER_OOB_FLAGS                                  0x11  /**< Security manager out of band flags. */
#define GAP_AD_TYPE_SLAVE_CONNECTION_INTERVAL_RANGE                             0x12  /**< Slave connection interval range. */
#define GAP_AD_TYPE_SERVICE_SOLICITATION_16B_UUID                               0x14  /**< List of 16-bit service solicitation UUIDs. */
#define GAP_AD_TYPE_SERVICE_SOLICITATION_128B_UUID                              0x15  /**< List of 128-bit service solicitation UUIDs. */
#define GAP_AD_TYPE_SERVICE_DATA                                                0x16  /**< Service sata - 16-bit UUID. */
#define GAP_AD_TYPE_PUBLIC_TARGET_ADDRESS                                       0x17  /**< Public target address. */
#define GAP_AD_TYPE_RANDOM_TARGET_ADDRESS                                       0x18  /**< Random target address. */
#define GAP_AD_TYPE_APPEARANCE                                                  0x19  /**< Appearance. */
#define GAP_AD_TYPE_ADVERTISING_INTERVAL                                        0x1A  /**< Advertising interval. */
#define GAP_AD_TYPE_3D_INFORMATION_DATA                                         0x3D  /**< 3D information data. */
#define GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA                                  0xFF  /**< Manufacturer specific data. */
/** @} */


/**
 * @defgroup bleGapFlags BLE Gap Flags Mode
 * @{
 * @ingroup ble_gap
 * @details  BLE Gap flags mode.
 */
//Bluetooth CSS. v9 page 12 of 37

#define GAP_FLAGS_LIMITED_DISCOVERABLE_MODE                                     0x01  /**< LE limited discoverable mode */
#define GAP_FLAGS_GENERAL_DISCOVERABLE_MODE                                     0x02  /**< LE general discoverable mode */
#define GAP_FLAGS_BR_EDR_NOT_SUPPORTED                                          0x04  /**< BR/EDR not supported. */
#define GAP_FLAGS_SIMUL_DEV_CAP_CONTROLLER                                      0x08  /**< Simultaneous LE and BR/EDR to same device capable (Controller). */
#define GAP_FLAGS_SIMUL_DEV_CAP_HOST                                            0x10  /**< Simultaneous LE and BR/EDR to same device capable (Host). */
#define BLE_GAP_FLAGS_LIMITED_DISCOVERABLE_MODE                                 (GAP_FLAGS_LIMITED_DISCOVERABLE_MODE|GAP_FLAGS_BR_EDR_NOT_SUPPORTED)  /**< LE limited discoverable mode, BR/EDR not supported. */
#define BLE_GAP_FLAGS_GENERAL_DISCOVERABLE_MODE                                 (GAP_FLAGS_GENERAL_DISCOVERABLE_MODE|GAP_FLAGS_BR_EDR_NOT_SUPPORTED)  /**< LE general discoverable mode, BR/EDR not supported. */
/** @} */



/** @defgroup bleGapAppearance BLE GAP Appearance values
 * @ingroup ble_gap
 * @note https://www.bluetooth.org/en-us/specification/assigned-numbers
 * @{ */
#define BLE_APPEARANCE_UNKNOWN                                                  0x0000  /**< Unknown. */
#define BLE_APPEARANCE_GENERIC_PHONE                                            0x0040  /**< Generic Phone. */
#define BLE_APPEARANCE_GENERIC_COMPUTER                                         0x0080  /**< Generic Computer. */
#define BLE_APPEARANCE_GENERIC_WATCH                                            0x00C0  /**< Generic Watch. */
#define BLE_APPEARANCE_WATCH_SPORTS_WATCH                                       0x00C1  /**< Watch, Sports Watch. */
#define BLE_APPEARANCE_GENERIC_CLOCK                                            0x0100  /**< Generic Clock. */
#define BLE_APPEARANCE_GENERIC_DISPLAY                                          0x0140  /**< Generic Display. */
#define BLE_APPEARANCE_GENERIC_REMOTE_CONTROL                                   0x0180  /**< Generic Remote Control. */
#define BLE_APPEARANCE_GENERIC_EYE_GLASSES                                      0x01C0  /**< Generic Eye-glasses. */
#define BLE_APPEARANCE_GENERIC_TAG                                              0x0200  /**< Generic Tag. */
#define BLE_APPEARANCE_GENERIC_KEYRING                                          0x0240  /**< Generic Keyring. */
#define BLE_APPEARANCE_GENERIC_MEDIA_PLAYER                                     0x0280  /**< Generic Media Player. */
#define BLE_APPEARANCE_GENERIC_BARCODE_SCANNER                                  0x02C0  /**< Generic Barcode Scanner. */
#define BLE_APPEARANCE_GENERIC_THERMOMETER                                      0x0300  /**< Generic Thermometer. */
#define BLE_APPEARANCE_THERMOMETER_EAR                                          0x0301  /**< Thermometer: Ear. */
#define BLE_APPEARANCE_GENERIC_HEART_RATE_SENSOR                                0x0340  /**< Generic Heart rate Sensor. */
#define BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT                        0x0341  /**< Heart Rate Sensor: Heart Rate Belt. */
#define BLE_APPEARANCE_GENERIC_BLOOD_PRESSURE                                   0x0380  /**< Generic Blood Pressure. */
#define BLE_APPEARANCE_BLOOD_PRESSURE_ARM                                       0x0381  /**< Blood Pressure: Arm. */
#define BLE_APPEARANCE_BLOOD_PRESSURE_WRIST                                     0x0382  /**< Blood Pressure: Wrist. */
#define BLE_APPEARANCE_GENERIC_HID                                              0x03C0  /**< Human Interface Device (HID). */
#define BLE_APPEARANCE_HID_KEYBOARD                                             0x03C1  /**< Keyboard (HID Subtype). */
#define BLE_APPEARANCE_HID_MOUSE                                                0x03C2  /**< Mouse (HID Subtype). */
#define BLE_APPEARANCE_HID_JOYSTICK                                             0x03C3  /**< Joystick (HID Subtype). */
#define BLE_APPEARANCE_HID_GAMEPAD                                              0x03C4  /**< Gamepad (HID Subtype). */
#define BLE_APPEARANCE_HID_DIGITIZERSUBTYPE                                     0x03C5  /**< Digitizer Tablet (HID Subtype). */
#define BLE_APPEARANCE_HID_CARD_READER                                          0x03C6  /**< Card Reader (HID Subtype). */
#define BLE_APPEARANCE_HID_DIGITAL_PEN                                          0x03C7  /**< Digital Pen (HID Subtype). */
#define BLE_APPEARANCE_HID_BARCODE                                              0x03C8  /**< Barcode Scanner (HID Subtype). */
#define BLE_APPEARANCE_GENERIC_GLUCOSE_METER                                    0x0400  /**< Generic Glucose Meter. */
#define BLE_APPEARANCE_GENERIC_RUNNING_WALKING_SENSOR                           0x0440  /**< Generic Running Walking Sensor. */
#define BLE_APPEARANCE_RUNNING_WALKING_SENSOR_IN_SHOE                           0x0441  /**< Running Walking Sensor: In-Shoe. */
#define BLE_APPEARANCE_RUNNING_WALKING_SENSOR_ON_SHOE                           0x0442  /**< Running Walking Sensor: On-Shoe. */
#define BLE_APPEARANCE_RUNNING_WALKING_SENSOR_ON_HIP                            0x0443  /**< Running Walking Sensor: On-Hip. */
#define BLE_APPEARANCE_GENERIC_CYCLING                                          0x0480  /**< Generic Cycling. */
#define BLE_APPEARANCE_CYCLING_CYCLING_COMPUTER                                 0x0481  /**< Cycling: Cycling Computer. */
#define BLE_APPEARANCE_CYCLING_SPEED_SENSOR                                     0x0482  /**< Cycling: Speed Sensor. */
#define BLE_APPEARANCE_CYCLING_CADENCE_SENSOR                                   0x0483  /**< Cycling: Cadence Sensor. */
#define BLE_APPEARANCE_CYCLING_POWER_SENSOR                                     0x0484  /**< Cycling: Power Sensor. */
#define BLE_APPEARANCE_CYCLING_SPEED_CADENCE_SENSOR                             0x0485  /**< Cycling: Speed and Cadence Sensor. */
#define BLE_APPEARANCE_GENERIC_PULSE_OXIMETER                                   0x0C40  /**< Generic Pulse Oximeter. */
#define BLE_APPEARANCE_PULSE_OXIMETER_FINGERTIP                                 0x0C41  /**< Fingertip (Pulse Oximeter subtype). */
#define BLE_APPEARANCE_PULSE_OXIMETER_WRIST_WORN                                0x0C42  /**< Wrist Worn(Pulse Oximeter subtype). */
#define BLE_APPEARANCE_GENERIC_WEIGHT_SCALE                                     0x0C80  /**< Generic Weight Scale. */
#define BLE_APPEARANCE_GENERIC_OUTDOOR_SPORTS_ACT                               0x1440  /**< Generic Outdoor Sports Activity. */
#define BLE_APPEARANCE_OUTDOOR_SPORTS_ACT_LOC_DISP                              0x1441  /**< Location Display Device (Outdoor Sports Activity subtype). */
#define BLE_APPEARANCE_OUTDOOR_SPORTS_ACT_LOC_AND_NAV_DISP                      0x1442  /**< Location and Navigation Display Device (Outdoor Sports Activity subtype). */
#define BLE_APPEARANCE_OUTDOOR_SPORTS_ACT_LOC_POD                               0x1443  /**< Location Pod (Outdoor Sports Activity subtype). */
#define BLE_APPEARANCE_OUTDOOR_SPORTS_ACT_LOC_AND_NAV_POD                       0x1444  /**< Location and Navigation Pod (Outdoor Sports Activity subtype). */
/** @} */

#endif // _BLE_GAP_H_

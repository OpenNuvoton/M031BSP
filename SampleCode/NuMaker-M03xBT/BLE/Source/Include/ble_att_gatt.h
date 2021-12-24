/**************************************************************************//**
* @file       ble_att_gatt.h
* @brief      Provide the Definition of BLE Attributes and Generic Attributes Protocol.
*
* @defgroup ble_att_gatt BLE ATT/ GATT Definition
* @{
* @details  Common definitions for the BLE Attributes (ATT)/ BLE Generic Attributes(GATT). (ble_att_gatt.h).
* @}
*****************************************************************************/

#ifndef _BLE_ATT_GATT_H_
#define _BLE_ATT_GATT_H_

/**
* @defgroup ble_attTable BLE ATT/ GATT Definition for "ATTRIBUTE_BLE"
* @{
 * @ingroup ble_att_gatt
* @}
*/

/**
 * @defgroup bleCharPropertie BLE Characteristic Properties Definition
 * @{
 * @details  BLE characteristic properties definition for "propertyValue" in @ref ATTRIBUTE_BLE.
 * @note The characteristic properties bit field determines how the characteristic value can be used, or how the characteristic descriptors can be accessed.
 * @ingroup ble_attTable
 */
#define GATT_DECLARATIONS_PROPERTIES_BROADCAST                          0x01  /**< Broadcasting of the value permitted. */
#define GATT_DECLARATIONS_PROPERTIES_READ                               0x02  /**< Reading the value permitted. */
#define GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE             0x04  /**< Writing the value with write command permitted. */
#define GATT_DECLARATIONS_PROPERTIES_WRITE                              0x08  /**< Writing the value with write request permitted. */
#define GATT_DECLARATIONS_PROPERTIES_NOTIFY                             0x10  /**< Notification of the value permitted. */
#define GATT_DECLARATIONS_PROPERTIES_INDICATE                           0x20  /**< Indications of the value permitted. */
#define GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES        0x40  /**< Writing the value with signed write command permitted. */
#define GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES                0x80  /**< Addional characteristic properties are defined in the characteristic extended properties descriptor. */
/** @} */



/**
 * @defgroup bleAttType BLE Attribute Type Format Definition
 * @{
 * @details  BLE attribute type UUID format definition for "permFormatDB" in @ref ATTRIBUTE_BLE.
 *
 * @ingroup ble_attTable
 *
 * @note bit[0] in "UUID format and characteristic permission".
 */
#define ATT_TYPE_FORMAT_128UUID                                         0x00  /**< 128-bits UUID. */
#define ATT_TYPE_FORMAT_16UUID                                          0x01  /**< 16-bits UUID. */
/** @} */


/**
 * @defgroup bleBondOption BLE Attribute Value Bond or Not Option Definition
 * @{
 * @details  BLE attribute data bond option definition for "permFormatDB" in @ref ATTRIBUTE_BLE.
 * @ingroup ble_attTable
 * @note bit[1] in "UUID format and characteristic permission".
 */
#define ATT_VALUE_BOND_DISABLE                                          0x00  /**< The attribute value will be bond when writing. */
#define ATT_VALUE_BOND_ENABLE                                           0x02  /**< The attribute value will be bond when writing. */
/** @} */



/**
 * @defgroup blePermission BLE Attribute Permission Definition
 * @{
 * @details  BLE attribute permission definition for "propertyValue" in @ref ATTRIBUTE_BLE.
 * @ingroup ble_attTable
 * @note bit[2:7] in "UUID format and characteristic permission".
 */
#define ATT_PERMISSION_ENC_READ                                         0x04  /**< The attribute is encryption required for the remote read access. */
#define ATT_PERMISSION_ENC_WRITE                                        0x08  /**< The attribute is encryption required for the remote write access. */
#define ATT_PERMISSION_AUTHE_READ                                       0x10  /**< The attribute is authentication required for the remote read access. */
#define ATT_PERMISSION_AUTHE_WRITE                                      0x20  /**< The attribute is authentication required for the remote write access. */
#define ATT_PERMISSION_AUTHO_READ                                       0x40  /**< The attribute is authorization required for the remote read access. */
#define ATT_PERMISSION_AUTHO_WRITE                                      0x80  /**< The attribute is authorization required for the remote write access. */
/** @} */




/**
 * @defgroup blepresentFormatMain BLE Characteristic Presentation Format
 * @{
 * @ingroup ble_att_gatt
 * @}
*/
/**
 * @defgroup blepresentFormat Format Field Definition
 * @{
 * @details  The format field determines how a single value contained in the characteristic value is formatted.
 *
 * @ingroup blepresentFormatMain
 */
#define GATT_CHARACTERISTIC_FORMAT_RFU                                  0x00  /**< Reserved for future use. */
#define GATT_CHARACTERISTIC_FORMAT_BOOLEAN                              0x01  /**< Boolean. */
#define GATT_CHARACTERISTIC_FORMAT_2BIT                                 0x02  /**< Unsigned 2-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_NIBBLE                               0x03  /**< Unsigned 4-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_UINT8                                0x04  /**< Unsigned 8-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_UINT12                               0x05  /**< Unsigned 12-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_UINT16                               0x06  /**< Unsigned 16-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_UINT24                               0x07  /**< Unsigned 24-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_UINT32                               0x08  /**< Unsigned 32-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_UINT48                               0x09  /**< Unsigned 48-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_UINT64                               0x0A  /**< Unsigned 64-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_UINT128                              0x0B  /**< Unsigned 128-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_SINT8                                0x0C  /**< Signed 2-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_SINT12                               0x0D  /**< Signed 12-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_SINT16                               0x0E  /**< Signed 16-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_SINT24                               0x0F  /**< Signed 24-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_SINT32                               0x10  /**< Signed 32-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_SINT48                               0x11  /**< Signed 48-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_SINT64                               0x12  /**< Signed 64-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_SINT128                              0x13  /**< Signed 128-bit integer. */
#define GATT_CHARACTERISTIC_FORMAT_FLOAT32                              0x14  /**< IEEE-754 32-bit floating point. */
#define GATT_CHARACTERISTIC_FORMAT_FLOAT64                              0x15  /**< IEEE-754 64-bit floating point. */
#define GATT_CHARACTERISTIC_FORMAT_SFLOAT                               0x16  /**< IEEE-11073 16-bit sfloat. */
#define GATT_CHARACTERISTIC_FORMAT_FLOAT                                0x17  /**< IEEE-11073 32-bit float. */
#define GATT_CHARACTERISTIC_FORMAT_DUINT16                              0x18  /**< IEEE-20601 format. */
#define GATT_CHARACTERISTIC_FORMAT_UTF8S                                0x19  /**< UTF-8 string. */
#define GATT_CHARACTERISTIC_FORMAT_UTF16S                               0x1A  /**< UTF-16 string. */
#define GATT_CHARACTERISTIC_FORMAT_STRUCT                               0x1B  /**< Opaque structure. */
/** @} */


/**
 * @defgroup bleNameSpace Namespace Field Definition
 * @{
 * @ingroup blepresentFormatMain
 * @details  The Name Space field is used to identify the organization as defined in the Assigned Numbers document.
 *
 * @note The name space of the description as defined in Assigned Numbers Specification: \n
 *       https://www.bluetooth.org/en-us/specification/assigned-numbers
 */
#define GATT_CHARACTERISTIC_BLUETOOTH_NAMESPACE_BLUETOOTH_SIG           0x01  /**< Bluetooth SIG defined namespace. */
/** @} */



/**
 * @defgroup bleAttOpCode BLE Attribute Operation Code Definition
 * @{
 * @details  BLE attribute operation code definition.
 * @ingroup ble_att_gatt
 */
typedef uint8_t BleAttOpcode;
#define OPCODE_ATT_ERROR_RESPONSE                                       0x01     /**< Error response. */
#define OPCODE_ATT_EXCHANGE_MTU_REQUEST                                 0x02     /**< Exchange MTU request. */
#define OPCODE_ATT_EXCHANGE_MTU_RESPONSE                                0x03     /**< Exchange MTU response. */
#define OPCODE_ATT_FIND_INFORMATION_REQUEST                             0x04     /**< Find information request. */
#define OPCODE_ATT_FIND_INFORMATION_RESPONSE                            0x05     /**< Find information response. */
#define OPCODE_ATT_FIND_BY_TYPE_VALUE_REQUEST                           0x06     /**< Find by type value request. */
#define OPCODE_ATT_FIND_BY_TYPE_VALUE_RESPONSE                          0x07     /**< Find by type value response. */
#define OPCODE_ATT_READ_BY_TYPE_REQUEST                                 0x08     /**< Read by type request. */
#define OPCODE_ATT_READ_BY_TYPE_RESPONSE                                0x09     /**< Read by type response. */
#define OPCODE_ATT_READ_REQUEST                                         0x0A     /**< Read request. */
#define OPCODE_ATT_READ_RESPONSE                                        0x0B     /**< Read response. */
#define OPCODE_ATT_READ_BLOB_REQUEST                                    0x0C     /**< Read blob request. */
#define OPCODE_ATT_READ_BLOB_RESPONSE                                   0x0D     /**< Read blob response. */
#define OPCODE_ATT_READ_MULTIPLE_REQUEST                                0x0E     /**< Read multiple request. */
#define OPCODE_ATT_READ_MULTIPLE_RESPONSE                               0x0F     /**< Read multiple response. */
#define OPCODE_ATT_READ_BY_GROUP_TYPE_REQUEST                           0x10     /**< Read by group type request. */
#define OPCODE_ATT_READ_BY_GROUP_TYPE_RESPONSE                          0x11     /**< Read by group type response. */
#define OPCODE_ATT_WRITE_REQUEST                                        0x12     /**< Write request. */
#define OPCODE_ATT_WRITE_RESPONSE                                       0x13     /**< Write response. */
#define OPCODE_ATT_WRITE_COMMAND                                        0x52     /**< Write command. */
#define OPCODE_ATT_PREPARE_WRITE_REQUEST                                0x16     /**< Prepare write request. */
#define OPCODE_ATT_PREPARE_WRITE_RESPONSE                               0x17     /**< Prepare write response. */
#define OPCODE_ATT_EXECUTE_WRITE_REQUEST                                0x18     /**< Execute write request. */
#define OPCODE_ATT_EXECUTE_WRITE_RESPONSE                               0x19     /**< Execute write response. */
#define OPCODE_ATT_HANDLE_VALUE_NOTIFICATION                            0x1B     /**< Handle value notification. */
#define OPCODE_ATT_HANDLE_VALUE_INDICATION                              0x1D     /**< Handle value indication. */
#define OPCODE_ATT_HANDLE_VALUE_CONFIRMATION                            0x1E     /**< Handle value confirmation. */
#define OPCODE_ATT_SIGNED_WRITE_COMMAND                                 0xD2     /**< Signed write command. */
#define OPCODE_ATT_RESTORE_BOND_DATA_COMMAND                            0xFF     /**< VENDOR DEFINE: restore bond data Command. */
/** @} */


/**
 * @defgroup bleAttErrorCode BLE Attribute Error Code Definition
 * @{
 * @details  BLE attribute error code definition.
 * @ingroup ble_att_gatt
 */
typedef uint8_t BleAttErrorRsp;
#define ERR_CODE_ATT_NO_ERROR                                           0x00  /**< The procedure finished without errors. */
#define ERR_CODE_ATT_INVALID_HANDLE                                     0x01  /**< Invalid attribute handle. */
#define ERR_CODE_ATT_READ_NOT_PERMITTED                                 0x02  /**< Read not permitted. */
#define ERR_CODE_ATT_WRITE_NOT_PERMITTED                                0x03  /**< Write not permitted. */
#define ERR_CODE_ATT_INVALID_PDU                                        0x04  /**< Invalid PDU. */
#define ERR_CODE_ATT_INSUFFICIENT_AUTHENTICATION                        0x05  /**< Authenticated link required. */
#define ERR_CODE_ATT_REQUEST_NOT_SUPPORTED                              0x06  /**< Request not supported. */
#define ERR_CODE_ATT_INVALID_OFFSET                                     0x07  /**< Invalid offset*/
#define ERR_CODE_ATT_INSUFFICIENT_AUTHORIZATION                         0x08  /**< Insufficient authorization. */
#define ERR_CODE_ATT_PREPARE_QUEUE_FULL                                 0x09  /**< Prepare queue full. */
#define ERR_CODE_ATT_ATTRIBUTE_NOT_FOUND                                0x0A  /**< Attribute not found. */
#define ERR_CODE_ATT_ATTRIBUTE_NOT_LONG                                 0x0B  /**< Attribute cannot be read or written using read/write blob requests. */
#define ERR_CODE_ATT_INSUFFICIENT_ENCRYPTION_KEY_SIZE                   0x0C  /**< Encryption key size is insufficient. */
#define ERR_CODE_ATT_INVALID_ATTRIBUTE_VALUE_LENGTH                     0x0D  /**< Invalid value length. */
#define ERR_CODE_ATT_UNLIKELY_ERROR                                     0x0E  /**< Unlikely error. */
#define ERR_CODE_ATT_INSUFFICIENT_ENCRYPTION                            0x0F  /**< Encrypted link required. */
#define ERR_CODE_ATT_UNSUPPORTED_GROUP_TYPE                             0x10  /**< Attribute type is not a supported grouping attribute. */
#define ERR_CODE_ATT_INSUFFICIENT_RESOURCES                             0x11  /**< Encrypted link required. */
#define ERR_CODE_ATT_DATABASE_OUT_OF_SYNC                               0x12  /**< The server requests the client to rediscover the database. */
#define ERR_CODE_ATT_VALUE_NOT_ALLOWED                                  0x13  /**< The attribute parameter value was not allowed. */
#define ERR_CODE_ATT_RESERVED                                           0x14  /**< Reserved for future use. */
#define ERR_CODE_ATT_APPLICATION_ERROR                                  0x80  /**< Attribute application error. */
#define ERR_CODE_ATT_WRITE_REQUEST_REJECTED                             0xFC  /**< Write request rejected. */
#define ERR_CODE_ATT_CLIENT_CHAR_CONFIG_DESCRTR_IMPROPERLY_CONFIGURED   0xFD  /**< Client characteristic configuration descriptor improperly configured. */
#define ERR_CODE_ATT_PROCEDURE_ALREADY_IN_PROGRESS                      0xFE  /**< Procedure already in progress. */
#define ERR_CODE_ATT_OUT_OF_RANGE                                       0xFF  /**< Out of range. */
/** @} */


#endif // _BLE_ATT_GATT_H_


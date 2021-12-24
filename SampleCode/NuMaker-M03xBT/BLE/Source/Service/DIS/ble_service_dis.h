#ifndef _BLE_SERVICE_DIS_H_
#define _BLE_SERVICE_DIS_H_

/**************************************************************************//**
 * @file  ble_service_dis.h
 * @brief Provide the Definition of DIS (Device Information Service).
*****************************************************************************/

#include "ble_service_base.h"
#include "ble_host.h"
#include "ble_cmd.h"


/**************************************************************************
 * DIS Definitions
 **************************************************************************/
/** @defgroup serviceDIS_def BLE DIS Definitions
 * @{
 * @ingroup service_def
 * @details Here shows the definitions of the DIS service.
 * @}
**************************************************************************/

/**
 * @ingroup serviceDIS_def
 * @defgroup serviceDIS_UUIDDef BLE DIS UUID Definitions
 * @{
 * @details Here shows the definitions of the BLE DIS UUID Definitions.
*/
extern const uint16_t ATTR_UUID_DIS_PRIMARY_SERVICE[];                  /**< DIS service UUID. */
extern const uint16_t ATTR_UUID_DIS_CHARC_SERIAL_NUMBER_STRING[];       /**< DIS characteristic serial number string UUID. */
extern const uint16_t ATTR_UUID_DIS_CHARC_MANUFACTURER_NAME_STRING[];   /**< DIS characteristic manufacturer name string UUID. */
extern const uint16_t ATTR_UUID_DIS_CHARC_SYSTEM_ID[];                  /**< DIS characteristic system ID UUID. */
extern const uint16_t ATTR_UUID_DIS_CHARC_FIRMWARE_REVISION_STRING[];   /**< DIS characteristic firmware revision string UUID. */
extern const uint16_t ATTR_UUID_DIS_CHARC_MODEL_NUMBER_STRING[];        /**< DIS characteristic model number string UUID. */
extern const uint16_t ATTR_UUID_DIS_CHARC_HARDWARE_REVISION_STRING[];   /**< DIS characteristic hardware revision string UUID. */
extern const uint16_t ATTR_UUID_DIS_CHARC_SOFTWARE_REVISION_STRING[];   /**< DIS characteristic software revision string UUID. */
extern const uint16_t ATTR_UUID_DIS_CHARC_PNP_ID[];                     /**< DIS characteristic PnP ID UUID. */
/** @} */


/**
 * @defgroup serviceDIS_dataDef BLE DIS Value Definitions
 * @{
 * @ingroup serviceDIS_def
 * @details Here shows the value definitions of the DIS service.
 * @note Return the "Read data" when central send "Read Request".
*/

extern const uint8_t ATTR_VALUE_DIS_SERIAL_NUMBER_STRING_PRESENTATION_FORMAT[];     /**< DIS serial number string presentation format value. */
extern const uint8_t ATTR_VALUE_DIS_COMMON_PRESENTATION_FORMAT[];                   /**< DIS common presentation format value. */
extern const uint8_t ATTR_VALUE_DIS_MANUFACTURER_NAME_STRING_PRESENTATION_FORMAT[]; /**< DIS manufacturer name string presentation format value. */
extern const uint8_t ATTR_VALUE_DIS_SYSTEM_ID[];                                    /**< DIS system ID string value. */
extern const uint8_t ATTR_VALUE_DIS_FIRMWARE_REVISION_STRING_PRESENTATION_FORMAT[]; /**< DIS firmware revision presentation string value. */
extern const uint8_t ATTR_VALUE_DIS_MODEL_NUMBER_STRING_PRESENTATION_FORMAT[];      /**< DIS model number string presentation format value. */
extern const uint8_t ATTR_VALUE_DIS_HARDWARE_REVISION_STRING_PRESENTATION_FORMAT[]; /**< DIS hardware revision string presentation format value. */
extern const uint8_t ATTR_VALUE_DIS_SOFTWARE_REVISION_STRING_PRESENTATION_FORMAT[]; /**< DIS software revision string presentation format value. */
extern const uint8_t ATTR_VALUE_DIS_PNP_ID[];                                       /**< DIS PnP ID value. */
/** @} */


/**
 * @defgroup serviceDIS_ServiceChardef BLE DIS Service and Characteristic Definitions
 * @{
 * @ingroup serviceDIS_def
 * @details Here shows the definitions of the DIS service and characteristic.
 * @}
*/

/**
 * @ingroup serviceDIS_ServiceChardef
 * @{
*/
extern const ATTRIBUTE_BLE ATT_DIS_PRIMARY_SERVICE;                                 /**< DIS primary service. */
extern const ATTRIBUTE_BLE ATT_DIS_CHARACTERISTIC_SERIAL_NUMBER_STRING;             /**< DIS characteristic serial number string. */
extern const ATTRIBUTE_BLE ATT_DIS_SERIAL_NUMBER_STRING;                            /**< DIS serial number string value. */
extern const ATTRIBUTE_BLE ATT_DIS_SERIAL_NUMBER_STRING_PRESENTATION_FORMAT;        /**< DIS characteristic serial number string presentation format. */
extern const ATTRIBUTE_BLE ATT_DIS_CHARACTERISTIC_MANUFACTURER_NAME_STRING;         /**< DIS characteristic manufacturer name string. */
extern const ATTRIBUTE_BLE ATT_DIS_MANUFACTURER_NAME_STRING;                        /**< DIS manufacturer name string. */
extern const ATTRIBUTE_BLE ATT_DIS_MANUFACTURER_NAME_STRING_PRESENTATION_FORMAT;    /**< DIS characteristic manufacturer name string presentation format. */
extern const ATTRIBUTE_BLE ATT_DIS_CHARACTERISTIC_SYSTEM_ID;                        /**< DIS characteristic system ID. */
extern const ATTRIBUTE_BLE ATT_DIS_SYSTEM_ID;                                       /**< DIS characteristic system ID value. */
extern const ATTRIBUTE_BLE ATT_DIS_CHARACTERISTIC_FIRMWARE_REVISION_STRING;         /**< DIS characteristic firmware revision string. */
extern const ATTRIBUTE_BLE ATT_DIS_FIRMWARE_REVISION_STRING;                        /**< DIS firmware revision string value. */
extern const ATTRIBUTE_BLE ATT_DIS_FIRMWARE_REVISION_STRING_PRESENTATION_FORMAT;    /**< DIS firmware revision string presentation format. */
extern const ATTRIBUTE_BLE ATT_DIS_CHARACTERISTIC_MODEL_NUMBER_STRING;              /**< DIS characteristic model number string. */
extern const ATTRIBUTE_BLE ATT_DIS_MODEL_NUMBER_STRING;                             /**< DIS model number string value. */
extern const ATTRIBUTE_BLE ATT_DIS_MODEL_NUMBER_STRING_PRESENTATION_FORMAT;         /**< DIS model number string presentation format. */
extern const ATTRIBUTE_BLE ATT_DIS_CHARACTERISTIC_HARDWARE_REVISION_STRING;         /**< DIS characteristic hardware revision string. */
extern const ATTRIBUTE_BLE ATT_DIS_HARDWARE_REVISION_STRING;                        /**< DIS hardware revision string value. */
extern const ATTRIBUTE_BLE ATT_DIS_HARDWARE_REVISION_STRING_PRESENTATION_FORMAT;    /**< DIS hardware revision string presentation format. */
extern const ATTRIBUTE_BLE ATT_DIS_CHARACTERISTIC_SOFTWARE_REVISION_STRING;         /**< DIS characteristic software revision string. */
extern const ATTRIBUTE_BLE ATT_DIS_SOFTWARE_REVISION_STRING;                        /**< DIS software revision string value. */
extern const ATTRIBUTE_BLE ATT_DIS_SOFTWARE_REVISION_STRING_PRESENTATION_FORMAT;    /**< DIS software revision string presentation format. */
extern const ATTRIBUTE_BLE ATT_DIS_CHARACTERISTIC_PNP_ID;                           /**< DIS characteristic PnP ID. */
extern const ATTRIBUTE_BLE ATT_DIS_PNP_ID;                                          /**< DIS PnP ID value. */
/** @} */


/** DIS Service Definition
 * @ingroup serviceDIS_ServiceChardef
*/
#define ATT_DIS_SERVICE                                             \
        &ATT_DIS_PRIMARY_SERVICE,                                   \
        &ATT_DIS_CHARACTERISTIC_SERIAL_NUMBER_STRING,               \
        &ATT_DIS_SERIAL_NUMBER_STRING,                              \
        &ATT_DIS_SERIAL_NUMBER_STRING_PRESENTATION_FORMAT,          \
        &ATT_DIS_CHARACTERISTIC_MANUFACTURER_NAME_STRING,           \
        &ATT_DIS_MANUFACTURER_NAME_STRING,                          \
        &ATT_DIS_MANUFACTURER_NAME_STRING_PRESENTATION_FORMAT,      \
        &ATT_DIS_CHARACTERISTIC_SYSTEM_ID,                          \
        &ATT_DIS_SYSTEM_ID,                                         \
        &ATT_DIS_CHARACTERISTIC_FIRMWARE_REVISION_STRING,           \
        &ATT_DIS_FIRMWARE_REVISION_STRING,                          \
        &ATT_DIS_FIRMWARE_REVISION_STRING_PRESENTATION_FORMAT,      \
        &ATT_DIS_CHARACTERISTIC_MODEL_NUMBER_STRING,                \
        &ATT_DIS_MODEL_NUMBER_STRING,                               \
        &ATT_DIS_MODEL_NUMBER_STRING_PRESENTATION_FORMAT,           \
        &ATT_DIS_CHARACTERISTIC_HARDWARE_REVISION_STRING,           \
        &ATT_DIS_HARDWARE_REVISION_STRING,                          \
        &ATT_DIS_HARDWARE_REVISION_STRING_PRESENTATION_FORMAT,      \
        &ATT_DIS_CHARACTERISTIC_SOFTWARE_REVISION_STRING,           \
        &ATT_DIS_SOFTWARE_REVISION_STRING,                          \
        &ATT_DIS_SOFTWARE_REVISION_STRING_PRESENTATION_FORMAT,      \
        &ATT_DIS_CHARACTERISTIC_PNP_ID,                             \
        &ATT_DIS_PNP_ID,                                            \


/**
 * @defgroup serviceDIS_pnpdef BLE DIS Characteristic PnP ID Vendor ID Source Field Definitions
 * @{
 * @ingroup serviceDIS_def
 * @details Here shows the definitions of the DIS PnP ID Vendor ID source field.
 *
 * @note The Bluetooth Special Interest Group assigns Device ID Vendor ID, and the USB Implementer Forum assigns Vendor IDs, either of which can be used for the Vendor ID field value. \n
 *       Device providers should procure the Vendor ID from the USB Implementer Forum or the Company Identifier from the Bluetooth SIG.
 * @note http://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.pnp_id.xml
*/
#define BLE_GATT_DIS_PNPID_VID_SOURCE_BLUETOOTH_SIG     0x01      /**< BLE SIG-assigned Device ID Vendor ID value form the Assigned Number Document. */
#define BLE_GATT_DIS_PNPID_VID_SOURCE_USB_FORUM         0x02      /**< USB Implement's Forum assigned Vendor ID value. */
/** @} */




/**************************************************************************
 * DIS Application Definitions
 **************************************************************************/
/** @defgroup appDIS_def BLE DIS Application Definitions
 * @{
 * @ingroup serviceDIS_def
 * @details Here shows the definitions of the DIS service for application.
 * @}
**************************************************************************/

/**
 * @ingroup appDIS_def
 * @defgroup appDIS_eventDef BLE DIS Service and Characteristic Definitions
 * @{
 * @details Here shows the event definitions of the DIS service.
*/
#define BLESERVICE_DIS_SERIAL_NUMBER_RR_EVENT               0x01   /**< DIS serial number read response event. */
#define BLESERVICE_DIS_SERIAL_NUMBER_FORMAT_RR_EVENT        0x02   /**< DIS serial number presentation format read response event. */
#define BLESERVICE_DIS_MANU_NAME_RR_EVENT                   0x03   /**< DIS manufacturer name read response event. */
#define BLESERVICE_DIS_MANU_NAME_FORMAT_RR_EVENT            0x04   /**< DIS manufacturer name presentation format read response event. */
#define BLESERVICE_DIS_SYSTEM_ID_RR_EVENT                   0x05   /**< DIS system id read response event. */
#define BLESERVICE_DIS_SYSTEM_ID_FORMAT_RR_EVENT            0x06   /**< DIS system id presentation format read response event. */
#define BLESERVICE_DIS_FIRMWARE_REVISION_RR_EVENT           0x07   /**< DIS firmware revision read response event. */
#define BLESERVICE_DIS_FIRMWARE_REVISION_FORMAT_RR_EVENT    0x08   /**< DIS firmware revision presentation format read response event. */
#define BLESERVICE_DIS_MODEL_NUMBER_RR_EVENT                0x09   /**< DIS model number read response event. */
#define BLESERVICE_DIS_MODEL_NUMBER_FORMAT_RR_EVENT         0x0A   /**< DIS model number presentation format read response event. */
#define BLESERVICE_DIS_HARDWARE_REVISION_RR_EVENT           0x0B   /**< DIS hardware revision read response event. */
#define BLESERVICE_DIS_HARDWARE_REVISION_FORMAT_RR_EVENT    0x0C   /**< DIS hardware revision presentation format read response event. */
#define BLESERVICE_DIS_SOFTWARE_REVISION_RR_EVENT           0x0D   /**< DIS software revision read response event. */
#define BLESERVICE_DIS_SOFTWARE_REVISION_FORMAT_RR_EVENT    0x0E   /**< DIS software revision presentation format read response event. */
#define BLESERVICE_DIS_PNP_ID_RR_EVENT                      0x0F   /**< DIS PnP id read response event. */
#define BLESERVICE_DIS_PNP_ID_FORMAT_RR_EVENT               0x10   /**< DIS PnP id presentation format read response event. */
/** @} */


/**
 * @ingroup appDIS_def
 * @defgroup appDIS_structureDef BLE DIS Structure Definitions
 * @{
 * @details Here shows the structure definitions of the DIS service.
 * @}
*/

/** DIS Handles Definition
 * @ingroup appDIS_structureDef
*/
typedef struct BLEATT_DIS_Handles
{
    uint16_t                        hdl_serial_number_string;             /**< Handle of DIS serial number string value. */
    uint16_t                        hdl_serial_number_string_format;      /**< Handle of DIS serial number string presentation format. */
    uint16_t                        hdl_manufacturer_name_string;         /**< Handle of DIS manufacturer name string value. */
    uint16_t                        hdl_manufacturer_name_string_format;  /**< Handle of DIS manufacturer name string presentation format. */
    uint16_t                        hdl_system_id;                        /**< Handle of DIS system ID value. */
    uint16_t                        hdl_firmware_revision_string;         /**< Handle of DIS firmware revision string value. */
    uint16_t                        hdl_firmware_revision_string_format;  /**< Handle of DIS firmware revision string presentation format. */
    uint16_t                        hdl_model_number_string;              /**< Handle of DIS model number string value. */
    uint16_t                        hdl_model_number_string_format;       /**< Handle of DIS model number string presentation format. */
    uint16_t                        hdl_hardware_revision_string;         /**< Handle of DIS hardware revision string value. */
    uint16_t                        hdl_hardware_revision_string_format;  /**< Handle of DIS hardware revision string presentation format. */
    uint16_t                        hdl_software_revision_string;         /**< Handle of DIS software revision string value. */
    uint16_t                        hdl_software_revision_string_format;  /**< Handle of DIS software revision string presentation format. */
    uint16_t                        hdl_pnp_id;                           /**< Handle of DIS PnP ID value. */
} BLEATT_DIS_Handles;


/** DIS System ID Definition
 * @ingroup appDIS_structureDef
*/
typedef struct SystemId_Format
{
    uint64_t                        manufacturer_id;              /**< Manufacturer ID, Only 5 bytes shall be used. */
    uint32_t                        organizationally_unique_id;   /**< Organizationally unique ID, Only 3 bytes shall be used. */
} SystemId_Format;


/** DIS PnP ID Definition
 * @ingroup appDIS_structureDef
*/
typedef struct PnPId_Format
{
    uint8_t                         vendor_id_source;             /**< Vendor ID Source. see @ref serviceDIS_pnpdef. */
    uint16_t                        vendor_id;                    /**< Vendor ID. */
    uint16_t                        product_id;                   /**< Product ID. */
    uint16_t                        product_version;              /**< Product Version. */
} PnPId_Format;


/** DIS Data Definition
 * @ingroup appDIS_structureDef
*/
typedef struct BLEATT_DIS_Data
{
    String_Format                   *serial_number_str;           /**< Serial number string. */
    Charac_Presentation_Format      *serial_number_format;        /**< Serial number string characteristic presentation format. */
    String_Format                   *manufacturer_name_str;       /**< Manufacturer name string. */
    Charac_Presentation_Format      *manufacturer_name_format;    /**< Manufacturer name string characteristic presentation format. */
    SystemId_Format                 *system_id;                   /**< System ID */
    String_Format                   *firmware_rev_str;            /**< Firmware revision string */
    Charac_Presentation_Format      *firmware_rev_format;         /**< Firmware revision string characteristic presentation format. */
    String_Format                   *model_number_str;            /**< Model number string */
    Charac_Presentation_Format      *model_number_format;         /**< Model number string characteristic presentation format. */
    String_Format                   *hardware_rev_str;            /**< Hardware revision string */
    Charac_Presentation_Format      *hardware_rev_format;         /**< Hardware revision string characteristic presentation format. */
    String_Format                   *software_rev_str;            /**< Software revision string */
    Charac_Presentation_Format      *software_rev_format;         /**< Software revision string characteristic presentation format. */
    PnPId_Format                    *pnp_id;                      /**< PnP ID */
} BLEATT_DIS_Data;


/** DIS Application Data Structure Definition
 * @ingroup appDIS_structureDef
*/
typedef struct BLEATT_DIS_Info
{
    BleGattRole           role;        /**< BLE GATT role. */
    BLEATT_DIS_Handles    handles;     /**< DIS attribute handles. */
} BLEATT_DIS_Info;


/**
 * @ingroup appDIS_def
 * @defgroup appDIS_App BLE DIS Definitions for Application
 * @{
 * @details Here shows the definitions of the DIS for application uses.
 * @}
*/

/** BleDIS_EventCallBack
 * @ingroup appDIS_App
 * @note This callback receives the DIS events. Each of these events can be associated with parameters.
*/
typedef void (*BleDIS_EventCallBack)(uint8_t hostId, uint8_t cmdAccess, uint8_t *data, uint16_t length);



/** Device Information Service (DIS) Initialization
 *
 * @ingroup appDIS_App
 *
 * @attention Due to there is only one instance of DIS shall be exposed on a device (if role is @ref BLE_GATT_ROLE_SERVER). \n
 *            Callback shall be ignored if role is @ref BLE_GATT_ROLE_SERVER).
 *
 * @param[in] hostId : the link's host id.
 * @param[in] gattRole : @ref BleGattRole "BLE GATT role".
 * @param[in] info : a pointer to DIS information.
 * @param[in] callback : a pointer to a callback function that receive the service events.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus setDIS_ServiceInit(uint8_t hostId, BleGattRole gattRole, BLEATT_DIS_Info *info, BleDIS_EventCallBack callback);



/** Get DIS Handle Numbers
 *
 * @ingroup appDIS_App
 *
 * @attention - role = @ref BLE_GATT_ROLE_CLIENT: \n
 *              MUST call this API to get service information after received @ref BLECMD_EVENT_ATT_DATABASE_PARSING_FINISHED  \n
 *            - role = @ref BLE_GATT_ROLE_SERVER: \n
 *              MUST call this API to get service information before connection established. \n
 *
 * @par BLE DIS Event
 * Wait for BLE DIS event callback to get characteristic value. \n
 *
 * @param[in] hostId : the link's host id.
 * @param[out] info : a pointer to DIS information
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus getDIS_ServiceHandles(uint8_t hostId, BLEATT_DIS_Info *info);


/** Get data from server by reading request (Client ONLY)
 *
 * @ingroup appDIS_App
 *
 * @param[in] hostId : the link's host id
 * @param[in] hdlNum : handle number of the data you want to read.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_ERR_DB_PARSING_IN_PROGRESS : parsing database process has NOT finished.
 * @retval BLESTACK_STATUS_ERR_INVALID_HANDLE : Invalid attribute handle.
 * @retval BLESTACK_STATUS_ERR_BUSY : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus getDIS_ClientDataRead(uint8_t hostId, uint16_t hdlNum);


#endif //_BLE_SERVICE_DIS_H_

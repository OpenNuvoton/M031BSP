/************************************************************************
 *
 * File Name  : BLE_SERVICE_DIS.c
 * Description: This file contains the definitions and functions of BLE DIS
 *
 *
 ************************************************************************/
#include "ble_service_dis.h"
#include "BleAppSetting.h"
#include "ble_profile.h"

/* will be defined in BleAppSetting.h, if there is no definition then set to the default value. */
#ifndef BLE_COMPANY_ID_L
#define BLE_COMPANY_ID_L            0xCC
#endif

#ifndef BLE_COMPANY_ID_H
#define BLE_COMPANY_ID_H            0x0A
#endif

/** ATTR_VALUE_DIS_General_Access
 * @note This callback receives the DIS Service events.  \n
 *  Each of these events can be associated with parameters.
 */
void ATTR_VALUE_DIS_General_Access(BLE_Event_AttParam *attParam);

/**************************************************************************
 * DIS Service Value Definitions
 **************************************************************************/

/** DIS characteristic serial number value.
 * @note Return the "Read data" when central send "Read Request".
*/
const uint8_t DIS_SERIAL_NUMBER_STRING[]     = "1587";

/** DIS characteristic manufacturer name string value.
 * @note Return the "Read data" when central send "Read Request".
*/
const uint8_t DIS_MANUFACTURER_NAME_STRING[] = "Nuvoton";


/** DIS characteristic firmware revision string value.
 * @note Return the "Read data" when central send "Read Request".
*/
const uint8_t DIS_FIRMWARE_REVISION_STRING[] = "01.1";


/** DIS characteristic model number string value.
 * @note Return the "Read data" when central send "Read Request".
*/
const uint8_t DIS_MODEL_NUMBER_STRING[]      = "M031BT";


/** DIS characteristic hardware revision string value.
 * @note Return the "Read data" when central send "Read Request".
*/
const uint8_t DIS_HARDWARE_REVISION_STRING[] = "U01";


/** DIS characteristic software revision string value.
 * @note Return the "Read data" when central send "Read Request".
*/
const uint8_t DIS_SOFTWARE_REVISION_STRING[] = "0093";


/** DIS characteristic system ID string value.
 * @note Return the "Read data" when central send "Read Request".
*/
SystemId_Format DIS_SYSTEM_ID =
{
    0x00000055AA55AA55,                                                   //Manufacturer, 5 bytes shall be used.
    0x00AA55AA,                                                           //Organizationally Unique ID, 3 bytes shall be used.
};

/** DIS characteristic PnP ID string value.
 * @note Return the "Read data" when central send "Read Request".
*/
PnPId_Format DIS_PNP_ID =
{
    BLE_GATT_DIS_PNPID_VID_SOURCE_BLUETOOTH_SIG,                          //Vendor ID Source, identifies the source of the Vendor ID field.
    ((uint16_t)BLE_COMPANY_ID_H << 8) | BLE_COMPANY_ID_L,                 //Vendor ID
    0x0000,                                                               //Product ID: Manufacturer managed identifier for this product
    0x0000,                                                               //Product Version: Manufacturer managed version for this product
};


/** DIS characteristic common presentation format.
 * @note DIS presentation format most of the definitions are the same that using the same presentation format table.
*/
Charac_Presentation_Format DIS_COMMON_PRESENTATION_FORMAT =
{
    GATT_CHARACTERISTIC_FORMAT_UTF8S,                                     //Format: UTF-8 string
    0x00,                                                                 //Exponent: 0
    0x0000,                                                               //Unit: [0x0000] **The Unit is a UUID**
    GATT_CHARACTERISTIC_BLUETOOTH_NAMESPACE_BLUETOOTH_SIG,                //Name Space: 0x01
    0x0000                                                                //Description: 0x0000
};


// Set DIS Server Default Data.
String_Format DIS_SERIAL_NUMBER =
{
    (uint8_t *)DIS_SERIAL_NUMBER_STRING,
    SIZE_STRING(DIS_SERIAL_NUMBER_STRING),
};

String_Format DIS_MANUFACTURER_NAME =
{
    (uint8_t *)DIS_MANUFACTURER_NAME_STRING,
    SIZE_STRING(DIS_MANUFACTURER_NAME_STRING),
};

String_Format DIS_FIRMWARE_REVISION =
{
    (uint8_t *)DIS_FIRMWARE_REVISION_STRING,
    SIZE_STRING(DIS_FIRMWARE_REVISION_STRING),
};

String_Format DIS_MODEL_NUMBER =
{
    (uint8_t *)DIS_MODEL_NUMBER_STRING,
    SIZE_STRING(DIS_MODEL_NUMBER_STRING),
};

String_Format DIS_HARDWARE_REVISION =
{
    (uint8_t *)DIS_HARDWARE_REVISION_STRING,
    SIZE_STRING(DIS_HARDWARE_REVISION_STRING),
};

String_Format DIS_SOFTWARE_REVISION =
{
    (uint8_t *)DIS_SOFTWARE_REVISION_STRING,
    SIZE_STRING(DIS_SOFTWARE_REVISION_STRING),
};

BLEATT_DIS_Data DIS_Data =
{
    &DIS_SERIAL_NUMBER,
    &DIS_COMMON_PRESENTATION_FORMAT,
    &DIS_MANUFACTURER_NAME,
    &DIS_COMMON_PRESENTATION_FORMAT,
    &DIS_SYSTEM_ID,
    &DIS_FIRMWARE_REVISION,
    &DIS_COMMON_PRESENTATION_FORMAT,
    &DIS_MODEL_NUMBER,
    &DIS_COMMON_PRESENTATION_FORMAT,
    &DIS_HARDWARE_REVISION,
    &DIS_COMMON_PRESENTATION_FORMAT,
    &DIS_SOFTWARE_REVISION,
    &DIS_COMMON_PRESENTATION_FORMAT,
    &DIS_PNP_ID,
};

/**************************************************************************
 * DIS Service UUID Definitions
 **************************************************************************/

/** DIS Service UUID.
 * @note 16-bits UUID
 * @note UUID: 180A
*/
const uint16_t ATTR_UUID_DIS_PRIMARY_SERVICE[] =
{
    GATT_SERVICES_DEVICE_INFORMATION,
};


/** DIS characteristic Serial Number String UUID.
 * @note 16-bits UUID
 * @note UUID: 252A
*/
const uint16_t ATTR_UUID_DIS_CHARC_SERIAL_NUMBER_STRING[] =
{
    GATT_SPEC_CHARC_SERIAL_NUMBER_STRING,
};


/** DIS characteristic Manufacturer Name String UUID.
 * @note 16-bits UUID
 * @note UUID: 2A29
*/
const uint16_t ATTR_UUID_DIS_CHARC_MANUFACTURER_NAME_STRING[] =
{
    GATT_SPEC_CHARC_MANUFACTURER_NAME_STRING,
};


/** DIS characteristic System ID UUID.
 * @note 16-bits UUID
 * @note UUID: 2A23
*/
const uint16_t ATTR_UUID_DIS_CHARC_SYSTEM_ID[] =
{
    GATT_SPEC_CHARC_SYSTEM_ID,
};


/** DIS characteristic Firmware Revision String UUID.
 * @note 16-bits UUID
 * @note UUID: 2A26
*/
const uint16_t ATTR_UUID_DIS_CHARC_FIRMWARE_REVISION_STRING[] =
{
    GATT_SPEC_CHARC_FIRMWARE_REVISION_STRING,
};


/** DIS characteristic Model Number UUID.
 * @note 16-bits UUID
 * @note UUID: 2A24
*/
const uint16_t ATTR_UUID_DIS_CHARC_MODEL_NUMBER_STRING[] =
{
    GATT_SPEC_CHARC_MODEL_NUMBER_STRING,
};



/** DIS characteristic Hardware Revision String UUID.
 * @note 16-bits UUID
 * @note UUID: 2A27
*/
const uint16_t ATTR_UUID_DIS_CHARC_HARDWARE_REVISION_STRING[] =
{
    GATT_SPEC_CHARC_HARDWARE_REVISION_STRING,
};


/** DIS characteristic Software Revision String UUID.
 * @note 16-bits UUID
 * @note UUID: 2A28
*/
const uint16_t ATTR_UUID_DIS_CHARC_SOFTWARE_REVISION_STRING[] =
{
    GATT_SPEC_CHARC_SOFTWARE_REVISION_STRING,
};


/** DIS characteristic PnP ID UUID.
 * @note 16-bits UUID
 * @note UUID: 2A50
*/
const uint16_t ATTR_UUID_DIS_CHARC_PNP_ID[] =
{
    GATT_SPEC_CHARC_PNP_ID,
};


/**************************************************************************
 * DIS Service Value Definitions
 **************************************************************************/


/**************************************************************************
 * DIS Service/ Characteristic Definitions
 **************************************************************************/

const ATTRIBUTE_BLE ATT_DIS_PRIMARY_SERVICE =
{
    (void *)ATTR_UUID_TYPE_PRIMARY_SERVICE,
    (void *)ATTR_UUID_DIS_PRIMARY_SERVICE,
    sizeof(ATTR_UUID_DIS_PRIMARY_SERVICE),
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_NULL_Access,                       //This function should be set to ATTR_NULL_Access when lenValue or uuidValue is a null value.
};

const ATTRIBUTE_BLE ATT_DIS_CHARACTERISTIC_SERIAL_NUMBER_STRING =
{
    (void *)ATTR_UUID_TYPE_CHARACTERISTIC,
    (void *)ATTR_UUID_DIS_CHARC_SERIAL_NUMBER_STRING,
    sizeof(ATTR_UUID_DIS_CHARC_SERIAL_NUMBER_STRING),
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_NULL_Access,                       //This function should be set to ATTR_NULL_Access when lenValue or uuidValue is a null value.
};

const ATTRIBUTE_BLE ATT_DIS_SERIAL_NUMBER_STRING =
{
    (void *)ATTR_UUID_DIS_CHARC_SERIAL_NUMBER_STRING,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_VALUE_DIS_General_Access,       //registered callback function
};

const ATTRIBUTE_BLE ATT_DIS_SERIAL_NUMBER_STRING_PRESENTATION_FORMAT =
{
    (void *)ATTR_UUID_TYPE_CHARC_PRESENTATION_FORMAT,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_VALUE_DIS_General_Access,       //registered callback function
};

const ATTRIBUTE_BLE ATT_DIS_CHARACTERISTIC_MANUFACTURER_NAME_STRING =
{
    (void *)ATTR_UUID_TYPE_CHARACTERISTIC,
    (void *)ATTR_UUID_DIS_CHARC_MANUFACTURER_NAME_STRING,
    sizeof(ATTR_UUID_DIS_CHARC_MANUFACTURER_NAME_STRING),
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_NULL_Access,                       //This function should be set to ATTR_NULL_Access when lenValue or uuidValue is a null value.
};

const ATTRIBUTE_BLE ATT_DIS_MANUFACTURER_NAME_STRING =
{
    (void *)ATTR_UUID_DIS_CHARC_MANUFACTURER_NAME_STRING,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_VALUE_DIS_General_Access,       //registered callback function
};

const ATTRIBUTE_BLE ATT_DIS_MANUFACTURER_NAME_STRING_PRESENTATION_FORMAT =
{
    (void *)ATTR_UUID_TYPE_CHARC_PRESENTATION_FORMAT,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_VALUE_DIS_General_Access,       //registered callback function
};

const ATTRIBUTE_BLE ATT_DIS_CHARACTERISTIC_SYSTEM_ID =
{
    (void *)ATTR_UUID_TYPE_CHARACTERISTIC,
    (void *)ATTR_UUID_DIS_CHARC_SYSTEM_ID,
    sizeof(ATTR_UUID_DIS_CHARC_SYSTEM_ID),
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_NULL_Access,                       //This function should be set to ATTR_NULL_Access when lenValue or uuidValue is a null value.
};

const ATTRIBUTE_BLE ATT_DIS_SYSTEM_ID =
{
    (void *)ATTR_UUID_DIS_CHARC_SYSTEM_ID,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_VALUE_DIS_General_Access,       //registered callback function
};

const ATTRIBUTE_BLE ATT_DIS_CHARACTERISTIC_FIRMWARE_REVISION_STRING =
{
    (void *)ATTR_UUID_TYPE_CHARACTERISTIC,
    (void *)ATTR_UUID_DIS_CHARC_FIRMWARE_REVISION_STRING,
    sizeof(ATTR_UUID_DIS_CHARC_FIRMWARE_REVISION_STRING),
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_NULL_Access,                       //This function should be set to ATTR_NULL_Access when lenValue or uuidValue is a null value.
};

const ATTRIBUTE_BLE ATT_DIS_FIRMWARE_REVISION_STRING =
{
    (void *)ATTR_UUID_DIS_CHARC_FIRMWARE_REVISION_STRING,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_VALUE_DIS_General_Access,       //registered callback function
};

const ATTRIBUTE_BLE ATT_DIS_FIRMWARE_REVISION_STRING_PRESENTATION_FORMAT =
{
    (void *)ATTR_UUID_TYPE_CHARC_PRESENTATION_FORMAT,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_VALUE_DIS_General_Access,       //registered callback function
};

const ATTRIBUTE_BLE ATT_DIS_CHARACTERISTIC_MODEL_NUMBER_STRING =
{
    (void *)ATTR_UUID_TYPE_CHARACTERISTIC,
    (void *)ATTR_UUID_DIS_CHARC_MODEL_NUMBER_STRING,
    sizeof(ATTR_UUID_DIS_CHARC_MODEL_NUMBER_STRING),
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_NULL_Access,                       //This function should be set to ATTR_NULL_Access when lenValue or uuidValue is a null value.
};

const ATTRIBUTE_BLE ATT_DIS_MODEL_NUMBER_STRING =
{
    (void *)ATTR_UUID_DIS_CHARC_MODEL_NUMBER_STRING,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_VALUE_DIS_General_Access,       //registered callback function
};

const ATTRIBUTE_BLE ATT_DIS_MODEL_NUMBER_STRING_PRESENTATION_FORMAT =
{
    (void *)ATTR_UUID_TYPE_CHARC_PRESENTATION_FORMAT,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_VALUE_DIS_General_Access,       //registered callback function
};

const ATTRIBUTE_BLE ATT_DIS_CHARACTERISTIC_HARDWARE_REVISION_STRING =
{
    (void *)ATTR_UUID_TYPE_CHARACTERISTIC,
    (void *)ATTR_UUID_DIS_CHARC_HARDWARE_REVISION_STRING,
    sizeof(ATTR_UUID_DIS_CHARC_HARDWARE_REVISION_STRING),
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_NULL_Access,                       //This function should be set to ATTR_NULL_Access when lenValue or uuidValue is a null value.
};

const ATTRIBUTE_BLE ATT_DIS_HARDWARE_REVISION_STRING =
{
    (void *)ATTR_UUID_DIS_CHARC_HARDWARE_REVISION_STRING,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_VALUE_DIS_General_Access,       //registered callback function
};

const ATTRIBUTE_BLE ATT_DIS_HARDWARE_REVISION_STRING_PRESENTATION_FORMAT =
{
    (void *)ATTR_UUID_TYPE_CHARC_PRESENTATION_FORMAT,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_VALUE_DIS_General_Access,       //registered callback function
};

const ATTRIBUTE_BLE ATT_DIS_CHARACTERISTIC_SOFTWARE_REVISION_STRING =
{
    (void *)ATTR_UUID_TYPE_CHARACTERISTIC,
    (void *)ATTR_UUID_DIS_CHARC_SOFTWARE_REVISION_STRING,
    sizeof(ATTR_UUID_DIS_CHARC_SOFTWARE_REVISION_STRING),
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_NULL_Access,                       //This function should be set to ATTR_NULL_Access when lenValue or uuidValue is a null value.
};

const ATTRIBUTE_BLE ATT_DIS_SOFTWARE_REVISION_STRING =
{
    (void *)ATTR_UUID_DIS_CHARC_SOFTWARE_REVISION_STRING,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_VALUE_DIS_General_Access,       //registered callback function
};

const ATTRIBUTE_BLE ATT_DIS_SOFTWARE_REVISION_STRING_PRESENTATION_FORMAT =
{
    (void *)ATTR_UUID_TYPE_CHARC_PRESENTATION_FORMAT,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_VALUE_DIS_General_Access,       //registered callback function
};

const ATTRIBUTE_BLE ATT_DIS_CHARACTERISTIC_PNP_ID =
{
    (void *)ATTR_UUID_TYPE_CHARACTERISTIC,
    (void *)ATTR_UUID_DIS_CHARC_PNP_ID,
    sizeof(ATTR_UUID_DIS_CHARC_PNP_ID),
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_NULL_Access,                       //This function should be set to ATTR_NULL_Access when lenValue or uuidValue is a null value.
};

const ATTRIBUTE_BLE ATT_DIS_PNP_ID =
{
    (void *)ATTR_UUID_DIS_CHARC_PNP_ID,
    (void *)0,
    0,
    (
        //GATT_DECLARATIONS_PROPERTIES_BROADCAST |
        GATT_DECLARATIONS_PROPERTIES_READ |
        //GATT_DECLARATIONS_PROPERTIES_WRITE_WITHOUT_RESPONSE |
        //GATT_DECLARATIONS_PROPERTIES_WRITE |
        //GATT_DECLARATIONS_PROPERTIES_NOTIFY |
        //GATT_DECLARATIONS_PROPERTIES_INDICATE |
        //GATT_DECLARATIONS_PROPERTIES_AUTHENTICATED_SIGNED_WRITES |
        //GATT_DECLARATIONS_PROPERTIES_EXTENDED_PROPERTIES |
        0x00
    ),
    (
        ATT_TYPE_FORMAT_16UUID |            //otherwise, 128bit UUID
        //ATT_VALUE_BOND_ENABLE |
        //ATT_PERMISSION_ENC_READ |
        //ATT_PERMISSION_ENC_WRITE |
        //ATT_PERMISSION_AUTHE_READ |
        //ATT_PERMISSION_AUTHE_WRITE |
        //ATT_PERMISSION_AUTHO_READ |
        //ATT_PERMISSION_AUTHO_WRITE |
        0x00
    ),
    ATTR_VALUE_DIS_General_Access,       //registered callback function
};


/**************************************************************************
 * BLE Service << DIS >> Local Variable
 **************************************************************************/
#ifndef MAX_NUM_CONN_DIS
// check MAX_NUM_CONN_DIS if defined or set to default 1.
#define MAX_NUM_CONN_DIS                 1
#endif


// Length of system id for decoded data to send read response to client.
#define BLE_DIS_SYSTEM_ID_LEN            8

// Length of PnP id for decoded data to send read response to client.
#define BLE_DIS_PNP_ID_LEN               7


// Service basic information
Service_Basic_Info     disBasicInfo[MAX_NUM_CONN_DIS];

// DIS information
BLEATT_DIS_Info        *disInfo[MAX_NUM_CONN_DIS];

// DIS callback function
BleDIS_EventCallBack   disCallback[MAX_NUM_CONN_DIS];

// DIS registered total count
uint8_t dis_count = 0;


// DIS decoded buffer
uint8_t                 disDecodedBuffer[8];

/**************************************************************************
 * BLE Service << DIS >> Public Function
 **************************************************************************/

/** Device Information Service (DIS) Initialization
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
BleStackStatus setDIS_ServiceInit(uint8_t hostId, BleGattRole gattRole, BLEATT_DIS_Info *info, BleDIS_EventCallBack callback)
{
    BleStackStatus status;
    uint8_t config_index;

    if (info == NULL)
    {
        return BLESTACK_STATUS_ERR_INVALID_PARAM;
    }

    // init service client basic information and get "config_index" & "dis_count"
    status = setBLE_ServiceBasicInit(hostId, gattRole, MAX_NUM_CONN_DIS, disBasicInfo, &config_index, &dis_count);
    BLESTACK_STATUS_CHECK(status);

    // Set service role
    info->role = gattRole;

    // Set DIS client info
    disInfo[config_index] = info;

    // Register DIS client callback function
    disCallback[config_index] = callback;

    if (gattRole == BLE_GATT_ROLE_SERVER)
    {
        // get server handles
        status = getDIS_ServiceHandles(hostId, disInfo[config_index]);
        BLESTACK_STATUS_CHECK(status);
    }

    return BLESTACK_STATUS_SUCCESS;
}


/** Get DIS Handle Numbers
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
BleStackStatus getDIS_ServiceHandles(uint8_t hostId, BLEATT_DIS_Info *info)
{
    BleStackStatus status;

    // Get DIS handles
    status = getBLEGATT_HandleNumAddr(hostId, info->role, (ATTRIBUTE_BLE *)&ATT_DIS_PRIMARY_SERVICE, (void *)&info->handles);
    BLESTACK_STATUS_CHECK(status);

    return BLESTACK_STATUS_SUCCESS;
}



/** Get data from server by reading request (Client ONLY)
 *
 * @param[in] hostId : the link's host id
 * @param[in] hdlNum : handle numnber of the data you want to read.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_HOSTID : Error host id.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_ERR_DB_PARSING_IN_PROGRESS : parsing database process has NOT finished.
 * @retval BLESTACK_STATUS_ERR_INVALID_HANDLE : Invalid attribute handle.
 * @retval BLESTACK_STATUS_ERR_BUSY : Message queue buffer full.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting success.
*/
BleStackStatus getDIS_ClientDataRead(uint8_t hostId, uint16_t hdlNum)
{
    BleStackStatus status;

    status = setBLEGATT_ReadCharacteristicValue(hostId, hdlNum);

    return status;
}


/**************************************************************************
 * BLE Service << DIS >> General Callback Function
 **************************************************************************/
// handle DIS post event to user
static void bleDIS_PostEvent(BLE_Event_AttParam *attParam, BleDIS_EventCallBack *callback)
{
    // check callback is null or not
    SERVICE_CALLBACK_NULL_CHECK(*callback);

    // post event to user
    (*callback)(attParam->hostId, attParam->cmdAccess, attParam->data, attParam->length);
}


// handle DIS client GATT event
static void handle_DIS_client(uint8_t index, BLE_Event_AttParam *attParam)
{
    switch (attParam->cmdAccess)
    {
    case OPCODE_ATT_READ_RESPONSE:
    {
        // receive read response from client -> post to user
        if (attParam->hdlNum == disInfo[index]->handles.hdl_serial_number_string)
        {
            attParam->cmdAccess = BLESERVICE_DIS_SERIAL_NUMBER_RR_EVENT;
            bleDIS_PostEvent(attParam, &disCallback[index]);
        }
        else if (attParam->hdlNum == disInfo[index]->handles.hdl_manufacturer_name_string)
        {
            attParam->cmdAccess = BLESERVICE_DIS_MANU_NAME_RR_EVENT;
            bleDIS_PostEvent(attParam, &disCallback[index]);
        }
        else if (attParam->hdlNum == disInfo[index]->handles.hdl_system_id)
        {
            attParam->cmdAccess = BLESERVICE_DIS_SYSTEM_ID_RR_EVENT;
            bleDIS_PostEvent(attParam, &disCallback[index]);
        }
        else if (attParam->hdlNum == disInfo[index]->handles.hdl_firmware_revision_string)
        {
            attParam->cmdAccess = BLESERVICE_DIS_FIRMWARE_REVISION_RR_EVENT;
            bleDIS_PostEvent(attParam, &disCallback[index]);
        }
        else if (attParam->hdlNum == disInfo[index]->handles.hdl_model_number_string)
        {
            attParam->cmdAccess = BLESERVICE_DIS_MODEL_NUMBER_RR_EVENT;
            bleDIS_PostEvent(attParam, &disCallback[index]);
        }
        else if (attParam->hdlNum == disInfo[index]->handles.hdl_hardware_revision_string)
        {
            attParam->cmdAccess = BLESERVICE_DIS_HARDWARE_REVISION_RR_EVENT;
            bleDIS_PostEvent(attParam, &disCallback[index]);
        }
        else if (attParam->hdlNum == disInfo[index]->handles.hdl_software_revision_string)
        {
            attParam->cmdAccess = BLESERVICE_DIS_SOFTWARE_REVISION_RR_EVENT;
            bleDIS_PostEvent(attParam, &disCallback[index]);
        }
        else if (attParam->hdlNum == disInfo[index]->handles.hdl_pnp_id)
        {
            attParam->cmdAccess = BLESERVICE_DIS_PNP_ID_RR_EVENT;
            bleDIS_PostEvent(attParam, &disCallback[index]);
        }
    }
    break;

    default:
        break;
    }
}


// handle DIS server GATT event
static void handle_DIS_server(uint8_t index, BLE_Event_AttParam *attParam)
{
    switch (attParam->cmdAccess)
    {
    case OPCODE_ATT_READ_BY_TYPE_REQUEST:
    case OPCODE_ATT_READ_REQUEST:
    {
        // received read or read by type request from client -> send read or read by type rsp with const read data to client
        if (attParam->hdlNum == disInfo[index]->handles.hdl_serial_number_string)
        {
            setBLEGATT_GeneralReadRsp(attParam->hostId, attParam->hdlNum, DIS_Data.serial_number_str->str, DIS_Data.serial_number_str->length);
        }
        else if (attParam->hdlNum == disInfo[index]->handles.hdl_manufacturer_name_string)
        {
            setBLEGATT_GeneralReadRsp(attParam->hostId, attParam->hdlNum, DIS_Data.manufacturer_name_str->str, DIS_Data.manufacturer_name_str->length);
        }
        else if (attParam->hdlNum == disInfo[index]->handles.hdl_system_id)
        {
            disDecodedBuffer[0] = (DIS_Data.system_id->manufacturer_id & 0x00000000FF);
            disDecodedBuffer[1] = (DIS_Data.system_id->manufacturer_id & 0x000000FF00) >> 8;
            disDecodedBuffer[2] = (DIS_Data.system_id->manufacturer_id & 0x0000FF0000) >> 16;
            disDecodedBuffer[3] = (DIS_Data.system_id->manufacturer_id & 0x00FF000000) >> 24;
            disDecodedBuffer[4] = (DIS_Data.system_id->manufacturer_id & 0xFF00000000) >> 32;

            disDecodedBuffer[5] = (DIS_Data.system_id->organizationally_unique_id & 0x0000FF);
            disDecodedBuffer[6] = (DIS_Data.system_id->organizationally_unique_id & 0x00FF00) >> 8;
            disDecodedBuffer[7] = (DIS_Data.system_id->organizationally_unique_id & 0xFF0000) >> 16;

            setBLEGATT_GeneralReadRsp(attParam->hostId, attParam->hdlNum, disDecodedBuffer, BLE_DIS_SYSTEM_ID_LEN);
        }
        else if (attParam->hdlNum == disInfo[index]->handles.hdl_firmware_revision_string)
        {
            setBLEGATT_GeneralReadRsp(attParam->hostId, attParam->hdlNum, DIS_Data.firmware_rev_str->str, DIS_Data.firmware_rev_str->length);
        }
        else if (attParam->hdlNum == disInfo[index]->handles.hdl_model_number_string)
        {
            setBLEGATT_GeneralReadRsp(attParam->hostId, attParam->hdlNum, DIS_Data.model_number_str->str, DIS_Data.model_number_str->length);
        }
        else if (attParam->hdlNum == disInfo[index]->handles.hdl_hardware_revision_string)
        {
            setBLEGATT_GeneralReadRsp(attParam->hostId, attParam->hdlNum, DIS_Data.hardware_rev_str->str, DIS_Data.hardware_rev_str->length);
        }
        else if (attParam->hdlNum == disInfo[index]->handles.hdl_software_revision_string)
        {
            setBLEGATT_GeneralReadRsp(attParam->hostId, attParam->hdlNum, DIS_Data.software_rev_str->str, DIS_Data.software_rev_str->length);
        }
        else if (attParam->hdlNum == disInfo[index]->handles.hdl_pnp_id)
        {
            disDecodedBuffer[0] = DIS_Data.pnp_id->vendor_id_source;

            disDecodedBuffer[1] = (uint8_t) ((DIS_Data.pnp_id->vendor_id & 0x00FF) >> 0);
            disDecodedBuffer[2] = (uint8_t) ((DIS_Data.pnp_id->vendor_id & 0xFF00) >> 8);

            disDecodedBuffer[3] = (uint8_t) ((DIS_Data.pnp_id->product_id & 0x00FF) >> 0);
            disDecodedBuffer[4] = (uint8_t) ((DIS_Data.pnp_id->product_id & 0xFF00) >> 8);

            disDecodedBuffer[5] = (uint8_t) ((DIS_Data.pnp_id->product_version & 0x00FF) >> 0);
            disDecodedBuffer[6] = (uint8_t) ((DIS_Data.pnp_id->product_version & 0xFF00) >> 8);

            setBLEGATT_GeneralReadRsp(attParam->hostId, attParam->hdlNum, disDecodedBuffer, BLE_DIS_PNP_ID_LEN);
        }
        else if ( (attParam->hdlNum == disInfo[index]->handles.hdl_serial_number_string_format)     ||
                  (attParam->hdlNum == disInfo[index]->handles.hdl_manufacturer_name_string_format) ||
                  (attParam->hdlNum == disInfo[index]->handles.hdl_firmware_revision_string_format) ||
                  (attParam->hdlNum == disInfo[index]->handles.hdl_model_number_string_format)      ||
                  (attParam->hdlNum == disInfo[index]->handles.hdl_hardware_revision_string_format) ||
                  (attParam->hdlNum == disInfo[index]->handles.hdl_software_revision_string_format) )
        {
            disDecodedBuffer[0] = DIS_Data.firmware_rev_format->format;
            disDecodedBuffer[1] = DIS_Data.firmware_rev_format->exponent;

            disDecodedBuffer[2] = (uint8_t) ((DIS_Data.firmware_rev_format->unit & 0x00FF) >> 0);
            disDecodedBuffer[3] = (uint8_t) ((DIS_Data.firmware_rev_format->unit & 0xFF00) >> 8);

            disDecodedBuffer[4] = DIS_Data.firmware_rev_format->name_space;

            disDecodedBuffer[5] = (uint8_t) ((DIS_Data.firmware_rev_format->description & 0x00FF) >> 0);
            disDecodedBuffer[6] = (uint8_t) ((DIS_Data.firmware_rev_format->description & 0xFF00) >> 8);

            setBLEGATT_GeneralReadRsp(attParam->hostId, attParam->hdlNum, disDecodedBuffer, BLE_CHARAC_PRESENTATION_FORMAT_LEN);
        }
    }
    break;

    default:
        break;
    }
}


// DIS general callback
void ATTR_VALUE_DIS_General_Access(BLE_Event_AttParam *attParam)
{
    uint8_t index;

    if (queryIndexByHostIdGattRole(attParam->hostId, attParam->gattRole, MAX_NUM_CONN_DIS, disBasicInfo, &index) != BLESTACK_STATUS_SUCCESS)
    {
        // Host id has not registered so there is no callback function -> do nothing
        return;
    }

    if (attParam->gattRole == BLE_GATT_ROLE_CLIENT)
    {
        // handle DIS client GATT event
        handle_DIS_client(index, attParam);
    }

    if (attParam->gattRole == BLE_GATT_ROLE_SERVER)
    {
        // handle DIS server GATT event
        handle_DIS_server(index, attParam);
    }
}



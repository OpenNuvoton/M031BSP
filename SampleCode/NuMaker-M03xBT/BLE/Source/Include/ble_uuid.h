/**************************************************************************//**
 * @file       ble_uuid.h
 * @brief      Provide the Definition of BLE UUIDs.
 *
 *
 * @defgroup ble_uuid BLE UUIDs
 * @{
 * @ingroup service_allDef
 * @details BLE UUID definitions.
 * @}
*****************************************************************************/

#ifndef _BLE_UUID_H_
#define _BLE_UUID_H_


/**
 * @defgroup bleService_uuid BLE GATT Service UUID Definitions
 * @{
 * @ingroup ble_uuid
 * @details BLE GATT service UUID definitions.
 * @}
 * @defgroup bleSIGService_uuid BLE SIG Defined Service UUID Definitions
 * @{
 * @ingroup bleService_uuid
 * @details BLE SIG defined service UUID definitions.
 */
#define GATT_SERVICES_GENERIC_ACCESS                        0x1800    /**< Generic Access Profile Service UUID */
#define GATT_SERVICES_GENERIC_ATTRIBUTE                     0x1801    /**< Generic Attribute Profile Service UUID */
#define GATT_SERVICES_IMMEDIATE_ALERT                       0x1802    /**< Immediate Alert Service UUID. */
#define GATT_SERVICES_LINK_LOSS                             0x1803    /**< Link Loss Service UUID. */
#define GATT_SERVICES_TX_POWER                              0x1804    /**< TX Power Service UUID. */
#define GATT_SERVICES_CURRENT_TIME_SERVICE                  0x1805    /**< Current Time Service UUID. */
#define GATT_SERVICES_REFERENCE_TIME_UPDATE_SERVICE         0x1806    /**< Reference Time Update Service UUID. */
#define GATT_SERVICES_NEXT_DST_CHANGE_SERVICE               0x1807    /**< Next Dst Change Service UUID. */
#define GATT_SERVICES_GLUCOSE                               0x1808    /**< Glucose Service UUID. */
#define GATT_SERVICES_HEALTH_THERMOMETER                    0x1809    /**< Health Thermometer Service UUID. */
#define GATT_SERVICES_DEVICE_INFORMATION                    0x180A    /**< Device Information Service UUID. */
#define GATT_SERVICES_HEART_RATE                            0x180D    /**< Heart Rate Service UUID. */
#define GATT_SERVICES_PHONE_ALERT_STATUS_SERVICE            0x180E    /**< Phone Alert Status Service UUID. */
#define GATT_SERVICES_BATTERY_SERVICE                       0x180F    /**< Battery Service UUID. */
#define GATT_SERVICES_BLOOD_PRESSURE                        0x1810    /**< Blood Pressure Service UUID. */
#define GATT_SERVICES_ALERT_NOTIFICATION_SERVICE            0x1811    /**< Alert Notification Service UUID. */
#define GATT_SERVICES_HUMAN_INTERFACE_DEVICE                0x1812    /**< Human Interface Device Service UUID. */
#define GATT_SERVICES_SCAN_PARAMETERS                       0x1813    /**< Scan Parameters Service UUID. */
#define GATT_SERVICES_RUNNING_SPEED_AND_CADENCE             0x1814    /**< Running Speed and Cadence Service UUID. */
#define GATT_SERVICES_AUTOMATION_IO                         0x1815    /**< Automation IO UUID. */
#define GATT_SERVICES_CYCLING_SPEED_AND_CADENCE             0x1816    /**< Cycling Speed and Cadence Service UUID. */
#define GATT_SERVICES_CYCLING_POWER                         0x1818    /**< Cycling Power Service UUID. */
#define GATT_SERVICES_LOCATION_AND_NAVIGATION               0x1819    /**< Location and Navigation Service UUID. */
#define GATT_SERVICES_BODY_COMPOSITION                      0x181B    /**< Body Composition Service UUID. */
#define GATT_SERVICES_USER_DATA                             0x181C    /**< User Data Service UUID. */
#define GATT_SERVICES_WEIGHT_SCALE                          0x181D    /**< Weight Scale Service UUID. */
#define GATT_SERVICES_BOND_MANAGEMENT                       0x181E    /**< Bond Management Service UUID. */
/** @} */



/**
 * @defgroup bleCustomerService_uuid BLE Customer Service UUID Definitions
 * @{
 * @ingroup bleService_uuid
 * @details BLE customer defined service UUID definitions.
 */
#define GATT_CSTM_SERVICES_UDF01S                           0xF901    /**< User Defined 01 Service UUID. */
#define GATT_CSTM_SERVICES_UDF02S                           0xF902    /**< User Defined 02 Service UUID. */
#define GATT_CSTM_SERVICES_UDF03S                           0xF903    /**< User Defined 03 Service UUID. */
/** @} */





/**
 * @defgroup bleCharacteristic_uuid BLE GATT Characteristic UUID Definitions
 * @{
 * @ingroup ble_uuid
 * @details BLE GATT characteristic UUID definitions.
 * @}
 * @defgroup bleSigCharacteristic_uuid BLE SIG Defined Characteristic UUID Definitions
 * @{
 * @ingroup bleCharacteristic_uuid
 * @details BLE SIG defined characteristic UUID definitions.
 * @note https://www.bluetooth.com/specifications/gatt/characteristics/
 */
#define GATT_SPEC_CHARC_DEVICE_NAME                                     0x2A00    /**< Device Name Characteristic UUID. */
#define GATT_SPEC_CHARC_APPEARANCE                                      0x2A01    /**< Appearance Characteristic UUID. */
#define GATT_SPEC_CHARC_PERIPHERAL_PRIVACY_FLAG                         0x2A02    /**< Peripheral Privacy Flag Characteristic UUID. */
#define GATT_SPEC_CHARC_RECONNECTION_ADDRESS                            0x2A03    /**< Reconnection Address Characteristic UUID. */
#define GATT_SPEC_CHARC_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS      0x2A04    /**< Peripheral Preferred Connection Parameters Characteristic UUID. */
#define GATT_SPEC_CHARC_SERVICE_CHANGED                                 0x2A05    /**< Service Changed Characteristic UUID. */
#define GATT_SPEC_CHARC_ALERT_LEVEL                                     0x2A06    /**< Alert Level Characteristic UUID. */
#define GATT_SPEC_CHARC_TX_POWER_LEVEL                                  0x2A07    /**< TX Power Level Characteristic UUID. */
#define GATT_SPEC_CHARC_DATE_TIME                                       0x2A08    /**< Date Time Characteristic UUID. */
#define GATT_SPEC_CHARC_DAY_OF_WEEK                                     0x2A09    /**< Day Of Week Characteristic UUID. */
#define GATT_SPEC_CHARC_DAY_DATE_TIME                                   0x2A0A    /**< Day Date Time Characteristic UUID. */
#define GATT_SPEC_CHARC_EXACT_TIME_100                                  0x2A0B    /**< Exact Time 100 Characteristic UUID. */
#define GATT_SPEC_CHARC_EXACT_TIME_256                                  0x2A0C    /**< Exact Time 256 Characteristic UUID. */
#define GATT_SPEC_CHARC_DST_OFFSET                                      0x2A0D    /**< Dst Offset Characteristic UUID. */
#define GATT_SPEC_CHARC_TIME_ZONE                                       0x2A0E    /**< Time Zone Characteristic UUID. */
#define GATT_SPEC_CHARC_LOCAL_TIME_INFORMATION                          0x2A0F    /**< Local Time Information Characteristic UUID. */
#define GATT_SPEC_CHARC_SECONDARY_TIME_ZONE                             0x2A10    /**< Secondary Time Zone Characteristic UUID. */
#define GATT_SPEC_CHARC_TIME_WITH_DST                                   0x2A11    /**< Time With Dst Characteristic UUID. */
#define GATT_SPEC_CHARC_TIME_ACCURACY                                   0x2A12    /**< Time Accuracy Characteristic UUID. */
#define GATT_SPEC_CHARC_TIME_SOURCE                                     0x2A13    /**< Time Source Characteristic UUID. */
#define GATT_SPEC_CHARC_REFERENCE_TIME_INFORMATION                      0x2A14    /**< Reference Time Information Characteristic UUID. */
#define GATT_SPEC_CHARC_TIME_BROADCAST                                  0x2A15    /**< Time Broadcast Characteristic UUID. */
#define GATT_SPEC_CHARC_TIME_UPDATE_CONTROL_POINT                       0x2A16    /**< Time Update Control Point Characteristic UUID. */
#define GATT_SPEC_CHARC_TIME_UPDATE_STATE                               0x2A17    /**< Update State Characteristic UUID. */
#define GATT_SPEC_CHARC_GLUCOSE_MEASUREMENT                             0x2A18    /**< Glucose Measurement Characteristic UUID. */
#define GATT_SPEC_CHARC_BATTERY_LEVEL                                   0x2A19    /**< Battery Level Characteristic UUID. */
#define GATT_SPEC_CHARC_BATTERY_POWER_STATE                             0x2A1A    /**< Battery Power State Characteristic UUID. */
#define GATT_SPEC_CHARC_BATTERY_LEVEL_STATE                             0x2A1B    /**< Battery Level State Characteristic UUID. */
#define GATT_SPEC_CHARC_TEMPERATURE_MEASUREMENT                         0x2A1C    /**< Temperature Measurement Characteristic UUID. */
#define GATT_SPEC_CHARC_TEMPERATURE_TYPE                                0x2A1D    /**< Temperature Type Characteristic UUID. */
#define GATT_SPEC_CHARC_INTERMEDIATE_TEMPERATURE                        0x2A1E    /**< Intermediate Temperature Characteristic UUID. */
#define GATT_SPEC_CHARC_TEMPERATURE_IN_CELSIUS                          0x2A1F    /**< Temperature in Celsius Characteristic UUID. */
#define GATT_SPEC_CHARC_TEMPERATURE_IN_FAHRENHEIT                       0x2A20    /**< Temperature in Fahrenheit Characteristic UUID. */
#define GATT_SPEC_CHARC_MEASUREMENT_INTERVAL                            0x2A21    /**< Measurement Interval Characteristic UUID. */
#define GATT_SPEC_CHARC_BOOT_KEYBOARD_INPUT_REPORT                      0x2A22    /**< Boot Keyboard Input Report Characteristic UUID. */
#define GATT_SPEC_CHARC_SYSTEM_ID                                       0x2A23    /**< System Id Characteristic UUID. */
#define GATT_SPEC_CHARC_MODEL_NUMBER_STRING                             0x2A24    /**< Model Number String Characteristic UUID. */
#define GATT_SPEC_CHARC_SERIAL_NUMBER_STRING                            0x2A25    /**< Serial Number String Characteristic UUID. */
#define GATT_SPEC_CHARC_FIRMWARE_REVISION_STRING                        0x2A26    /**< Firmware Revision String Characteristic UUID. */
#define GATT_SPEC_CHARC_HARDWARE_REVISION_STRING                        0x2A27    /**< Hardware Revision String Characteristic UUID. */
#define GATT_SPEC_CHARC_SOFTWARE_REVISION_STRING                        0x2A28    /**< Software Revision String Characteristic UUID. */
#define GATT_SPEC_CHARC_MANUFACTURER_NAME_STRING                        0x2A29    /**< Manufacturer Name String Characteristic UUID. */
#define GATT_SPEC_CHARC_IEEE_11073_REGULATORY_CERTIFICATION_DATA_LIST   0x2A2A    /**< IEEE Regulatory Certification Data List Characteristic UUID. */
#define GATT_SPEC_CHARC_CURRENT_TIME                                    0x2A2B    /**< Current Time Characteristic UUID. */
#define GATT_SPEC_CHARC_SCAN_REFRESH                                    0x2A31    /**< Scan Refresh Characteristic UUID. */
#define GATT_SPEC_CHARC_BOOT_KEYBOARD_OUTPUT_REPORT                     0x2A32    /**< Boot Keyboard Output Report Characteristic UUID. */
#define GATT_SPEC_CHARC_BOOT_MOUSE_INPUT_REPORT                         0x2A33    /**< Boot Mouse Input Report Characteristic UUID. */
#define GATT_SPEC_CHARC_GLUCOSE_MEASUREMENT_CONTEXT                     0x2A34    /**< Glucose Measurement Context Characteristic UUID. */
#define GATT_SPEC_CHARC_BLOOD_PRESSURE_MEASUREMENT                      0x2A35    /**< Blood Pressure Measurement Characteristic UUID. */
#define GATT_SPEC_CHARC_INTERMEDIATE_CUFF_PRESSURE                      0x2A36    /**< Intermediate Cuff Pressure Characteristic UUID. */
#define GATT_SPEC_CHARC_HEART_RATE_MEASUREMENT                          0x2A37    /**< Heart Rate Measurement Characteristic UUID. */
#define GATT_SPEC_CHARC_BODY_SENSOR_LOCATION                            0x2A38    /**< Body Sensor Location Characteristic UUID. */
#define GATT_SPEC_CHARC_HEART_RATE_CONTROL_POINT                        0x2A39    /**< Heart Rate Control Point Characteristic UUID. */
#define GATT_SPEC_CHARC_SCIENTIFIC_TEMPERATURE_IN_CELSIUS               0x2A3C    /**< Scientific Temperature in Celsius Characteristic UUID. */
#define GATT_SPEC_CHARC_STRING                                          0x2A3D    /**< String Characteristic UUID. */
#define GATT_SPEC_CHARC_NETWORK_AVAILABILITY                            0x2A3E    /**< Network Availability Characteristic UUID. */
#define GATT_SPEC_CHARC_ALERT_STATUS                                    0x2A3F    /**< Alert Status Characteristic UUID. */
#define GATT_SPEC_CHARC_RINGER_CONTROL_POINT                            0x2A40    /**< Ringer Control Point Characteristic UUID. */
#define GATT_SPEC_CHARC_RINGER_SETTING                                  0x2A41    /**< Ringer Setting Characteristic UUID. */
#define GATT_SPEC_CHARC_ALERT_CATEGORY_ID_BIT_MASK                      0x2A42    /**< Alert Category Id Bit Mask Characteristic UUID. */
#define GATT_SPEC_CHARC_ALERT_CATEGORY_ID                               0x2A43    /**< Alert Category Id Characteristic UUID. */
#define GATT_SPEC_CHARC_ALERT_NOTIFICATION_CONTROL_POINT                0x2A44    /**< Alert Notification Control Point Characteristic UUID. */
#define GATT_SPEC_CHARC_UNREAD_ALERT_STATUS                             0x2A45    /**< Unread Alert Status Characteristic UUID. */
#define GATT_SPEC_CHARC_NEW_ALERT                                       0x2A46    /**< New Alert Characteristic UUID. */
#define GATT_SPEC_CHARC_SUPPORTED_NEW_ALERT_CATEGORY                    0x2A47    /**< Supported New Alert Category Characteristic UUID. */
#define GATT_SPEC_CHARC_SUPPORTED_UNREAD_ALERT_CATEGORY                 0x2A48    /**< Supported Unread Alert Category Characteristic UUID. */
#define GATT_SPEC_CHARC_BLOOD_PRESSURE_FEATURE                          0x2A49    /**< Blood Pressure Feature Characteristic UUID. */
#define GATT_SPEC_CHARC_HID_INFORMATION                                 0x2A4A    /**< Hid Information Characteristic UUID. */
#define GATT_SPEC_CHARC_REPORT_MAP                                      0x2A4B    /**< Report Map Characteristic UUID. */
#define GATT_SPEC_CHARC_HID_CONTROL_POINT                               0x2A4C    /**< Hid Control Point Characteristic UUID. */
#define GATT_SPEC_CHARC_REPORT                                          0x2A4D    /**< Report Characteristic UUID. */
#define GATT_SPEC_CHARC_PROTOCOL_MODE                                   0x2A4E    /**< Protocol Mode Characteristic UUID. */
#define GATT_SPEC_CHARC_SCAN_INTERVAL_WINDOW                            0x2A4F    /**< Scan Interval Window Characteristic UUID. */
#define GATT_SPEC_CHARC_PNP_ID                                          0x2A50    /**< PnP Id Characteristic UUID. */
#define GATT_SPEC_CHARC_GLUCOSE_FEATURE                                 0x2A51    /**< Glucose Feature Characteristic UUID. */
#define GATT_SPEC_CHARC_RECORD_ACCESS_CONTROL_POINT                     0x2A52    /**< Record Access Control Point Characteristic UUID. */
#define GATT_SPEC_CHARC_RSC_MEASUREMENT                                 0x2A53    /**< Running Speed and Cadence Measurement Characteristic UUID. */
#define GATT_SPEC_CHARC_RSC_FEATURE                                     0x2A54    /**< Running Speed and Cadence Feature Characteristic UUID. */
#define GATT_SPEC_CHARC_SC_CONTROL_POINT                                0x2A55    /**< Speed and Cadence Control Point Characteristic UUID. */
#define GATT_SPEC_CHARC_DIGITAL_INPUT                                   0x2A56    /**< Digital Input Characteristic UUID. */
#define GATT_SPEC_CHARC_DIGITAL_OUTPUT                                  0x2A57    /**< Digital Output Characteristic UUID. */
#define GATT_SPEC_CHARC_ANALOG_INPUT                                    0x2A58    /**< Analog Input Characteristic UUID. */
#define GATT_SPEC_CHARC_ANALOG_OUTPUT                                   0x2A59    /**< Analog Output Characteristic UUID. */
#define GATT_SPEC_CHARC_AGGREGATE_INPUT                                 0x2A5A    /**< Aggregate Input Characteristic UUID. */
#define GATT_SPEC_CHARC_CSC_MEASUREMENT                                 0x2A5B    /**< Cycling Speed and Cadence Measurement Characteristic UUID. */
#define GATT_SPEC_CHARC_CSC_FEATURE                                     0x2A5C    /**< Cycling Speed and Cadence Feature Characteristic UUID. */
#define GATT_SPEC_CHARC_SENSOR_LOCATION                                 0x2A5D    /**< Sensor Location Characteristic UUID. */
#define GATT_SPEC_CHARC_PULSE_OXIMETRY_CONTINUOUS_MEASUREMENT           0x2A5F    /**< Pulse Oximetry Continuous Measurement Characteristic UUID. */
#define GATT_SPEC_CHARC_CYCLING_POWER_MEASUREMENT                       0x2A63    /**< Cycling Power Measurement Characteristic UUID. */
#define GATT_SPEC_CHARC_CYCLING_POWER_VECTOR                            0x2A64    /**< Cycling Power Vector Characteristic UUID. */
#define GATT_SPEC_CHARC_CYCLING_POWER_FEATURE                           0x2A65    /**< Cycling Power Feature Characteristic UUID. */
#define GATT_SPEC_CHARC_CYCLING_POWER_CONTROL_POINT                     0x2A66    /**< Cycling Power Control Point Characteristic UUID. */
#define GATT_SPEC_CHARC_LOCATION_AND_SPEED                              0x2A67    /**< Location and Speed Characteristic UUID. */
#define GATT_SPEC_CHARC_NAVIGATION                                      0x2A68    /**< Navigation Characteristic UUID. */
#define GATT_SPEC_CHARC_POSITION_QUALITY                                0x2A69    /**< Position Quality Characteristic UUID. */
#define GATT_SPEC_CHARC_LN_FEATURE                                      0x2A6A    /**< Location Navigation Feature Characteristic UUID. */
#define GATT_SPEC_CHARC_LN_CONTROL_POINT                                0x2A6B    /**< Location Navigation Control point Characteristic UUID. */
/** @} */




/**
 * @defgroup bleCustomerCharacteristic_uuid BLE Customer Characteristic UUID Definitions
 * @{
 * @ingroup bleCharacteristic_uuid
 * @details BLE customer characteristic UUID definitions.
 */
#define GATT_CSTM_CHARC_UDATR01                                         0xFA01    /**< User Data Read 01 Characteristic UUID. */
#define GATT_CSTM_CHARC_UDATN01                                         0xFA02    /**< User Data Notification 01 Characteristic UUID. */
#define GATT_CSTM_CHARC_UDATRW01                                        0xFA03    /**< User Data Read Write 01 Characteristic UUID. */
#define GATT_CSTM_CHARC_UDATR02                                         0xFA04    /**< User Data Read 02 Characteristic UUID. */
#define GATT_CSTM_CHARC_UDATN02                                         0xFA05    /**< User Data Notification 02 Characteristic UUID. */
#define GATT_CSTM_CHARC_UDATRW02                                        0xFA06    /**< User Data Read Write 02 Characteristic UUID. */
#define GATT_CSTM_CHARC_UDATR03                                         0xFA07    /**< User Data Read 03 Characteristic UUID. */
#define GATT_CSTM_CHARC_UDATN03                                         0xFA08    /**< User Data Notification 03 Characteristic UUID. */
#define GATT_CSTM_CHARC_UDATRW03                                        0xFA09    /**< User Data Read Write 03 Characteristic UUID. */
/** @} */





/**
 * @defgroup bleDescriptor_uuid BLE GATT Descriptors UUID Definitions
 * @{
 * @ingroup ble_uuid
 * @details BLE GATT descriptors UUID definitions.
 * @note https://www.bluetooth.com/specifications/gatt/descriptors/
 */
#define GATT_DECL_PRIMARY_SERVICE                                           0x2800    /**< Primary Service Descriptor. */
#define GATT_DECL_SECONDARY_SERVICE                                         0x2801    /**< Secondary Service Descriptor. */
#define GATT_DECL_INCLUDE                                                   0x2802    /**< Included Service Descriptor. */
#define GATT_DECL_CHARACTERISTIC                                            0x2803    /**< Characteristic Descriptor. */


#define GATT_DESC_CHARC_EXTENDED_PROPERTIES                                 0x2900    /**< Characteristic Extended Properties Descriptor. */
#define GATT_DESC_CHARC_USER_DESCRIPTION                                    0x2901    /**< Characteristic User Description Descriptor. */
#define GATT_DESC_CLIENT_CHARC_CONFIGURATION                                0x2902    /**< Client Characteristic Configuration Descriptor. */
#define GATT_DESC_SERVER_CHARC_CONFIGURATION                                0x2903    /**< Server Characteristic Configuration Descriptor. */
#define GATT_DESC_CHARC_PRESENTATION_FORMAT                                 0x2904    /**< Characteristic Presentation Format Descriptor. */
#define GATT_DESC_CHARC_AGGREGATE_FORMAT                                    0x2905    /**< Characteristic Aggregate Format Descriptor. */
#define GATT_DESC_VALID_RANGE                                               0x2906    /**< Valid Range Descriptor. */
#define GATT_DESC_EXTERNAL_REPORT_REFERENCE                                 0x2907    /**< External Report Reference Descriptor. */
#define GATT_DESC_REPORT_REFERENCE                                          0x2908    /**< Report Reference Descriptor. */
/** @} */


/**
 * @defgroup bleUnit_uuid BLE Unit UUID Definitions
 * @{
 * @ingroup ble_uuid
 * @details BLE unit UUID definitions.
 */
#define GATT_UNIT_UNITLESS                                                  0x2700    /**< Unitless UUID.*/
#define GATT_UNIT_LENGTH__METRE                                             0x2701    /**< Length (metre) UUID.*/
#define GATT_UNIT_MASS__KILOGRAM                                            0x2702    /**< Mass (kilogram) UUID.*/
#define GATT_UNIT_TIME__SECOND                                              0x2703    /**< Time (second) UUID.*/
#define GATT_UNIT_ELECTRIC_CURRENT__AMPERE                                  0x2704    /**< Electric Current (ampere) UUID.*/
#define GATT_UNIT_THERMODYNAMIC_TEMPERATURE__KELVIN                         0x2705    /**< Thermodynamic Temperature (kelvin) UUID.*/
#define GATT_UNIT_AMOUNT_OF_SUBSTANCE__MOLE                                 0x2706    /**< Amount of Substance (mole) UUID.*/
#define GATT_UNIT_LUMINOUS_INTENSITY__CANDELA                               0x2707    /**< Luminous Intensity (candela) UUID.*/
#define GATT_UNIT_AREA__SQUARE_METRES                                       0x2710    /**< Area (square metres) UUID.*/
#define GATT_UNIT_VOLUME__CUBIC_METRES                                      0x2711    /**< Volume (cubic metres) UUID.*/
#define GATT_UNIT_VELOCITY__METRES_PER_SECOND                               0x2712    /**< Velocity (meres oer second) UUID.*/
#define GATT_UNIT_ACCELERATION__METRES_PER_SECOND_SQUARED                   0x2713    /**< Acceleration (metres per second squared) UUID.*/
#define GATT_UNIT_WAVENUMBER__RECIPROCAL_METRE                              0x2714    /**< Wavenumber (reciprocal metre) UUID.*/
#define GATT_UNIT_DENSITY__KILOGRAM_PER_CUBIC_METRE                         0x2715    /**< Density (kilogram per cubic metre) UUID.*/
#define GATT_UNIT_SURFACE_DENSITY__KILOGRAM_PER_SQUARE_METRE                0x2716    /**< Surface Density (kilogram per square metre) UUID.*/
#define GATT_UNIT_SPECIFIC_VOLUME__CUBIC_METRE_PER_KILOGRAM                 0x2717    /**< Specific Volume (cubic metre per kilogram) UUID.*/
#define GATT_UNIT_CURRENT_DENSITY__AMPERE_PER_SQUARE_METRE                  0x2718    /**< Current Density (ampere per square metre) UUID.*/
#define GATT_UNIT_MAGNETIC_FIELD_STRENGTH__AMPERE_PER_METRE                 0x2719    /**< Magnetic Field Strength (ampere per metre) UUID.*/
#define GATT_UNIT_AMOUNT_CONCENTRATION__MOLE_PER_CUBIC_METRE                0x271A    /**< Aount Concentration (molde per cubic metre) UUID.*/
#define GATT_UNIT_MASS_CONCENTRATION__KILOGRAM_PER_CUBIC_METRE              0x271B    /**< Mass Concentration (kilogram per cubic meter) UUID.*/
#define GATT_UNIT_LUMINANCE__CANDELA_PER_SQUARE_METRE                       0x271C    /**< Luminance (candela per square metre) UUID.*/
#define GATT_UNIT_REFRACTIVE_INDEX                                          0x271D    /**< Refractive (index) UUID.*/
#define GATT_UNIT_RELATIVE_PERMEABILITY                                     0x271E    /**< Relative (permeability) UUID.*/
#define GATT_UNIT_PLANE_ANGLE__RADIAN                                       0x2720    /**< PlaneAngle (radian) UUID.*/
#define GATT_UNIT_SOLID_ANGLE__STERADIAN                                    0x2721    /**< Solid Angle (steradian) UUID.*/
#define GATT_UNIT_FREQUENCY__HERTZ                                          0x2722    /**< Frequency (hertz) UUID.*/
#define GATT_UNIT_FORCE__NEWTON                                             0x2723    /**< Force (newton) UUID.*/
#define GATT_UNIT_PRESSURE__PASCAL                                          0x2724    /**< Pressure (pascal) UUID.*/
#define GATT_UNIT_ENERGY__JOULE                                             0x2725    /**< Energy (joule) UUID.*/
#define GATT_UNIT_POWER__WATT                                               0x2726    /**< Power (watt) UUID.*/
#define GATT_UNIT_ELECTRIC_CHARGE__COULOMB                                  0x2727    /**< Electric Charge (coulomb) UUID.*/
#define GATT_UNIT_ELECTRIC_POTENTIAL_DIFFERENCE__VOLT                       0x2728    /**< Electric Potential Difference (volt) UUID.*/
#define GATT_UNIT_CAPACITANCE__FARAD                                        0x2729    /**< Capacitance (farad) UUID.*/
#define GATT_UNIT_ELECTRIC_RESISTANCE__OHM                                  0x272A    /**< Electric Resistance (ohm) UUID.*/
#define GATT_UNIT_ELECTRIC_CONDUCTANCE__SIEMENS                             0x272B    /**< Electric Conductance (siemens) UUID.*/
#define GATT_UNIT_MAGNETIC_FLEX__WEBER                                      0x272C    /**< Magnetic Flex (weber) UUID.*/
#define GATT_UNIT_MAGNETIC_FLEX_DENSITY__TESLA                              0x272D    /**< Magnetic Flex Density (tesla) UUID.*/
#define GATT_UNIT_INDUCTANCE__HENRY                                         0x272E    /**< Inductance (henry) UUID.*/
#define GATT_UNIT_CELSIUS_TEMPERATURE__DEGREE_CELSIUS                       0x272F    /**< Celsius Temperature (degree celsius) UUID.*/
#define GATT_UNIT_LUMINOUS_FLUX__LUMEN                                      0x2730    /**< Luminous Flux (lumen) UUID.*/
#define GATT_UNIT_ILLUMINANCE__LUX                                          0x2731    /**< Illuminance (lux) UUID.*/
#define GATT_UNIT_ACTIVITY_REFERRED_TO_A_RADIONUCLIDE__BECQUEREL            0x2732    /**< Activity Referred to a Radionuclide (becquerel) UUID.*/
#define GATT_UNIT_ABSORBED_DOSE__GRAY                                       0x2733    /**< Absorbed Dose (Gray) UUID.*/
#define GATT_UNIT_DOSE_EQUIVALENT__SIEVERT                                  0x2734    /**< Dose Equivalent (sievert) UUID.*/
#define GATT_UNIT_CATALYTIC_ACTIVITY__KATAL                                 0x2735    /**< Catalytic Activity (katal) UUID.*/
#define GATT_UNIT_DYNAMIC_VISCOSITY__PASCAL_SECOND                          0x2740    /**< Dynamic Viscosity (pascal second) UUID.*/
#define GATT_UNIT_MOMENT_OF_FORCE__NEWTON_METRE                             0x2741    /**< Moment of Force (newton metre) UUID.*/
#define GATT_UNIT_SURFACE_TENSION__NEWTON_PER_METRE                         0x2742    /**< Surface Tension (newton per metre) UUID.*/
#define GATT_UNIT_ANGULAR_VELOCITY__RADIAN_PER_SECOND                       0x2743    /**< Angular Velocity (radian per second) UUID.*/
#define GATT_UNIT_ANGULAR_ACCELERATION__RADIAN_PER_SECOND_SQUARED           0x2744    /**< Angular Acceleration (radian per second squared) UUID.*/
#define GATT_UNIT_HEAT_FLUX_DENSITY__WATT_PER_SQUARE_METRE                  0x2745    /**< Heat Flux Density (watt per square metre) UUID.*/
#define GATT_UNIT_HEAT_CAPACITY__JOULE_PER_KELVIN                           0x2746    /**< Heat Capacity (joule per kelvin) UUID.*/
#define GATT_UNIT_SPECIFIC_HEAT_CAPACITY__JOULE_PER_KILOGRAM_KELVIN         0x2747    /**< Specific Heat Capacity (joule per kilogram kelvin) UUID.*/
#define GATT_UNIT_SPECIFIC_ENERGY__JOULE_PER_KILOGRAM                       0x2748    /**< Specific Energy (joule per kilogram) UUID.*/
#define GATT_UNIT_THERMAL_CONDUCTIVITY__WATT_PER_METRE_KELVIN               0x2749    /**< Thermal Conductivity (watt per metre kelvin) UUID.*/
#define GATT_UNIT_ENERGY_DENSITY__JOULE_PER_CUBIC_METRE                     0x274A    /**< Energy Density (joule per cubic metre) UUID.*/
#define GATT_UNIT_ELECTRIC_FIELD_STRENGTH__VOLT_PER_METRE                   0x274B    /**< Electric Field Strength (volt per metre) UUID.*/
#define GATT_UNIT_ELECTRIC_CHARGE_DENSITY__COULOMB_PER_CUBIC_METRE          0x274C    /**< Electric Charge Density (coulomb per cubic metre) UUID.*/
#define GATT_UNIT_SURFACE_CHARGE_DENSITY__COULOMB_PER_SQUARE_METRE          0x274D    /**< Surface Charge Density (coulomb per square metre) UUID.*/
#define GATT_UNIT_ELECTRIC_FLUX_DENSITY__COULOMB_PER_SQUARE_METRE           0x274E    /**< Electric Flux Density (coulomb per square metre) UUID.*/
#define GATT_UNIT_PERMITTIVITY__FARAD_PER_METRE                             0x274F    /**< Permittivity (farad per metre) UUID.*/
#define GATT_UNIT_PERMEABILITY__HENRY_PER_METRE                             0x2750    /**< Permeability (henry per metre) UUID.*/
#define GATT_UNIT_MOLAR_ENERGY__JOULE_PER_MOLE                              0x2751    /**< Molar Energy (joule per mole) UUID.*/
#define GATT_UNIT_MOLAR_ENTROPY__JOULE_PER_MOLE_KELVIN                      0x2752    /**< Molar Entropy (joule per mole kelvin) UUID.*/
#define GATT_UNIT_EXPOSURE__COULOMB_PER_KILOGRAM                            0x2753    /**< Exposure (coulomb per kilogram) UUID.*/
#define GATT_UNIT_ABSORBED_DOSE_RATE__GRAY_PER_SECOND                       0x2754    /**< Absorbed Dose Rate (gray per second) UUID.*/
#define GATT_UNIT_RADIANT_INTENSITY__WATT_PER_STERADIAN                     0x2755    /**< Radiant Intensity (watt per steradian) UUID.*/
#define GATT_UNIT_RADIANCE__WATT_PER_SQUARE_METER_STERADIAN                 0x2756    /**< Radiance (watt per square metre sterdian) UUID.*/
#define GATT_UNIT_CATALYTIC_ACTIVITY_CONCENTRATION__KATAL_PER_CUBIC_METRE   0x2757    /**< Catalytic Activity Concentration (katal per cubic metre) UUID.*/
#define GATT_UNIT_TIME__MINUTE                                              0x2760    /**< Time (minute) UUID.*/
#define GATT_UNIT_TIME__HOUR                                                0x2761    /**< Time (hour) UUID.*/
#define GATT_UNIT_TIME__DAY                                                 0x2762    /**< Time (day) UUID.*/
#define GATT_UNIT_PLANE_ANGLE__DEGREE                                       0x2763    /**< Plane Angle (degree) UUID.*/
#define GATT_UNIT_PLANE_ANGLE__MINUTE                                       0x2764    /**< Plane Angle (minute) UUID.*/
#define GATT_UNIT_PLANE_ANGLE__SECOND                                       0x2765    /**< Plane Angle (second) UUID.*/
#define GATT_UNIT_AREA__HECTARE                                             0x2766    /**< Area (hectare) UUID.*/
#define GATT_UNIT_VOLUME__LITRE                                             0x2767    /**< Volume (litre) UUID.*/
#define GATT_UNIT_MASS__TONNE                                               0x2768    /**< Mass (tonne) UUID.*/
#define GATT_UNIT_PRESSURE__BAR                                             0x2780    /**< Pressure (bar) UUID.*/
#define GATT_UNIT_PRESSURE__MILLIMETRE_OF_MERCURY                           0x2781    /**< Pressure (millimeter of mercury) UUID.*/
#define GATT_UNIT_LENGTH__ANGSTROM                                          0x2782    /**< Length (angstrom) UUID.*/
#define GATT_UNIT_LENGTH__NAUTICAL_MILE                                     0x2783    /**< Length (nautical mile) UUID.*/
#define GATT_UNIT_AREA__BARN                                                0x2784    /**< Area (barn) UUID.*/
#define GATT_UNIT_VELOCITY__KNOT                                            0x2785    /**< Velocity (knot) UUID.*/
#define GATT_UNIT_LOGARITHMIC_RADIO_QUANTITY__NEPER                         0x2786    /**< Logarithmic Radio Quantity (neper) UUID.*/
#define GATT_UNIT_LOGARITHMIC_RADIO_QUANTITY__BEL                           0x2787    /**< Logarithmic Radio Quantity (bel) UUID.*/
#define GATT_UNIT_LENGTH__YARD                                              0x27A0    /**< Length (yard) UUID.*/
#define GATT_UNIT_LENGTH__PARSEC                                            0x27A1    /**< Length (parsec) UUID.*/
#define GATT_UNIT_LENGTH__INCH                                              0x27A2    /**< Length (inch) UUID.*/
#define GATT_UNIT_LENGTH__FOOT                                              0x27A3    /**< Length (foot) UUID.*/
#define GATT_UNIT_LENGTH__MILE                                              0x27A4    /**< Length (mile) UUID.*/
#define GATT_UNIT_PRESSURE__POUND_FORCE_PER_SQUARE_INCH                     0x27A5    /**< Pressure (pound force per square inch) UUID.*/
#define GATT_UNIT_VELOCITY__KILOMETRE_PER_HOUR                              0x27A6    /**< Velocity (kilometer per hour) UUID.*/
#define GATT_UNIT_VELOCITY__MILE_PER_HOUR                                   0x27A7    /**< Velocity (mile per hour) UUID.*/
#define GATT_UNIT_ANGULAR_VELOCITY__REVOLUTION_PER_MINUTE                   0x27A8    /**< Angular Velocity (revolution per minute) UUID.*/
#define GATT_UNIT_ENERGY__GRAM_CALORIE                                      0x27A9    /**< Energy (gram calorie) UUID.*/
#define GATT_UNIT_ENERGY__KILOGRAM_CALORIE                                  0x27AA    /**< Energy (kilogram calorie) UUID.*/
#define GATT_UNIT_ENERGY__KILOWATT_HOUR                                     0x27AB    /**< Energy (kilowatt hour) UUID.*/
#define GATT_UNIT_THERMODYNAMIC_TEMPERATURE__DEGREE_FAHRENHEIT              0x27AC    /**< Thermodynamic Temperature (degree fahrenheit) UUID.*/
#define GATT_UNIT_PERCENTAGE                                                0x27AD    /**< Percentage UUID.*/
#define GATT_UNIT_PER_MILLE                                                 0x27AE    /**< Per Mile UUID.*/
#define GATT_UNIT_PERIOD__BEATS_PER_MINUTE                                  0x27AF    /**< Period (beats per minute) UUID.*/
#define GATT_UNIT_ELECTRIC_CHARGE__AMPERE_HOURS                             0x27B0    /**< Electric Charge (ampere hour) UUID.*/
#define GATT_UNIT_MASS_DENSITY__MILLIGRAM_PER_DECILITRE                     0x27B1    /**< Mass Density (milligram per decilitre) UUID.*/
#define GATT_UNIT_MASS_DENSITY__MILLIMOLE_PER_LITRE                         0x27B2    /**< Mass Density (milligram per litre) UUID.*/
#define GATT_UNIT_TIME__YEAR                                                0x27B3    /**< Time (year) UUID.*/
#define GATT_UNIT_TIME__MONTH                                               0x27B4    /**< Time (month) UUID.*/
/** @} */



#endif // _BLE_UUID_H_


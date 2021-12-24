/************************************************************************
 *
 * File Name  : ble_profile_def.c
 * Description: This file contains the definitions of BLE profiles.
 *
 *******************************************************************/
#include <stdio.h>
#include "ble_profile.h"

/**************************************************************************
 * Profile Definitions
 **************************************************************************/
/** Service Combination #00
 * @note included @ref ATT_GAP_SERVICE, @ref ATT_GATT_SERVICE, @ref ATT_DIS_SERVICE and @ref ATT_UDF01S_SERVICE services
*/
const ATTRIBUTE_BLE *const ATT_SERVICE_COMB00[] =
{
    &ATT_NULL_INVALID,       //mandatory, don't remove it.
    ATT_GAP_SERVICE
    ATT_GATT_SERVICE
    ATT_DIS_SERVICE
    ATT_UDF01S_SERVICE
};


/**************************************************************************
 * BLE Connection Link Definitions
 **************************************************************************/
/** BLE Connection Links Definition
 * @attention Do NOT modify the name of this definition.
 * @note If there does not support server or client please set to "((const ATTRIBUTE_BLE **)0)" which means NULL.
*/
const ATTR_DB_Role_by_ID ATT_DB_LINK[] =
{
    // Link 0
    {
        ATT_SERVICE_COMB00,            // Client Profile
        ((const ATTRIBUTE_BLE **)0),   // Server Profile
    },
    // Link 1
    {
        ATT_SERVICE_COMB00,            // Client Profile
        ((const ATTRIBUTE_BLE **)0),   // Server Profile
    },
    // Link 2
    {
        ATT_SERVICE_COMB00,            // Client Profile
        ((const ATTRIBUTE_BLE **)0),   // Server Profile
    },
};


/** BLE Connection Link Parameter Definition
 * @attention Every active role in active links shall be defined related link parameters.
*/
ATTRIBUTE_BLE_Hdl_Para ATT_Hdl_Para_LinkC00[SIZE_ARRAY_ROW(ATT_SERVICE_COMB00)]; // Link 0/ Client
ATTRIBUTE_BLE_Hdl_Para ATT_Hdl_Para_LinkC01[SIZE_ARRAY_ROW(ATT_SERVICE_COMB00)]; // Link 1/ Client
ATTRIBUTE_BLE_Hdl_Para ATT_Hdl_Para_LinkC02[SIZE_ARRAY_ROW(ATT_SERVICE_COMB00)]; // Link 2/ Client



/** BLE Connection Link Parameter Table Definition
 * @attention Do NOT modify the name of this definition.
 * @note If there does not support server or client please set to "((ATTRIBUTE_BLE_Hdl_Para *)0)" which means NULL.
*/
const ATTR_DB_Mapping_by_ID ATT_DB_MAPPING[] =
{
    // Link 0
    {
        ATT_Hdl_Para_LinkC00,             // Client Link Parameter
        ((ATTRIBUTE_BLE_Hdl_Para *)0),    // Server Link Parameter
    },
    // Link 1
    {
        ATT_Hdl_Para_LinkC01,             // Client Link Parameter
        ((ATTRIBUTE_BLE_Hdl_Para *)0),    // Server Link Parameter
    },
    // Link 2
    {
        ATT_Hdl_Para_LinkC02,             // Client Link Parameter
        ((ATTRIBUTE_BLE_Hdl_Para *)0),    // Server Link Parameter
    },
};



/** BLE Connection Link Mapping Size Definition
 * @attention Do NOT modify the name of this definition.
 * @note If there does not support server or client please set to 0.
*/
const ATTR_DB_Mapping_by_ID_size ATT_DB_MAPPING_SIZE[] =
{
    // Link 0
    {
        SIZE_ARRAY_ROW(ATT_SERVICE_COMB00),    // Client Link Mapping Size
        0,                                     // Server Link Mapping Size
    },
    // Link 1
    {
        SIZE_ARRAY_ROW(ATT_SERVICE_COMB00),    // Client Link Mapping Size
        0,                                     // Server Link Mapping Size
    },
    // Link 2
    {
        SIZE_ARRAY_ROW(ATT_SERVICE_COMB00),    // Client Link Mapping Size
        0,                                     // Server Link Mapping Size
    },
};


/** Maximum Number of Host Connection Link Definition
 * @attention Do NOT modify this definition.
 * @note Defined for host layer.
*/
const uint8_t MAX_NUM_CONN_HOST = (SIZE_ARRAY_ROW(ATT_DB_MAPPING_SIZE));


/** Host Connection Link Information Definition
 * @attention Do NOT modify this definition.
 * @note Defined for host layer.
*/
uint8_t *param_rsv_host[SIZE_ARRAY_ROW(ATT_DB_MAPPING_SIZE)][(REF_SIZE_LE_HOST_PARA >> 2)];


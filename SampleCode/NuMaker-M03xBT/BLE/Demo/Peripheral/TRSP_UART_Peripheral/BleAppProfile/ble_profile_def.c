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
 * @note included @ref ATT_GAP_SERVICE, @ref ATT_DIS_SERVICE and @ref ATT_UDF01S_SERVICE services
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
        ((const ATTRIBUTE_BLE **)0),    // Client Profile
        ATT_SERVICE_COMB00,             // Server Profile
    },
};


/** BLE Connection Link Parameter Definition
 * @attention Every active role in active links shall be defined related link parameters.
*/
ATTRIBUTE_BLE_Hdl_Para ATT_Hdl_Para_LinkP00[SIZE_ARRAY_ROW(ATT_SERVICE_COMB00)]; // Link 0



/** BLE Connection Link Parameter Table Definition
 * @attention Do NOT modify the name of this definition.
 * @note If there does not support server or client please set to "((ATTRIBUTE_BLE_Hdl_Para *)0)" which means NULL.
*/
const ATTR_DB_Mapping_by_ID ATT_DB_MAPPING[] =
{
    {
        ((ATTRIBUTE_BLE_Hdl_Para *)0),    // Client Link Parameter
        ATT_Hdl_Para_LinkP00,             // Server Link Parameter
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
        0,                                    // Client Link Mapping Size
        SIZE_ARRAY_ROW(ATT_SERVICE_COMB00),   // Server Link Mapping Size
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


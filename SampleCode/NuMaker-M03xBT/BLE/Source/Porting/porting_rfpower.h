#ifndef _BLE_RF_POWER_DEFINITION_H_
#define _BLE_RF_POWER_DEFINITION_H_
/**************************************************************************//**
 * @file       porting_rfpower.h
 * @brief
 *
 *
 * @defgroup rfPowerDef RF Power Definition
 * @{
 * @ingroup mcuPortingDef
 * @details RF Power definition.
 * @}
 **************************************************************************/

#include "ble_stack_status.h"
#include "ble_cmd.h"

/** BLE TX power level table
 * @ingroup rfPowerDef
 * @attention  BLE TX power level table can be modified but please noted that the default TX power setting is set to index 0. \n
 * @note  @ref TXPOWER_TABLE is defined in "porting_rfpower.c.
*/
extern const int8_t (*TXPOWER_TABLE)[6];


/** The size of defined BLE TX power level table
 * @ingroup rfPowerDef
 * @note  @ref TXPOWER_TABLE is defined in "porting_rfpower.c.
*/
extern uint8_t size_TXPOWER_TABLE;


/** Define Tx Power Level Mapping Index for user uses.
 * @enum TxPowerLevel
 * @note The index is mapping to the TX power level table, and the default Tx power is set to the index "0" of "TXPOWER_HV_TABLE".
*/
typedef enum txPowerLevel
{
    TXPOWER_9DBM,      /**< Default Setting is index 0 = TX power is +9dBm (mapping to "TXPOWER_REMAP" index 0). */
    TXPOWER_8DBM,      /**< TX power is +8dBm (mapping to "TXPOWER_REMAP" index 1). */
    TXPOWER_7DBM,      /**< TX power is +7dBm (mapping to "TXPOWER_REMAP" index 2). */
    TXPOWER_6DBM,      /**< TX power is +6dBm (mapping to "TXPOWER_REMAP" index 3). */
    TXPOWER_5DBM,      /**< TX power is +5dBm (mapping to "TXPOWER_REMAP" index 4). */
    TXPOWER_4DBM,      /**< TX power is +4dBm (mapping to "TXPOWER_REMAP" index 5). */
    TXPOWER_3DBM,      /**< TX power is +3dBm (mapping to "TXPOWER_REMAP" index 6). */
    TXPOWER_2DBM,      /**< TX power is +2dBm (mapping to "TXPOWER_REMAP" index 7). */
    TXPOWER_1DBM,      /**< TX power is +1dBm (mapping to "TXPOWER_REMAP" index 8). */
    TXPOWER_0DBM,      /**< TX power is +0dBm (mapping to "TXPOWER_REMAP" index 9). */
    TXPOWER_m1DBM,     /**< TX power is -1dBm (mapping to "TXPOWER_REMAP" index 10). */
    TXPOWER_m2DBM,     /**< TX power is -2dBm (mapping to "TXPOWER_REMAP" index 11). */
    TXPOWER_m3DBM,     /**< TX power is -3dBm (mapping to "TXPOWER_REMAP" index 12). */
    TXPOWER_m4DBM,     /**< TX power is -4dBm (mapping to "TXPOWER_REMAP" index 13). */
    TXPOWER_m5DBM,     /**< TX power is -5dBm (mapping to "TXPOWER_REMAP" index 14). */
    TXPOWER_m6DBM,     /**< TX power is -6dBm (mapping to "TXPOWER_REMAP" index 15). */
    TXPOWER_m7DBM,     /**< TX power is -7dBm (mapping to "TXPOWER_REMAP" index 16). */
    TXPOWER_m8DBM,     /**< TX power is -8dBm (mapping to "TXPOWER_REMAP" index 17). */
    TXPOWER_m9DBM,     /**< TX power is -9dBm (mapping to "TXPOWER_REMAP" index 18). */
    TXPOWER_m10DBM,    /**< TX power is -10dBm (mapping to "TXPOWER_REMAP" index 19). */
    TXPOWER_m15DBM,    /**< TX power is -15dBm (mapping to "TXPOWER_REMAP" index 20). */
    TXPOWER_m20DBM,    /**< TX power is -20dBm (mapping to "TXPOWER_REMAP" index 21). */
} TxPowerLevel;

/** @} */ // (@ingroup rfPowerTableDef)


/**************************************************************************
* Functions
**************************************************************************/

/* Set the TX Power remapping table for different chipset */
void setBLE_TxPower_Wrap_Init(void);

/* Set BLE TX Power level according different chipset */
BleStackStatus setBLE_TxPower_Wrap(uint8_t power_index, BleMode bleMode);

#endif // (_BLE_RF_POWER_DEFINITION_H_)

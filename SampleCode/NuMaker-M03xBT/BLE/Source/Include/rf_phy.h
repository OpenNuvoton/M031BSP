/**************************************************************************//**
* @file       rf_phy.h
* @brief      This file provides the declaration that RF IC control needed.
*
* @defgroup rf_phy RF PHY
* @{
* @details  Provides the declaration that RF IC control needed. (rf_phy.h).
* @}
*****************************************************************************/

#ifndef __RF_PHY_H__
#define __RF_PHY_H__

#include <stdint.h>
#include "ble_cmd.h"


/**************************************************************************
 * Definitions
 **************************************************************************/
/** @defgroup rf_phy_def RF PHY Definition.
 * @{
 * @details Here shows the definitions of RF PHY.
 * @ingroup rf_phy
 * @}
 */

/** @defgroup rf_phy_def_chipId Chip ID Definition
 * @{
 * @ingroup rf_phy_def
 */
#define MP_A1                         102u          /**< chip_id = 102 (0x66). */
#define MP_A2                         103u          /**< chip_id = 103 (0x67). */
/** @} */



/** @defgroup mcuWakeupTime MCU Wake-Up Required Time from Sleep Mode
 * @{
 * @ingroup rf_phy_def
 *
 * This parameter adjusts the retention time required for the MCU to wake up from sleep mode, plus some necessary parameters set via SPI operation.
 * The unit of the MCU wake-up retention time is 125us.
 *
 * @note The range of MCU wake-up retention time is 125us to 2ms.
*/
#define MCU_WAKEUP_RETENSIONTIME_MAX  16            /**< 2ms = 16*125us */
#define MCU_WAKEUP_RETENSIONTIME_MIN  1             /**< 125us = 1*125us */
/** @} */



/** @defgroup rf_phy_def_TRxWritePort TX/RX Buffer Write Port Definition
 * @{
 * @ingroup rf_phy_def
 */
#define RFIP_REG_254                  254U          /**< Register 254. */
#define RFIP_REG_255                  255U          /**< Register 255. */


#define TX_BUFFER_WRITE_PORT          RFIP_REG_254  /**< TX Buffer Write Port. */
#define RX_BUFFER_READ_PORT           RFIP_REG_255  /**< RX Buffer Write Port. */
/** @} */


/**
 * @ingroup rf_phy_def
 * @defgroup ble_cmd_rfMode BLE RF Mode Definition
 * @{
 * @details BLE RF mode definition.
 * @name BleRFMode
 * @brief  The definition of BLE RF Mode.
 * @{
 */
typedef uint8_t BleRF_Mode;
#define BLERFMODE_ACTIVE              0x00          /**< RF is in active mode. */
#define BLERFMODE_SLEEP               0x01          /**< RF entered sleep mode. */
#define BLERFMODE_IDLE                0x02          /**< RF is in idle mode. */
/** @} */
/** @} */


/** @defgroup rf_phy_def_xtalFreq Crystal Frequency Definition
 * @{
 * @ingroup rf_phy_def
 */
typedef uint8_t Xtal_Freq;
#define XTAL_16M                      0x00          /**< 16MHz. */
#define XTAL_32M                      0x01          /**< 32MHz. */
/** @} */


/** @defgroup rf_phy_def_powerMode Power Regulator Definition
 * @{
 * @ingroup rf_phy_def
 */
typedef uint8_t PowerRegulator_Mode;
#define LDO_REGULATOR                 0x00          /**< LDO. */
#define DCDC_REGULATOR                0x01          /**< DCDC. */
/** @} */


/**************************************************************************
 * Functions
 **************************************************************************/

/** @defgroup rf_phy_function RF PHY Function
 * @{
 * @details Here shows the functions of RF PHY.
 * @ingroup rf_phy
 * @}
 */


/** Initializes the BLE RF Phy.
 *
 * @ingroup rf_phy_function
 *
 * @attention Must initial BLE RF Phy after @ref setRF_SpiIoMapping.
 *
 * @param[in] powerMode : @ref rf_phy_def_powerMode "power regulator."
 * @param[in] xtalFeq : @ref rf_phy_def_xtalFreq "crystal frequency."
 *
 * @retval  BLESTACK_STATUS_ERR_INVALID_PARAM : invalid parameter.
 * @retval  BLESTACK_STATUS_SUCCESS : RF initialization is successful.
 */
BleStackStatus setRF_Init(PowerRegulator_Mode powerMode, uint8_t xtalFeq);


/** Get Current BLE Device RF Mode
 *
 *  This function is used to get current RF mode.
 *
 * @ingroup rf_phy_function
 *
 * @retval  BLERFMODE_ACTIVE : The BLE device RF is in active mode.
 * @retval  BLERFMODE_SLEEP  : The BLE device RF is in sleep mode.
 */
BleRF_Mode getRF_Mode(void);


/** Set BLE RSSI Offset Value to adjust RSSI Value
 *
 * @ingroup rf_phy_function
 *
 * @note The default BLE RSSI Offset Value is +26dBm.
 *
 * @param[in] offset : rssi offset value.
 * @return none
 */
void setRF_RssiValueOffset(uint8_t offset);


/** Set Crystal Clock Output
 *
 * This function is used to enable crystal 16MHz Clock output.
 *
 * @ingroup rf_phy_function
 *
 * @attention This command MUST be called before @ref setRF_Init. \n
 * @attention RF sleep mode will be disabled automatically when user enable crystal clock (16MHz) output.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : Crystal clock output can NOT be enabled while running BLE stack.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting completed.
 */
BleStackStatus setRF_EnableXtalClkOutput(void);


/** Set RF Enter Deep Sleep Mode
 *
 * @ingroup rf_phy_function
 *
 * @attention  RF can NOT enter Deep Sleep Mode if enabled crystal clock output. \n
 * @attention  It will not wake up until call @ref setRF_WakeUpFromDeepSleep. \n
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : RF phy or BLE stack has not initialized.
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED : This command is not supported if crystal clock output enabled.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting completed.
 */
BleStackStatus setRF_EnterDeepSleep(void);


/** Set RF External Wakeup From Deep Sleep Mode
 *
 * @ingroup rf_phy_function
 *
 * @attention   This command is not supported if enabled crystal clock output. \n
 * @note   After external wakeup from deep sleep mode, the BLE stack will do @ref setRF_Init and @ref setBLE_BleStackInit again. \n
 *         Then BLE is in standby mode.
 *
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : RF phy or BLE stack has not initialized or not in deep sleep mode.
 * @retval BLESTACK_STATUS_ERR_NOT_SUPPORTED : This command is not supported if crystal clock output enabled.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting completed.
 */
BleStackStatus setRF_WakeUpFromDeepSleep(void);



/** Enable BLE Kernel.
 *  Must do it after @ref setRF_SpiIoMapping and @ref setRF_Init.
 *
 * @ingroup rf_phy_function
 *
 * @note If no any BLE Stack (LL&Host) task is running then run user application task. \n
 * The task priority is LL>Host>User_app.
 *
 * @retval  BLESTACK_STATUS_FREE     : Stack (LL&Host task) is not running, can run user application task.
 * @retval  BLESTACK_STATUS_ERR_BUSY : Stack (LL&Host task) is running, can NOT run user application task.
 */
BleStackStatus setBLE_KernelStateHandle(void);



/** Get BLE Device Chip ID
 *
 *  This function is used to get BLE Device Chip ID.
 *
 * @ingroup rf_phy_function
 *
 * @return  @ref rf_phy_def_chipId "Chip ID"
 */
uint8_t getBLE_ChipId(void);



/** Get TX FIFI Address
 *
 *  This function is used to get TX FIFO RAM start address.
 *
 * @ingroup rf_phy_function
 *
 * @return  TX FIFO address.
 */
uint32_t getBLE_TxFIFOAddr(void);



/** Set MCU Wake-up Retention Time
 *
 * This parameter adjusts the retention time required for the MCU to wake up from sleep mode, plus some necessary parameters set via SPI operation. \n
 * The unit of the MCU wake-up retention time is 125us.
 *
 * @ingroup rf_phy_function
 *
 * @attention This command MUST be called before @ref setRF_Init. \n
 * @note The default MCU Wake-up Retention Time is set to 1 (125us = 1*125us). \n
 * @note MCU wake-up retention time range is from @ref MCU_WAKEUP_RETENSIONTIME_MIN to @ref MCU_WAKEUP_RETENSIONTIME_MAX. \n
 *
 * @param[in] retentionTime : MCU wake-up retention time in 125us.
 * @retval BLESTACK_STATUS_ERR_INVALID_STATE : MCU Wake-up Retention Time can NOT be modified while running BLE stack.
 * @retval BLESTACK_STATUS_ERR_INVALID_PARAM : Invalid parameter.
 * @retval BLESTACK_STATUS_SUCCESS  : Setting completed.
 */
BleStackStatus setMCU_WakeupRetentionTime(uint8_t retentionTime);

#endif

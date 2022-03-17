#ifndef __MCU_DEFINITION_H__
#define __MCU_DEFINITION_H__

#include <stdint.h>
#include "NuMicro.h"

/**************************************************************************
 * Definitions
 **************************************************************************/
/**
 * @defgroup porting Porting
 * @{
 * @details MCU porting & BLE Bonding Porting Related \n
 * -  MCU Definition
 * -  MCU SPI Porting Definition
 * -  MCU MISC Porting Definition
 * -  MCU Flash Porting Definition
 * -  RF Power Definition
 * -  BLE Bonding Porting Related
 *
 * @defgroup mcuPortingDef MCU Porting Related
 * @{
 * @details MCU porting related definitions and functions. \n
 * -  MCU Definition
 * -  MCU SPI Porting Definition
 * -  MCU MISC Porting Definition
 * -  MCU Flash Porting Definition
 * -  RF Power Definition
 *
 * @defgroup mcuDef MCU Definition
 * @{
 * @details MCU setting definition for user configuration are listed as following. \n
 * @verbatim
   1. _USE_MCU_CLK_
   2. CPU_CLOCK_RATE
   3. PCLK_DIV
   4. InterruptDisable
   5. InterruptEnable
 @endverbatim
 * @}@}@}
 **************************************************************************/

/**
 * @ingroup mcuDef
 * @{
 * @brief Defined the MCU chip.
 */
#define _CHIP_M031BT                        0
#define _CHIP_M032BT                        1
#define _CHIP_SELECTION_                    _CHIP_M031BT


/* Use the timer to check BLE kernel task in time if defined ENABLE_DEF. */
#define BLE_USE_TIMER                       ENABLE_DEF


/**
 * @ingroup mcuDef
 * @defgroup mcuClockDef MCU Clock Source Definition
 * @{
 * @brief Defined supported MCU clock source for user.
 */
#define MCU_CLK_SOURCE_HXT                  0               /**< HXT */
#define MCU_CLK_SOURCE_HIRC                 1               /**< HIRC */
#define MCU_CLK_SOURCE_PLL                  2               /**< PLL */
/** @} */

/** Selected the clock source from @ref mcuClockDef "MCU clock source definition list".
 * @ingroup mcuDef
*/
#if (_CHIP_SELECTION_ == _CHIP_M031BT)
#define _USE_MCU_CLK_                       MCU_CLK_SOURCE_HIRC     /**< Selected the clock source from @ref mcuClockDef "MCU clock source definition list" */
#elif (_CHIP_SELECTION_ == _CHIP_M032BT)
#define _USE_MCU_CLK_                       MCU_CLK_SOURCE_HIRC     /**< Selected the clock source from @ref mcuClockDef "MCU clock source definition list" */
#endif


/**
 * @ingroup mcuDef
 * @{
 */
#if (_CHIP_SELECTION_ == _CHIP_M031BT)
#define CPU_CLOCK_RATE                      48000000u       /**< CPU clock rate */
#define PCLK_DIV                            2u              /**< PCLK=CPU_CLOCK_RATE/PCLK_DIV. it is better to choose PCLK=8*N (24MHz or 32MHz) for SPI_clock=12M */
#elif (_CHIP_SELECTION_ == _CHIP_M032BT)
#if (_USE_MCU_CLK_ == MCU_CLK_SOURCE_HIRC)
#define CPU_CLOCK_RATE                      48000000u       /**< CPU clock rate */
#elif (_USE_MCU_CLK_ == MCU_CLK_SOURCE_PLL)
#define CPU_CLOCK_RATE                      72000000u       /**< CPU clock rate */
#endif
#define PCLK_DIV                            2u              /**< PCLK=CPU_CLOCK_RATE/PCLK_DIV. it is better to choose PCLK=8*N (24MHz or 32MHz) for SPI_clock=12M */
#endif


/**
 * @brief Different MCU may supports different interrupt definition.
 * @note If it is not ARM based MCU please undefined it and re-defined @ref InterruptDisable and @ref InterruptEnable.
 */
#define InterruptDisable                    __disable_irq   /**< Disable interrupt function definition for ARM based. */
#define InterruptEnable                     __enable_irq    /**< Enable interrupt function definition for ARM based. */
/** @} */

#endif  //__MCU_DEFINITION_H__


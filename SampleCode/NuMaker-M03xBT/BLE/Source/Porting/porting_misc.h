#ifndef __PORTING_MISC_H__
#define __PORTING_MISC_H__

#include "NuMicro.h"
#include "mcu_definition.h"

/**************************************************************************
 * Definitions
 **************************************************************************/
/**
 * @defgroup mcuMisc MCU Miscellaneous Porting
 * @{
 * @ingroup mcuPortingDef
 * @details MCU miscellaneous porting related definition and functions.
 * @}
 * @defgroup mcuMiscDef MCU Miscellaneous Porting Definition
 * @{
 * @ingroup mcuMisc
 * @details MCU miscellaneous setting definitions.
 * @}
 * @defgroup mcuBasicTypeDef Basic Type Porting Definition
 * @{
 * @ingroup mcuMiscDef
 * @details Basic type definitions.
 * @}
 * @defgroup mcuIOMappingDef MCU SPI I/O Re-mapping Definition
 * @{
 * @ingroup mcuMiscDef
 * @details Users can dynamically configure SPI pins through SPI I/O re-mapping settings.
 * @}
 * @defgroup mcuOtherDef MCU Other Definitions
 * @{
 * @ingroup mcuMiscDef
 * @}
 **************************************************************************/

/**
 * @ingroup mcuBasicTypeDef
 * @{
 */
#define ENABLE_DEF          ENABLE   /**< Define enable.   */
#define DISABLE_DEF         DISABLE  /**< Define disable.   */

#define TRUE_DEF            TRUE     /**< Define true.   */
#define FALSE_DEF           FALSE    /**< Define false.   */
/** @} */


/**
 * @ingroup mcuIOMappingDef
 * @{
 */
//GPIO_sel order(0:INT, 1:CS, 2:CK, 3:MISO, 4:MOSI)
#if (_CHIP_SELECTION_ == _CHIP_M031BT)
#define SPI_CS              PC2           /**< (GPIO0) Initial SPI CS pin.      */
#define SPI_CK              PD3           /**< (GPIO1) Initial SPI Clock pin.   */
#define SPI_MOSI            PD2           /**< (GPIO2) Initial SPI MOSI pin.    */
#define SPI_MISO            PD1           /**< (GPIO3) Initial SPI MISO pin.    */
#define DEFAULT_INT         PD0           /**< (GPIO4) Initial interrupt pin.   */
#define RESET_RF            PA12          /**< RF reset pin.   */
#elif (_CHIP_SELECTION_ == _CHIP_M032BT)
#define SPI_CS              PD12          /**< (GPIO0) Initial SPI CS pin.      */
#define SPI_CK              PF9           /**< (GPIO1) Initial SPI Clock pin.   */
#define SPI_MOSI            PF8           /**< (GPIO2) Initial SPI MOSI pin.    */
#define SPI_MISO            PF7           /**< (GPIO3) Initial SPI MISO pin.    */
#define DEFAULT_INT         PF6           /**< (GPIO4) Initial interrupt pin.   */
#define RESET_RF            PH4           /**< RF reset pin.   */
#endif
/** @} */ //(@ingroup mcuIOMappingDef)



/**************************************************************************
* Functions
**************************************************************************/
/** @defgroup mcuMiscFunc MCU Miscellaneous Porting Function
 * @{
 * @ingroup mcuMisc
 * @details MCU miscellaneous function definitions.
 * @}
 *@defgroup mcuIOMappingFunc MCU SPI I/O Re-mapping Function
 * @{
 * @details Here shows the MCU SPI I/O re-mapping function.
 * @ingroup mcuMiscFunc
 * @}
 * @defgroup mcuGpioFunc MCU GPIO Related Function
 * @{
 * @details Here shows the MCU GPIO related function definitions.
 * @ingroup mcuMiscFunc
 * @}
 * @defgroup mcuSysFunc MCU System Related Function
 * @{
 * @details Here shows the MCU system related function definitions.
 * @ingroup mcuMiscFunc
 * @}
 **************************************************************************/

/** MCU SPI IO mapping.
 *  Must do this after Power ON and MCU GPIO initialed.
 *
 * @ingroup mcuIOMappingFunc
 */
void setRF_SpiIoMapping(void);


/** MCU Implemented RF Reset
 *
 * @ingroup mcuGpioFunc
 */
void seBLE_GpioReset(void);


/** MCU Set Enable RF Interrupt Pin.
 *
 * @ingroup mcuGpioFunc
 */
void setBLE_GpioIntEnable(void);


/** MCU Set Disable RF Interrupt Pin.
 *
 * @ingroup mcuGpioFunc
 */
void setBLE_GpioIntDisable(void);


/** MCU Enter System Power Down Mode.
 *
 * @ingroup mcuSysFunc
 */
void setMCU_SystemPowerDown(void);


/** MCU Implemented Tiny Delay Function for BLE Stack.
 *
 * @ingroup mcuSysFunc
 * @param[in] u32Usec   : delay time in microseconds.
 */
void setMCU_TinyDelay(uint32_t u32Usec);


/** MCU Set Enable Timer for BLE kernel task.
 *
 * @ingroup mcuTimerFunc
 */
void setBLE_TimerInit(void);

#endif  //__PORTING_MISC_H__


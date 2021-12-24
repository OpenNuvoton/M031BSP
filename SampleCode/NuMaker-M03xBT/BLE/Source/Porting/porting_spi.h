#ifndef __PORTING_SPI_H__
#define __PORTING_SPI_H__

#include "mcu_definition.h"

/**************************************************************************
 * Definitions
 **************************************************************************/
/**
 * @defgroup mcuSPIDef MCU SPI Porting Definition
 * @{
 * @ingroup mcuPortingDef
 * @details MCU SPI setting definition.
 * @}
 **************************************************************************/

/**
 * @defgroup spiSSControlDef SPI SS Control Definition
 * @{
 * @ingroup mcuSPIDef
 */
#define AUTO_SPI_SS                 0               /**< Auto SS   */
#define MANUAL_SPI_SS               1               /**< Manual SS */

#define SPI_SS_CONTROL              MANUAL_SPI_SS   /**< Selected a SPI SS setting from @ref spiSSControlDef "SPI SS control definition list" */
/** @} */


/**
 * @defgroup mcuSPIConfig SPI Configurations
 * @{
 * @ingroup mcuSPIDef
*/
#define SPI_MASTER_TX_DMA_CH        3                                                             /**< The selected TX DMA channel.  */
#define SPI_MASTER_RX_DMA_CH        4                                                             /**< The selected RX DMA channel.  */
#define SPI_OPENED_CH               ((1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH))   /**< The SPI channel enable bits.  */
#define SPI_CLK_FREQ                16000000                                                      /**< The expected frequency of SPI bus clock in Hz. */
#define SPI0_ClearRxFIFO()          (SPI0->FIFOCTL |= SPI_FIFOCTL_RXFBCLR_Msk)                    /**< SPI clear RX FIFO function definition. */
/** @} */

/**************************************************************************
* Functions
**************************************************************************/
/** @defgroup mcuSPIFunc MCU SPI Function Definition
 * @{
 * @details Here shows the MCU SPI related functions.
 * @ingroup mcuSPIDef
 * @}
 **************************************************************************/

/** This function is used to initial MCU SPI.
 *
 * @ingroup mcuSPIFunc
 *
 * @return none
 */
void setBLE_SpiInit(void);


/** This function is used to initial MCU SPI PDMA.
 *
 * @ingroup mcuSPIFunc
 *
 * @return none
 */
void setBLE_SpiPDMAInit(void);


/** This function is used to manual set SPI SS pin to high state immediately.
 *
 * @ingroup mcuSPIFunc
 *
 * @return none
 */
void setBLE_SpiSSHigh(void);


/** This function is used to manual set SPI SS pin to low state immediately.
 *
 * @ingroup mcuSPIFunc
 *
 * @return none
 */
void setBLE_SpiSSLow(void);


/** This function is used to manual set SPI SS pin to high state until SPI is not in busy state.
 *
 * @ingroup mcuSPIFunc
 *
 * @note If @ref SPI_SS_CONTROL is set to @ref AUTO_SPI_SS then user do not have to wait and manual set SS pin to hight state.
 *
 * @return none
 */
void setBLE_SpiWaitAndSSHigh(void);


/** This function is used to read RF 1 byte register.
 *
 * @ingroup mcuSPIFunc
 *
 * @param[in] regAddr : register address
 * @return 1 byte RX data.
 */
uint8_t setBLE_SpiOneByteRx(uint8_t regAddr);


/** This function is used to read RF 1 byte register or RX_FIFO in ISR.
 *
 * @ingroup mcuSPIFunc
 *
 * @param[in] regAddr : register address
 * @return 1 byte RX data.
 */
uint8_t setBLE_SpiOneByteRxIsr(uint8_t regAddr);


/** This function is used to write RF 1 byte register.
 *
 * @ingroup mcuSPIFunc
 *
 * @note Not use for writing TX buffer.
 *
 * @param[in] regAddr   : register address
 * @param[in] u8SrcData : data which user attempt to transfer through SPI bus
 * @return none
 */
void setBLE_SpiOneByteTx(uint8_t regAddr, uint8_t u8SrcData);


/** This function is used to write RF 1 byte register in ISR.
 *
 * @ingroup mcuSPIFunc
 *
 * @note Not use for writing TX buffer.
 *
 * @param[in] regAddr   : register address
 * @param[in] u8SrcData : data which user attempt to transfer through SPI bus
 * @return none
 */
void setBLE_SpiOneByteTxIsr(uint8_t regAddr, uint8_t u8SrcData);


/** This function is used to write RF 2 byte register in ISR.
 *
 * @ingroup mcuSPIFunc
 *
 * @param[in] regAddr   : register address
 * @param[in] u8SrcData : data which user attempt to transfer through SPI bus
 * @return none
 */
void setBLE_SpiTwoByteTxIsr(uint8_t regAddr, uint8_t *u8SrcAddr);


/** This function is used to using SPI PDMA read RF in ISR.
 *
 * @ingroup mcuSPIFunc
 *
 * @attention Because of the set-and-forget SPI DMA behavior, user should avoid modification of the source data.
 *
 * @param[in] regAddr   : register address
 * @param[in] u8SrcData : data which user attempt to transfer through SPI bus
 * @param[in] u32TransCount : transfer count
 * @return none
 */
void setBLE_SpiPDMARxIsr(uint8_t regAddr, uint32_t u32DstAddr, uint32_t u32TransCount);


/** This function is used to using SPI PDMA write RF in ISR.
 *
 * @ingroup mcuSPIFunc
 *
 * @attention Because of the set-and-forget SPI DMA behavior, user should avoid modification of the source data.
 *
 * @param[in] regAddr   : register address
 * @param[in] u8SrcData : data which user attempt to transfer through SPI bus
 * @param[in] u32TransCount : transfer count
 * @return none
 */
void setBLE_SpiPDMATx(uint8_t regAddr, uint32_t u32SrcAddr, uint32_t u32TransCount);


/** This function is used to wait PDMA complete the operation.
 *
 * @ingroup mcuSPIFunc
 *
 * @return  SPI PDMA Result success or not.
 * @retval  0:  Success
 * @retval  1:  FAIL
 */
uint32_t setBLE_SpiPDMAWaitFinish(void);

#endif  //__PORTING_SPI_H__


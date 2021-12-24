#ifndef __PORTING_DTM_H__
#define __PORTING_DTM_H__

#include "mcu_definition.h"

/**************************************************************************
 * Definitions
 **************************************************************************/
/**
 * @defgroup portingDtm BLE DTM Porting
 * @{
 * @details BLE DTM porting.
 * @ingroup ble_dtm
 **************************************************************************/

/** DTM UART Baud Rate definition.
 * @ingroup portingDtm
*/
#define UART_BAURRATE                  115200


/**
 * @ingroup portingDtm
 * @details The UART poll cycle in micro seconds. \n
 *          A baud rate of e.g. 19200 bits / second, and 8 data bits, 1 start/stop bit, no flow control, \n
 *          give the time to transmit a byte: 10 bits * 1/19200 = approx: 520 us. \n
 *          To ensure no loss of bytes, the UART should be polled every 260 us.
 */
#define UART_POLL_CYCLE                ((uint32_t)(10*1e6/(UART_BAURRATE)/2))


/** DTM Timer Interrupt Enable Function.
 * @ingroup portingDtm
*/
void setDTM_timerInt_enable(void);



/** DTM Timer Initialization Function.
 * @ingroup portingDtm
*/
void setDTM_timer_init(void);


#endif //__PORTING_DTM_H__


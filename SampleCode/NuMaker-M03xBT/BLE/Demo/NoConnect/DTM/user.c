/**************************************************************************//**
 * @file     user.c
 * @version  V0.9
 * $Revision: 01 $
 * @brief
 *           Demonstrate how to use LIB pre-build function to start and stop a BLE
 *           connection.
 * @note
 *
 ******************************************************************************/
#include <stdio.h>
#include "BleAppSetting.h"
#include "rf_phy.h"
#include "ble_dtm.h"
#include "porting_spi.h"
#include "porting_misc.h"

/**************************************************************************
* Definitions
**************************************************************************/
/**
 * The maximum time between the first and second byte of the command or event (end of stop bit to start of start bit) is 5ms.
 * As the time is only known when a byte is received, then the time between stop bit 1st byte and stop bit 2nd byte becomes:
 *      5000us + transmission time of 2nd byte.
 *
 * Byte transmission time is (Baud rate of 19200):
 *      10bits * 1/19200 = approx. 520 us/byte (8 data bits + start & stop bit).
 *
 * Loop time on polling UART register for received byte is defined in ble_dtm.c as:
 *   UART_POLL_CYCLE = 260 us
 *
 * The max time between two bytes thus becomes (loop time: 260us / iteration):
 *      (5000us + 520us) / 260us / iteration = 21.2 iterations.
 *
 * This is rounded down to 21.
 *
 * If UART bit rate is changed, this value should be recalculated as well.
 */
#define MAX_ITERATIONS_NEEDED_FOR_NEXT_BYTE ((5000 + 2 * UART_POLL_CYCLE) / UART_POLL_CYCLE)


/**************************************************************************
* Variables
**************************************************************************/
uint32_t    msb_time          = 0;     // Time when MSB of the DTM command was read. Used to catch stray bytes from "misbehaving" testers.
uint8_t     is_msb_read       = 0;     // True when MSB of the DTM command has been read and the application is waiting for LSB.
uint16_t    dtm_cmd_from_uart = 0;     // Packed command containing command_code:frequency:length:payload in 2:6:6:2 bits.



/**************************************************************************
* Functions
**************************************************************************/

/* Combine DTM command through 2-wire UART */
uint8_t BleDTM_2wire_cmd_combine(uint8_t rx_byte, uint32_t current_time)
{
    if (is_msb_read == 0)
    {
        // This is first byte of two-byte command.
        is_msb_read       = 1;
        dtm_cmd_from_uart = ((uint32_t)rx_byte) << 8;
        msb_time          = current_time;
        return false; // Go back and wait for 2nd byte of command word.
    }

    // This is the second byte read; combine it with the first and process command
    if (current_time > (msb_time + MAX_ITERATIONS_NEEDED_FOR_NEXT_BYTE))
    {
        // More than ~5mS after msb: Drop old byte, take the new byte as MSB.
        // The variable is_msb_read will remains true.
        dtm_cmd_from_uart = ((uint32_t)rx_byte) << 8;
        msb_time          = current_time;
        return false; // Go back and wait for 2nd byte of the command word.
    }

    // 2-byte UART command received.
    is_msb_read        = 0;
    dtm_cmd_from_uart |= (uint32_t)rx_byte;

    return true;
}


/* UART get API provided by platform */
uint8_t BleDTM_2wire_uart_get(uint8_t *rx_byte)
{
    // UART status check
    if ((UART0->INTSTS & UART_INTSTS_RDAIF_Msk) == 0)
    {
        return false;
    }

    // Clear UART status
    UART0->INTSTS = UART0->INTSTS & (~UART_INTSTS_RDAIF_Msk);

    // get 1 byte data from UART
    *rx_byte = UART_READ(UART0);

    return true;
}


/* UART send API provided by platform */
void BleDTM_2wire_uart_send(uint16_t dtm_event)
{
    extern uint8_t rssi_read_data[3];
    // Send first byte (MSB) through UART
    UART_WRITE(UART0, (dtm_event >> 8) & 0xFF);

    // Wait until first byte (MSB) is sent.
    while (((UART0->INTSTS & UART_INTSTS_THREIF_Msk) >> UART_INTSTS_THREIF_Pos) != 1);
    UART0->INTSTS = UART0->INTSTS & (~UART_INTSTS_THREIF_Msk);

    // Transmit second byte (LSB) of the result.
    UART_WRITE(UART0, dtm_event & 0xFF);

    // Wait until second byte (LSB) is sent.
    while (((UART0->INTSTS & UART_INTSTS_THREIF_Msk) >> UART_INTSTS_THREIF_Pos) != 1);

    // Clear UART TX status.
    UART0->INTSTS = UART0->INTSTS & (~UART_INTSTS_THREIF_Msk);


#if (ENABLE_LAB_TEST_TOOL_SUPPORTS == ENABLE_DEF)
    // command for proprietary tool (Lab Test Tool).
    if (BleDTM_gui_rssiFlag_get() == 1)
    {
        setBLE_SpiPDMARxIsr(166, (uint32_t)&rssi_read_data[0], 3);
        printf("RSSI:%d\n", BleDTM_rssi_get()); //in dBm
    }
#endif
}



void BleDTM_2wire_cmd_handler(uint32_t current_time)
{
    uint8_t     rx_byte = 0xFF;   // Last byte read from UART.
    uint32_t    result;           // Result of a DTM operation.

    // 16 bit DTM command from equipment: command_code:frequency:length:payload in 2:6:6:2 bits
    if (BleDTM_2wire_uart_get(&rx_byte) == false)
    {
        return;
    }

    // combine to 16bit dtm command
    if (BleDTM_2wire_cmd_combine(rx_byte, current_time) == false)
    {
        return;
    }

    if (BleDTM_cmd_process(dtm_cmd_from_uart) != BLEDTM_SUCCESS)
    {
        // Extended error handling may be put here.
        // Default behavior is to return the event on the UART (see below);
        // the event report will reflect any lack of success.
    }

    // Retrieve result of the operation. This implementation will busy-loop
    // for the duration of the byte transmissions on the UART.
    if (BleDTM_event_get(&result)) //"true" after running bledtm_cmd()
    {
        // Report command status on the UART. (Send EVENT out)
        BleDTM_2wire_uart_send(result);
    }
}



void BleDTM_Main(void)
{
    uint32_t  current_time;

    // handle radio first to give it the highest priority.
    BleDTM_radio_process();

    // return every 260 us.
    current_time = BleDTM_timerCount_update();

    // DTM command handling, DTM command is received from equipment through 2-wire UART.
    BleDTM_2wire_cmd_handler(current_time);
}


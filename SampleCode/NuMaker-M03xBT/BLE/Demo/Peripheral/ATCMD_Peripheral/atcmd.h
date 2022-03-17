#ifndef _AT_CMD_H_
#define _AT_CMD_H_

#include "NuMicro.h"


#define NUVOTON_DEBUG   0

#if NUVOTON_DEBUG
#define debug_printf            printf
#else
#define debug_printf(...)
#endif

#define AT_VERSION              "1.1.2"
#define AT_DATE                 "20220314"

typedef uint8_t AtCmdStatus;
#define ATCMD_SUCCESS           0x00
#define ATCMD_FAIL              0x01
#define ATCMD_BUSY              0x02

typedef uint8_t AtCmdMode;
#define AT_STATE_FREE           0x00
#define AT_STATE_WAIT           0x01
#define AT_STATE_DONE           0x02


/* BLE module default settings */
extern uint8_t ble_scan_name_def[24];
extern uint32_t ble_uart_def;
extern int8_t ble_tx_power_def[3];

/* BLE module current settings */
extern uint8_t ble_scan_name[24];
extern uint32_t ble_uart;
extern int8_t ble_tx_power[3];

/* AT command variables */
extern AtCmdMode atcmd_state;

/* AT command functions */
AtCmdStatus atcmd_handler(char *data);

/* Data flash functions */
int atcmd_set_data_flash(void);
void atcmd_initial_data_flash(void);
void atcmd_update_data_flash(void);
int atcmd_load_data_flash(void);


#endif

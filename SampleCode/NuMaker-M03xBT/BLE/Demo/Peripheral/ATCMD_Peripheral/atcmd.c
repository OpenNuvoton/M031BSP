/**************************************************************************//**
 * @file     atcmd.c
 * @version  V0.1
 * $Revision: 01 $
 * @brief
 *           Support AT command to control BLE
 * @note
 *
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "atcmd.h"
#include "ble_cmd.h"
#include "ble_profile.h"
#include "rf_phy.h"
#include "porting_rfpower.h"

extern BleStackStatus setBLE_AdvInterval(uint16_t interval_min, uint16_t interval_max);
extern BleStackStatus setBLE_ScanName(char *name);
extern uint8_t ble_phy;
extern uint8_t echo_enabled;
extern uint8_t sleep_enabled;
extern uint8_t dsleep_enabled;
extern uint16_t ble_adv_interval;
extern BLE_Event_ConnUpdateParam ble_conn_update;

AtCmdMode atcmd_state = AT_STATE_FREE;

/* BLE module default settings */
/* First 3 bytes of ble_scan_name[] are total length(N+2), AD Length(N+1) and AD data(GAP_AD_TYPE_LOCAL_NAME_COMPLETE) */
uint8_t ble_scan_name_def[24] = {15, 14, GAP_AD_TYPE_LOCAL_NAME_COMPLETE, 'N', 'u', 'v', 'o', 't', 'o', 'n', '_', 'A', 'T', 'C', 'M', 'D'};
uint32_t ble_uart_def = 115200;                                                 // baudrate is 115200
int8_t ble_tx_power_def[3] = {TXPOWER_9DBM, TXPOWER_9DBM, TXPOWER_9DBM};        // all modes are set to 9dBm

/* BLE module current settings */
uint8_t ble_scan_name[24];
uint32_t ble_uart;
int8_t ble_tx_power[3];

#define WRONG_TX_POWER          99                                              // notify the wrong tx power setting


/* Get firmware version */
AtCmdStatus atcmd_get_version(void)
{
    printf("+VERSION:%s-%s\r\n", AT_VERSION, AT_DATE);

    return ATCMD_SUCCESS;
}


/* Set ECHO mode */
AtCmdStatus atcmd_set_echo(int mode)
{
    AtCmdStatus ret = ATCMD_FAIL;

    switch (mode)
    {
    case 0:
    case 1:
    {
        atcmd_state = AT_STATE_FREE;

        printf("OK\r\n");
        echo_enabled = mode;

        ret = ATCMD_SUCCESS;
    }
    break;

    default:
        break;
    }

    return ret;
}


/* Get ECHO mode */
AtCmdStatus atcmd_get_echo(void)
{
    printf("+ECHO:%d\r\n", echo_enabled);

    return ATCMD_SUCCESS;
}


/* Reset BLE module */
AtCmdStatus atcmd_reset(void)
{
    atcmd_state = AT_STATE_FREE;

    printf("OK\r\n");
    printf("Reboot BLE module...\r\n\n");
    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(UART0);

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Perform chip reset to make new User Config take effect */
    SYS_ResetChip();

    return ATCMD_SUCCESS;
}


/* Factory reset BLE module */
AtCmdStatus atcmd_factory_reset(void)
{
    AtCmdStatus ret = ATCMD_FAIL;

    atcmd_state = AT_STATE_FREE;

    if (bleProfile_link0_info.bleState == STATE_BLE_STANDBY)
    {
        /* update data flash */
        SYS_UnlockReg();
        FMC_Open();
        atcmd_initial_data_flash();

        printf("OK\r\n");
        printf("Factory reset BLE module...\r\n\n");
        /* Check if all the debug messages are finished */
        UART_WAIT_TX_EMPTY(UART0);

        /* Unlock protected registers */
        SYS_UnlockReg();
        /* Perform chip reset to make new User Config take effect */
        SYS_ResetChip();

        ret = ATCMD_SUCCESS;
    }
    else
    {
        printf("ERROR\r\n");
    }

    return ret;
}


/* Get BLE state */
AtCmdStatus atcmd_get_state(void)
{
    printf("+STATE:%d\r\n", bleProfile_link0_info.bleState);

    return ATCMD_SUCCESS;
}


/* Set UART baud rate */
AtCmdStatus atcmd_set_uart(int baudrate)
{
    AtCmdStatus ret = ATCMD_FAIL;

    switch (baudrate)
    {
    case 9600:
    case 19200:
    case 38400:
    case 57600:
    case 115200:
    {
        atcmd_state = AT_STATE_FREE;

        printf("OK\r\n");

        if (baudrate != ble_uart)
        {
            ble_uart = baudrate;

            /* update data flash */
            SYS_UnlockReg();
            FMC_Open();
            atcmd_update_data_flash();
            FMC_Close();
            SYS_LockReg();

            /* Check if all the debug messages are finished */
            UART_WAIT_TX_EMPTY(UART0);

            /* Set the UART baudrate */
            UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, ble_uart);

            ret = ATCMD_SUCCESS;
        }
    }
    break;

    default:
        break;
    }

    return ret;
}


/* Get UART baud rate */
AtCmdStatus atcmd_get_uart(void)
{
    printf("+UART:%d\r\n", ble_uart);

    return ATCMD_SUCCESS;
}


/* Set scan response data */
AtCmdStatus atcmd_set_name(char *name)
{
    AtCmdStatus ret = ATCMD_FAIL;
    BleStackStatus status;

    if (bleProfile_link0_info.bleState < STATE_BLE_CONNECTION)
    {
        status = setBLE_ScanName(name);

        if (status == BLESTACK_STATUS_SUCCESS)
        {
            /* update data flash */
            SYS_UnlockReg();
            FMC_Open();
            atcmd_update_data_flash();
            FMC_Close();
            SYS_LockReg();

            ret = ATCMD_SUCCESS;
        }
    }

    return ret;
}


/* Get scan response data */
AtCmdStatus atcmd_get_name(void)
{
    printf("+NAME:%s\r\n", &ble_scan_name[3]);

    return ATCMD_SUCCESS;
}


/* Set advertising enable */
AtCmdStatus atcmd_set_adven(void)
{
    AtCmdStatus ret = ATCMD_FAIL;
    BleStackStatus status;

    status = setBLE_AdvEnable(bleProfile_link0_info.hostId);
    debug_printf("setBLE_AdvEnable(%d) = %d\n", bleProfile_link0_info.hostId, status);
    if (status == BLESTACK_STATUS_SUCCESS)
    {
        bleProfile_link0_info.bleState = STATE_BLE_ADVERTISING;
        ret = ATCMD_SUCCESS;
        /* Wait for BLE event callback */
        atcmd_state = AT_STATE_WAIT;
    }

    return ret;
}


/* Set advertising disable */
AtCmdStatus atcmd_set_advdis(void)
{
    AtCmdStatus ret = ATCMD_FAIL;
    BleStackStatus status;

    status = setBLE_AdvDisable();
    debug_printf("setBLE_AdvDisable() = %d\n", status);
    if (status == BLESTACK_STATUS_SUCCESS)
    {
        bleProfile_link0_info.bleState = STATE_BLE_STANDBY;
        ret = ATCMD_SUCCESS;
        atcmd_state = AT_STATE_WAIT;
    }

    return ret;
}


/* Set advertising interval */
AtCmdStatus atcmd_set_advint(int interval)
{
    AtCmdStatus ret = ATCMD_FAIL;
    BleStackStatus status;

    /* interval = value * 0.625ms, and the range is 20ms ~ 10.24s */
    if ((interval < 32) || (interval > 16384))
    {
    }
    /* Set connection interval in connection mode only */
    else if (bleProfile_link0_info.bleState == STATE_BLE_STANDBY)
    {
        if (interval == ble_adv_interval)
            ret = ATCMD_SUCCESS;
        else
        {
            status = setBLE_AdvInterval(interval, interval);
            debug_printf("setBLE_AdvInterval() = %d\n", status);
            if (status == BLESTACK_STATUS_SUCCESS)
                ret = ATCMD_SUCCESS;
        }
    }

    return ret;
}


/* Get advertising interval */
AtCmdStatus atcmd_get_advint(void)
{
    AtCmdStatus ret = ATCMD_FAIL;

    /* Get advertising interval in standby or advertising mode only */
    if (bleProfile_link0_info.bleState == STATE_BLE_STANDBY)
    {
        printf("+ADVINT:%d\r\n", ble_adv_interval);
        ret = ATCMD_SUCCESS;
    }

    return ret;
}


/* Set connection interval */
AtCmdStatus atcmd_set_conint(int interval)
{
    AtCmdStatus ret = ATCMD_FAIL;
    BleStackStatus status;
    BLE_Conn_Param connParam;
    uint16_t ref_time;

    /* interval = value * 1.25ms, and the range is 7.5ms ~ 4s */
    if ((interval < 6) || (interval > 3200))
    {
    }
    /* Set connection interval in connection mode only */
    else if (bleProfile_link0_info.bleState == STATE_BLE_CONNECTION)
    {
        if (interval == ble_conn_update.connInterval)
            ret = ATCMD_SUCCESS;
        else
        {
            connParam.connIntervalMin = interval;
            connParam.connIntervalMax = interval;
            connParam.connLatency = ble_conn_update.connLatency;
            connParam.connSupervisionTimeout = ble_conn_update.connSupervisionTimeout;

            /* Set appropriate value of connSupervisionTimeout, default is ((1+Latency)*IntervalMax)*4) */
            ref_time = (((1 + connParam.connLatency) * connParam.connIntervalMax + 7) / 8) * 4;
            if (ref_time > 3200)
                connParam.connSupervisionTimeout = 3200;
            else if (ref_time < 500)
                connParam.connSupervisionTimeout = 500;
            else
                connParam.connSupervisionTimeout = ref_time;

            debug_printf("connIntervalMin = %d\n", connParam.connIntervalMin);
            debug_printf("connIntervalMin = %d\n", connParam.connIntervalMax);
            debug_printf("connLatency = %d\n", connParam.connLatency);
            debug_printf("connSupervisionTimeout = %d\n", connParam.connSupervisionTimeout);
            status = setBLE_ConnUpdate(ble_conn_update.hostId, &connParam);
            debug_printf("setBLE_ConnUpdate() = %d\n", status);
            if (status == BLESTACK_STATUS_SUCCESS)
            {
                ret = ATCMD_SUCCESS;
                atcmd_state = AT_STATE_WAIT;
            }
        }
    }

    return ret;
}


/* Get connection interval */
AtCmdStatus atcmd_get_conint(void)
{
    AtCmdStatus ret = ATCMD_FAIL;

    /* Get connection interval in connection mode only */
    if (bleProfile_link0_info.bleState == STATE_BLE_CONNECTION)
    {
        printf("+CONINT:%d\r\n", ble_conn_update.connInterval);
        ret = ATCMD_SUCCESS;
    }

    return ret;
}


/* Disconnect the connection */
AtCmdStatus atcmd_set_discon(void)
{
    AtCmdStatus ret = ATCMD_FAIL;
    BleStackStatus status;

    /* Disconnect in connection mode only */
    if (bleProfile_link0_info.bleState == STATE_BLE_CONNECTION)
    {
        status = setBLE_Disconnect(ble_conn_update.hostId);
        debug_printf("setBLE_Disconnect() = %d\n", status);
        if (status == BLESTACK_STATUS_SUCCESS)
            ret = ATCMD_SUCCESS;
    }

    return ret;
}


/* Get BLE device address */
AtCmdStatus atcmd_get_address(void)
{
    BLE_Addr_Param bleAddrParam;
    uint8_t i;

    getBLE_BleDeviceAddr(&bleAddrParam);
    printf("+ADDR:%02X", bleAddrParam.addr[5]);
    for (i = 5; i > 0; i--)
        printf("-%02X", bleAddrParam.addr[i-1]);
    printf("\r\n");

    return ATCMD_SUCCESS;
}


/* Transfer the TX power level to index */
uint8_t power_level_to_index(int8_t level)
{
    uint8_t power_index;

    if (GET_CHIP_SERIES_NUM == CHIP_SERIES_NUM_G)
    {
        /* For 256 KB chipset, the maximum power level is 8 dBm */
        if (level > 8)
        {
            printf("Exceed! Set the TX power level to be maximum 8 dBm.\n");
            level = 8;
        }
    }
    else if (GET_CHIP_SERIES_NUM == CHIP_SERIES_NUM_I)
    {
        /* For 512 KB chipset, the maximum power level is 7 dBm */
        if (level > 7)
        {
            printf("Exceed! Set the TX power level to be maximum 7 dBm.\n");
            level = 7;
        }
    }

    switch (level)
    {
    case 9:
        power_index = TXPOWER_9DBM;
        break;

    case 8:
        power_index = TXPOWER_8DBM;
        break;

    case 7:
        power_index = TXPOWER_7DBM;
        break;

    case 6:
        power_index = TXPOWER_6DBM;
        break;

    case 4:
        power_index = TXPOWER_4DBM;
        break;

    case 2:
        power_index = TXPOWER_2DBM;
        break;

    case 0:
        power_index = TXPOWER_0DBM;
        break;

    case -3:
        power_index = TXPOWER_m3DBM;
        break;

    case -6:
        power_index = TXPOWER_m6DBM;
        break;

    case -10:
        power_index = TXPOWER_m10DBM;
        break;

    case -15:
        power_index = TXPOWER_m15DBM;
        break;

    case -20:
        power_index = TXPOWER_m20DBM;
        break;

    default:
        power_index = WRONG_TX_POWER;
    }

    return power_index;
}


/* Transfer the TX power index to level */
int8_t power_index_to_level(uint8_t power_index)
{
    int8_t level;

    switch (power_index)
    {
    case TXPOWER_9DBM:
        level = 9;
        break;

    case TXPOWER_8DBM:
        level = 8;
        break;

    case TXPOWER_7DBM:
        level = 7;
        break;

    case TXPOWER_6DBM:
        level = 6;
        break;

    case TXPOWER_4DBM:
        level = 4;
        break;

    case TXPOWER_2DBM:
        level = 2;
        break;

    case TXPOWER_0DBM:
        level = 0;
        break;

    case TXPOWER_m3DBM:
        level = -3;
        break;

    case TXPOWER_m6DBM:
        level = -6;
        break;

    case TXPOWER_m10DBM:
        level = -10;
        break;

    case TXPOWER_m15DBM:
        level = -15;
        break;

    case TXPOWER_m20DBM:
        level = -20;
        break;

    default:
        level = WRONG_TX_POWER;
    }

    return level;
}


/* Set TX power */
AtCmdStatus atcmd_set_txpower(uint8_t mode, int8_t level)
{
    AtCmdStatus ret = ATCMD_FAIL;
    BleStackStatus status;
    BleMode ble_mode;
    uint8_t power_index;

    /* Set TX power in standby mode only */
    if ((bleProfile_link0_info.bleState == STATE_BLE_STANDBY) && (mode > 0) && (mode <= 3))
    {
        /* Transfer the BLE mode to index */
        if (mode == 1)
            ble_mode = STATE_BLE_ADVERTISING;
        else if (mode == 2)
            ble_mode = STATE_BLE_SCANNING;
        else
            ble_mode = STATE_BLE_INITIATING;

        /* Get power index from level */
        power_index = power_level_to_index(level);

        if (power_index != WRONG_TX_POWER)
        {
            /* New setting is different to current */
            if (power_index != ble_tx_power[mode-1])
            {
                status = setBLE_TxPower_Wrap(power_index, ble_mode);
                debug_printf("setBLE_TxPower() = %d\n", status);

                if (status == BLESTACK_STATUS_SUCCESS)
                {
                    ble_tx_power[mode-1] = power_index;

                    /* Update data flash */
                    SYS_UnlockReg();
                    FMC_Open();

                    atcmd_update_data_flash();

                    FMC_Close();
                    SYS_LockReg();
                    ret = ATCMD_SUCCESS;
                }
            }
            else
            {
                ret = ATCMD_SUCCESS;
            }
        }
    }

    return ret;
}


/* Get TX power */
AtCmdStatus atcmd_get_txpower(uint8_t mode)
{
    AtCmdStatus ret = ATCMD_FAIL;

    /* Get TX power in standby mode only */
    if ((bleProfile_link0_info.bleState == STATE_BLE_STANDBY) && (mode > 0) && (mode <= 3))
    {
        printf("+TXPOWER:%d,%d\r\n", mode, power_index_to_level(ble_tx_power[mode-1]));
        ret = ATCMD_SUCCESS;
    }

    return ret;
}


/* Set Phy mode */
AtCmdStatus atcmd_set_phy(int phy)
{
    AtCmdStatus ret = ATCMD_FAIL;
    BleStackStatus status;
    BLE_Phy_Param blePhy;

    /* Set Phy in connection mode only */
    if (bleProfile_link0_info.bleState == STATE_BLE_CONNECTION)
    {
        if (phy == ble_phy)
            ret = ATCMD_SUCCESS;
        else
        {
            //set phy
            blePhy.txPhy = phy;
            blePhy.rxPhy = phy;
            status = setBLE_Phy(ble_conn_update.hostId, &blePhy);
            debug_printf("setBLE_Phy() = %d\n", status);
            if (status == BLESTACK_STATUS_SUCCESS)
            {
                ret = ATCMD_SUCCESS;
                atcmd_state = AT_STATE_WAIT;
            }
        }
    }

    return ret;
}


/* Get Phy mode */
AtCmdStatus atcmd_get_phy(void)
{
    AtCmdStatus ret = ATCMD_FAIL;
    BleStackStatus status;

    /* Get Phy in connection mode only */
    if (bleProfile_link0_info.bleState == STATE_BLE_CONNECTION)
    {
        status = getBLE_Phy(ble_conn_update.hostId);
        debug_printf("getBLE_Phy() = %d\n", status);
        if (status == BLESTACK_STATUS_SUCCESS)
        {
            ret = ATCMD_SUCCESS;
            atcmd_state = AT_STATE_WAIT;
        }
    }

    return ret;
}


/* Set BLE to deep sleep */
AtCmdStatus atcmd_set_dsleep(void)
{
    AtCmdStatus ret = ATCMD_FAIL;
    BleStackStatus status;

    /* Set deep-sleep in standby mode only */
    if (bleProfile_link0_info.bleState == STATE_BLE_STANDBY)
    {
        /* RF enters to deep-sleep */
        status = setRF_EnterDeepSleep();
        debug_printf("setRF_EnterDeepSleep() = %d\n", status);
        if (status == BLESTACK_STATUS_SUCCESS)
        {
            dsleep_enabled = 1;
            ret = ATCMD_SUCCESS;
        }
    }

    return ret;
}


/* Set value in memory address */
AtCmdStatus atcmd_set_memory(uint32_t addr, uint32_t value)
{
    *(uint32_t*)addr = value;

    return ATCMD_SUCCESS;
}


/* Get value in memory address */
AtCmdStatus atcmd_get_memory(uint32_t addr)
{
    printf("+MEMORY:0x%08X,0x%08X\r\n", addr, *((uint32_t *)addr));

    return ATCMD_SUCCESS;
}


/* List the supported AT commands */
AtCmdStatus atcmd_help(void)
{
    printf("+HELP:\r\n");
    printf("AT+ADDR?\n");
    printf("AT+ADVEN\n");
    printf("AT+ADVDIS\n");
    printf("AT+ADVINT=\n");
    printf("AT+ADVINT?\n");
    printf("AT+CONINT=\n");
    printf("AT+CONINT?\n");
    printf("AT+DISCON\n");
    printf("AT+DSLEEP\n");
    printf("AT+MEMORY=\n");
    printf("AT+MEMORY?\n");
    printf("AT+NAME=\n");
    printf("AT+NAME?\n");
    printf("AT+NOSLEEP\n");
    printf("AT+ORGL\n");
    printf("AT+PHY=\n");
    printf("AT+PHY?\n");
    printf("AT+RESET\n");
    printf("AT+SLEEP\n");
    printf("AT+STATE?\n");
    printf("AT+TXPWR=\n");
    printf("AT+TXPWR?\n");
    printf("AT+UART=\n");
    printf("AT+UART?\n");
    printf("AT+VERSION?\n");

    return ATCMD_SUCCESS;
}


AtCmdStatus atcmd_handler(char *data)
{
    AtCmdStatus ret = ATCMD_FAIL;
    char *pcDelim = "+=?,\r\n";
    char *pcPtr;
    char pcCopy[48];
    char cDelim;
    int iNum1, iNum2;
    uint32_t u32Addr, u32Val;

    pcPtr = strstr(data, "AT+");

    atcmd_state = AT_STATE_DONE;

    if (pcPtr == data)
    {
        strcpy(pcCopy, data);

        pcPtr = strtok(data, pcDelim);
        pcPtr = strtok(NULL, pcDelim);
        cDelim = pcCopy[pcPtr - data + strlen(pcPtr)];

        if (strcmp(pcPtr, "VERSION") == 0)
        {
            pcPtr = strtok(NULL, pcDelim);
            /* Ensure there is no following parameter*/
            if ((cDelim == '?') && (pcPtr == NULL))
                ret = atcmd_get_version();
        }
        else if (strcmp(pcPtr, "ECHO") == 0)
        {
            pcPtr = strtok(NULL, pcDelim);
            if (cDelim == '=')
            {
                iNum1 = atoi(pcPtr);
                cDelim = pcCopy[pcPtr - data + strlen(pcPtr)];
                if ((cDelim == '\r') || (cDelim == '\n'))
                    ret = atcmd_set_echo(iNum1);
            }
            else if ((cDelim == '?') && (pcPtr == NULL))
                ret = atcmd_get_echo();
        }
        else if (strcmp(pcPtr, "RESET") == 0)
        {
            /* Ensure there is no following parameter*/
            if ((cDelim == '\r') || (cDelim == '\n'))
                ret = atcmd_reset();
        }
        else if (strcmp(pcPtr, "STATE") == 0)
        {
            pcPtr = strtok(NULL, pcDelim);
            if ((cDelim == '?') && (pcPtr == NULL))
                ret = atcmd_get_state();
        }
        else if (strcmp(pcPtr, "ORGL") == 0)
        {
            if ((cDelim == '\r') || (cDelim == '\n'))
                ret = atcmd_factory_reset();
        }
        else if (strcmp(pcPtr, "UART") == 0)
        {
            pcPtr = strtok(NULL, pcDelim);
            if (cDelim == '=')
            {
                iNum1 = atoi(pcPtr);
                cDelim = pcCopy[pcPtr - data + strlen(pcPtr)];
                if ((cDelim == '\r') || (cDelim == '\n'))
                    ret = atcmd_set_uart(iNum1);
            }
            else if ((cDelim == '?') && (pcPtr == NULL))
                ret = atcmd_get_uart();
        }
        else if (strcmp(pcPtr, "NAME") == 0)
        {
            pcPtr = strtok(NULL, pcDelim);
            if (cDelim == '=')
            {
                cDelim = pcCopy[pcPtr - data + strlen(pcPtr)];
                if ((cDelim == '\r') || (cDelim == '\n'))
                    ret = atcmd_set_name(pcPtr);
            }
            else if ((cDelim == '?') && (pcPtr == NULL))
                ret = atcmd_get_name();
        }
        else if (strcmp(pcPtr, "ADVEN") == 0)
        {
            if ((cDelim == '\r') || (cDelim == '\n'))
                ret = atcmd_set_adven();
        }
        else if (strcmp(pcPtr, "ADVDIS") == 0)
        {
            if ((cDelim == '\r') || (cDelim == '\n'))
                ret = atcmd_set_advdis();
        }
        else if (strcmp(pcPtr, "ADVINT") == 0)
        {
            pcPtr = strtok(NULL, pcDelim);
            if (cDelim == '=')
            {
                iNum1 = atoi(pcPtr);
                cDelim = pcCopy[pcPtr - data + strlen(pcPtr)];
                if ((cDelim == '\r') || (cDelim == '\n'))
                    ret = atcmd_set_advint(iNum1);
            }
            else if ((cDelim == '?') && (pcPtr == NULL))
                ret = atcmd_get_advint();
        }
        else if (strcmp(pcPtr, "CONINT") == 0)
        {
            pcPtr = strtok(NULL, pcDelim);
            if (cDelim == '=')
            {
                iNum1 = atoi(pcPtr);
                cDelim = pcCopy[pcPtr - data + strlen(pcPtr)];
                if ((cDelim == '\r') || (cDelim == '\n'))
                    ret = atcmd_set_conint(iNum1);
            }
            else if ((cDelim == '?') && (pcPtr == NULL))
                ret = atcmd_get_conint();
        }
        else if (strcmp(pcPtr, "DISCON") == 0)
        {
            if ((cDelim == '\r') || (cDelim == '\n'))
                ret = atcmd_set_discon();
        }
        else if (strcmp(pcPtr, "ADDR") == 0)
        {
            pcPtr = strtok(NULL, pcDelim);
            if ((cDelim == '?') && (pcPtr == NULL))
                ret = atcmd_get_address();
        }
        else if (strcmp(pcPtr, "TXPWR") == 0)
        {
            pcPtr = strtok(NULL, pcDelim);
            iNum1 = atoi(pcPtr);
            if (cDelim == '=')
            {
                /* AT+TXPOWER=mode,level */
                cDelim = pcCopy[pcPtr - data + strlen(pcPtr)];
                pcPtr = strtok(NULL, pcDelim);
                if (cDelim == ',')
                {
                    iNum2 = atoi(pcPtr);
                    cDelim = pcCopy[pcPtr - data + strlen(pcPtr)];
                    if ((cDelim == '\r') || (cDelim == '\n'))
                        ret = atcmd_set_txpower(iNum1, iNum2);
                }
            }
            else if (cDelim == '?')
            {
                /* AT+TXPOWER?mode */
                cDelim = pcCopy[pcPtr - data + strlen(pcPtr)];
                pcPtr = strtok(NULL, pcDelim);
                if (((cDelim == '\r') || (cDelim == '\n')) && (pcPtr == NULL))
                    ret = atcmd_get_txpower(iNum1);
            }
        }
        else if (strcmp(pcPtr, "PHY") == 0)
        {
            pcPtr = strtok(NULL, pcDelim);
            if (cDelim == '=')
            {
                iNum1 = atoi(pcPtr);
                cDelim = pcCopy[pcPtr - data + strlen(pcPtr)];
                if ((cDelim == '\r') || (cDelim == '\n'))
                    ret = atcmd_set_phy(iNum1);
            }
            else if ((cDelim == '?') && (pcPtr == NULL))
                ret = atcmd_get_phy();
        }
        else if (strcmp(pcPtr, "SLEEP") == 0)
        {
            if ((cDelim == '\r') || (cDelim == '\n'))
            {
                sleep_enabled = 1;
                ret = ATCMD_SUCCESS;
            }
        }
        else if (strcmp(pcPtr, "NOSLEEP") == 0)
        {
            if ((cDelim == '\r') || (cDelim == '\n'))
            {
                sleep_enabled = 0;
                ret = ATCMD_SUCCESS;
            }
        }
        else if (strcmp(pcPtr, "DSLEEP") == 0)
        {
            if ((cDelim == '\r') || (cDelim == '\n'))
                ret = atcmd_set_dsleep();
        }
        else if (strcmp(pcPtr, "MEMORY") == 0)
        {
            pcPtr = strtok(NULL, pcDelim);
            u32Addr = strtol(pcPtr, NULL, 16);
            /* Address must be 4 byte alignment */
            if ((u32Addr % 4) == 0)
            {
                if (cDelim == '=')
                {
                    /* AT+MEMORY=address,value */
                    cDelim = pcCopy[pcPtr - data + strlen(pcPtr)];
                    pcPtr = strtok(NULL, pcDelim);
                    if (cDelim == ',')
                    {
                        u32Val = strtol(pcPtr, NULL, 16);
                        cDelim = pcCopy[pcPtr - data + strlen(pcPtr)];
                        if ((cDelim == '\r') || (cDelim == '\n'))
                            ret = atcmd_set_memory(u32Addr, u32Val);
                    }
                }
                else if (cDelim == '?')
                {
                    /* AT+MEMORY?address */
                    cDelim = pcCopy[pcPtr - data + strlen(pcPtr)];
                    pcPtr = strtok(NULL, pcDelim);
                    if (((cDelim == '\r') || (cDelim == '\n')) && (pcPtr == NULL))
                        ret = atcmd_get_memory(u32Addr);
                }
            }
        }
        else if (strcmp(pcPtr, "HELP") == 0)
        {
            pcPtr = strtok(NULL, pcDelim);
            if ((cDelim == '?') && (pcPtr == NULL))
                ret = atcmd_help();
        }
        else
        {
        }
    }
    else if (strcmp(data, "AT\r") == 0)
    {
        ret = ATCMD_SUCCESS;
    }

    if (atcmd_state == AT_STATE_DONE)
    {
        if (ret == ATCMD_SUCCESS)
            printf("OK\r\n");
        else
            printf("ERROR\r\n");

        atcmd_state = AT_STATE_FREE;
    }

    return ret;
}


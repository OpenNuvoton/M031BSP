/*----------------------------------------------------------------------------*/
/* This file implement RF TX Power Register Settings                          */
/*----------------------------------------------------------------------------*/
#include "stdio.h"
#include "ble_cmd.h"
#include "mcu_definition.h"
#include "porting_rfpower.h"

/** BLE TX power level definition without filter
 * @attention  1. BLE TX power level table can be modified but please noted that the default TX power setting is set to index 0. \n
 *             2. BLE channel 36/39 TX power can be modified by setting different CH36_idx/ CH39_idx index. \n
 *                i.e., default TX power is set to index 0 and reduce BLE channel 36/39 maximum TX power to stage 3.
 *                          default stage 0  : {0xC9,  0x87,  0xBC,  0x1A,  0x20,   0},
 *                      --> modified stage 0 : {0xC9,  0x87,  0xBC,  0x1A,  0x20,   3},
*/

const int8_t TXPOWER_TABLE_MATCHING[][6] =
{
    //R22   R23    R24    R25    R26  CH36_idx/ CH39_idx
    {0xC9,  0x87,  0x8C,  0x30,  0x00,    0},   /**< stage EXT3, default Tx power is set to index 0. */
    {0xC9,  0x87,  0x9C,  0x1A,  0x00,    1},   /**< stage EXT2 */
    {0xC9,  0x87,  0xAC,  0x1A,  0x10,    2},   /**< stage EXT1 */
    {0xC9,  0x87,  0xBC,  0x1A,  0x20,    3},   /**< stage 1  */
    {0xC9,  0x82,  0xBC,  0x9A,  0x20,    4},   /**< stage 2  */
    {0xCB,  0x93,  0xAB,  0x7C,  0x27,    5},   /**< stage 3  */
    {0xCB,  0x8E,  0xAB,  0xFC,  0x27,    6},   /**< stage 4  */
    {0xCB,  0x9E,  0xAB,  0xFC,  0x27,    7},   /**< stage 5  */
    {0xCB,  0x8A,  0x8A,  0xDA,  0x2D,    8},   /**< stage 6  */
    {0xCB,  0x87,  0x8A,  0x5E,  0x3F,    9},   /**< stage 7  */
    {0xCB,  0x82,  0xAA,  0xFE,  0x2F,    10},  /**< stage 8  */
    {0xCB,  0x8D,  0xA9,  0xBC,  0x2B,    11},  /**< stage 9  */
    {0xCB,  0x86,  0xA8,  0x3E,  0x2B,    12},  /**< stage 10 */
    {0xCB,  0x8B,  0xA4,  0xFE,  0x2F,    13},  /**< stage 11 */
    {0xCB,  0x9A,  0xA4,  0xFE,  0x2A,    14},  /**< stage 12 */
    {0xCB,  0x92,  0xA4,  0xBE,  0x3A,    15},  /**< stage 13 */
    {0xCB,  0x52,  0xA4,  0xBE,  0x3A,    16},  /**< stage 14 */
    {0xCB,  0x4E,  0xA4,  0xFE,  0x3E,    17},  /**< stage 15 */
    {0xCB,  0x13,  0xA4,  0xBE,  0x3E,    18},  /**< stage 16 */
    {0xCB,  0x40,  0xB1,  0x7E,  0x2F,    19},  /**< stage 17 */
    {0xCB,  0x44,  0xB1,  0xFC,  0x3F,    20},  /**< stage 18 */
    {0xCB,  0x48,  0xB1,  0xFE,  0x3F,    21},  /**< stage 19 */
    {0xCB,  0x48,  0xB0,  0xFE,  0x2F,    22},  /**< stage 20 */
    {0xCB,  0x4C,  0xB0,  0x3E,  0x3F,    23},  /**< stage 21 */
    {0xCB,  0x00,  0xB0,  0x2A,  0x3E,    24},  /**< stage 22 */
    {0xCB,  0x20,  0xB0,  0xCE,  0x3F,    25},  /**< stage 23 */
};

/** BLE TX power level remapping table definition */
const int8_t TXPOWER_REMAP_M031BT_64K[] =
{
    0,      /**< +9dBm,  stage EXT3 */
    1,      /**< +8dBm,  stage EXT2 */
    2,      /**< +7dBm,  stage EXT1 */
    3,      /**< +6dBm,  stage 1  */
    4,      /**< +5dBm,  stage 2  */
    5,      /**< +4dBm,  stage 3  */
    6,      /**< +3dBm,  stage 4  */
    7,      /**< +2dBm,  stage 5  */
    8,      /**< +1dBm,  stage 6  */
    9,      /**< +0dBm,  stage 7  */
    10,     /**< -1dBm,  stage 8  */
    12,     /**< -2dBm,  stage 10 */
    13,     /**< -3dBm,  stage 11 */
    14,     /**< -4dBm,  stage 12 */
    15,     /**< -5dBm,  stage 13 */
    16,     /**< -6dBm,  stage 14 */
    17,     /**< -7dBm,  stage 15 */
    18,     /**< -8dBm,  stage 16 */
    19,     /**< -9dBm,  stage 17 */
    20,     /**< -10dBm, stage 18 */
    24,     /**< -15dBm, stage 22 */
    25,     /**< -20dBm, stage 23 */
};

const int8_t TXPOWER_REMAP_M031BT_128K[] =
{
    1,      /**< +9dBm,  stage EXT2 */
    2,      /**< +8dBm,  stage EXT1 */
    3,      /**< +7dBm,  stage 1  */
    4,      /**< +6dBm,  stage 2  */
    5,      /**< +5dBm,  stage 3  */
    6,      /**< +4dBm,  stage 4  */
    7,      /**< +3dBm,  stage 5  */
    8,      /**< +2dBm,  stage 6  */
    9,      /**< +1dBm,  stage 7  */
    11,     /**< +0dBm,  stage 9  */
    12,     /**< -1dBm,  stage 10 */
    13,     /**< -2dBm,  stage 11 */
    14,     /**< -3dBm,  stage 12 */
    15,     /**< -4dBm,  stage 13 */
    16,     /**< -5dBm,  stage 14 */
    17,     /**< -6dBm,  stage 15 */
    18,     /**< -7dBm,  stage 16 */
    19,     /**< -8dBm,  stage 17 */
    20,     /**< -9dBm,  stage 18 */
    21,     /**< -10dBm, stage 19 */
    24,     /**< -15dBm, stage 22 */
    25,     /**< -20dBm, stage 23 */
};

const int8_t TXPOWER_REMAP_M032BT_256K[] =
{
    0,      /**< +8dBm,  stage EXT3 */
    0,      /**< +8dBm,  stage EXT3 */
    1,      /**< +7dBm,  stage EXT2 */
    1,      /**< +6dBm,  stage EXT2 */
    2,      /**< +5dBm,  stage EXT1 */
    3,      /**< +4dBm,  stage 1  */
    4,      /**< +3dBm,  stage 2  */
    5,      /**< +2dBm,  stage 3  */
    6,      /**< +1dBm,  stage 4  */
    7,      /**< +0dBm,  stage 5  */
    8,      /**< -1dBm,  stage 6  */
    9,      /**< -2dBm,  stage 7  */
    11,     /**< -3dBm,  stage 9  */
    12,     /**< -4dBm,  stage 10 */
    13,     /**< -5dBm,  stage 11 */
    14,     /**< -6dBm,  stage 12 */
    15,     /**< -7dBm,  stage 13 */
    16,     /**< -8dBm,  stage 14 */
    17,     /**< -9dBm,  stage 15 */
    18,     /**< -10dBm, stage 16 */
    22,     /**< -15dBm, stage 20 */
    23,     /**< -20dBm, stage 21 */
};

const int8_t TXPOWER_REMAP_M032BT_512K[] =
{
    0,      /**< +7dBm,  stage EXT3 */
    0,      /**< +7dBm,  stage EXT3 */
    0,      /**< +7dBm,  stage EXT3 */
    1,      /**< +6dBm,  stage EXT2 */
    2,      /**< +5dBm,  stage EXT1 */
    2,      /**< +4dBm,  stage EXT1 */
    3,      /**< +3dBm,  stage 1  */
    4,      /**< +2dBm,  stage 2  */
    5,      /**< +1dBm,  stage 3  */
    6,      /**< +0dBm,  stage 4  */
    7,      /**< -1dBm,  stage 5  */
    8,      /**< -2dBm,  stage 6  */
    9,      /**< -3dBm,  stage 7  */
    11,     /**< -4dBm,  stage 9  */
    12,     /**< -5dBm,  stage 10 */
    13,     /**< -6dBm,  stage 11 */
    14,     /**< -7dBm,  stage 12 */
    15,     /**< -8dBm,  stage 13 */
    16,     /**< -9dBm,  stage 14 */
    17,     /**< -10dBm, stage 15 */
    22,     /**< -15dBm, stage 20 */
    23,     /**< -20dBm, stage 21 */
};

/** BLE TX power level table definition */
const int8_t (*TXPOWER_TABLE)[6] = TXPOWER_TABLE_MATCHING;

/** The size of BLE TX power level table definition */
uint8_t size_TXPOWER_TABLE = sizeof(TXPOWER_TABLE_MATCHING) / sizeof(TXPOWER_TABLE_MATCHING[0]);

/** BLE TX power level remapping table definition */
const int8_t (*TXPOWER_REMAP) = 0;


/* Set the TX Power remapping table for different chipset */
void setBLE_TxPower_Wrap_Init()
{
    if (GET_CHIP_SERIES_NUM == CHIP_SERIES_NUM_D)
    {
        TXPOWER_REMAP = TXPOWER_REMAP_M031BT_64K;       /* 64 KB */
    }
    else if (GET_CHIP_SERIES_NUM == CHIP_SERIES_NUM_E)
    {
        TXPOWER_REMAP = TXPOWER_REMAP_M031BT_128K;      /* 128 KB */
        /* Set default level form EXT3 to a legal maximum value for all states */
        setBLE_TxPower(TXPOWER_REMAP[TXPOWER_9DBM], STATE_BLE_ADVERTISING);
        setBLE_TxPower(TXPOWER_REMAP[TXPOWER_9DBM], STATE_BLE_SCANNING);
        setBLE_TxPower(TXPOWER_REMAP[TXPOWER_9DBM], STATE_BLE_INITIATING);
    }
    else if (GET_CHIP_SERIES_NUM == CHIP_SERIES_NUM_G)
    {
        TXPOWER_REMAP = TXPOWER_REMAP_M032BT_256K;      /* 256 KB */
    }
    else if (GET_CHIP_SERIES_NUM == CHIP_SERIES_NUM_I)
    {
        TXPOWER_REMAP = TXPOWER_REMAP_M032BT_512K;      /* 512 KB */
    }
    else
    {
        printf("Do not support the chipset for BLE !\n");
        while (1);
    }
}

/* Set BLE TX Power level according remapping table */
BleStackStatus setBLE_TxPower_Wrap(uint8_t power_index, BleMode bleMode)
{
    BleStackStatus status;

    /* Comfirm the TX Power remapping table is set */
    if (TXPOWER_REMAP == 0)
    {
        printf("Warning! Does not initial the TX Power remap table.\n");
        setBLE_TxPower_Wrap_Init();
    }

#if (_CHIP_SELECTION_ == _CHIP_M032BT)
    if (GET_CHIP_SERIES_NUM == CHIP_SERIES_NUM_G)
    {
        /* For 256 KB chipset, the maximum power level is 8 dBm */
        if (power_index < TXPOWER_8DBM)
        {
            printf("Exceed! Set the TX power level to be maximum 8 dBm.\n");
            power_index = TXPOWER_8DBM;
        }
    }
    else if (GET_CHIP_SERIES_NUM == CHIP_SERIES_NUM_I)
    {
        /* For 512 KB chipset, the maximum power level is 7 dBm */
        if (power_index < TXPOWER_7DBM)
        {
            printf("Exceed! Set the TX power level to be maximum 7 dBm.\n");
            power_index = TXPOWER_7DBM;
        }
    }
#endif

    status = setBLE_TxPower(TXPOWER_REMAP[power_index], bleMode);

    return status;
}


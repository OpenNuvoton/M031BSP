/*----------------------------------------------------------------------------*/
/* This file implement BLE store the bonding information to Flash             */
/*----------------------------------------------------------------------------*/
#include "string.h"
#include "BleAppSetting.h"
#include "porting_misc.h"
#include "porting_flash.h"
#include "porting_bonding.h"
#include "ble_bonding.h"

/*******************************************************************
 *      Global Variable Defines
 *******************************************************************/
#ifndef BLE_SUPPORT_BOND
#error "Undefine BLE_SUPPORT_BOND."
#endif

#if (BLE_SUPPORT_BOND == ENABLE_DEF)

#if defined (__CC_ARM)
const uint8_t INFO_FLASHBOND[NUM_OF_TOTAL_INFO_BLK][SIZE_OF_INFO_BLK] __attribute__((at(BONDING_INFORMATION_ADDRESS)));
#elif defined (__ICCARM__)
const uint8_t INFO_FLASHBOND[NUM_OF_TOTAL_INFO_BLK][SIZE_OF_INFO_BLK] @(BONDING_INFORMATION_ADDRESS);
#elif defined (__GNUC__)
volatile const uint8_t INFO_FLASHBOND[NUM_OF_TOTAL_INFO_BLK][SIZE_OF_INFO_BLK] __attribute__((section(".bondingInfo_section")));
#endif

uint8_t flashBackUpRam[SIZE_OF_INFO_BLK];
#endif  //(#if (BLE_SUPPORT_BOND == ENABLE_DEF))

/*******************************************************************
 *      Functions
 *******************************************************************/
#if (BLE_SUPPORT_BOND == ENABLE_DEF)
/** This function is used to flash program for BLE.
 *
 * @param[in] u32Addr    : Address of the flash location to be programmed.
 * @param[in] u8DataPtr  : The data pointer to be programmed.
 * @param[in] u32DataLen : The total length to be programmed.
 * @return none
 */
static void program_BleFlash(uint32_t u32Addr, uint8_t *u8DataPtr, uint32_t u32DataLen)
{
    uint8_t ProgramCnt, ProgramIdx, DataIdx = 0, Remainder;
    uint32_t u32Pattern;

    /*calculate need to program how much times*/
    ProgramCnt = (u32DataLen >> FLASH_PROGRAM_SIZE_FOR_POWER_OF_2);

    for (ProgramIdx = 0; ProgramIdx < ProgramCnt; ProgramIdx++)
    {
        memcpy((uint8_t *)(&u32Pattern), u8DataPtr + DataIdx, FLASH_PROGRAM_SIZE);

        setBLE_FlashProgram((u32Addr + DataIdx), u32Pattern);
        DataIdx = DataIdx + FLASH_PROGRAM_SIZE;
    }

    Remainder = (u32DataLen & (FLASH_PROGRAM_SIZE - 1));
    if (Remainder != 0)     /* ex: divider 4; flash program = 4bytes*/
    {
        /*padding useless byte at the last of programming data to meet the limitation of flash programming size */
        u32Pattern = 0xFFFFFFFF;
        memcpy((uint8_t *)(&u32Pattern), u8DataPtr + DataIdx, Remainder);
        setBLE_FlashProgram((u32Addr + DataIdx), u32Pattern);
    }
}

/** This function is used to erase information bond space for BLE.
 *
 * @param[in] offset    : offset value of the information space to be erased.
 * @return none
 */
static void erase_InfoBond(uint8_t offset)
{
    setBLE_FlashErase((uint32_t)&INFO_FLASHBOND + ((uint16_t)offset << FLASH_PAGE_SIZE_FOR_POWER_OF_2));
}


/* Function : Mark PID to BLE Bond information space.

Description:
    Segment the flash page by BLE Bond INFORMATION block size for efficiency flash usage,
    the way of segmentation is marking sequence number (PID) at the first two bytes of each block.

    block format:
    ____________________________________________
    octets1   |2    |2        | variable         |
    ____________________________________________
    parameter | PID | host id | bond information |
    ____________________________________________

    PID marked into flash when initialization and PBID marked when new bond information needs to be program into flash.
Notice:
    although the PID & PBID mark into flash with 2 bytes but only single byte is meaningful,
    in other word we mark same sequence number twice at the flash and the maximum sequence number of block for now is 0xFE.*/
static void markPID_InfoBond(uint8_t offset)
{
    uint8_t Pattern[2];
    uint16_t i, SeqNum;

    /*mark PID at each block */
    SeqNum = (offset * NUM_OF_INFO_BLK_ONE_PAGE) + 1;

    for (i = 0; i < FLASH_PAGE_SIZE; i = i + SIZE_OF_INFO_BLK)
    {
        Pattern[0] = SeqNum >> 8;
        Pattern[1] = SeqNum;

        program_BleFlash(((uint32_t)&INFO_FLASHBOND + ((uint16_t)offset << FLASH_PAGE_SIZE_FOR_POWER_OF_2) + i), Pattern, sizeof(uint8_t) * 2);
        SeqNum++;
    }
}

/*******************************************************************
 *      Public Functions
 *******************************************************************/
/** This function is used to control Bonding operation for BLE.
 *
 * @param[in] opcode     : control command of bonding operation.
 * @param[in] para_data  : the pointer of command data.
 *
 * @param[out] para_data : pointer to return data. \n
 */
uint8_t *setBLE_CmdFlashBond(uint8_t opcode, uint8_t *para_data)
{
    uint8_t j, hdl_h, hdl_l;
    uint16_t i, PageIdx, Pid;
    uint16_t index, length_value;
    uint16_t host_idx, program_cnt;
    uint32_t u32Pattern;

    switch (opcode)
    {
    case CMD_FB_INIT_INFO_FLASHBOND:
        for (PageIdx = 0; PageIdx < NUM_OF_FLASH_PAGE_FOR_BONDING_INFO_BLK; PageIdx++) // PageIdx = 0,1,2,3
        {
            erase_InfoBond(PageIdx);
            markPID_InfoBond(PageIdx);
        }
        *para_data = FLH_BND_ERR_CODE_NO_ERR;
        break;

    case CMD_FB_INIT_DATA_FLASHBOND:
        break;

    case CMD_FB_CHK_IF_FLASH_INITED:

        /*if the flash was initialized, each block would match the following conditions:
            1. first byte & second byte is the sequence number (uint16_t).

            return parameter:
              *para_data: status*/

        *para_data = FLH_BND_ERR_CODE_FLASH_NOT_INI;
        for (PageIdx = 0; PageIdx < NUM_OF_TOTAL_INFO_BLK; PageIdx++) // PageIdx = 0,1,2,3,4,5,6,7
        {
            if ((uint16_t)((INFO_FLASHBOND[PageIdx][TAB_INFO_FLASHBOND_PID] << 8) + INFO_FLASHBOND[PageIdx][TAB_INFO_FLASHBOND_PID + 1]) == (0x01 + PageIdx))
            {
                if (INFO_FLASHBOND[PageIdx][TAB_INFO_FLASHBOND_HOSTID] == FLASH_DEFAULT_VALUE)
                {
                    for (i = TAB_INFO_FLASHBOND_HOSTID; i < SIZE_OF_INFO_BLK; i++)
                    {
                        if (INFO_FLASHBOND[PageIdx][i] != FLASH_DEFAULT_VALUE)
                        {
                            return para_data; //FLH_BND_ERR_CODE_FLASH_NOT_INI
                        }
                    }
                }
                else
                {
                    for (i = TAB_INFO_FLASHBOND_HOSTID; i < SIZE_OF_INFO_BLK; i++)
                    {
                        if (INFO_FLASHBOND[PageIdx][i] != FLASH_DEFAULT_VALUE)
                        {
                            /* this block seems already saving BLE bond information, double check condition 5*/
                            break;
                        }
                    }
                }
            }
            else
            {
                return para_data; //FLH_BND_ERR_CODE_FLASH_NOT_INI
            }

            /*if block already saving BLE bond information*/
            if (i < SIZE_OF_INFO_BLK)
            {
                /*start check condition 5*/
                for (i = (2 + 2 + SMP_PARA_BOND_SIZE); i < SIZE_OF_KEY_BLK ; i++)
                {
                    if (INFO_FLASHBOND[PageIdx][i] != FLASH_DEFAULT_VALUE)
                    {
                        return para_data; //FLH_BND_ERR_CODE_FLASH_NOT_INI
                    }
                }
            }
        }
        if (PageIdx == NUM_OF_TOTAL_INFO_BLK)
        {
            /*all blocks check finish*/
            *para_data = FLH_BND_ERR_CODE_NO_ERR;
        }
        break;

    case CMD_FB_GET_EXIST_PID_BY_HOST_ID:

        /*Get PID of exist bond information block. (Notice: check all block from last to first)
            "Exist bond information block" means the last block with Host ID.

            return parameter:
              *para_data: status
              *(para_data+TAB_PARA_DATA_PID): PID of BLE Bound data block
              */

        *para_data = FLH_BND_ERR_CODE_NO_ERR;
        Pid = 0;
        for (PageIdx = 0; PageIdx < NUM_OF_TOTAL_INFO_BLK; PageIdx++) // PageIdx = 0,1,2,3,4,5,6,7
        {
            if ((INFO_FLASHBOND[(NUM_OF_TOTAL_INFO_BLK - 1) - PageIdx][TAB_INFO_FLASHBOND_HOSTID] == *(para_data + TAB_PARA_DATA_HOSTID)))
            {
                Pid = (uint16_t)((INFO_FLASHBOND[(NUM_OF_TOTAL_INFO_BLK - 1) - PageIdx][TAB_INFO_FLASHBOND_PID] << 8) + INFO_FLASHBOND[(NUM_OF_TOTAL_INFO_BLK - 1) - PageIdx][TAB_INFO_FLASHBOND_PID + 1]);
                break;  //FLH_BND_ERR_CODE_NO_ERR
            }
        }

        /*all blocks were unused*/
        if (Pid == 0)
        {
            *para_data = FLH_BND_ERR_CODE_NO_EXIST_HOST_ID;
        }
        else
        {
            *(para_data + TAB_PARA_DATA_PID_H) = Pid >> 8;
            *(para_data + TAB_PARA_DATA_PID_L) = Pid;
        }
        break;

    case CMD_FB_GET_NEXT_PID:

        /*Get next empty block (the block without host id) and mark new "host id" into the block to make it as exist block.
            */

        *para_data = FLH_BND_ERR_CODE_NO_ERR;
        for (PageIdx = 0; PageIdx < NUM_OF_TOTAL_INFO_BLK; PageIdx++) //PageIdx = 0,1,2,3,4,5,6,7
        {
            if (INFO_FLASHBOND[(NUM_OF_TOTAL_INFO_BLK - 1) - PageIdx][TAB_INFO_FLASHBOND_HOSTID] != FLASH_DEFAULT_VALUE)
            {
                if ((NUM_OF_TOTAL_INFO_BLK - 1 - PageIdx + 1) <= NUM_OF_TOTAL_INFO_BLK)
                {
                    *(para_data + TAB_PARA_DATA_PID_H) = INFO_FLASHBOND[(NUM_OF_TOTAL_INFO_BLK - 1) - PageIdx + 1][TAB_INFO_FLASHBOND_PID];
                    *(para_data + TAB_PARA_DATA_PID_L) = INFO_FLASHBOND[(NUM_OF_TOTAL_INFO_BLK - 1) - PageIdx + 1][TAB_INFO_FLASHBOND_PID + 1];
                    break;
                }
                else
                {
                    *para_data = FLH_BND_ERR_CODE_NO_FREE_PID;/*reserved for special case*/
                }
            }
        }

        if (*para_data != FLH_BND_ERR_CODE_NO_FREE_PID)
        {
            if (PageIdx == NUM_OF_TOTAL_INFO_BLK)
            {
                *(para_data + TAB_PARA_DATA_PID_H) = 0x00;
                *(para_data + TAB_PARA_DATA_PID_L) = 0x01;
                PageIdx = 0;
            }
            else
            {
                PageIdx = (NUM_OF_TOTAL_INFO_BLK - 1 - PageIdx + 1);
            }

            /*mark PBID into the found block so that this block become exist block*/
            u32Pattern = *(para_data + TAB_PARA_DATA_HOSTID);
            u32Pattern = (u32Pattern << 8) | *(para_data + TAB_PARA_DATA_HOSTID);
            u32Pattern = (u32Pattern << 8) | *(para_data + TAB_PARA_DATA_PID_L);
            u32Pattern = (u32Pattern << 8) | *(para_data + TAB_PARA_DATA_PID_H);
            program_BleFlash(((uint32_t)&INFO_FLASHBOND + (PageIdx << SIZE_OF_INFO_BLK_FOR_POWER_OF_2)),
                             (uint8_t *)(&u32Pattern), sizeof(uint8_t) * 4);

        }
        break;

    case CMD_FB_GET_KEY_FLASHBOND_PARA_BOND:

        /* get the parameters of BLE bond information block that with specific PBID & PID
            input parameter:
              *(para_data+TAB_PARA_DATA_HOSTID): specific host ID
              *(para_data+TAB_PARA_DATA_PID_H & L): specific PID
            return parameter:
              *para_data: status
              *(para_data+TAB_PARA_DATA_INI_ADDR+j) : parameters
           */

        *para_data = FLH_BND_ERR_CODE_NO_ERR;

        PageIdx = (uint16_t)((*(para_data + TAB_PARA_DATA_PID_H) << 8) + * (para_data + TAB_PARA_DATA_PID_L)) - 1;
        if (INFO_FLASHBOND[PageIdx][TAB_INFO_FLASHBOND_HOSTID] == *(para_data + TAB_PARA_DATA_HOSTID))
        {
            /*copy BLE bond information*/
            for (j = 0; j < SMP_PARA_BOND_SIZE; j++)
            {
                *(para_data + TAB_PARA_DATA_INI_ADDR + j) = INFO_FLASHBOND[PageIdx][TAB_INFO_FLASHBOND_INI_ADDR + j];
            }
        }
        else
        {
            *para_data = FLH_BND_ERR_CODE_HOST_ID_DNT_MATCH;
        }
        break;

    case CMD_FB_GET_DATA_FLASHBOND_EXIST_HOSTID_DBLK_START:

        /*get the data start index of BLE Bond data block that with exist host id
            input parameter:
              *(para_data+TAB_PARA_DATA_HOSTID): Host Id
            return parameter:
              *para_data: status
              *(para_data+TAB_PARA_DATA_PID): PID of BLE Bond data block
              *(para_data+TAB_PARA_DATA_DAT_START): data start index of exist BLE Bond data block */

        *para_data = FLH_BND_ERR_CODE_NO_ERR;
        if (SIZE_OF_DATA_BLK)
        {
            for (PageIdx = 0; PageIdx < NUM_OF_TOTAL_INFO_BLK; PageIdx++)
            {
                if (INFO_FLASHBOND[(NUM_OF_TOTAL_INFO_BLK - 1) - PageIdx][TAB_INFO_FLASHBOND_HOSTID] == *(para_data + TAB_PARA_DATA_HOSTID))
                {
                    break;
                }
            }
            if (PageIdx == NUM_OF_TOTAL_INFO_BLK)
            {
                *para_data = FLH_BND_ERR_CODE_NO_EXIST_HOST_ID;
            }
            else
            {
                PageIdx = (NUM_OF_TOTAL_INFO_BLK - 1) - PageIdx;
                *(para_data + TAB_PARA_DATA_PID_H) = INFO_FLASHBOND[PageIdx][TAB_INFO_FLASHBOND_PID];
                *(para_data + TAB_PARA_DATA_PID_L) = INFO_FLASHBOND[PageIdx][TAB_INFO_FLASHBOND_PID + 1];

                index = SIZE_OF_KEY_BLK;
                while (index < SIZE_OF_INFO_BLK)
                {
                    if ((INFO_FLASHBOND[PageIdx][index] != FLASH_DEFAULT_VALUE) || (INFO_FLASHBOND[PageIdx][index + 1] != FLASH_DEFAULT_VALUE))
                    {
                        length_value = INFO_FLASHBOND[PageIdx][index + 3];
                        index = index + length_value + 4;
                    }
                    else
                    {
                        break;
                    }
                }
                if (index >= (SIZE_OF_INFO_BLK - 1 - 4))
                {
                    *para_data = ERR_CODE_FLH_BND_NO_ENOUGH_REST_SPACE;
                }
                *(para_data + TAB_PARA_DATA_DAT_START_H) = index >> 8;
                *(para_data + TAB_PARA_DATA_DAT_START_L) = index;
            }
        }
        else
        {
            *para_data = FLH_BND_ERR_CODE_NO_EXIST_HOST_ID;
        }
        break;

    case CMD_FB_GET_DATA_FLASHBOND_NXT_PID_DBLK_START:

        /*find the start index of next BLE Bound data block and mark host id into the block to make it as exist BLE Bound data block
            return parameter:
              *para_data: status
              *(para_data+TAB_PARA_DATA_PID): PID of block
              *(para_data+TAB_PARA_DATA_DAT_START): data start index of block */

        *para_data = FLH_BND_ERR_CODE_NO_ERR;
        if (SIZE_OF_DATA_BLK)
        {
            for (PageIdx = 0; PageIdx < NUM_OF_TOTAL_INFO_BLK; PageIdx++)
            {
                if (INFO_FLASHBOND[(NUM_OF_TOTAL_INFO_BLK - 1) - PageIdx][TAB_INFO_FLASHBOND_HOSTID] != FLASH_DEFAULT_VALUE)
                {
                    break;
                }
            }

            if (PageIdx == 0)
            {
                *para_data = FLH_BND_ERR_CODE_NO_FREE_PID;
                break;
            }
            else if (PageIdx == NUM_OF_TOTAL_INFO_BLK)
            {
                PageIdx = 0;
            }
            else
            {
                PageIdx = (NUM_OF_TOTAL_INFO_BLK - 1) - PageIdx + 1;
            }

            *(para_data + TAB_PARA_DATA_PID_H) = INFO_FLASHBOND[PageIdx][TAB_INFO_FLASHBOND_PID];
            *(para_data + TAB_PARA_DATA_PID_L) = INFO_FLASHBOND[PageIdx][TAB_INFO_FLASHBOND_PID + 1];

            /*mark Host Id into the found block so that this block become exist block*/
            u32Pattern = *(para_data + TAB_PARA_DATA_HOSTID);
            u32Pattern = (u32Pattern << 8) | *(para_data + TAB_PARA_DATA_HOSTID);
            u32Pattern = (u32Pattern << 8) | *(para_data + TAB_PARA_DATA_PID_L);
            u32Pattern = (u32Pattern << 8) | *(para_data + TAB_PARA_DATA_PID_H);
            program_BleFlash(((uint32_t)&INFO_FLASHBOND + (PageIdx << SIZE_OF_INFO_BLK_FOR_POWER_OF_2)),
                             (uint8_t *)(&u32Pattern), sizeof(uint8_t) * 4);

            *(para_data + TAB_PARA_DATA_DAT_START_H) = SIZE_OF_KEY_BLK >> 8;
            *(para_data + TAB_PARA_DATA_DAT_START_L) = SIZE_OF_KEY_BLK;
        }
        else
        {
            *para_data = FLH_BND_ERR_CODE_NO_FREE_PID;
        }
        break;

    case CMD_FB_PSH_DATA_FLASHBOND_EXIST_HOSTID_DBLK:

        /*push data into the BLE Bound data block which with specific PBID,
        input parameter:
          *(para_data+TAB_PARA_DATA_DAT_START_L & H): current start index of BLE Bound data block which with specific Host ID
          *(para_data+TAB_PARA_DATA_PID):
          *(para_data+TAB_PARA_DATA_DAT_PTR): the index of push data
          *(para_data+TAB_PARA_DATA_GATT_ROLE): the index of GATT data role.
          *(para_data+TAB_PARA_DATA_DAT_SIZE): the size of push data, size does not include length of field index & size */

        *para_data = FLH_BND_ERR_CODE_NO_ERR;
        if (SIZE_OF_DATA_BLK)
        {
            index = ((*(para_data + TAB_PARA_DATA_DAT_START_H)) << 8) + *(para_data + TAB_PARA_DATA_DAT_START_L);
            if ((index + (*(para_data + TAB_PARA_DATA_DAT_SIZE)) + 4) >= (SIZE_OF_INFO_BLK - 1))
            {
                *para_data = ERR_CODE_FLH_BND_NO_ENOUGH_REST_SPACE;
            }
            else
            {
                PageIdx = (*(para_data + TAB_PARA_DATA_PID_H) << 8) + *(para_data + TAB_PARA_DATA_PID_L) - 1;
                if (INFO_FLASHBOND[PageIdx][TAB_INFO_FLASHBOND_HOSTID] == *(para_data + TAB_PARA_DATA_HOSTID))
                {
                    if ((index & (FLASH_PROGRAM_SIZE - 1)) != 0)
                    {
                        u32Pattern = 0xFFFFFF;

                        for (i = 0; i < FLASH_PROGRAM_SIZE ; i++)
                        {
                            u32Pattern = u32Pattern >> 8;
                            if (i < (index & (FLASH_PROGRAM_SIZE - 1)))
                            {
                                u32Pattern |= (uint32_t)INFO_FLASHBOND[PageIdx][index - (index & (FLASH_PROGRAM_SIZE - 1)) + i] << (8 * (FLASH_PROGRAM_SIZE - 1));
                            }
                            else
                            {
                                if ((*(para_data + TAB_PARA_DATA_DAT_SIZE) + 4) >= i)
                                {
                                    u32Pattern |= (uint32_t)(*(para_data + TAB_PARA_DATA_DAT_HDL_H + ( i - (index & (FLASH_PROGRAM_SIZE - 1))))) << (8 * (FLASH_PROGRAM_SIZE - 1));
                                }
                                else
                                {
                                    u32Pattern |= (uint32_t)0xFF << (8 * (FLASH_PROGRAM_SIZE - 1));
                                }
                            }
                        }

                        program_BleFlash(((uint32_t)&INFO_FLASHBOND + (PageIdx << SIZE_OF_INFO_BLK_FOR_POWER_OF_2) + (index - ( index & (FLASH_PROGRAM_SIZE - 1)))),
                                         (uint8_t *)&u32Pattern, FLASH_PROGRAM_SIZE);

                        if ((*(para_data + TAB_PARA_DATA_DAT_SIZE) + 4) > (FLASH_PROGRAM_SIZE - (index & (FLASH_PROGRAM_SIZE - 1))))
                        {
                            program_BleFlash((uint32_t)&INFO_FLASHBOND + (PageIdx << SIZE_OF_INFO_BLK_FOR_POWER_OF_2) + index + (FLASH_PROGRAM_SIZE - (index & (FLASH_PROGRAM_SIZE - 1))),
                                             (uint8_t *)(para_data + TAB_PARA_DATA_DAT_HDL_H + (FLASH_PROGRAM_SIZE - (index & (FLASH_PROGRAM_SIZE - 1)))),
                                             ((*(para_data + TAB_PARA_DATA_DAT_SIZE) + 4) - (FLASH_PROGRAM_SIZE - (index & (FLASH_PROGRAM_SIZE - 1)))));
                        }
                    }
                    else
                    {
                        program_BleFlash((uint32_t)&INFO_FLASHBOND + (PageIdx << SIZE_OF_INFO_BLK_FOR_POWER_OF_2) + index,
                                         (uint8_t *)(para_data + TAB_PARA_DATA_DAT_HDL_H),
                                         (*(para_data + TAB_PARA_DATA_DAT_SIZE) + 4));
                    }
                }
                else
                {
                    *para_data = FLH_BND_ERR_CODE_HOST_ID_DNT_MATCH;
                }
            }
        }
        else
        {
            *para_data = ERR_CODE_FLH_BND_NO_ENOUGH_REST_SPACE;
        }
        break;

    case CMD_FB_CHK_IF_FLASHBOND_NEED_TO_ERASE_PAGE:

        /*easing flash page if following conditions matched
            1. Find already Use block.
            2. If use block more than threshold. Backup => Erase => Program backup data.
            */

        *para_data = FLH_BND_ERR_CODE_NO_ERR;
        for (PageIdx = 0; PageIdx < NUM_OF_TOTAL_INFO_BLK; PageIdx++)
        {
            if (INFO_FLASHBOND[NUM_OF_TOTAL_INFO_BLK - 1 - PageIdx][TAB_INFO_FLASHBOND_HOSTID] != FLASH_DEFAULT_VALUE)
            {
                break;
            }
        }
        if (PageIdx != NUM_OF_TOTAL_INFO_BLK)
        {
            if ((NUM_OF_TOTAL_INFO_BLK - 1 - PageIdx) >= ERASE_THRESHOLD)
            {
                PageIdx = (NUM_OF_TOTAL_INFO_BLK - 1) - PageIdx + 1;
                for (host_idx = 0; host_idx < NUM_HOST_LINK; host_idx++)
                {
                    for (i = 0; i < NUM_OF_TOTAL_INFO_BLK; i++)
                    {
                        if (INFO_FLASHBOND[NUM_OF_TOTAL_INFO_BLK - 1 - i][TAB_INFO_FLASHBOND_HOSTID] == host_idx)
                        {
                            for (index = 0; index < SIZE_OF_INFO_BLK; index++)
                            {
                                if (index < TAB_INFO_FLASHBOND_INI_ADDR)
                                {
                                    flashBackUpRam[index] = 0;
                                }
                                else
                                {
                                    flashBackUpRam[index] = INFO_FLASHBOND[NUM_OF_TOTAL_INFO_BLK - 1 - i][index];
                                }
                            }

                            if (SIZE_OF_BONDING_INFORMATION <= FLASH_PAGE_SIZE)
                            {
                                erase_InfoBond(0);
                                markPID_InfoBond(0);
                                flashBackUpRam[0] =  INFO_FLASHBOND[0][TAB_INFO_FLASHBOND_PID];
                                flashBackUpRam[1] =  INFO_FLASHBOND[0][TAB_INFO_FLASHBOND_PID + 1];
                                flashBackUpRam[2] =  host_idx;
                                flashBackUpRam[3] =  host_idx;

                                program_BleFlash((uint32_t)&INFO_FLASHBOND, (uint8_t *)(&flashBackUpRam), SIZE_OF_INFO_BLK);
                            }
                            else
                            {
                                if (PageIdx < NUM_OF_TOTAL_INFO_BLK)
                                {
                                    flashBackUpRam[0] =  INFO_FLASHBOND[PageIdx - 1][TAB_INFO_FLASHBOND_PID];
                                    flashBackUpRam[1] =  INFO_FLASHBOND[PageIdx - 1][TAB_INFO_FLASHBOND_PID + 1];
                                    flashBackUpRam[2] =  host_idx;
                                    flashBackUpRam[3] =  host_idx;

                                    program_BleFlash(((uint32_t)&INFO_FLASHBOND + (PageIdx << SIZE_OF_INFO_BLK_FOR_POWER_OF_2)),
                                                     (uint8_t *)(&flashBackUpRam), SIZE_OF_INFO_BLK);

                                    PageIdx++;
                                    break;
                                }
                                else
                                {
                                    break;
                                }
                            }
                        }
                    }
                }

                if (SIZE_OF_BONDING_INFORMATION > FLASH_PAGE_SIZE)
                {
                    Pid = 0;
                    host_idx = 0;
                    program_cnt = 0;
                    for (index = 0; index < NUM_OF_FLASH_PAGE_FOR_BONDING_INFO_BLK; index++)
                    {
                        erase_InfoBond(index);
                        markPID_InfoBond(index);

                        while (host_idx < NUM_HOST_LINK)
                        {
                            if ((program_cnt - (NUM_OF_INFO_BLK_ONE_PAGE * index)) <= (NUM_OF_INFO_BLK_ONE_PAGE - 1))
                            {
                                for (PageIdx = 0; PageIdx < NUM_OF_TOTAL_INFO_BLK; PageIdx++)
                                {
                                    if (INFO_FLASHBOND[NUM_OF_TOTAL_INFO_BLK - 1 - PageIdx][TAB_INFO_FLASHBOND_HOSTID] == host_idx)
                                    {
                                        for (i = 0; i < SIZE_OF_INFO_BLK; i++)
                                        {
                                            if (i < TAB_INFO_FLASHBOND_INI_ADDR)
                                            {
                                                flashBackUpRam[i] = 0;
                                            }
                                            else
                                            {
                                                flashBackUpRam[i] = INFO_FLASHBOND[NUM_OF_TOTAL_INFO_BLK - 1 - PageIdx][i];
                                            }
                                        }

                                        flashBackUpRam[0] =  INFO_FLASHBOND[Pid][TAB_INFO_FLASHBOND_PID];
                                        flashBackUpRam[1] =  INFO_FLASHBOND[Pid][TAB_INFO_FLASHBOND_PID + 1];
                                        flashBackUpRam[2] =  host_idx;
                                        flashBackUpRam[3] =  host_idx;

                                        program_BleFlash(((uint32_t)&INFO_FLASHBOND + (Pid << SIZE_OF_INFO_BLK_FOR_POWER_OF_2)),
                                                         (uint8_t *)(&flashBackUpRam), SIZE_OF_INFO_BLK);

                                        Pid++;
                                        program_cnt++;
                                        break;
                                    }
                                }
                                host_idx++;
                            }
                            else
                            {
                                break;
                            }
                        }
                    }
                }
            }
        }
        break;

    case CMD_FB_PSH_BACKUP_KEY_FLASH_PARA_BOND:

        /* Program Backup INFO data. */

        *para_data = FLH_BND_ERR_CODE_NO_ERR;
        PageIdx = (*(para_data + TAB_PARA_DATA_PID_H) << 8) + *(para_data + TAB_PARA_DATA_PID_L) - 1;
        if (INFO_FLASHBOND[PageIdx][TAB_INFO_FLASHBOND_HOSTID] == *(para_data + TAB_PARA_DATA_HOSTID))
        {
            for (i = 0; i < PageIdx ; i++)
            {
                if (INFO_FLASHBOND[(PageIdx - 1) - i][TAB_INFO_FLASHBOND_HOSTID] == *(para_data + TAB_PARA_DATA_HOSTID))
                {
                    for (index = 0; index < SIZE_OF_KEY_BLK; index++)
                    {
                        if (index < TAB_INFO_FLASHBOND_INI_ADDR)
                        {
                            flashBackUpRam[index] = INFO_FLASHBOND[PageIdx][index];
                        }
                        else
                        {
                            flashBackUpRam[index] = INFO_FLASHBOND[(PageIdx - 1) - i][index];
                        }
                    }

                    program_BleFlash(((uint32_t)&INFO_FLASHBOND + (PageIdx << SIZE_OF_INFO_BLK_FOR_POWER_OF_2)),
                                     (uint8_t *)(&flashBackUpRam), SIZE_OF_KEY_BLK);
                    break;
                }
            }
            if ((i == PageIdx) && (PageIdx != 0))
            {
                *para_data = FLH_BND_ERR_CODE_NO_EXIST_HOST_ID;
            }
        }
        else
        {
            *para_data = FLH_BND_ERR_CODE_NO_EXIST_HOST_ID;
        }
        break;

    case CMD_FB_PSH_BACKUP_DATA_FLASH_PARA_BOND:

        /* Program Backup data. */

        *para_data = FLH_BND_ERR_CODE_NO_ERR;
        if (SIZE_OF_DATA_BLK)
        {
            /* check host id is true or not. */
            PageIdx = (*(para_data + TAB_PARA_DATA_PID_H) << 8) + *(para_data + TAB_PARA_DATA_PID_L) - 1;
            if (INFO_FLASHBOND[PageIdx][TAB_INFO_FLASHBOND_HOSTID] == *(para_data + TAB_PARA_DATA_HOSTID))
            {
                /* Program Backup data. */
                for (i = 0; i < PageIdx ; i++)
                {
                    if (INFO_FLASHBOND[(PageIdx - 1) - i][TAB_INFO_FLASHBOND_HOSTID] == *(para_data + TAB_PARA_DATA_HOSTID))
                    {
                        for (index = 0; index < SIZE_OF_DATA_BLK; index++)
                        {
                            flashBackUpRam[index] = INFO_FLASHBOND[(PageIdx - 1) - i][SIZE_OF_KEY_BLK + index];
                        }

                        *(para_data + TAB_PARA_DATA_DAT_START_H) = SIZE_OF_KEY_BLK >> 8;
                        *(para_data + TAB_PARA_DATA_DAT_START_L) = SIZE_OF_KEY_BLK;

                        program_BleFlash(((uint32_t)&INFO_FLASHBOND + (PageIdx << SIZE_OF_INFO_BLK_FOR_POWER_OF_2) + SIZE_OF_KEY_BLK),
                                         (uint8_t *)(&flashBackUpRam), SIZE_OF_DATA_BLK);

                        break;
                    }
                }
                if ((i == PageIdx) && (PageIdx != 0))
                {
                    *para_data = FLH_BND_ERR_CODE_NO_EXIST_HOST_ID;
                }
            }
            else
            {
                *para_data = FLH_BND_ERR_CODE_NO_EXIST_HOST_ID;
            }
        }
        else
        {
            *para_data = FLH_BND_ERR_CODE_NO_EXIST_HOST_ID;
        }
        break;

    case CMD_FB_GET_DATA_VALUE_BY_HANDLE:

        /* 1. check valid host id.
           2. find the handle is exist or not.
            */

        if (SIZE_OF_DATA_BLK)
        {
            /* check host id is true or not. */
            PageIdx = (*(para_data + TAB_PARA_DATA_PID_H) << 8) + *(para_data + TAB_PARA_DATA_PID_L) - 1;
            if (INFO_FLASHBOND[PageIdx][TAB_INFO_FLASHBOND_HOSTID] == *(para_data + TAB_PARA_DATA_HOSTID))
            {
                *para_data = ERR_BND_ERR_CODE_NOT_EXIST_HANDLE;

                /*get data when handle number match. */
                hdl_l = *(para_data + TAB_PARA_DATA_DAT_HDL_L);
                hdl_h = *(para_data + TAB_PARA_DATA_DAT_HDL_H);
                i = (SIZE_OF_INFO_BLK - SIZE_OF_KEY_BLK);
                PageIdx = (*(para_data + TAB_PARA_DATA_PID_H) << 8) + *(para_data + TAB_PARA_DATA_PID_L) - 1;
                while (i < SIZE_OF_INFO_BLK)
                {
                    if ((INFO_FLASHBOND[PageIdx][i] != FLASH_DEFAULT_VALUE) || (INFO_FLASHBOND[PageIdx][i + 1] != FLASH_DEFAULT_VALUE))
                    {
                        if ((INFO_FLASHBOND[PageIdx][i] == hdl_h) && (INFO_FLASHBOND[PageIdx][i + 1] == hdl_l))
                        {
                            if (INFO_FLASHBOND[PageIdx][i + 2] == *(para_data + TAB_PARA_DATA_GATT_ROLE))
                            {
                                *(para_data + TAB_PARA_DATA_DAT_START_H) = i >> 8;
                                *(para_data + TAB_PARA_DATA_DAT_START_L) = i;
                                *para_data = FLH_BND_ERR_CODE_NO_ERR;
                            }
                        }
                    }
                    i = i + INFO_FLASHBOND[PageIdx][i + 3] + 4;
                }
                if (*para_data == FLH_BND_ERR_CODE_NO_ERR)
                {
                    i = (*(para_data + TAB_PARA_DATA_DAT_START_H) << 8) + (*(para_data + TAB_PARA_DATA_DAT_START_L));
                    *(para_data + TAB_PARA_DATA_DAT_SIZE) = INFO_FLASHBOND[PageIdx][i + 3];
                    memcpy((para_data + TAB_PARA_DATA_DAT), (uint8_t *)&INFO_FLASHBOND[PageIdx][i + 4], INFO_FLASHBOND[PageIdx][i + 3]);
                }
            }
            else
            {
                *para_data = FLH_BND_ERR_CODE_HOST_ID_DNT_MATCH;
            }
        }
        else
        {
            *para_data = FLH_BND_ERR_CODE_NO_EXIST_HOST_ID;
        }
        break;

    case CMD_FB_GET_INFO_FLASHBOND_WATER_LEVEL:
        /* check the water level exceeds or not
            1. Find already Use block.
            2. If use block more than threshold. Backup => Erase => Program backup data.
            */
        *para_data = FLH_BND_ERR_CODE_NO_ERR;
        for (PageIdx = 0; PageIdx < NUM_OF_TOTAL_INFO_BLK; PageIdx++)
        {
            if (INFO_FLASHBOND[NUM_OF_TOTAL_INFO_BLK - 1 - PageIdx][TAB_INFO_FLASHBOND_HOSTID] != FLASH_DEFAULT_VALUE)
            {
                break;
            }
        }
        if (PageIdx != NUM_OF_TOTAL_INFO_BLK)
        {
            if ((NUM_OF_TOTAL_INFO_BLK - 1 - PageIdx) >= ERASE_THRESHOLD)
            {
                *para_data = FLH_BND_ERR_CODE_WATER_LEVEL_EXCEEDS;
            }
        }
        break;

    default:
        break;
    }
    return para_data;
}

/** This function is used to fill Bonding information in Flash.
 *
 * @param[in] none
 *
 * @return none
 */
void setBLE_FillInfoBondwithExistPID(uint8_t *para_data)
{
    uint8_t *tmp;
    uint16_t PageIdx;

    /*fill BLE bond information into NEXT PID*/
    tmp = setBLE_CmdFlashBond(CMD_FB_GET_NEXT_PID, (uint8_t *)para_data);
    if (*tmp == FLH_BND_ERR_CODE_NO_FREE_PID)
    {
        //flash error;
        return;
    }
    tmp = setBLE_CmdFlashBond(CMD_FB_GET_EXIST_PID_BY_HOST_ID, (uint8_t *)para_data);

    /*get PageIdx by PID (PageIdx =  0,1,2,3,4,5,6,7)*/
    PageIdx = (uint16_t)((*(tmp + TAB_PARA_DATA_PID_H) << 8) + * (tmp + TAB_PARA_DATA_PID_L)) - 1;

    program_BleFlash(((uint32_t)&INFO_FLASHBOND + (PageIdx << SIZE_OF_INFO_BLK_FOR_POWER_OF_2) + TAB_INFO_FLASHBOND_INI_ADDR),
                     (uint8_t *)(para_data + TAB_PARA_DATA_INI_ADDR), SMP_PARA_BOND_SIZE);
}

/** This function is used to restore bonding data to service and profile.
 *
 * @param[in] none
 *
 * @return none
 */
uint8_t *setBLE_RestoreDataBondwithExistHOSTID(uint8_t *para_data)
{
    uint8_t *tmp;

    if (SIZE_OF_DATA_BLK)
    {
        /*Find the PID by host id*/
        tmp = setBLE_CmdFlashBond(CMD_FB_GET_EXIST_PID_BY_HOST_ID, (uint8_t *)para_data);
        if (*tmp == FLH_BND_ERR_CODE_NO_ERR)
        {
            tmp = setBLE_CmdFlashBond(CMD_FB_GET_DATA_VALUE_BY_HANDLE, (uint8_t *)para_data);
        }
        else
        {
            *para_data = FLH_BND_ERR_CODE_NO_EXIST_HOST_ID;
        }
    }
    else
    {
        *para_data = FLH_BND_ERR_CODE_NO_EXIST_HOST_ID;
    }

    return para_data;
}
#else
/*******************************************************************
 *      Public Functions
 *******************************************************************/

uint8_t *setBLE_CmdFlashBond(uint8_t opcode, uint8_t *para_data)
{
    switch (opcode)
    {
    case CMD_FB_INIT_INFO_FLASHBOND:
        *para_data = FLH_BND_ERR_CODE_NO_ERR;
        break;

    case CMD_FB_INIT_DATA_FLASHBOND:
        *para_data = FLH_BND_ERR_CODE_NO_ERR;
        break;

    case CMD_FB_CHK_IF_FLASH_INITED:
        *para_data = FLH_BND_ERR_CODE_NO_ERR;
        break;

    case CMD_FB_GET_EXIST_PID_BY_HOST_ID:
        *para_data = FLH_BND_ERR_CODE_NO_EXIST_HOST_ID;
        break;

    case CMD_FB_GET_NEXT_PID:
        *para_data = FLH_BND_ERR_CODE_NO_FREE_PID;
        break;

    case CMD_FB_GET_KEY_FLASHBOND_PARA_BOND:
        *para_data = FLH_BND_ERR_CODE_HOST_ID_DNT_MATCH;
        break;

    case CMD_FB_PSH_DATA_FLASHBOND_EXIST_HOSTID_DBLK:
        *para_data = ERR_CODE_FLH_BND_NO_ENOUGH_REST_SPACE;
        break;

    case CMD_FB_GET_DATA_FLASHBOND_NXT_PID_DBLK_START:
        *para_data = FLH_BND_ERR_CODE_NO_FREE_PID;
        break;

    case CMD_FB_GET_DATA_FLASHBOND_EXIST_HOSTID_DBLK_START:
        *para_data = FLH_BND_ERR_CODE_NO_EXIST_HOST_ID;
        break;

    case CMD_FB_CHK_IF_FLASHBOND_NEED_TO_ERASE_PAGE:
        *para_data = FLH_BND_ERR_CODE_NO_ERR;
        break;

    case CMD_FB_PSH_BACKUP_KEY_FLASH_PARA_BOND:
        *para_data = FLH_BND_ERR_CODE_NO_EXIST_HOST_ID;
        break;

    case CMD_FB_PSH_BACKUP_DATA_FLASH_PARA_BOND:
        *para_data = FLH_BND_ERR_CODE_NO_EXIST_HOST_ID;
        break;

    case CMD_FB_GET_INFO_FLASHBOND_WATER_LEVEL:
        *para_data = FLH_BND_ERR_CODE_NO_ERR;
        break;

    default:
        break;
    }
    return para_data;
}

void setBLE_FillInfoBondwithExistPID(uint8_t *para_data)
{
}

uint8_t *setBLE_RestoreDataBondwithExistHOSTID(uint8_t *para_data)
{
    *para_data = FLH_BND_ERR_CODE_NO_EXIST_HOST_ID;
    return para_data;
}
#endif  //(#if (BLE_SUPPORT_BOND == 1))


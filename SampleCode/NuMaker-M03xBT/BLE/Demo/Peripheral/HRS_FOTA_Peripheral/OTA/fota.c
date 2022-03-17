/************************************************************************
 *
 * File Name  : Fota.c
 * Description:
 *
 *
 *******************************************************************/

#include <stdint.h>
#include <stdio.h>
#include "rf_phy.h"
#include "fota.h"
#include "ble_cmd.h"
#include "ble_profile.h"
#include "ble_bonding.h"
#include "porting_ota.h"
#include "porting_flash.h"

/******************************************************************************
 * Definitions
 ******************************************************************************/

typedef uint8_t OtaState;
#define OTA_STATE_IDLE                          0x00  /* FOTA procedure idle */
#define OTA_STATE_START                         0x01  /* FOTA procedure start */
#define OTA_STATE_COMPLETE                      0x02  /* FOTA procedure complete */
#define OTA_STATE_ERASING                       0x03  /* Legacy FW erase */

typedef uint8_t OtaCmd;
#define OTA_CMD_QUERY                           0x00  /* Query current system information */
#define OTA_CMD_START                           0x01  /* Start FW OTA update */
#define OTA_CMD_ERASE                           0x02  /* Erase legacy FW */
#define OTA_CMD_APPLY                           0x03  /* FW transmission completed, apply new FW */

/*Indication error codes for FOTA command*/
typedef uint8_t OtaErrCode;
#define OTA_ERR_CODE_NO_ERR                     0x00  /* Command success */
#define OTA_ERR_CODE_CMD_ERR                    0x01  /* Unsupported command ID  */
#define OTA_ERR_CODE_ALREADY_START              0x02  /* FOTA procedure already start */
#define OTA_ERR_CODE_UPDATE_NOT_START           0x03  /* FOTA procedure was not start */
#define OTA_ERR_CODE_FLASH_ERASE_ERR            0x04  /* Flash erase fail */
#define OTA_ERR_CODE_FW_CRC_ERR                 0x05  /* Receiving FW's CRC incorrect */
#define OTA_ERR_CODE_FW_LEN_ERR                 0x06  /* Receiving FW's length incorrect */
#define OTA_ERR_CODE_OUT_OF_BANK_SIZE           0x07  /* Updating FW's length larger than bank size */

/*Notification reasons for FOTA data*/
typedef uint8_t OtaNotify;
#define OTA_DATA_NOTIFY_PERIODIC                0x00  /* Periodic notification, the notify interval set from FOTA start command */
#define OTA_DATA_NOTIFY_TIMEOUT                 0x01  /* FOTA data was not received in a specific interval */
#define OTA_DATA_NOTIFY_ADDRESS_UNEXPECTED      0x02  /* Received FOTA data's address was not continuously */
#define OTA_DATA_NOTIFY_LEN_ERROR               0x03  /* Received FOTA data length incorrect */
#define OTA_DATA_NOTIFY_TOTAL_LEN_ERR           0x04  /* Total received FOTA data length is larger than bank size */
#define OTA_DATA_NOTIFY_ADDRESS_ERR             0x05  /* Received FOTA data's address is larger than bank size */
#define OTA_DATA_NOTIFY_NOT_START               0x06  /* FOTA data received but FOTA procedure was not start */
#define OTA_DATA_NOTIFY_NONE                    0xFF  /* No notification needs to send */

typedef uint8_t OtaTimer;
#define OTA_TIMER_OTA_DATA                      0x00  /* Timer for send notification if FOTA data was not received in a specific interval */
#define OTA_TIMER_OTA_COMPLETE                  0x01  /* Timer for system reboot */
#define OTA_TIMER_OTA_DISCONNECT                0x02  /* Timer for disconnection & complete FOTA procedure */
#define OTA_TIMER_OTA_ERASING                   0x03  /* Timer for disconnection & erasing legacy FW */

/*OTA step is use for continue the transmission if FOTA procedure restart*/
typedef uint8_t OtaStep;
#define OTA_STEP_INIT                           0x00  /* Step size & current step initialization */
#define OTA_STEP_UPDATE                         0x01  /* Current step update */
#define OTA_STEP_RESET                          0x02  /* Step size and current step reset */

typedef uint32_t Bank1Status;
#define BANK1_STATUS_FLASH_PROGRAMMING          0x00000000
#define BANK1_STATUS_FLASH_ERASE_FAIL           0x00000001


#define OTA_DATA_STEP_TOTAL_NUM                 100  /* Step number means upgrading FW been splitted to how much steps to stamp. 
                                                        Cause each step stamp were saving to the end of structure "ota_information_t", 
                                                        the size of structure "ota_information_t" for now is defined as 512 bytes,
                                                        so that maximum total step number * FLASH_PROGRAM_SIZE for current definitions shall smaller than 448*/
#define OTA_DATA_STEP_STAMPED                   136  /* Specific number use for stamped each steps */

#define PREFIX_LEN                              7
#define FW_INFO_LEN                             16


typedef struct
{
    Bank1Status Bank1Status;     /* Two possible results for Bank 1: already programming, erase failure */
    uint32_t Bank1Ready;         /* Notify bootloader that new FW is stored in Bank 1 */
    uint32_t Bank1Crc;           /* Crc32 checksum of FW in Bank 1 */
    uint32_t Bank1DataLen;       /* Data length of FW in Bank 1 */
    uint32_t Bank1StartAddress;  /* The starting flash address of Bank 1 */
    uint32_t Bank0StartAddress;  /* The starting flash address of Bank 0 */
    uint32_t Reserved[10];       /* Reserved for future use */
    uint32_t ExpectAddrInitStep; /* The initial (first) address for stored step that use for calculate expecting address to retransmission.
                                    For resume FOTA transmission, we use steps to counting so far how much data were store into Bank 1,
                                    more step means more data been stored. Therefore, we can use step number to calculate "Expecting start address".*/
} ota_information_t;

typedef struct
{
    uint8_t Prefix[PREFIX_LEN];
    uint8_t SysInfo[FW_INFO_LEN];

} sys_information_t;


const sys_information_t SystemInfo =
{
    {"Prefix"},          /* Specific string for FOTA tool to search system information, in the code base should not appear second string like this */
    {"sys ver 0001"}
};   /* An example of current system information, able to modify by user */

/******************************************************************************
 * Extern Variables
 ******************************************************************************/
extern uint32_t flash_fota_bank_addr;       // The start address of FOTA bank partition
extern uint32_t flash_fota_info_addr;       // The start address of FOTA info partition

/******************************************************************************
 * Local Variables
 ******************************************************************************/
ota_information_t *OtaInfoPtr;

OtaState          FwUpgradeState        = OTA_STATE_IDLE;

uint32_t          OtaDataExpectAddr     = 0;
uint32_t          OtaDataCurrLen        = 0;

uint32_t          OtaDataNotifyInterval = 0xFFFFFFFF;
uint32_t          LastNotifyDataAddr    = 0;

OtaTimer          TimerType             = 0;
uint32_t          ExpiryTime            = 0;
uint32_t          CurrTime              = 0;

uint8_t           OtaDataBuffer[5];               // 1 byte for header
uint8_t           OtaCmdBuffer[FW_INFO_LEN + 1];  // 1 byte for header

/******************************************************************************
 * Private Functions
 ******************************************************************************/
static void bleFota_SystemReboot(void)
{
    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(UART0);

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable FMC ISP function */
    FMC_Open();

    FMC_SET_LDROM_BOOT();
    NVIC_SystemReset();
}

static uint32_t bleFota_Crc32Checksum(uint32_t flash_addr, uint32_t data_len)
{
    uint32_t i, k;
    uint8_t *Buf = ((uint8_t *)flash_addr);
    uint32_t ChkSum = ~0, len = data_len;

    for (i = 0; i < len; i ++ )
    {
        ChkSum ^= *Buf++;
        for (k = 0; k < 8; k ++)
        {
            ChkSum = ChkSum & 1 ? (ChkSum >> 1) ^ 0xedb88320 : ChkSum >> 1;
        }
    }
    return ~ChkSum;
}


/** FOTA timer start to countdown.
 *
 * @note       The purpose for each fota timer were listed as below: \n
 *             OTA_TIMER_OTA_DATA : Send notify when the FOTA data was not send to device in time. \n
 *             OTA_TIMER_OTA_COMPLETE : System reboot directly if disconnection event is missing. \n
 *             OTA_TIMER_OTA_ERASING : Terminated the connection to erase legacy bank FW and information. \n
 *             OTA_TIMER_OTA_DISCONNECT : After valid FW received, terminated the connection and trigger system reboot when disconnection event received. \n
 *
 * @param[in] Timeout : timer timeout value (unit:sec).
 * @param[in] Type : timer type.
 *
 * @return none
 */
static void bleFota_TimerStart(uint8_t timout, OtaTimer type)
{
    ExpiryTime = timout + CurrTime;
    TimerType = type;
}

/** The actions after FOTA timer expiry.
 *
 * @param[in] Type : expired timer type.
 *
 * @return none
 */
static void bleFota_TimerExpiry(OtaTimer type)
{
    BleStackStatus status;

    switch (type)
    {
    case OTA_TIMER_OTA_DATA:
        OtaDataBuffer[0] = OTA_DATA_NOTIFY_TIMEOUT;
        OtaDataBuffer[1] = (OtaDataExpectAddr & 0xFF);
        OtaDataBuffer[2] = ((OtaDataExpectAddr >> 8) & 0xFF);
        OtaDataBuffer[3] = ((OtaDataExpectAddr >> 16) & 0xFF);

        status = setBLEGATT_Notification( bleProfile_link0_info.hostId,
                                          bleProfile_link0_info.serviceFOTA_info_s.handles.hdl_fota_data,
                                          OtaDataBuffer,
                                          sizeof(uint8_t) * 4);

        if (status != BLESTACK_STATUS_SUCCESS)
        {
            printf("Notify send fail\n");
        }
        LastNotifyDataAddr = OtaDataExpectAddr;

        break;

    case OTA_TIMER_OTA_COMPLETE:
    {
        bleFota_SystemReboot();
    }
    break;

    case OTA_TIMER_OTA_ERASING:
    {
        setBLE_Disconnect(bleProfile_link0_info.hostId);
    }
    break;

    case OTA_TIMER_OTA_DISCONNECT:
    {
        bleFota_TimerStart(3, OTA_TIMER_OTA_COMPLETE);
        setBLE_Disconnect(bleProfile_link0_info.hostId);
    }
    break;

    }
}

/** update Ble FOTA step by input parameter "Action"
 *
 * @note       FOTA step is used to record the bank saving how much upgrading FW roughly,
 *             so that we can continue the transmission if FOTA upgrade process restart unexpectedly. \n
 *             OTA_STEP_INIT : Get current FOTA step \n
 *             OTA_STEP_UPDATE : Stamp next FOTA step \n
 *             OTA_STEP_RESET : Reset current FOTA step to zero \n
 *
 * @param[in] Action : the actions for update FOTA step.
 * @param[in] ExpectAddPtr : the next expect address to program to bank.
 *
 * @return none
 */
static void bleFota_Step(OtaStep action, uint32_t *expect_add)
{
    static uint32_t StepSize = 0;
    static uint32_t CurrStep = 0;

    if (action == OTA_STEP_INIT)
    {
        uint32_t *StepPtr = 0;

        /*get suitable step size*/
        while (StepSize * OTA_DATA_STEP_TOTAL_NUM < SIZE_OF_FOTA_BANK)
        {
            StepSize += FLASH_PROGRAM_SIZE;
        }

        /*calculate current step*/
        StepPtr = &OtaInfoPtr->ExpectAddrInitStep;
        while (*(StepPtr + CurrStep) == OTA_DATA_STEP_STAMPED)
        {
            CurrStep++;
        }

        /*get expect address*/
        *expect_add = CurrStep * StepSize;

    }
    else if (action == OTA_STEP_UPDATE)
    {
        /*stamp new step*/
        if (*expect_add >= (CurrStep + 1)*StepSize)
        {
            setBLE_FlashProgram((uint32_t)(&OtaInfoPtr->ExpectAddrInitStep + CurrStep), OTA_DATA_STEP_STAMPED);
            CurrStep++;
        }

    }
    else if (action == OTA_STEP_RESET)
    {
        CurrStep = 0;
        StepSize = 0;
        *expect_add = 0;
    }
}


/******************************************************************************
 * Public Functions
 ******************************************************************************/

/** Ble FOTA Timer tick
 *
 *
 * @note       This function should be called once every second, \n
 *             so that the fota related timers can be normally operation. \n
 *
 * @retval RUNNING : FOTA timer is running.
 * @retval EXPIRED : FOTA timer is expired.
 */
FotaTimerState BleFota_TimerTick(void)
{
    CurrTime++;

    if ((ExpiryTime != 0) && (CurrTime > ExpiryTime))
    {
        ExpiryTime = 0;
        return EXPIRED;
    }

    return RUNNING;
}


/** Ble FOTA handle FOTA timer expired event
 *
 *
 * @note       This function should be called when FOTA timer is expired.
 *
 * @return none
 */
void BleFota_TimerExpiryHandler(void)
{
    bleFota_TimerExpiry(TimerType);
}

/** Ble FOTA parameters initialization
 *
 *
 * @return none
 */
void BleFota_Init(void)
{
    uint8_t i = 0;

    if (!flash_fota_info_addr)
    {
        /* Set the base address of flash partitions */
        setBLE_FlashPartitionsBA(1);
        OtaInfoPtr = (ota_information_t *)(flash_fota_info_addr);
    }

    bleFota_Step(OTA_STEP_INIT, &OtaDataExpectAddr);

    OtaDataCurrLen = LastNotifyDataAddr = OtaDataExpectAddr;
    OtaDataNotifyInterval = 0xFFFFFFFF;

    printf("sysinfo: ");
    for (i = 0; i < FW_INFO_LEN; i++)
    {
        printf("%c", SystemInfo.SysInfo[i]);
    }
    printf("\n");

}

/** The actions related to FOTA after complete the disconnection.
 *
 * @note       perform the action by FwUpgradeState. \n
 *             OTA_STATE_COMPLETE : System reboot for bootloader to check new FW. \n
 *             OTA_STATE_ERASING : Erase bank FW and bank information and waiting for reconnection. \n
 *
 * @return none
 */
void BleFota_Disconnect(void)
{
    if (FwUpgradeState == OTA_STATE_COMPLETE)
    {
        bleFota_SystemReboot();
    }
    else if (FwUpgradeState == OTA_STATE_ERASING)
    {
        uint32_t PageIdx = 0;

        setBLE_FlashErase(flash_fota_info_addr);
        for (PageIdx = 0 ; PageIdx < SIZE_OF_FOTA_BANK ; PageIdx += FLASH_PAGE_SIZE)
        {
            if (setBLE_FlashErase(flash_fota_bank_addr + PageIdx) != 0)
            {
                setBLE_FlashProgram((uint32_t)(&OtaInfoPtr->Bank1Status), BANK1_STATUS_FLASH_ERASE_FAIL);
                break;
            }
        }
        bleFota_Step(OTA_STEP_RESET, &OtaDataExpectAddr);
        BleFota_Init();


    }

}

/** Ble FOTA command processing
 *
 *
 * @param[in] length : command length.
 * @param[in] data : command payload.
 *
 * @note       First byte of command payload contains command ID and each command ID may contain different information behind.
 *             OTA_CMD_QUERY : Get device current system information. \n
 *             OTA_CMD_START : Start FW upgrade, this command contains new FW length and CRC. \n
 *             OTA_CMD_ERASE : Terminated the connection and erasing legacy FW and information. \n
 *             OTA_CMD_APPLY : Apply the new FW if receiving FW length and CRC matched with OTA_CMD_START. \n
 *
 * @return none
 */
void BleFota_Cmd(uint8_t length, uint8_t *data)
{
    OtaCmd Cmd = data[0];
    OtaErrCode ErrCode = OTA_ERR_CODE_NO_ERR;
    uint8_t IndLen = sizeof(uint8_t); /*first byte of indication always contains error code*/
    uint8_t i;

    switch (Cmd)
    {
    case OTA_CMD_QUERY:
    {
        /* FOTA query command format.
            _______________________
            octets    |1          |
            _______________________
            parameter | CommandID |
                      | (0x00)    |
            _______________________ */
        /*Send system information by indication*/
        for (i = 0 ; i < FW_INFO_LEN ; i++)
        {
            OtaCmdBuffer[IndLen + i] = SystemInfo.SysInfo[i];
        }
        IndLen += FW_INFO_LEN;
    }
    break;

    case OTA_CMD_START:
    {
        uint32_t FwLen;
        uint32_t FwCrc;
        uint8_t CmdIdx = 1;
        /* FOTA start command format.
            _________________________________________________
            octets    |1          |4       |4    |4         |
            _________________________________________________
            parameter | CommandID | FW     | FW  | Notify   |
                      | (0x01)    | length | CRC | interval |
            _________________________________________________ */
        FwLen = ((data[3 + CmdIdx] << 24) | (data[2 + CmdIdx] << 16) | (data[1 + CmdIdx] << 8) | data[CmdIdx]) ;
        CmdIdx += sizeof(uint32_t);

        FwCrc = ((data[3 + CmdIdx] << 24) | (data[2 + CmdIdx] << 16) | (data[1 + CmdIdx] << 8) | data[CmdIdx]) ;
        CmdIdx += sizeof(uint32_t);

        OtaDataNotifyInterval = ((data[3 + CmdIdx] << 24) | (data[2 + CmdIdx] << 16) | (data[1 + CmdIdx] << 8) | data[CmdIdx]) ;
        CmdIdx += sizeof(uint32_t);

        if ((OtaInfoPtr->Bank1Status == BANK1_STATUS_FLASH_PROGRAMMING) || (FwUpgradeState == OTA_STATE_START)) /*Check if there were unfinish FOTA data */
        {
            printf("ExpectAddr: 0x%08x\n", OtaDataExpectAddr);
            OtaDataCurrLen = LastNotifyDataAddr = OtaDataExpectAddr;
            /*Send legacy FW informations (FW length, FW CRC & next expected FW address) by indication*/
            ErrCode = OTA_ERR_CODE_ALREADY_START;
            OtaCmdBuffer[1] = ((OtaInfoPtr->Bank1DataLen) & 0xFF);
            OtaCmdBuffer[2] = ((OtaInfoPtr->Bank1DataLen >> 8) & 0xFF);
            OtaCmdBuffer[3] = ((OtaInfoPtr->Bank1DataLen >> 16) & 0xFF);
            OtaCmdBuffer[4] = ((OtaInfoPtr->Bank1DataLen >> 24) & 0xFF);
            OtaCmdBuffer[5] = ((OtaInfoPtr->Bank1Crc) & 0xFF);
            OtaCmdBuffer[6] = ((OtaInfoPtr->Bank1Crc >> 8) & 0xFF);
            OtaCmdBuffer[7] = ((OtaInfoPtr->Bank1Crc >> 16) & 0xFF);
            OtaCmdBuffer[8] = ((OtaInfoPtr->Bank1Crc >> 24) & 0xFF);
            OtaCmdBuffer[9] = (OtaDataExpectAddr & 0xFF);
            OtaCmdBuffer[10] = ((OtaDataExpectAddr >> 8) & 0xFF);
            OtaCmdBuffer[11] = ((OtaDataExpectAddr >> 16) & 0xFF);

            IndLen += sizeof(uint8_t) * 11;
            FwUpgradeState = OTA_STATE_START;

        }
        else if (FwLen > SIZE_OF_FOTA_BANK) /*Check if updating FW length larger than bank size */
        {
            /*Send bank size by indication*/
            ErrCode = OTA_ERR_CODE_OUT_OF_BANK_SIZE;
            OtaCmdBuffer[1] = (SIZE_OF_FOTA_BANK & 0xFF);
            OtaCmdBuffer[2] = ((SIZE_OF_FOTA_BANK >> 8) & 0xFF);
            OtaCmdBuffer[3] = ((SIZE_OF_FOTA_BANK >> 16) & 0xFF);
            OtaCmdBuffer[4] = ((SIZE_OF_FOTA_BANK >> 24) & 0xFF);
            IndLen += sizeof(uint8_t) * 4;
        }
        else if (OtaInfoPtr->Bank1Status == BANK1_STATUS_FLASH_ERASE_FAIL) /*Check if bank flash was fail to erase*/
        {
            /*Send bank flash fail to erase by indication*/
            ErrCode = OTA_ERR_CODE_FLASH_ERASE_ERR;
        }
        else
        {
            /*Record updating FW information into flash and start FOTA update procedure*/
            FwUpgradeState = OTA_STATE_START;
            OtaDataCurrLen = OtaDataExpectAddr = 0;
            setBLE_FlashProgram((uint32_t)(&OtaInfoPtr->Bank1Status), BANK1_STATUS_FLASH_PROGRAMMING);
            setBLE_FlashProgram((uint32_t)(&OtaInfoPtr->Bank1Crc), FwCrc);
            setBLE_FlashProgram((uint32_t)(&OtaInfoPtr->Bank1DataLen), FwLen);
        }
        printf("fota start %d, ExpectAddr: 0x%08x interval %d\n", ErrCode, OtaDataExpectAddr, OtaDataNotifyInterval);

    }
    break;

    case OTA_CMD_ERASE:
    {
        /* FOTA erase command format.
            _______________________
            octets    |1          |
            _______________________
            parameter | CommandID |
                      | (0x02)    |
            _______________________ */
        /*Erasing the legacy updating FW and FW information*/
        FwUpgradeState = OTA_STATE_ERASING;
        bleFota_TimerStart(1, OTA_TIMER_OTA_ERASING);
    }
    break;

    case OTA_CMD_APPLY:
    {
        uint32_t FwUpdateAdd;
        uint32_t ChkSum;
        uint8_t CmdIdx = 1;

        /* FOTA apply command format.
            _________________________________
            octets    |1          |4        |
            _________________________________
            parameter | CommandID | Bank0   |
                      | (0x03)    | address |
            _________________________________  */

        if (FwUpgradeState != OTA_STATE_START) /*Check if FOTA procedure has started */
        {
            ErrCode = OTA_ERR_CODE_UPDATE_NOT_START;
        }
        else
        {
            FwUpdateAdd = ((data[3 + CmdIdx] << 24) | (data[2 + CmdIdx] << 16) | (data[1 + CmdIdx] << 8) | data[CmdIdx]) ;
            CmdIdx += sizeof(uint32_t);
            FwUpgradeState = OTA_STATE_ERASING;

            ChkSum = bleFota_Crc32Checksum((uint32_t)flash_fota_bank_addr, OtaInfoPtr->Bank1DataLen);
            printf("ChkSum 0x%08x 0x%08x\n", ChkSum,  OtaInfoPtr->Bank1Crc);
            if (OtaDataCurrLen != OtaInfoPtr->Bank1DataLen) /*Check if receiving FW length matched FOTA start command*/
            {
                ErrCode = OTA_ERR_CODE_FW_LEN_ERR;
            }
            else if (ChkSum != OtaInfoPtr->Bank1Crc) /*Check if receiving FW CRC matched FOTA start command*/
            {
                printf("ChkSum 0x%08x 0x%08x\n", ChkSum,  OtaInfoPtr->Bank1Crc);
                ErrCode = OTA_ERR_CODE_FW_CRC_ERR;
            }
            else
            {
                /*Set FOTA informations into flash for bootloader*/
                setBLE_FlashProgram((uint32_t)(&OtaInfoPtr->Bank1Ready), FOTA_IMAGE_READY);
                setBLE_FlashProgram((uint32_t)(&OtaInfoPtr->Bank1StartAddress), (uint32_t)flash_fota_bank_addr);
                setBLE_FlashProgram((uint32_t)(&OtaInfoPtr->Bank0StartAddress), FwUpdateAdd);

                FwUpgradeState = OTA_STATE_COMPLETE;

            }
            bleFota_TimerStart(1, OTA_TIMER_OTA_DISCONNECT);
        }
        printf("fota apply %d\n", ErrCode);
    }
    break;

    default:
        printf("not supported %d\n", Cmd);
        ErrCode = OTA_ERR_CODE_CMD_ERR;
        break;
    }

    OtaCmdBuffer[0] = ErrCode;

    setBLEGATT_Indication(bleProfile_link0_info.hostId,
                          bleProfile_link0_info.serviceFOTA_info_s.handles.hdl_fota_command,
                          OtaCmdBuffer,
                          IndLen);

}


/** Ble FOTA data processing
 *
 *
 * @param[in] length : data length.
 * @param[in] data : data payload.
 *
 * @note       First 4 bytes of data payload is data header which contains the FOTA data address (3 bytes) and length (1 byte),
 *             if there were invalid data header, send notification to response it. \n
 *
 * @return none
 */
void BleFota_Data(uint8_t length, uint8_t *data)
{
    BleStackStatus status;
    uint32_t DataAddr = 0;
    uint32_t OtaData = 0;
    uint32_t OtaDataIdx = 0;
    OtaNotify Notify = OTA_DATA_NOTIFY_NONE;
    uint8_t DataLen;
    uint8_t NotifyLen = 0;
    uint8_t DataOffset;

    /* FOTA data format.
    _____________________________________
    octets    |3        |1       | Var  |
    _____________________________________
    parameter | Data    | Data   | Data |
              | address | length |      |
    _____________________________________  */
    DataAddr = ((data[2] << 16) | (data[1] << 8) | data[0]) ;
    DataLen = data[3];
    OtaDataIdx += sizeof(uint32_t);
    if (FwUpgradeState != OTA_STATE_START) /*Check if FOTA procedure has started*/
    {
        Notify = OTA_DATA_NOTIFY_NOT_START;
    }
    else if (DataAddr > SIZE_OF_FOTA_BANK) /*Check if FOTA data's address is larger than bank size*/
    {
        Notify = OTA_DATA_NOTIFY_ADDRESS_ERR;

    }
    else if (DataAddr != OtaDataExpectAddr) /*Check if FOTA data's address is matched with expecting*/
    {
        Notify = OTA_DATA_NOTIFY_ADDRESS_UNEXPECTED;

        OtaDataBuffer[1] = (OtaDataExpectAddr & 0xFF);
        OtaDataBuffer[2] = ((OtaDataExpectAddr >> 8) & 0xFF);
        OtaDataBuffer[3] = ((OtaDataExpectAddr >> 16) & 0xFF);
        NotifyLen += (sizeof(uint8_t) * 3);
        LastNotifyDataAddr = OtaDataExpectAddr;
        printf("Add unexp %d %d\n", DataAddr, OtaDataExpectAddr);
    }
    else if ((DataLen == 0) || (DataLen > length)) /*Check if FOTA data length vaild*/
    {
        Notify = OTA_DATA_NOTIFY_LEN_ERROR;

        OtaDataBuffer[1] = DataLen;
        NotifyLen += sizeof(uint8_t);
        printf("data len %d length %d\n", DataLen, length);
    }
    else if ((DataLen + OtaDataCurrLen) >  SIZE_OF_FOTA_BANK)/*Check if total received FOTA data length larger than bank size*/
    {
        Notify = OTA_DATA_NOTIFY_TOTAL_LEN_ERR;

        OtaDataBuffer[1] = ((OtaDataCurrLen + DataLen) & 0xFF);
        OtaDataBuffer[2] = (((OtaDataCurrLen + DataLen) >> 8) & 0xFF);
        OtaDataBuffer[3] = (((OtaDataCurrLen + DataLen) >> 16) & 0xFF);
        OtaDataBuffer[4] = (((OtaDataCurrLen + DataLen) >> 24) & 0xFF);
        NotifyLen += (sizeof(uint8_t) * 4);
    }

    if (Notify != OTA_DATA_NOTIFY_NONE)/*Check if notification needs to send*/
    {
        OtaDataBuffer[0] = Notify;
        NotifyLen += sizeof(uint8_t);

        status = setBLEGATT_Notification(bleProfile_link0_info.hostId,
                                         bleProfile_link0_info.serviceFOTA_info_s.handles.hdl_fota_data,
                                         OtaDataBuffer,
                                         NotifyLen);
        if (status != BLESTACK_STATUS_SUCCESS)
        {
            printf("Notify send %d\n", Notify);
        }
    }
    else
    {
        /*waiting for RF enter sleep mode*/
        while (getRF_Mode() == BLERFMODE_ACTIVE);

        /*programming received FOTA data into flash bank1*/
        for (DataOffset = 0; DataOffset < DataLen; DataOffset += FLASH_PROGRAM_SIZE)
        {
            uint8_t CopyIdx;
            while (getRF_Mode() == BLERFMODE_ACTIVE);

            OtaData = 0;

            /*check whether the bin file size align with flash programming size*/
            if (DataOffset + FLASH_PROGRAM_SIZE > DataLen)
            {
                uint8_t i, RemainB = (DataOffset + FLASH_PROGRAM_SIZE - DataLen);

                for (i = 0; i < (FLASH_PROGRAM_SIZE- RemainB); i++)
                {
                    OtaData |= (data[i + OtaDataIdx] << (i * 8));
                }

                for (i = (FLASH_PROGRAM_SIZE- RemainB); i < FLASH_PROGRAM_SIZE; i++)
                {
                    OtaData |= (0xFF << (i * 8));
                }
            }
            else
            {
                for (CopyIdx = 0; CopyIdx < FLASH_PROGRAM_SIZE; CopyIdx++)
                {
                    OtaData |= (data[CopyIdx + OtaDataIdx] << (CopyIdx * 8));
                }
            }
            setBLE_FlashProgram((uint32_t)(OtaDataExpectAddr + flash_fota_bank_addr), OtaData);

            OtaDataExpectAddr += FLASH_PROGRAM_SIZE;
            OtaDataIdx += FLASH_PROGRAM_SIZE;

            bleFota_Step(OTA_STEP_UPDATE, &OtaDataExpectAddr);
        }

        /*Check if periodic notification interval reached*/
        if (OtaDataExpectAddr >= (OtaDataNotifyInterval + LastNotifyDataAddr))
        {
            LastNotifyDataAddr = OtaDataExpectAddr;
            OtaDataBuffer[0] = OTA_DATA_NOTIFY_PERIODIC;
            OtaDataBuffer[1] = (OtaDataExpectAddr & 0xFF);
            OtaDataBuffer[2] = ((OtaDataExpectAddr >> 8) & 0xFF);
            OtaDataBuffer[3] = ((OtaDataExpectAddr >> 16) & 0xFF);

            status = setBLEGATT_Notification(bleProfile_link0_info.hostId,
                                             bleProfile_link0_info.serviceFOTA_info_s.handles.hdl_fota_data,
                                             OtaDataBuffer,
                                             (sizeof(uint8_t) * 4));

            printf("Notify int 0x%04x %d\n", OtaDataExpectAddr, status);
        }

        OtaDataCurrLen += DataLen;
        bleFota_TimerStart(1, OTA_TIMER_OTA_DATA);

    }
}


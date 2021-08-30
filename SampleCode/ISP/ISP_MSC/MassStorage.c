/******************************************************************************//**
 * @file     MassStorage.c
 * @version  V1.00
 * @brief    M031 series USBD driver Sample file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include "M031Series_User.h"
#include "massstorage.h"

#if 0
#define DBG_PRINTF      printf
#else
#define DBG_PRINTF(...)
#endif


#define EP0_MAX_PKT_SIZE    64

/*--------------------------------------------------------------------------*/
/* Global variables for Control Pipe */
uint32_t g_TotalSectors;
uint32_t g_u32u32StorageSize;
uint8_t volatile g_u8EP3Ready = 0;

/* USB flow control variables */
uint8_t g_u8BulkState;
uint8_t g_u8Size;
uint8_t g_au8SenseKey[4];
uint32_t g_u32DataFlashStartAddr;
uint32_t g_u32Address;
uint32_t g_u32Length;
uint32_t g_u32LbaAddress;
uint32_t g_u32BytesInStorageBuf;
uint32_t g_u32BulkBuf0, g_u32BulkBuf1;

/* CBW/CSW variables */
struct CBW g_sCBW;
struct CSW g_sCSW;

uint32_t MassBlock[MASS_BUFFER_SIZE / 4];
uint32_t Storage_Block[STORAGE_BUFFER_SIZE / 4];

/*--------------------------------------------------------------------------*/
uint8_t g_au8InquiryID[36] =
{
    0x00,                   /* Peripheral Device Type */
    0x80,                   /* RMB */
    0x00,                   /* ISO/ECMA, ANSI Version */
    0x00,                   /* Response Data Format */
    0x1F, 0x00, 0x00, 0x00, /* Additional Length */

    /* Vendor Identification */
    'N', 'u', 'v', 'o', 't', 'o', 'n', ' ',

    /* Product Identification */
    'U', 'S', 'B', ' ', 'M', 'a', 's', 's', ' ', 'S', 't', 'o', 'r', 'a', 'g', 'e',

    /* Product Revision */
    '1', '.', '0', '0'
};

/* code = 5Ah, Mode Sense */
static uint8_t g_au8ModePage_3F[64] =
{
    0x01, 0x0A, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00,
    0x03, 0x00, 0x00, 0x00, 0x05, 0x1E, 0x13, 0x88,
    0x08, 0x20, 0x02, 0x00, 0x01, 0xF4, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05,
    0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x68, 0x00, 0x00, 0x1B, 0x0A, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x1C, 0x06, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00
};


void USBD_IRQHandler(void)
{
    uint32_t u32IntSts = USBD_GET_INT_FLAG();
    uint32_t u32State = USBD_GET_BUS_STATE();

    if(u32IntSts & USBD_INTSTS_FLDET)
    {
        // Floating detect
        USBD_CLR_INT_FLAG(USBD_INTSTS_FLDET);

        if (USBD->VBUSDET)
        {
            /* USB Plug In */
            USBD_ENABLE_USB();
        }
        else
        {
            /* USB Un-plug */
            USBD_DISABLE_USB();
        }
    }

    if(u32IntSts & USBD_INTSTS_BUS)
    {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_BUS);

        if(u32State & USBD_STATE_USBRST)
        {
            /* Bus reset */
            USBD_ENABLE_USB();
            USBD_SwReset();
            DBG_PRINTF("Bus reset\n");
        }
        if(u32State & USBD_STATE_SUSPEND)
        {
            /* Enable USB but disable PHY */
            USBD_DISABLE_PHY();
            DBG_PRINTF("Suspend\n");
        }
    }

    if(u32IntSts & USBD_INTSTS_USB)
    {
        // EP events
        if(u32IntSts & USBD_INTSTS_EP0)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP0);
            /* control IN */
            USBD_CtrlIn();
        }

        if(u32IntSts & USBD_INTSTS_EP1)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP1);

            /* control OUT */
        }

        if(u32IntSts & USBD_INTSTS_EP2)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP2);
            /* Bulk IN */
            MSC_AckCmd();
        }

        if(u32IntSts & USBD_INTSTS_EP3)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP3);
            /* Bulk OUT */
            g_u8EP3Ready = 1;
        }

        /* USB event */
        if(u32IntSts & USBD_INTSTS_SETUP)
        {
            /* Setup packet */
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SETUP);

            /* Clear the data IN/OUT ready flag of control end-points */
            USBD_STOP_TRANSACTION(EP0);
            USBD_STOP_TRANSACTION(EP1);

            USBD_ProcessSetupPacket();
        }
    }

    /* clear unknown event */
    USBD_CLR_INT_FLAG(u32IntSts);
}


void USBD_MSC_Stall(void)
{
    USBD_SET_EP_STALL(EP2);
    USBD_SET_EP_STALL(EP3);
}

void MSC_Init(void)
{
    /* Init setup packet buffer */
    /* Buffer range for setup packet -> [0 ~ 0x7] */
    USBD->STBUFSEG = SETUP_BUF_BASE;

    /*****************************************************/
    /* EP0 ==> control IN endpoint, address 0 */
    USBD_CONFIG_EP(EP0, USBD_CFG_CSTALL | USBD_CFG_EPMODE_IN | 0);
    /* Buffer range for EP0 */
    USBD_SET_EP_BUF_ADDR(EP0, EP0_BUF_BASE);

    /* EP1 ==> control OUT endpoint, address 0 */
    USBD_CONFIG_EP(EP1, USBD_CFG_CSTALL | USBD_CFG_EPMODE_OUT | 0);
    /* Buffer range for EP1 */
    USBD_SET_EP_BUF_ADDR(EP1, EP1_BUF_BASE);

    /*****************************************************/
    /* EP2 ==> Bulk IN endpoint, address 2 */
    USBD_CONFIG_EP(EP2, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM);
    /* Buffer range for EP2 */
    USBD_SET_EP_BUF_ADDR(EP2, EP2_BUF_BASE);

    /* EP3 ==> Bulk Out endpoint, address 3 */
    USBD_CONFIG_EP(EP3, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM);
    /* Buffer range for EP3 */
    USBD_SET_EP_BUF_ADDR(EP3, EP3_BUF_BASE);

    /* trigger to receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);

    /*****************************************************/
    g_u32BulkBuf0 = EP3_BUF_BASE;
    g_u32BulkBuf1 = EP2_BUF_BASE;

    g_sCSW.dCSWSignature = CSW_SIGNATURE;
    g_u32u32StorageSize = FMC_Init();
    g_u32u32StorageSize += DATA_SEC_ADDR;
    g_TotalSectors = g_u32u32StorageSize / UDC_SECTOR_SIZE;
}

void MSC_ClassRequest(void)
{
    uint8_t buf[8];
    uint8_t u8Temp;
    uint32_t u32Sum;

    USBD_GetSetupPacket(buf);

    u32Sum = buf[2] + buf[3] + buf[6] + buf[7];
    u8Temp = gsInfo.gu8ConfigDesc[LEN_CONFIG + 2];
    switch(buf[1])
    {
    case GET_MAX_LUN:
    {
        /* Check interface number with cfg descriptor and check wValue = 0, wLength = 1 */
        if((buf[4] == u8Temp) && (u32Sum == 1))
        {
            M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = 0;
            /* Data stage */
            USBD_SET_DATA1(EP0);
            USBD_SET_PAYLOAD_LEN(EP0, 1);

            /* Status stage */
            USBD_SET_PAYLOAD_LEN(EP1, EP0_MAX_PKT_SIZE);
        }
        else
            USBD_SET_EP_STALL(EP1); /* Stall when wrong parameter */

        break;
    }
    case BULK_ONLY_MASS_STORAGE_RESET:
    {
        /* Check interface number with cfg descriptor and check wValue = 0, wLength = 0 */
        if ((buf[4] == u8Temp) && (u32Sum == 0))
        {
            g_u32Length = 0; /* Reset all read/write data transfer */

            /* Clear ready */
            USBD->EP[EP2].CFGP |= USBD_CFGP_CLRRDY_Msk;
            USBD->EP[EP3].CFGP |= USBD_CFGP_CLRRDY_Msk;

            /* Prepare to receive the CBW */
            g_u8EP3Ready = 0;
            g_u8BulkState = BULK_CBW;

            USBD_SET_DATA1(EP3);
            USBD_SET_EP_BUF_ADDR(EP3, g_u32BulkBuf0);
            USBD_SET_PAYLOAD_LEN(EP3, 31);
        }
        else
        {
            /* Stall when wrong parameter */
            USBD_SET_EP_STALL(EP1);
        }

        /* Status stage */
        USBD_SET_DATA1(EP0);
        USBD_SET_PAYLOAD_LEN(EP0, 0);

        break;
    }
    }
}


void MSC_ReadCapacity1(uint32_t u32Offset, uint8_t u8OPCode)
{
    uint32_t tmp = g_TotalSectors;
    uint8_t *pu8Desc;

    pu8Desc = (uint8_t *)MassCMD_BUF;

    USBD_MemReset(pu8Desc, 36);

    if (u8OPCode)
    {
        tmp--;

        pu8Desc[u32Offset + 2] = (uint8_t)(tmp >> 8);
        pu8Desc[u32Offset + 3] = tmp & 0xFF;

        pu8Desc[u32Offset + 6] = 0x02;
    }
    else
    {
        /*---------- Capacity List Header ----------*/
        /* Capacity List Length */
        pu8Desc[3] = 0x10;

        /*---------- Current/Maximum Capacity Descriptor ----------*/
        /* Number of blocks (MSB first) */
        pu8Desc[6] = (uint8_t)(tmp >> 8);
        pu8Desc[7] = tmp & 0xFF;

        /* Descriptor Code:                                                             */
        /* 01b = Unformatted Media - Maximum formattable capacity for this cartridge    */
        /* 10b = Formatted Media - Current media capacity                               */
        /* 11b = No Cartridge in Drive - Maximum formattable capacity for any cartridge */
        pu8Desc[8] = 0x02;

        // Block Length. Fixed to be 512 (MSB first)
        pu8Desc[10] = 0x02;

        /*---------- Formattable Capacity Descriptor ----------*/
        /* Number of Blocks */
        pu8Desc[14] = (uint8_t)(tmp >> 8);
        pu8Desc[15] = tmp & 0xFF;

        /* Block Length. Fixed to be 512 (MSB first) */
        pu8Desc[18] = 0x02;
    }
}


void MSC_Read(uint8_t u8IsTrig)
{
    uint32_t u32Len;
    uint32_t u32Buf;

    if(USBD_GET_EP_BUF_ADDR(EP2) == g_u32BulkBuf1)
        u32Buf = g_u32BulkBuf0;
    else
        u32Buf = g_u32BulkBuf1;

    if (!u8IsTrig)
    {
        USBD_SET_EP_BUF_ADDR(EP2, u32Buf);

        /* Trigger to send out the data packet */
        USBD_SET_PAYLOAD_LEN(EP2, g_u8Size);

        g_u32Length -= g_u8Size;
        g_u32BytesInStorageBuf -= g_u8Size;
    }

    if (g_u32Length)
    {
        if (!g_u32BytesInStorageBuf)
        {
            u32Len = g_u32Length;
            if (u32Len > STORAGE_BUFFER_SIZE)
                u32Len = STORAGE_BUFFER_SIZE;

            DataFlashRead(g_u32LbaAddress, Storage_Block);

            g_u32BytesInStorageBuf = u32Len;
            g_u32LbaAddress += u32Len;
            g_u32Address = STORAGE_DATA_BUF;
        }

        /* Prepare next data packet */
        g_u8Size = EP2_MAX_PKT_SIZE;
        if (g_u8Size > g_u32Length)
            g_u8Size = g_u32Length;

        USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + u32Buf), (uint8_t *)g_u32Address, g_u8Size);
        g_u32Address += g_u8Size;

        if (u8IsTrig)
        {
            /* DATA0/DATA1 Toggle */
            USBD_SET_EP_BUF_ADDR(EP2, u32Buf);

            /* Trigger to send out the data packet */
            USBD_SET_PAYLOAD_LEN(EP2, g_u8Size);

            g_u32Length -= g_u8Size;
            g_u32BytesInStorageBuf -= g_u8Size;
        }
    }
    else if (u8IsTrig)
        USBD_SET_PAYLOAD_LEN(EP2, 0);
}


void MSC_ModeSense10(void)
{
    uint8_t NumHead, NumSector;
    uint16_t NumCyl = 0;

    /* Clear the command buffer */
    *((uint32_t *)MassCMD_BUF) = 0;
    *((uint32_t *)MassCMD_BUF + 1) = 0;

    switch(g_sCBW.au8Data[0])
    {
    case 0x3F:
        *((uint8_t *)MassCMD_BUF) = 0x47;

        USBD_MemCopy((uint8_t *)MassCMD_BUF + 8, g_au8ModePage_3F, sizeof(g_au8ModePage_3F));

        NumHead = 2;
        NumSector = 64;
        NumCyl = g_TotalSectors / 128;

        *((uint8_t *)(MassCMD_BUF + 24)) = NumHead;
        *((uint8_t *)(MassCMD_BUF + 25)) = NumSector;
        *((uint8_t *)(MassCMD_BUF + 28)) = (uint8_t)(NumCyl >> 8);
        *((uint8_t *)(MassCMD_BUF + 29)) = (uint8_t)(NumCyl & 0x00FF);
        break;

    default:
        g_au8SenseKey[0] = 0x05;
        g_au8SenseKey[1] = 0x24;
        g_au8SenseKey[2] = 0x00;
    }
}

void MSC_Write(void)
{
    uint32_t lba, len;
    uint32_t u32Bufa, u32Bufb;

    if(USBD_GET_EP_BUF_ADDR(EP3) == g_u32BulkBuf0)
    {
        u32Bufa = g_u32BulkBuf1;
        u32Bufb = g_u32BulkBuf0;
    }
    else
    {
        u32Bufa = g_u32BulkBuf0;
        u32Bufb = g_u32BulkBuf1;
    }

    if (g_u32Length > EP3_MAX_PKT_SIZE)
    {
        USBD_SET_EP_BUF_ADDR(EP3, u32Bufa);
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
        USBD_MemCopy((uint8_t *)g_u32Address, (uint8_t *)((uint32_t)USBD_BUF_BASE + u32Bufb), EP3_MAX_PKT_SIZE);

        g_u32Address += EP3_MAX_PKT_SIZE;
        g_u32Length -= EP3_MAX_PKT_SIZE;

        /* Buffer full. Writer it to storage first. */
        if (g_u32Address >= (STORAGE_DATA_BUF + STORAGE_BUFFER_SIZE))
        {
            DataFlashWrite(g_u32DataFlashStartAddr, Storage_Block);
            g_u32Address = STORAGE_DATA_BUF;
            g_u32DataFlashStartAddr += STORAGE_BUFFER_SIZE;
        }
    }
    else
    {
        USBD_MemCopy((uint8_t *)g_u32Address, (uint8_t *)((uint32_t)USBD_BUF_BASE + u32Bufb), g_u32Length);

        g_u32Address += g_u32Length;
        g_u32Length = 0;

        lba = get_be32(&g_sCBW.au8Data[0]);

        len = lba * UDC_SECTOR_SIZE + g_sCBW.dCBWDataTransferLength - g_u32DataFlashStartAddr;

        if (len)
        {
            DataFlashWrite(g_u32DataFlashStartAddr, Storage_Block);
        }

        g_u8BulkState = BULK_IN;
        MSC_AckCmd();
    }
}

void MSC_ProcessCmd(void)
{
    uint32_t u32Buf;
    int32_t i;

    if (!g_u8EP3Ready)
        return;

    g_u8EP3Ready = 0;

    if (g_u8BulkState == BULK_CBW)
    {
        /* Check Signature & length of CBW */
        /* Bulk Out buffer */
        u32Buf = USBD_BUF_BASE + g_u32BulkBuf0;

        if ((*(uint32_t *)u32Buf) != CBW_SIGNATURE)
        {
            USBD_MSC_Stall();

            g_u8BulkState = BULK_CBW;
            DBG_PRINTF("CBW signature fail. stall bulk out pipe\n");
            return;
        }

        /* Get the CBW */
        USBD_MemCopy((uint8_t *)&g_sCBW.dCBWSignature, (uint8_t *)u32Buf, 31);

        /* Prepare to echo the tag from CBW to CSW */
        g_sCSW.dCSWTag = g_sCBW.dCBWTag;

        /* Parse Op-Code of CBW */
        switch(g_sCBW.u8OPCode)
        {
        case UFI_VERIFY_10:
        case UFI_TEST_UNIT_READY:
        {
            DBG_PRINTF("Test Unit\n");
            g_u8BulkState = BULK_IN;
            MSC_AckCmd();
            return;
        }
        case UFI_READ_FORMAT_CAPACITY:
        case UFI_READ_CAPACITY:
        case UFI_READ_CAPACITY_16:
        case UFI_MODE_SENSE_10:
        {
            uint32_t u32Offset = 0;
            uint32_t u32Limit = 36;

            g_u32Address = MassCMD_BUF;

            if (g_sCBW.u8OPCode == UFI_MODE_SENSE_10)
            {
                if (g_u32Length == 0)
                    g_u32Length = g_sCBW.dCBWDataTransferLength;

                MSC_ModeSense10();
            }
            else
            {
                if (g_sCBW.u8OPCode == UFI_READ_FORMAT_CAPACITY)
                    u32Limit = 20;

                g_u32Length = g_sCBW.dCBWDataTransferLength;
                if (g_u32Length > u32Limit)
                    g_u32Length = u32Limit;

                if (g_sCBW.u8OPCode == UFI_READ_CAPACITY_16)
                    u32Offset = 4;

                MSC_ReadCapacity1(u32Offset, g_sCBW.u8OPCode & 0x04);
            }

            g_u8BulkState = BULK_IN;

            if (g_u32Length > 0)
            {
                if(g_u32Length > EP2_MAX_PKT_SIZE)
                    g_u8Size = EP2_MAX_PKT_SIZE;
                else
                    g_u8Size = g_u32Length;

                /* Bulk IN buffer */
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_u32Address, g_u8Size);
                g_u32BytesInStorageBuf = g_u8Size;

                g_u32Address += g_u8Size;
                USBD_SET_EP_BUF_ADDR(EP2, g_u32BulkBuf0);

                MSC_Read(0);
            }
            return;
        }

        case UFI_INQUIRY:
        {
            uint8_t u8PageCode;

            u8PageCode = g_sCBW.au8Data[0];

            g_u8BulkState = BULK_IN;

            /* u8PageCode should be zero */
            if(u8PageCode)
            {
                /* Expecting a STALL after data phase completes with a zero-length or short packet */
                USBD_SET_EP_STALL(EP3);

                DBG_PRINTF("INQUIRY page code = %d", u8PageCode);
            }
            /* Bulk IN buffer */
            USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_au8InquiryID, sizeof(g_au8InquiryID));
            USBD_SET_PAYLOAD_LEN(EP2, 36);

            return;
        }
        case UFI_READ_10:
        case UFI_READ_12:
        {
            /* Check if it is a new transfer */
            if (g_u32Length == 0)
            {
                /* Prepare the data for Bulk IN transfer */
                /* Get LBA address */
                g_u32Address = get_be32(&g_sCBW.au8Data[0]);
                g_u32LbaAddress = g_u32Address * UDC_SECTOR_SIZE;
                g_u32Length = g_sCBW.dCBWDataTransferLength;
                g_u32BytesInStorageBuf = g_u32Length;

                DBG_PRINTF("Read addr=0x%x, len=0x%x\n", g_u32LbaAddress, g_u32Length);

                /* Error check  */
                if ((g_u32LbaAddress > g_u32u32StorageSize) || ((g_u32LbaAddress + g_u32Length) > g_u32u32StorageSize))
                {
                    USBD_MSC_Stall();

                    DBG_PRINTF("Stall ep2, ep3. addr=0x%x, len=0x%x\n", g_u32LbaAddress, g_u32Length);
                    return;
                }

                i = g_u32Length;
                if (i > STORAGE_BUFFER_SIZE)
                    i = STORAGE_BUFFER_SIZE;

                DataFlashRead(g_u32Address * UDC_SECTOR_SIZE, Storage_Block);

                g_u32BytesInStorageBuf = i;
                g_u32LbaAddress += i;
            }
            g_u32Address = STORAGE_DATA_BUF;

            /* Indicate the next packet should be Bulk IN Data packet */
            g_u8BulkState = BULK_IN;

            if (g_u32BytesInStorageBuf > 0)
            {
                /* Set the packet size */
                if(g_u32BytesInStorageBuf > EP2_MAX_PKT_SIZE)
                    g_u8Size = EP2_MAX_PKT_SIZE;
                else
                    g_u8Size = g_u32BytesInStorageBuf;

                /* Prepare the first data packet (DATA1) */
                /* Bulk IN buffer */
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)g_u32Address, g_u8Size);
                g_u32Address += g_u8Size;

                /* kick - start */
                USBD_SET_EP_BUF_ADDR(EP2, g_u32BulkBuf1);
                /* Trigger to send out the data packet */
                USBD_SET_PAYLOAD_LEN(EP2, g_u8Size);
                g_u32Length -= g_u8Size;
                g_u32BytesInStorageBuf -= g_u8Size;
            }

            return;
        }
        case UFI_WRITE_10:
        case UFI_WRITE_12:
        {
            if (g_u32Length == 0)
            {
                g_u32Length = g_sCBW.dCBWDataTransferLength;
                g_u32Address = STORAGE_DATA_BUF;
                g_u32DataFlashStartAddr = get_be32(&g_sCBW.au8Data[0]) * UDC_SECTOR_SIZE;
            }
            DBG_PRINTF("Write 0x%x  0x%x\n", g_u32Address, g_u32Length);

            if (g_u32Length > 0)
            {
                USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
                g_u8BulkState = BULK_OUT;
            }
            return;
        }
        case UFI_MODE_SENSE_6:
        {
            uint32_t u32Data = 0x3;
            g_u8BulkState = BULK_IN;
            USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)u32Data, 4);

            USBD_SET_PAYLOAD_LEN(EP2, 4);
            return;
        }
        }
    }
    else if (g_u8BulkState == BULK_OUT)
    {
        switch(g_sCBW.u8OPCode)
        {
        case UFI_WRITE_10:
        case UFI_WRITE_12:
        {
            MSC_Write();
            return;
        }
        }
    }
}


void MSC_AckCmd(void)
{
    /* Bulk IN */
    if(g_u8BulkState == BULK_CSW)
    {
        /* Prepare to receive the CBW */
        g_u8BulkState = BULK_CBW;

        USBD_SET_EP_BUF_ADDR(EP3, g_u32BulkBuf0);
        USBD_SET_PAYLOAD_LEN(EP3, 31);

        DBG_PRINTF("CSW ack\n");
    }
    else if(g_u8BulkState == BULK_IN)
    {
        uint32_t u32len = g_sCBW.dCBWDataTransferLength;

        g_sCSW.dCSWDataResidue = 0;
        g_sCSW.bCSWStatus = 0;

        switch(g_sCBW.u8OPCode)
        {
        case UFI_READ_FORMAT_CAPACITY:
        case UFI_READ_CAPACITY:
        case UFI_READ_CAPACITY_16:
        case UFI_MODE_SENSE_10:
        case UFI_INQUIRY:
        {
            if((g_u32Length > 0) && (g_sCBW.u8OPCode != UFI_INQUIRY))
            {
                MSC_Read(0);
                return;
            }

            if (u32len > 36)
                g_sCSW.dCSWDataResidue = u32len - 36;

            break;
        }
        case UFI_READ_10:
        case UFI_READ_12:
        {
            if(g_u32Length > 0)
            {
                MSC_Read(1);
                return;
            }
        }
        case UFI_VERIFY_10:
        case UFI_WRITE_10:
        case UFI_WRITE_12:
        {
            int32_t tmp;

            tmp = u32len - STORAGE_BUFFER_SIZE;
            if (tmp < 0)
                tmp = 0;

            g_sCSW.dCSWDataResidue = tmp;
            break;
        }
        case UFI_TEST_UNIT_READY:
        {
            break;
        }
        case UFI_MODE_SENSE_6:
        {
            g_sCSW.dCSWDataResidue = u32len - 4;
            break;
        }
        default:
        {
            /* Unknown command */
            g_sCSW.dCSWDataResidue = u32len;
            g_sCSW.bCSWStatus = 1;   /* return command failed */

            break;
        }
        }

        /* Return the CSW */
        USBD_SET_EP_BUF_ADDR(EP2, g_u32BulkBuf1);

        /* Bulk IN buffer */
        USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + g_u32BulkBuf1), (uint8_t *)&g_sCSW.dCSWSignature, 16);

        g_u8BulkState = BULK_CSW;
        USBD_SET_PAYLOAD_LEN(EP2, 13);

        DBG_PRINTF("Prepare CSW\n");
    }
    else
    {
        /* This should be a DATA phase error */
        USBD_MSC_Stall();

        DBG_PRINTF("Unexpected IN ack\n");
    }
}


void MSC_SetConfig(void)
{
    /* Clear stall status and ready */
    USBD->EP[2].CFGP = 1;
    USBD->EP[3].CFGP = 1;

    /*****************************************************/
    /* EP2 ==> Bulk IN endpoint, address 2 */
    USBD_CONFIG_EP(EP2, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM);
    /* Buffer range for EP2 */
    USBD_SET_EP_BUF_ADDR(EP2, EP2_BUF_BASE);

    /* EP3 ==> Bulk Out endpoint, address 3 */
    USBD_CONFIG_EP(EP3, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM);
    /* Buffer range for EP3 */
    USBD_SET_EP_BUF_ADDR(EP3, EP3_BUF_BASE);

    /* trigger to receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);

    g_u8BulkState = BULK_CBW;

    DBG_PRINTF("Set config\n");
}


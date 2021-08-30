/******************************************************************************//**
 * @file     usbd_user.c
 * @version  V1.00
 * @brief    M031 series USBD driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <string.h>
#include "M031Series_User.h"

#if 0
#define DBG_PRINTF      printf
#else
#define DBG_PRINTF(...)
#endif

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup USBD_Driver USBD Driver
  @{
*/


/** @addtogroup USBD_EXPORTED_FUNCTIONS USBD Exported Functions
  @{
*/

#define EP0_MAX_PKT_SIZE    64

/* Global variables for Control Pipe */
uint8_t g_usbd_SetupPacket[8] = {0};        /*!< Setup packet buffer */

/**
 * @cond HIDDEN_SYMBOLS
 */
static volatile uint8_t *g_usbd_CtrlInPointer = 0;
static volatile uint32_t g_usbd_CtrlInSize = 0;

static volatile uint32_t g_usbd_UsbAddr = 0;
static volatile uint32_t g_usbd_UsbConfig = 0;
static volatile uint32_t g_usbd_UsbAltInterface = 0;
/**
 * @endcond
 */

const S_USBD_INFO_T *g_usbd_sInfo;                  /*!< A pointer for USB information structure */

void MSC_ClassRequest(void);
void MSC_SetConfig(void);

void USBD_Ctrl_Stall(void)
{
    USBD_SET_EP_STALL(EP0);
    USBD_SET_EP_STALL(EP1);
}

void USBD_MemReset(uint8_t *src, uint32_t size)
{
    while(size--) *src++ = 0;
}

/**
  * @brief      This function makes USBD module to be ready to use
  *
  * @param[in]  param           The structure of USBD information.
  * @param[in]  pfnClassReq     USB Class request callback function.
  * @param[in]  pfnSetInterface USB Set Interface request callback function.
  *
  * @return     None
  *
  * @details    This function will enable USB controller, USB PHY transceiver and pull-up resistor of USB_D+ pin. USB PHY will drive SE0 to bus.
  */
void USBD_Open(const S_USBD_INFO_T *param)
{
    g_usbd_sInfo = param;

    /* Initial USB engine */
    USBD->ATTR = 0x7D0;

    /* Force SE0 */
    USBD->SE0 = 1;
}

/**
  * @brief    This function makes USB host to recognize the device
  *
  * @param    None
  *
  * @return   None
  *
  * @details  Enable WAKEUP, FLDET, USB and BUS interrupts. Disable software-disconnect function after 100ms delay with SysTick timer.
  */
// void USBD_Start(void)
// {
//     CLK_SysTickDelay(100000);
//     /* Disable software-disconnect function */
//     USBD_CLR_SE0();

//     /* Clear USB-related interrupts before enable interrupt */
//     USBD_CLR_INT_FLAG(USBD_INT_BUS | USBD_INT_USB | USBD_INT_FLDET | USBD_INT_WAKEUP);

//     /* Enable USB-related interrupts. */
//     USBD_ENABLE_INT(USBD_INT_BUS | USBD_INT_USB | USBD_INT_FLDET | USBD_INT_WAKEUP);
// }

/**
  * @brief      Get the received SETUP packet
  *
  * @param[in]  buf A buffer pointer used to store 8-byte SETUP packet.
  *
  * @return     None
  *
  * @details    Store SETUP packet to a user-specified buffer.
  *
  */
void USBD_GetSetupPacket(uint8_t *buf)
{
    USBD_MemCopy(buf, g_usbd_SetupPacket, 8);
}

/**
  * @brief    Process SETUP packet
  *
  * @param    None
  *
  * @return   None
  *
  * @details  Parse SETUP packet and perform the corresponding action.
  *
  */
void USBD_ProcessSetupPacket(void)
{
    /* Get SETUP packet from USB buffer */
    USBD_MemCopy(g_usbd_SetupPacket, (uint8_t *)USBD_BUF_BASE, 8);

    /* Check the request type */
    switch(g_usbd_SetupPacket[0] & 0x60)
    {
    case REQ_STANDARD:   // Standard
    {
        USBD_StandardRequest();
        break;
    }
    case REQ_CLASS:   // Class
    {
        MSC_ClassRequest();
        break;
    }
//        default:   // reserved
//        {
//            /* Setup error, stall the device */
//            USBD_Ctrl_Stall();
//            break;
//        }
    }
}

/**
  * @brief    Process GetDescriptor request
  *
  * @param    None
  *
  * @return   None
  *
  * @details  Parse GetDescriptor request and perform the corresponding action.
  *
  */
void USBD_GetDescriptor(void)
{
    uint32_t u32Len;

    u32Len = g_usbd_SetupPacket[7];
    u32Len = g_usbd_SetupPacket[6] + (u32Len << 8);

    switch(g_usbd_SetupPacket[3])
    {
    // Get Device Descriptor
    case DESC_DEVICE:
    {
        u32Len = Minimum(u32Len, LEN_DEVICE);
        DBG_PRINTF("Get device desc, %d\n", u32Len);

        USBD_PrepareCtrlIn((uint8_t *)g_usbd_sInfo->gu8DevDesc, u32Len);
        break;
    }
    // Get Configuration Descriptor
    case DESC_CONFIG:
    {
        uint32_t u32TotalLen;

        u32TotalLen = g_usbd_sInfo->gu8ConfigDesc[3];
        u32TotalLen = g_usbd_sInfo->gu8ConfigDesc[2] + (u32TotalLen << 8);

        DBG_PRINTF("Get config desc len %d, acture len %d\n", u32Len, u32TotalLen);
        u32Len = Minimum(u32Len, u32TotalLen);

        DBG_PRINTF("Minimum len %d\n", u32Len);

        USBD_PrepareCtrlIn((uint8_t *)g_usbd_sInfo->gu8ConfigDesc, u32Len);
        break;
    }
    // Get String Descriptor
    case DESC_STRING:
    {
        // Get String Descriptor
        if (g_usbd_SetupPacket[2] < 2)
        {
            u32Len = Minimum(u32Len, g_usbd_sInfo->gu8StringDesc[g_usbd_SetupPacket[2]][0]);

            DBG_PRINTF("Get string desc %d\n", u32Len);

            USBD_PrepareCtrlIn((uint8_t *)g_usbd_sInfo->gu8StringDesc[g_usbd_SetupPacket[2]], u32Len);
        }
        else
        {
            // Not support. Reply STALL.
            USBD_Ctrl_Stall();
            DBG_PRINTF("Unsupported string desc (%d). Stall ctrl pipe.\n", g_usbd_SetupPacket[2]);
        }

        break;
    }
    default:
        // Not support. Reply STALL.
        USBD_Ctrl_Stall();
        DBG_PRINTF("Unsupported get desc type. stall ctrl pipe\n");
        break;
    }
}

/**
  * @brief    Process standard request
  *
  * @param    None
  *
  * @return   None
  *
  * @details  Parse standard request and perform the corresponding action.
  *
  */
void USBD_StandardRequest(void)
{
    uint32_t u32Addr;
    uint32_t u32Len0 = 0;

    /* clear global variables for new request */
    g_usbd_CtrlInPointer = 0;
    g_usbd_CtrlInSize = 0;

    u32Addr = USBD_BUF_BASE +  USBD_GET_EP_BUF_ADDR(EP0);

    switch(g_usbd_SetupPacket[1])
    {
    // Device to host
    case GET_CONFIGURATION:
    {
        // Return current configuration setting
        /* Data stage */
        M8(u32Addr) = g_usbd_UsbConfig;
        USBD_SET_DATA1(EP1);
        USBD_SET_PAYLOAD_LEN(EP1, 0);
        u32Len0 = 1;

        DBG_PRINTF("Get configuration\n");

        break;
    }
    case GET_DESCRIPTOR:
    {
        USBD_GetDescriptor();
        break;
    }
    case GET_INTERFACE:
    {
        // Return current interface setting
        /* Data stage */
        M8(u32Addr) = g_usbd_UsbAltInterface;

        u32Len0 = 1;

        /* Status stage */
        USBD_SET_PAYLOAD_LEN(EP1, EP0_MAX_PKT_SIZE);

        DBG_PRINTF("Get interface\n");

        break;
    }
    case GET_STATUS:
    {
        uint8_t u8data = 0;

        // Device
        if(g_usbd_SetupPacket[0] == 0x80)
            u8data = 1;
        // Interface
        else if(g_usbd_SetupPacket[0] == 0x81)
            u8data = 0;
        // Endpoint
        else if(g_usbd_SetupPacket[0] == 0x82)
        {
            uint32_t u32CfgAddr;

            uint8_t ep = g_usbd_SetupPacket[4] & 0xF;
//                M8(u32Addr) = USBD_GetStall(ep) ? 1 : 0;

            u32CfgAddr = (uint32_t)(ep << 4) + (uint32_t)&USBD->EP[0].CFG;
            u8data = ((*((__IO uint32_t *)(u32CfgAddr))) >> 1) & 0x01;
        }

        M8(u32Addr) = u8data;
        M8(u32Addr + 1) = 0;
        /* Data stage */
        u32Len0 = 2;

        /* Status stage */
        USBD_SET_PAYLOAD_LEN(EP1, EP0_MAX_PKT_SIZE);

        DBG_PRINTF("Get status\n");

        break;
    }
    // Host to device
    case SET_ADDRESS:
    {
        g_usbd_UsbAddr = g_usbd_SetupPacket[2];

        DBG_PRINTF("Set addr to %d\n", g_usbd_UsbAddr);

        // DATA IN for end of setup
        /* Status Stage */

        break;
    }
    case SET_CONFIGURATION:
    {
        g_usbd_UsbConfig = g_usbd_SetupPacket[2];

        MSC_SetConfig();

        // DATA IN for end of setup
        /* Status stage */

        DBG_PRINTF("Set config to %d\n", g_usbd_UsbConfig);
        break;
    }
    case SET_INTERFACE:
    {
        g_usbd_UsbAltInterface = g_usbd_SetupPacket[2];

        /* Status stage */
        break;
    }
    case CLEAR_FEATURE:
    case SET_FEATURE:
    {
        DBG_PRINTF("Clear feature op %d\n", g_usbd_SetupPacket[2]);
        break;
    }
    default:
    {
        /* Setup error, stall the device */
        USBD_Ctrl_Stall();
        return;
    }
    }

    if (!(g_usbd_SetupPacket[0] & 0x80) || u32Len0)
    {
        USBD_SET_DATA1(EP0);
        USBD_SET_PAYLOAD_LEN(EP0, u32Len0);
    }
}

/**
  * @brief      Prepare the first Control IN pipe
  *
  * @param[in]  pu8Buf  The pointer of data sent to USB host.
  * @param[in]  u32Size The IN transfer size.
  *
  * @return     None
  *
  * @details    Prepare data for Control IN transfer.
  *
  */
void USBD_PrepareCtrlIn(uint8_t *pu8Buf, uint32_t u32Size)
{
    DBG_PRINTF("Prepare Ctrl In %d\n", u32Size);

    g_usbd_CtrlInPointer = 0;
    g_usbd_CtrlInSize = 0;

    if (u32Size > EP0_MAX_PKT_SIZE)
    {
        g_usbd_CtrlInPointer = pu8Buf + EP0_MAX_PKT_SIZE;
        g_usbd_CtrlInSize = u32Size - EP0_MAX_PKT_SIZE;
        u32Size = EP0_MAX_PKT_SIZE;
    }

    USBD_SET_DATA1(EP0);
    USBD_MemCopy((uint8_t *)USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0), pu8Buf, u32Size);
    USBD_SET_PAYLOAD_LEN(EP0, u32Size);
}

/**
  * @brief    Repeat Control IN pipe
  *
  * @param    None
  *
  * @return   None
  *
  * @details  This function processes the remained data of Control IN transfer.
  *
  */
void USBD_CtrlIn(void)
{
    DBG_PRINTF("Ctrl In Ack. residue %d\n", g_usbd_CtrlInSize);

    if (g_usbd_CtrlInSize)
    {
        uint32_t u32InSize = g_usbd_CtrlInSize;
        if (u32InSize > EP0_MAX_PKT_SIZE)
            u32InSize = EP0_MAX_PKT_SIZE;

        USBD_MemCopy((uint8_t *)USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0), (uint8_t *)g_usbd_CtrlInPointer, u32InSize);
        USBD_SET_PAYLOAD_LEN(EP0, u32InSize);

        g_usbd_CtrlInSize -= u32InSize;

        if (g_usbd_CtrlInSize)
        {
            g_usbd_CtrlInPointer += u32InSize;
        }
        else
            g_usbd_CtrlInPointer = 0;
    }
    else
    {
        // In ACK for Set address
        if((g_usbd_SetupPacket[0] == 0) && (g_usbd_SetupPacket[1] == 5))
        {
            uint32_t u32Addr = USBD_GET_ADDR();

            if ((u32Addr != g_usbd_UsbAddr) && (u32Addr == 0))
            {
                USBD_SET_ADDR(g_usbd_UsbAddr);
            }
        }

        // No more data for IN token
        USBD_SET_PAYLOAD_LEN(EP1, EP0_MAX_PKT_SIZE);
        DBG_PRINTF("Ctrl In done. Prepare OUT 0\n");
    }
}



/**
  * @brief    Reset software flags
  *
  * @param    None
  *
  * @return   None
  *
  * @details  This function resets all variables for protocol and resets USB device address to 0.
  *
  */
void USBD_SwReset(void)
{
    // Reset all variables for protocol
    g_usbd_CtrlInPointer = 0;
    g_usbd_CtrlInSize = 0;

    USBD_MemReset(g_usbd_SetupPacket, 8);

    // Reset USB device address
    USBD->FADDR = 0;
}


/*@}*/ /* end of group USBD_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group USBD_Driver */

/*@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

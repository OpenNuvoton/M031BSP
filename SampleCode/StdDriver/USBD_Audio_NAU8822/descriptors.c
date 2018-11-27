/****************************************************************************//**
 * @file     descriptors.c
 * @version  V0.10
 * @brief    NuMicro series USBD descriptor file
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
/*!<Includes */
#include "NuMicro.h"
#include "usbd_audio.h"

/*----------------------------------------------------------------------------*/
/*!<USB Device Descriptor */
uint8_t gu8DeviceDescriptor[] =
{
    LEN_DEVICE,         /* bLength */
    DESC_DEVICE,        /* bDescriptorType */
#ifdef SUPPORT_LPM
    0x01, 0x02,         /* bcdUSB => 0x0201 to support LPM */
#else
    0x10, 0x01,         /* bcdUSB */
#endif
    0x00,               /* bDeviceClass */
    0x00,               /* bDeviceSubClass */
    0x00,               /* bDeviceProtocol */
    EP0_MAX_PKT_SIZE,   /* bMaxPacketSize0 */
    /* idVendor */
    USBD_VID & 0x00FF,
    (USBD_VID & 0xFF00) >> 8,
    /* idProduct */
    USBD_PID & 0x00FF,
    (USBD_PID & 0xFF00) >> 8,
    0x00, 0x01,         /* bcdDevice */
    0x01,               /* iManufacture */
    0x02,               /* iProduct */
    0x00,               /* iSerialNumber - no serial */
    0x01                /* bNumConfigurations */
};

/*!<USB Configure Descriptor */
uint8_t gu8ConfigDescriptor[] =
{
    LEN_CONFIG,         /* bLength */
    DESC_CONFIG,        /* bDescriptorType */
    0xC2, 0x00,         /* wTotalLength */
    0x03,               /* bNumInterfaces */
    0x01,               /* bConfigurationValue */
    0x00,               /* iConfiguration */
    0x80,               /* bmAttributes */
    0x20,               /* Max power */

    /* Standard AC inteface */
    0x09,               /* bLength */
    0x04,               /* bDescriptorType */
    0x00,               /* bInterfaceNumber */
    0x00,               /* bAlternateSetting */
    0x00,               /* bNumEndpoints */
    0x01,               /* bInterfaceClass:AUDIO */
    0x01,               /* bInterfaceSubClass:AUDIOCONTROL */
    0x00,               /* bInterfaceProtocol */
    0x00,               /* iInterface */

    /* Class-spec AC interface descriptor */
    0x0A,               /* bLength */
    0x24,               /* bDescriptorType:CS_INTERFACE */
    0x01,               /* bDescriptorSubType:HEADER */
    0x00, 0x01,         /* bcdADC:1.0 */
    0x48, 0x00,         /* wTotalLength */
    0x02,               /* bInCollection */
    0x01,               /* baInterfaceNr(1) */
    0x02,               /* baInterfaceNr(n) */

    /* TID 1: Input for usb streaming */
    0x0C,               /* bLength */
    0x24,               /* bDescriptorType:CS_INTERFACE */
    0x02,               /* bDescriptorSubType:INPUT_TERMINAL */
    0x01,               /* bTerminalID */
    0x01, 0x01,         /* wTerminalType: 0x0101 usb streaming */
    0x00,               /* bAssocTerminal */
    PLAY_CHANNELS,      /* bNrChannels */
    PLAY_CH_CFG, 0x00,  /* wChannelConfig */
    0x00,               /* iChannelNames */
    0x00,               /* iTerminal */

    /* UNIT ID 5: Feature Unit */
    0x0A,               /* bLength */
    0x24,               /* bDescriptorType */
    0x06,               /* bDescriptorSubType */
    REC_FEATURE_UNITID, /* bUnitID */
    0x04,               /* bSourceID */
    0x01,               /* bControlSize */
    0x01,               /* bmaControls(0) */
    0x02,               /* bmaControls(0) */
    0x02,               /* bmaControls(0) */
    0x00,               /* iFeature */

    /* TID 2: Output Terminal for usb streaming */
    0x09,               /* bLength */
    0x24,               /* bDescriptorType:CS_INTERFACE */
    0x03,               /* bDescriptorSubType:OUTPUT_TERMINAL */
    0x02,               /* bTerminalID */
    0x01, 0x01,         /* wTerminalType: 0x0101 usb streaming */
    0x00,               /* bAssocTerminal */
    REC_FEATURE_UNITID, /* bSourceID */
    0x00,               /* iTerminal */

    /* UNIT ID 6: Feature Unit */
    0x0A,               /* bLength */
    0x24,               /* bDescriptorType */
    0x06,               /* bDescriptorSubType */
    PLAY_FEATURE_UNITID,/* bUnitID */
    0x01,               /* bSourceID */
    0x01,               /* bControlSize */
    0x01,               /* bmaControls(0) */
    0x02,               /* bmaControls(0) */
    0x02,               /* bmaControls(0) */
    0x00,               /* iFeature */

    /* TID 3: Output for speaker */
    0x09,               /* bLength*/
    0x24,               /* bDescriptorType:CS_INTERFACE*/
    0x03,               /* bDescriptorSubType:OUTPUT_TERMINAL*/
    0x03,               /* bTerminalID*/
    0x01, 0x03,         /* wTerminalType: 0x0301 speaker*/
    0x00,               /* bAssocTerminal*/
    0x06,               /* bSourceID*/
    0x00,               /* iTerminal*/

    /* TID 4: Input Terminal for microphone */
    0x0C,               /* bLength */
    0x24,               /* bDescriptorType:CS_INTERFACE */
    0x02,               /* bDescriptorSubType:INPUT_TERMINAL*/
    0x04,               /* bTerminalID*/
    0x01, 0x02,         /* wTerminalType: 0x0201 microphone*/
    0x00,               /* bAssocTerminal*/
    REC_CHANNELS,       /* bNrChannels*/
    REC_CH_CFG, 0x00,   /* wChannelConfig*/
    0x00,               /* iChannelNames*/
    0x00,               /* iTerminal*/

    /* Standard AS interface 1, alternate 0 */
    0x09,               /* bLength */
    0x04,               /* bDescriptorType */
    0x01,               /* bInterfaceNumber */
    0x00,               /* bAlternateSetting */
    0x00,               /* bNumEndpoints */
    0x01,               /* bInterfaceClass:AUDIO */
    0x02,               /* bInterfaceSubClass:AUDIOSTREAMING */
    0x00,               /* bInterfaceProtocol */
    0x00,               /* iInterface */

    /* Standard AS interface 1, alternate 1 */
    0x09,               /* bLength */
    0x04,               /* bDescriptorType */
    0x01,               /* bInterfaceNumber */
    0x01,               /* bAlternateSetting */
    0x01,               /* bNumEndpoints */
    0x01,               /* bInterfaceClass:AUDIO */
    0x02,               /* bInterfaceSubClass:AUDIOSTREAMING */
    0x00,               /* bInterfaceProtocol */
    0x00,               /* iInterface */

    /* Class-spec AS interface, this interface's endpoint connect to TID 0x02 */
    0x07,               /* bLength */
    0x24,               /* bDescriptorType:CS_INTERFACE */
    0x01,               /* bDescriptorSubType:AS_GENERAL */
    0x02,               /* bTernimalLink */
    0x01,               /* bDelay */
    0x01, 0x00,         /* wFormatTag:0x0001 PCM */

    /* Type I format type Descriptor */
    0x0B,               /* bLength */
    0x24,               /* bDescriptorType:CS_INTERFACE */
    0x02,               /* bDescriptorSubType:FORMAT_TYPE */
    0x01,               /* bFormatType:FORMAT_TYPE_I */
    REC_CHANNELS,       /* bNrChannels */
    0x02,               /* bSubFrameSize */
    0x10,               /* bBitResolution */
    0x01,               /* bSamFreqType : 0 continuous; 1 discrete */
    REC_RATE_LO,
    REC_RATE_MD,
    REC_RATE_HI,        /* Sample Frequency */

    /* Standard AS ISO Audio Data Endpoint */
    0x09,                              /* bLength */
    0x05,                              /* bDescriptorType */
    ISO_IN_EP_NUM | EP_INPUT,          /* bEndpointAddress */
    0x0d,                              /* bmAttributes */
    (EP2_MAX_PKT_SIZE & 0xFF), 
    ((EP2_MAX_PKT_SIZE >> 8) & 0xFF),  /* wMaxPacketSize*/
    0x01,                              /* bInterval*/
    0x00,                              /* bRefresh*/
    0x00,                              /* bSynchAddress*/

    /* Class-spec AS ISO Audio Data endpoint Descriptor */
    0x07,               /* bLength */
    0x25,               /* bDescriptorType:CS_ENDPOINT */
    0x01,               /* bDescriptorSubType:EP_GENERAL */
    0x00,               /* bmAttributes */
    0x00,               /* bLockDelayUnits */
    0x00, 0x00,         /* wLockDelay */

    /* Standard AS interface 2, alternate 0 */
    0x09,               /* bLength */
    0x04,               /* bDescriptorType */
    0x02,               /* bInterfaceNumber */
    0x00,               /* bAlternateSetting */
    0x00,               /* bNumEndpoints */
    0x01,               /* bInterfaceClass:AUDIO */
    0x02,               /* bInterfaceSubClass:AUDIOSTREAMING */
    0x00,               /* bInterfaceProtocol */
    0x00,               /* iInterface */

    /* Standard AS interface 2, alternate 1 */
    0x09,               /* bLength */
    0x04,               /* bDescriptorType */
    0x02,               /* bInterfaceNumber */
    0x01,               /* bAlternateSetting */
    0x01,               /* bNumEndpoints */
    0x01,               /* bInterfaceClass:AUDIO */
    0x02,               /* bInterfaceSubClass:AUDIOSTREAMING */
    0x00,               /* bInterfaceProtocol */
    0x00,               /* iInterface */

    /* Class-spec AS inf this interface's endpoint connect to TID 0x01 */
    0x07,               /* bLength */
    0x24,               /* bDescriptorType:CS_INTERFACE */
    0x01,               /* bDescriptorSubType:AS_GENERAL */
    0x01,               /* bTernimalLink */
    0x01,               /* bDelay */
    0x01, 0x00,         /* wFormatTag:0x0001 PCM */

    /* Type I format type Descriptor */
    0x0B,               /* bLength */
    0x24,               /* bDescriptorType:CS_INTERFACE */
    0x02,               /* bDescriptorSubType:FORMAT_TYPE */
    0x01,               /* bFormatType:FORMAT_TYPE_I */
    PLAY_CHANNELS,      /* bNrChannels */
    0x02,               /* bSubFrameSize */
    0x10,               /* bBitResolution */
    0x01,               /* bSamFreqType : 0 continuous; 1 discrete */
    PLAY_RATE_LO,
    PLAY_RATE_MD,
    PLAY_RATE_HI,       /* Sample Frequency */

    /* Standard AS ISO Audio Data Endpoint, output, addtess 2, Max 0x40 */
    0x09,                             /* bLength */
    0x05,                             /* bDescriptorType */
    ISO_OUT_EP_NUM | EP_OUTPUT,       /* bEndpointAddress */
    0x0d,                             /* bmAttributes */
    (EP3_MAX_PKT_SIZE & 0xff),
    ((EP3_MAX_PKT_SIZE >> 8) & 0xff), /* wMaxPacketSize */
    0x01,                             /* bInterval */
    0x00,                             /* bRefresh */
    0x00,                             /* bSynchAddress */

    /* Class-spec AS ISO Audio Data endpoint Descriptor */
    0x07,               /* bLength */
    0x25,               /* bDescriptorType:CS_ENDPOINT */
    0x01,               /* bDescriptorSubType:EP_GENERAL */
    0x80,               /* bmAttributes */
    0x00,               /* bLockDelayUnits */
    0x00, 0x00,         /* wLockDelay */
};

/*!<USB BOS Descriptor */
uint8_t gu8BosDescriptor[] = {
    LEN_BOS,                         /* bLength         */
    DESC_BOS,                        /* bDescriptorType */
    ((LEN_BOS + LEN_BOSCAP) & 0xFF), /* wTotalLength    */
    ((LEN_BOS + LEN_BOSCAP) >> 8),   /* wTotalLength    */
    0x01,                            /* bNumDevcieCaps  */
    LEN_BOSCAP,                      /* bLength         */
    DESC_CAPABILITY,                 /* bDescriptorType */
    CAP_USB20_EXT,                   /* bDevCapabilityType, 0x02 is USB 2.0 Extension */
    0x06, 0x04, 0x00, 0x00  /* bmAttributs, 32 bits     */
    /* bit 0 : Reserved. Must 0.                                         */
    /* bit 1 : 1 to support LPM.                                         */
    /* bit 2 : 1 to support BSL & Alternat HIRD                          */
    /* bit 3 : 1 to recommend Baseline BESL                              */
    /* bit 4 : 1 to recommand Deep BESL                                  */
    /* bit 11:8 : Recommend Baseline BESL value. Ignore by bit3 is zero. */
    /* bit 15:12 : Recommend Deep BESL value. Ignore by bit4 is zero.    */
    /* bit 31:16 : Reserved. Must 0.                                     */
};


/*!<USB Language String Descriptor */
uint8_t gu8StringLang[4] =
{
    4,              /* bLength */
    DESC_STRING,    /* bDescriptorType */
    0x09, 0x04
};

/*!<USB Vendor String Descriptor */
uint8_t gu8VendorStringDesc[] =
{
    16,
    DESC_STRING,
    'N', 0, 'u', 0, 'v', 0, 'o', 0, 't', 0, 'o', 0, 'n', 0
};

/*!<USB Product String Descriptor */
uint8_t gu8ProductStringDesc[] =
{
    20,
    DESC_STRING,
    'U', 0, 'S', 0, 'B', 0, ' ', 0, 'A', 0, 'u', 0, 'd', 0, 'i', 0, 'o', 0
};


uint8_t gu8StringSerial[26] =
{
    26,             /* bLength */
    DESC_STRING,    /* bDescriptorType */
    'A', 0, '0', 0, '2', 0, '0', 0, '0', 0, '8', 0, '0', 0, '4', 0, '0', 0, '1', 0, '1', 0, '4', 0
};

uint8_t *gpu8UsbString[4] =
{
    gu8StringLang,
    gu8VendorStringDesc,
    gu8ProductStringDesc,
    gu8StringSerial
};

const S_USBD_INFO_T gsInfo = {
    gu8DeviceDescriptor,
    gu8ConfigDescriptor,
    gpu8UsbString,
    NULL,
    gu8BosDescriptor,
    NULL,
    NULL
};


/****************************************************************************//**
 * @file     descriptors.c
 * @version  V0.10
 * @brief    NuMicro series USBD descriptor file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
/*!<Includes */
#include "NuMicro.h"
#include "usbd_audio.h"

/*
  *Microphone - Interface alternate 1
  +-----------+------------+----------------+------------------+
  | Alternate | Channel(s) | Bit Resolution | Sampling Rate(s) |
  +-----------+------------+----------------+------------------+
  |     1     |      2     |     16 bits    |   32kHz, 48kHz   |
  +-----------+------------+----------------+------------------+

  Note:
  1.If you want to add / remove sampling rate to certain alternate for Microphone interface,
    please modify Audio Streaming Format Type Descriptor (bLength, bSamFreqType, tSamFreq fields)
    and the Total Length field of Configuration Descriptor.
    For example,
      Add 48kHz to Microphone Interface alternate 1 (16 bit resolution) from Speaker descriptor with HID

      1.Modify Audio Streaming Format Type Descriptor for Microphone Interface alternate 1

         Audio Streaming Format Type Descriptor
         +--------------------+-------------------------------+-------------------------------+
         | *bLength           |        Original Value         |         Modified Value        |
         +--------------------+-------------------------------+-------------------------------+
         | bLength            |              0x0E             |         0x11(+3 Bytes)        |
         +--------------------+-------------------------------+-------------------------------+
         | bDescriptorType    |              0x24             |              0x24             |
         +--------------------+-------------------------------+-------------------------------+
         | bDescriptorSubType |              0x02             |              0x02             |
         +--------------------+-------------------------------+-------------------------------+
         | bFormatType        |              0x01             |              0x01             |
         +--------------------+-------------------------------+-------------------------------+
         | bNrChannels        |              0x02             |              0x02             |
         +--------------------+-------------------------------+-------------------------------+
         | bSubFrameSize      |              0x02             |              0x02             |
         +--------------------+-------------------------------+-------------------------------+
         | bBitResolution     |              0x10             |              0x10             |
         +--------------------+-------------------------------+-------------------------------+
         | *bSamFreqType      |              0x02             |       0x03(+1 Frequency)      |
         +--------------------+-------------------------------+-------------------------------+
         | *tSamFreq          |  AUDIO_RATE_48K & 0xFF        |  AUDIO_RATE_48K & 0xFF        |
         |                    | (AUDIO_RATE_48K >>  8) & 0xFF | (AUDIO_RATE_48K >>  8) & 0xFF | 
         |                    | (AUDIO_RATE_48K >> 16) & 0xFF | (AUDIO_RATE_48K >> 16) & 0xFF |
         |                    |  AUDIO_RATE_16K & 0xFF        |  AUDIO_RATE_32K & 0xFF        |
         |                    | (AUDIO_RATE_16K >>  8) & 0xFF | (AUDIO_RATE_32K >>  8) & 0xFF | 
         |                    | (AUDIO_RATE_16K >> 16) & 0xFF | (AUDIO_RATE_32K >> 16) & 0xFF |
         |                    |                               |  AUDIO_RATE_16K & 0xFF        |
         |                    |                               | (AUDIO_RATE_16K >>  8) & 0xFF | 
         |                    |                               | (AUDIO_RATE_16K >> 16) & 0xFF |
         +--------------------+-------------------------------+-------------------------------+

      2.Modify the Total Length field of Configuration Descriptor to 0x86

         0x86(Original Total Length) - 0x03(The decrease Length of Audio Streaming Format Type Descriptor) = 0x89

  2.If you want to change the support function of audio control, please modify the bmaControls field of
    Audio Control Feature Unit Descriptor for Microphone
      A bit set to 1 indicates that the mentioned Control is supported
         0:
         D0: Mute
         D1: Volume
         D2: Bass
         D3: Mid
         D4: Treble
         D5: Graphic Equalizer
         D6: Automatic Gain
         D7: Delay
         D8: Bass Boost
         D9: Loudness
         D10..(n*8-1): Reserved
  3.If you want to change the polling interal of HID Endpoint, please modify the bInterval field of Endpoint Descriptor for HID.
*/

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
    0x03,               /* iSerialNumber - no serial */
    0x01                /* bNumConfigurations */
};

#ifdef __HID__
#ifdef __JOYSTICK__
uint8_t HID_DeviceReportDescriptor[] =
{
    0x05, 0x01,        /* Usage Page(Generic Desktop Controls) */
    0x09, 0x04,        /* Usage(Joystick) */
    0xA1, 0x01,        /* Collection(Application)  */
    0xA1, 0x02,        /* Collection(Logical)  */
    0x75, 0x08,        /* Report Size(0x8) */
    0x95, 0x05,        /* Report Count(0x5) */
    0x15, 0x00,        /* Logical Minimum(0x0) */
    0x26, 0xFF, 0x00,  /* Logical Maximum(0xFF) */
    0x35, 0x00,        /* Physical Minimum(0x0) */
    0x46, 0xFF, 0x00,  /* Physical Maximum(0xFF) */
    0x09, 0x30,        /* Usage(X) */
    0x09, 0x31,        /* Usage(Y) */
    0x09, 0x32,        /* Usage(Z) */
    0x09, 0x32,        /* Usage(Z) */
    0x09, 0x35,        /* Usage(Rz) */
    0x81, 0x02,        /* Input(Data, Variable, Absolute, No Wrap, Linear, Preferred State, No Null Position, Bit Field) */
    0x75, 0x04,        /* Report Size(0x4) */
    0x95, 0x01,        /* Report Count(0x1) */
    0x25, 0x07,        /* Logical Maximum(0x7) */
    0x46, 0x3B, 0x01,  /* Physical Maximum(0x13B) */
    0x65, 0x14,        /* Unit(0x14) */
    0x09, 0x39,        /* Usage(Hat switch) */
    0x81, 0x42,        /* Input(Data, Variable, Absolute, No Wrap, Linear, Preferred State, Null State, Bit Field) */
    0x65, 0x00,        /* Unit(0x0) */
    0x75, 0x01,        /* Report Size(0x1) */
    0x95, 0x0C,        /* Report Count(0xC) */  
    0x25, 0x01,        /* Logical Maximum(0x1) */
    0x45, 0x01,        /* Physical Maximum(0x1) */
    0x05, 0x09,        /* Usage Page(Button) */
    0x19, 0x01,        /* Usage Minimum(0x1) */
    0x29, 0x0C,        /* Usage Maximum(0xC) */
    0x81, 0x02,        /* Input(Data, Variable, Absolute, No Wrap, Linear, Preferred State, No Null Position, Bit Field) */  
    0x06, 0x00, 0xFF,  /* Usage Page(Undefined) */
    0x75, 0x01,        /* Report Size(0x1) */
    0x95, 0x08,        /* Report Count(0x8) */
    0x25, 0x01,        /* Logical Maximum(0x1) */
    0x45, 0x01,        /* Physical Maximum(0x1) */
    0x09, 0x01,        /* Usage(1) */
    0x81, 0x02,        /* Input(Data, Variable, Absolute, No Wrap, Linear, Preferred State, No Null Position, Bit Field) */
    0xC0,              /* End Collection */
    0xC0,              /* End Collection */
};

#elif defined __MEDIAKEY__
uint8_t HID_DeviceReportDescriptor[] =
{
    0x05, 0x0C,        /* Usage Page (Consumer) */
    0x09, 0x01,        /* Usage(Consumer Control) */
    0xA1, 0x01,        /* Collection(Application) */
    0x15, 0x00,        /* Logical Minimum(0x0) */
    0x25, 0x01,        /* Logical Maximum(0x1) */
    0x09, 0xE2,        /* Usage(Mute) */
    0x09, 0xE9,        /* Usage(Volume Increment) */
    0x09, 0xEA,        /* Usage(Volume Decrement) */
    0x75, 0x01,        /* Report Size(0x1) */
    0x95, 0x03,        /* Report Count(0x3) */
    0x81, 0x02,        /* Input(Data, Variable, Absolute, No Wrap, Linear, Preferred State, No Null Position, Bit Field) */
    0x75, 0x01,        /* Report Size(0x1) */
    0x95, 0x05,        /* Report Count(0x5) */
    0x81, 0x03,        /* Input(Constant, Variable, Absolute) */
    0x09, 0xB0,        /* Usage(Play) */
    0x09, 0xB7,        /* Usage(Stop) */
    0x09, 0xCD,        /* Usage(Play/Pause) */
    0x09, 0xB5,        /* Usage(Scan Next Track) */
    0x09, 0xB6,        /* Usage(Scan Previous Track) */
    0x09, 0xB2,        /* Usage(Record) */
    0x09, 0xB4,        /* Usage(Rewind) */
    0x09, 0xB3,        /* Usage(Fast Forward) */
    0x75, 0x01,        /* Report Size(0x1) */
    0x95, 0x08,        /* Report Count(0x8) */
    0x81, 0x02,        /* Input(Data, Variable, Absolute, No Wrap, Linear, Preferred State, No Null Position, Bit Field) */
    0x09, 0x00,        /* Usage(Undefined) */
    0x75, 0x08,        /* Report Size(0x8) */
    0x95, 0x06,        /* Report Count(0x6) */
    0x81, 0x02,        /* Input(Data, Variable, Absolute, No Wrap, Linear, Preferred State, No Null Position, Bit Field) */
    0x09, 0x00,        /* Usage(Undefined) */
    0x75, 0x08,        /* Report Size(0x8) */
    0x95, 0x08,        /* Report Count(0x8) */
    0x91, 0x00,        /* Output(Data, Array, Absolute, No Wrap, Linear, Preferred State, No Null Position, Non VolatileBit Field) */
    0xC0
};
#endif
#define HID_REPORT_DESC_SIZE \
    sizeof(HID_DeviceReportDescriptor) / sizeof(HID_DeviceReportDescriptor[0])
#define HID_REPORT_DESCRIPTOR_SIZE   HID_REPORT_DESC_SIZE
#endif

/*!<USB Configure Descriptor */
uint8_t gu8ConfigDescriptor[] =
{
    /* Configuration Descriptor */
    LEN_CONFIG,         /* bLength */
    DESC_CONFIG,        /* bDescriptorType */
#ifdef __HID__
    0x86, 0x00,         /* wTotalLength
                           Descriptor without HID                      (0x6D)
                           HID Descriptor
                             Interface Descriptor                      (0x09)
                             HID Descriptor                            (0x09)
                             Endpoint Descriptor                       (0x07)

                           0x6D + 0x09 + 0x09 + 0x07 = 0x86
                        */
    0x03,               /* bNumInterfaces - Interface 0, Interface 1 (Microphone), Interface 2 (HID) */
#else
    0x6D, 0x00,         /* wTotalLength */
    /*
       Configuration Descriptor                    (0x09)
       Interface Descriptor (Audio Class)          (0x09)
       Audio Control Interface Header Descriptor   (0x09)
       Microphone - Audio Control
         Audio Control Input Terminal Descriptor   (0x0C)
         Audio Control Feature Unit Descriptor     (0x08)
         Audio Control Output Terminal Descriptor  (0x09)
       Microphone - Interface alternate 0
         Standard AS interface                     (0x09)
       Microphone - Interface alternate 1
         Standard AS interface                                         (0x09)
         Audio Streaming Class Specific Interface Descriptor           (0x07)
         Audio Streaming Format Type Descriptor                        (0x0E)
         Endpoint Descriptor                                           (0x07)
         Audio Streaming Class Specific Audio Data Endpoint Descriptor (0x07)
         *Each Interface alternate Summary                             (0x2C)

       0x09 + 0x09 + 0x9 + (0x0C + 0x08 + 0x09) +
       0x09 + 0x2C = 0x6D
    */
    0x02,               /* bNumInterfaces - Interface 0, Interface 1 (Microphone) */
#endif
    0x01,               /* bConfigurationValue */
    0x00,               /* iConfiguration */
    0x80,               /* bmAttributes */
    0x20,               /* Max power */

    /* Interface Descriptor (Audio Class) */
    0x09,               /* bLength */
    0x04,               /* bDescriptorType */
    0x00,               /* bInterfaceNumber */
    0x00,               /* bAlternateSetting */
    0x00,               /* bNumEndpoints */
    0x01,               /* bInterfaceClass:AUDIO */
    0x01,               /* bInterfaceSubClass:AUDIOCONTROL */
    0x00,               /* bInterfaceProtocol */
    0x00,               /* iInterface */

    /* Audio Control Interface Header Descriptor */
    0x09,               /* bLength */
    0x24,               /* bDescriptorType:CS_INTERFACE */
    0x01,               /* bDescriptorSubType:HEADER */
    0x00, 0x01,         /* bcdADC:1.0 */
    0x26, 0x00,         /* wTotalLength
                           Audio Control Interface Header Descriptor   (0x09)
                           Microphone - Audio Control
                             Audio Control Input Terminal Descriptor   (0x0C)
                             Audio Control Feature Unit Descriptor     (0x08)
                             Audio Control Output Terminal Descriptor  (0x09)

                             0x09 + (0x0C + 0x08 + 0x09) = 0x26
                        */
    0x01,               /* bInCollection */
    0x01,               /* baInterfaceNr(1) - Microphone */

    /* Audio Control Input Terminal Descriptor (Terminal ID 4) */
    0x0C,               /* bLength */
    0x24,               /* bDescriptorType:CS_INTERFACE */
    0x02,               /* bDescriptorSubType:INPUT_TERMINAL*/
    0x04,               /* bTerminalID*/
    0x01,0x02,          /* wTerminalType: 0x0201 microphone*/
    0x00,               /* bAssocTerminal*/
    REC_CHANNELS,       /* bNrChannels : a number that specifies how many logical audio channels are present in the cluster */
    REC_CH_CFG, 0x00,   /* wChannelConfig: a bit field that indicates which spatial locations are present in the cluster.
                           The bit allocations are as follows:
                             D0: Left Front (L)
                             D1: Right Front (R)
                             D2: Center Front (C)
                             D3: Low Frequency Enhancement (LFE)
                             D4: Left Surround (LS)
                             D5: Right Surround (RS)
                             D6: Left of Center (LC)
                             D7: Right of Center (RC)
                             D8: Surround (S)
                             D9: Side Left (SL)
                             D10: Side Right (SR)
                             D11: Top (T)
                             D15..12: Reserved
                        */
    0x00,               /* iChannelNames*/
    0x00,               /* iTerminal*/

    /* Audio Control Feature Unit Descriptor - Microphone (UNIT ID 5 - Source ID 4) */
    0x08,               /* bLength */
    0x24,               /* bDescriptorType */
    0x06,               /* bDescriptorSubType */
    REC_FEATURE_UNITID, /* bUnitID */
    0x04,               /* bSourceID */
    0x01,               /* bControlSize - Size, in bytes, of the bmControls field: n */
    0x03,               /* bmaControls(0) */
    /* A bit set to 1 indicates that the mentioned
       Control is supported for master channel
       0:
       D0: Mute
       D1: Volume
       D2: Bass
       D3: Mid
       D4: Treble
       D5: Graphic Equalizer
       D6: Automatic Gain
       D7: Delay
       D8: Bass Boost
       D9: Loudness
       D10..(n*8-1): Reserved
    */
    0x00,               /* iFeature */

    /* Audio Control Output Terminal Descriptor - Microphone (Terminal ID 2 - Source ID 5) */
    0x09,               /* bLength */
    0x24,               /* bDescriptorType:CS_INTERFACE */
    0x03,               /* bDescriptorSubType:OUTPUT_TERMINAL */
    0x02,               /* bTerminalID */
    0x01, 0x01,         /* wTerminalType: 0x0101 usb streaming */
    0x00,               /* bAssocTerminal */
    REC_FEATURE_UNITID, /* bSourceID */
    0x00,               /* iTerminal */

    /* Interface Descriptor - Interface 1, alternate 0 */
    0x09,               /* bLength */
    0x04,               /* bDescriptorType */
    0x01,               /* bInterfaceNumber */
    0x00,               /* bAlternateSetting */
    0x00,               /* bNumEndpoints */
    0x01,               /* bInterfaceClass:AUDIO */
    0x02,               /* bInterfaceSubClass:AUDIOSTREAMING */
    0x00,               /* bInterfaceProtocol */
    0x00,               /* iInterface */

    /* Interface Descriptor - Interface 1, alternate 1 */
    0x09,               /* bLength */
    0x04,               /* bDescriptorType */
    0x01,               /* bInterfaceNumber */
    0x01,               /* bAlternateSetting */
    0x01,               /* bNumEndpoints */
    0x01,               /* bInterfaceClass:AUDIO */
    0x02,               /* bInterfaceSubClass:AUDIOSTREAMING */
    0x00,               /* bInterfaceProtocol */
    0x00,               /* iInterface */

    /* Audio Streaming Class Specific Interface Descriptor (this interface's endpoint connect to Terminal ID 0x02) */
    0x07,               /* bLength */
    0x24,               /* bDescriptorType:CS_INTERFACE */
    0x01,               /* bDescriptorSubType:AS_GENERAL */
    0x02,               /* bTernimalLink (Microphone) */
    0x01,               /* bDelay */
    0x01, 0x00,         /* wFormatTag:0x0001 PCM */

    /* Audio Streaming Format Type Descriptor */
    0x0E,               /* bLength */
    0x24,               /* bDescriptorType:CS_INTERFACE */
    0x02,               /* bDescriptorSubType:FORMAT_TYPE */
    0x01,               /* bFormatType:FORMAT_TYPE_I */
    /* Standard AS interface 1, alternate 1 */
    REC_CHANNELS,       /* bNrChannels    :  2 Channels */
    0x02,               /* bSubFrameSize  :  2 bytes per sample */
    0x10,               /* bBitResolution : 16 bits  per sample */
    0x02,               /* bSamFreqType :
                           0 Continuous sampling frequency
                           1 The number of discrete sampling frequencies */
    /* bSamFreqType  */
    (AUDIO_RATE_16K & 0xFF),
    ((AUDIO_RATE_16K >> 8) & 0xFF),
    ((AUDIO_RATE_16K >> 16) & 0xFF),

    (AUDIO_RATE_48K & 0xFF),
    ((AUDIO_RATE_48K >> 8) & 0xFF),
    ((AUDIO_RATE_48K >> 16) & 0xFF),

    /* Endpoint Descriptor (ISO IN Audio Data Endpoint - alternate 1) */
    0x07,                             /* bLength */
    0x05,                             /* bDescriptorType */
    ISO_IN_EP_NUM | EP_INPUT,         /* bEndpointAddress */
    0x05,                             /* bmAttributes */
    (EP2_MAX_PKT_SIZE & 0xFF), 
    ((EP2_MAX_PKT_SIZE >> 8) & 0xFF),  /* wMaxPacketSize*/
    0x01,                             /* bInterval*/

    /* Audio Streaming Class Specific Audio Data Endpoint Descriptor */
    0x07,               /* bLength */
    0x25,               /* bDescriptorType:CS_ENDPOINT */
    0x01,               /* bDescriptorSubType:EP_GENERAL */
    0x01,               /* bmAttributes, Bit 7: MaxPacketsOnly, Bit 0: Sampling Frequency */
    0x00,               /* bLockDelayUnits */
    0x00, 0x00,         /* wLockDelay */

#ifdef __HID__
    /* Interface Descriptor for HID */
    LEN_INTERFACE,      /* bLength */
    DESC_INTERFACE,     /* bDescriptorType */
    0x02,               /* bInterfaceNumber */
    0x00,               /* bAlternateSetting */
    0x01,               /* bNumEndpoints */
    0x03,               /* bInterfaceClass */
    0x00,               /* bInterfaceSubClass */
    0x00,               /* bInterfaceProtocol */
    0x00,               /* iInterface */

    /* HID Descriptor */
    LEN_HID,            /* Size of this descriptor in UINT8s */
    DESC_HID,           /* HID descriptor type. */
    0x10, 0x01,         /* HID Class Spec. release number.*/
    0x00,               /* H/W target country. */
    0x01,               /* Number of HID class descriptors to follow. */
    DESC_HID_RPT,       /* Dscriptor type. */

    /* Total length of report descriptor */
    HID_REPORT_DESCRIPTOR_SIZE & 0x00FF,
    (HID_REPORT_DESCRIPTOR_SIZE & 0xFF00) >> 8,

    /* Endpoint Descriptor (Interrupt IN Endpoint) */
    LEN_ENDPOINT,                     /* bLength */
    DESC_ENDPOINT,                    /* bDescriptorType */
    (HID_IN_EP_NUM | EP_INPUT),       /* bEndpointAddress */
    EP_INT,                           /* bmAttributes */
    /* wMaxPacketSize */
    EP4_MAX_PKT_SIZE & 0x00FF,
    (EP4_MAX_PKT_SIZE & 0xFF00) >> 8,
    10                                /* bInterval */
#endif
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
#ifdef __HID__
    60,
    DESC_STRING,
    'U', 0, 'A', 0, 'C', 0, ' ', 0, '1', 0, '.', 0, '0', 0, ' ', 0,
    'M', 0, 'i', 0, 'c', 0, 'r', 0, 'o', 0, 'p', 0, 'h', 0, 'o', 0, 'n', 0, 'e', 0,
    ' ', 0, '&', 0,
#ifdef __JOYSTICK__
    ' ', 0, 'J', 0, 'o', 0, 'y', 0, 's', 0, 't', 0, 'i', 0, 'c', 0, 'k', 0,
#elif defined __MEDIAKEY__
    ' ', 0, 'M', 0, 'e', 0, 'd', 0, 'i', 0, 'a', 0, 'k', 0, 'e', 0, 'y', 0,
#else
    ' ', 0, 'P', 0, 'h', 0, 'o', 0, 'n', 0, 'e', 0, 'k', 0, 'e', 0, 'y', 0,
#endif
#else
    38,
    DESC_STRING,
    'U', 0, 'A', 0, 'C', 0, ' ', 0, '1', 0, '.', 0, '0', 0, ' ', 0,
    'M', 0, 'i', 0, 'c', 0, 'r', 0, 'o', 0, 'p', 0, 'h', 0, 'o', 0, 'n', 0, 'e', 0
#endif
};
/*!<USB Serial String Descriptor */
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

uint8_t *gu8UsbHidReport[4] =
{
    0,
    0,
#ifdef __HID__
    HID_DeviceReportDescriptor,
#else
    0,
#endif
    0
};

uint32_t gu32UsbHidReportLen[4] =
{
    0,
    0,
#ifdef __HID__
    sizeof(HID_DeviceReportDescriptor),
#else
    0,
#endif
    0
};

uint32_t gu32ConfigHidDescIdx[3] =
{
    0,
    0,
    (sizeof(gu8ConfigDescriptor) - LEN_HID - LEN_ENDPOINT),
};


const S_USBD_INFO_T gsInfo = {
    gu8DeviceDescriptor,
    gu8ConfigDescriptor,
    gpu8UsbString,
    gu8UsbHidReport,
    0,
    gu32UsbHidReportLen,
    gu32ConfigHidDescIdx,
};

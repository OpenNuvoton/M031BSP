/******************************************************************************
 * @file     usbd_audio.h
 * @brief    NuMicro series USB driver header file
 * @version  1.0.0
 * @date     22, December, 2013
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __USBD_UAC_H__
#define __USBD_UAC_H__

#include "NuMicro.h"

#define OPT_I2C0
//#define INPUT_IS_LIN
#define OPT_I2S_SLAVE_MODE

#define NAU8822_ADDR     0x1A                /* NAU8822 Device ID */
#define CRYSTAL_LESS        1

//#define dgb_printf printf
#define dgb_printf(...)

void I2C_WriteNAU8822(uint8_t u8addr, uint16_t u16data);

/* HIRC trim setting:
 *    HIRC trim reference clock is from USB SOF (Start-Of-Frame) packet.
 *    HIRC trim operation is keep going if clock is inaccuracy.
 *    HIRC Trim retry count limitation is 512 loops.
 *    Trim value calculation is based on average difference in 4 clocks of reference clock.
 *    Enable HIRC auto trim function and trim HIRC to 48 MHz.
 */
#define DEFAULT_HIRC_TRIM_SETTING    ((0x1ul<<SYS_HIRCTRIMCTL_REFCKSEL_Pos)| \
                                      (0x0ul<<SYS_HIRCTRIMCTL_CESTOPEN_Pos)| \
                                      (0x3ul<<SYS_HIRCTRIMCTL_RETRYCNT_Pos)| \
                                      (0x0ul<<SYS_HIRCTRIMCTL_LOOPSEL_Pos) | \
                                      (0x1ul<<SYS_HIRCTRIMCTL_FREQSEL_Pos))

/* Define the vendor id and product id */
#define USBD_VID        0x0416

#define USBD_PID        0xB006

#define UAC_MICROPHONE  0
#define UAC_SPEAKER     1

/*!<Define Audio information */
#define SAMPLING_RATE  48000            /* The Sampling rate. It could be 8000, 16000, 32000, and 48000 */

#define REC_RATE        SAMPLING_RATE   /* The record sampling rate. Must be the same with PLAY_RATE */
#define REC_CHANNELS    2           /* Number of channels. Don't Change */

#define REC_FEATURE_UNITID      0x05

#define AUDIO_RATE_8K      8000
#define AUDIO_RATE_16K    16000
#define AUDIO_RATE_32K    32000
#define AUDIO_RATE_441K   44100
#define AUDIO_RATE_48K    48000
#define AUDIO_RATE_96K    96000

#if(REC_CHANNELS == 1)
    #define REC_CH_CFG     1
#endif
#if(REC_CHANNELS == 2)
    #define REC_CH_CFG     3
#endif


/********************************************/
/* Audio Class Current State                */
/********************************************/
/*!<Define Audio Class Current State */
#define UAC_STOP_AUDIO_RECORD           0
#define UAC_START_AUDIO_RECORD          1
#define UAC_PROCESSING_AUDIO_RECORD     2
#define UAC_BUSY_AUDIO_RECORD           3

/***************************************************/
/*      Audio Class-Specific Request Codes         */
/***************************************************/
/*!<Define Audio Class Specific Request */
#define UAC_REQUEST_CODE_UNDEFINED  0x00
#define UAC_SET_CUR                 0x01
#define UAC_GET_CUR                 0x81
#define UAC_SET_MIN                 0x02
#define UAC_GET_MIN                 0x82
#define UAC_SET_MAX                 0x03
#define UAC_GET_MAX                 0x83
#define UAC_SET_RES                 0x04
#define UAC_GET_RES                 0x84
#define UAC_SET_MEM                 0x05
#define UAC_GET_MEM                 0x85
#define UAC_GET_STAT                0xFF

#define MUTE_CONTROL                0x01
#define VOLUME_CONTROL              0x02


/*!<Define HID Class Specific Request */
#define GET_REPORT              0x01
#define GET_IDLE                0x02
#define GET_PROTOCOL            0x03
#define SET_REPORT              0x09
#define SET_IDLE                0x0A
#define SET_PROTOCOL            0x0B

#ifdef __HID__
#ifdef __MEDIAKEY__
/* Byte 0 */
#define HID_CTRL_MUTE        0x01
#define HID_CTRL_VOLUME_INC  0x02
#define HID_CTRL_VOLUME_DEC  0x04
/* Byte 1 */
#define HID_CTRL_PLAY        0x01
#define HID_CTRL_STOP        0x02
#define HID_CTRL_PAUSE       0x04
#define HID_CTRL_NEXT        0x08
#define HID_CTRL_PREVIOUS    0x10
#define HID_CTRL_RECORD      0x20
#define HID_CTRL_REWIND      0x40
#define HID_CTRL_FF          0x80
#endif
#endif

/*-------------------------------------------------------------*/
/* Define EP maximum packet size */
#define EP0_MAX_PKT_SIZE    8
#define EP1_MAX_PKT_SIZE    EP0_MAX_PKT_SIZE

/* Maximum Packet Size for Recprd Endpoint */
#define EP2_MAX_PKT_SIZE    (REC_RATE * REC_CHANNELS * 2 / 1000)

#define EP4_MAX_PKT_SIZE    8

#define EP5_MAX_PKT_SIZE    8

#define SETUP_BUF_BASE      0
#define SETUP_BUF_LEN       8
#define EP0_BUF_BASE        (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define EP0_BUF_LEN         EP0_MAX_PKT_SIZE
#define EP1_BUF_BASE        (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define EP1_BUF_LEN         EP1_MAX_PKT_SIZE
#define EP2_BUF_BASE        (EP1_BUF_BASE + EP1_BUF_LEN)
#define EP2_BUF_LEN         EP2_MAX_PKT_SIZE
#define EP4_BUF_BASE        (EP2_BUF_BASE + EP2_BUF_LEN)
#define EP4_BUF_LEN         EP4_MAX_PKT_SIZE
#define EP5_BUF_BASE        (EP4_BUF_BASE + EP4_BUF_LEN)
#define EP5_BUF_LEN         EP5_MAX_PKT_SIZE

/* Define the interrupt In EP number */
#define ISO_IN_EP_NUM    0x01
#define ISO_OUT_EP_NUM   0x02
#define HID_IN_EP_NUM    0x03
#define HID_OUT_EP_NUM   0x04

/*-------------------------------------------------------------*/
extern volatile uint32_t g_usbd_UsbAudioState;
void UAC_DeviceEnable(uint8_t u8Object);
void UAC_DeviceDisable(uint8_t u8Object);
void UAC_SendRecData(void);
void UAC_GetPlayData(int16_t *pi16src, int16_t i16Samples);
void UAC_Init(void);
void UAC_ClassRequest(void);
void UAC_SetInterface(void);
void EP2_Handler(void);
void EP3_Handler(void);
void EP4_Handler(void);
void EP5_Handler(void);
void NAU8822_Setup(void);
void SamplingControl(void);
void AdjFreq(void);
void AdjFreq1(void);
void VolumnControl(void);
void I2C_WriteWAU8822(uint8_t u8addr, uint16_t u16data);
void HID_UpdateHidData(void);
void GPIO_Init(void);

extern volatile uint8_t  g_u8EP4Ready;
extern volatile uint32_t g_usbd_RecSampleRate;
extern volatile uint32_t g_PreRecSampleRate;

#endif  /* __USBD_UAC_H_ */

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/

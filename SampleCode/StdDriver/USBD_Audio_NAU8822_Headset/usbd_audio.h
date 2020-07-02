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

//#define __FEEDBACK__    /* Enable Feedback Endpoint */

#define OPT_I2C0
//#define INPUT_IS_LIN
#define OPT_I2S_SLAVE_MODE

#define NAU8822_ADDR     0x1A                /* NAU8822 Device ID */
#define CRYSTAL_LESS        1

//#define dgb_printf printf
#define dgb_printf(...)

/*
    PLAY_PIPE_RESAMPLE_ENABLE : Only set Audio Codec Setting when initialization & Do resample when changing play sampling rate
    Otherwise                 : Change Audio Codec Setting when changing play sampling rate
 */
//#define PLAY_PIPE_RESAMPLE_ENABLE 

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

#define PLAY_RATE       SAMPLING_RATE   /* The play sampling rate */
#define PLAY_CHANNELS   2               /* Number of channels. Don't Change */

#define REC_RATE        SAMPLING_RATE   /* The record sampling rate */
#define REC_CHANNELS    2               /* Number of channels. Don't Change */

#define REC_FEATURE_UNITID      0x05
#define PLAY_FEATURE_UNITID     0x06

#define AUDIO_RATE_8K      8000
#define AUDIO_RATE_16K    16000
#define AUDIO_RATE_32K    32000
#define AUDIO_RATE_441K   44100
#define AUDIO_RATE_48K    48000
#define AUDIO_RATE_96K    96000

/*
    BUF_LEN is used to define buffer length, each word buffer is two channels and 16 bits,
    we use 4 times larger buffer to store the play data
*/
//#define BUF_LEN   (PLAY_RATE*2*PLAY_CHANNELS/1000/2*4)
//#define BUF_LEN   (PLAY_RATE*2*PLAY_CHANNELS/1000/2 * 16)
#define BUF_LEN     32*12*4

#define BUF_LEN_7  BUF_LEN*7/8
#define BUF_LEN_6  BUF_LEN*6/8
#define BUF_LEN_5  BUF_LEN*5/8
#define BUF_LEN_3  BUF_LEN*3/8
#define BUF_LEN_2  BUF_LEN*2/8
#define BUF_LEN_1  BUF_LEN*1/8


/* Define Descriptor information */
#if(PLAY_CHANNELS == 1)
    #define PLAY_CH_CFG     1
#endif
#if(PLAY_CHANNELS == 2)
    #define PLAY_CH_CFG     3
#endif

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

#ifdef __FEEDBACK__
/* Maximum Packet Size for Play Endpoint */
#define EP3_MAX_PKT_SIZE    (PLAY_RATE * PLAY_CHANNELS * 2 / 1000) + 64
#else
#define EP3_MAX_PKT_SIZE    (PLAY_RATE * PLAY_CHANNELS * 2 / 1000)
#endif

/* Maximum Packet Size for Feedback Endpoint */
#define EP4_MAX_PKT_SIZE    8

/* Maximum Packet Size for HID Endpoint */
#define EP5_MAX_PKT_SIZE    8

#define SETUP_BUF_BASE      0
#define SETUP_BUF_LEN       8
#define EP0_BUF_BASE        (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define EP0_BUF_LEN         EP0_MAX_PKT_SIZE
#define EP1_BUF_BASE        (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define EP1_BUF_LEN         EP1_MAX_PKT_SIZE
#define EP2_BUF_BASE        (EP1_BUF_BASE + EP1_BUF_LEN)
#define EP2_BUF_LEN         EP2_MAX_PKT_SIZE
#define EP3_BUF_BASE        (EP2_BUF_BASE + EP2_BUF_LEN)
#define EP3_BUF_LEN         EP3_MAX_PKT_SIZE
#define EP4_BUF_BASE        (EP3_BUF_BASE + EP3_BUF_LEN)
#define EP4_BUF_LEN         EP4_MAX_PKT_SIZE
#define EP5_BUF_BASE        (EP4_BUF_BASE + EP4_BUF_LEN)
#define EP5_BUF_LEN         EP5_MAX_PKT_SIZE

/* Define the interrupt In EP number */
#define ISO_IN_EP_NUM       0x01
#define ISO_OUT_EP_NUM      0x02
#define ISO_IN_FB_EP_NUM    0x03
#define HID_IN_EP_NUM       0x04

typedef enum
{
    E_RS_REC_CH0,          /* Record Channel 0 */
    E_RS_REC_CH1,          /* Record Channel 1 */
    E_RS_PLAY_CH0,         /* Play Channel 0 */
    E_RS_PLAY_CH1          /* Play Channel 1 */
} RESAMPLE_MODE_T;


typedef struct {
    int r;
    int s;
    int d;
    int count;
} SR_T;

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
int SamplingFactor(uint32_t u32SrceRate, uint32_t u32DestRate);

int Resamples(RESAMPLE_MODE_T mode, short *x, int ch_num, int samples, short *y, int s_idx);

extern volatile uint8_t  g_u8EP5Ready;
extern volatile uint32_t g_usbd_CodecSampleRate;
extern volatile uint32_t g_usbd_PlaySampleRate;
extern volatile uint32_t g_usbd_RecSampleRate;
extern volatile uint32_t g_PrePlaySampleRate;
extern volatile uint32_t g_PreRecSampleRate;
extern volatile uint32_t g_resample_play_s_idx;
extern volatile uint32_t g_resample_rec_s_idx;
extern volatile uint32_t g_usbd_PlaySampleRate;
extern volatile uint32_t g_usbd_RecSampleRate;
extern volatile uint32_t g_play_max_packet_size;
extern volatile uint32_t g_rec_max_packet_size;
extern volatile uint32_t g_u32SampleRate;
extern volatile uint32_t g_play_len_frame;
extern const SR_T srt[];

#endif  /* __USBD_UAC_H_ */

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/

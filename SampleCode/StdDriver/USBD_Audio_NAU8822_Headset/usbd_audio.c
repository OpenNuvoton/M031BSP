/******************************************************************************
 * @file     usbd_audio.c
 * @brief    USBD driver Sample file
 * @version  1.0.0
 * @date     23, December, 2013
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include <stdio.h>
#include "NuMicro.h"
#include "usbd_audio.h"

typedef enum
{
    E_RS_NONE,          /* no resampling */
    E_RS_UP,            /* up sampling   */
    E_RS_DOWN           /* down sampling */
} RESAMPLE_STATE_T;

/*--------------------------------------------------------------------------*/
/* Global variables for Audio class */
volatile uint32_t g_usbd_UsbAudioState = 0;
volatile uint32_t g_usbd_CodecSampleRate = SAMPLING_RATE;
volatile uint32_t g_usbd_PlaySampleRate  = PLAY_RATE;
volatile uint32_t g_usbd_RecSampleRate   = REC_RATE;
volatile uint32_t g_PrePlaySampleRate  = PLAY_RATE;
volatile uint32_t g_PreRecSampleRate   = REC_RATE;  
volatile uint32_t g_PreCodecSampleRate = SAMPLING_RATE;

volatile uint32_t g_play_max_packet_size = (PLAY_RATE * PLAY_CHANNELS * 2 / 1000);
volatile uint32_t g_rec_max_packet_size = (REC_RATE * REC_CHANNELS * 2 / 1000);

volatile uint32_t g_resample_play_s_idx = 25;
volatile uint32_t g_resample_rec_s_idx = 25;

volatile uint32_t g_u32SampleRate = PLAY_RATE / 1000;
volatile uint32_t g_play_len_frame = PLAY_RATE / 1000;

volatile uint8_t g_usbd_RecMute       = 0x0001;   /* Record MUTE control. 0 = normal. 1 = MUTE */
volatile int16_t g_usbd_RecVolumeL    = 0x0000;   /* Record left channel volume. Range is -8176 ~ 8176 */
volatile int16_t g_usbd_RecVolumeR    = 0x0000;   /* Record right channel volume. Range is -8176 ~ 8176 */
volatile int16_t g_usbd_RecMaxVolume  =   8176;   /*  8176 */
volatile int16_t g_usbd_RecMinVolume  =  -8176;   /* -8176 */
volatile int16_t g_usbd_RecResVolume  = 0x0020;

volatile uint8_t g_usbd_PlayMute      = 0x0000;   /* Play MUTE control. 0 = normal. 1 = MUTE */
volatile int16_t g_usbd_PlayVolumeL   = 0xFFC0;   /* Play left channel volume. Range is -32752 ~ -25504 */
volatile int16_t g_usbd_PlayVolumeR   = 0xFFC0;   /* Play right channel volume. Range is -32752 ~ -25504 */
volatile int16_t g_usbd_PlayMaxVolume = 0xFFF0;   /* -32752 */
volatile int16_t g_usbd_PlayMinVolume = 0xE3A0;   /* -25504 */
volatile int16_t g_usbd_PlayResVolume = 0x0030;

static volatile uint8_t g_u8RecEn = 0;
static volatile uint8_t g_u8PlayEn = 0;      /* To indicate data is output to I2S */
static volatile int32_t g_i32AdjFlag = 0;    /* To indicate current I2S frequency adjustment status */

/* Temp buffer for play and record */
uint32_t g_au32UsbTmpBuf[SAMPLING_RATE * 2 * PLAY_CHANNELS / 1000 / 4 + 16] = {0};

short g_a16AudioTmpBuf0[SAMPLING_RATE * 2 * REC_CHANNELS / 1000 + 16] = {0};

/* Recoder Buffer and its pointer */
uint32_t g_au32PcmRecBuf[SAMPLING_RATE *  2 * REC_CHANNELS  / 1000 / 4 * 4] = {0};
volatile uint32_t g_u32RecPos = 0;

/* Player Buffer and its pointer */
uint32_t g_au32PcmPlayBuf[BUF_LEN] = {0};
volatile uint32_t g_u32PlayPos_Out = 0;
volatile uint32_t g_u32PlayPos_In = 0;

uint8_t g_u8ReportBuf[3] = {0};
void Delay(int count)
{
    volatile uint32_t i;
    for(i = 0; i < count ; i++);
}

uint32_t GetSamplesInBuf(void)
{
    int32_t i32Tmp;

    i32Tmp = g_u32PlayPos_In;
    i32Tmp -= g_u32PlayPos_Out;
    if(i32Tmp < 0)
        i32Tmp += BUF_LEN;

    return (uint32_t)i32Tmp;
}

/**
 * @brief       USBD Interrupt Service Routine
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is the USBD ISR
 */
void USBD_IRQHandler(void)
{
    uint32_t u32IntSts = USBD_GET_INT_FLAG();
    uint32_t u32State = USBD_GET_BUS_STATE();

    if (u32IntSts & USBD_INTSTS_FLDET)
    {
        /* Floating detect */
        USBD_CLR_INT_FLAG(USBD_INTSTS_FLDET);

        if (USBD_IS_ATTACHED())
        {
            /* USB Plug In */
            USBD_ENABLE_USB();

            /*Enable HIRC tirm*/
            SYS->HIRCTRIMCTL = DEFAULT_HIRC_TRIM_SETTING;
        }
        else
        {
            /* USB Un-plug */
            USBD_DISABLE_USB();

            /*Disable HIRC tirm*/
            SYS->HIRCTRIMCTL = DEFAULT_HIRC_TRIM_SETTING & (~SYS_HIRCTRIMCTL_FREQSEL_Msk);
        }
    }

    if (u32IntSts & USBD_INTSTS_BUS) 
    {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_BUS);

        if (u32State & USBD_STATE_USBRST) 
        {
            /* Bus reset */
            USBD_ENABLE_USB();
            USBD_SwReset();

            /* Disable I2S Tx function */
            SPII2S_DISABLE_TX(SPI0);
            /* Disable I2S Rx function */
            SPII2S_DISABLE_RX(SPI0);

            /*Enable HIRC tirm*/
            SYS->HIRCTRIMCTL = DEFAULT_HIRC_TRIM_SETTING;
        }

        if (u32State & USBD_STATE_SUSPEND)
        {
            /* Enable USB but disable PHY */
            USBD_DISABLE_PHY();

            /*Disable HIRC tirm*/
            SYS->HIRCTRIMCTL = DEFAULT_HIRC_TRIM_SETTING & (~SYS_HIRCTRIMCTL_FREQSEL_Msk);
        }

        if (u32State & USBD_STATE_RESUME)        /* Power Down Wakeup Source */
        {
            /* Enable USB and enable PHY */
            USBD_ENABLE_USB();

            /*Enable HIRC tirm*/
            SYS->HIRCTRIMCTL = DEFAULT_HIRC_TRIM_SETTING;
        }

#ifdef SUPPORT_LPM
        if (u32State & USBD_STATE_L1SUSPEND)
        {
            /*
               TODO: Implement LPM SUSPEND flag here.
                     Recommend implementing the power-saving function in main loop.
            */
        }

        if (u32State & USBD_STATE_L1RESUME)
        {
            /*
               TODO: Implement LPM RESUME flag here.
            */
        }
#endif
    }

    if(u32IntSts & USBD_INTSTS_USB)
    {
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

        /* EP events */
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
            USBD_CtrlOut();
        }

        if(u32IntSts & USBD_INTSTS_EP2)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP2);

            /* Isochronous IN for Record pipe */
            EP2_Handler();
        }

        if(u32IntSts & USBD_INTSTS_EP3)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP3);

            /* Isochronous OUT for Play pipe */
            EP3_Handler();
        }

        if(u32IntSts & USBD_INTSTS_EP4)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP4);
#ifdef __FEEDBACK__
            /* Isochronous IN for Feedback */
            EP4_Handler();
#endif
        }

        if(u32IntSts & USBD_INTSTS_EP5)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP5);
#ifdef __HID__
            /* Interrupt IN for HID */
            EP5_Handler();
#endif
        }

        if(u32IntSts & USBD_INTSTS_EP6)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP6);
        }

        if(u32IntSts & USBD_INTSTS_EP7)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP7);
        }
    }
}

/**
 * @brief       EP2 Handler
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to process EP2 event for recording.
 */

void EP2_Handler(void)
{
    /* ISO IN transfer ACK */
    if(g_usbd_UsbAudioState == UAC_START_AUDIO_RECORD)
    {
        UAC_DeviceEnable(UAC_MICROPHONE);
        g_usbd_UsbAudioState = UAC_PROCESSING_AUDIO_RECORD;
    }
    else if(g_usbd_UsbAudioState == UAC_PROCESSING_AUDIO_RECORD)
        g_usbd_UsbAudioState = UAC_BUSY_AUDIO_RECORD;

    if(g_usbd_UsbAudioState == UAC_BUSY_AUDIO_RECORD)
        UAC_SendRecData();
    else
        USBD_SET_PAYLOAD_LEN(EP2, 0);
}

/**
 * @brief       EP3 Handler (ISO OUT interrupt handler)
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to process EP3 event (ISO OUT transfer ACK) for play audio.
 */
void EP3_Handler(void)
{
    uint32_t u32Len;
#ifdef PLAY_PIPE_RESAMPLE_ENABLE
    uint32_t u32Samples = 0;
    uint32_t u32SampleCount = 0;
#endif
    uint32_t u32Idx;
    int32_t i;
    short *pu16Buf;
    short *p16Src;

    /* Get the address in USB buffer */
    p16Src = (short *)((uint32_t)USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP3));

    /* Get the temp buffer */
    pu16Buf = (short *)g_au32UsbTmpBuf;

#ifdef PLAY_PIPE_RESAMPLE_ENABLE
    /* Audio data is resampled to 48MHz (Audio Codec is fixed to 48MHz) */
    /* Do Resample operation to data in USB buffer to SRAM buffer */
    if(PLAY_CHANNELS == 2)
    {
        /* u32SampleCount = total length / 2 (16 bits) / 2 (Two Channel) */
        u32SampleCount = USBD_GET_PAYLOAD_LEN(EP3) >> 2;

        u32Samples  = Resamples(E_RS_PLAY_CH0,     p16Src, PLAY_CHANNELS, u32SampleCount,     pu16Buf, g_resample_play_s_idx);

        u32Samples += Resamples(E_RS_PLAY_CH1, &p16Src[1], PLAY_CHANNELS, u32SampleCount, &pu16Buf[1], g_resample_play_s_idx);
    }
    else
    {
        /* u32SampleCount = total length / 2 (16 bits) / 1 (One Channel) */
        u32SampleCount = USBD_GET_PAYLOAD_LEN(EP3) >> 1;

        u32Samples  = Resamples(E_RS_PLAY_CH0,     p16Src, PLAY_CHANNELS, u32SampleCount,     pu16Buf, g_resample_play_s_idx);
    }
    /* Play Data Length for SAMPLING_RATE */
    u32Len = u32Samples * 2;
#else
    u32Len = USBD_GET_PAYLOAD_LEN(EP3);

    /* Copy all data from USB buffer to SRAM buffer */
    for(i = 0; i < u32Len >> 1; i += 2)
    {
        pu16Buf[i] = p16Src[i];
        pu16Buf[i + 1] = p16Src[i + 1];
    }
#endif
    /* Calculate length (Word) */
    u32Len = u32Len >> 2;

    for(i = 0; i < u32Len; i++)
    {
        /* Check ring buffer turn around */
        u32Idx = g_u32PlayPos_In + 1;
        if(u32Idx >= BUF_LEN)
            u32Idx = 0;

        /* Check if buffer full */
        if(u32Idx != g_u32PlayPos_Out)
        {
            /* Update play ring buffer only when it is not full */
            g_au32PcmPlayBuf[u32Idx] = g_au32UsbTmpBuf[i];

            /* Update IN index */
            g_u32PlayPos_In = u32Idx;
        }
    }

#ifdef __FEEDBACK__
    /* Prepare for nex OUT packet */
    USBD_SET_PAYLOAD_LEN(EP3, g_play_max_packet_size + 64);
#else
    /* Prepare for nex OUT packet */
    USBD_SET_PAYLOAD_LEN(EP3, g_play_max_packet_size);
#endif

    if(g_u8PlayEn == 0)
    {
        /* Start play data output through I2S only when we have enough data in buffer */
        if(GetSamplesInBuf() > BUF_LEN / 2)
            g_u8PlayEn = 1;
    }
}

#ifdef __FEEDBACK__
/**
 * @brief       EP4 Handler (Iso IN feedback interrupt handler)
 *
 * @param[in]   None
 *
 * @return      None
 *
 */
void EP4_Handler(void)
{
    uint8_t *pu8Buf;
    uint32_t u32Size;

    if (g_u8PlayEn != 0)
    { 

        /* Get sample size in play buffer */
        u32Size = GetSamplesInBuf();

        if( u32Size > BUF_LEN_7)
        {
            g_u32SampleRate = g_play_len_frame -5;
            dgb_printf("overrun = %d, %d\n", g_u32SampleRate, u32Size);
        }
        else if( u32Size > BUF_LEN_6 )
        {
            g_u32SampleRate = g_play_len_frame -3;
        }
        else if( u32Size > BUF_LEN_5 )
        {
            g_u32SampleRate = g_play_len_frame -1;
        }
        else if( u32Size < BUF_LEN_1 )
        {
            g_u32SampleRate = g_play_len_frame + 5;
            dgb_printf("underrun = %d,  %d\n", g_u32SampleRate, u32Size);
        }
        else if( u32Size < BUF_LEN_2 )
        {
            g_u32SampleRate = g_play_len_frame + 3;
        }
        else if( u32Size < BUF_LEN_3 )
        {
            g_u32SampleRate = g_play_len_frame + 1;
        }
        else
        {
            g_u32SampleRate = g_play_len_frame;
        }
    }
    /* Get the address in USB buffer */
    pu8Buf = (uint8_t *)((uint32_t)USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP4));

    /* Prepare the data to USB IN buffer */
    *pu8Buf++ = 0x00;
    *pu8Buf++ = (g_u32SampleRate & 0x3) << 6;
    *pu8Buf = (g_u32SampleRate & 0xFC) >> 2;
    /* Trigger ISO IN */
    USBD_SET_PAYLOAD_LEN(EP4, 3);
}
#endif
#ifdef __HID__
void EP5_Handler(void)  /* Interrupt IN handler */
{
    g_u8EP5Ready = 1;
}
#endif

/*--------------------------------------------------------------------------*/
/**
 * @brief       UAC Class Initial
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to configure endpoints for UAC class
 */
void UAC_Init(void)
{
#ifdef __FEEDBACK__
    uint8_t *pu8Buf;
#endif 
    /* Init setup packet buffer */
    /* Buffer for setup packet -> [0 ~ 0x7] */
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
    /* EP2 ==> Isochronous IN endpoint, address 1 */
    USBD_CONFIG_EP(EP2, USBD_CFG_EPMODE_IN | ISO_IN_EP_NUM | USBD_CFG_TYPE_ISO);
    /* Buffer offset for EP2 */
    USBD_SET_EP_BUF_ADDR(EP2, EP2_BUF_BASE);

    /*****************************************************/
    /* EP3 ==> Isochronous OUT endpoint, address 2 */
    USBD_CONFIG_EP(EP3, USBD_CFG_EPMODE_OUT | ISO_OUT_EP_NUM | USBD_CFG_TYPE_ISO);
    /* Buffer offset for EP3 */
    USBD_SET_EP_BUF_ADDR(EP3, EP3_BUF_BASE);
    /* trigger receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
#ifdef __FEEDBACK__
    /*****************************************************/
    /* EP4 ==> Isochronous IN endpoint, address 3 */
    USBD_CONFIG_EP(EP4, USBD_CFG_EPMODE_IN | ISO_IN_FB_EP_NUM | USBD_CFG_TYPE_ISO);
    /* Buffer offset for EP4 */
    USBD_SET_EP_BUF_ADDR(EP4, EP4_BUF_BASE);

    g_u32SampleRate = PLAY_RATE/1000;
#endif
#ifdef __HID__
    /*****************************************************/
    /* EP5 ==> Interrupt IN endpoint, address 4 */
    USBD_CONFIG_EP(EP5, USBD_CFG_EPMODE_IN | HID_IN_EP_NUM);
    /* Buffer offset for EP5 */
    USBD_SET_EP_BUF_ADDR(EP5, EP5_BUF_BASE);

    g_u8EP5Ready = 1;
#endif
#ifdef __FEEDBACK__
    /* Feedback Endpoint */
        pu8Buf = (uint8_t *)((uint32_t)USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP4));

        g_u32SampleRate = g_usbd_PlaySampleRate / 1000;

        g_play_len_frame = g_usbd_PlaySampleRate / 1000;

        /* Prepare the data to USB IN buffer */
        *pu8Buf++ = 0x00;
        *pu8Buf++ = (g_u32SampleRate & 0x3) << 6;
        *pu8Buf = (g_u32SampleRate & 0xFC) >> 2;

        /* Trigger ISO IN */
        USBD_SET_DATA1(EP4);

        USBD_SET_PAYLOAD_LEN(EP4, 3);

    /* End of Feedback Endpoint */
#endif

}

/**
 * @brief       UAC class request
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to process UAC class requests
 */
void UAC_ClassRequest(void)
{
    uint8_t buf[8];
#ifdef __HID__
    static uint8_t u8Protocol = 0;
    static uint8_t u8Idle = 0;
#endif
    USBD_GetSetupPacket(buf);

    if(buf[0] & 0x80)    /* request data transfer direction */
    {
        /* Device to host */
        switch(buf[1])
        {
            case UAC_GET_CUR:
            {
                if((buf[4] & 0x0F) == ISO_IN_EP_NUM)
                {
                    M32(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_RecSampleRate;
                    /* Status stage */
                    USBD_SET_DATA1(EP0);
                    USBD_SET_PAYLOAD_LEN(EP0, 0);
                    break;
                }
                else if((buf[4] & 0x0F) == ISO_OUT_EP_NUM)
                {
                    M32(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_PlaySampleRate;
                    /* Status stage */
                    USBD_SET_DATA1(EP0);
                    USBD_SET_PAYLOAD_LEN(EP0, 0);
                    break;
                }
                else
                {
                    switch(buf[3])
                    {
                        case MUTE_CONTROL:
                        {
                            if(REC_FEATURE_UNITID == buf[5])
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_RecMute;
                            else if(PLAY_FEATURE_UNITID == buf[5])
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_PlayMute;

                            /* Data stage */
                            USBD_SET_DATA1(EP0);
                            USBD_SET_PAYLOAD_LEN(EP0, 1);
                            break;
                        }
                        case VOLUME_CONTROL:
                        {
                            if(REC_FEATURE_UNITID == buf[5])
                            {
                                /* Left or right channel */
                                if(buf[2] == 1)
                                {
                                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_RecVolumeL;
                                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_RecVolumeL >> 8;
                                }
                                else
                                {
                                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_RecVolumeR;
                                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_RecVolumeR >> 8;
                                }
                            }
                            else if(PLAY_FEATURE_UNITID == buf[5])
                            {
                                /* Left or right channel */
                                if(buf[2] == 1)
                                {
                                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_PlayVolumeL;
                                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_PlayVolumeL >> 8;
                                }
                                else
                                {
                                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_PlayVolumeR;
                                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_PlayVolumeR >> 8;
                                }
                            }
                            /* Data stage */
                            USBD_SET_DATA1(EP0);
                            USBD_SET_PAYLOAD_LEN(EP0, 2);
                            break;
                        }
                        default:
                        {
                            /* Setup error, stall the device */
                            USBD_SetStall(0);
                        }
                    }
                    /* Trigger next Control Out DATA1 Transaction. */
                    /* Status stage */
                    USBD_PrepareCtrlOut(0, 0);
                    break;
                }

                case UAC_GET_MIN:
                {
                    switch(buf[3])
                    {
                        case VOLUME_CONTROL:
                        {
                            if(REC_FEATURE_UNITID == buf[5])
                            {
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_RecMinVolume;
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_RecMinVolume >> 8;
                            }
                            else if(PLAY_FEATURE_UNITID == buf[5])
                            {
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_PlayMinVolume;
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_PlayMinVolume >> 8;
                            }
                            /* Data stage */
                            USBD_SET_DATA1(EP0);
                            USBD_SET_PAYLOAD_LEN(EP0, 2);
                            break;
                        }
                        default:
                            /* STALL control pipe */
                            USBD_SetStall(0);
                    }
                    /* Trigger next Control Out DATA1 Transaction. */
                    /* Status stage */
                    USBD_PrepareCtrlOut(0, 0);
                    break;
                }

                case UAC_GET_MAX:
                {
                    switch(buf[3])
                    {
                        case VOLUME_CONTROL:
                        {
                            if(REC_FEATURE_UNITID == buf[5])
                            {
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_RecMaxVolume;
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_RecMaxVolume >> 8;
                            }
                            else if(PLAY_FEATURE_UNITID == buf[5])
                            {
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_PlayMaxVolume;
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_PlayMaxVolume >> 8;
                            }
                            /* Data stage */
                            USBD_SET_DATA1(EP0);
                            USBD_SET_PAYLOAD_LEN(EP0, 2);
                            break;
                        }
                        default:
                            /* STALL control pipe */
                            USBD_SetStall(0);
                    }
                    /* Trigger next Control Out DATA1 Transaction. */
                    /* Status stage */
                    USBD_PrepareCtrlOut(0, 0);
                    break;
                }

                case UAC_GET_RES:
                {
                    switch(buf[3])
                    {
                        case VOLUME_CONTROL:
                        {
                            if(REC_FEATURE_UNITID == buf[5])
                            {
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_RecResVolume;
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_RecResVolume >> 8;
                            }
                            else if(PLAY_FEATURE_UNITID == buf[5])
                            {
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_PlayResVolume;
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_PlayResVolume >> 8;
                            }
                            /* Data stage */
                            USBD_SET_DATA1(EP0);
                            USBD_SET_PAYLOAD_LEN(EP0, 2);
                            break;
                        }
                        default:
                            /* STALL control pipe */
                            USBD_SetStall(0);
                    }
                    /* Trigger next Control Out DATA1 Transaction. */
                    /* Status stage */
                    USBD_PrepareCtrlOut(0, 0);
                    break;
                }

#ifdef __HID__
//                 case GET_REPORT:
//                 {
//                     break;
//                 }
                case GET_IDLE:
                {
                    /* Data stage */
                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = u8Idle;
                    USBD_SET_DATA1(EP0);
                    USBD_SET_PAYLOAD_LEN(EP0, 1);
                    /* Status stage */
                    USBD_PrepareCtrlOut(0,0);
                    break;
                }
                case GET_PROTOCOL:
                {
                    /* Data stage */
                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = u8Protocol;
                    USBD_SET_DATA1(EP0);
                    USBD_SET_PAYLOAD_LEN(EP0, 1);
                    /* Status stage */
                    USBD_PrepareCtrlOut(0,0);
                    break;
                }
#endif
                default:
                {
                    /* Setup error, stall the device */
                    USBD_SetStall(0);
                }
            }
        }
    }
    else
    {
        /* Host to device */
        switch(buf[1])
        {
            case UAC_SET_CUR:
            {
                if((buf[4] & 0x0F) == ISO_IN_EP_NUM)
                {
                    USBD_PrepareCtrlOut((uint8_t *)&g_usbd_RecSampleRate, buf[6]);
                    /* Status stage */
                    USBD_SET_DATA1(EP0);
                    USBD_SET_PAYLOAD_LEN(EP0, 0);
                    break;
                }
                else if((buf[4] & 0x0F) == ISO_OUT_EP_NUM)
                {                     
                    USBD_PrepareCtrlOut((uint8_t *)&g_usbd_PlaySampleRate, buf[6]);
                    /* Status stage */
                    USBD_SET_DATA1(EP0);
                    USBD_SET_PAYLOAD_LEN(EP0, 0);
                    break;
                }
                else
                {
                    switch(buf[3])
                    {
                        case MUTE_CONTROL:
                            if(REC_FEATURE_UNITID == buf[5])
                                USBD_PrepareCtrlOut((uint8_t *)&g_usbd_RecMute, buf[6]);
                            else if(PLAY_FEATURE_UNITID == buf[5])
                            {
                                USBD_PrepareCtrlOut((uint8_t *)&g_usbd_PlayMute, buf[6]);
                            }
                            /* Status stage */
                            USBD_SET_DATA1(EP0);
                            USBD_SET_PAYLOAD_LEN(EP0, 0);
                            break;

                        case VOLUME_CONTROL:
                            if(REC_FEATURE_UNITID == buf[5])
                            {
                                if(buf[2] == 1)
                                {
                                    /* Prepare the buffer for new record volume of left channel */
                                    USBD_PrepareCtrlOut((uint8_t *)&g_usbd_RecVolumeL, buf[6]);
                                }
                                else
                                {
                                    /* Prepare the buffer for new record volume of right channel */
                                    USBD_PrepareCtrlOut((uint8_t *)&g_usbd_RecVolumeR, buf[6]);
                                }
                            }
                            else if(PLAY_FEATURE_UNITID == buf[5])
                            {
                                if(buf[2] == 1)
                                {
                                    /* Prepare the buffer for new play volume of left channel */
                                    USBD_PrepareCtrlOut((uint8_t *)&g_usbd_PlayVolumeL, buf[6]);
                                }
                                else
                                {
                                    /* Prepare the buffer for new play volume of right channel */
                                    USBD_PrepareCtrlOut((uint8_t *)&g_usbd_PlayVolumeR, buf[6]);
                                }
                            }
                            /* Status stage */
                            USBD_SET_DATA1(EP0);
                            USBD_SET_PAYLOAD_LEN(EP0, 0);
                            break;

                        default:
                            /* STALL control pipe */
                            USBD_SetStall(0);
                            break;
                    }
                }
                break;
            }

#ifdef __HID__
            case SET_REPORT:
            {
                dgb_printf("Set Report\n");

                if(buf[6]!= 0)
                    USBD_PrepareCtrlOut((uint8_t *)&g_u8ReportBuf, buf[6]);

                /* Request Type = Feature */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
                break;
            }
            case SET_IDLE:
            {
                u8Idle = buf[3]; 
                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
                break;
            }
            case SET_PROTOCOL:
            {
                u8Protocol = buf[2]; 
                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
                break;
            }
#endif

            default:
            {
                /* Setup error, stall the device */
                USBD_SetStall(0);
                break;
            }
        }
    }
}

/**
 * @brief       Set Interface standard request
 *
 * @param[in]   u32AltInterface Interface
 *
 * @return      None
 *
 * @details     This function is used to set UAC Class relative setting
 */
void UAC_SetInterface(void)
{
    uint8_t buf[8];
    uint32_t u32AltInterface;

    USBD_GetSetupPacket(buf);

    u32AltInterface = buf[2];

    if(buf[4] == 1)
    {
        /* Audio Iso IN interface */
        if(u32AltInterface == 1)
        {
            /* Enable I2S Rx function */
            SPII2S_ENABLE_RX(SPI0);

            g_usbd_UsbAudioState = UAC_START_AUDIO_RECORD;
            USBD_SET_DATA1(EP2);
            USBD_SET_PAYLOAD_LEN(EP2, 0);
            UAC_DeviceEnable(UAC_MICROPHONE);
        }
        else if(u32AltInterface == 0)
        {
            /* Disable I2S Rx function */
            SPII2S_DISABLE_RX(SPI0);

            UAC_DeviceDisable(UAC_MICROPHONE);
            USBD_SET_DATA1(EP2);
            USBD_SET_PAYLOAD_LEN(EP2, 0);
            g_usbd_UsbAudioState = UAC_STOP_AUDIO_RECORD;
        }
    }
    else if(buf[4] == 2)
    {
        /* Audio Iso OUT interface */
        if(u32AltInterface == 1)
        {
            /* Enable I2S Tx function */
            SPII2S_ENABLE_TX(SPI0);

            USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);

            UAC_DeviceEnable(UAC_SPEAKER);
        }
        else
        {
            /* Disable I2S Tx function */
            SPII2S_DISABLE_TX(SPI0);

            UAC_DeviceDisable(UAC_SPEAKER);
        }
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Write 9-bit data to 7-bit address register of NAU8822 with I2C0                                        */
/*---------------------------------------------------------------------------------------------------------*/
#ifdef OPT_I2C0
void I2C_WriteNAU8822(uint8_t u8addr, uint16_t u16data)
{
    uint8_t audata[2];
    uint8_t u8temp;
    audata[0] = (uint8_t)((u8addr << 1) | (u16data >> 8));
    audata[1] = u16data & 0x00FF;
    do
    {
        u8temp = I2C_WriteMultiBytes(I2C0, NAU8822_ADDR,audata,2 );
    }while(u8temp!=2);
}
#else
void I2C_WriteNAU8822(uint8_t u8addr, uint16_t u16data)
{
    I2C_START(I2C1);
    I2C_WAIT_READY(I2C1);

    I2C_SET_DATA(I2C1, NAU8822_ADDR << 1);
    I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
    I2C_WAIT_READY(I2C1);

    I2C_SET_DATA(I2C1, (uint8_t)((u8addr << 1) | (u16data >> 8)));
    I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
    I2C_WAIT_READY(I2C1);

    I2C_SET_DATA(I2C1, (uint8_t)(u16data & 0x00FF));
    I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
    I2C_WAIT_READY(I2C1);

    I2C_STOP(I2C1);
}
#endif

void NAU8822_Setup(void)
{
    dgb_printf("\nConfigure NAU8822 ...");

    I2C_WriteNAU8822(0,  0x000);   /* Reset all registers */

    Delay(0x200);

#ifdef INPUT_IS_LIN                /* input source is LIN */
    I2C_WriteNAU8822(1,  0x02F);
    I2C_WriteNAU8822(2,  0x1B3);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    I2C_WriteNAU8822(3,  0x07F);   /* Enable L/R main mixer, DAC */
    I2C_WriteNAU8822(4,  0x010);   /* 16-bit word length, I2S format, Stereo */
    I2C_WriteNAU8822(5,  0x000);   /* Companding control and loop back mode (all disable) */

    if(g_usbd_CodecSampleRate == 48000)
    {
        I2C_WriteNAU8822(6,  0x14D);   /* Divide by 2, 48K@16Bit */
        I2C_WriteNAU8822(7,  0x000);   /* 48K for internal filter coefficients */
    }
    else if(g_usbd_CodecSampleRate == 32000)
    {
        I2C_WriteNAU8822(6,  0x16D);   /* Divide by 3, 32K@16Bit */
        I2C_WriteNAU8822(7,  0x002);   /* 32K for internal filter coefficients */
    }
    else if(g_usbd_CodecSampleRate == 16000)
    {
        I2C_WriteNAU8822(6,  0x1AD);   /* Divide by 6, 16K@16Bit */
        I2C_WriteNAU8822(7,  0x006);   /* 16K for internal filter coefficients */
    }
    else
    {
        I2C_WriteNAU8822(6,  0x1ED);   /* Divide by 12, 8K@16Bit */
        I2C_WriteNAU8822(7,  0x00A);   /* 8K for internal filter coefficients */
    }

    I2C_WriteNAU8822(10, 0x008);   /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    I2C_WriteNAU8822(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */
    I2C_WriteNAU8822(15, 0x1FF);   /* ADC left digital volume control */
    I2C_WriteNAU8822(16, 0x1FF);   /* ADC right digital volume control */

    I2C_WriteNAU8822(44, 0x000);   /* LLIN/RLIN is not connected to PGA */
    I2C_WriteNAU8822(47, 0x060);   /* LLIN connected, and its Gain value */
    I2C_WriteNAU8822(48, 0x060);   /* RLIN connected, and its Gain value */
    I2C_WriteNAU8822(50, 0x001);   /* Left DAC connected to LMIX */
    I2C_WriteNAU8822(51, 0x001);   /* Right DAC connected to RMIX */
#else                              /* input source is MIC */
    I2C_WriteNAU8822(1,  0x03F);
    I2C_WriteNAU8822(2,  0x1BF);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    I2C_WriteNAU8822(3,  0x07F);   /* Enable L/R main mixer, DAC */
    I2C_WriteNAU8822(4,  0x010);   /* 16-bit word length, I2S format, Stereo */
    I2C_WriteNAU8822(5,  0x000);   /* Companding control and loop back mode (all disable) */

    if(g_usbd_CodecSampleRate == 48000)
    {
        I2C_WriteNAU8822(6,  0x14D);   /* Divide by 2, 48K@16Bit */
        I2C_WriteNAU8822(7,  0x000);   /* 48K for internal filter coefficients */
    }
    else if(g_usbd_CodecSampleRate == 32000)
    {
        I2C_WriteNAU8822(6,  0x16D);   /* Divide by 3, 32K@16Bit */
        I2C_WriteNAU8822(7,  0x002);   /* 32K for internal filter coefficients */
    }
    else if(g_usbd_CodecSampleRate == 16000)
    {
        I2C_WriteNAU8822(6,  0x1AD);   /* Divide by 6, 16K@16Bit */
        I2C_WriteNAU8822(7,  0x006);   /* 16K for internal filter coefficients */
    }
    else
    {
        I2C_WriteNAU8822(6,  0x1ED);   /* Divide by 12, 8K@16Bit */
        I2C_WriteNAU8822(7,  0x00A);   /* 8K for internal filter coefficients */
    }

    I2C_WriteNAU8822(10, 0x008);   /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    I2C_WriteNAU8822(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */
    I2C_WriteNAU8822(15, 0x1EF);   /* ADC left digital volume control */
    I2C_WriteNAU8822(16, 0x1EF);   /* ADC right digital volume control */
    I2C_WriteNAU8822(44, 0x033);   /* LMICN/LMICP is connected to PGA */
    I2C_WriteNAU8822(50, 0x001);   /* Left DAC connected to LMIX */
    I2C_WriteNAU8822(51, 0x001);   /* Right DAC connected to RMIX */
#endif

    dgb_printf("[OK]\n");
}

void SPI0_IRQHandler(void)
{
    uint32_t u32I2SIntFlag;
    uint32_t i, u32Idx;
    u32I2SIntFlag = SPI0->I2SSTS;

    if (u32I2SIntFlag & SPI_I2SSTS_TXTHIF_Msk) 
    {
        /* Fill 2 word data when it is Tx threshold interrupt */
        for(i = 0; i < 2; i++)
        {
            /* Check buffer empty */
            if ((g_u32PlayPos_Out != g_u32PlayPos_In) && g_u8PlayEn)
            {
                /* Check ring buffer trun around */
                u32Idx = g_u32PlayPos_Out + 1;

                if (u32Idx >= BUF_LEN)
                    u32Idx = 0;

                /* Play to I2S */
                SPII2S_WRITE_TX_FIFO(SPI0, g_au32PcmPlayBuf[u32Idx]);

                /* Update OUT index */
                g_u32PlayPos_Out = u32Idx;
            }
            else
            {
                /* Fill 0x0 when buffer is empty */
                SPII2S_WRITE_TX_FIFO(SPI0, 0x00);

                /* Buffer underrun. Disable play */
                g_u8PlayEn = 0;
            }
        }
    }

    if (u32I2SIntFlag & SPI_I2SSTS_RXTHIF_Msk) {
        /* one word stores the data which are 16 bits and two channel */
        if ((g_u32RecPos < sizeof(g_au32PcmRecBuf) / 4) && g_u8RecEn)
        {
            /*16-Bit*/
            g_au32PcmRecBuf[g_u32RecPos    ] = SPII2S_READ_RX_FIFO(SPI0);
            g_au32PcmRecBuf[g_u32RecPos + 1] = SPII2S_READ_RX_FIFO(SPI0);
            g_u32RecPos += 2;
        }
        else
        {
            /*16-Bit*/
            SPII2S_READ_RX_FIFO(SPI0);
            SPII2S_READ_RX_FIFO(SPI0);
        }
    }
}

/**
  * @brief  SendRecData, prepare the record data for next ISO transfer.
  * @param  None.
  * @retval None.
  */
void UAC_SendRecData(void)
{
    short *p16Buf;
    short *p16Src;
    uint32_t u32Size;
    int32_t i;

    /* Get the address in USB buffer */
    p16Buf = (short *)((uint32_t)USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2));

    p16Src = (short *)g_au32PcmRecBuf;

    u32Size = g_u32RecPos * 4;

    if(u32Size > EP2_MAX_PKT_SIZE) u32Size = EP2_MAX_PKT_SIZE;

    /* Audio data is resampled to 48MHz (Audio Codec is fixed to 48MHz) */
    /* Do Resample operation to data in USB buffer to SRAM buffer */

    if(REC_CHANNELS == 2)
    {
        Resamples(E_RS_REC_CH0,     p16Src, REC_CHANNELS, srt[g_resample_rec_s_idx].count,     g_a16AudioTmpBuf0, g_resample_rec_s_idx);

        Resamples(E_RS_REC_CH1, &p16Src[1], REC_CHANNELS, srt[g_resample_rec_s_idx].count, &g_a16AudioTmpBuf0[1], g_resample_rec_s_idx);
    }
    else
        Resamples(E_RS_REC_CH0,     p16Src, REC_CHANNELS, srt[g_resample_rec_s_idx].count,     p16Buf, g_resample_rec_s_idx);

    USBD_MemCopy((uint8_t *)p16Buf, (uint8_t *)g_a16AudioTmpBuf0, g_rec_max_packet_size);

    /* Trigger ISO IN */
    USBD_SET_PAYLOAD_LEN(EP2, g_rec_max_packet_size);

    __set_PRIMASK(1);

    /*g_au32PcmRecBuf is not ring buffer. We need to copy remained data to buffer front */
    g_u32RecPos -= (u32Size / 4);
    if(g_u32RecPos)
    {
        for(i = 0; i < g_u32RecPos; i++)
            g_au32PcmRecBuf[i] = g_au32PcmRecBuf[i + u32Size / 4];
    }
    __set_PRIMASK(0);
}

/**
  * @brief  UAC_DeviceEnable. To enable the device to play or record audio data.
  * @param  u8Object: To select the device, UAC_MICROPHONE or UAC_SPEAKER.
  * @retval None.
  */
void UAC_DeviceEnable(uint8_t u8Object)
{
    if(u8Object == UAC_MICROPHONE)
    {
        /* Enable record hardware */
        g_u8RecEn = 1;

        if(g_u8RecEn == 0)
        {
            /* Reset record buffer */
            memset(g_au32PcmRecBuf, 0, sizeof(g_au32PcmRecBuf));
            g_u32RecPos = 0;
        }
    }
    else
    {
        /* Eanble play hardware */
        /* Reset Play buffer */
        if(g_u8PlayEn == 0)
        {
            /* Fill 0x0 to buffer before playing for buffer operation smooth */
            memset(g_au32PcmPlayBuf, 0, sizeof(g_au32PcmPlayBuf));
            g_u32PlayPos_In = BUF_LEN / 2;
            g_u32PlayPos_Out = 0;
            g_u8PlayEn = 1;
        }
    }
}


/**
  * @brief  UAC_DeviceDisable. To disable the device to play or record audio data.
  * @param  u8Object: To select the device, UAC_MICROPHONE or UAC_SPEAKER.
  * @retval None.
  */
void UAC_DeviceDisable(uint8_t u8Object)
{
    if(u8Object ==  UAC_MICROPHONE)
    {
        /* Disable record hardware/stop record */
        g_u8RecEn = 0;
    }
    else
    {
        /* Disable play hardware/stop play */
        g_u8PlayEn = 0;
    }
}

#if CRYSTAL_LESS

#define TRIM_INIT           (SYS_BASE+0x118)
int volatile complete = 0;
int volatile complete1 = 0;

/* Timer 0 is working in event count mode, and Timer 1 in capture mode, so we read the */
/* capture value from Timer 1, _not_ timer 0. */
uint32_t u32CapSOF;
void TMR1_IRQHandler(void)
{
    /* TIMER1 clock source = PCLK0 = HCLK / 2 = HIRC / 2 */
    /* Timer clock is 24 MHz, counter value records the duration for 100 event counts. */
    u32CapSOF = TIMER_GetCounter(TIMER1);
    TIMER_ClearCaptureIntFlag(TIMER1);
    complete = 1;
    complete1 =1;
}

void AdjFreq1(void)
{
    uint32_t u32Size;
    static int32_t i32PreFlag = 0;
    static int32_t i32Cnt = 0;

    /* Only adjust the frequency when play data */
    if(g_u8PlayEn == 0)
        return;

    /* Get sample size in play buffer */
    u32Size = GetSamplesInBuf();

    if(g_i32AdjFlag == 0)
    {
        /* Check if we need to adjust the frequency when we didn't in adjusting state */
        if(u32Size > (BUF_LEN * 3 / 4))
        {
            /* USB rate > I2S rate. So we increase I2S rate here */
            g_i32AdjFlag = -1;
            dgb_printf("U\n");
        }
        else if(u32Size < (BUF_LEN * 1 / 4))
        {
            /* USB rate < I2S rate. So we decrease I2S rate here */
            g_i32AdjFlag = 1;
            dgb_printf("D\n");
        }
    }
    else
    {
        /* Check if we need to stop adjust the frequency when we are in adjusting state */
        if((g_i32AdjFlag > 0) && (u32Size > BUF_LEN / 2))
        {
            g_i32AdjFlag = 0;
            dgb_printf("N\n");
        }

        if((g_i32AdjFlag < 0) && (u32Size < BUF_LEN / 2))
        {
            g_i32AdjFlag = 0;
            dgb_printf("N\n");
        }
        if(complete1)
        {
            complete1 = 0;
            if(g_i32AdjFlag == -1)
            {
                if((480000 > u32CapSOF)&&(u32CapSOF > 478000))
                {
                    M32(TRIM_INIT) = M32(TRIM_INIT)-1;
                }
            }
            if(g_i32AdjFlag == 1)
            {
                if((482000 > u32CapSOF)&&(u32CapSOF > 480000))
                {
                    M32(TRIM_INIT) = M32(TRIM_INIT)+1;
                }
            }
        }
    }

    /* Show adjustment, buffer, volume status */
    if(i32PreFlag != g_i32AdjFlag || (i32Cnt++ > 40000))
    {
        i32PreFlag = g_i32AdjFlag;
        dgb_printf("%d %d %d %d %d\n", g_i32AdjFlag, u32Size, g_usbd_PlayVolumeL, g_usbd_RecVolumeL, u32CapSOF);
        i32Cnt = 0;
    }
    if(complete)
    {
        complete=0;
        TIMER_EnableFreqCounter(TIMER0, 0, 0, TRUE);
    }  
}
#else
void AdjustCodecPll(RESAMPLE_STATE_T r)
{
    static uint16_t tb[3][3] = {
        {0x00C, 0x093, 0x0E9},
        {0x00E, 0x1D2, 0x1E3},
        {0x009, 0x153, 0x1EF}
    };
    static RESAMPLE_STATE_T current = E_RS_NONE;
    int i, s;

    if(r == current)
        return;
    else
        current = r;

    switch(r)
    {
        case E_RS_UP:
            s = 1;
            break;
        case E_RS_DOWN:
            s = 2;
            break;
        case E_RS_NONE:
        default:
            s = 0;
    }

    for(i = 0; i < 3; i++)
        I2C_WriteNAU8822(37 + i, tb[s][i]);
}

void AdjFreq(void)
{
    uint32_t u32Size;
    static int32_t i32PreFlag = 0;
    static int32_t i32Cnt = 0;

    /* Only adjust the frequency when play data */
    if(g_u8PlayEn == 0)
        return;

    /* Get sample size in play buffer */
    u32Size = GetSamplesInBuf();

    if(g_i32AdjFlag == 0)
    {
        /* Check if we need to adjust the frequency when we didn't in adjusting state */
        if(u32Size > (BUF_LEN * 3 / 4))
        {
            /* USB rate > I2S rate. So we increase I2S rate here */
            AdjustCodecPll(E_RS_UP);
            g_i32AdjFlag = -1;
            dgb_printf("U\n");
        }
        else if(u32Size < (BUF_LEN * 1 / 4))
        {
            /* USB rate < I2S rate. So we decrease I2S rate here */
            AdjustCodecPll(E_RS_DOWN);
            g_i32AdjFlag = 1;
            dgb_printf("D\n");
        }
    }
    else
    {
        /* Check if we need to stop adjust the frequency when we are in adjusting state */
        if((g_i32AdjFlag > 0) && (u32Size > BUF_LEN / 2))
        {
            AdjustCodecPll(E_RS_NONE);
            g_i32AdjFlag = 0;
            dgb_printf("N\n");
        }

        if((g_i32AdjFlag < 0) && (u32Size < BUF_LEN / 2))
        {
            AdjustCodecPll(E_RS_NONE);
            g_i32AdjFlag = 0;
            dgb_printf("N\n");
        }
    }

    /* Show adjustment, buffer, volume status */
    if(i32PreFlag != g_i32AdjFlag || (i32Cnt++ > 40000))
    {
        i32PreFlag = g_i32AdjFlag;
        dgb_printf("%d %d %d %d\n", g_i32AdjFlag, u32Size, g_usbd_PlayVolumeL, g_usbd_RecVolumeL);
        i32Cnt = 0;
    }
}
#endif

void VolumnControl(void)
{
    static uint8_t volatile u8PrePlayMute = 0;
    static uint8_t volatile u8PreRecMute = 0;
    static int16_t volatile i16PrePlayVolumeL = 0;
    static int16_t volatile i16PrePlayVolumeR = 0;
    static int16_t volatile i16PreRecVolumeL = 0;
    static int16_t volatile i16PreRecVolumeR = 0;
    uint8_t IsChange = 0;
    uint32_t volatile u32R52, u32R53;
    uint32_t volatile u32R15, u32R16;
    /*
        g_usbd_PlayMute is used for MUTE control. 0 = not MUTE. 1 = MUTE.
        g_usbd_PlayVolumeL is volume of left channel. Range is -32752(0xFFF0) ~ -25504(0xE3A0)
        g_usbd_PlayVolumeR is volume of right channel. Range is -32752(0xFFF0) ~ -25504(0xE3A0)

        NAU8822 LHPGAIN (R52) = -57dB ~ +6dB. Code is 0x0 ~ 0x3F.
        NAU8822 RHPGAIN (R53) = -57dB ~ +6dB. Code is 0x0 ~ 0x3F.
        Play volume mapping to code will be (Volume - 0xE3A0 >> 7) & 0x3F = 0x0 ~ 0x38

        g_usbd_RecVolumeL is volume of left channel. Range is 0(0x0000) ~ 8176(0x1FF0)
        g_usbd_RecVolumeR is volume of right channel. Range is 0(0x0000) ~ 8176(0x1FF0)
        NAU8822 LADCGAIN (R15) = MUTE, -127dB ~ 0dB. Code is 0x0, 0x1 ~ 0xFF.
        NAU8822 RADCGAIN (R16) = MUTE, -127dB ~ 0dB. Code is 0x0, 0x1 ~ 0xFF.
        Record volume mapping to code will be (Volume >> 5)
    */

    u32R52 = 0;
    u32R53 = 0;

    /* Update MUTE and volume to u32R52/53 when MUTE changed for play */   
    if(u8PrePlayMute != g_usbd_PlayMute)
    {
        u8PrePlayMute = g_usbd_PlayMute;
        u32R52 = u32R52 | (g_usbd_PlayMute << 6);
        u32R53 = u32R53 | (g_usbd_PlayMute << 6);
        i16PrePlayVolumeL = g_usbd_PlayVolumeL;
        u32R52 |= ((g_usbd_PlayVolumeL - 0xE3A0) >> 7) & 0x3F;
        i16PrePlayVolumeR = g_usbd_PlayVolumeR;
        u32R53 |= ((g_usbd_PlayVolumeR - 0xE3A0) >> 7) & 0x3F;
        IsChange |= 3;
    }

    /* Update left volume to u32R52 when left volume changed for play */
    if(i16PrePlayVolumeL != g_usbd_PlayVolumeL)
    {
        i16PrePlayVolumeL = g_usbd_PlayVolumeL;
        u32R52 |= ((g_usbd_PlayVolumeL - 0xE3A0) >> 7) & 0x3F;
        IsChange |= 1;
    }

    /* Update right volume to u32R53 when left volume changed for play */
    if(i16PrePlayVolumeR != g_usbd_PlayVolumeR)
    {
        i16PrePlayVolumeR = g_usbd_PlayVolumeR;
        u32R53 |= ((g_usbd_PlayVolumeR - 0xE3A0) >> 7) & 0x3F;
        IsChange |= 2;
    }

    /* Update MUTE and volume to u32R15/16 when MUTE changed for record */
    if(u8PreRecMute != g_usbd_RecMute)
    {
        u8PreRecMute = g_usbd_RecMute;

        if(!g_usbd_RecMute)
        {
            i16PreRecVolumeL = g_usbd_RecVolumeL;
            i16PreRecVolumeR = g_usbd_RecVolumeR;
            u32R15 = 0xFF - ((g_usbd_RecMaxVolume - g_usbd_RecVolumeL) >> 9);
            u32R16 = 0xFF - ((g_usbd_RecMaxVolume - g_usbd_RecVolumeR) >> 9);
        }

        IsChange |= 0xC;
    }

    /* Update left volume to u32R15 when left volume changed for record */
    if(i16PreRecVolumeL != g_usbd_RecVolumeL)
    {
        i16PreRecVolumeL = g_usbd_RecVolumeL;
        u32R15 = 0xFF - ((g_usbd_RecMaxVolume - g_usbd_RecVolumeL) >> 9);
        IsChange |= 4;
    }

    /* Update right volume to u32R16 when left volume changed for record */
    if(i16PreRecVolumeR != g_usbd_RecVolumeR)
    {
        i16PreRecVolumeR = g_usbd_RecVolumeR;
        u32R16 = 0xFF - ((g_usbd_RecMaxVolume - g_usbd_RecVolumeR) >> 9); 
        IsChange |= 8;
    }

    /* Update R52, R53 when MUTE or volume changed */
    if((IsChange & 3) == 3)
    {
        /* Both channels need to be changed */
        I2C_WriteNAU8822(52, u32R52);
        I2C_WriteNAU8822(53, u32R53 | 0x100);
        IsChange ^= 3;
    }
    else if(IsChange & 1)
    {
        /* Only change left channel */
        I2C_WriteNAU8822(52, u32R52 | 0x100);
        IsChange ^= 1;
    }
    else if(IsChange & 2)
    {
        /* Only change right channel */
        I2C_WriteNAU8822(53, u32R53 | 0x100);
        IsChange ^= 2;
    }

    /* Update R15, R16 when MUTE or volume changed */
    if((IsChange & 0xC) == 0xC)
    {
        /* Both channels need to be changed */
        I2C_WriteNAU8822(15, u32R15);
        I2C_WriteNAU8822(16, u32R16 | 0x100);
        IsChange ^= 0xC;
    }
    else if(IsChange & 4)
    {
        /* Only change left channel */
        I2C_WriteNAU8822(15, u32R15 | 0x100);
        IsChange ^= 4;
    }
    else if(IsChange & 8)
    {
        /* Only change right channel */
        I2C_WriteNAU8822(16, u32R16 | 0x100);
        IsChange ^= 8;
    }
}


void SamplingControl(void)
{
#ifdef __FEEDBACK__
    uint8_t *pu8Buf;
#endif 
    if(g_PrePlaySampleRate != g_usbd_PlaySampleRate)
    {
        g_PrePlaySampleRate = g_usbd_PlaySampleRate;

#ifdef PLAY_PIPE_RESAMPLE_ENABLE
        g_resample_play_s_idx = SamplingFactor(g_usbd_PlaySampleRate, SAMPLING_RATE);   /* Audio Codec is fixed to SAMPLING_RATE */

        printf("Play   Freq. %5dHz (%02d) - Src %5dHz to Dest %5dHz\n", g_usbd_PlaySampleRate, g_resample_play_s_idx, srt[g_resample_play_s_idx].s, srt[g_resample_play_s_idx].d);
#else
        g_usbd_CodecSampleRate = g_usbd_PlaySampleRate;

        /* Set NAU8822 codec */
        NAU8822_Setup();

        printf("Set Audio Codec %5dHz\n", g_usbd_PlaySampleRate);
#endif
        if(g_usbd_PlaySampleRate == AUDIO_RATE_441K)
            g_play_max_packet_size = 180;
        else
            g_play_max_packet_size = g_usbd_PlaySampleRate * PLAY_CHANNELS * 2 / 1000;

#ifdef __FEEDBACK__
    /* Feedback Endpoint */
        pu8Buf = (uint8_t *)((uint32_t)USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP4));

        g_u32SampleRate = g_usbd_PlaySampleRate / 1000;

        g_play_len_frame = g_usbd_PlaySampleRate / 1000;

        /* Prepare the data to USB IN buffer */
        *pu8Buf++ = 0x00;
        *pu8Buf++ = (g_u32SampleRate & 0x3) << 6;
        *pu8Buf = (g_u32SampleRate & 0xFC) >> 2;

        /* Trigger ISO IN */
        USBD_SET_DATA1(EP4);

        USBD_SET_PAYLOAD_LEN(EP4, 3);

    /* End of Feedback Endpoint */
#endif
    }
#ifdef PLAY_PIPE_RESAMPLE_ENABLE
    if(g_PreRecSampleRate != g_usbd_RecSampleRate)
#else
    /* Change Record Sampling Rate or Codec Sampling Rate */
    if(g_PreRecSampleRate != g_usbd_RecSampleRate || g_PreCodecSampleRate != g_usbd_CodecSampleRate)
#endif
    { 
        g_PreRecSampleRate = g_usbd_RecSampleRate;

        g_rec_max_packet_size = g_usbd_RecSampleRate * REC_CHANNELS * 2 / 1000;

#ifdef PLAY_PIPE_RESAMPLE_ENABLE
        g_resample_rec_s_idx = SamplingFactor(SAMPLING_RATE, g_usbd_RecSampleRate);             /* Audio Codec is fixed to SAMPLING_RATE */
#else
        g_PreCodecSampleRate = g_usbd_CodecSampleRate;

        g_resample_rec_s_idx = SamplingFactor(g_usbd_CodecSampleRate, g_usbd_RecSampleRate);    /* Audio Codec is g_usbd_CodecSampleRate */
#endif
        printf("Record Freq. %5dHz (%02d) - Src %5dHz to Dest %5dHz\n\n", g_usbd_RecSampleRate, g_resample_rec_s_idx, srt[g_resample_rec_s_idx].s, srt[g_resample_rec_s_idx].d);
    }
}

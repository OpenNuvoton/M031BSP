/******************************************************************************
 * @file     hid.c
 * @version  V0.10
 * @brief
 *           Demonstrate how to implement a USB hid class device.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "usbd_audio.h"

const SR_T srt[] = {
    {  683,  8000, 96000,  8},    /* resamle factor, source sampling rate, destination sampling rate, source sample count */
    { 1365,  8000, 48000,  8},
    { 1486,  8000, 44100,  8},
    { 2048,  8000, 32000,  8},
    { 4096,  8000, 16000,  8},
    { 8192,  8000,  8000,  8},  
    { 1365, 16000, 96000, 16},
    { 2731, 16000, 48000, 16},
    { 2972, 16000, 44100, 16},
    { 4096, 16000, 32000, 16},
    { 8192, 16000, 16000, 16},
    {16384, 16000,  8000, 16},
    { 2731, 32000, 96000, 32},
    { 5461, 32000, 48000, 32},
    { 5944, 32000, 44100, 32},
    { 8192, 32000, 32000, 32},
    {16384, 32000, 16000, 32},
    {32768, 32000,  8000, 32},
    { 3763, 44100, 96000, 44},
    { 7526, 44100, 48000, 44},
    { 8192, 44100, 44100, 44},
    {11289, 44100, 32000, 44},
    {22579, 44100, 16000, 44},
    {45158, 44100,  8000, 44},
    { 4096, 48000, 96000, 48},
    { 8192, 48000, 48000, 48},
    { 8916, 48000, 44100, 48},
    {12288, 48000, 32000, 48},
    {24576, 48000, 16000, 48},
    {49152, 48000,  8000, 48},
    { 8192, 96000, 96000, 96},
    {16384, 96000, 48000, 96},
    {17833, 96000, 44100, 96},
    {24576, 96000, 32000, 96},
    {49152, 96000, 16000, 96},
    {98304, 96000,  8000, 96}
};

int SamplingFactor(uint32_t u32SrceRate, uint32_t u32DestRate)
{
    uint32_t i ,n = 0;

    for(i = n; i < sizeof(srt)/sizeof(SR_T);i++)
    {
        if(srt[i].s == u32SrceRate && srt[i].d == u32DestRate)
            return i;
    }
    return 25; /* 48K to 48K */
}

/*
int Resamples(RESAMPLE_MODE_T mode, short *x, int ch_num, int samples, short *y, int s_idx)
    
    mode    : E_RS_REC_CH0 / E_RS_REC_CH1 / E_RS_PLAY_CH0 / E_RS_PLAY_CH1
    x       : point to input buffer of 16-bit PCM samples
    ch_num  :
    samples : samples in input buffer
    y       : point to output buffer
    s_idx   : resample index
              0 :    8K to   96K
              1 :    8K to   48K
              2 :    8K to 44.1K
              3 :    8K to   32K
              4 :    8K to   16K
              5 :    8K to    8K
              6 :   16K to   96K
              7 :   16K to   48K
              8 :   16K to 44.1K
              9 :   16K to   32K
             10 :   16K to   16K
             11 :   16K to    8K
             12 :   32K to   96K
             13 :   32K to   48K
             14 :   32K to 44.1K
             15 :   32K to   32K
             16 :   32K to   16K
             17 :   32K to    8K
             18 : 44.1K to   96K
             19 : 44.1K to   48K
             20 : 44.1K to 44.1K
             21 : 44.1K to   32K
             22 : 44.1K to   16K
             23 : 44.1K to    8K
             24 :   48K to   96K
             25 :   48K to   48K
             26 :   48K to 44.1K
             27 :   48K to   32K
             28 :   48K to   16K
             29 :   48K to    8K
             30 :   96K to   96K
             31 :   96K to   48K
             32 :   96K to 44.1K
             33 :   96K to   32K
             34 :   96K to   16K
             35 :   96K to    8K
    return  : samples in output buffer
Example:
    Resample 32kHz to 48kHz
    outSamples = Resamples(E_RS_PLAY_CH0, x    , 32,     y, 3);
    outSamples = Resamples(E_RS_PLAY_CH1, x + 1, 32, y + 1, 3);
*/ 
int Resamples(RESAMPLE_MODE_T mode, short *x, int ch_num, int samples, short *y, int s_idx)
{
    int i;
    static int px_play_ch0 = 0, px_play_ch1 = 0, px_rec_ch0 = 0, px_rec_ch1 = 0;
    static int yt_play_ch0 = 0, yt_play_ch1 = 0, yt_rec_ch0 = 0, yt_rec_ch1 = 0;
    static int xt_play_ch0 = 0, xt_play_ch1 = 0, xt_rec_ch0 = 0, xt_rec_ch1 = 0;
    int px = 0;
    int yt = 0;
    int xt = 0;
    int idx;
    int offset;
    int ii;
    int f, r,s;

    switch(mode)
    {
        case E_RS_REC_CH0:
            px = px_rec_ch0;
            yt = yt_rec_ch0;
            xt = xt_rec_ch0;
            break;
        case E_RS_REC_CH1:
            px = px_rec_ch1;
            yt = yt_rec_ch1;
            xt = xt_rec_ch1;
            break;
        case E_RS_PLAY_CH0:
            px = px_play_ch0;
            yt = yt_play_ch0;
            xt = xt_play_ch0;
            break;
        case E_RS_PLAY_CH1:
            px = px_play_ch1;
            yt = yt_play_ch1;
            xt = xt_play_ch1;
            break;
    }
    r = srt[s_idx].r;
    s = srt[s_idx].s;

    offset = xt >> 13;
    i = 0;
    do
    {
        idx = yt >> 13;
        f = yt & ((1 << 13) - 1);
        ii = idx - offset;
        if (ii < 0)
        {
            y[i * ch_num] = ((8192 - f)*px + x[0] * f) >> 13;
        }
        else
        {
            y[i * ch_num] = ((8192 - f)*x[ii * ch_num] + x[ii * ch_num + ch_num] * f) >> 13;
        }

        i++;
        yt += r;
    } while (yt < (xt + ((samples - 1) << 13)));

    px = x[(samples - 1) * ch_num];
    xt += (samples << 13);

    if (xt > (s << 13))
    {
        xt = 0;
        yt = 0;
    }

    switch(mode)
    {
        case E_RS_REC_CH0:
            px_rec_ch0 = px;
            yt_rec_ch0 = yt;
            xt_rec_ch0 = xt;
            break;
        case E_RS_REC_CH1:
            px_rec_ch1 = px;
            yt_rec_ch1 = yt;
            xt_rec_ch1 = xt;
            break;
        case E_RS_PLAY_CH0:
            px_play_ch0 = px;
            yt_play_ch0 = yt;
            xt_play_ch0 = xt;
            break;
        case E_RS_PLAY_CH1:
            px_play_ch1 = px;
            yt_play_ch1 = yt;
            xt_play_ch1 = xt;
            break;
    }

    return i;
}


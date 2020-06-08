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

#ifdef __HID__

uint8_t volatile g_u8EP4Ready = 0;

void GPIO_Init(void)
{
    /* PA7 / PB0 / PB1 / PC2 / PC3 /PC5  */
    PA->MODE |= 0xC000;
    PB->MODE |= 0x000F;
    PC->MODE |= 0x0FF0;

    PA->INTSRC |= 0x80;
    PB->INTSRC |= 0x03;
    PC->INTSRC |= 0x3C;

    PA->INTEN |= 0x80 | (0x80 << 16);
    PB->INTEN |= 0x03 | (0x03 << 16);
    PC->INTEN |= 0x3C | (0x3C << 16);

    PA->DBEN |= 0x80;
    PB->DBEN |= 0x03;
    PC->DBEN |= 0x3C;      /* Enable key debounce */

    GPIO->DBCTL = 0x16;    /* Debounce time is about 6ms */
}

void HID_UpdateHidData(void)
{
    uint8_t *buf;
    uint32_t u32RegA, u32RegB, u32RegC;
#ifndef __JOYSTICK__
    uint32_t static u32PreRegA, u32PreRegB, u32PreRegC;
#endif
    int32_t volatile i;

    if(g_u8EP4Ready)
    {
        buf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP4));

        /*
           Key definition:
             Down          PC2
             right         PC3
             up            PC5
             left          PA7
             right key     PB0
             left key      PB1
        */
        u32RegC = PC->PIN & 0x3C;
        u32RegA = PA->PIN & 0x80;
        u32RegB = PB->PIN & 0x3;

#ifdef __JOYSTICK__
        for(i = 0; i < 5; i++)
            buf[i] = 0x7F;

        buf[5] = 0x0F;    /* Hat switch */
        buf[6] = 0x00;
        buf[7] = 0x00;
#elif defined  __MEDIAKEY__
        for(i = 0; i < 8; i++)
            buf[i] = 0;
#else
        for(i = 0; i < 8; i++)
            buf[i] = 0;
#endif

#ifdef __JOYSTICK__
/* Input Report
 +--------+--------+--------+--------+--------+------------------+------------------+--------+
 | Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 |      Byte 5      |      Byte 6      | Byte 7 |
 +--------+--------+--------+--------+--------+-------+----------+------------------+--------+
 |        |        |        |        |        | Button|Hat switch|      Button      |        |
 | X-axis | Y-axis | Z-axis | Z-axis |   Rz   |-------+----------+------------------|  Pad   |
 |        |        |        |        |        |4|3|2|1|   0xF    |12|11|10|9|8|7|6|5|        |
 +--------+--------+--------+--------+--------+-------+----------+------------------+--------+
*/
    /* Byte 1 */
        if((u32RegC & (1<<5)) == 0)          /* PC5 - Up       */
            buf[1] = 0x00;
        if((u32RegC & (1<<2)) == 0)          /* PC2 - Down     */
            buf[1] = 0xFF;
    /* Byte 0 */
        if((u32RegA & (1<<7)) == 0)          /* PA7 - Left     */
            buf[0] = 0x00;
        if((u32RegC & (1<<3)) == 0)          /* PC3 - Right    */
            buf[0] = 0xFF;
    /* Byte 5 */
        if((u32RegB & (1<<1)) == 0)          /* PB1 - Button 1 */ 
            buf[5] |= 0x10;
        if((u32RegB & (1<<0)) == 0)          /* PB0 - Button 2 */
            buf[5] |= 0x20;

#elif defined  __MEDIAKEY__
/* Input Report
 +--------+---------+--------+--------+----------+--------+----------+----------+------+
 |        |   BIT7  |  BIT6  |  BIT5  |   BIT4   |  BIT3  |   BIT2   |   BIT1   | BIT0 |
 +--------+---------+--------+--------+----------+--------+----------+----------+------+
 |        |                                               |  Volume  |  Volume  |      |
 | Byte 0 |                     Pad                       |Decrement |Increment | Mute |
 +--------+---------+--------+--------+----------+--------+----------+----------+------+
 |        |         |        |        |   Scan   |  Scan  |          |          |      | 
 | Byte 1 |   Fast  | Rewind | Record | Previous |  Next  |Play/Pause|   Stop   | Play |
 |        | Forward |        |        |   Track  |  Track |          |          |      |
 +--------+---------+--------+--------+----------+--------+----------+----------+------+
 |  Byte  |                                                                            |
 | 2 ~ 7  |                                 Pad                                        |
 +--------+----------------------------------------------------------------------------+
*/
        buf[0] = 0;
        buf[1] = 0;

        if(u32RegB != u32PreRegB)
        {
    /* Byte 1 */
            if((u32RegB & (1<<1)) == 0)          /* PB1 - Button 1             */
                buf[1] |= HID_CTRL_PAUSE;        /* Play/Pause - 0x04          */ 
            u32PreRegB = u32RegB;
        }
        if(u32RegC != u32PreRegC)
        {
    /* Byte 1 */
            if((u32RegC & (1<<3)) == 0)          /* PC3 - Right                */
                buf[1] |= HID_CTRL_NEXT;         /* Scan Next Track - 0x08     */
    /* Byte 0 */
            if((u32RegC & (1<<5)) == 0)          /* PC5 - Up                   */
                buf[0] |= HID_CTRL_VOLUME_INC;   /* Volume Increment - 0x02    */
            if((u32RegC & (1<<2)) == 0)          /* PC2 - Down                 */
                buf[0] |= HID_CTRL_VOLUME_DEC;   /* Volume Decrement -0x04     */
            u32PreRegC = u32RegC;
        }
        if(u32RegA != u32PreRegA)
        {
    /* Byte 1 */
            if((u32RegA & (1<<7)) == 0)          /* PA7 - Left                 */
                buf[1] |= HID_CTRL_PREVIOUS;     /* Scan Previous Track - 0x10 */
            u32PreRegA = u32RegA;
        }
#endif
        g_u8EP4Ready = 0;

        /* Set transfer length and trigger IN transfer */
        USBD_SET_PAYLOAD_LEN(EP4, 8);
    }
}
#endif

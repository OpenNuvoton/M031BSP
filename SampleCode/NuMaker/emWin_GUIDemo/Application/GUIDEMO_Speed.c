/*********************************************************************
*                 SEGGER Software GmbH                               *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2018  SEGGER Microcontroller GmbH                *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V5.46 - Graphical user interface for embedded applications **
All  Intellectual Property rights in the Software belongs to  SEGGER.
emWin is protected by  international copyright laws.  Knowledge of the
source code may not be used to write a similar product. This file may
only be used in accordance with the following terms:

The  software has  been licensed by SEGGER Software GmbH to Nuvoton Technology Corporation
at the address: No. 4, Creation Rd. III, Hsinchu Science Park, Taiwan
for the purposes  of  creating  libraries  for its
Arm Cortex-M and  Arm9 32-bit microcontrollers, commercialized and distributed by Nuvoton Technology Corporation
under  the terms and conditions  of  an  End  User
License  Agreement  supplied  with  the libraries.
Full source code is available at: www.segger.com

We appreciate your understanding and fairness.
----------------------------------------------------------------------
Licensing information
Licensor:                 SEGGER Software GmbH
Licensed to:              Nuvoton Technology Corporation, No. 4, Creation Rd. III, Hsinchu Science Park, 30077 Hsinchu City, Taiwan
Licensed SEGGER software: emWin
License number:           GUI-00735
License model:            emWin License Agreement, signed February 27, 2018
Licensed platform:        Cortex-M and ARM9 32-bit series microcontroller designed and manufactured by or for Nuvoton Technology Corporation
----------------------------------------------------------------------
Support and Update Agreement (SUA)
SUA period:               2018-03-26 - 2019-03-27
Contact to extend SUA:    sales@segger.com
----------------------------------------------------------------------
File        : GUIDEMO_Speed.c
Purpose     : Speed demo
----------------------------------------------------------------------
*/

#include "GUIDEMO.h"

#if (SHOW_GUIDEMO_SPEED)

#include <stdlib.h>  // rand()

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/
static const GUI_COLOR _aColor[8] =
{
    0x000000,
    0x0000FF,
    0x00FF00,
    0x00FFFF,
    0xFF0000,
    0xFF00FF,
    0xFFFF00,
    0xFFFFFF
};

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/
/*********************************************************************
*
*       _GetPixelsPerSecond
*/
static U32 _GetPixelsPerSecond(void)
{
    GUI_COLOR BkColor;
    GUI_COLOR Color;
    I32       PixelsPerSecond;
    I32       PixelCnt;
    I32       t0;
    I32       t;
    U32       xSize;
    U32       ySize;
    U32       Cnt;
    U32       x0;
    U32       x1;
    U32       y0;
    U32       y1;

    //
    // Find an area which is not obstructed by any windows
    //
    xSize   = LCD_GetXSize();
    ySize   = LCD_GetYSize();
    Cnt     = 0;
    x0      = 0;
    x1      = xSize - 1;
    y0      = 65;
    y1      = ySize - 60 - 1;
    Color   = GUI_GetColor();
    BkColor = GUI_GetBkColor();
    GUI_SetColor(BkColor);
    //
    // Repeat fill as often as possible in 100 ms
    //
    t0 = GUIDEMO_GetTime();
    do
    {
        GUI_FillRect(x0, y0, x1, y1);
        Cnt++;
        t = GUIDEMO_GetTime();
    }
    while((t - (t0 + 100)) <= 0);
    //
    // Compute result
    //
    t -= t0;
    PixelCnt = (x1 - x0 + 1) * (y1 - y0 + 1) * Cnt;
    PixelsPerSecond = PixelCnt / t * 1000;
    GUI_SetColor(Color);
    return PixelsPerSecond;
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       GUIDEMO_Speed
*/
void GUIDEMO_Speed(void)
{
#if GUI_SUPPORT_TOUCH
    GUI_PID_STATE State;
#endif
    GUI_RECT        ClipRect;
    GUI_RECT        Rect;
    char            acText[40] = { 0 };
    U32             PixelsPerSecond;
    int             aColorIndex[8];
    int             TimeStart;
    int             vySize;
    int             xSize;
    int             ySize;
    int             i;

    GUIDEMO_ConfigureDemo("High speed", "Multi layer clipping\nHighly optimized drivers", 0);
    xSize  = LCD_GetXSize();
    ySize  = LCD_GetYSize();
    vySize = LCD_GetVYSize();
    if(vySize > ySize)
    {
        ClipRect.x0 = 0;
        ClipRect.y0 = 0;
        ClipRect.x1 = xSize;
        ClipRect.y1 = ySize;
        GUI_SetClipRect(&ClipRect);
    }
    for(i = 0; i < 8; i++)
    {
        aColorIndex[i] = GUI_Color2Index(_aColor[i]);
    }
    TimeStart = GUIDEMO_GetTime();
    for(i = 0; ((GUIDEMO_GetTime() - TimeStart) < 5000) && (GUIDEMO_CheckCancel() == 0); i++)
    {
        GUI_SetColorIndex(aColorIndex[i & 7]);
        //
        // Calculate random positions
        //
        Rect.x0 = rand() % xSize - xSize / 2;
        Rect.y0 = rand() % ySize - ySize / 2;
        Rect.x1 = Rect.x0 + 20 + rand() % xSize;
        Rect.y1 = Rect.y0 + 20 + rand() % ySize;
        GUI_FillRect(Rect.x0, Rect.y0, Rect.x1, Rect.y1);
        //
        // Clip rectangle to visible area and add the number of pixels (for speed computation)
        //
        if(Rect.x1 >= xSize)
        {
            Rect.x1 = xSize - 1;
        }
        if(Rect.y1 >= ySize)
        {
            Rect.y1 = ySize - 1;
        }
        if(Rect.x0 < 0)
        {
            Rect.x0 = 0;
        }
        if(Rect.y1 < 0)
        {
            Rect.y1 = 0;
        }
        //
        // There is no control window. A simple click on any position has to skip the demo.
        //
#if GUI_SUPPORT_TOUCH
        GUI_PID_GetState(&State);
        if(State.Pressed)
        {
            break;
        }
#endif
    }
    GUIDEMO_NotifyStartNext();
    PixelsPerSecond = _GetPixelsPerSecond();
    GUI_SetClipRect(NULL);
    GUIDEMO_AddStringToString(acText, "Pixels/sec: ");
    GUIDEMO_AddIntToString(acText, PixelsPerSecond);
    GUIDEMO_DrawBk();
    GUI_SetColor(GUI_WHITE);
    GUI_SetTextMode(GUI_TM_TRANS);
#ifdef __DEMO_160x128__
    GUI_SetFont(&GUI_Font10S_ASCII);
#else
    GUI_SetFont(&GUI_FontRounded22);
#endif
    GUI_DispStringHCenterAt(acText, xSize / 2, (ySize - GUI_GetFontSizeY()) / 2);
    GUIDEMO_ConfigureDemo(NULL, NULL, GUIDEMO_SHOW_CURSOR | GUIDEMO_SHOW_CONTROL);
    GUIDEMO_Delay(4000);
}

#else

void GUIDEMO_Speed_C(void);
void GUIDEMO_Speed_C(void) {}

#endif  // SHOW_GUIDEMO_SPEED

/*************************** End of file ****************************/

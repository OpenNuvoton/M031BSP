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
File        : AA_Text.c
Purpose     : Shows text with different antialiasing qualities
----------------------------------------------------------------------
*/

#include "GUIDEMO.h"

#if (SHOW_GUIDEMO_AATEXT)

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/
#define BORDER 3

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/
/*********************************************************************
*
*       _DrawAlphaCircles
*/
static void _DrawAlphaCircles(int mx, int my, int r, int a, int FactorAA)
{
    int Index;
    int x;
    int y;
    I32 SinHQ;
    I32 CosHQ;
    U32 a1000;
    U32 i;
    const GUI_COLOR aColor[] =
    {
#if (GUI_USE_ARGB)
        0x3F38FF0F,
        0x3F8EFF00,
        0x3FEAFC00,
        0x3FFFB400,
        0x3FFF4E00,
        0x3FFF0413,
        0x3FFF006E,
        0x3FFF00D2,
        0x3FD200FF,
        0x3F6E00FF,
        0x3F1304FF,
        0x3F004EFF,
        0x3F00B4FF,
        0x3F00FCEA,
        0x3F00FF8E,
        0x3F0FFF38
#else
        0xC00FFF38,
        0xC000FF8E,
        0xC000FCEA,
        0xC000B4FF,
        0xC0004EFF,
        0xC01304FF,
        0xC06E00FF,
        0xC0D200FF,
        0xC0FF00D2,
        0xC0FF006E,
        0xC0FF0413,
        0xC0FF4E00,
        0xC0FFB400,
        0xC0EAFC00,
        0xC08EFF00,
        0xC038FF0F
#endif
    };

    mx    *= FactorAA;
    my    *= FactorAA;
    r     *= FactorAA;
    a1000  = a * -1000;
    GUI_AA_EnableHiRes();
    GUI_AA_SetFactor(FactorAA);
    for(i = 0, Index = 0; i < 360000; i += 22500, Index++)
    {
        SinHQ = GUI__SinHQ(i + a1000);
        CosHQ = GUI__CosHQ(i + a1000);
        x     = SHIFT_RIGHT_16(r * CosHQ);
        y     = SHIFT_RIGHT_16(r * SinHQ);
        GUI_SetColor(
            aColor[Index % (int)GUI_COUNTOF(aColor)]);
        GUI_AA_FillCircle(mx + x, my + y, r);
    }
    GUI_AA_DisableHiRes();
}

/*********************************************************************
*
*       _DrawSample
*/
static void _DrawSample(GUI_RECT Rect, const GUI_FONT * pFont, const char * pText)
{
    GUI_RECT CurrentRect;
    int      yDistDiv3;

    Rect.x0 += BORDER;
    Rect.y0 += BORDER;
    Rect.x1 -= BORDER;
    Rect.y1 -= BORDER;
    yDistDiv3      = (Rect.y1 - Rect.y0) / 3;
    CurrentRect.x0 = Rect.x0;
    CurrentRect.y0 = Rect.y0;
    CurrentRect.x1 = Rect.x0 + 59;
    CurrentRect.y1 = Rect.y0 + 3 * yDistDiv3;
    //
    // Display info text
    //
    GUI_SetFont(GUI_FONT_13_ASCII);
    GUI_SetColor(GUI_WHITE);
    GUI_DispStringInRectWrap(pText, &CurrentRect, GUI_TA_HCENTER | GUI_TA_VCENTER, GUI_WRAPMODE_WORD);
    //
    // Alpha circles
    //
    GUI_MoveRect(&CurrentRect, 63, 0);
    GUI_SetColor(GUI_BLACK);
    GUI_FillRectEx(&CurrentRect);
    GUI_SetClipRect(&CurrentRect);
    _DrawAlphaCircles((CurrentRect.x0 + CurrentRect.x1) / 2, (CurrentRect.y0 + CurrentRect.y1) / 2, 35, 0, 4);
    GUI_SetClipRect(NULL);
    GUI_SetColor(GUI_WHITE);
    GUI_SetFont(pFont);
    CurrentRect.y1 = CurrentRect.y0 + yDistDiv3;
    GUI_DispStringInRect("ABC", &CurrentRect, GUI_TA_HCENTER | GUI_TA_VCENTER);
    GUI_MoveRect(&CurrentRect, 0, yDistDiv3);
    GUI_DispStringInRect("ABC", &CurrentRect, GUI_TA_HCENTER | GUI_TA_VCENTER);
    GUI_MoveRect(&CurrentRect, 0, yDistDiv3);
    GUI_DispStringInRect("ABC", &CurrentRect, GUI_TA_HCENTER | GUI_TA_VCENTER);
    //
    // Black to white gradient
    //
    CurrentRect.y0 = Rect.y0;
    CurrentRect.y1 = Rect.y0 + 3 * yDistDiv3;
    GUI_MoveRect(&CurrentRect, 63, 0);
    GUI_DrawGradientH(CurrentRect.x0, CurrentRect.y0, CurrentRect.x1, CurrentRect.y1, GUI_BLACK, GUI_WHITE);
    CurrentRect.y1 = CurrentRect.y0 + yDistDiv3;
    GUI_SetColor(GUI_RED);
    GUI_DispStringInRect("ABC", &CurrentRect, GUI_TA_HCENTER | GUI_TA_VCENTER);
    GUI_MoveRect(&CurrentRect, 0, yDistDiv3);
    GUI_SetColor(GUI_GREEN);
    GUI_DispStringInRect("ABC", &CurrentRect, GUI_TA_HCENTER | GUI_TA_VCENTER);
    GUI_MoveRect(&CurrentRect, 0, yDistDiv3);
    GUI_SetColor(GUI_BLUE);
    GUI_DispStringInRect("ABC", &CurrentRect, GUI_TA_HCENTER | GUI_TA_VCENTER);
    //
    // RGB
    //
    CurrentRect.y0 = Rect.y0;
    CurrentRect.y1 = CurrentRect.y0 + yDistDiv3;
    GUI_MoveRect(&CurrentRect, 63, 0);
    GUI_SetColor(GUI_RED);
    GUI_FillRectEx(&CurrentRect);
    GUI_MoveRect(&CurrentRect, 0, yDistDiv3);
    GUI_SetColor(GUI_GREEN);
    GUI_FillRectEx(&CurrentRect);
    GUI_MoveRect(&CurrentRect, 0, yDistDiv3);
    GUI_SetColor(GUI_BLUE);
    GUI_FillRectEx(&CurrentRect);
    GUI_SetColor(GUI_WHITE);
    CurrentRect.y0 = Rect.y0;
    CurrentRect.y1 = CurrentRect.y0 + yDistDiv3;
    GUI_DispStringInRect("ABC", &CurrentRect, GUI_TA_HCENTER | GUI_TA_VCENTER);
    GUI_MoveRect(&CurrentRect, 0, yDistDiv3);
    GUI_DispStringInRect("ABC", &CurrentRect, GUI_TA_HCENTER | GUI_TA_VCENTER);
    GUI_MoveRect(&CurrentRect, 0, yDistDiv3);
    GUI_DispStringInRect("ABC", &CurrentRect, GUI_TA_HCENTER | GUI_TA_VCENTER);
    //
    // RGB gradients
    //
    CurrentRect.y0 = Rect.y0;
    CurrentRect.y1 = CurrentRect.y0 + yDistDiv3;
    GUI_MoveRect(&CurrentRect, 63, 0);
    GUI_DrawGradientV(CurrentRect.x0, CurrentRect.y0, CurrentRect.x1, CurrentRect.y1, GUI_RED,   GUI_BLACK);
    GUI_MoveRect(&CurrentRect, 0, yDistDiv3);
    GUI_DrawGradientV(CurrentRect.x0, CurrentRect.y0, CurrentRect.x1, CurrentRect.y1, GUI_GREEN, GUI_BLACK);
    GUI_MoveRect(&CurrentRect, 0, yDistDiv3);
    GUI_DrawGradientV(CurrentRect.x0, CurrentRect.y0, CurrentRect.x1, CurrentRect.y1, GUI_BLUE,  GUI_BLACK);
    CurrentRect.y0 = Rect.y0;
    CurrentRect.y1 = CurrentRect.y0 + yDistDiv3;
    GUI_SetColor(GUI_WHITE);
    GUI_DispStringInRect("ABC", &CurrentRect, GUI_TA_HCENTER | GUI_TA_VCENTER);
    GUI_MoveRect(&CurrentRect, 0, yDistDiv3);
    GUI_DispStringInRect("ABC", &CurrentRect, GUI_TA_HCENTER | GUI_TA_VCENTER);
    GUI_MoveRect(&CurrentRect, 0, yDistDiv3);
    GUI_DispStringInRect("ABC", &CurrentRect, GUI_TA_HCENTER | GUI_TA_VCENTER);
    //
    // Disable application defined clip rectangle
    //
    GUI_SetClipRect(NULL);
}

/*********************************************************************
*
*       _DrawScreen
*/
static void _DrawScreen(void)
{
    GUI_RECT Rect;
    int      TitleSize;
    int      xSize;
    int      ySize;
    int      xOff;
    int      yOff;

    xSize = LCD_GetXSize();
    ySize = LCD_GetYSize();
    if((xSize < XSIZE_MIN) || (ySize < YSIZE_MIN))
    {
        GUIDEMO_ConfigureDemo("Antialiased text", "This demo requires a screen\r\nresolution of QVGA at least.\r\n\r\nThe next demo will start in a moment...", GUIDEMO_SHOW_CURSOR | GUIDEMO_SHOW_CONTROL);
        return;
    }
    GUIDEMO_ConfigureDemo("Antialiased text", "Output antialiased text\non different backgrounds.", GUIDEMO_SHOW_CURSOR | GUIDEMO_SHOW_CONTROL);
    GUIDEMO_DrawBk();
    GUIDEMO_DispTitle("Antialiased text");
    TitleSize = GUIDEMO_GetTitleSizeY();
    xOff      = (xSize - XSIZE_MIN) / 2;
    yOff      = ((ySize - TitleSize) - (YSIZE_MIN - TitleSize)) / 2;
    //
    // 4 bit anti-aliasing sample
    //
    Rect.x0 = xOff;
    Rect.y0 = TitleSize + yOff;
    Rect.x1 = xSize - xOff;
    Rect.y1 = TitleSize + (ySize - TitleSize) / 2 - 1;
    _DrawSample(Rect, &GUI_FontAA4_32, "Antialiased text\n(4 bpp)");
    //
    // 2 bit anti-aliasing sample
    //
    Rect.y0 = Rect.y1 + 1;
    Rect.y1 = ySize - yOff;
    _DrawSample(Rect, &GUI_FontAA2_32, "Antialiased text\n(2 bpp)");
    GUIDEMO_Wait(4000);
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       GUIDEMO_AntialiasedText
*/
void GUIDEMO_AntialiasedText(void)
{
    unsigned OldAlphaState;

    OldAlphaState = GUI_EnableAlpha(1);
    _DrawScreen();
    GUI_EnableAlpha(OldAlphaState);
}

#else

void GUIDEMO_AntialiasedText_C(void);
void GUIDEMO_AntialiasedText_C(void) {}

#endif  // SHOW_GUIDEMO_AATEXT

/*************************** End of file ****************************/

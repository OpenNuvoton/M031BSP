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
File        : GUIDEMO_ColorBar.c
Purpose     : Draws color bars
----------------------------------------------------------------------
*/

#include "GUIDEMO.h"

#if (SHOW_GUIDEMO_COLORBAR)

/*********************************************************************
*
*       Define
*
**********************************************************************
*/
#define TIME_PAUSE  500
#define TIME_STEP   500
#define TIME_RUN    ((TIME_PAUSE + TIME_STEP) * 6)
#define NUM_COLORS  8

/*********************************************************************
*
*       GUIDEMO_ColorBar
*/
extern U8 g_u8GUIItem;
void GUIDEMO_ColorBar(void)
{
    GUI_COLOR ColorStartBlack;
    GUI_COLOR ColorStartWhite;
    char      acTitle[]        = "Color bar";
    char      acDesc[]         = "emWin features an integrated\ncolor management which automatically finds\nthe best available color for any logical color";
    char      acText[80]       = { 0 };
    int       BitsPerPixel;
    int       NumColors;
    int       TimeStart;
    int       ScreenX0;
    int       ScreenY0;
    int       FontY0;
    int       Index;
    int       xSize;
    int       ySize;
    int       yStep;
    int       Time;
    int       Dir;

    xSize    = LCD_GetXSize();
    ySize    = LCD_GetYSize();
#ifdef __DEMO_160x128__
    ScreenX0 = 30;
#else
    ScreenX0 = 60;
#endif
    ScreenY0 = 60;
    yStep    = (ySize - ScreenY0  * 2) / (NUM_COLORS * 2);
    if(yStep < 10)
    {
        yStep = 10;
    }
    GUIDEMO_ConfigureDemo(acTitle, acDesc, GUIDEMO_SHOW_CURSOR | GUIDEMO_SHOW_CONTROL);
    g_u8GUIItem = 2;
    GUIDEMO_DrawBk();
    //
    // Heading
    //
    GUI_SetColor(GUI_WHITE);
#ifdef __DEMO_160x128__
    GUI_SetFont(&GUI_Font10S_ASCII);
    GUI_DispStringAt("Color bars", 80, 12);
    GUI_SetFont(&GUI_Font10S_ASCII);
#else
    GUI_SetFont(&GUI_FontRounded22);
    GUI_DispStringHCenterAt("Color bars", xSize / 2, 12);
    GUI_SetFont(&GUI_FontSouvenir18);
#endif
    //
    // Colors
    //
    FontY0 = ScreenY0 + ((yStep * 2 - GUI_GetFontDistY()) / 2);
    GUI_DispStringAt("Red",     1, FontY0);
    GUI_DispStringAt("Green",   1, FontY0 + yStep *  2);
    GUI_DispStringAt("Blue",    1, FontY0 + yStep *  4);
#ifdef __DEMO_160x128__
    GUI_SetFont(&GUI_Font10S_ASCII);
#else
    GUI_DispStringAt("Grey",    1, FontY0 + yStep *  6);
    GUI_DispStringAt("Yellow",  1, FontY0 + yStep *  8);
    GUI_DispStringAt("Cyan",    1, FontY0 + yStep * 10);
    GUI_DispStringAt("Magenta", 1, FontY0 + yStep * 12);
    GUI_SetFont(&GUI_Font8_ASCII);
#endif
    //
    // LCD Controller
    //
#ifdef LCD_CONTROLLER
    GUIDEMO_AddStringToString(acText, "LCD Controller: ");
    GUIDEMO_AddStringToString(acText, LCD_CONTROLLER);
    GUI_DispStringAt(acText, 12, ySize - 45);
    GUIDEMO_ClearText(acText);
#endif
    //
    // BPP and number of colors
    //
    BitsPerPixel = LCD_GetBitsPerPixel();
    GUIDEMO_AddIntToString(acText, BitsPerPixel);
    GUIDEMO_AddStringToString(acText, " bpp");
    NumColors = LCD_GetDevCap(LCD_DEVCAP_NUMCOLORS);
    if(NumColors)
    {
        GUIDEMO_AddStringToString(acText, ", ");
        GUIDEMO_AddIntToString(acText, NumColors);
        GUIDEMO_AddStringToString(acText, " colors");
    }
#ifdef __DEMO_160x128__
    GUI_DispStringAt(acText, 12, ySize - 14);
#else
    GUI_DispStringAt(acText, 12, ySize - 25);
#endif
    //
    // Gradients
    //
    TimeStart = GUIDEMO_GetTime();
    while(((GUIDEMO_GetTime() - TimeStart) < TIME_RUN) && (GUIDEMO_CheckCancel() == 0))
    {
        Time  = (GUIDEMO_GetTime() - TimeStart) % ((TIME_PAUSE + TIME_STEP) << 1);
        Dir   = Time / (TIME_PAUSE + TIME_STEP);
        Time -= Dir * (TIME_PAUSE + TIME_STEP);
        GUI_Exec();
        if(Time > TIME_PAUSE)
        {
            continue;
        }
        Index           = ((Time * 0xFF) / TIME_STEP) ^ (Dir * 0xFF);
        ColorStartBlack = 0x000000 + 0x010101 * Index;
        ColorStartWhite = 0xFFFFFF - ColorStartBlack;
        GUI_DrawGradientH(ScreenX0, ScreenY0 + yStep *  0, xSize - ScreenX0, (ScreenY0 + yStep *  1) - 1, GUI_RED,     ColorStartBlack);
        GUI_DrawGradientH(ScreenX0, ScreenY0 + yStep *  1, xSize - ScreenX0, (ScreenY0 + yStep *  2) - 1, GUI_RED,     ColorStartWhite);
        GUI_DrawGradientH(ScreenX0, ScreenY0 + yStep *  2, xSize - ScreenX0, (ScreenY0 + yStep *  3) - 1, GUI_GREEN,   ColorStartBlack);
        GUI_DrawGradientH(ScreenX0, ScreenY0 + yStep *  3, xSize - ScreenX0, (ScreenY0 + yStep *  4) - 1, GUI_GREEN,   ColorStartWhite);
        GUI_DrawGradientH(ScreenX0, ScreenY0 + yStep *  4, xSize - ScreenX0, (ScreenY0 + yStep *  5) - 1, GUI_BLUE,    ColorStartBlack);
        GUI_DrawGradientH(ScreenX0, ScreenY0 + yStep *  5, xSize - ScreenX0, (ScreenY0 + yStep *  6) - 1, GUI_BLUE,    ColorStartWhite);
#ifdef __DEMO_160x128__
#else
        GUI_DrawGradientH(ScreenX0, ScreenY0 + yStep *  6, xSize - ScreenX0, (ScreenY0 + yStep *  7) - 1, GUI_GRAY,    ColorStartBlack);
        GUI_DrawGradientH(ScreenX0, ScreenY0 + yStep *  7, xSize - ScreenX0, (ScreenY0 + yStep *  8) - 1, GUI_GRAY,    ColorStartWhite);
        GUI_DrawGradientH(ScreenX0, ScreenY0 + yStep *  8, xSize - ScreenX0, (ScreenY0 + yStep *  9) - 1, GUI_YELLOW,  ColorStartBlack);
        GUI_DrawGradientH(ScreenX0, ScreenY0 + yStep *  9, xSize - ScreenX0, (ScreenY0 + yStep * 10) - 1, GUI_YELLOW,  ColorStartWhite);
        GUI_DrawGradientH(ScreenX0, ScreenY0 + yStep * 10, xSize - ScreenX0, (ScreenY0 + yStep * 11) - 1, GUI_CYAN,    ColorStartBlack);
        GUI_DrawGradientH(ScreenX0, ScreenY0 + yStep * 11, xSize - ScreenX0, (ScreenY0 + yStep * 12) - 1, GUI_CYAN,    ColorStartWhite);
        GUI_DrawGradientH(ScreenX0, ScreenY0 + yStep * 12, xSize - ScreenX0, (ScreenY0 + yStep * 13) - 1, GUI_MAGENTA, ColorStartBlack);
        GUI_DrawGradientH(ScreenX0, ScreenY0 + yStep * 13, xSize - ScreenX0, (ScreenY0 + yStep * 14) - 1, GUI_MAGENTA, ColorStartWhite);
#endif
    }
    g_u8GUIItem = 0;
}

#else

void GUIDEMO_ColorBar_C(void);
void GUIDEMO_ColorBar_C(void) {}

#endif  // SHOW_GUIDEMO_COLORBAR

/*************************** End of file ****************************/

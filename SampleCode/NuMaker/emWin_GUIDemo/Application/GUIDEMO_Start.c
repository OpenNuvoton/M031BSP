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
File        : GUIDEMO_Start.c
Purpose     : GUIDEMO initialization
----------------------------------------------------------------------
*/

#include "GUIDEMO.h"

/*********************************************************************
*
*       MainTask
*/
void MainTask(void)
{
    extern GUI_CONST_STORAGE GUI_BITMAP bmnuvoton_logo;    
    
#if GUI_WINSUPPORT
    WM_SetCreateFlags(WM_CF_MEMDEV);
#endif
    GUI_Init();
#if GUI_WINSUPPORT
    WM_MULTIBUF_Enable(1);
#endif
    GUI_SetBkColor(GUI_WHITE);
    GUI_Clear();
#ifdef __DEMO_160x128__
    GUI_DrawBitmap(&bmnuvoton_logo, 1, 55);
#else
    GUI_DrawBitmap(&bmnuvoton_logo, (320 - bmnuvoton_logo.XSize) >> 1, (240 - bmnuvoton_logo.YSize) >> 1);
#endif
    GUI_Delay(3000);
    
    GUIDEMO_Main();
}

/*************************** End of file ****************************/


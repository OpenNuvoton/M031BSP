/****************************************************************
 *                                                             *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved. *
 *                                                              *
 ****************************************************************/

#ifndef __LCM_H__
#define __LCM_H__

#ifdef  __cplusplus
extern "C"
{
#endif

U8 _Read1(void);
void _ReadM1(U8 * pData, int NumItems);
void _Write0(U8 Cmd);
void _Write1(U8 Data);
void _WriteM1(U8 * pData, int NumItems);
void _InitController(void);

#ifdef  __cplusplus
}
#endif

#endif  // __LCM_H__

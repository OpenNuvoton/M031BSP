;/*---------------------------------------------------------------------------------------------------------*/
;/*                                                                                                         */
;/* SPDX-License-Identifier: Apache-2.0                                                                     */
;/* Copyright(c) 2017 Nuvoton Technology Corp. All rights reserved.                                         */
;/*                                                                                                         */
;/*---------------------------------------------------------------------------------------------------------*/

	.syntax	unified
	.arch	armv8-m.base

	.globl	loaderImage1Base
	.globl	loaderImage1Limit

	.align	4
 
	.text
        
loaderImage1Base:
#ifndef _ARM_VSCODE_
    .incbin  "../../LDROM_iap/LDROM_iap.bin"
#else
    .incbin  "./LDROM_iap.bin"
#endif    
loaderImage1Limit:
    .space   4
    
    .end
   

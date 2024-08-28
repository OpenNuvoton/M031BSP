;/*---------------------------------------------------------------------------------------------------------*/
;/*                                                                                                         */
;/* SPDX-License-Identifier: Apache-2.0                                                                     */
;/* Copyright(c) 2017 Nuvoton Technology Corp. All rights reserved.                                         */
;/*                                                                                                         */
;/*---------------------------------------------------------------------------------------------------------*/

	.syntax	unified
	.arch	armv8-m.base

	.globl	loaderImageBase
	.globl	loaderImageLimit

	.globl	appImageBase
	.globl	appImageLimit

	.align	4
 
	.text
        
loaderImageBase:
    .incbin  "./Loader1.bin"
loaderImageLimit:
    .space   4
  
appImageBase:
    .incbin  "./App1.bin"
appImageLimit:
    .space   4
  
    .end
   

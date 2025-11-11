;/*---------------------------------------------------------------------------------------------------------*/
;/*                                                                                                         */
;/* SPDX-License-Identifier: Apache-2.0                                                                     */
;/* Copyright(c) 2020 Nuvoton Technology Corp. All rights reserved.                                         */
;/*                                                                                                         */
;/*---------------------------------------------------------------------------------------------------------*/

	.syntax	unified
	.arch	armv8-m.base

	.globl	loaderImage1Base
	.globl	loaderImage1Limit

	.align	4
 
	.text
        
loaderImage1Base:
    .incbin  "../../bin/fmc_ld_iap.bin"
loaderImage1Limit:
    .space   4
    
    .end
   

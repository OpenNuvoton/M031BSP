;/*---------------------------------------------------------------------------------------------------------*/
;/*                                                                                                         */
;/* Copyright(c) 2024 Nuvoton Technology Corp. All rights reserved.                                         */
;/*                                                                                                         */
;/*---------------------------------------------------------------------------------------------------------*/

	.section	.rodata
	.global		loaderImage1Base, loaderImage1Limit, loaderImage1Size
	.align		4

loaderImage1Base:
	.incbin	"../../bin/fmc_ld_iap.bin"
loaderImage1Limit:
loaderImage1Size = loaderImage1Limit - loaderImage1Base
	
	.end

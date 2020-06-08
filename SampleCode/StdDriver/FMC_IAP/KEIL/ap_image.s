;/*---------------------------------------------------------------------------------------------------------*/
;/*                                                                                                         */
;/* SPDX-License-Identifier: Apache-2.0                                                                     */
;/* Copyright(c) 2018 Nuvoton Technology Corp. All rights reserved.                                         */
;/*                                                                                                         */
;/*---------------------------------------------------------------------------------------------------------*/


    AREA _image, DATA, READONLY

    EXPORT  loaderImage1Base
    EXPORT  loaderImage1Limit
    
    ALIGN   4
        
loaderImage1Base
    INCBIN ./obj/fmc_ld_iap.bin
loaderImage1Limit

    END
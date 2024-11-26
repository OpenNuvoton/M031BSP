;/*---------------------------------------------------------------------------------------------------------*/
;/*                                                                                                         */
;/* Copyright(c) 2023 Nuvoton Technology Corp. All rights reserved.                                         */
;/*                                                                                                         */
;/*---------------------------------------------------------------------------------------------------------*/
.eabi_attribute Tag_ABI_align8_preserved,1
.global     SH_DoCommand
.type       SH_DoCommand, %function
.align      4

.extern SH_Return

SH_DoCommand:
    BKPT    #0xAB               /* Wait ICE or HardFault */
    LDR     r3, =SH_Return
    PUSH    {r3, lr}
    BLX     r3                  /* Call SH_Return. The return value is in R0 */
    POP     {r3, pc}            /* Return value = R0 */

.end

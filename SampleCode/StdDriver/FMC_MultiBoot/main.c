/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Implement a multi-boot system to boot from different applications in APROM.
 *           A LDROM code and 4 APROM code are implemented in this sample code.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#if !defined(__ICCARM__) && !defined(__GNUC__)
extern uint32_t Image$$RO$$Base;
#endif

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set PLL to Power-down mode and PLLSTB bit in CLK_STATUS register will be cleared by hardware.*/
    CLK_DisablePLL();

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))
                    |(SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

/**
 * @brief    Routine to get a char
 * @param    None
 * @returns  Get value from UART debug port or semihost
 * @details  Wait UART debug port or semihost to input a char.
 */
static char GetChar(void)
{
    while(1)
    {
        if ((UART0->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0)
        {
            return (UART0->DAT);
        }
    }
}

/*
 * @returns     Send value from UART debug port
 * @details     Send a target char to UART debug port .
 */
static void SendChar_ToUART(int ch)
{
    while (UART0->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);

    UART0->DAT = ch;
    if(ch == '\n')
    {
        while (UART0->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);
        UART0->DAT = '\r';
    }
}

static void PutString(char *str)
{
    while (*str != '\0')
    {
        SendChar_ToUART(*str++);
    }
}

static void PutHexNumber(uint32_t u32Num)
{
    int32_t i = 28;
    uint32_t Digit;
    PutString("0x");
    do
    {
        Digit = (u32Num >> i) & 0x0F;
        if(Digit != 0)
            break;
        i = i - 4;
    }
    while(i!=0);

    while (i >= 0)
    {
        Digit =  (u32Num >> i) & 0x0F;
        if(Digit < 10)
            SendChar_ToUART('0'+Digit);
        else
            SendChar_ToUART('A'+Digit-10);
        i = i - 4;
    }
}
#ifdef __GNUC__                        /* for GNU C compiler */
/**
 * @brief       Hard fault handler
 * @param[in]   stack pointer points to the dumped registers in SRAM
 * @return      None
 * @details     Replace while(1) at the end of this function with chip reset if WDT is not enabled for end product
 */
void Hard_Fault_Handler(uint32_t stack[])
{
    PutString("In Hard Fault Handler\n");
    while(1);
}
#endif
int32_t main(void)
{
    uint8_t u8Ch;
    uint32_t u32Data;


    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    
    /* Unlock protected registers to operate FMC ISP function */
    SYS_UnlockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Checking if target device supports the feature */
    if( (GET_CHIP_SERIES_NUM == CHIP_SERIES_NUM_I) || (GET_CHIP_SERIES_NUM == CHIP_SERIES_NUM_G) )
    {
        /* Checking if flash size matches with target device */
        if(FMC_FLASH_PAGE_SIZE != 2048)
        {
            /* FMC_FLASH_PAGE_SIZE is different from target device's */
            printf("Please enable the compiler option - PAGE_SIZE_2048 in fmc.h\n");
            while(SYS->PDID);
        }
    }
    else
    {
        if(FMC_FLASH_PAGE_SIZE != 512)
        {
            /* FMC_FLASH_PAGE_SIZE is different from target device's */
            printf("Please disable the compiler option - PAGE_SIZE_2048 in fmc.h\n");
            while(SYS->PDID);
        }
    }

    /* Enable FMC ISP function */
    FMC_Open();

    /*
        This sample code shows how to boot with different firmware images in APROM.
        In the code, VECMAP is used to implement multi-boot function. Software set VECMAP
        to remap page of VECMAP to 0x0~0x1ff.
        NOTE: VECMAP only valid when CBS = 00'b or 10'b.

        The sample code didn't use stanard C library for UART message due to the size of LDROM.

        To use this sample code, please:
        1. Build all targets and download to device individually. The targets are:
            FMC_MultiBoot, RO=0x0, ICE download algorithm to APROM
            FMC_Boot0, RO=0x4000, ICE download algorithm to APROM
            FMC_Boot1, RO=0x8000, ICE download algorithm to APROM
            FMC_Boot2, RO=0xC000, ICE download algorithm to APROM
            FMC_Boot3, RO=0x10000, ICE download algorithm to APROM
            FMC_BootLD, RO=0x100000. ICE download algorithm to LDROM
        2. Reset MCU to execute FMC_MultiBoot.
    */

    PutString("\n\n");
    PutString("+---------------------------+\n");
    PutString("|   Multi-Boot Sample Code  |\n");
    PutString("+---------------------------+\n");
#if defined(__BASE__)
    PutString("Boot from 0\n");
#endif
#if defined(__BOOT0__)
    PutString("Boot from 0x4000\n");
#endif
#if defined(__BOOT1__)
    PutString("Boot from 0x8000\n");
#endif
#if defined(__BOOT2__)
    PutString("Boot from 0xC000\n");
#endif
#if defined(__BOOT3__)
    PutString("Boot from 0x10000\n");
#endif
#if defined(__LDROM__)
    PutString("Boot from 0x100000\n");
#endif

    u32Data = FMC_GetVECMAP();
    PutString("VECMAP = ");
    PutHexNumber(u32Data);
    PutString("\n");

    PutString("Select one boot image: \n");
#if !defined(__GNUC__)
    PutString("[0] Boot 0, base = 0x4000\n");
#endif
    PutString("[1] Boot 1, base = 0x8000\n");
#if !defined(__GNUC__)
    PutString("[2] Boot 2, base = 0xC000\n");
#endif
    PutString("[3] Boot 3, base = 0x10000\n");
#if !defined(__GNUC__)
    PutString("[4] Boot 4, base = 0x100000\n");
#endif
    PutString("[Others] Boot, base = 0x0\n");

    u8Ch = GetChar();//getchar();

    switch (u8Ch)
    {
#if !defined(__GNUC__)
    case '0':
        FMC_SetVectorPageAddr(0x4000);
        break;
#endif

    case '1':
        FMC_SetVectorPageAddr(0x8000);
        break;

#if !defined(__GNUC__)
    case '2':
        FMC_SetVectorPageAddr(0xC000);
        break;
#endif

    case '3':
        FMC_SetVectorPageAddr(0x10000);
        break;
#if !defined(__GNUC__)

    case '4':
        FMC_SetVectorPageAddr(0x100000);
        break;
#endif

    default:
        FMC_SetVectorPageAddr(0x0);
        break;
    }
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_SetVectorPageAddr failed!\n");
        while (1);
    }

    /* Reset CPU only to reset to new vector page */
    SYS_ResetCPU();

    /* Reset System to reset to new vector page. */
    //NVIC_SystemReset();

    /* Disable ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    PutString("\nDone\n");

    while (SYS->PDID) __WFI();

}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    WDT function for level 1 training course
 *           MCU will be woken up by WDT 10 times and then reset.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

#define LED_R   PC4
#define LED_B   PC3
#define LED_G   PC5

#define LED_ON      0
#define LED_OFF     1

volatile uint32_t g_u32WDTINTCounts;
volatile uint8_t g_u8IsWDTWakeupINT;

extern int IsDebugFifoEmpty(void);

void WDT_IRQHandler(void)
{
    if(g_u32WDTINTCounts < 10)
        WDT_RESET_COUNTER();

    if(WDT_GET_TIMEOUT_INT_FLAG() == 1)
    {
        /* Clear WDT time-out interrupt flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG();
        g_u32WDTINTCounts++;
    }

    if(WDT_GET_TIMEOUT_WAKEUP_FLAG() == 1)
    {
        /* Clear WDT time-out wake-up flag */
        WDT_CLEAR_TIMEOUT_WAKEUP_FLAG();

        g_u8IsWDTWakeupINT = 1;
    }
}

void SYS_Init(void)
{
    /* Enable LIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

    /* Wait for LIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(WDT_MODULE);

    /* Set module clock */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDTSEL_LIRC, NULL);
}

void UART0_Init()
{
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

void LED_Init(void)
{
    /* Set PC.3 ~ PC.5 to GPIO */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC3MFP_Msk | SYS_GPC_MFPL_PC4MFP_Msk | SYS_GPC_MFPL_PC5MFP_Msk)) |
                    (SYS_GPC_MFPL_PC3MFP_GPIO | SYS_GPC_MFPL_PC4MFP_GPIO | SYS_GPC_MFPL_PC5MFP_GPIO);
    /* Set PC.3 ~ PC.5 to GPIO output */
    GPIO_SetMode(PC, (BIT3 | BIT4 | BIT5), GPIO_MODE_OUTPUT);

    /* Let LED off after initialize */
    LED_R = LED_OFF;
    LED_G = LED_OFF;
    LED_B = LED_OFF;
}

void WDT_Init(void)
{
    /* Because of all bits can be written in WDT Control Register are write-protected;
       To program it needs to disable register protection first. */
    SYS_UnlockReg();

    /* Set WDT :
        - clock source: LIRC
        - Time-out interval: 2^14 * WDT clock (around 426.745 ms ~ 453.385 ms, if LIRC is 38.4kHz)
        - Reset delay: 18 WDT clock
        - Enable Reset
        - Enable Wake-up
    */

    WDT_Open(WDT_TIMEOUT_2POW14, WDT_RESET_DELAY_18CLK, TRUE, TRUE);

    /* Enable WDT interrupt */
    WDT_EnableInt();
    NVIC_EnableIRQ(WDT_IRQn);
}

int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init LED */
    LED_Init();

    /* To check if system has been reset by WDT time-out reset or not */
    if(WDT_GET_RESET_FLAG() == 1)
    {
        LED_G = LED_ON;
        WDT_CLEAR_RESET_FLAG();
        printf("*** System has been reset by WDT time-out event ***\n\n");

        while(1);
    }

    printf("+------------------------------------+\n");
    printf("|    Level1 Watch Dog Sample Code    |\n");
    printf("+------------------------------------+\n\n");

    printf("# WDT Settings:\n");
    printf("    - Clock source is LIRC                  \n");
    printf("    - Time-out interval is 2^14 * WDT clock \n");
    printf("      around 426.745 ms ~ 453.385 ms(LIRC is 38.4kHz)\n");
    printf("    - Interrupt enable                      \n");
    printf("    - Wake-up function enable               \n");
    printf("    - Reset function enable                 \n");
    printf("    - Reset delay period is 18 * WDT clock  \n");
    printf("# System will generate a WDT time-out interrupt event after 426.745 ms ~ 453.385 ms, \n");
    printf("    and will be wake up from power-down mode also.\n");
    printf("    (Use LED_R(PC.4) high/low period time to check WDT time-out interval)\n");
    printf("# When WDT interrupt counts large than 10, \n");
    printf("    we will not reset WDT counter value and system will be reset immediately by WDT time-out reset signal.\n");

    /* Init watchdog */
    WDT_Init();

    while(1)
    {
        /* System enter to Power-down */
        /* To program PWRCTL register, it needs to disable register protection first. */
        SYS_UnlockReg();
        printf("\nSystem enter to power-down mode ...\n");
        /* To check if all the debug messages are finished */
        while(IsDebugFifoEmpty() == 0);

        CLK_PowerDown();

        /* Check if WDT time-out interrupt and wake-up occurred or not */
        while(g_u8IsWDTWakeupINT == 0);

        g_u8IsWDTWakeupINT = 0;
        LED_R ^= 1;

        printf("System has been waken up done. WDT interrupt counts: %d.\n\n", g_u32WDTINTCounts);
    }
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/

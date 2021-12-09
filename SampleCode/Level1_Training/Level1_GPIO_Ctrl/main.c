/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    GPIO function for level1 training course
 *
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

#define LED_R   PC4
#define LED_G   PC5
#define LED_B   PC3

#define LED_ON      0
#define LED_OFF     1

volatile uint32_t sw1_int_cnt = 0;
volatile uint32_t sw2_int_cnt = 0;

void SYS_Init(void)
{
    /* Enable HXT clock (external XTAL 12MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable LIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

    /* Wait for LIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_48MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Set module clock */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));
}

void UART0_Init()
{
    /* Set GPB multi-function pins to UART0 RXD and TXD */
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

void BTN_Init(void)
{

    /**************  SW1 ***************/
    /* Set PB.4 to GPIO */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB4MFP_Msk)) | (SYS_GPB_MFPL_PB4MFP_GPIO);
    /* Set PB.4 to GPIO intput */
    GPIO_SetMode(PB, BIT4, GPIO_MODE_INPUT);
    GPIO_EnableInt(PB, 4, GPIO_INT_FALLING);
    NVIC_EnableIRQ(GPIO_PAPB_IRQn);

    /**************  SW2 ***************/
    /* Set PB.0 to GPIO */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB0MFP_Msk)) | (SYS_GPB_MFPL_PB0MFP_GPIO);
    /* Set PB.0 to GPIO intput */
    GPIO_SetMode(PB, BIT0, GPIO_MODE_INPUT);
    GPIO_EnableInt(PB, 0, GPIO_INT_FALLING);

    /* Set de-bounce function */
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_512);
    GPIO_ENABLE_DEBOUNCE(PB, BIT4);
}

int main(void)
{
    uint32_t sw1_cnt = 0, sw2_cnt = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("+---------------------------------------+\n");
    printf("|    Level1 GPIO control Sample Code    |\n");
    printf("+---------------------------------------+\n\n");

    /* Init LED */
    LED_Init();

    /* Init BTN */
    BTN_Init();

    while(1)
    {
        /* Check if the SW1 is pressed */
        if (sw1_int_cnt != sw1_cnt)
        {
            sw1_cnt = sw1_int_cnt;
            printf("SW1 interrupt count: %d\n", sw1_cnt);
        }
        /* Check if the SW2 is pressed */
        if (sw2_int_cnt != sw2_cnt)
        {
            sw2_cnt = sw2_int_cnt;
            printf("SW2 interrupt count: %d\n", sw2_cnt);
        }
    }
}

void GPABGH_IRQHandler(void)
{
    /* Check if PB.4 the interrupt occurred */
    if(GPIO_GET_INT_FLAG(PB, BIT4))
    {
        LED_R ^= 1;
        sw1_int_cnt++;
        /* Clear PB.4 interrupt flag */
        GPIO_CLR_INT_FLAG(PB, BIT4);
        /* Check if PB.0 the interrupt occurred */
    }
    else if(GPIO_GET_INT_FLAG(PB, BIT0))
    {
        LED_G ^= 1;
        sw2_int_cnt++;
        /* Clear PB.0 interrupt flag */
        GPIO_CLR_INT_FLAG(PB, BIT0);
    }
    else
    {
        /* Un-expected interrupt. Just clear all PB interrupts */
        PB->INTSRC = PB->INTSRC;
        printf("Un-expected interrupts.\n");
    }
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/

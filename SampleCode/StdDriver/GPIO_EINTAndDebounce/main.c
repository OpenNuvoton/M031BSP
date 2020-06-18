/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show the usage of GPIO external interrupt function and de-bounce function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/**
 * @brief       External INT024 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The External INT024 default IRQ, declared in startup_M031Series.s.
 */
void EINT024_IRQHandler(void)
{
    /* To check if PA.6 external interrupt occurred */
    if(GPIO_GET_INT_FLAG(PA, BIT6))
    {
        GPIO_CLR_INT_FLAG(PA, BIT6);
        printf("PA.6 EINT0 occurred.\n");
    }

    /* To check if PB.5 external interrupt occurred */
    if(GPIO_GET_INT_FLAG(PB, BIT5))
    {
        GPIO_CLR_INT_FLAG(PB, BIT5);
        printf("PB.5 EINT0 occurred.\n");
    }
}

/**
 * @brief       External INT135 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The External INT135 default IRQ, declared in startup_M031Series.s.
 */
void EINT135_IRQHandler(void)
{
    /* To check if PA.7 external interrupt occurred */
    if(GPIO_GET_INT_FLAG(PA, BIT7))
    {
        GPIO_CLR_INT_FLAG(PA, BIT7);
        printf("PA.7 EINT1 occurred.\n");
    }

    /* To check if PB.4 external interrupt occurred */
    if(GPIO_GET_INT_FLAG(PB, BIT4))
    {
        GPIO_CLR_INT_FLAG(PB, BIT4);
        printf("PB.4 EINT1 occurred.\n");
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set PA multi-function pin for EINT0(PA.6) and EINT1(PA.7) */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA6MFP_Msk | SYS_GPA_MFPL_PA7MFP_Msk)) |
                    (SYS_GPA_MFPL_PA6MFP_INT0 | SYS_GPA_MFPL_PA7MFP_INT1);

    /* Set PB multi-function pin for EINT0(PB.5) and EINT1(PB.4) */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB5MFP_Msk | SYS_GPB_MFPL_PB4MFP_Msk)) |
                    (SYS_GPB_MFPL_PB5MFP_INT0 | SYS_GPB_MFPL_PB4MFP_INT1);

    /* Lock protected registers */
    SYS_LockReg();
}

/*----------------------------------------------------------------------*/
/* Init UART0                                                           */
/*----------------------------------------------------------------------*/
void UART0_Init(void)
{
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

int main(void)
{
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------------------------+\n");
    printf("|    GPIO EINT0/EINT1 Interrupt and De-bounce Sample Code    |\n");
    printf("+------------------------------------------------------------+\n\n");

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO External Interrupt Function Test                                                               */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("EINT0(PA.6 and PB.5) and EINT1(PA.7 and PB.4) are used to test interrupt\n");
    printf("    PA.6 is falling edge trigger.\n");
    printf("    PB.5 is rising edge trigger.\n");
    printf("    PA.7 and PB.4 are both falling edge and rising edge trigger.\n");

    /* Configure PA.6 as EINT0 pin and enable interrupt by falling edge trigger */
    GPIO_SetMode(PA, BIT6, GPIO_MODE_INPUT);
    GPIO_EnableInt(PA, 6, GPIO_INT_FALLING);

    /* Configure PB.5 as EINT0 pin and enable interrupt by rising edge trigger */
    GPIO_SetMode(PB, BIT5, GPIO_MODE_INPUT);
    GPIO_EnableInt(PB, 5, GPIO_INT_RISING);
    NVIC_EnableIRQ(EINT024_IRQn);

    /* Configure PA.7 as EINT1 pin and enable interrupt by falling and rising edge trigger */
    GPIO_SetMode(PA, BIT7, GPIO_MODE_INPUT);
    GPIO_EnableInt(PA, 7, GPIO_INT_BOTH_EDGE);

    /* Configure PB.4 as EINT1 pin and enable interrupt by falling and rising edge trigger */
    GPIO_SetMode(PB, BIT4, GPIO_MODE_INPUT);
    GPIO_EnableInt(PB, 4, GPIO_INT_BOTH_EDGE);

    NVIC_EnableIRQ(EINT135_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
    GPIO_ENABLE_DEBOUNCE(PA, BIT6 | BIT7);
    GPIO_ENABLE_DEBOUNCE(PB, BIT4 | BIT5);

    /* Waiting for interrupts */
    while(1);
}

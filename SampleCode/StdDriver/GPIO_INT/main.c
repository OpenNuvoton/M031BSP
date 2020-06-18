/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show the usage of GPIO interrupt function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/**
 * @brief       GPIO PA/PB/PG/PH IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PA/PB/PG/PH default IRQ, declared in startup_M031Series.s.
 */
void GPABGH_IRQHandler(void)
{
    volatile uint32_t temp;

    /* To check if PB.2 interrupt occurred */
    if(GPIO_GET_INT_FLAG(PB, BIT2))
    {
        GPIO_CLR_INT_FLAG(PB, BIT2);
        printf("PB.2 INT occurred.\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PB interrupts */
        temp = PB->INTSRC;
        PB->INTSRC = temp;
        printf("Un-expected interrupts.\n");
    }
}

/**
 * @brief       GPIO PC/PD/PE/PF IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PC/PD/PE/PF default IRQ, declared in startup_M031Series.s.
 */
void GPCDEF_IRQHandler(void)
{
    volatile uint32_t temp;

    /* To check if PC.5 interrupt occurred */
    if(GPIO_GET_INT_FLAG(PC, BIT1))
    {
        GPIO_CLR_INT_FLAG(PC, BIT1);
        printf("PC.1 INT occurred.\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PC interrupts */
        temp = PC->INTSRC;
        PC->INTSRC = temp;
        printf("Un-expected interrupts.\n");
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
    printf("+------------------------------------------------+\n");
    printf("|    GPIO PB.2 and PC.1 Interrupt Sample Code    |\n");
    printf("+------------------------------------------------+\n\n");

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Interrupt Function Test                                                                        */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("PB.2 and PC.1 are used to test interrupt ......\n");
    printf("    PB.2 is rising edge trigger.\n");
    printf("    PC.1 is falling edge trigger.\n");

    /* Configure PB.2 as Input mode and enable interrupt by rising edge trigger */
    GPIO_SetMode(PB, BIT2, GPIO_MODE_INPUT);
    GPIO_EnableInt(PB, 2, GPIO_INT_RISING);
    NVIC_EnableIRQ(GPIO_PAPBPGPH_IRQn);

    /* Configure PC.1 as Quasi-bidirection mode and enable interrupt by falling edge trigger */
    GPIO_SetMode(PC, BIT1, GPIO_MODE_QUASI);
    GPIO_EnableInt(PC, 1, GPIO_INT_FALLING);
    NVIC_EnableIRQ(GPIO_PCPDPEPF_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
    GPIO_ENABLE_DEBOUNCE(PB, BIT2);
    GPIO_ENABLE_DEBOUNCE(PC, BIT1);

    /* Waiting for interrupts */
    while(1);
}

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
    if(PA->INTSRC & BIT6)
    {
        PA->INTSRC = BIT6;
        printf("PA.6 EINT0 occurred.\n");
    }

    /* To check if PB.5 external interrupt occurred */
    if(PB->INTSRC & BIT5)
    {
        PB->INTSRC = BIT5;
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
    if(PA->INTSRC & BIT7)
    {
        PA->INTSRC = BIT7;
        printf("PA.7 EINT1 occurred.\n");
    }

    /* To check if PB.4 external interrupt occurred */
    if(PB->INTSRC & BIT4)
    {
        PB->INTSRC = BIT4;
        printf("PB.4 EINT1 occurred.\n");
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) != CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(1);

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Switch UART0 clock source to HIRC */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART0SEL_Msk) | CLK_CLKSEL1_UART0SEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_UART0DIV_Msk) | CLK_CLKDIV0_UART0(1);

    /* Enable UART0 peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

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
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
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
    /* Configure PA.7 as EINT1 pin and enable interrupt by falling and rising edge trigger */
    PA->MODE = (PA->MODE & ~(GPIO_MODE_MODE6_Msk | GPIO_MODE_MODE7_Msk)) |
               ((GPIO_MODE_INPUT << GPIO_MODE_MODE6_Pos) | (GPIO_MODE_INPUT << GPIO_MODE_MODE7_Pos));

    /* Configure interrupt mode of specified pin */
    PA->INTTYPE = (PA->INTTYPE & ~(BIT6 | BIT7));       /* set PA6 and PA7 to edge trigger interrupt */
    /* Enable interrupt function of specified pin */
    PA->INTEN = (PA->INTEN & ~(BIT6<<GPIO_INTEN_RHIEN0_Pos)) |  /* PA.6 disable rising  edge trigger interrupt */
                (BIT6 | (BIT7<<GPIO_INTEN_RHIEN0_Pos) | BIT7);  /* PA.6 enable  falling edge trigger interrupt */
    /* PA.7 enable  rising  edge trigger interrupt */
    /* PA.7 enable  falling edge trigger interrupt */

    /* Configure PB.5 as EINT0 pin and enable interrupt by rising edge trigger */
    /* Configure PB.4 as EINT0 pin and enable interrupt by rising edge trigger */
    PB->MODE = (PB->MODE & ~(GPIO_MODE_MODE5_Msk | GPIO_MODE_MODE4_Msk)) |
               ((GPIO_MODE_INPUT << GPIO_MODE_MODE5_Pos) | (GPIO_MODE_INPUT << GPIO_MODE_MODE4_Pos));

    /* Configure interrupt mode of specified pin */
    PB->INTTYPE &= ~BIT5;   /* set PB5 to edge trigger interrupt */
    /* Enable interrupt function of specified pin */
    PB->INTEN = (PB->INTEN & ~(BIT5)) |                         /* PB.5 disable falling edge trigger interrupt */
                ((BIT5<<GPIO_INTEN_RHIEN0_Pos) | BIT4 | (BIT4<<GPIO_INTEN_RHIEN0_Pos));
    /* PB.5 enable  rising  edge trigger interrupt */
    /* PB.4 enable  falling edge trigger interrupt */
    /* PB.4 enable  rising  edge trigger interrupt */

    NVIC_EnableIRQ(EINT024_IRQn);
    NVIC_EnableIRQ(EINT135_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO->DBCTL = GPIO_DBCTL_ICLKON_Msk | GPIO_DBCTL_DBCLKSRC_LIRC | GPIO_DBCTL_DBCLKSEL_1024;
    PA->DBEN |= (BIT6 | BIT7);
    PB->DBEN |= (BIT5 | BIT4);

    /* Waiting for interrupts */
    while(1);
}

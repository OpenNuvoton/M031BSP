/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to set GPIO pin mode and use pin data input/output control.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "NuMicro.h"


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

int32_t main(void)
{
    int32_t i32Err, i32TimeOutCnt;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+-------------------------------------------------+\n");
    printf("|    PB.3(Output) and PC.1(Input) Sample Code     |\n");
    printf("+-------------------------------------------------+\n\n");

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Basic Mode Test --- Use Pin Data Input/Output to control GPIO pin                              */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("  >> Please connect PB.3 and PC.1 first << \n");
    printf("     Press any key to start test by using [Pin Data Input/Output Control] \n\n");
    getchar();

    /* Configure PB.3 as Output mode and PC.7 as Input mode then close it */
    PB->MODE = (PB->MODE & ~(GPIO_MODE_MODE3_Msk)) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE3_Pos);
    PC->MODE = (PC->MODE & ~(GPIO_MODE_MODE1_Msk)) | (GPIO_MODE_INPUT << GPIO_MODE_MODE1_Pos);

    i32Err = 0;
    printf("GPIO PB.3(output mode) connect to PC.1(input mode) ......");

    /* Use Pin Data Input/Output Control to pull specified I/O or get I/O pin status */
    /* Set PB.3 output pin value is low */
    PB3 = 0;

    /* Set time out counter */
    i32TimeOutCnt = 100;

    /* Wait for PC.1 input pin status is low for a while */
    while(PC1 != 0)
    {
        if(i32TimeOutCnt > 0)
        {
            i32TimeOutCnt--;
        }
        else
        {
            i32Err = 1;
            break;
        }
    }

    /* Set PB.3 output pin value is high */
    PB3 = 1;

    /* Set time out counter */
    i32TimeOutCnt = 100;

    /* Wait for PC.1 input pin status is high for a while */
    while(PC1 != 1)
    {
        if(i32TimeOutCnt > 0)
        {
            i32TimeOutCnt--;
        }
        else
        {
            i32Err = 1;
            break;
        }
    }

    /* Print test result */
    if(i32Err)
    {
        printf("  [FAIL].\n");
    }
    else
    {
        printf("  [OK].\n");
    }

    /* Configure PB.3 and PC.1 to default Quasi-bidirectional mode */
    PB->MODE = (PB->MODE & ~(GPIO_MODE_MODE3_Msk)) | (GPIO_MODE_QUASI << GPIO_MODE_MODE3_Pos);
    PC->MODE = (PC->MODE & ~(GPIO_MODE_MODE1_Msk)) | (GPIO_MODE_QUASI << GPIO_MODE_MODE1_Pos);

    while(1);
}

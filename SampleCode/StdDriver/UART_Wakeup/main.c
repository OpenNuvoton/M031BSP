/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 6 $
 * $Date: 18/07/16 10:31a $
 * @brief    Show how to wake up system form Power-down mode by UART interrupt.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define RS485_ADDRESS 0xC0

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART_DataWakeUp(void);
void UART_CTSWakeUp(void);
void UART_RxThresholdWakeUp(void);
void UART_RS485WakeUp(void);
void UART_PowerDown_TestItem(void);
void UART_PowerDownWakeUpTest(void);


/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(UART0);

    /* Enter to Power-down mode */
    CLK_PowerDown();
}

void SYS_Init(void)
{

    /* Set X32_OUT(PF.4) and X32_IN(PF.5) to input mode */
    GPIO_SetMode(PF, (BIT4 | BIT5), GPIO_MODE_INPUT);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC and LIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

    /* Wait for HIRC and LIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set both PCLK0 and PCLK1 as HCLK */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1;

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(UART1_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))    |   \
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set PA multi-function pins for UART1 TXD, RXD, CTS and RTS */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk |  \
                                       SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk)) |   \
                    (SYS_GPA_MFPL_PA0MFP_UART1_nRTS | SYS_GPA_MFPL_PA1MFP_UART1_nCTS |  \
                     SYS_GPA_MFPL_PA2MFP_UART1_RXD | SYS_GPA_MFPL_PA3MFP_UART1_TXD);


}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

void UART1_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART1 */
    SYS_ResetModule(UART1_RST);

    /* Configure UART1 and set UART1 baud rate */
    UART_Open(UART1, 9600);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init UART1 for test */
    UART1_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    printf("\nUART Sample Program\n");

    /* UART Power-down and Wake-up sample function */
    UART_PowerDownWakeUpTest();

    printf("UART Sample Program End.\n");

    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 1 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART13_IRQHandler(void)
{
    uint32_t u32Data;

    if(UART_GET_INT_FLAG(UART1, UART_INTSTS_WKINT_Msk))     /* UART wake-up interrupt flag */
    {
        UART_ClearIntFlag(UART1, UART_INTSTS_WKINT_Msk);
        printf("UART wake-up.\n");
        UART_WAIT_TX_EMPTY(UART0);
    }
    else if(UART_GET_INT_FLAG(UART1, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART1) == 0)
        {
            u32Data = UART_READ(UART1);
            if(u32Data & UART_DAT_PARITY_Msk)
                printf("Address: 0x%X\n", (u32Data & 0xFF));
            else
                printf("Data: 0x%X\n", u32Data);
        }
    }

}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART nCTS Wake-up Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void UART_CTSWakeUp(void)
{
    /* Enable UART nCTS wake-up frunction */
    UART1->WKCTL |= UART_WKCTL_WKCTSEN_Msk;

    printf("System enter to Power-down mode.\n");
    printf("Toggle UART1 nCTS to wake-up system.\n\n");

}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Data Wake-up Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void UART_DataWakeUp(void)
{
    /* Enable UART data wake-up frunction */
    UART1->WKCTL |= UART_WKCTL_WKDATEN_Msk;

    /* Set UART data wake-up start bit compensation value.
       It indicates how many clock cycle selected by UART_CLK does the UART controller can get the first bit (start bit)
       when the device is wake-up from power-down mode.
       If UART_CLK is selected as HIRC(48MHz) and the HIRC stable time is about 52.03us,
       the data wake-up start bit compensation value can be set as 0x9C2. */
    UART1->DWKCOMP = 0x9C2;

    printf("System enter to Power-down mode.\n");
    printf("Send data with baud rate 9600bps to UART1 to wake-up system.\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Rx threshold and time-out Function                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void UART_RxThresholdWakeUp(void)
{
    /* Wait data transmission is finished and select UART clock source as LIRC */
    while((UART0->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) == 0);
    while((UART0->FIFOSTS & UART_FIFOSTS_RXIDLE_Msk) == 0);
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_LIRC, CLK_CLKDIV0_UART1(1));

    /* Set UART baud rate and baud rate compensation */
    UART_Open(UART1, 9600);
    UART1->BRCOMP = 0x80000001;

    /* Enable UART Rx Threshold and Rx time-out wake-up frunction */
    UART1->WKCTL |= UART_WKCTL_WKRFRTEN_Msk | UART_WKCTL_WKTOUTEN_Msk;

    /* Set Rx FIFO interrupt trigger level */
    UART1->FIFO = (UART1->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_4BYTES;

    /* Enable UART Rx time-out function */
    UART_SetTimeoutCnt(UART1, 40);

    printf("System enter to Power-down mode.\n");
    printf("Send data with baud rate 9600bps to UART1 to wake-up system.\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART RS485 address match (AAD mode) function                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void UART_RS485WakeUp(void)
{
    /* Wait data transmission is finished and select UART clock source as LIRC */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_LIRC, CLK_CLKDIV0_UART1(1));

    /* Set UART baud rate and baud rate compensation */
    UART_Open(UART1, 9600);
    UART1->BRCOMP = 0x80000001;

    /* RS485 address match (AAD mode) setting */
    UART_SelectRS485Mode(UART1, UART_ALTCTL_RS485AAD_Msk, RS485_ADDRESS);

    /* Enable parity source selection function */
    UART1->LINE |= (UART_LINE_PSS_Msk | UART_LINE_PBE_Msk);

    /* Enable UART RS485 address match, Rx Threshold and Rx time-out wake-up frunction */
    UART1->WKCTL |= UART_WKCTL_WKRFRTEN_Msk | UART_WKCTL_WKRS485EN_Msk | UART_WKCTL_WKTOUTEN_Msk;

    /* Enable UART Rx time-out function */
    UART_SetTimeoutCnt(UART1, 40);

    printf("System enter to Power-down mode.\n");
    printf("Send RS485 address byte 0x%X with baud rate 9600bps to UART1 to wake-up system.\n\n", RS485_ADDRESS);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Power-down and Wake-up Menu                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void UART_PowerDown_TestItem(void)
{
    printf("+-----------------------------------------------------------+\n");
    printf("|  UART Power-down and wake-up test                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| [1] nCTS wake-up test                                     |\n");
    printf("| [2] Data wake-up test                                     |\n");
    printf("| [3] Rx threshold and time-out wake-up test                |\n");
    printf("| [4] RS485 wake-up test                                    |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Quit                                           - [Others] |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("Please Select key (1~4): ");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Power-down and Wake-up Test Function                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART_PowerDownWakeUpTest(void)
{
    uint32_t u32Item;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Lock protected registers */
    SYS_LockReg();

    /* Enable UART wake-up and receive data available interrupt */
    NVIC_EnableIRQ(UART13_IRQn);
    UART_EnableInt(UART1, UART_INTEN_WKIEN_Msk | UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);

    UART_PowerDown_TestItem();
    u32Item = getchar();
    printf("%c\n\n", u32Item);
    switch(u32Item)
    {
    case '1':
        UART_CTSWakeUp();
        break;
    case '2':
        UART_DataWakeUp();
        break;
    case '3':
        UART_RxThresholdWakeUp();
        break;
    case '4':
        UART_RS485WakeUp();
        break;
    default:
        return;
    }

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    /* Enter to Power-down mode */
    PowerDownFunction();

    /* Lock protected registers after entering Power-down mode */
    SYS_LockReg();

    printf("Enter any key to end test.\n\n");
    getchar();

    /* Disable UART wake-up function */
    UART1->WKCTL = 0;

    /* Disable UART Interrupt */
    UART_DisableInt(UART1, UART_INTEN_WKIEN_Msk | UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);

}

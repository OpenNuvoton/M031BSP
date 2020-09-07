/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 4 $
 * $Date: 18/07/13 4:48p $
 * @brief    Show how to wake up system from Power-down mode by UART interrupt.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define RS485_ADDRESS 0xC0U

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
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

    /* Set the processor is deep sleep as its low power mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Set system Power-down enabled*/
    CLK->PWRCTL |= CLK_PWRCTL_PDEN_Msk;

    /* Chip enter Power-down mode after CPU run WFI instruction */
    __WFI();

}

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set X32_OUT(PF.4) and X32_IN(PF.5) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE4_Msk | GPIO_MODE_MODE5_Msk);

    /* Enable HIRC and LIRC clock */
    CLK->PWRCTL |= (CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_LIRCEN_Msk );

    /* Wait for HIRC and LIRC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));
    while(!(CLK->STATUS & CLK_STATUS_LIRCSTB_Msk));

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    CLK->APBCLK0 |= (CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_UART1CKEN_Msk);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART0SEL_Msk)) | CLK_CLKSEL1_UART0SEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UART0DIV_Msk)) | CLK_CLKDIV0_UART0(1);

    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART1SEL_Msk)) | CLK_CLKSEL1_UART1SEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UART1DIV_Msk)) | CLK_CLKDIV0_UART1(1);
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))    |       \
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set PA multi-function pins for UART1 TXD, RXD, CTS and RTS */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk |  \
                                       SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk)) |   \
                    (SYS_GPA_MFPL_PA0MFP_UART1_nRTS | SYS_GPA_MFPL_PA1MFP_UART1_nCTS |  \
                     SYS_GPA_MFPL_PA2MFP_UART1_RXD | SYS_GPA_MFPL_PA3MFP_UART1_TXD);

    /* Lock protected registers */
    SYS_LockReg();

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void UART1_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART1 */
    SYS->IPRST1 |=  SYS_IPRST1_UART1RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART1RST_Msk;

    /* Configure UART1 and set UART1 baud rate */
    UART1->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 9600);
    UART1->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init UART1 for test */
    UART1_Init();

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
    uint32_t u32IntSts = UART1->INTSTS;
    uint32_t u32Data;
    uint32_t u32WKSts  = UART1->WKSTS;

    if(u32IntSts & UART_INTSTS_WKINT_Msk)               /* UART wake-up interrupt flag */
    {
        UART1->WKSTS = u32WKSts;
        printf("UART wake-up.\n");
        UART_WAIT_TX_EMPTY(UART0);
    }
    else if(u32IntSts & (UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while((UART1->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0)
        {
            u32Data = UART1->DAT;
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
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART1SEL_Msk)) | CLK_CLKSEL1_UART1SEL_LIRC;

    /* Set UART baud rate and baud rate compensation */
    UART1->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__LIRC, 9600);
    UART1->BRCOMP = 0x80000001;

    /* Enable UART Rx Threshold and Rx time-out wake-up frunction */
    UART1->WKCTL |= UART_WKCTL_WKRFRTEN_Msk | UART_WKCTL_WKTOUTEN_Msk;

    /* Set Rx FIFO interrupt trigger level */
    UART1->FIFO = (UART1->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_4BYTES;

    /* Enable UART Rx time-out function */
    UART1->INTEN |= UART_INTEN_TOCNTEN_Msk;
    UART1->TOUT = (UART1->TOUT & (~UART_TOUT_TOIC_Msk)) | (40);

    printf("System enter to Power-down mode.\n");
    printf("Send data with baud rate 9600bps to UART1 to wake-up system.\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART RS485 address match (AAD mode) function                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void UART_RS485WakeUp(void)
{
    /* Wait data transmission is finished and select UART clock source as LIRC */
    while((UART0->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) == 0);
    while((UART0->FIFOSTS & UART_FIFOSTS_RXIDLE_Msk) == 0);
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART1SEL_Msk)) | CLK_CLKSEL1_UART1SEL_LIRC;

    /* Set UART baud rate and baud rate compensation */
    UART1->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__LIRC, 9600);
    UART1->BRCOMP = 0x80000001;

    /* RS485 address match (AAD mode) setting */
    UART1->FUNCSEL = (UART1->FUNCSEL & (~UART_FUNCSEL_FUNCSEL_Msk)) | UART_FUNCSEL_RS485;
    UART1->ALTCTL |= UART_ALTCTL_RS485AAD_Msk;
    UART1->ALTCTL = (UART1->ALTCTL & (~UART_ALTCTL_ADDRMV_Msk)) | (RS485_ADDRESS << UART_ALTCTL_ADDRMV_Pos);

    /* Enable parity source selection function */
    UART1->LINE |= (UART_LINE_PSS_Msk | UART_LINE_PBE_Msk);

    /* Enable UART RS485 address match, Rx Threshold and Rx time-out wake-up frunction */
    UART1->WKCTL |= UART_WKCTL_WKRFRTEN_Msk | UART_WKCTL_WKRS485EN_Msk | UART_WKCTL_WKTOUTEN_Msk;

    /* Enable UART Rx time-out function */
    UART1->INTEN |= UART_INTEN_TOCNTEN_Msk;
    UART1->TOUT = (UART1->TOUT & (~UART_TOUT_TOIC_Msk)) | (40);

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

    /* Enable UART wake-up and receive data available interrupt */
    UART1->INTEN |= (UART_INTEN_WKIEN_Msk | UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    NVIC_EnableIRQ(UART13_IRQn);

    UART_PowerDown_TestItem();
    u32Item = getchar();
    printf("%c\n\n", u32Item);
    UART_WAIT_TX_EMPTY(UART0);

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
    UART1->INTEN = 0;
    NVIC_DisableIRQ(UART13_IRQn);

}

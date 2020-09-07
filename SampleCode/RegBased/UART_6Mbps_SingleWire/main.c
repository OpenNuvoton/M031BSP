/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to transfer 6 Mbps UART data through Single-Wire
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include "stdio.h"
#include "string.h"
#include "NuMicro.h"


#define PLL_CLOCK   FREQ_96MHZ
#define BUFSIZE   128
#define UART0_BaudRate 115200
#define UART1_BaudRate 6000000
#define UART2_BaudRate 6000000
/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8RxData[BUFSIZE] = {0};
uint8_t g_u8TxData[BUFSIZE] = {0};
volatile uint32_t g_u32RecLen  =  0;
volatile int32_t  g_i32RecOK  = FALSE;


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_TEST_HANDLE(void);
void UART02_TEST_HANDLE(void);
void UART_FunctionTest(void);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Wait for HIRC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);

    /* Enable UART0/1/2 peripheral clock */
    CLK->APBCLK0 |= (CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_UART1CKEN_Msk | CLK_APBCLK0_UART2CKEN_Msk);

    /* Select IP clock source */
    /* Select UART0 clock source is HXT */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART0SEL_Msk)) | CLK_CLKSEL1_UART0SEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UART0DIV_Msk)) | CLK_CLKDIV0_UART0(1);

    /* Select UART1 clock source is HXT */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART1SEL_Msk)) | CLK_CLKSEL1_UART1SEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UART1DIV_Msk)) | CLK_CLKDIV0_UART1(1);

    /* Select UART2 clock source is HXT */
    CLK->CLKSEL3 = (CLK->CLKSEL3 & (~CLK_CLKSEL3_UART2SEL_Msk)) | CLK_CLKSEL3_UART2SEL_HIRC;
    CLK->CLKDIV4 = (CLK->CLKDIV4 & (~CLK_CLKDIV4_UART2DIV_Msk)) | CLK_CLKDIV4_UART2(1);


    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) \
                    | SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD;

    /* Set PA multi-function pins for UART1 RXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA2MFP_Msk)) | SYS_GPA_MFPL_PA2MFP_UART1_RXD ;

    /* Set PB multi-function pins for UART2 RXD */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB0MFP_Msk)) | SYS_GPB_MFPL_PB0MFP_UART2_RXD;

    /* The RX pin needs to pull-high for single-wire */
    /* If the external circuit doesn't pull-high, set GPIO pin as Quasi-directional mode for this purpose here */
    PA->MODE &= (PA->MODE & ~GPIO_MODE_MODE2_Msk) | (GPIO_MODE_QUASI << GPIO_MODE_MODE2_Pos);

    PB->MODE = (PB->MODE & ~GPIO_MODE_MODE0_Msk) | (GPIO_MODE_QUASI << GPIO_MODE_MODE0_Pos);


    /* Lock protected registers */
    SYS_LockReg();

}
/*---------------------------------------------------------------------------------------------------------*/
/*                                     Init UART0                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init()
{
    /* Configure UART0 and set UART0 baud rate 115200 bps */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, UART0_BaudRate);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}
/*---------------------------------------------------------------------------------------------------------*/
/*                               Init Single Wire(UART1)                                                   */
/*---------------------------------------------------------------------------------------------------------*/

void UART1_Init()
{
    /* Configure Single Wire(UART1) and set Single Wire(UART1) baud rate 6M bps */
    UART1->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, UART1_BaudRate);
    UART1->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

    UART1->FUNCSEL = ((UART1->FUNCSEL & (~UART_FUNCSEL_FUNCSEL_Msk)) | UART_FUNCSEL_SINGLE_WIRE);
}


/*---------------------------------------------------------------------------------------------------------*/
/*                               Init Single Wire(UART2)                                                   */
/*---------------------------------------------------------------------------------------------------------*/

void UART2_Init()
{
    /* Configure Single Wire(UART2) and set Single Wire(UART2) baud rate 6M bps*/
    UART2->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, UART2_BaudRate);
    UART2->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

    UART2->FUNCSEL = ((UART2->FUNCSEL & (~UART_FUNCSEL_FUNCSEL_Msk)) | UART_FUNCSEL_SINGLE_WIRE);

}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Test Sample                                                                                        */
/* Test Item                                                                                               */
/* 1. The Single wire 1(UART1) sends data to Single wire 2(UART2)                                           */
/* 2. The Single wire 2(UART2) sends data to Single wire 1(UART1)                                           */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/*                                         Main Function                                                   */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for print message */
    UART0_Init();
    /* Init UART1 and UART2 for Single Wire Test*/
    UART1_Init();
    UART2_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /*                                           SAMPLE CODE                                                   */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("\nUART Sample Program\n");

    /* UART sample function */
    UART_FunctionTest();

    while (1);

}

/*---------------------------------------------------------------------------------------------------------*/
/*                       ISR to handle UART Channel 1 interrupt event                                      */
/*---------------------------------------------------------------------------------------------------------*/
void UART13_IRQHandler(void)
{
    uint32_t u32IntSts = UART1->INTSTS;

    if (u32IntSts & UART_INTSTS_RDAIF_Msk)
    {
        /* Get all the input characters */
        while (UART_IS_RX_READY(UART1))
        {
            /* Get the character from UART Buffer */
            g_u8RxData[g_u32RecLen] = UART_READ(UART1);

            if (g_u32RecLen == BUFSIZE - 1)
            {
                g_i32RecOK = TRUE;
                g_u32RecLen = 0;
            }
            else
            {
                g_u32RecLen++;
            }
        }
    }

    if (u32IntSts & UART_INTSTS_SWBEIF_Msk)
    {
        printf("Single-wire Bit Error Detection \n");
        UART1->INTSTS = UART_INTSTS_SWBEIF_Msk;
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 2 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART02_IRQHandler(void)
{
    uint32_t u32IntSts = UART2->INTSTS;

    if (u32IntSts & UART_INTSTS_RDAIF_Msk)
    {
        /* Get all the input characters */
        while (UART_IS_RX_READY(UART2))
        {
            /* Get the character from UART Buffer */
            g_u8RxData[g_u32RecLen] = UART_READ(UART2);

            if (g_u32RecLen == BUFSIZE - 1)
            {
                g_i32RecOK = TRUE;
                g_u32RecLen = 0;
            }
            else
            {
                g_u32RecLen++;
            }
        }
    }

    if (u32IntSts & UART_INTSTS_SWBEIF_Msk)
    {
        printf("Single-wire Bit Error Detection \n");
        UART2->INTSTS = UART_INTSTS_SWBEIF_Msk;

    }
}
/*---------------------------------------------------------------------------------------------------------*/
/*                              Create Source Pattern function                                              */
/*---------------------------------------------------------------------------------------------------------*/
void Create_Src_Pattern(uint32_t u32Addr, uint8_t type, uint32_t u32Length)
{
    uint32_t i = 0, pattern = 0;
    uint8_t *pAddr;
    pAddr = (uint8_t *)u32Addr;

    if (type == 0)      pattern = 0x1f;
    else if (type == 1) pattern = 0x3f;
    else if (type == 2) pattern = 0x7f;
    else if (type == 3) pattern = 0xff;
    else  pattern = 0xff;

    for (i = 0; i < u32Length ; i++)
        pAddr[i] = (i & pattern);

}

/*---------------------------------------------------------------------------------------------------------*/
/*                    Verify that the received data is correct                                             */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t Check_Pattern(uint32_t u32Addr0, uint32_t u32Addr1, uint32_t u32Length)
{
    uint32_t i = 0;
    uint8_t result = 1;
    uint8_t *pAddr0;
    uint8_t *pAddr1;
    pAddr0 = (uint8_t *)u32Addr0;
    pAddr1 = (uint8_t *)u32Addr1;

    for (i = 0; i < u32Length ; i++)
    {
        if (pAddr0[i] != pAddr1[i])
        {
            printf("Data Error Idex=%d,tx =%d,rx=%d\n", i, pAddr0[i], pAddr1[i]) ;
            result = 0;
        }
    }

    return result;
}


/*---------------------------------------------------------------------------------------------------------*/
/*  UART Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void UART_FunctionTest()
{
    uint8_t cmd ;
    uint32_t  u32i;

    printf("+-----------------------------------------------------------+\n");
    printf("|            UART Single Wire Function Test                 |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will print input char on terminal      |\n");
    printf("|    The user must connect the UART2 RX pin(PA2) to         |\n");
    printf("|    UART1 Rx Pin(PB0) 6MSPS.                               |\n");
    printf("|    Single Wire 1(PA2)sends data to Single Wire 2(PB0).    |\n");
    printf("|    Single Wire 2(PB0)sends data to Single Wire 1(PA2).    |\n");
    printf("|    Please enter any to start    (Press 'E' to exit)       |\n");
    printf("+-----------------------------------------------------------+\n");

    /*
        Using a RS232 cable to connect UART0 and PC.UART0 is set to debug port.
          UART1 and UART2 is enable RDA and RLS interrupt.
          The user can use UART0 to control the transmission or reception of UART1(Single Wire mode)
        When UART1(Single Wire 1)transfers data to UART2(Single Wire 2), if data is valid,
          it will enter the interrupt and receive the data.And then check the received data.
        When UART2(Single Wire 2)transfers data to UART1(Single Wire 1), if data is valid,
          it will enter the interrupt and receive the data.And then check the received data.
    */

    /* Enable UART1 RDA/Time-out/Single-wire Bit Error Detection interrupt */
    NVIC_EnableIRQ(UART13_IRQn);
    UART1->INTEN |= (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_SWBEIEN_Msk);
    /* Enable UART2 RDA/Time-out/Single-wire Bit Error Detection interrupt */
    NVIC_EnableIRQ(UART02_IRQn);
    UART2->INTEN |= (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_SWBEIEN_Msk);

    do
    {
        printf("+----------------------------------------------------------------+\n");
        printf("|                UART Single Wire Test Item                      |\n");
        printf("+----------------------------------------------------------------+\n");
        printf("| (1)Single Wire UART1(PA2)send data to Single Wire UART2(PB0).  |\n");
        printf("| (2)Single Wire UART2(PB0)send data to Single Wire UART1(PA2).  |\n");
        printf("| (E)Exit                                                        |\n");
        printf("+----------------------------------------------------------------+\n");

        // User input
        cmd = getchar();

        switch (cmd)
        {
        case '1':
        {
            printf("Send data from UART1 to UART2 through single wire :");

            /* Create data pattern in g_u8TxData[0:127] for test. */
            Create_Src_Pattern((uint32_t)g_u8TxData, UART_WORD_LEN_8, BUFSIZE);

            /* Wait for UART1 Rx idle*/
            while (!UART_RX_IDLE(UART1)) {};

            /* Clear software flag and then write data from g_u8TxData ram buffer to UART1 Tx FIFO */
            g_i32RecOK  = FALSE;
            for(u32i = 0; u32i < BUFSIZE; u32i++)
            {
                /* Send 1 byte data */
                UART1->DAT = g_u8TxData[u32i];

                /* Wait if Tx FIFO is full */
                while((UART1->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk)==0);
            }

            /* Wait for UART2 receive done */
            while (g_i32RecOK != TRUE) {}

            /* Verify received data */
            if(Check_Pattern((uint32_t)g_u8TxData, (uint32_t)g_u8RxData, BUFSIZE))
            {
                printf(" Pass\n");
            }
            else
            {
                printf(" Fail\n");
            }

            /* Clear UART RAM buffer */
            memset((uint8_t *)g_u8TxData, 0, BUFSIZE);
            memset((uint8_t *)g_u8RxData, 0, BUFSIZE);
        }
        break;

        case '2':
        {
            printf("Send data from UART2 to UART1 through single wire :");

            /* Create data pattern in g_u8TxData[0:127] for test. */
            Create_Src_Pattern((uint32_t)g_u8TxData, UART_WORD_LEN_8, BUFSIZE);

            /* Wait for UART2 Rx idle*/
            while (!UART_RX_IDLE(UART2)) {};

            /* Clear software flag and then write data from g_u8TxData ram buffer to UART2 Tx FIFO */
            g_i32RecOK  = FALSE;
            for(u32i = 0; u32i < BUFSIZE; u32i++)
            {
                /* Send 1 byte data */
                UART2->DAT = g_u8TxData[u32i];

                /* Wait if Tx FIFO is full */
                while((UART2->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk)==0);
            }

            /* Wait for UART1 receive done */
            while (g_i32RecOK != TRUE) {};

            /* Verify received data */
            if(Check_Pattern((uint32_t)g_u8TxData, (uint32_t)g_u8RxData, BUFSIZE))
            {
                printf(" Pass\n");
            }
            else
            {
                printf(" Fail\n");
            }
            /* Clear UART RAM buffer */
            memset((uint8_t *)g_u8TxData, 0, BUFSIZE);
            memset((uint8_t *)g_u8RxData, 0, BUFSIZE);
        }
        break;

        default:
            break;
        }

    }
    while ((cmd != 'E') && (cmd != 'e'));

    /* Disable UART1 RDA/Time-out interrupt */
    UART1->INTEN &= ~(UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_SWBEIEN_Msk);
    /* Disable UART2 RDA/Time-out interrupt */
    UART2->INTEN &= ~(UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_SWBEIEN_Msk);
    printf("\nUART Sample Demo End.\n");

}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/



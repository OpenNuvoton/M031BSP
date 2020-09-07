/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief
 *           Two Single-Wire Loopback data test.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include "stdio.h"
#include "string.h"
#include "NuMicro.h"


#define PLL_CLOCK   FREQ_96MHZ
#define BUFSIZE   128

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8RecData[BUFSIZE] = {0};
uint8_t g_u8TxData [BUFSIZE] = {0};
volatile uint32_t g_u32RecLen  =  0;
volatile int32_t  g_i32RecOK  = FALSE;


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
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
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set both PCLK0 and PCLK1 as HCLK */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1;

    /* Enable UART0 peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Enable UART1 peripheral clock */
    CLK_EnableModuleClock(UART1_MODULE);
    /* Enable UART2 peripheral clock */
    CLK_EnableModuleClock(UART2_MODULE);

    /* Select IP clock source */
    /* Select UART0 clock source is HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    /* Select UART1 clock source is HIRC */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));
    /* Select UART2 clock source is HIRC */
    CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HIRC, CLK_CLKDIV4_UART2(1));


    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) | \
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set PA multi-function pins for UART1 TXD, RXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA2MFP_Msk)) | SYS_GPA_MFPL_PA2MFP_UART1_RXD;

    /* Set PB multi-function pins for UART2 TXD and RXD */
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

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

}
/*---------------------------------------------------------------------------------------------------------*/
/*                               Init Single Wire(UART1)                                                   */
/*---------------------------------------------------------------------------------------------------------*/

void UART1_Init()
{
    /* Configure Single Wire(UART1) and set Single Wire(UART1) baud rate */
    //UART_Open(UART0, 115200);
    //UART_SelectSingleWireMode(UART0);
    UART_Open(UART1, 115200);
    UART_SelectSingleWireMode(UART1);
}


/*---------------------------------------------------------------------------------------------------------*/
/*                               Init Single Wire(UART2)                                                   */
/*---------------------------------------------------------------------------------------------------------*/

void UART2_Init()
{
    /* Configure Single Wire(UART2) and set Single Wire(UART2) baud rate */
    //UART_Open(UART1, 115200);
    //UART_SelectSingleWireMode(UART1);

    UART_Open(UART2, 115200);
    UART_SelectSingleWireMode(UART2);

}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Test Sample                                                                                        */
/* Test Item                                                                                               */
/* Debug port control the Single wire 1(UART1) send data to Single wire 2(UART2) or Single wire 2(UART1)   */
/* send data to Single wire 1(UART1)                                                                       */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/*                                         Main Function                                                   */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
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
    UART1_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                  UART1 Callback function                                                */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_TEST_HANDLE()
{

    if (UART_GET_INT_FLAG(UART1,UART_INTSTS_RDAIF_Msk))
    {
        /* Get all the input characters */
        while (UART_IS_RX_READY(UART1))
        {
            /* Get the character from UART Buffer */
            g_u8RecData[g_u32RecLen] = UART_READ(UART1);

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

    if (UART_GET_INT_FLAG(UART1,UART_INTSTS_SWBEIF_Msk))
    {
        printf("Single-wire Bit Error Detection \n");
        UART_ClearIntFlag(UART1, UART_INTSTS_SWBEINT_Msk);
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 2 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART02_IRQHandler(void)
{
    UART02_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART1 Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART02_TEST_HANDLE()
{

    if (UART_GET_INT_FLAG(UART2,UART_INTSTS_RDAIF_Msk))
    {
        /* Get all the input characters */
        while (UART_IS_RX_READY(UART2))
        {
            /* Get the character from UART Buffer */
            g_u8RecData[g_u32RecLen] = UART_READ(UART2);

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

    if (UART_GET_INT_FLAG(UART2,UART_INTSTS_SWBEIF_Msk))
    {
        printf("Single-wire Bit Error Detection \n");
        UART_ClearIntFlag(UART2, UART_INTSTS_SWBEINT_Msk);

    }
}
/*---------------------------------------------------------------------------------------------------------*/
/*                              Bulid Source Pattern function                                              */
/*---------------------------------------------------------------------------------------------------------*/
void Build_Src_Pattern(uint32_t u32Addr, uint8_t type, uint32_t u32Length)
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
    char cmmd ;

    printf("+-----------------------------------------------------------+\n");
    printf("|            UART Single Wire Function Test                 |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will print input char on terminal      |\n");
    printf("|    The user must connect the UART1 RX pin(PA2) to         |\n");
    printf("|    UART_Rx Pin(PB0).                                      |\n");
    printf("|    Single Wire 1(PA2)send data to Single Wire 2(PB0).     |\n");
    printf("|    Single Wire 2(PB0)send data to Single Wire 1(PA2).     |\n");
    printf("|    Please enter any to start    (Press '0' to exit)       |\n");
    printf("+-----------------------------------------------------------+\n");

    /*
        Using a RS232 cable to connect UART0 and PC.UART0 is set to debug port.
          UART1 and UART2 is enable RDA and RLS interrupt.
          The user can use URT0 to control the transmission or reception of UART1(Single Wire mode)
        When UART1(Single Wire 1)transfers data to UART2(Single Wire 2), if data is valid,
          it will enter the interrupt and receive the data.And then check the received data.
        When UART2(Single Wire 2)transfers data to UART1(Single Wire 1), if data is valid,
          it will enter the interrupt and receive the data.And then check the received data.
      */

    /* Enable UART1 RDA/Time-out/Single-wire Bit Error Detection interrupt */
    NVIC_EnableIRQ(UART13_IRQn);
    UART_EnableInt(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_SWBEIEN_Msk));
    /* Enable UART2 RDA/Time-out/Single-wire Bit Error Detection interrupt */
    NVIC_EnableIRQ(UART02_IRQn);
    UART_EnableInt(UART2, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_SWBEIEN_Msk));

    do
    {
        printf("+--------------------------------------------------------------+\n");
        printf("|                UART Single Wire Test Item                    |\n");
        printf("+--------------------------------------------------------------+\n");
        printf("|    (1)Single Wire 1(PA2)send data to Single Wire 2(PB0).     |\n");
        printf("|    (2)Single Wire 2(PB0)send data to Single Wire 1(PA2).     |\n");
        printf("|    (E)Exit                                                   |\n");
        printf("+--------------------------------------------------------------+\n");

        cmmd = getchar();

        switch (cmmd)
        {
        case '1':
        {
            printf("SW1(UART1) --> SW2(UART2)Test :");
            g_i32RecOK  = FALSE;
            Build_Src_Pattern((uint32_t)g_u8TxData, UART_WORD_LEN_8, BUFSIZE);

            /* Check the Rx status is Idel */
            while (!UART_RX_IDLE(UART1)) {};

            UART_Write(UART1, g_u8TxData, BUFSIZE);

            while (g_i32RecOK != TRUE) {}

            Check_Pattern((uint32_t)g_u8TxData, (uint32_t)g_u8RecData, BUFSIZE) ? printf(" Pass\n") : printf(" Fail\n");
            /* Clear the Tx and Rx data buffer */
            memset((uint8_t *)g_u8TxData, 0, BUFSIZE);
            memset((uint8_t *)g_u8RecData, 0, BUFSIZE);
        }
        break;

        case '2':
        {
            printf("SW2(UART2) --> SW1(UART1)Test :");
            g_i32RecOK  = FALSE;
            Build_Src_Pattern((uint32_t)g_u8TxData, UART_WORD_LEN_8, BUFSIZE);

            /* Check the Rx status is Idel */
            while (!UART_RX_IDLE(UART2)) {};

            UART_Write(UART2, g_u8TxData, BUFSIZE);

            while (g_i32RecOK != TRUE) {};

            Check_Pattern((uint32_t)g_u8TxData, (uint32_t)g_u8RecData, BUFSIZE) ? printf(" Pass\n") :   printf(" Fail\n");

            /* Clear the Tx and Rx data buffer */
            memset((uint8_t *)g_u8TxData, 0, BUFSIZE);

            memset((uint8_t *)g_u8RecData, 0, BUFSIZE);
        }
        break;

        default:
            break;
        }

    }
    while ((cmmd != 'E') && (cmmd != 'e'));

    /* Disable UART1 RDA/Time-out interrupt */
    UART_DisableInt(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_SWBEIEN_Msk));
    /* Disable UART2 RDA/Time-out interrupt */
    UART_DisableInt(UART2, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_SWBEIEN_Msk));
    printf("\nUART Sample Demo End.\n");

}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/



/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to use the Timer capture function to capture Timer counter value.
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


volatile uint32_t g_au32TMRINTCount[4] = {0};

void TMR1_IRQHandler(void)
{
    if(TIMER1->EINTSTS == 1)
    {
        /* Clear Timer1 capture trigger interrupt flag */
        TIMER1->EINTSTS = TIMER_EINTSTS_CAPIF_Msk;

        g_au32TMRINTCount[1]++;
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~32 MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for 32MHz clock ready */
    while((CLK->STATUS & CLK_STATUS_HXTSTB_Msk) != CLK_STATUS_HXTSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(1);

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Switch UART0 clock source to XTAL */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART0SEL_Msk) | CLK_CLKSEL1_UART0SEL_HXT;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_UART0DIV_Msk) | CLK_CLKDIV0_UART0(1);

    /* Enable UART0 peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Enable TIMER peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_TMR0CKEN_Msk;
    CLK->APBCLK0 |= CLK_APBCLK0_TMR1CKEN_Msk;
    CLK->APBCLK0 |= CLK_APBCLK0_TMR3CKEN_Msk;
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_TMR0SEL_Msk) | CLK_CLKSEL1_TMR0SEL_HXT;
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_TMR1SEL_Msk) | CLK_CLKSEL1_TMR1SEL_HIRC;
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_TMR3SEL_Msk) | CLK_CLKSEL1_TMR3SEL_HXT;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set multi-function pins for Timer0/Timer3 toggle-output pin and Timer1 event counter pin */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB5MFP_Msk | SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB5MFP_TM0 | SYS_GPB_MFPL_PB4MFP_TM1 | SYS_GPB_MFPL_PB2MFP_TM3);

    /* Set multi-function pin for Timer1 external capture pin */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB14MFP_Msk);
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB14MFP_TM1_EXT;

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
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

int main(void)
{
    uint32_t u32InitCount;
    uint32_t au32CAPValue[10], u32CAPDiff;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();


    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------+\n");
    printf("|    Timer1 Capture Counter Sample Code    |\n");
    printf("+------------------------------------------+\n\n");

    printf("# Timer0 Settings:\n");
    printf("    - Clock source is HXT\n");
    printf("    - Time-out frequency is 1000 Hz\n");
    printf("    - Toggle-output mode and frequency is 500 Hz\n");
    printf("# Timer3 Settings:\n");
    printf("    - Clock source is HXT\n");
    printf("    - Time-out frequency is 2 Hz\n");
    printf("    - Toggle-output mode and frequency is 1 Hz\n");
    printf("# Timer1 Settings:\n");
    printf("    - Clock source is HIRC              \n");
    printf("    - Continuous counting mode          \n");
    printf("    - Interrupt enable                  \n");
    printf("    - Compared value is 0xFFFFFF        \n");
    printf("    - Event counter mode enable         \n");
    printf("    - External capture mode enable      \n");
    printf("    - Capture trigger interrupt enable  \n");
    printf("# Connect TM0(PB.5) toggle-output pin to TM1(PB.4) event counter pin.\n");
    printf("# Connect TM3(PB.2) toggle-output pin to TM1_EXT(PB.14) external capture pin.\n\n");

    /* Enable Timer1 NVIC */
    NVIC_EnableIRQ(TMR1_IRQn);

    /* Open Timer0 in toggle-output mode and toggle-output frequency is 500 Hz*/
    TIMER0->CTL = TIMER_TOGGLE_MODE;
    TIMER0->CMP = __HXT / 1000;

    /* Open Timer3 in toggle-output mode and toggle-output frequency is 1 Hz */
    TIMER3->CTL = TIMER_TOGGLE_MODE;
    TIMER3->CMP = __HXT / 2;

    /* Enable Timer1 event counter input and external capture function */
    TIMER1->CTL = TIMER_CONTINUOUS_MODE;
    TIMER1->CTL = (TIMER1->CTL & ~TIMER_CTL_PSC_Msk);
    TIMER1->CMP = 0xFFFFFF;
    TIMER1->EXTCTL = (TIMER1->EXTCTL & ~TIMER_EXTCTL_CNTPHASE_Msk) | TIMER_COUNTER_FALLING_EDGE;
    TIMER1->CTL |= TIMER_CTL_EXTCNTEN_Msk;
    TIMER1->EXTCTL = (TIMER1->EXTCTL & ~(TIMER_EXTCTL_CAPFUNCS_Msk | TIMER_EXTCTL_CAPEDGE_Msk)) |
                     TIMER_CAPTURE_FREE_COUNTING_MODE | TIMER_CAPTURE_FALLING_EDGE | TIMER_EXTCTL_CAPEN_Msk;
    TIMER1->CTL |= TIMER_CTL_INTEN_Msk;
    TIMER1->EXTCTL |= TIMER_EXTCTL_CAPIEN_Msk;

    /* case 1. */
    printf("# Period between two falling edge captured event should be 500 counts.\n");

    /* Clear Timer1 interrupt counts to 0 */
    u32InitCount = g_au32TMRINTCount[1] = 0;

    /* Start Timer0, Timer3 and Timer1 counting */
    TIMER0->CTL |= TIMER_CTL_CNTEN_Msk;
    TIMER1->CTL |= TIMER_CTL_CNTEN_Msk;
    TIMER3->CTL |= TIMER_CTL_CNTEN_Msk;

    /* Check Timer1 capture trigger interrupt counts */
    while(g_au32TMRINTCount[1] <= 10)
    {
        if(g_au32TMRINTCount[1] != u32InitCount)
        {
            au32CAPValue[u32InitCount] = TIMER1->CAP;
            if(u32InitCount ==  0)
            {
                printf("    [%2d]: %4d. (1st captured value)\n", g_au32TMRINTCount[1], au32CAPValue[u32InitCount]);
            }
            else
            {
                u32CAPDiff = au32CAPValue[u32InitCount] - au32CAPValue[u32InitCount - 1];
                printf("    [%2d]: %4d. Diff: %d.\n", g_au32TMRINTCount[1], au32CAPValue[u32InitCount], u32CAPDiff);
                if(u32CAPDiff != 500)
                {
                    printf("*** FAIL ***\n");
                    while(1);
                }
            }
            u32InitCount = g_au32TMRINTCount[1];
        }
    }
    printf("*** PASS ***\n\n");

    /* case 2. */
    TIMER1->EXTCTL &= ~TIMER_EXTCTL_CAPEN_Msk;
    TIMER1->CTL &= ~TIMER_CTL_CNTEN_Msk;
    while(TIMER1->CTL & TIMER_CTL_ACTSTS_Msk);
    TIMER1->INTSTS = TIMER_INTSTS_TIF_Msk;
    TIMER1->EINTSTS = TIMER_EINTSTS_CAPIF_Msk;

    /* Enable Timer1 event counter input and external capture function */
    TIMER1->CTL = TIMER_CONTINUOUS_MODE;
    TIMER1->CTL = (TIMER1->CTL & ~TIMER_CTL_PSC_Msk);
    TIMER1->CMP = 0xFFFFFF;
    TIMER1->EXTCTL = (TIMER1->EXTCTL & ~TIMER_EXTCTL_CNTPHASE_Msk) | TIMER_COUNTER_FALLING_EDGE;
    TIMER1->CTL |= TIMER_CTL_EXTCNTEN_Msk;
    TIMER1->EXTCTL = (TIMER1->EXTCTL & ~(TIMER_EXTCTL_CAPFUNCS_Msk | TIMER_EXTCTL_CAPEDGE_Msk)) |
                     TIMER_CAPTURE_FREE_COUNTING_MODE | TIMER_CAPTURE_RISING_EDGE | TIMER_EXTCTL_CAPEN_Msk;
    TIMER1->CTL |= TIMER_CTL_INTEN_Msk;
    TIMER1->EXTCTL |= TIMER_EXTCTL_CAPIEN_Msk;
    TIMER1->CTL |= TIMER_CTL_CNTEN_Msk;

    printf("# Get first low duration should be 250 counts.\n");
    printf("# And follows duration between two rising edge captured event should be 500 counts.\n");

    /* Clear Timer1 interrupt counts to 0 */
    u32InitCount = g_au32TMRINTCount[1] = 0;

    /* Enable Timer1 event counter input and external capture function */
    TIMER1->CTL = TIMER_CONTINUOUS_MODE;
    TIMER1->CTL = (TIMER1->CTL & ~TIMER_CTL_PSC_Msk);
    TIMER1->CMP = 0xFFFFFF;
    TIMER1->EXTCTL = (TIMER1->EXTCTL & ~TIMER_EXTCTL_CNTPHASE_Msk) | TIMER_COUNTER_FALLING_EDGE;
    TIMER1->CTL |= TIMER_CTL_EXTCNTEN_Msk;
    TIMER1->EXTCTL = (TIMER1->EXTCTL & ~(TIMER_EXTCTL_CAPFUNCS_Msk | TIMER_EXTCTL_CAPEDGE_Msk)) |
                     TIMER_CAPTURE_FREE_COUNTING_MODE | TIMER_CAPTURE_RISING_EDGE | TIMER_EXTCTL_CAPEN_Msk;
    TIMER1->CTL |= TIMER_CTL_INTEN_Msk;
    TIMER1->EXTCTL |= TIMER_EXTCTL_CAPIEN_Msk;
    TIMER1->CTL |= TIMER_CTL_CNTEN_Msk;

    /* Check Timer1 capture trigger interrupt counts */
    while(g_au32TMRINTCount[1] <= 10)
    {
        if(g_au32TMRINTCount[1] != u32InitCount)
        {
            au32CAPValue[u32InitCount] = TIMER1->CAP;
            if(u32InitCount ==  0)
            {
                printf("    [%2d]: %4d. (1st captured value)\n", g_au32TMRINTCount[1], au32CAPValue[u32InitCount]);
            }
            else
            {
                u32CAPDiff = au32CAPValue[u32InitCount] - au32CAPValue[u32InitCount - 1];
                printf("    [%2d]: %4d. Diff: %d.\n", g_au32TMRINTCount[1], au32CAPValue[u32InitCount], u32CAPDiff);
                if(u32CAPDiff != 500)
                {
                    printf("*** FAIL ***\n");
                    while(1);
                }
            }
            u32InitCount = g_au32TMRINTCount[1];
        }
    }

    /* Stop Timer0, Timer1 and Timer3 counting */
    TIMER0->CTL &= ~TIMER_CTL_CNTEN_Msk;
    TIMER1->CTL &= ~TIMER_CTL_CNTEN_Msk;
    TIMER3->CTL &= ~TIMER_CTL_CNTEN_Msk;

    printf("*** PASS ***\n");

    while(1);
}

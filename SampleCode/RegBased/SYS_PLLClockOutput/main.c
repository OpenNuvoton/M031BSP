/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Change system clock to different PLL frequency and output system clock to CLKO pin.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "NuMicro.h"

#define SIGNATURE       0x125ab234
#define FLAG_ADDR       0x200007FC

/*---------------------------------------------------------------------------------------------------------*/
/*  Brown Out Detector IRQ Handler                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void BOD_IRQHandler(void)
{
    /* Clear BOD Interrupt Flag */
    SYS->BODCTL |= SYS_BODCTL_BODIF_Msk;

    printf("Brown Out is Detected\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Simple calculation test function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define PI_NUM  256
int32_t f[PI_NUM + 1];
uint32_t piTbl[19] =
{
    3141,
    5926,
    5358,
    9793,
    2384,
    6264,
    3383,
    2795,
    288,
    4197,
    1693,
    9937,
    5105,
    8209,
    7494,
    4592,
    3078,
    1640,
    6284
};

int32_t piResult[19];

int32_t pi(void)
{
    int32_t i, i32Err;
    int32_t a = 10000, b = 0, c = PI_NUM, d = 0, e = 0, g = 0;

    for(; b - c;)
        f[b++] = a / 5;

    i = 0;
    for(; d = 0, g = c * 2; c -= 14, piResult[i++] = e + d / a, e = d % a)
    {
        if(i == 19)
            break;

        for(b = c; d += f[b] * a, f[b] = d % --g, d /= g--, --b; d *= b);
    }
    i32Err = 0;
    for(i = 0; i < 19; i++)
    {
        if(piTbl[i] != piResult[i])
            i32Err = -1;
    }

    return i32Err;
}

void Delay(uint32_t x)
{
    int32_t i;

    for(i = 0; i < x; i++)
    {
        __NOP();
        __NOP();
    }
}

uint32_t g_au32PllSetting[] =
{
    28000000,   /* PLL = 56MHz, HCLK = PLL/2  */
    36000000,   /* PLL = 72MHz, HCLK = PLL/2  */
    48000000,   /* PLL = 96MHz, HCLK = PLL/2 */
};

void SetCoreClockPLL(uint32_t u32Hclk)
{
    uint32_t u32NO, u32NR, u32NF;

    /* Switch HCLK clock source to HIRC clock for safe */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 &= (~CLK_CLKDIV0_HCLKDIV_Msk);

    /* Disable PLL first to avoid unstable when setting PLL */
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Enable and apply new PLL setting. */
    /* FOUT = (FIN * NF) / (NR * NO)
       --> NF = (FOUT * NR * NO) / FIN
       FOUT = (u32Hclk * 2) since the HCLK divider will be 2 */
    u32NO = 4;
    u32NR = 3;
    u32NF = ((u32Hclk * 2) * u32NR * u32NO) / (__HIRC/4);
    CLK->PLLCTL = CLK_PLLCTL_PLLSRC_HIRC_DIV4 |         /* PLL source clock is from HIRC/4 */
                  (3 << CLK_PLLCTL_OUTDIV_Pos) |        /* assign 3 for NO = 4 */
                  ((u32NR - 2) << CLK_PLLCTL_INDIV_Pos) |
                  ((u32NF - 2) << CLK_PLLCTL_FBDIV_Pos);

    /* Wait for PLL clock stable */
    while((CLK->STATUS & CLK_STATUS_PLLSTB_Msk) != CLK_STATUS_PLLSTB_Msk);

    /* Select HCLK clock source to PLL and HCLK divider as 2 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_PLL;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(2);

    SystemCoreClockUpdate();

    return;
}

void SYS_PLL_Test(void)
{
    int32_t  i;

    /*---------------------------------------------------------------------------------------------------------*/
    /* PLL clock configuration test                                                                            */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n-------------------------[ Test PLL ]-----------------------------\n");
    printf("  Select HCLK clock source from PLL/2.\n");
    printf("  Please measure HCLK on CLKO pin (PB.14) by scope ...\n");

    for(i = 0; i < sizeof(g_au32PllSetting) / sizeof(g_au32PllSetting[0]) ; i++)
    {
        /* Select HCLK clock source from PLL.
           PLL will be configured to twice specified frequency.
        */
        SetCoreClockPLL(g_au32PllSetting[i]);

        printf("  Change system clock to %d Hz ...................... ", SystemCoreClock);

        /* Output HCLK to CKO, CKO Clock = HCLK / 1 */
        CLK->CLKOCTL = CLK_CLKOCTL_CLKOEN_Msk | 0 | (1 << CLK_CLKOCTL_DIV1EN_Pos);
        /* Enable CKO clock source */
        CLK->APBCLK0 |= CLK_APBCLK0_CLKOCKEN_Msk;
        /* Select CKO clock source */
        CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_CLKOSEL_Msk)) | (CLK_CLKSEL1_CLKOSEL_HCLK);

        /* The delay loop is used to check if the CPU speed is increasing */
        Delay(0x1000000);

        if(pi())
        {
            printf("[FAIL]\n");
        }
        else
        {
            printf("[OK]\n");
        }

        /* Disable CLKO clock */
        CLK->APBCLK0 &= (~CLK_APBCLK0_CLKOCKEN_Msk);
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
    /* Set PB multi-function pins for UART0 RXD and TXD */
    /* Set PB multi-function pins for CLKO(PB.14) */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk | SYS_GPB_MFPH_PB14MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD | SYS_GPB_MFPH_PB14MFP_CLKO);

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
    uint32_t u32data;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+---------------------------------------+\n");
    printf("|      System Driver Sample Code        |\n");
    printf("+---------------------------------------+\n");

    if(M32(FLAG_ADDR) == SIGNATURE)
    {
        printf("  CPU Reset success!\n");
        M32(FLAG_ADDR) = 0;
        printf("  Press any key to continue ...\n");
        getchar();
    }

    /*---------------------------------------------------------------------------------------------------------*/
    /* Misc system function test                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Read Part Device ID */
    printf("Product ID 0x%x\n", SYS->PDID);

    /* Get reset source from last operation */
    u32data = SYS->RSTSTS;
    printf("Reset Source 0x%x\n", u32data);

    /* Clear reset source */
    SYS->RSTSTS = u32data;

    /* Unlock protected registers for Brown-Out Detector settings */
    SYS_UnlockReg();

    /* Check if the write-protected registers are unlocked before BOD setting and CPU Reset */
    if ((!(SYS->REGLCTL & BIT0)) == 0)
    {
        printf("Protected Address is Unlocked\n");
    }

    /* Enable Brown-Out Detector and Low Voltage Reset function, and set Brown-Out Detector voltage 2.5V */
    SYS->BODCTL = (SYS->BODCTL & ~(SYS_BODCTL_BODRSTEN_Msk | SYS_BODCTL_BODVL_Msk)) |
                  (SYS_BODCTL_BOD_INTERRUPT_EN | SYS_BODCTL_BODEN_Msk | SYS_BODCTL_BODVL_2_5V);

    /* Enable BOD interrupt */
    NVIC_EnableIRQ(BOD_IRQn);

    /* Run PLL Test */
    SYS_PLL_Test();

    /* Write a signature work to SRAM to check if it is reset by software */
    M32(FLAG_ADDR) = SIGNATURE;
    printf("\n\n  >>> Reset CPU <<<\n");

    /* Wait for message send out */
    UART_WAIT_TX_EMPTY(UART0);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);

    /* Set PLL to Power down mode and HW will also clear PLLSTB bit in CLK_STATUS register */
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Reset CPU */
    SYS->IPRST0 |= SYS_IPRST0_CPURST_Msk;
}

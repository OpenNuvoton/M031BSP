/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 18/07/24 1:16p $
 * @brief
 *           Show how to wake up MCU from Power-down mode via the I2C interface.
 *           This sample code needs to work with I2C_Master.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_LENGTH    256

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t slave_buff_addr;
volatile uint8_t g_au8SlvData[TEST_LENGTH];
volatile uint8_t g_au8SlvRxData[3];
volatile uint8_t g_u8SlvPWRDNWK, g_u8SlvI2CWK;

volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_u8SlvDataLen;

typedef void (*I2C_FUNC)(uint32_t u32Status);

static I2C_FUNC s_I2C0HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    /* Check I2C Wake-up interrupt flag set or not */
    if ((I2C0->WKSTS & I2C_WKSTS_WKIF_Msk) == I2C_WKSTS_WKIF_Msk)
    {
        /* Make sure that the ACK bit is done */
        while (!(I2C0->WKSTS & I2C_WKSTS_WKAKDONE_Msk)) {};

        /* Clear I2C Wake-up interrupt flag */
        I2C0->WKSTS = I2C_WKSTS_WKIF_Msk;

        /* Clear Wakeup done flag, I2C will release bus */
        I2C0->WKSTS = I2C_WKSTS_WKAKDONE_Msk;
        g_u8SlvI2CWK = 1;

        return;
    }

    u32Status = I2C0->STATUS0;

    if (I2C0->TOCTL & I2C_TOCTL_TOIF_Msk)
    {
        /* Clear I2C0 Timeout Flag */
        I2C0->TOCTL |= I2C_TOCTL_TOIF_Msk;
    }
    else
    {
        if (s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
}
/*---------------------------------------------------------------------------------------------------------*/
/*  Power Wake-up IRQ Handler                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void PWRWU_IRQHandler(void)
{
    /* Check system power down mode wake-up interrupt flag */
    if (((CLK->PWRCTL) & CLK_PWRCTL_PDWKIF_Msk) != 0)
    {
        /* Clear system power down wake-up interrupt flag */
        CLK->PWRCTL |= CLK_PWRCTL_PDWKIF_Msk;
        g_u8SlvPWRDNWK = 1;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Slave Transmit/Receive Callback Function                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveTRx(uint32_t u32Status)
{
    if (u32Status == 0x60)                      /* Own SLA+W has been receive; ACK has been return */
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        g_au8SlvRxData[g_u8SlvDataLen] = (unsigned char)(I2C0->DAT);
        g_u8SlvDataLen++;

        if (g_u8SlvDataLen == 2)
        {
            slave_buff_addr = (g_au8SlvRxData[0] << 8) + g_au8SlvRxData[1];
        }

        if (g_u8SlvDataLen == 3)
        {
            g_au8SlvData[slave_buff_addr] = g_au8SlvRxData[2];
            g_u8SlvDataLen = 0;
        }

        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA8)                 /* Own SLA+R has been receive; ACK has been return */
    {

        I2C0->DAT = g_au8SlvData[slave_buff_addr];
        slave_buff_addr++;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

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
    while((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) != CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);

    /* Enable UART0 clock and I2C controller */
    CLK->APBCLK0 |= (CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_I2C0CKEN_Msk);

    /* Switch UART0 clock source to HIRC and UART0 clock divider as 1 */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART0SEL_Msk)) | CLK_CLKSEL1_UART0SEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UART0DIV_Msk)) | CLK_CLKDIV0_UART0(1);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set I2C0 multi-function pins */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk)) |
                    (SYS_GPB_MFPL_PB4MFP_I2C0_SDA | SYS_GPB_MFPL_PB5MFP_I2C0_SCL);

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void I2C0_Init(void)
{
    uint32_t u32BusClock;

    /* Reset I2C0 */
    SYS->IPRST1 |=  SYS_IPRST1_I2C0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_I2C0RST_Msk;

    /* Enable I2C0 Controller */
    I2C0->CTL0 |= I2C_CTL0_I2CEN_Msk;

    /* I2C0 bus clock 100K divider setting, I2CLK = PCLK/(100K*4)-1 */
    u32BusClock = 100000;
    I2C0->CLKDIV = (uint32_t)(((SystemCoreClock * 10) / (u32BusClock * 4) + 5) / 10 - 1); /* Compute proper divider for I2C clock */

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", (SystemCoreClock / (((I2C0->CLKDIV) + 1) << 2)));

    /* Set I2C0 4 Slave Addresses */
    /* Slave Address : 0x15 */
    I2C0->ADDR0 = (I2C0->ADDR0 & ~I2C_ADDR0_ADDR_Msk) | (0x15 << I2C_ADDR0_ADDR_Pos);
    /* Slave Address : 0x35 */
    I2C0->ADDR1 = (I2C0->ADDR1 & ~I2C_ADDR1_ADDR_Msk) | (0x35 << I2C_ADDR1_ADDR_Pos);
    /* Slave Address : 0x55 */
    I2C0->ADDR2 = (I2C0->ADDR2 & ~I2C_ADDR2_ADDR_Msk) | (0x55 << I2C_ADDR2_ADDR_Pos);
    /* Slave Address : 0x75 */
    I2C0->ADDR3 = (I2C0->ADDR3 & ~I2C_ADDR3_ADDR_Msk) | (0x75 << I2C_ADDR3_ADDR_Pos);

    /* Set I2C0 4 Slave Addresses Mask Bits*/
    /* Slave Address Mask Bits: 0x01 */
    I2C0->ADDRMSK0 = (I2C0->ADDRMSK0 & ~I2C_ADDRMSK0_ADDRMSK_Msk) | (0x01 << I2C_ADDRMSK0_ADDRMSK_Pos);
    /* Slave Address Mask Bits: 0x04 */
    I2C0->ADDRMSK1 = (I2C0->ADDRMSK1 & ~I2C_ADDRMSK1_ADDRMSK_Msk) | (0x04 << I2C_ADDRMSK1_ADDRMSK_Pos);
    /* Slave Address Mask Bits: 0x01 */
    I2C0->ADDRMSK2 = (I2C0->ADDRMSK2 & ~I2C_ADDRMSK2_ADDRMSK_Msk) | (0x01 << I2C_ADDRMSK2_ADDRMSK_Pos);
    /* Slave Address Mask Bits: 0x04 */
    I2C0->ADDRMSK3 = (I2C0->ADDRMSK3 & ~I2C_ADDRMSK3_ADDRMSK_Msk) | (0x04 << I2C_ADDRMSK3_ADDRMSK_Pos);

    /* Enable I2C0 interrupt and set corresponding NVIC bit */
    I2C0->CTL0 |= I2C_CTL0_INTEN_Msk;
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C0_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C0->CTL0 &= ~I2C_CTL0_INTEN_Msk;
    NVIC_DisableIRQ(I2C0_IRQn);

    /* Disable I2C0 and close I2C0 clock */
    I2C0->CTL0 &= ~I2C_CTL0_I2CEN_Msk;
    CLK->APBCLK0 &= ~CLK_APBCLK0_I2C0CKEN_Msk;
}

int main()
{
    uint32_t i;

    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART0_Init();

    /*
        This sample code is I2C SLAVE mode and it simulates EEPROM function
    */
    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("| I2C Driver Sample Code (Slave) for wake-up & access Slave test       |\n");
    printf("|                                                                      |\n");
    printf("| I2C Master (I2C0) <---> I2C Slave(I2C0)                              |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("Configure I2C0 as a slave.\n");
    printf("The I/O connection for I2C0:\n");
    printf("I2C0_SDA(PB.4), I2C0_SCL(PB.5)\n");

    /* Init I2C0 */
    I2C0_Init();

    for (i = 0; i < TEST_LENGTH; i++)
    {
        g_au8SlvData[i] = 0;
    }

    /* I2C function to Transmit/Receive data as slave */
    s_I2C0HandlerFn = I2C_SlaveTRx;

    /* Set I2C0 enter Not Address SLAVE mode */
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable power wake-up interrupt */
    CLK->PWRCTL |= CLK_PWRCTL_PDWKIEN_Msk;
    NVIC_EnableIRQ(PWRWU_IRQn);
    g_u8SlvPWRDNWK = 0;

    /* Enable I2C wake-up */
    I2C0-> WKCTL |= I2C_WKCTL_WKEN_Msk;
    g_u8SlvI2CWK = 0;

    /* Set the processor uses deep sleep as its low power mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Set system Power-down enabled*/
    CLK->PWRCTL |= CLK_PWRCTL_PDEN_Msk;

    printf("\n");
    printf("Enter PD 0x%x 0x%x 0x%x\n", I2C0->CTL0 , I2C0->STATUS0, CLK->PWRCTL);

    printf("\n");
    printf("CHIP enter power down status.\n");

    /* Waiting for UART printf finish*/
    while (((UART0->FIFOSTS) & UART_FIFOSTS_TXEMPTYF_Msk) == 0);

    if (((I2C0->CTL0)&I2C_CTL0_SI_Msk) != 0)
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }

    /*  Use WFI instruction to idle the CPU. NOTE:
        If ICE is attached, system will wakeup immediately because ICE is a wakeup event. */
    __WFI();
    __NOP();
    __NOP();
    __NOP();

    /* Waiting for syteem wake-up and I2C wake-up finish*/
    while ((g_u8SlvPWRDNWK & g_u8SlvI2CWK) == 0);

    /* Wake-up Interrupt Message */
    printf("Power-down Wake-up INT 0x%x\n", (uint32_t)(CLK->PWRCTL & CLK_PWRCTL_PDWKIF_Msk));
    printf("I2C0 WAKE INT 0x%x\n", I2C0->WKSTS);

    /* Disable power wake-up interrupt */
    CLK->PWRCTL &= ~CLK_PWRCTL_PDWKIEN_Msk;
    NVIC_DisableIRQ(PWRWU_IRQn);

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n");
    printf("Slave wake-up from power down status.\n");

    printf("\n");
    printf("Slave Waiting for receiving data.\n");

    while (1);
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 18/07/09 7:03p $
 * @brief
 *           Show how to control SMBus interface and use SMBus protocol between Host and Slave.
 *           This sample code needs to work with I2C_SMBus_Master.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define SMBUS_ALERT_RESPONSE_ADDRESS 0x0C
#define SMBUS_DEFAULT_ADDRESS        0x61
#define ARP_COMMAND 0x01

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
const uint8_t g_u8SlaveAddr[4] = {0x18, 0x38, 0x58, 0x78};

volatile uint32_t slave_buff_addr;
volatile uint8_t g_u8SlvData[256];
volatile uint8_t g_au8RxData[4];
volatile uint8_t g_u8DataLen0;
volatile uint8_t g_u8EndFlag = 0;
volatile uint8_t g_u8SendPEC = 0;
volatile uint8_t g_u8AlertInt0 = 0;
volatile uint8_t g_u8PECErr = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);

static volatile I2C_FUNC s_I2C0HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);

    /* Check Transmit byte done interrupt flag */
    if ((I2C_SMBusGetStatus(I2C0) & I2C_BUSSTS_BCDONE_Msk) == I2C_BUSSTS_BCDONE_Msk)
    {
        I2C_SMBusClearInterruptFlag(I2C0, I2C_BUSSTS_BCDONE_Msk);
        return;
    }

    /* Occur receive PEC packet error */
    if ((I2C_SMBusGetStatus(I2C0) & I2C_BUSSTS_PECERR_Msk) == I2C_BUSSTS_PECERR_Msk)
    {
        I2C_SMBusClearInterruptFlag(I2C0, I2C_BUSSTS_PECERR_Msk);
        return;
    }

    /* Check Alert Interrupt when I2C0 is Host */
    if (((I2C_SMBusGetStatus(I2C0) & I2C_BUSSTS_ALERT_Msk) == I2C_BUSSTS_ALERT_Msk) &
            ((I2C0->BUSCTL & I2C_BUSCTL_BMHEN_Msk) == I2C_BUSCTL_BMHEN_Msk))
    {
        I2C_SMBusClearInterruptFlag(I2C0, I2C_BUSSTS_ALERT_Msk);
        g_u8AlertInt0 = 1;
        return ;
    }

    if (I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
    }
    else
    {
        if (s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Slave TRx Callback Function                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveTRx(uint32_t u32Status)
{
    if (u32Status == 0x60)                      /* Own SLA+W has been receive; ACK has been return */
    {
        g_u8DataLen0 = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        g_au8RxData[g_u8DataLen0] = (unsigned char)I2C_GET_DATA(I2C0);
        g_u8DataLen0++;

        if (g_u8DataLen0 == 2)
        {
            slave_buff_addr = (g_au8RxData[0] << 8) + g_au8RxData[1];
        }

        if (g_u8DataLen0 == 3)
        {
            g_u8SlvData[slave_buff_addr] = g_au8RxData[2];
        }

        if (g_u8DataLen0 == 4)
        {
            if (g_au8RxData[3] != I2C_SMBusGetPECValue(I2C0))
            {
                g_u8PECErr = 1;
            }

            g_u8DataLen0 = 0;
        }

        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA8)                 /* Own SLA+R has been receive; ACK has been return */
    {

        I2C_SET_DATA(I2C0, g_u8SlvData[slave_buff_addr]);
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
        g_u8DataLen0 = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u8DataLen0 = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Slave Alert Callback Function                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveAlert(uint32_t u32Status)
{
    if (u32Status == 0x60)                      /* Own SLA+W has been receive; ACK has been return */
    {
        g_u8DataLen0 = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        g_au8RxData[g_u8DataLen0] = (unsigned char) I2C_GET_DATA(I2C0);
        g_u8DataLen0++;

        if (g_u8DataLen0 == 2)
        {
            slave_buff_addr = (g_au8RxData[0] << 8) + g_au8RxData[1];
        }

        if (g_u8DataLen0 == 3)
        {
            g_u8SlvData[slave_buff_addr] = g_au8RxData[2];

        }

        if (g_u8DataLen0 == 4)
        {
            if (g_au8RxData[3] != I2C_SMBusGetPECValue(I2C0))
            {
                g_u8PECErr = 1;
            }

            g_u8DataLen0 = 0;
        }

        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA8)                 /* Own SLA+R has been receive; ACK has been return */
    {
        I2C_SET_DATA(I2C0, g_u8SlaveAddr[0]);
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
        g_u8DataLen0 = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u8DataLen0 = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xB8)
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Slave Default Address and Acknowledge by Manual Callback Function                                  */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveDefaultAddrACKM(uint32_t u32Status)
{
    if (u32Status == 0x60)                      /* Own SLA+W has been receive; ACK has been return */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0xF0)
    {
        /* Enable ACKMEN & SLV in Receive Mode */
        g_au8RxData[g_u8DataLen0] = (unsigned char) I2C_GET_DATA(I2C0);
        g_u8DataLen0++;

        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);           //Acknowledge by Manual
    }
    else if (u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        g_au8RxData[g_u8DataLen0] = (unsigned char) I2C_GET_DATA(I2C0);
        g_u8DataLen0++;

        if (g_u8DataLen0 == 2)
        {
            if (g_au8RxData[1] != I2C_SMBusGetPECValue(I2C0))
            {
                g_u8PECErr = 1;
            }

            g_u8DataLen0 = 0;
        }

        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA8)                 /* Own SLA+R has been receive; ACK has been return */
    {
        I2C_SET_DATA(I2C0, g_u8SlvData[slave_buff_addr]);
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
        g_u8DataLen0 = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u8DataLen0 = 0;
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
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable I2C0 clock */
    CLK_EnableModuleClock(I2C0_MODULE);

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

    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC2MFP_Msk | SYS_GPC_MFPL_PC3MFP_Msk)) |
                    (SYS_GPC_MFPL_PC2MFP_I2C0_SMBSUS | SYS_GPC_MFPL_PC3MFP_I2C0_SMBAL);

    /* Lock protected registers */
    SYS_LockReg();
}

void I2C0_Init(void)
{
    /* Open I2C module and set bus clock */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C0 clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

    /* Set I2C0 4 Slave addresses */
    /* Slave address : 0x18 */
    I2C_SetSlaveAddr(I2C0, 0, g_u8SlaveAddr[0], 0);   /* Slave Address : 0x18 */
    /* Slave address : 0x38 */
    I2C_SetSlaveAddr(I2C0, 1, g_u8SlaveAddr[1], 0);   /* Slave Address : 0x38 */
    /* Slave address : 0x58 */
    I2C_SetSlaveAddr(I2C0, 2, g_u8SlaveAddr[2], 0);   /* Slave Address : 0x58 */
    /* Slave address : 0x78 */
    I2C_SetSlaveAddr(I2C0, 3, g_u8SlaveAddr[3], 0);   /* Slave Address : 0x78 */


    /* Set I2C0 4 Slave addresses mask bits*/
    /* Slave address mask Bits: 0x04 */
    I2C_SetSlaveAddrMask(I2C0, 0, 0x04);
    /* Slave address mask Bits: 0x02 */
    I2C_SetSlaveAddrMask(I2C0, 1, 0x02);
    /* Slave address mask Bits: 0x04 */
    I2C_SetSlaveAddrMask(I2C0, 2, 0x04);
    /* Slave address mask Bits: 0x02 */
    I2C_SetSlaveAddrMask(I2C0, 3, 0x02);

    /* Enable I2C0 interrupt */
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C0_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C_DisableInt(I2C0);
    NVIC_DisableIRQ(I2C0_IRQn);

    /* Disable I2C0 and close I2C0 clock */
    I2C_Close(I2C0);
    CLK_DisableModuleClock(I2C0_MODULE);

}

int main()
{
    uint32_t i, ch = 0;

    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    while (ch != 0x30)
    {
        /*
            This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
            and Byte Read operations, and check if the read data is equal to the programmed data.
        */
        printf("\n");
        printf("+-------------------------------------------------------+\n");
        printf("|      I2C SMBUS Driver Sample Code                     |\n");
        printf("|                                                       |\n");
        printf("| I2C Master (I2C0) <---> I2C Slave(I2C0)               |\n");
        printf("+-------------------------------------------------------+\n");

        printf("Configure I2C0 as a slave.\n");
        printf("\n");
        printf("I2C0_SDA(PB.4), I2C0_SCL(PB.5), I2C_ALERT(PC.3), I2C_SUSPEND(PC.2)\n");

        /* Init I2C0 */
        I2C0_Init();

        /* I2C0 enter no address SLV mode */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);

        printf("\n");
        printf("I2C Slave Mode is Running.\n");

        for (i = 0; i < 0x100; i++)
        {
            g_u8SlvData[i] = 0;
        }

        printf("\n");
        printf("[1] SMBus Send Bytes Protocol with PEC Test\n");
        printf("[2] SMBus Alert Function Test\n");
        printf("[3] Simple ARP and ACK Control by Manual Test\n");
        printf("[0] Exit\n");
        ch = getchar();

        if (ch == '1')
        {
            /* I2C0 Bus Management init */
            I2C_SMBusOpen(I2C0, I2C_SMBD_ENABLE);

            /* I2C0 Bus PEC check enable */
            I2C_SMBusPECTxEnable(I2C0, I2C_PECTX_DISABLE);

            /* Set I2C0 Payload bytes */
            I2C_SMBusSetPacketByteCount(I2C0, 4);            // I2C0: 1byte address + 3byte data

            /* I2C function to Slave receive/transmit data */
            s_I2C0HandlerFn = I2C_SlaveTRx;

            /* Access Slave with no address mask */
            printf("\n");
            printf(" == SMBus Send Bytes Protocol test ==\n");

            printf("\n");
            printf("Wait Master transmit data\n");
            printf("Press any key to exit\n");
            getchar();
        }
        else if (ch == '2')
        {
            /* I2C0 Bus Management init */
            I2C_SMBusOpen(I2C0, I2C_SMBD_ENABLE);

            /* I2C0 Bus PEC Check enable */
            I2C_SMBusPECTxEnable(I2C0, I2C_PECTX_ENABLE);

            /* Set I2C0 Payload bytes */
            I2C_SMBusSetPacketByteCount(I2C0, 2);            // I2C0: 1byte address + 1byte data

            /* Release Slave Alert pin to Hi */
            I2C_SMBUS_DISABLE_ALERT(I2C0);

            /* Set Slave SUSCON pin is Input */
            I2C_SMBUS_SET_SUSCON_IN(I2C0);

            g_u8PECErr = 0;

            /* I2C function to Slave for receive/transmit data */
            s_I2C0HandlerFn = I2C_SlaveAlert;

            printf("\n");
            printf(" == SMBus Alert Function Test ==\n");

            /* I2C0 Slave has a Alert request, pull Alert Pin to Lo */
            I2C_SMBUS_ENABLE_ALERT(I2C0);

            printf("\n");
            printf("I2C0 has Alert Request and Alert Pin Pull Lo. \n");

            /* Show Slave SUSCON pin state before Master pull Lo */
            printf("\n");
            printf("Press any key to show I2C0 SUSCON pin state\n");
            getchar();
            printf("I2C0 SUSCON Pin state is %d\n", (int)((I2C0->BUSSTS & I2C_BUSSTS_SCTLDIN_Msk) >> 4));

            /* Wait Master pull SUSCON pin Low */
            printf("\n");
            printf("Press any key to show I2C0 SUSCON pin state\n");
            getchar();

            /* Show I2C0 SUSCON pin state after Master pull Lo */
            printf("I2C0 SUSCON Pin state change to %d\n", (int)((I2C0->BUSSTS & I2C_BUSSTS_SCTLDIN_Msk) >> 4));

            /* Release Slave Alert pin to Hi */
            I2C_SMBUS_DISABLE_ALERT(I2C0);

            printf("\n");
            printf("SMBus Alert Test Done\n");
            printf("Press any key to exit\n");
            getchar();
        }
        else if (ch == '3')
        {
            /* I2C0 Bus Management init */
            I2C_SMBusOpen(I2C0, I2C_SMBD_ENABLE);

            /* I2C0 Bus PEC check enable */
            I2C_SMBusPECTxEnable(I2C0, I2C_PECTX_DISABLE);

            /* I2C0 Acknowledge by Manual enable */
            I2C_SMBUS_ACK_MANUAL(I2C0);

            /* Set I2C0 Payload bytes */
            I2C_SMBusSetPacketByteCount(I2C0, 2);            // I2C0: 1byte address + 1byte data

            g_u8PECErr = 0;

            /* I2C function to Slave receive/transmit data */
            s_I2C0HandlerFn = I2C_SlaveDefaultAddrACKM;

            printf("\n");
            printf("== Simple ARP and Acknowledge by Manual Test ==\n");
            printf("Press any key to show ARP command\n");
            getchar();

            /* Show Slave get ARP command from Master */
            printf("\n");
            printf("Slave Get ARP Command is 0x%X\n", g_au8RxData[0]);

            /* Check Slave get command */
            printf("\n");

            if (g_au8RxData[0] != ARP_COMMAND)
            {
                printf("Get Wrong ARP Command, Please Check again !\n");
            }
            else
            {
                printf("Default Address and Acknowledge by Manual test OK.\n");
            }
            printf("Press any key to exit\n");
            getchar();
        }
    }

    s_I2C0HandlerFn = NULL;

    printf("SMBus Test Exit\n");

    /* Close I2C0 */
    I2C0_Close();

    while (1);
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

/**************************************************************************//**
 * @file     main.c
 * @version  V0.9
 * $Revision: 01 $
 * @brief
 *           Demonstrate BLE operation.
 *           Includes the basic initialization and the loop for kernel(BLE) operations.
 * @note
 *
 ******************************************************************************/
#include <stdio.h>
#include "mcu_definition.h"
#include "BleAppSetting.h"
#include "rf_phy.h"
#include "porting_spi.h"
#include "porting_misc.h"
#include "porting_rfpower.h"
#include "ble_stack_status.h"

/**************************************************************************
* Application Definitions
**************************************************************************/
uint8_t uartBuffer[DEFAULT_MTU];
extern uint8_t  TRSPX_mtu;

/**************************************************************************
 * Extern Function
 **************************************************************************/
extern void trspx_send(uint8_t *data, uint16_t len);  //send out RF data


/**************************************************************************
 * Function
 **************************************************************************/

/*!
   \brief Initial necessary peripheral on MCU.
*/
void SYS_Init(void)
{
    int8_t irqno;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
#if (_USE_MCU_CLK_==MCU_CLK_SOURCE_HXT)         //HXT
    GPIO_SetMode(PF, (BIT2 | BIT3), GPIO_MODE_INPUT);
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk | CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk | CLK_STATUS_HIRCSTB_Msk);
#elif (_USE_MCU_CLK_==MCU_CLK_SOURCE_HIRC)      //HIRC
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
#elif (_USE_MCU_CLK_==MCU_CLK_SOURCE_PLL)       //PLL
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    /* Set core clock */
    CLK_SetCoreClock(CPU_CLOCK_RATE);
#endif

    /* Set both PCLK0 and PCLK1 as HCLK/PCLK_DIV */
#if (PCLK_DIV==4)
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV4 | CLK_PCLKDIV_APB1DIV_DIV4;
#elif (PCLK_DIV==2)
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2;  //48/2=24MHz
#elif (PCLK_DIV==1)
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1;
#endif //(PCLK_DIV==4)

    /*---------------------------------------------------------------------------------------------------------*/
    /* Debug print use UART0                                                                              */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Select HIRC as the clock source of UART0 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PA multi-function pins for UART0 RXD=PA.0 and TXD=PA.1 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk)) |
                    (SYS_GPA_MFPL_PA0MFP_UART0_RXD | SYS_GPA_MFPL_PA1MFP_UART0_TXD);


    /* Set only BLE interrupt with the highest priority to make sure RF can handle event in time */
    for (irqno = BOD_IRQn; irqno <= RTC_IRQn; irqno++)
    {
        NVIC_SetPriority((IRQn_Type)irqno, 1);
    }
#if (_CHIP_SELECTION_ == _CHIP_M031BT)
    NVIC_SetPriority(GPIO_PCPDPEPF_IRQn, 0);
#elif (_CHIP_SELECTION_ == _CHIP_M032BT)
    NVIC_SetPriority(EINT135_IRQn, 0);
#endif

    /* Change HIRC stable count; 0x00 = 1024, 0x01 = 512 (default), 0x02 = 256, 0x03 = 16 */
    CLK->PWRCTL = (CLK->PWRCTL & ~(0x3 << 16)) | (0x02 << 16);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Update System Core Clock                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

void UART_Init(void)
{
    /* Open UART0 for debug */
    UART_Open(UART0, 115200);

    /* Enable Data wake up function */
    UART0->WKCTL |= UART_WKCTL_WKDATEN_Msk;

    /* Set UART data wake-up start bit compensation value */
    UART0->DWKCOMP = 256;

    /* Enable UART wake up interrupt */
    UART_EnableInt(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_WKIEN_Msk));

    NVIC_EnableIRQ(UART02_IRQn);
}

BleStackStatus BLE_StackInit()
{
    BleStackStatus status;

    /* Wait RF PHY stable */
    CLK_SysTickDelay(200000);    // MUST wait for stable voltage( >= 1.8V) after power-up

    /* Do Gpio Reset */
    seBLE_GpioReset();
    CLK_SysTickDelay(50000);     // HW 32K clk count 15ms, but need to consider 32K deviation & MCU HIRC deviation

    /* SPI IO remapping */
    setRF_SpiIoMapping();

    /* Init SPI PDMA */
    setBLE_SpiPDMAInit();

#if (_USE_MCU_CLK_==MCU_CLK_SOURCE_PLL)
    /* Set MCU Wake-up Retention Time to 625us */
    status = setMCU_WakeupRetentionTime(5);
    BLESTACK_STATUS_CHECK(status);
#endif

    /* Init RF PHY */
    status = setRF_Init(DCDC_REGULATOR, XTAL_16M);   //EnableGpioInterrupt in the end of this function
    BLESTACK_STATUS_CHECK(status);

    /* Init BLE Stack */
    status = setBLE_BleStackInit();
    BLESTACK_STATUS_CHECK(status);

    /* Set the TX Power remapping table for different chipset */
    setBLE_TxPower_Wrap_Init();

    return BLESTACK_STATUS_SUCCESS;
}

/*!
   \brief main loop for initialization and BLE kernel
*/
int main(void)
{
    BleStackStatus status;

    extern void BleApp_Main(void);
    extern BleStackStatus BleApp_Init(void);
    extern BleStackStatus Set_BleAddr(void);
    extern volatile uint8_t tx_data_transmit_enable;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 */
    UART_Init();

    /* Init BLE Stack */
    status = BLE_StackInit();
    if (status != BLESTACK_STATUS_SUCCESS)
    {
        printf("BLE_StackInit() returns fail %d\n", status);
        while (1);
    }

    printf("-----------------------------------------\n");
    printf("  BLE demo: TRSP_UART(Multi-Central) start.....\n");
    printf("-----------------------------------------\n");

    printf("CPU clock run @%d\n", CLK_GetCPUFreq());

    /* Set BLE device address */
    Set_BleAddr();

    /* Init BLE App */
    status = BleApp_Init();
    if (status != BLESTACK_STATUS_SUCCESS)
    {
        printf("BleApp_Init() returns fail %d\n", status);
        while (1);
    }

#if (BLE_USE_TIMER == ENABLE_DEF)
    setBLE_TimerInit();
#endif

    while (1)
    {
#if (BLE_USE_TIMER == ENABLE_DEF)
        /* Stop Timer3 to prevent re-entry setBLE_KernelStateHandle() */
        TIMER_Stop(TIMER3);
#endif
        /* Run BLE kernel, the task priority is LL > Host */
        if (setBLE_KernelStateHandle() == BLESTACK_STATUS_FREE)
        {
#if (BLE_USE_TIMER == ENABLE_DEF)
            /* Reset Timer3 counter value and internal prescale counter value */
            TIMER_ResetCounter(TIMER3);
            /* Start Timer3 counting */
            TIMER_Start(TIMER3);
#endif
            /* Run user application */
            BleApp_Main();

            /* Make sure UART TX is empty and RX is idle, and the there is no BLE transmit */
            if (((UART0->FIFOSTS & (UART_FIFOSTS_RXIDLE_Msk | UART_FIFOSTS_TXEMPTYF_Msk)) == (UART_FIFOSTS_RXIDLE_Msk | UART_FIFOSTS_TXEMPTYF_Msk)) &&
                    (tx_data_transmit_enable == 0))
            {
                /* System enter Power Down mode & wait interrupt event. */
                setMCU_SystemPowerDown();
            }
        }
    }
}


//show data on UART
void UART_TX_Send(uint32_t len, uint8_t *ptr)
{
    uint32_t i;

    for (i = 0; i < len; i++)
    {
        UART_WRITE(UART0, *ptr++);
        UART_WAIT_TX_EMPTY(UART0);
    }

    //add a new line
    UART_WRITE(UART0, 0x0D);
    UART_WRITE(UART0, 0x0A);
    UART_WAIT_TX_EMPTY(UART0);
}


//received UART data ISR: send out data by RF
/* Be careful that Central device should enable NOTIFY */
void UART02_IRQHandler(void)
{
    static uint32_t index = 0u;
    uint8_t volatile uartReceiveByte;

    uint32_t u32IntSts = UART0->INTSTS;

    if (u32IntSts & UART_INTSTS_WKINT_Msk)
    {
        /* Clear UART data wakeup flag */
        if (UART0->WKSTS == UART_WKSTS_DATWKF_Msk)
        {
            UART0->WKSTS = UART_WKSTS_DATWKF_Msk;
        }
        else
        {
            printf("Unexpected INT! WKSTS[0x%X]\n", UART0->WKSTS);
            while (1);
        }
    }

    if (u32IntSts & UART_INTSTS_RDAINT_Msk)
    {
        while (UART_IS_RX_READY(UART0))
        {
            uartReceiveByte = UART_READ(UART0);
            uartBuffer[index++] = uartReceiveByte;

            if (index >= TRSPX_mtu)
            {
                trspx_send(uartBuffer, index);              //Send out UART data by RF
                index = 0;
            }
            else if ((uartBuffer[index - 1] == '\r') || (uartBuffer[index - 1] == '\n'))
            {
                if (index > 1)
                {
                    uint32_t length = (uint32_t)index - 1;  //Remove '\r' or '\n'
                    trspx_send(uartBuffer, length);         //Send out UART data by RF
                }
                index = 0;
            }
        }
    }
}


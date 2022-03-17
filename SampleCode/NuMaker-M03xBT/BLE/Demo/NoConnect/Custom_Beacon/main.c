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
#include "rf_phy.h"
#include "porting_spi.h"
#include "porting_misc.h"
#include "porting_rfpower.h"
#include "ble_stack_status.h"

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


    /*---------------------------------------------------------------------------------------------------------*/
    /* Update System Core Clock                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
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


void MCU_Timer0Enable(void)
{
    /* Enable TIMER module clock */
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_LIRC, 0);

    /* Open Timer0 in periodic mode, enable interrupt and 1 interrupt tick per second */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1);
    TIMER_EnableInt(TIMER0);

    /* Enable Timer0 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);

    /* Start Timer0 counting */
    TIMER_Start(TIMER0);
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

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Open UART0 for debug */
    UART_Open(UART0, 115200);

    /* Init BLE Stack */
    status = BLE_StackInit();
    if (status != BLESTACK_STATUS_SUCCESS)
    {
        printf("BLE_StackInit() returns fail %d\n", status);
        while (1);
    }

    /* Enable Timer0 for send heart rate measurement */
    MCU_Timer0Enable();


    printf("--------------------------------------------\n");
    printf("  BLE demo: Custom Beacon start...\n");
    printf("--------------------------------------------\n");

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

    while (1)
    {
        /* Run BLE kernel, the task priority is LL > Host */
        if (setBLE_KernelStateHandle() == BLESTACK_STATUS_FREE)
        {
            /* Run user application */
            BleApp_Main();

            /* System enter Power Down mode & wait interrupt event. */
            setMCU_SystemPowerDown();
        }
    }
}


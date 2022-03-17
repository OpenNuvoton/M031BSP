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
#include "ble_dtm.h"
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
#if (_USE_MCU_CLK_==MCU_CLK_SOURCE_HXT) //HXT
    GPIO_SetMode(PF, (BIT2 | BIT3), GPIO_MODE_INPUT);
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk | CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk | CLK_STATUS_HIRCSTB_Msk);
#else                                   //HIRC
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
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

    /* Enable TIMER module clock */
    CLK_EnableModuleClock(TMR0_MODULE);     //N*625us timer
    CLK_EnableModuleClock(TMR2_MODULE);     //260us timer
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0); //50MHz
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_PCLK1, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Update System Core Clock                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}


BleStackStatus BleRF_Init()
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

    /* Init RF PHY */
    status = setRF_Init(DCDC_REGULATOR, XTAL_16M);   //EnableGpioInterrupt in the end of this function
    BLESTACK_STATUS_CHECK(status);

    return BLESTACK_STATUS_SUCCESS;
}


BleDtmStatus BleDTM_SysInit(void)
{
    BleDtmStatus status;

    /* init BLE DTM */
    status = BleDTM_init();
    if (status != BLEDTM_SUCCESS)
    {
        return status;
    }

    /* enable BLE DTM timer */
    setDTM_timerInt_enable();

    return BLEDTM_SUCCESS;
}


/*!
   \brief main loop for initialization and BLE kernel
*/
int main(void)
{
    uint8_t status;
    extern void BleDTM_Main(void);

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* The BLE DTM 2-wire UART :
    * Baud rate: 4522, 2400 9600, 14400, 19200, 38400, 57600, 115200.
    * Number of data bits: 8
    * No parity
    * 1 stop bit
    * No flow control (RTS or CTS)
    */
    UART_Open(UART0, 115200);

    /* Init BLE Stack */
    status = BleRF_Init();
    if (status != BLESTACK_STATUS_SUCCESS)
    {
        printf("BLE_StackInit() returns fail %d\n", status);
        while (1);
    }

    /* Init BLE DTM */
    status = BleDTM_SysInit();
    if (status != BLEDTM_SUCCESS)
    {
        printf("BleDTM_SysInit() returns fail %d\n", status);
        while (1);
    }

    while (1)
    {
        BleDTM_Main();
    }
}


/*----------------------------------------------------------------------------*/
/* This file implement MCU peripherals like: UART, GPIO, etc for BLE          */
/*----------------------------------------------------------------------------*/
#include "rf_phy.h"
#include "ble_cmd.h"
#include "ble_dtm.h"
#include "porting_misc.h"
#include "porting_spi.h"
#include "BleAppSetting.h"

// check if defined BLE_DTM_ENABLE
#ifndef BLE_DTM_ENABLE
#define BLE_DTM_ENABLE  DISABLE_DEF
#endif

/**************************************************************************
* Private Functions
**************************************************************************/
/** MCU Initial RF Reset Pin
 */
static void setGpioResetInit(void)
{
    //Assign RF Reset pin
#if (_CHIP_SELECTION_ == _CHIP_M031BT)
    GPIO_SetMode(PA, BIT12, GPIO_MODE_OUTPUT);
#elif (_CHIP_SELECTION_ == _CHIP_M032BT)
    GPIO_SetMode(PH, BIT4,  GPIO_MODE_OUTPUT);
#endif
    RESET_RF = 1;  //RESET_RF defined in porting_misc.h
}


/** MCU Set RF Reset to Idle Mode to Avoid Leakage Current.
 *
 */
static void setGpioResetIdle(void)
{
    //Assign RF Reset pin
#if (_CHIP_SELECTION_ == _CHIP_M031BT)
    GPIO_SetMode(PA, BIT12, GPIO_MODE_INPUT);
#elif (_CHIP_SELECTION_ == _CHIP_M032BT)
    GPIO_SetMode(PH, BIT4,  GPIO_MODE_INPUT);
#endif
}


/** MCU Set General GPIO Pin.
 *
 * @note Here defines RF interrupt pin and LED pin.
 */
static void setGpioPinInit(void)
{
    //Configure Interrupt pin as Input mode
#if (_CHIP_SELECTION_ == _CHIP_M031BT)
    GPIO_SetMode(PC, BIT2, GPIO_MODE_INPUT);
#elif (_CHIP_SELECTION_ == _CHIP_M032BT)
    GPIO_SetMode(PD, BIT12, GPIO_MODE_INPUT);
#endif

    //LED pin assign
    GPIO_SetMode(PF, BIT15, GPIO_MODE_OUTPUT); //LED
    PF15 = 1;                                  //initial off
}


/** spiWriteReg.
 *
 * @note Writes to an 8-bit register with the SPI port.
 */
static void spiGpioWriteReg(const unsigned char regAddr, const unsigned char regData)
{
    unsigned char SPICount;                               // Counter used to clock out the data
    unsigned char SPIData;                                // Define a data structure for the SPI data.

    SPI_CS = 1;                                           // Make sure we start with /CS high
    SPI_CK = 0;                                           // and CK low
    SPI_CS = 0;                                           // Set /CS low to start the SPI cycle 25nS


    setMCU_TinyDelay(1);

    //Address 1th byte
    SPIData = regAddr & 0x7F;
    for (SPICount = 0; SPICount < 8; SPICount++)          // Prepare to clock out the Address byte
    {
        if (SPIData & 0x80)                               // Check for a 1
        {
            SPI_MOSI = 1;    // and set the MOSI line appropriately
        }
        else
        {
            SPI_MOSI = 0;
        }

        setMCU_TinyDelay(1);                              // Delay half clk cycle
        SPI_CK = 1;                                       // Toggle the clock line
        setMCU_TinyDelay(1);
        SPI_CK = 0;
        SPIData <<= 1;                                    // Rotate to get the next bit
    }                                                     // and loop back to send the next bit
    // Repeat for the Data byte
    //Address 2nd byte
    SPIData = (regAddr & 0x80) >> 7;
    for (SPICount = 0; SPICount < 8; SPICount++)          // Prepare to clock out the Address byte
    {
        if (SPIData & 0x80)                               // Check for a 1
        {
            SPI_MOSI = 1;    // and set the MOSI line appropriately
        }
        else
        {
            SPI_MOSI = 0;
        }

        setMCU_TinyDelay(1);                              // Delay half clk cycle
        SPI_CK = 1;                                       // Toggle the clock line
        setMCU_TinyDelay(1);
        SPI_CK = 0;
        SPIData <<= 1;                                    // Rotate to get the next bit
    }

    //Data
    SPIData = regData;                                    // Preload the data to be sent with Data
    for (SPICount = 0; SPICount < 8; SPICount++)          // Prepare to clock out the Data
    {
        if (SPIData & 0x80)
        {
            SPI_MOSI = 1;
        }
        else
        {
            SPI_MOSI = 0;
        }

        setMCU_TinyDelay(1);
        SPI_CK = 1;
        setMCU_TinyDelay(1);
        SPI_CK = 0;
        SPIData <<= 1;
    }

    setMCU_TinyDelay(1);

    SPI_CS = 1;
    SPI_MOSI = 0;
}


/** SPI_GPIO_Init.
 *
 */
static void SPI_GPIO_Init(void)
{
    //For SPI I/O remapping, set all GPIO pin as output
#if (_CHIP_SELECTION_ == _CHIP_M031BT)
    GPIO_SetMode(PD, BIT2, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PD, BIT0, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PD, BIT1, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PD, BIT3, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PC, BIT2, GPIO_MODE_OUTPUT);
#elif (_CHIP_SELECTION_ == _CHIP_M032BT)
    GPIO_SetMode(PF, BIT6,  GPIO_MODE_OUTPUT);
    GPIO_SetMode(PF, BIT7,  GPIO_MODE_OUTPUT);
    GPIO_SetMode(PF, BIT8,  GPIO_MODE_OUTPUT);
    GPIO_SetMode(PF, BIT9,  GPIO_MODE_OUTPUT);
    GPIO_SetMode(PD, BIT12, GPIO_MODE_OUTPUT);
#endif
}


/** SYS_Disable_AnalogPORCircuit.
 *
 */
static void SYS_Disable_AnalogPORCircuit(void)
{
    SYS->PORDISAN = 0x5AA5;
}



/******************************************************************************
 * Public Functions
 ******************************************************************************/
/** MCU Initial RF Reset Pin
 *
 */
void seBLE_GpioReset(void)
{
    //Init reset gpio pin
    setGpioResetInit();

    //Do Reset: pulse low
    RESET_RF = 1;
    setMCU_TinyDelay(1000);       //1ms

    RESET_RF = 0;
    setMCU_TinyDelay(1000);       //1ms

    RESET_RF = 1;

    //Set reset pin to input mode to avoid leakage current in sleep/ deep sleep mode
    setGpioResetIdle();
}



/** MCU Set Enable RF Interrupt Pin.
 *
 */
void setBLE_GpioIntEnable(void)
{
#if (_CHIP_SELECTION_ == _CHIP_M031BT)
    // Configure PC.2 as Input mode and enable interrupt by rising edge trigger
    GPIO_SetMode(PC, BIT2, GPIO_MODE_INPUT);

    if (getBLE_ChipId() == MP_A1)
    {
        GPIO_EnableInt(PC, 2, GPIO_INT_RISING);
    }
    else
    {
        GPIO_EnableInt(PC, 2, GPIO_INT_HIGH);
    }
    NVIC_EnableIRQ(GPIO_PCPDPEPF_IRQn);
#elif (_CHIP_SELECTION_ == _CHIP_M032BT)
    SYS_UnlockReg();

    // Set PD.12 to EINT5 mode ...
    SYS->GPD_MFPH &= (~SYS_GPD_MFPH_PD12MFP_Msk);
    SYS->GPD_MFPH |= SYS_GPD_MFPH_PD12MFP_INT5;

    GPIO_CLR_INT_FLAG(PD, BIT12);
    if (getBLE_ChipId() == MP_A1)
    {
        GPIO_EnableInt(PD, 12, GPIO_INT_RISING);
    }
    else
    {
        GPIO_EnableInt(PD, 12, GPIO_INT_HIGH);
    }

    // Enable NVIC EINT5 interrupt
    NVIC_EnableIRQ(EINT135_IRQn);

    SYS_LockReg();
#endif  // _CHIP_SELECTION_
}


/** MCU Set Disable RF Interrupt Pin.
 */
void setBLE_GpioIntDisable(void)
{
#if (_CHIP_SELECTION_ == _CHIP_M031BT)
    GPIO_DisableInt(PC, 2);
#elif (_CHIP_SELECTION_ == _CHIP_M032BT)
    GPIO_DisableInt(PD, 12);
#endif
    NVIC_DisableIRQ(GPIO_PCPDPEPF_IRQn);
}


/** MCU SPI IO mapping.
 *  Must do this after Power ON and MCU GPIO initialed.
 *
 */
void setRF_SpiIoMapping(void)
{
    //(1) Set all SPI GPIO pin as output. Be careful not to set 5 pins as HIGH, it will trigger HW mapping SPI pin
    SPI_GPIO_Init();                //Here set all CLK,CS,MOSI,NISO,INT as output
    SPI_CS = 1;
    SPI_CK = 0;
    SPI_MOSI = 0;
    SPI_MISO = 0;
    DEFAULT_INT = 0;

    //(2) Write R248, R249 GPIO select
    //Write R248=8'b01,000,100, R249=8'b0,010,011,0
    //GPIO0[2:0]=4 - INT
    //GPIO1[2:0]=0 - CS
    //GPIO2[2:0]=1 - CK
    //GPIO3[2:0]=3 - MISO
    //GPIO4[2:0]=2 - MOSI
    setMCU_TinyDelay(50);
    spiGpioWriteReg(248, 4 | (0 << 3) | ((1 & 0x03) << 6));
    setMCU_TinyDelay(50);
    spiGpioWriteReg(249, ((1 & 0x04) >> 2) | (3 << 1) | (2 << 4));
    setMCU_TinyDelay(50);

    //(3) Output all pin as HIGH, last 10ms(>1ms). trigger HW take effect
    SPI_GPIO_Init();                //Set all as output
    SPI_CS = 1;
    SPI_CK = 1;
    SPI_MOSI = 1;
    SPI_MISO = 1;
    DEFAULT_INT = 1;
    setMCU_TinyDelay(10000);        //Delay 10ms

    //(4) Init GPIO & SPI
    setGpioPinInit();               //Set GPIO interrupt pin as input
    setBLE_SpiInit();               //Initial SPI pin, change MISO direction as INPUT

    setBLE_SpiOneByteTx(249, ((1 & 0x04) >> 2) | (3 << 1) | (2 << 4) | (1 << 7)); //set RF MISO, INT as output

    //manual control
    setBLE_SpiOneByteTx(53, 0xC0);

    //enable LDO
    setBLE_SpiOneByteTx(40, 0xC0);
    setBLE_SpiWaitAndSSHigh();
    setMCU_TinyDelay(25000);        //Put delay after LDO_enable, or set register may have strange behavior!

    //enable chip
    setBLE_SpiOneByteTx(53, 0x80);  //Enable chip
    setBLE_SpiWaitAndSSHigh();
    setMCU_TinyDelay(10000);        //Put delay after chip_enable, or set register may have strange behavior!
}



/** MCU Enter System Power Down Mode.
 *
 */
void setMCU_SystemPowerDown(void)
{
    if (getRF_Mode() != BLERFMODE_ACTIVE) //RF is idle or sleep mode
    {
        /* Check if all the debug messages are finished */
        UART_WAIT_TX_EMPTY(UART0);

        /* Unlock protected registers before entering Power-down mode */
        SYS_UnlockReg();

        /* LVR must be enabled and the POR will be enabled automatically */
        SYS_ENABLE_LVR();

        /* Turn off internal analog POR circuit */
        SYS_Disable_AnalogPORCircuit();

        /* Disable Power-on Reset */
        SYS_DISABLE_POR();

        /* Enter to Power-down mode */
        CLK_PowerDown();

        /* Lock protected registers */
        SYS_LockReg();
    }
    else
    {
        CLK_Idle();
    }
}



/** MCU Implemented Tiny Delay Function.
 *
 * @param[in] u32Usec   : delay time in microseconds.
 */
void setMCU_TinyDelay(uint32_t u32Usec)
{
    CLK_SysTickDelay(u32Usec);
}


/* GPIO Interrupt Handler */
#if defined (__CC_ARM)
#pragma push
#pragma Otime
#endif

#if (_CHIP_SELECTION_ == _CHIP_M031BT)
void GPCDEF_IRQHandler(void)
{
    volatile uint32_t temp;

    //Clear MCU GPIO Int status
    if (GPIO_GET_INT_FLAG(PC, BIT2))
    {
        GPIO_CLR_INT_FLAG(PC, BIT2);

#if (BLE_DTM_ENABLE == ENABLE_DEF)
        /* BLE DTM INT ISR */
        BleDTM_Isr();
#else
        /* BLE GPIO INT ISR */
        setBLE_BleStackGpio_Isr();
#endif
    }
    else
    {
        /* Un-expected interrupt. Just clear all PC interrupts */
        temp = PC->INTSRC;
        PC->INTSRC = temp;
    }
}
#elif (_CHIP_SELECTION_ == _CHIP_M032BT)
void EINT135_IRQHandler(void)
{
    volatile uint32_t temp;

    //Clear MCU GPIO Int status
    if (GPIO_GET_INT_FLAG(PD, BIT12))
    {
        GPIO_CLR_INT_FLAG(PD, BIT12);

#if (BLE_DTM_ENABLE == ENABLE_DEF)
        /* BLE DTM INT ISR */
        BleDTM_Isr();
#else
        /* BLE GPIO INT ISR */
        setBLE_BleStackGpio_Isr();
#endif
    }
    else
    {
        /* Un-expected interrupt. Just clear all PC interrupts */
        temp = PD->INTSRC;
        PD->INTSRC = temp;
    }
}
#endif

#if defined (__CC_ARM)
#pragma pop
#endif

#if (BLE_USE_TIMER == ENABLE_DEF)
void setBLE_TimerInit(void)
{
    /* Enable TIMER module clock */
    CLK_EnableModuleClock(TMR3_MODULE);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_HIRC, 0);

    /* Open Timer3 in periodic mode, enable interrupt and 33 interrupt ticks per second */
    TIMER_Open(TIMER3, TIMER_PERIODIC_MODE, 33);
    TIMER_EnableInt(TIMER3);

    /* Enable Timer3 NVIC */
    NVIC_EnableIRQ(TMR3_IRQn);

    /* Start Timer3 counting */
    TIMER_Start(TIMER3);
}

void TMR3_IRQHandler(void)
{
    if (TIMER_GetIntFlag(TIMER3) == 1)
    {
        /* Clear Timer3 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER3);

        /* Run BLE kernel, the task priority is LL > Host */
        setBLE_KernelStateHandle();
    }
}
#endif


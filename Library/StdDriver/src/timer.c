/**************************************************************************//**
 * @file     timer.c
 * @version  V3.00
 * $Revision: 5 $
 * $Date: 18/07/13 5:00p $
 * @brief    M031 Series Timer Controller (TIMER) Driver Source File
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include "M031Series.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup TIMER_Driver TIMER Driver
  @{
*/

/** @addtogroup TIMER_EXPORTED_FUNCTIONS TIMER Exported Functions
  @{
*/

/**
  * @brief      Open Timer with Operate Mode and Frequency
  *
  * @param[in]  timer       The pointer of the specified Timer module.
  * @param[in]  u32Mode     Operation mode. Possible options are
  *                         - \ref TIMER_ONESHOT_MODE
  *                         - \ref TIMER_PERIODIC_MODE
  *                         - \ref TIMER_TOGGLE_MODE
  *                         - \ref TIMER_CONTINUOUS_MODE
  * @param[in]  u32Freq     Target working frequency
  *
  * @return     Real timer working frequency
  *
  * @details    This API is used to configure timer to operate in specified mode and frequency.
  *             If timer cannot work in target frequency, a closest frequency will be chose and returned.
  * @note       After calling this API, Timer is \b NOT running yet. But could start timer running be calling
  *             \ref TIMER_Start macro or program registers directly.
  */
uint32_t TIMER_Open(TIMER_T *timer, uint32_t u32Mode, uint32_t u32Freq)
{
    uint32_t u32Clk = TIMER_GetModuleClock(timer);
    uint32_t u32Cmpr = 0, u32Prescale = 0;

    /* Fastest possible timer working freq is (u32Clk / 2). While cmpr = 2, pre-scale = 0. */
    if(u32Freq >= (u32Clk >> 1))
    {
        u32Cmpr = 2;
    }
    else
    {
        u32Cmpr = u32Clk / u32Freq;
        u32Prescale = (u32Cmpr >> 24);  /* for 24 bits CMPDAT */
        if (u32Prescale > 0)
            u32Cmpr = u32Cmpr / (u32Prescale + 1);
    }

    timer->CTL = u32Mode | u32Prescale;
    timer->CMP = u32Cmpr;

    return(u32Clk / (u32Cmpr * (u32Prescale + 1)));
}

/**
  * @brief      Stop Timer Counting
  *
  * @param[in]  timer   The pointer of the specified Timer module.
  *
  * @return     None
  *
  * @details    This API stops timer counting and disable all timer interrupt function.
  */
void TIMER_Close(TIMER_T *timer)
{
    timer->CTL = 0;
    timer->EXTCTL = 0;
}

/**
  * @brief      Create a specify Delay Time
  *
  * @param[in]  timer       The pointer of the specified Timer module.
  * @param[in]  u32Usec     Delay period in micro seconds. Valid values are between 100~1000000 (100 micro second ~ 1 second).
  *
  * @return     Delay success or not
  * @retval     0 Success, target delay time reached
  * @retval     TIMER_TIMEOUT_ERR Delay function execute failed due to timer stop working
  *
  * @details    This API is used to create a delay loop for u32Usec micro seconds by using timer one-shot mode.
  * @note       This API overwrites the register setting of the timer used to count the delay time.
  * @note       This API use polling mode. So there is no need to enable interrupt for the timer module used to generate delay.
  */
int32_t TIMER_Delay(TIMER_T *timer, uint32_t u32Usec)
{
    uint32_t u32Clk = TIMER_GetModuleClock(timer);
    uint32_t u32Prescale = 0UL, u32Delay;
    uint32_t u32Cmpr, u32Cntr, u32NsecPerTick, i = 0UL;

    /* Clear current timer configuration */
    timer->CTL = 0;
    timer->EXTCTL = 0;

    if(u32Clk <= 1000000)   /* min delay is 1000 us if timer clock source is <= 1 MHz */
    {
        if(u32Usec < 1000)
            u32Usec = 1000;
        if(u32Usec > 1000000)
            u32Usec = 1000000;
    }
    else
    {
        if(u32Usec < 100)
            u32Usec = 100;
        if(u32Usec > 1000000)
            u32Usec = 1000000;
    }

    if(u32Clk <= 1000000)
    {
        u32Prescale = 0;
        u32NsecPerTick = 1000000000 / u32Clk;
        u32Cmpr = (u32Usec * 1000) / u32NsecPerTick;
    }
    else
    {
        u32Cmpr = u32Usec * (u32Clk / 1000000);
        u32Prescale = (u32Cmpr >> 24);  /* for 24 bits CMPDAT */
        if (u32Prescale > 0)
            u32Cmpr = u32Cmpr / (u32Prescale + 1);
    }

    timer->CMP = u32Cmpr;
    timer->CTL = TIMER_CTL_CNTEN_Msk | TIMER_ONESHOT_MODE | u32Prescale;

    /* When system clock is faster than timer clock, it is possible timer active bit cannot set
       in time while we check it. And the while loop below return immediately, so put a tiny
       delay larger than 1 ECLK here allowing timer start counting and raise active flag. */
    for(u32Delay = (SystemCoreClock / u32Clk) + 1UL; u32Delay > 0UL; u32Delay--)
    {
        __NOP();
    }

    /* Add a bail out counter here in case timer clock source is disabled accidentally.
       Prescale counter reset every ECLK * (prescale value + 1).
       The u32Delay here is to make sure timer counter value changed when prescale counter reset */
    u32Delay = (SystemCoreClock / TIMER_GetModuleClock(timer)) * (u32Prescale + 1);
    u32Cntr = timer->CNT;
    i = 0;
    while(timer->CTL & TIMER_CTL_ACTSTS_Msk)
    {
        /* Bailed out if timer stop counting e.g. Some interrupt handler close timer clock source. */
        if(u32Cntr == timer->CNT)
        {
            if(i++ > u32Delay)
            {
                return TIMER_TIMEOUT_ERR;
            }
        }
        else
        {
            i = 0;
            u32Cntr = timer->CNT;
        }
    }
    return 0;
}

/**
  * @brief      Enable Timer Capture Function
  *
  * @param[in]  timer       The pointer of the specified Timer module.
  * @param[in]  u32CapMode  Timer capture mode. Could be
  *                         - \ref TIMER_CAPTURE_FREE_COUNTING_MODE
  *                         - \ref TIMER_CAPTURE_COUNTER_RESET_MODE
  * @param[in]  u32Edge     Timer capture event trigger edge. Possible values are
  *                         - \ref TIMER_CAPTURE_FALLING_EDGE
  *                         - \ref TIMER_CAPTURE_RISING_EDGE
  *                         - \ref TIMER_CAPTURE_FALLING_AND_RISING_EDGE
  *
  * @return     None
  *
  * @details    This API is used to enable timer capture function with specify capture trigger edge \n
  *             to get current counter value or reset counter value to 0.
  * @note       Timer frequency should be configured separately by using \ref TIMER_Open API, or program registers directly.
  */
void TIMER_EnableCapture(TIMER_T *timer, uint32_t u32CapMode, uint32_t u32Edge)
{
    timer->EXTCTL = (timer->EXTCTL & ~(TIMER_EXTCTL_CAPFUNCS_Msk | TIMER_EXTCTL_CAPEDGE_Msk)) |
                    u32CapMode | u32Edge | TIMER_EXTCTL_CAPEN_Msk;
}

/**
  * @brief      Disable Timer Capture Function
  *
  * @param[in]  timer   The pointer of the specified Timer module.
  *
  * @return     None
  *
  * @details    This API is used to disable the timer capture function.
  */
void TIMER_DisableCapture(TIMER_T *timer)
{
    timer->EXTCTL &= ~TIMER_EXTCTL_CAPEN_Msk;
}

/**
  * @brief      Enable Timer Counter Function
  *
  * @param[in]  timer       The pointer of the specified Timer module.
  * @param[in]  u32Edge     Detection edge of counter pin. Could be ether
  *                         - \ref TIMER_COUNTER_FALLING_EDGE, or
  *                         - \ref TIMER_COUNTER_RISING_EDGE
  *
  * @return     None
  *
  * @details    This function is used to enable the timer counter function with specify detection edge.
  * @note       Timer compare value should be configured separately by using \ref TIMER_SET_CMP_VALUE macro or program registers directly.
  * @note       While using event counter function, \ref TIMER_TOGGLE_MODE cannot set as timer operation mode.
  */
void TIMER_EnableEventCounter(TIMER_T *timer, uint32_t u32Edge)
{
    timer->EXTCTL = (timer->EXTCTL & ~TIMER_EXTCTL_CNTPHASE_Msk) | u32Edge;
    timer->CTL |= TIMER_CTL_EXTCNTEN_Msk;
}

/**
  * @brief      Disable Timer Counter Function
  *
  * @param[in]  timer   The pointer of the specified Timer module.
  *
  * @return     None
  *
  * @details    This API is used to disable the timer event counter function.
  */
void TIMER_DisableEventCounter(TIMER_T *timer)
{
    timer->CTL &= ~TIMER_CTL_EXTCNTEN_Msk;
}

/**
  * @brief      Get Timer Clock Frequency
  *
  * @param[in]  timer   The pointer of the specified Timer module.
  *
  * @return     Timer clock frequency
  *
  * @details    This API is used to get the timer clock frequency.
  * @note       This API cannot return correct clock rate if timer source is from external clock input.
  */
uint32_t TIMER_GetModuleClock(TIMER_T *timer)
{
    uint32_t u32Src;
    const uint32_t au32Clk[] = {__HXT, __LXT, 0, 0, 0, __LIRC, 0, __HIRC};

    if(timer == TIMER0)
        u32Src = (CLK->CLKSEL1 & CLK_CLKSEL1_TMR0SEL_Msk) >> CLK_CLKSEL1_TMR0SEL_Pos;
    else if(timer == TIMER1)
        u32Src = (CLK->CLKSEL1 & CLK_CLKSEL1_TMR1SEL_Msk) >> CLK_CLKSEL1_TMR1SEL_Pos;
    else if(timer == TIMER2)
        u32Src = (CLK->CLKSEL1 & CLK_CLKSEL1_TMR2SEL_Msk) >> CLK_CLKSEL1_TMR2SEL_Pos;
    else  /* Timer 3 */
        u32Src = (CLK->CLKSEL1 & CLK_CLKSEL1_TMR3SEL_Msk) >> CLK_CLKSEL1_TMR3SEL_Pos;

    if(u32Src == 2)
    {
        if ((timer == TIMER0) || (timer == TIMER1))
            return CLK_GetPCLK0Freq();
        else
            return CLK_GetPCLK1Freq();
    }

    return (au32Clk[u32Src]);
}

/**
  * @brief      Enable the Timer Frequency Counter Function
  *
  * @param[in]  timer           The pointer of the specified Timer module.
  * @param[in]  u32DropCount    This parameter has no effect in M031 series BSP.
  * @param[in]  u32Timeout      This parameter has no effect in M031 series BSP.
  * @param[in]  u32EnableInt    Enable interrupt assertion after capture complete or not. Valid values are TRUE and FALSE.
  *
  * @return     None
  *
  * @details    This function is used to calculate input event frequency. After enable
  *             this function, a pair of timers, such as TIMER0 and TIMER1,
  *             will be configured for this function. The mode used to calculate input
  *             event frequency is mentioned as "Inter Timer Trigger Mode" in Technical
  *             Reference Manual
  */
void TIMER_EnableFreqCounter(TIMER_T *timer, uint32_t u32DropCount, uint32_t u32Timeout, uint32_t u32EnableInt)
{
    TIMER_T *t;    /* store the timer base to configure compare value */

    t = (timer == TIMER0) ? TIMER1 : TIMER3;

    t->CMP = 0xFFFFFF;
    t->EXTCTL = u32EnableInt ? TIMER_EXTCTL_CAPIEN_Msk : 0;
    timer->CTL = TIMER_CTL_INTRGEN_Msk | TIMER_CTL_CNTEN_Msk;

    return;
}

/**
  * @brief      Disable the Timer Frequency Counter Function
  *
  * @param[in]  timer       The pointer of the specified Timer module.
  *
  * @return     None
  */
void TIMER_DisableFreqCounter(TIMER_T *timer)
{
    timer->CTL &= ~TIMER_CTL_INTRGEN_Msk;
}

/**
  * @brief      Select Other Modules Triggered Source
  *
  * @param[in]  timer       The pointer of the specified Timer module.
  * @param[in]  u32Src      Selects the interrupt source to trigger other modules. Could be:
  *                         - \ref TIMER_TRGSRC_TIMEOUT_EVENT
  *                         - \ref TIMER_TRGSRC_CAPTURE_EVENT
  *
  * @return     None
  */
void TIMER_SetTriggerSource(TIMER_T *timer, uint32_t u32Src)
{
    timer->CTL = (timer->CTL & ~TIMER_CTL_TRGSSEL_Msk) | u32Src;
}

/**
  * @brief      Set Modules Trigger by Timer Interrupt Event
  *
  * @param[in]  timer       The pointer of the specified Timer module.
  * @param[in]  u32Mask     The mask of modules (PWM, ADC, BPWM and PDMA) trigger by timer. Is the combination of
  *                         - \ref TIMER_TRG_TO_PWM
  *                         - \ref TIMER_TRG_TO_ADC
  *                         - \ref TIMER_TRG_TO_PDMA
  *                         - \ref TIMER_TRG_TO_BPWM
  *
  * @return     None
  */
void TIMER_SetTriggerTarget(TIMER_T *timer, uint32_t u32Mask)
{
    timer->CTL = (timer->CTL & ~(TIMER_CTL_TRGPWM_Msk | TIMER_CTL_TRGADC_Msk | TIMER_CTL_TRGPDMA_Msk | TIMER_CTL_TRGBPWM_Msk)) | (u32Mask);
}

/**
  * @brief      Select Timer Capture Source
  *
  * @param[in]  timer       The pointer of the specified Timer module.
  * @param[in]  u32Src      Timer capture source. Possible values are
  *                         - \ref TIMER_CAPTURE_FROM_EXTERNAL
  *                         - \ref TIMER_CAPTURE_FROM_INTERNAL
  *                         - \ref TIMER_CAPTURE_FROM_ACMP0
  *                         - \ref TIMER_CAPTURE_FROM_ACMP1
  *                         - \ref TIMER_CAPTURE_FROM_LIRC
  *
  * @return     None
  *
  * @details    This API is used to select timer capture source from Tx_EXT or internal signal.
  */
void TIMER_CaptureSelect(TIMER_T *timer, uint32_t u32Src)
{
    if (u32Src == TIMER_CAPTURE_FROM_EXTERNAL)
    {
        timer->CTL = (timer->CTL & ~(TIMER_CTL_CAPSRC_Msk)) |
                     (TIMER_CAPSRC_TX_EXT);
    }
    else
    {
        timer->CTL = (timer->CTL & ~(TIMER_CTL_CAPSRC_Msk)) |
                     (TIMER_CAPSRC_INTERNAL);
        timer->EXTCTL = (timer->EXTCTL & ~(TIMER_EXTCTL_INTERCAPSEL_Msk)) |
                        (u32Src);
    }
}

/**
  * @brief      Reset Counter
  *
  * @param[in]  timer The base address of Timer module
  *
  * @return     Reset success or not
  * @retval     0 Timer reset success
  * @retval     TIMER_TIMEOUT_ERR Timer reset failed
  *
  * @details    This function is used to reset current counter value and internal prescale counter value.
  */
int32_t TIMER_ResetCounter(TIMER_T *timer)
{
    uint32_t u32Delay;

    timer->CTL |= TIMER_CTL_RSTCNT_Msk;
    /* Takes 2~3 ECLKs to reset timer counter */
    u32Delay = (SystemCoreClock / TIMER_GetModuleClock(timer)) * 3;
    while(((timer->CTL & TIMER_CTL_RSTCNT_Msk) == TIMER_CTL_RSTCNT_Msk) && (--u32Delay))
    {
        __NOP();
    }
    return u32Delay > 0 ? 0 : TIMER_TIMEOUT_ERR;
}

/*@}*/ /* end of group TIMER_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group TIMER_Driver */

/*@}*/ /* end of group Standard_Driver */

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

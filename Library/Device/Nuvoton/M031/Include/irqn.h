/**************************************************************************//**
 * @file     irqn.h
 * @version  V1.00
 * @brief    IRQ number definition for M031Series
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __IRQN_H__
#define __IRQN_H__


/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */

/**
 * @details  Interrupt Number Definition. The maximum of 32 Specific Interrupts are possible.
 */
typedef enum IRQn
{
    /******  Cortex-M0 Processor Exceptions Numbers ***************************************************/
    NonMaskableInt_IRQn       = -14,      /*!< 2 Non Maskable Interrupt                             */
    HardFault_IRQn            = -13,      /*!< 3 Cortex-M0 Hard Fault Interrupt                     */
    SVCall_IRQn               = -5,       /*!< 11 Cortex-M0 SV Call Interrupt                       */
    PendSV_IRQn               = -2,       /*!< 14 Cortex-M0 Pend SV Interrupt                       */
    SysTick_IRQn              = -1,       /*!< 15 Cortex-M0 System Tick Interrupt                   */

    /******  ARMIKMCU Swift specific Interrupt Numbers ************************************************/
    BOD_IRQn                  = 0,        /*!< Brown-Out Low Voltage Detected Interrupt             */
    WDT_IRQn                  = 1,        /*!< Watch Dog Timer Interrupt                            */
    EINT024_IRQn              = 2,        /*!< EINT0, EINT2 and EINT4 Interrupt                     */
    EINT135_IRQn              = 3,        /*!< EINT1, EINT3 and EINT5 Interrupt                     */
    GPIO_PAPB_IRQn            = 4,        /*!< GPIO_PAPBPGPH Interrupt                              */
    GPIO_PAPBPGPH_IRQn        = 4,        /*!< GPIO_PAPBPGPH Interrupt                              */
    GPIO_PCPDPEPF_IRQn        = 5,        /*!< GPIO_PCPDPEPF Interrupt                              */
    PWM0_IRQn                 = 6,        /*!< PWM0 Interrupt                                       */
    PWM1_IRQn                 = 7,        /*!< PWM1 Interrupt                                       */
    TMR0_IRQn                 = 8,        /*!< TIMER0 Interrupt                                     */
    TMR1_IRQn                 = 9,        /*!< TIMER1 Interrupt                                     */
    TMR2_IRQn                 = 10,       /*!< TIMER2 Interrupt                                     */
    TMR3_IRQn                 = 11,       /*!< TIMER3 Interrupt                                     */
    UART02_IRQn               = 12,       /*!< UART0 and UART2 Interrupt                            */
    UART1_IRQn                = 13,       /*!< UART1 and UART3 Interrupt                            */
    UART13_IRQn               = 13,       /*!< UART1 and UART3 Interrupt                            */
    SPI0_IRQn                 = 14,       /*!< SPI0 Interrupt                                       */
    QSPI0_IRQn                = 15,       /*!< QSPI0 Interrupt                                      */
    ISP_IRQn                  = 16,       /*!< ISP Interrupt	                                    */
    UART57_IRQn               = 17,       /*!< UART5 and UART7 Interrupt                            */
    I2C0_IRQn                 = 18,       /*!< I2C0 Interrupt                                       */
    I2C1_IRQn                 = 19,       /*!< I2C1 Interrupt                                       */
    BPWM0_IRQn                = 20,       /*!< BPWM0 Interrupt                                      */
    BPWM1_IRQn                = 21,       /*!< BPWM1 Interrupt                                      */
    USCI_IRQn                 = 22,       /*!< USCI0 and USCI1 interrupt                            */
    USCI01_IRQn               = 22,       /*!< USCI0 and USCI1 interrupt                            */
    USBD_IRQn                 = 23,       /*!< USB Device Interrupt                                 */
    ACMP01_IRQn               = 25,       /*!< ACMP0/1 Interrupt                                    */
    PDMA_IRQn                 = 26,       /*!< PDMA Interrupt                                       */
    UART46_IRQn               = 27,       /*!< UART4 and UART6 Interrupt                            */
    PWRWU_IRQn                = 28,       /*!< Power Down Wake Up Interrupt                         */
    ADC_IRQn                  = 29,       /*!< ADC Interrupt                                        */
    CKFAIL_IRQn               = 30,       /*!< Clock fail detect Interrupt                          */
    RTC_IRQn                  = 31       /*!< RTC Interrupt                                        */
} IRQn_Type;


#endif /* __IRQN_H__ */
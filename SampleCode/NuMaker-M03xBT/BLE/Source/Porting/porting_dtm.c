#include "porting_dtm.h"

void setDTM_timerInt_enable(void)
{
    TIMER_EnableInt(TIMER0);
    TIMER_EnableInt(TIMER2);
}

//-----------------------------------------------------------------------------//
//@brief Function for configuring the timer for 260us and N*625us cycle time.
//called by BleDTM_init()
//-----------------------------------------------------------------------------//
void setDTM_timer_init(void)
{
    TIMER_DisableInt(TIMER0);  //enable timer0 interrupt
    TIMER_DisableInt(TIMER2);  //enable timer2 interrupt
    TIMER_Stop(TIMER0);        //stop timer0 counting
    TIMER_Stop(TIMER2);        //stop timer2 counting

    /* Open Timer0 in periodic mode, enable interrupt and 400 interrupt ticks per second */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 400);  //2500us per interrupt
    TIMER_EnableInt(TIMER0);  //enable timer0 interrupt

    /* Open Timer2 in periodic mode, enable interrupt and 3846 interrupt ticks per second */
    TIMER_Open(TIMER2, TIMER_PERIODIC_MODE, 3846);  //3846Hz <-> 260us per interrupt
    TIMER_EnableInt(TIMER2);  //enable timer2 interrupt

    /* Enable Timer0 ~ Timer1 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);
    NVIC_EnableIRQ(TMR2_IRQn);

    TIMER_Start(TIMER0);      //start timer0 counting
    TIMER_Start(TIMER2);      //start timer2 counting
}



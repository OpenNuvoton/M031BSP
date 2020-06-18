/**************************************************************************//**
 * @file     SW_RTC.c
 * @version  V1.00
 * @brief    Use software to simulate RTC.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ****************************************************************************/
#include "SW_RTC.h"

/* The month table used in day of week calculation */
const uint8_t monthsTable [12] = {0,3,3,6,1,4,6,2,5,0,3,5};

/* The number of days in each month */
const uint8_t DaysInMonth [12]= {31,28,31,30,31,30,31,31,30,31,30,31};

/* DATETIME structure which contains the time and date */
DATETIME rtc;

void RTC_Process(void)
{
    rtc.time.seconds++;
    /* If seconds >= 60, it will carry on minute. */
    if ( rtc.time.seconds >= 60 )
    {
        rtc.time.seconds = 0;
        rtc.time.minutes++;
        /* If minutes >= 60, it will carry on hour. */
        if ( rtc.time.minutes >= 60 )
        {
            rtc.time.minutes = 0;
            rtc.time.hour++;
            /* If hour >= 60, it will carry on day. */
            if ( rtc.time.hour >= 24 )
            {
                rtc.time.hour = 0;
                rtc.date.day++;
                /* Update the day of the week. */
                rtc.date.dayofweek++;
                if(rtc.date.dayofweek >= 7)
                    rtc.date.dayofweek %= 7;
                /* If hour >= 60, it will carry on day. */
                if ( rtc.date.day > DaysInMonth [rtc.date.month - 1]  )
                {
                    /* Check whether it is a February or not */
                    if ( rtc.date.month == February )
                    {
                        /* Check whether it is a leap year or not */
                        if ( RTC_CheckLeapYear(rtc.date.year) == LeapYear )
                        {
                            /* In leap year Faburary, there is 29 days(DaysInMonth[rtc.date.month - 1] + 1) */
                            if ( rtc.date.day > DaysInMonth[rtc.date.month - 1] + 1)
                            {
                                /* If day >= days of month, it will carry on month. */
                                rtc.date.month++;
                                rtc.date.day = 1;
                            }
                        }
                        else
                        {
                            /* leap year other months, it will carry on month. */
                            rtc.date.month++;
                            rtc.date.day = 1;
                        }
                    }
                    else
                    {
                        /* Not Leap Year */
                        /* If day >= days of month, it will carry on month. */
                        rtc.date.month++;
                        rtc.date.day = 1;
                        if ( rtc.date.month > 12 )
                        {
                            /* If month >= 12, it will carry on Year. */
                            rtc.date.year++;
                            rtc.date.month=1;
                        }
                    }
                }
            }
        }
    }
}

void UpdateDayOfWeek (void)
{
    /* Update the day Of week by status of rtc */
    /* This algorithm starts from 2000/01/01 */

    uint32_t temp = 0;
    /* 2000/01/01 is Saturday */
    temp += 6;
    /* Add the passing days from 2000 */
    temp += (rtc.date.year-2000);
    /* Add the day shift of month */
    temp += monthsTable[rtc.date.month-1];
    /* Add the day shift of day */
    temp += rtc.date.day;

    /* Add the leap days. There is 3 condition of leap year. */
    /* (1) Per 4 years is leap year */
    /* (2) Per 100 years is not leap year */
    /* (3) Per 400 years is leap year */
    /* (4) Piority (3) > (2) > (1) */

    temp += ((rtc.date.year-2000)>>2);
    temp -= ((rtc.date.year-2000)/100);
    temp += ((rtc.date.year-2000)/400);

    /* If it's not reach 2/29, do temp -= 1 or temp += 6 to minus the Leapday */
    if(RTC_CheckLeapYear(rtc.date.year) == LeapYear)
    {
        if ((rtc.date.month == January) || (rtc.date.month == February ) )
            temp += 6;
    }

    /* temp%7 for getting the day of week */
    rtc.date.dayofweek = (temp%7);
}

LeapYearStatus RTC_CheckLeapYear(uint32_t year)
{
    /* To check whether input year is leap year or not. */
    if((year % 400) == 0)
    {
        return LeapYear;
    }
    else if((year % 100) == 0)
    {
        return NotLeapYear;
    }
    else if((year % 4) == 0)
    {
        return LeapYear;
    }
    else
    {
        return NotLeapYear;
    }
}

void RTC_Init(void)
{
    /* Init SW RTC Setting */
    rtc.time.seconds = 55;
    rtc.time.minutes = 59;
    rtc.time.hour = 23;
    rtc.date.day = 28;
    rtc.date.month = 2;
    rtc.date.year = 2056;

    /* Initial day of week by RTC Setting */
    UpdateDayOfWeek();
}

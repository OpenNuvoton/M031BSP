/**************************************************************************//**
 * @file     SW_RTC.h
 * @version  V1.00
 * @brief    Use software to simulate RTC.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ****************************************************************************/
#ifndef __SW_RTC__
#define __SW_RTC__

#include <stdio.h>
#include "NuMicro.h"

//Display type Selection
#define DisplayIn24H DISABLE
#define DisplayIn12H ENABLE

//Struct for date and time
typedef struct
{
    struct
    {
        uint8_t hour;
        uint8_t minutes;
        uint8_t seconds;
    } time;
    struct
    {
        uint32_t year;
        uint8_t month;
        uint8_t day;
        uint8_t dayofweek;
    } date;
} DATETIME;

// Enumrate for month
enum
{
    January = 1,
    February,
};

// Enumrate for day of week
enum
{
    Monday = 1,
    Tuesday,
    Wednesday,
    Thursday,
    Friday,
    Saturday,
    Sunday
};

// Type define for LeapYear
typedef enum
{
    LeapYear = 0,
    NotLeapYear
} LeapYearStatus;

void RTC_Init(void);
void RTC_Process(void);
void UpdateDayOfWeek (void);
LeapYearStatus RTC_CheckLeapYear(uint32_t year);

#endif //__SW_RTC__

/**
  ****************************************************************************************
  * @file    hal_calendar.c
  * @author  BLE Driver Team
  * @brief   CALENDAR HAL module driver.
  ****************************************************************************************
  * @attention
  #####Copyright (c) 2019 GOODIX
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   * Neither the name of GOODIX nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
  ****************************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "hal.h"

/** @addtogroup HAL_DRIVER
  * @{
  */

#ifdef HAL_CALENDAR_MODULE_ENABLED
/** @addtogroup CALENDAR CALENDAR
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define CALENDAR_ABS(X, Y)                      ((X) - (Y))
#define CALENDAR_DIF(X, Y)                      (((X) >= (Y)) ? ((X) - (Y)) : ((Y) - (X)))

#define CALENDAR_FIRST_MONTH                    (1UL)
#define CALENDAR_FIRST_DATE                     (1UL)
#define CALENDAR_FIRST_WEEK                     (6UL)
#define CALENDAR_SECONDS_PER_HOUR               (3600UL)
#define CALENDAR_SECONDS_PER_DAY                (86400UL)

#define CALENDAR_CLOCK(SECOND)                  (SECOND)
#define CALENDAR_TICKS_PER_MS(SECOND)           ((CALENDAR_CLOCK(SECOND) / 1000.0f))
#define CALENDAR_TICKS_PER_DAY(SECOND)          ((CALENDAR_SECONDS_PER_DAY) * (CALENDAR_CLOCK(SECOND)))
#define CALENDAR_TICKS_PER_WRAP                 (0x100000000UL)
#define CALENDAR_SECONDS_PER_WRAP(SECOND)       ((CALENDAR_TICKS_PER_WRAP) / (CALENDAR_CLOCK(SECOND)))
#define CALENDAR_WRAPCNT_MAX                    (0x0UL)
#define CALENDAR_SECONDS_WRAPCNT_OVER(SECOND)   ((CALENDAR_WRAPCNT_MAX + 1) * (CALENDAR_SECONDS_PER_WRAP(CALENDAR_CLOCK(SECOND))))

#define CALENDAR_ALARM_DATE                     0x01U
#define CALENDAR_ALARM_TICK                     0x02U
/* Private variables ---------------------------------------------------------*/
static uint32_t s_value_tick = 0;
static volatile float s_decimal = 0.0f;
static float pre_clock_freq = 0.0f;
/* Exported variables --------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void calendar_cover_time2seconds(calendar_time_t *p_time, uint32_t *p_seconds);
static void calendar_cover_seconds2time(calendar_time_t *p_time, uint32_t seconds);
static hal_status_t calendar_wait_flag_state_until_timeout(void);
/* Exported functions --------------------------------------------------------*/

/** @defgroup CALENDAR_Exported_Functions CALENDAR Exported Functions
  * @{
  */

/** @defgroup CALENDAR_Exported_Functions_Group1 Initialization and Configuration functions
 *  @brief    Initialization and Configuration functions.
  * @{
  */

__WEAK hal_status_t hal_calendar_init(calendar_handle_t *p_calendar)
{
    hal_status_t status     = HAL_OK;
    uint32_t     wait_count = 10000;

    GLOBAL_EXCEPTION_DISABLE();
    do {
        /* Disable CALENDAR */
        //lint -e923 Cast from pointer to unsigned int is necessary
        __HAL_CALENDAR_DISABLE();
        status = calendar_wait_flag_state_until_timeout();
        if (status != HAL_OK)
        {
            break;
        }

        /* Select clock div NONE*/
        ll_calendar_set_clock_div(LL_CALENDAR_DIV_NONE);

        /* Load reload counter value into CALENDAR */
        ll_calendar_reload_counter_and_request(0);
        status = calendar_wait_flag_state_until_timeout();
        if (status != HAL_OK)
        {
            break;
        }
        do {
            if (ll_calendar_get_read_counter() == 0U)
            {
                break;
            }
        } while (--wait_count);

        if (0U == wait_count)
        {
            status = HAL_ERROR;
            break;
        }

        p_calendar->pre_count = 0U;
        s_decimal = 0.0f;

        /* Calculate seconds from 01.01.2000 00:00 to 1970.1.1 */
        p_calendar->utc = 946684800;

        /* Store previous slow clock frequency */
        pre_clock_freq = p_calendar->clock_freq;

        /* Clear wrap interrupt flag */
        __HAL_CALENDAR_CLEAR_FLAG(CALENDAR_FLAG_WRAP | CALENDAR_FLAG_ALARM | CALENDAR_FLAG_TICK);

        /* Enable wrap interrupt */
        __HAL_CALENDAR_ENABLE_IT(CALENDAR_IT_WRAP);
        __HAL_CALENDAR_DISABLE_IT(CALENDAR_IT_ALARM | CALENDAR_IT_TICK);

        /* Clear pending IRQ and eable NVIC interrupt */
        NVIC_ClearPendingIRQ(CALENDAR_IRQn);
        NVIC_EnableIRQ(CALENDAR_IRQn);

        ll_calendar_clear_wrap();
        status = calendar_wait_flag_state_until_timeout();
        if (status != HAL_OK)
        {
            break;
        }

        /* Enable CALENDAR */
        __HAL_CALENDAR_ENABLE();
        status = calendar_wait_flag_state_until_timeout();
        if (status != HAL_OK)
        {
            break;
        }
    } while (0);
    GLOBAL_EXCEPTION_ENABLE();

    return status;
}

__WEAK hal_status_t hal_calendar_deinit(calendar_handle_t *p_calendar)
{
    /* Disable CALENDAR */
    __HAL_CALENDAR_DISABLE();
    hal_status_t status = calendar_wait_flag_state_until_timeout();

    /* Disable NVIC interrupt and Clear pending IRQ */
    NVIC_DisableIRQ(CALENDAR_IRQn);
    NVIC_ClearPendingIRQ(CALENDAR_IRQn);

    p_calendar->pre_count = 0;
    s_decimal = 0.0f;
    p_calendar->utc = 0;
    pre_clock_freq = 0.0f;

    /* Disable wrap and alarm interrupt */
    __HAL_CALENDAR_DISABLE_IT(CALENDAR_IT_WRAP | CALENDAR_IT_ALARM | CALENDAR_IT_TICK);

    return status;
}

/** @} */

/** @defgroup CALENDAR_Exported_Functions_Group2 IO operation functions
 *  @brief    IO operation functions
  * @{
  */
__WEAK hal_status_t hal_calendar_init_time(calendar_handle_t *p_calendar, calendar_time_t *p_time)
{
    hal_status_t status = HAL_OK;
    uint32_t     timer_value = 0;

    if (ll_calendar_is_running() == 0U)
    {
        return HAL_ERROR;
    }

    GLOBAL_EXCEPTION_DISABLE();
    do {
        /* Read counter value */
        timer_value = ll_calendar_get_read_counter();

        /* Record the value of the current register */
        p_calendar->pre_count = timer_value;

        /* Update system UTC time */
        calendar_cover_time2seconds(p_time, &p_calendar->utc);

        /* Update system ms time */
        s_decimal =(float)((float)p_time->ms * CALENDAR_TICKS_PER_MS(p_calendar->clock_freq));
    } while (0);

    /* Sync Alarm */
    do {
        calendar_time_t s_time = { 0 };
        uint32_t dif_sec = 0, alarm_sec = 0, alarm_value = 0;

        if (HAL_OK != status)
        {
            break;
        }

        if ((p_calendar->mode & CALENDAR_ALARM_DATE) != 0U)
        {
            //lint -e934 Taking address of near auto variable is necessary
            calendar_cover_seconds2time(&s_time, p_calendar->utc);
            s_time.hour = p_calendar->alarm.hour;
            s_time.min = p_calendar->alarm.min;
            s_time.sec = 0;
            calendar_cover_time2seconds(&s_time, &alarm_sec);
            dif_sec = CALENDAR_DIF(alarm_sec, p_calendar->utc);
            /* Asynchronous ALARM configuration within 10S */
            if (dif_sec < 10U)
            {
                break;
            }

            if (alarm_sec < p_calendar->utc)
            {
                alarm_sec += CALENDAR_SECONDS_PER_DAY;
            }

            /* Calculate alarm value */
            //lint -e9029 -e9033
            alarm_value = timer_value + (uint32_t)(((alarm_sec - p_calendar->utc) * CALENDAR_CLOCK(p_calendar->clock_freq)) - s_decimal + 0.5f);

            /* Load alarm value into CALENDAR */
            ll_calendar_reload_alarm_and_request(alarm_value);
            status = calendar_wait_flag_state_until_timeout();

            uint32_t wait_count = 10000;
            while (ll_calendar_get_read_alarm() != alarm_value)
            {
                if ((--wait_count) == 0U)
                {
                    break;
                }
            }
        }
    } while (0);
    GLOBAL_EXCEPTION_ENABLE();

    return status;
}

__WEAK hal_status_t hal_calendar_get_time(calendar_handle_t *p_calendar, calendar_time_t *p_time)
{
    hal_status_t status = HAL_OK;
    uint32_t timer_value = 0;
    uint32_t run_sec = 0, run_count = 0;
    float rev_count = 0.0f;

    if (ll_calendar_is_running() == 0U)
    {
        return HAL_ERROR;
    }

    GLOBAL_EXCEPTION_DISABLE();
    do {
        /* Read counter value */
        timer_value = ll_calendar_get_read_counter();

        if (timer_value != p_calendar->pre_count)
        {
            run_count = CALENDAR_ABS(timer_value, p_calendar->pre_count);
        }
        else
        {
            run_count = 0;
        }
        run_sec = (uint32_t)(run_count / CALENDAR_CLOCK(p_calendar->clock_freq));
        rev_count = fmodf((float)run_count, CALENDAR_CLOCK(p_calendar->clock_freq));

        /*
        * The formula :
        * utc = last utc + (RTC current count value - last count value) / latest frequency
        */
        p_calendar->pre_count = timer_value;
        p_calendar->utc += run_sec;
        s_decimal += rev_count;

        if (s_decimal > CALENDAR_CLOCK(p_calendar->clock_freq))
        {
            run_sec = (uint32_t)(s_decimal / CALENDAR_CLOCK(p_calendar->clock_freq));
            p_calendar->utc += run_sec;
            s_decimal = fmodf(s_decimal, CALENDAR_CLOCK(p_calendar->clock_freq));
        }

        /* Convert system UTC time */
        calendar_cover_seconds2time(p_time, p_calendar->utc);

        /* Convert system ms time */
        p_time->ms = (uint16_t)((s_decimal / CALENDAR_CLOCK(p_calendar->clock_freq) * 1000U) + 0.5f);
        if (p_time->ms >= 1000U)
        {
            p_time->sec += 1U;
            p_time->ms = 0;
        }
    } while (0);
    GLOBAL_EXCEPTION_ENABLE();

    return status;
}

__WEAK hal_status_t hal_calendar_set_alarm(calendar_handle_t *p_calendar, calendar_alarm_t *p_alarm)
{
    hal_status_t status = HAL_OK;
    calendar_time_t curr_time;
    uint32_t curr_sec, alarm_sec;
    uint32_t curr_value, alarm_value;

    if (ll_calendar_is_running() == 0)
    {
        return HAL_ERROR;
    }

    GLOBAL_EXCEPTION_DISABLE();

    status = hal_calendar_get_time(p_calendar, &curr_time);
    calendar_cover_time2seconds(&curr_time, &curr_sec);

    do {
        curr_time.hour = p_alarm->hour;
        curr_time.min = p_alarm->min;
        curr_time.sec = 0;
        calendar_cover_time2seconds(&curr_time, &alarm_sec);

        /* If alarm time is befor the present time, or within 3s after. */
        if (!((alarm_sec > curr_sec) && ((alarm_sec - curr_sec) > 3U)))
        {
            alarm_sec += CALENDAR_SECONDS_PER_DAY;
        }

        p_calendar->mode |= CALENDAR_ALARM_DATE;
        memcpy(&p_calendar->alarm, p_alarm, sizeof(calendar_alarm_t));

        /* Calculate alarm value */
        curr_value = ll_calendar_get_read_counter();
        alarm_value = curr_value + (uint32_t)(((alarm_sec - curr_sec) * CALENDAR_CLOCK(p_calendar->clock_freq)) - s_decimal + 0.5f);

        /* Load alarm value into CALENDAR */
        ll_calendar_reload_alarm_and_request(alarm_value);
        status = calendar_wait_flag_state_until_timeout();

        uint32_t wait_count = 10000;
        while (ll_calendar_get_read_alarm() != alarm_value)
        {
            if ((--wait_count) == 0U)
            {
                break;
            }
        }
        if (status == HAL_OK)
        {
            ll_calendar_enable_alarm();
            status = calendar_wait_flag_state_until_timeout();
        }

        /* Clear alarm interrupt flag */
        __HAL_CALENDAR_CLEAR_FLAG(CALENDAR_FLAG_ALARM);
        /* Enable alarm interrupt */
        __HAL_CALENDAR_ENABLE_IT(CALENDAR_IT_ALARM);
    } while (0);

    GLOBAL_EXCEPTION_ENABLE();

    return status;
}

__WEAK hal_status_t hal_calendar_sync_time(calendar_handle_t *p_calendar, float slow_clock_freq)
{
    hal_status_t status = HAL_OK;
    uint32_t timer_value = 0;
    uint32_t run_sec = 0, run_count = 0;
    float rev_count = 0.0f;

    if (ll_calendar_is_running() == 0)
    {
        return HAL_ERROR;
    }

    /* No need to sync slow clock frequency if same */
    if (fabs((double)slow_clock_freq - (double)pre_clock_freq) < 0.5)
    {
        return HAL_OK;
    }
    else
    {
        pre_clock_freq = slow_clock_freq;
    }

    GLOBAL_EXCEPTION_DISABLE();
    /* Sync Time */
    do {
        /* Read counter value */
        timer_value = ll_calendar_get_read_counter();

        p_calendar->clock_freq = slow_clock_freq;

        if (timer_value != p_calendar->pre_count)
        {
            run_count = CALENDAR_ABS(timer_value, p_calendar->pre_count);
        }
        else
        {
            run_count = 0;
        }
        run_sec = (uint32_t)(run_count / CALENDAR_CLOCK(p_calendar->clock_freq));
        rev_count = fmodf((float)run_count, CALENDAR_CLOCK(p_calendar->clock_freq));

        p_calendar->pre_count = timer_value;
        p_calendar->utc += run_sec;
        s_decimal += rev_count;

        if (s_decimal > CALENDAR_CLOCK(p_calendar->clock_freq))
        {
            run_sec = (uint32_t)(s_decimal / CALENDAR_CLOCK(p_calendar->clock_freq));
            p_calendar->utc += run_sec;
            s_decimal = fmodf(s_decimal, CALENDAR_CLOCK(p_calendar->clock_freq));
        }
    } while (0);

    /* Sync Alarm */
    do {
        calendar_time_t s_time = { 0 };
        uint32_t dif_sec = 0, alarm_sec = 0, alarm_value = 0;

        if ((p_calendar->mode & CALENDAR_ALARM_DATE) != 0U)
        {
            calendar_cover_seconds2time(&s_time, p_calendar->utc);
            s_time.hour = p_calendar->alarm.hour;
            s_time.min = p_calendar->alarm.min;
            s_time.sec = 0;
            calendar_cover_time2seconds(&s_time, &alarm_sec);
            dif_sec = CALENDAR_DIF(alarm_sec, p_calendar->utc);
            /* Asynchronous ALARM configuration within 10S */
            if (dif_sec < 10U)
            {
                break;
            }

            if (alarm_sec < p_calendar->utc)
            {
                alarm_sec += CALENDAR_SECONDS_PER_DAY;
            }

            /* Calculate alarm value */
            alarm_value = timer_value + (uint32_t)(((alarm_sec - p_calendar->utc) * CALENDAR_CLOCK(p_calendar->clock_freq)) - s_decimal + 0.5f);

            /* Load alarm value into CALENDAR */
            ll_calendar_reload_alarm_and_request(alarm_value);
            status = calendar_wait_flag_state_until_timeout();
            if (HAL_OK != status)
            {
                break;
            }

            uint32_t wait_count = 10000;
            while ((ll_calendar_get_read_alarm() != alarm_value))
            {
                if ((--wait_count) == 0U)
                {
                    break;
                }
            }
        }
    } while (0);
    GLOBAL_EXCEPTION_ENABLE();

    return status;
}

__WEAK hal_status_t hal_calendar_set_tick(calendar_handle_t *p_calendar, uint32_t interval)
{
    hal_status_t status = HAL_OK;

    if (ll_calendar_is_running() == 0U)
    {
        return HAL_ERROR;
    }

    do {
        p_calendar->interval = interval;
        p_calendar->mode |= CALENDAR_ALARM_TICK;

        GLOBAL_EXCEPTION_DISABLE();

        /* Calculate alarm value */
        /* BALBOA2-166: Tick value reload need one clock */
        s_value_tick = (uint32_t)((p_calendar->interval * CALENDAR_TICKS_PER_MS(p_calendar->clock_freq)) - 0.5f);

        /* Load tick0 value into CALENDAR */
        ll_calendar_reload_tick_and_request(LL_CLDR_TIMER_TICK, s_value_tick);
        status = calendar_wait_flag_state_until_timeout();

        if (status == HAL_OK)
        {
            ll_calendar_enable_tick(LL_CLDR_TIMER_TICK, LL_CLDR_TIMER_TICK_TYPE_AUTO);
            status = calendar_wait_flag_state_until_timeout();
        }

        /* Clear tick 0 interrupt flag */
        __HAL_CALENDAR_CLEAR_FLAG(CALENDAR_FLAG_TICK);
        /* Enable tick 0 interrupt */
        __HAL_CALENDAR_ENABLE_IT(CALENDAR_IT_TICK);

        GLOBAL_EXCEPTION_ENABLE();
    } while (0);

    return status;
}

__WEAK hal_status_t hal_calendar_disable_event(calendar_handle_t *p_calendar, uint32_t disable_mode)
{
    hal_status_t status = HAL_OK;

    if (ll_calendar_is_running() == 0U)
    {
        return HAL_ERROR;
    }

    GLOBAL_EXCEPTION_DISABLE();
    switch (disable_mode)
    {
        case CALENDAR_ALARM_DISABLE_DATE:
            if ((p_calendar->mode & CALENDAR_ALARM_DATE) != 0U)
            {
                __HAL_CALENDAR_DISABLE_IT(CALENDAR_IT_ALARM);
                ll_calendar_disable_alarm();
                status = calendar_wait_flag_state_until_timeout();
                if (status == HAL_OK)
                {
                    p_calendar->mode &= ~CALENDAR_ALARM_DATE;
                }
            }
            else
            {
                status = HAL_ERROR;
            }
            break;
        case CALENDAR_ALARM_DISABLE_TICK:
            if ((p_calendar->mode & CALENDAR_ALARM_TICK) != 0U)
            {
                __HAL_CALENDAR_DISABLE_IT(CALENDAR_IT_TICK);
                ll_calendar_disable_tick(LL_CLDR_TIMER_TICK);
                status = calendar_wait_flag_state_until_timeout();
                if (status == HAL_OK)
                {
                    p_calendar->mode &= ~CALENDAR_ALARM_TICK;
                }
            }
            else
            {
                status = HAL_ERROR;
            }
            break;
        case CALENDAR_ALARM_DISABLE_ALL:
            if ((p_calendar->mode & (CALENDAR_ALARM_TICK | CALENDAR_ALARM_DATE)) != 0U)
            {
                __HAL_CALENDAR_DISABLE_IT(CALENDAR_IT_ALARM | CALENDAR_IT_TICK);
                ll_calendar_disable_alarm();
                status = calendar_wait_flag_state_until_timeout();
                if (status == HAL_OK)
                {
                    ll_calendar_disable_tick(LL_CLDR_TIMER_TICK);
                    status = calendar_wait_flag_state_until_timeout();
                }
                if (status == HAL_OK)
                {
                    p_calendar->mode = 0;
                }
            }
            else
            {
                status = HAL_ERROR;
            }
            break;
        default:
            status = HAL_ERROR;
            break;
    }
    GLOBAL_EXCEPTION_ENABLE();

    return status;
}

__WEAK void hal_calendar_irq_handler(calendar_handle_t *p_calendar)
{
    if (ll_calendar_it_is_enabled_alarm())
    {
        if (__HAL_CALENDAR_GET_IT_SOURCE(CALENDAR_FLAG_ALARM))
        {
        calendar_time_t curr_time;
        uint32_t alarm_value;

        __HAL_CALENDAR_CLEAR_FLAG(CALENDAR_FLAG_ALARM);
        ll_calendar_clear_it_event();

        GLOBAL_EXCEPTION_DISABLE();

        /* Set alarm after 24h */
        alarm_value = ll_calendar_get_read_alarm();
        alarm_value += (uint32_t)(CALENDAR_TICKS_PER_DAY(p_calendar->clock_freq));

        /* Load alarm value into CALENDAR */
        ll_calendar_reload_alarm_and_request(alarm_value);

        calendar_wait_flag_state_until_timeout();/*lint !e534 MISRA exception. ignore the return value */

        GLOBAL_EXCEPTION_ENABLE();

        hal_calendar_get_time(p_calendar, &curr_time);/*lint !e534 MISRA exception. ignore the return value */
        if (CALENDAR_ALARM_SEL_WEEKDAY == p_calendar->alarm.alarm_sel)
        {
            if (p_calendar->alarm.alarm_date_week_mask & (1UL << curr_time.week))
            {
                /* Alarm callback  */
                hal_calendar_alarm_callback(p_calendar);
            }
        }
        else
        {
            if (p_calendar->alarm.alarm_date_week_mask == curr_time.date)
            {
                /* Alarm callback  */
                hal_calendar_alarm_callback(p_calendar);
            }
        }
        }
    }

    if (ll_calendar_it_is_enabled_wrap())
    {
        if (__HAL_CALENDAR_GET_IT_SOURCE(CALENDAR_FLAG_WRAP))
        {
            __HAL_CALENDAR_CLEAR_FLAG(CALENDAR_FLAG_WRAP);
            ll_calendar_clear_it_event();

            /* overflow callback  */
            hal_calendar_overflow_callback(p_calendar);
        }
    }

    if (ll_calendar_it_is_enabled_tick())
    {
        if (__HAL_CALENDAR_GET_IT_SOURCE(CALENDAR_FLAG_TICK))
        {
            __HAL_CALENDAR_CLEAR_FLAG(CALENDAR_FLAG_TICK);
            ll_calendar_clear_it_event();

            /* tick callback  */
            hal_calendar_tick_callback(p_calendar);
        }
    }
}

__WEAK void hal_calendar_alarm_callback(calendar_handle_t *p_calendar)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_calendar);

    /* NOTE: This function should not be modified, when the callback is needed,
             the hal_calendar_alarm_callback could be implemented in the user file
    */
}

__WEAK void hal_calendar_tick_callback(calendar_handle_t *p_calendar)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_calendar);

    /* NOTE: This function should not be modified, when the callback is needed,
             the hal_calendar_tick_callback could be implemented in the user file
    */
}

__WEAK void hal_calendar_overflow_callback(calendar_handle_t *p_calendar)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_calendar);

    /* NOTE: This function should not be modified, when the callback is needed,
             the hal_calendar_overflow_callback could be implemented in the user file
    */
}

/** @} */

/** @} */

/* Calculate seconds from 01.01.2000 00:00 to the time */
static void calendar_cover_time2seconds(calendar_time_t *p_time, uint32_t *p_seconds)
{
    uint32_t year;
    uint32_t utc;

    // 10957 is the days between 1970/1/1 and 2000/1/1
    year = (p_time->year + 2000) % 100U;
    utc  = 10957;
    utc += ((year * 365) + ((year + 3) / 4));
    utc += (((367 * p_time->mon) - 362) / 12) - ((p_time->mon <= 2) ? 0 : (((year % 4) == 0) ? 1 : 2));
    utc += (p_time->date - 1);
    utc *= 86400;
    utc += ((p_time->hour * 3600) + (p_time->min * 60) + p_time->sec);

    *p_seconds = utc;
}

static void calendar_cover_seconds2time(calendar_time_t *p_time, uint32_t seconds)
{
    const uint16_t days0[] = { 0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335, 366 };
    const uint16_t days1[] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365 };

    calendar_time_t time;
    uint32_t days, secs;
    uint32_t year;
    const uint16_t * dayp;

    // 10957 is the days between 1970/1/1 and 2000/1/1
    if (seconds >= (10957 * 86400))
    {
        days = (seconds / 86400) - 10957;
        secs = seconds % 86400;
    }
    else
    {
        days = 0;
        secs = 0;
    }

    time.sec = (uint8_t)(secs % 60);
    secs /= 60;
    time.min = (uint8_t)(secs % 60);
    secs /= 60;
    time.hour = (uint8_t)secs;

    year = 2000;
    time.week = (uint8_t)((days + 6) % 7);

    // 1461 represents the number of days in a four-year cycle,
    // which is used to calculate the total number of days since the year 2000 and to deduce the year based on those days.
    // Whenever the expression days / 1461 reaches a certain value in the code, it indicates that a complete four-year cycle has passed.
    year += (days / 1461) * 4; days %= 1461;
    if (days >= 366)
    {
        dayp = days1;
    }
    else
    {
        dayp = days0;
    }
    if (days >= 366)
    {
        year += (days - 1) / 365;
        days = (days - 1) % 365;
    }

    time.mon = (uint8_t)((days / 31) + 1);
    if (days >= dayp[time.mon])
    {
        time.mon += 1U;
    }

    time.date = (uint8_t)(days - dayp[time.mon - 1] + 1U);
    time.year = (uint8_t)(year - 2000U);

    memcpy(p_time, &time, sizeof(calendar_time_t));
}

static hal_status_t calendar_wait_flag_state_until_timeout(void)
{
    uint32_t timeout = 500000;

    while (__HAL_CALENDAR_BUSY_FLAG())
    {
        if ((--timeout) == 0U)
        {
            break;
        }
    }

    return ((0U == timeout) ? HAL_TIMEOUT : HAL_OK);
}


#endif /* HAL_CALENDAR_MODULE_ENABLED */
/** @} */

/** @} */

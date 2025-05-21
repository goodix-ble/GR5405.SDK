/**
  ****************************************************************************************
  * @file    hal_rtc.c
  * @author  BLE Driver Team
  * @brief   RTC HAL module driver.
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
#include "hal.h"

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void hal_rtc_overflow_det(const rtc_handle_t *p_rtc);
static uint8_t is_rtc_timeout(const rtc_handle_t *p_rtc);

/**
  * @brief  init rtc.
  * @param  rtc_handle_t *p_rtc.
  * @attention count up from zero if not set start_value.
               eg:p_rtc->init.alarm_value = 0;
  */
hal_status_t hal_rtc_init(rtc_handle_t *p_rtc)
{
    hal_status_t status = HAL_OK;

    if (ll_calendar_is_running())
    {
        status = HAL_ERROR;
        goto EXIT;
    }

    do{
        /*close RTC config*/
        __HAL_RTC_DISABLE(p_rtc->p_instance);

        /*wait config go into effect*/
        if(is_rtc_timeout(p_rtc))
        {
            p_rtc->state = HAL_RTC_ERROR;
            p_rtc->error_code = HAL_RTC_ERROR_TIMEOUT;
            status = HAL_TIMEOUT;
            goto EXIT;
        }

        /*select clock div */
        ll_rtc_set_clock_div(p_rtc->p_instance, p_rtc->init.prescaler_div);

        if(p_rtc->init.start_value != 0U)
        {
            /*set rtc start value*/
            ll_rtc_start_value_set_and_request(p_rtc->p_instance, p_rtc->init.start_value);

            /*wait config go into effect*/
            if(is_rtc_timeout(p_rtc))
            {
                p_rtc->state = HAL_RTC_ERROR;
                p_rtc->error_code = HAL_RTC_ERROR_TIMEOUT;
                status = HAL_TIMEOUT;
                goto EXIT;
            }
        }

        /* clear wrap flag*/
        ll_rtc_clear_wrap_and_request(p_rtc->p_instance);

        /*wait config go into effect*/
        if(is_rtc_timeout(p_rtc))
        {
            p_rtc->state = HAL_RTC_ERROR;
            p_rtc->error_code = HAL_RTC_ERROR_TIMEOUT;
            status = HAL_TIMEOUT;
            goto EXIT;
        }

        /*clear pending IRQ and eable NVIC interrupt */
        NVIC_ClearPendingIRQ(CALENDAR_IRQn);
        NVIC_EnableIRQ(CALENDAR_IRQn);

        /* Clear wrap interrupt flag */
        __HAL_RTC_CLEAR_FLAG(p_rtc->p_instance, RTC_FLAG_WRAP | RTC_FLAG_ALARM | RTC_FLAG_TICK);

        /* Disable wrap interrupt */
        __HAL_RTC_DISABLE_IT(p_rtc->p_instance, RTC_IT_ALARM | RTC_IT_TICK | RTC_IT_WRAP);

        /*set overflow_det or not*/
        hal_rtc_overflow_det(p_rtc);
        p_rtc->state = HAL_RTC_READY;

        /* Enable RTC */
        __HAL_RTC_ENABLE(p_rtc->p_instance);

        if(is_rtc_timeout(p_rtc))
        {
            p_rtc->state = HAL_RTC_ERROR;
            p_rtc->error_code = HAL_RTC_ERROR_TIMEOUT;

            status = HAL_TIMEOUT;
            goto EXIT;
        }
    }while(0);

    p_rtc->state = HAL_RTC_RUNNING;
    p_rtc->tick.state = TICK_READY;
    p_rtc->alarm.state = ALARM_READY;
    p_rtc->error_code = HAL_RTC_ERROR_NONE;

    EXIT:
    return status;
}


hal_status_t hal_rtc_deinit(rtc_handle_t *p_rtc)
{
    hal_status_t ret = HAL_OK;
    
    /*close RTC config*/
    __HAL_RTC_DISABLE(p_rtc->p_instance);

    if(is_rtc_timeout(p_rtc))
    {
        p_rtc->state = HAL_RTC_ERROR;
        p_rtc->error_code = HAL_RTC_ERROR_TIMEOUT;
        ret = HAL_TIMEOUT;
    }
    else
    {
        NVIC_DisableIRQ(CALENDAR_IRQn);
        NVIC_ClearPendingIRQ(CALENDAR_IRQn);

        __HAL_RTC_DISABLE_IT(p_rtc->p_instance, RTC_IT_ALARM | RTC_IT_WRAP | RTC_IT_TICK);
        __HAL_RTC_CLEAR_FLAG(p_rtc->p_instance, RTC_FLAG_ALARM | RTC_FLAG_WRAP | RTC_FLAG_TICK);

        p_rtc->state = HAL_RTC_RESET;
        p_rtc->error_code = HAL_RTC_ERROR_NONE;
    }
    return ret;
}

hal_status_t hal_rtc_set_tick_and_start(rtc_handle_t *p_rtc, uint8_t mode,uint32_t value)
{
    hal_status_t status = HAL_OK;
    
    /* set tick mode */
    ll_rtc_set_tick_mode(p_rtc->p_instance, mode);

    /* Enable tick */
    ll_rtc_reload_tick_and_request(p_rtc->p_instance, value);

    if(is_rtc_timeout(p_rtc))
    {
        p_rtc->tick.state = TICK_ERROR;
        p_rtc->error_code = HAL_RTC_ERROR_TIMEOUT;

        status = HAL_TIMEOUT;
    }
    else
    {
        /* Enable TICK interrupt */
        __HAL_RTC_CLEAR_FLAG(p_rtc->p_instance, RTC_FLAG_TICK);
        __HAL_RTC_ENABLE_IT(p_rtc->p_instance, RTC_IT_TICK);

        p_rtc->tick.mode = mode;
        p_rtc->tick.value = value;
        p_rtc->tick.state = TICK_RUNNING;
        p_rtc->error_code = HAL_RTC_ERROR_NONE;
    }
    return status;
}


hal_status_t hal_rtc_restart_tick(rtc_handle_t *p_rtc)
{
    hal_status_t status = HAL_OK;

    /* Enable tick */
    ll_rtc_restart_tick(p_rtc->p_instance);

    if(is_rtc_timeout(p_rtc))
    {
        p_rtc->tick.state = TICK_ERROR;
        p_rtc->error_code = HAL_RTC_ERROR_TIMEOUT;
        status = HAL_TIMEOUT;
    }
    else
    {
        /* Enable TICK interrupt */
        __HAL_RTC_CLEAR_FLAG(p_rtc->p_instance, RTC_FLAG_TICK);
        __HAL_RTC_ENABLE_IT(p_rtc->p_instance, RTC_IT_TICK);

        p_rtc->tick.state = TICK_RUNNING;
        p_rtc->error_code = HAL_RTC_ERROR_NONE;
    }
    return status;
}

hal_status_t hal_rtc_stop_tick(rtc_handle_t *p_rtc)
{
    hal_status_t status = HAL_OK;

    /* Disable TICK */
    ll_rtc_disable_tick(p_rtc->p_instance);

    if(is_rtc_timeout(p_rtc))
    {
        p_rtc->tick.state = TICK_ERROR;
        p_rtc->error_code = HAL_RTC_ERROR_TIMEOUT;
        status = HAL_TIMEOUT;
    }
    else
    {
        __HAL_RTC_DISABLE_IT(p_rtc->p_instance, RTC_IT_TICK);
        p_rtc->tick.state = TICK_READY;
        p_rtc->error_code = HAL_RTC_ERROR_NONE;
    }
    return status;
}

hal_status_t hal_rtc_set_alarm(rtc_handle_t *p_rtc, uint32_t value)
{
    hal_status_t status = HAL_OK;

    /* Set alarm value */
    ll_rtc_set_alarm(p_rtc->p_instance, value);

    if(is_rtc_timeout(p_rtc))
    {
        p_rtc->alarm.state = ALARM_ERROR;
        p_rtc->error_code = HAL_RTC_ERROR_TIMEOUT;
        status = HAL_TIMEOUT;
    }
    else
    {
        /* Enable Alarm */
        __HAL_RTC_CLEAR_FLAG(p_rtc->p_instance, RTC_FLAG_ALARM);
        __HAL_RTC_ENABLE_IT(p_rtc->p_instance, RTC_IT_ALARM);

        p_rtc->alarm.value = value;
        p_rtc->alarm.state = ALARM_RUNNING;
        p_rtc->error_code = HAL_RTC_ERROR_NONE;
    }
    return status;
}

hal_status_t hal_rtc_stop_alarm(rtc_handle_t *p_rtc)
{
    hal_status_t status = HAL_OK;

    ll_rtc_disable_alarm(p_rtc->p_instance);

    if(is_rtc_timeout(p_rtc))
    {
        p_rtc->alarm.state = ALARM_ERROR;
        p_rtc->error_code = HAL_RTC_ERROR_TIMEOUT;
        status = HAL_TIMEOUT;
    }
    else
    {
        __HAL_RTC_DISABLE_IT(p_rtc->p_instance, RTC_IT_ALARM);

        p_rtc->alarm.state = ALARM_READY;
        p_rtc->error_code = HAL_RTC_ERROR_NONE;
    }
    return status;
}

hal_status_t hal_rtc_clear_wrap(rtc_handle_t* p_rtc)
{
    hal_status_t status = HAL_OK;

    ll_rtc_clear_wrap_and_request(p_rtc->p_instance);

    if(is_rtc_timeout(p_rtc))
    {
        p_rtc->state = HAL_RTC_ERROR;
        p_rtc->error_code = HAL_RTC_ERROR_TIMEOUT;
        status = HAL_TIMEOUT;
    }

    return status;
}

uint32_t hal_rtc_get_wrap_count(const rtc_handle_t *p_rtc)
{
    return ll_rtc_get_wrapcnt(p_rtc->p_instance);
}

uint32_t hal_rtc_get_cur_count(const rtc_handle_t *p_rtc)
{
    return ll_rtc_get_read_counter(p_rtc->p_instance);
}

uint32_t hal_rtc_get_cur_tick(const rtc_handle_t *p_rtc)
{
    return ll_rtc_get_read_tick(p_rtc->p_instance);
}

uint32_t hal_rtc_get_alarm_value(const rtc_handle_t *p_rtc)
{
    return ll_rtc_get_read_alarm(p_rtc->p_instance);
}

hal_rtc_state_t hal_rtc_get_state(const rtc_handle_t *p_rtc)
{
    return p_rtc->state;
}

static void hal_rtc_overflow_det(const rtc_handle_t *p_rtc)
{
    if(p_rtc->init.overflow_det_state == OPENED)
    {
        __HAL_RTC_CLEAR_FLAG(p_rtc->p_instance, RTC_FLAG_WRAP);
        __HAL_RTC_ENABLE_IT(p_rtc->p_instance,RTC_IT_WRAP);
    }
    else
    {
        __HAL_RTC_DISABLE_IT(p_rtc->p_instance,RTC_IT_WRAP);
        __HAL_RTC_CLEAR_FLAG(p_rtc->p_instance, RTC_FLAG_WRAP);
    }
}

static uint8_t is_rtc_timeout(const rtc_handle_t *p_rtc)
{
    uint8_t ret = 0x0;
    uint32_t timeout = 50000;
    while( __HAL_RTC_BUSY_FLAG(p_rtc->p_instance) )
    { 
        if (--timeout == 0U)
        {
            ret = 0x1;
            break;
        }
    }
    return ret;
}

void hal_rtc_irq_handler(rtc_handle_t *p_rtc)
{
    if ( ll_rtc_it_is_enabled_alarm(p_rtc->p_instance) )
    {
        if ( __HAL_RTC_GET_IT_SOURCE(p_rtc->p_instance, RTC_FLAG_ALARM) )
        {
            __HAL_RTC_CLEAR_FLAG(p_rtc->p_instance, RTC_FLAG_ALARM);
            ll_rtc_clear_it_event(p_rtc->p_instance);
            hal_rtc_alarm_callback(p_rtc);
        }
    }

    if ( ll_rtc_it_is_enabled_wrap(p_rtc->p_instance) )
    {
        if ( __HAL_RTC_GET_IT_SOURCE(p_rtc->p_instance, RTC_FLAG_WRAP) )
        {
            __HAL_RTC_CLEAR_FLAG(p_rtc->p_instance, RTC_FLAG_WRAP);
            ll_rtc_clear_it_event(p_rtc->p_instance);
            hal_rtc_overflow_callback(p_rtc);
        }
    }

    if ( ll_rtc_it_is_enabled_tick(p_rtc->p_instance) )
    {
        if ( __HAL_RTC_GET_IT_SOURCE(p_rtc->p_instance, RTC_FLAG_TICK) )
        {
            __HAL_RTC_CLEAR_FLAG(p_rtc->p_instance, RTC_FLAG_TICK);
            ll_rtc_clear_it_event(p_rtc->p_instance);
            hal_rtc_tick_callback(p_rtc);
        }
    }
}

__WEAK void hal_rtc_alarm_callback(rtc_handle_t *p_rtc)
{
    UNUSED(p_rtc);
}

__WEAK void hal_rtc_overflow_callback(rtc_handle_t *p_rtc)
{
    UNUSED(p_rtc);
}

__WEAK void hal_rtc_tick_callback(rtc_handle_t *p_rtc)
{
    UNUSED(p_rtc);
}


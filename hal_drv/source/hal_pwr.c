/**
  ****************************************************************************************
  * @file    hal_pwr.c
  * @author  BLE Driver Team
  * @brief   PWR HAL module driver.
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


/** @addtogroup HAL_DRIVER
  * @{
  */

#ifdef HAL_PWR_MODULE_ENABLED
typedef void (*diag_trace_func_t)(void);

#define SWDIAG_SHUTDOWN()  if(diag_shutdown_func) diag_shutdown_func()

diag_trace_func_t diag_shutdown_func = NULL;

/* Private functions ---------------------------------------------------------*/

/** @defgroup PWR_Exported_Functions PWR Exported Functions
  * @{
  */
__WEAK hal_status_t hal_pwr_get_timer_current_value(uint32_t timer_type, uint32_t *p_value)
{
    uint32_t val0    = 0;
    uint32_t val1    = 1;
    uint32_t timeout = HAL_PWR_TIMEOUT_DEFAULT_VALUE;

    /* Check the parameters */
    gr_assert_param(IS_PWR_PWR_TIMER_TYPE(timer_type));

    /* Check the value allocation */
    if (NULL == p_value)
    {
        return HAL_ERROR;
    }

    uint32_t (*get_timer_read_value_func)(void) = NULL;
    switch(timer_type)
    {
        case PWR_TIMER_TYPE_SLP_TIMER:
        {
            get_timer_read_value_func = ll_pwr_get_sleep_timer_read_value;
        }
        break;

        case PWR_TIMER_TYPE_AON_WDT:
        {
            get_timer_read_value_func = ll_aon_wdt_get_alarm_read_counter;
        }
        break;

        case PWR_TIMER_TYPE_CAL_TIMER:
        {
            get_timer_read_value_func = ll_calendar_get_read_counter;
        }
        break;

        case PWR_TIMER_TYPE_CAL_ALARM:
        {
            get_timer_read_value_func = ll_calendar_get_read_alarm;
        }
        break;
        case PWR_TIMER_TYPE_AON_WDT_TIMER:
        {
            get_timer_read_value_func = ll_aon_wdt_get_reload_read_counter;
        }
        break;
    }

    /* Use count to check timeout */
    timeout *= 1000;
    while (val0 != val1)
    {
        if (0 == timeout)
        {
            return HAL_TIMEOUT;
        }
        timeout--;

        val0 = get_timer_read_value_func();
        val1 = get_timer_read_value_func();
    }

    *p_value = val0;
    return HAL_OK;
}

void hal_pwr_enter_chip_deepsleep_func(void)
{
     /* Hook Reserved For Sleep Debug, Default is NULL */
    SWDIAG_SHUTDOWN();

    /* Set dpad_le value during sleep and after wake up  */
    ll_pwr_set_dpad_le_value(LL_PWR_DPAD_LE_OFF, LL_PWR_DPAD_LE_OFF);

    /* Request System Shutdown */
    SET_BITS(AON_PWR->AON_SLP_CTRL, AON_PWR_AON_SLP_CTRL_REQ);

    /* Wait Sleep Process Status */
    while(AON_PWR_AON_SLP_CTRL_PRO == READ_BITS(AON_PWR->AON_SLP_CTRL, AON_PWR_AON_SLP_CTRL_PRO));

    return;
}
void (*p_hal_pwr_enter_chip_deepsleep_func)(void) = hal_pwr_enter_chip_deepsleep_func;
void hal_pwr_enter_chip_deepsleep(void)
{
    p_hal_pwr_enter_chip_deepsleep_func();
}

/** @} */

#endif /* HAL_PWR_MODULE_ENABLED */

/** @} */

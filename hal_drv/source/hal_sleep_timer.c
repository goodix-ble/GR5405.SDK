/**
  ****************************************************************************************
  * @file    hal_aon_timer.c
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
#include "gr5x.h"

/** @addtogroup HAL_DRIVER
  * @{
  */

#ifdef HAL_SLEEP_TIMER_MODULE_ENABLED
#include "hal_sleep_timer.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* extern variables ----------------------------------------------------------*/
//extern uint32_t SystemSlowClock;

/** @defgroup PWR_Exported_Functions PWR Exported Functions
  * @{
  */

static hal_status_t sleep_timer_wait_not_busy_until_timeout(uint32_t timeout)
{
    hal_status_t ret = HAL_ERROR;

    if (HAL_MAX_DELAY > timeout)
    {
        //lint -e923 Cast from pointer to unsigned int is necessary
        HAL_TIMEOUT_INIT();
        //lint -e923 Cast from pointer to unsigned int is necessary
        uint32_t tickstart = HAL_TIMEOUT_GET_TICK();
        /* Update SystemCoreClock */
        SystemCoreUpdateClock();
        uint32_t timeout_counter = ((SystemCoreClock / 1000U) * timeout);

        while (ll_sleep_timer_is_busy(SLP_TIMER0))
        {
            if ( (HAL_NEVER_TIMEOUT != timeout) && (0U == timeout) )
            {
                ret = HAL_TIMEOUT;
            }
            //lint -e923 Cast from pointer to unsigned int is necessary
            if ( (HAL_TIMEOUT_GET_TICK() - tickstart) > timeout_counter )
            {
                ret = HAL_TIMEOUT;
            }

            if ( ret == HAL_TIMEOUT )
            {
                break;
            }
        }

        HAL_TIMEOUT_DEINIT();

        if ( ret != HAL_TIMEOUT )
        {
            ret = HAL_OK;
        }
    }
    return ret;
}


__WEAK void hal_pwr_sleep_timer_irq_handler(void)
{
    /* Clear sleep timer wakeup event */
    ll_sleep_timer_clear_flag_it();

    /* Call sleep timer elapsed callback */
    hal_pwr_sleep_timer_elapsed_callback();
}


__WEAK void hal_pwr_sleep_timer_elapsed_callback(void)
{
    return;
}

__WEAK void hal_sleep_timer_stop(void)
{
    hal_status_t status;
    status = sleep_timer_wait_not_busy_until_timeout(20);
    UNUSED(status);
    ll_sleep_timer_disable(SLP_TIMER0);
}

__WEAK hal_status_t hal_sleep_timer_config_and_start(uint8_t mode, uint32_t value)
{
    hal_status_t status;

    status = sleep_timer_wait_not_busy_until_timeout(20);

    ll_sleep_timer_set_counter_value(SLP_TIMER0, value);
    ll_sleep_timer_config_and_start(SLP_TIMER0, mode);

    return status;
}

__WEAK uint32_t hal_sleep_timer_get_reload_value(void)
{
    return ll_sleep_timer_get_counter_value(SLP_TIMER0);
}

__WEAK uint32_t hal_sleep_timer_get_current_value(void)
{
    return ll_sleep_timer_get_counter_int_value(SLP_TIMER0);
}

__WEAK uint32_t hal_sleep_timer_get_clock_freq(void)
{
    //return SystemSlowClock;
    return 32768;
}

__WEAK uint32_t hal_sleep_timer_status_get(void)
{
    return ll_sleep_timer_is_running(SLP_TIMER0);
}

/** @} */

#endif /* HAL_PWR_MODULE_ENABLED */

/** @} */

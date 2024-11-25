/**
  ****************************************************************************************
  * @file    hal_aon_wdt.c
  * @author  BLE Driver Team
  * @brief   AON WDT HAL module driver.
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

#ifdef HAL_AON_WDT_MODULE_ENABLED
#include "gr_common.h"
#include "hal_aon_wdt.h"

/** @addtogroup AON_WDT AON_WDT
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define AON_WDT_PER_MS(SECOND)                      ((double)((SECOND) / 1000.0) +  (double)(((SECOND) % 1000U) / 1000.0))

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static hal_status_t aon_wdt_wait_flag_state_until_timeout(void);
/* Exported functions --------------------------------------------------------*/

/** @defgroup AON_WDT_Exported_Functions AON_WDT Exported Functions
  * @{
  */

/** @defgroup AON_WDT_Exported_Functions_Group1 Initialization and Configuration functions
 *  @brief    Initialization and Configuration functions.
  * @{
  */

__WEAK hal_status_t hal_aon_wdt_init(aon_wdt_handle_t *p_aon_wdt)
{
    uint32_t wait_count = 1000;
    hal_status_t status = HAL_OK;
    uint32_t counter = 32768;

    /* Check the AON_WDT handle allocation */
    if (p_aon_wdt->SystemCoreLowClock == NULL)
    {
        status = HAL_ERROR;
        goto EXIT;
    }

    ll_aon_wdt_unlock();
    do {
        /* Disable AON_WDT */
        ll_aon_wdt_disable();
        status = aon_wdt_wait_flag_state_until_timeout();
        if (status != HAL_OK)
        {
            break;
        }
        //lint -e9029 -e9033 for higher accuracy
        counter = (uint32_t)(((double)p_aon_wdt->init.counter * AON_WDT_PER_MS((*p_aon_wdt->SystemCoreLowClock))));
        /* Set WDT reload counter value */
        ll_aon_wdt_set_reload_counter(counter);

        /* Load reload counter value into AON_WDT */
        ll_aon_wdt_reload_counter();
        status = aon_wdt_wait_flag_state_until_timeout();
        if (status != HAL_OK)
        {
            break;
        }

        ll_aon_wdt_clear_flag_alarm();

        /* Set alarm counter value */
        if (0U != p_aon_wdt->init.alarm_counter)
        {
            counter = (uint32_t)(((double)p_aon_wdt->init.alarm_counter * AON_WDT_PER_MS((*p_aon_wdt->SystemCoreLowClock))));
            if(counter > 0xFFFFU)
            {
                counter = 0xFFFFU;
            }

            ll_aon_wdt_set_alarm_counter_and_request(counter);
            status = aon_wdt_wait_flag_state_until_timeout();
            if (status != HAL_OK)
            {
                break;
            }

            ll_aon_wdt_it_enable_alarm();
            status = aon_wdt_wait_flag_state_until_timeout();
            if (status != HAL_OK)
            {
                break;
            }

            /* Clear pending IRQ and eable NVIC interrupt */
            NVIC_ClearPendingIRQ(AON_WDT_IRQn);
            NVIC_EnableIRQ(AON_WDT_IRQn);
        }

        while(wait_count--)
        {
        }

        /* Enable AON_WDT */
        ll_aon_wdt_enable();
        status = aon_wdt_wait_flag_state_until_timeout();
        if (status != HAL_OK)
        {
            break;
        }
    } while(0);
    ll_aon_wdt_lock();
    EXIT:
    return status;
}

__WEAK hal_status_t hal_aon_wdt_deinit(const aon_wdt_handle_t *p_aon_wdt)
{
    UNUSED(p_aon_wdt);

    ll_aon_wdt_unlock();

    /* Disable AON_WDT */
    ll_aon_wdt_disable();
    hal_status_t status = aon_wdt_wait_flag_state_until_timeout();

    if (status == HAL_OK)
    {
        ll_aon_wdt_it_disable_alarm();
        status = aon_wdt_wait_flag_state_until_timeout();
    }

    /* Diseable NVIC interrupt and Clear pending IRQ */
    NVIC_DisableIRQ(AON_WDT_IRQn);
    NVIC_ClearPendingIRQ(AON_WDT_IRQn);

    /* Clear reboot flag */
    //ll_aon_wdt_clear_flag_reboot();
    ll_aon_wdt_clear_flag_alarm();

    ll_aon_wdt_lock();

    return status;
}

/** @} */

/** @defgroup AON_WDT_Exported_Functions_Group2 IO operation functions
 *  @brief    IO operation functions
  * @{
  */

__WEAK hal_status_t hal_aon_wdt_refresh(const aon_wdt_handle_t *p_aon_wdt)
{
    uint32_t counter = 0;

    if ((ll_aon_wdt_is_active_flag_running() == 0U) ||
        (p_aon_wdt->init.counter == 0U) || (p_aon_wdt->SystemCoreLowClock == NULL))
    {
        return HAL_ERROR;
    }

    ll_aon_wdt_unlock();

    counter = (uint32_t)(((double)p_aon_wdt->init.counter * AON_WDT_PER_MS((*p_aon_wdt->SystemCoreLowClock))));
    /* Set WDT reload counter value */
    ll_aon_wdt_set_reload_counter(counter);

    ll_aon_wdt_reload_counter();

    if (HAL_TIMEOUT == aon_wdt_wait_flag_state_until_timeout())
    {
        ll_aon_wdt_lock();
        return HAL_TIMEOUT;
    }

    ll_aon_wdt_lock();

    return HAL_OK;
}

__WEAK void hal_aon_wdt_irq_handler(aon_wdt_handle_t *p_aon_wdt)
{
    /* Alarm callback  */
    hal_aon_wdt_alarm_callback(p_aon_wdt);

    ll_aon_wdt_clear_flag_alarm();

    /* Wait for more than 32us */
    for (uint32_t count = 0; count < 300U; count++)
    {
        __asm("nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n");
    }
}

__WEAK void hal_aon_wdt_alarm_callback(aon_wdt_handle_t *p_aon_wdt)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_aon_wdt);

    /* NOTE: This function should not be modified, when the callback is needed,
             the hal_aon_wdt_alarm_callback could be implemented in the user file
    */
}

static hal_status_t aon_wdt_wait_flag_state_until_timeout(void)
{
    uint32_t timeout = 500000;

    while(ll_aon_wdt_is_busy())
    {
        if (--timeout == 0U)
        {
            break ;
        }
    }

    return ((0U == timeout) ? HAL_TIMEOUT : HAL_OK);
}

hal_status_t hal_aon_wdt_disable(void)
{
    hal_status_t status = HAL_OK;
    ll_aon_wdt_unlock();
    ll_aon_wdt_disable();

    status = aon_wdt_wait_flag_state_until_timeout();

    ll_aon_wdt_lock();
    return status;
}

hal_status_t hal_aon_wdt_enable(void)
{
    hal_status_t status = HAL_OK;
    ll_aon_wdt_unlock();
    ll_aon_wdt_enable();

    status = aon_wdt_wait_flag_state_until_timeout();

    ll_aon_wdt_lock();
    return status;
}

/** @} */

/** @} */

#endif /* HAL_AON_WDT_MODULE_ENABLED */
/** @} */

/** @} */

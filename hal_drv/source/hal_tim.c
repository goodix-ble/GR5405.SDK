/**
  ****************************************************************************************
  * @file    hal_tim.c
  * @author  BLE Driver Team
  * @brief   TIMER HAL module driver.
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

#include "gr5x.h"

#ifdef HAL_TIMER_MODULE_ENABLED
#include <string.h>
#include "hal_tim.h"

#ifdef HAL_CLOCK_UNIFORM_CONTROL
#include "hal_clock.h"
#endif

void ll_timer_deinit(timer_regs_t *TIMERx);
void ll_timer_init(timer_regs_t *TIMERx, const ll_timer_init_t *p_timer_init);

/**
  * @brief  Set TIMERx registers to their reset values.
  * @param  TIMERx Timer instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TIMERx registers are de-initialized
  *          - ERROR: invalid TIMERx instance
  */
void ll_timer_deinit(timer_regs_t *TIMERx)
{
    /* Check the parameters */
    gr_assert_param(IS_TIMER_ALL_INSTANCE(TIMERx));

    LL_TIMER_WriteReg(TIMERx, CTRL, 0);
    LL_TIMER_WriteReg(TIMERx, RELOAD, 0);
    LL_TIMER_WriteReg(TIMERx, INTEN, 0);
    LL_TIMER_WriteReg(TIMERx, INTSTAT, 0);
}

/**
  * @brief  Configure the TIMERx time base unit.
  * @param  TIMERx Timer instance
  * @param  p_timer_init pointer to a @ref ll_timer_init_t structure (TIMERx time base unit configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TIMERx registers are de-initialized
  *          - ERROR: not applicable
  */
void ll_timer_init(timer_regs_t *TIMERx, const ll_timer_init_t *p_timer_init)
{
    /* Check the parameters */
    gr_assert_param(IS_TIMER_ALL_INSTANCE(TIMERx));

    ll_timer_set_auto_reload(TIMERx, p_timer_init->auto_reload);

    ll_timer_set_channel0_capture_type(TIMERx, p_timer_init->ll_capture_channel0.ll_edge_capture);
    ll_timer_set_channel1_capture_type(TIMERx, p_timer_init->ll_capture_channel1.ll_edge_capture);
    ll_timer_set_channel2_capture_type(TIMERx, p_timer_init->ll_capture_channel2.ll_edge_capture);
    ll_timer_set_channel3_capture_type(TIMERx, p_timer_init->ll_capture_channel3.ll_edge_capture);

    ll_timer_set_channel0_capture_pin(TIMERx, p_timer_init->ll_capture_channel0.ll_capture_pin);
    ll_timer_set_channel1_capture_pin(TIMERx, p_timer_init->ll_capture_channel1.ll_capture_pin);
    ll_timer_set_channel2_capture_pin(TIMERx, p_timer_init->ll_capture_channel2.ll_capture_pin);
    ll_timer_set_channel3_capture_pin(TIMERx, p_timer_init->ll_capture_channel3.ll_capture_pin);
}

hal_status_t hal_timer_init(timer_handle_t *p_timer)
{
    hal_status_t status = HAL_OK;
    ll_timer_init_t timer_init;

    /* Check the parameters */
    gr_assert_param(IS_TIMER_ALL_INSTANCE(p_timer->p_instance));

    if (HAL_TIMER_STATE_RESET == p_timer->state)
    {
        /* init the low level hardware : GPIO, CLOCK */
#ifdef HAL_CLOCK_UNIFORM_CONTROL
        //lint -e923 Cast from pointer to unsigned int is necessary and safe
        hal_clock_enable_module((uint32_t)p_timer->p_instance);
#endif
        hal_timer_msp_init(p_timer);
    }

    p_timer->state = HAL_TIMER_STATE_BUSY;
    //lint -e934 Taking address of near auto variable is necessary
    memcpy(&timer_init, &p_timer->init, sizeof(ll_timer_init_t));

    //lint -e934 Taking address of near auto variable is necessary
    ll_timer_init(p_timer->p_instance, &timer_init);

    /* Initialize the TIMER state */
    p_timer->state = HAL_TIMER_STATE_READY;

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_timer_deinit(timer_handle_t *p_timer)
{
    /* Disable the TIMER Peripheral Clock */
    ll_timer_deinit(p_timer->p_instance);

    /* DeInit the low level hardware: GPIO, CLOCK... */
    hal_timer_msp_deinit(p_timer);

#ifdef HAL_CLOCK_UNIFORM_CONTROL
    //lint -e923 Cast from pointer to unsigned int is necessary and safe
    hal_clock_disable_module((uint32_t)p_timer->p_instance);
#endif

    /* Initialize the TIMER state */
    p_timer->state = HAL_TIMER_STATE_RESET;

    return HAL_OK;
}

__WEAK void hal_timer_msp_init(timer_handle_t *p_timer)
{
    UNUSED(p_timer);
    /* Prevent unused argument(s) compilation warning */
    return;
}

__WEAK void hal_timer_msp_deinit(timer_handle_t *p_timer)
{
    UNUSED(p_timer);
    /* Prevent unused argument(s) compilation warning */
    return;
}

__WEAK void hal_timer_irq_handler(timer_handle_t *p_timer)
{
    uint32_t itflag = ll_timer_get_it_flag(p_timer->p_instance);

    if(itflag & LL_TIMER_INTSTAT_COUNTDONE)
    {
        ll_timer_clear_countdone_flag_it(p_timer->p_instance);
        hal_timer_period_elapsed_callback(p_timer);
    }

    if(itflag & LL_TIMER_INTSTAT_CH0)
    {
        ll_timer_clear_channel0_flag_it(p_timer->p_instance);
        hal_timer_channel0_event_callback(p_timer);
    }

    if(itflag & LL_TIMER_INTSTAT_CH1)
    {
        ll_timer_clear_channel1_flag_it(p_timer->p_instance);
        hal_timer_channel1_event_callback(p_timer);
    }

    if(itflag & LL_TIMER_INTSTAT_CH2)
    {
        ll_timer_clear_channel2_flag_it(p_timer->p_instance);
        hal_timer_channel2_event_callback(p_timer);
    }

    if(itflag & LL_TIMER_INTSTAT_CH3)
    {
        ll_timer_clear_channel3_flag_it(p_timer->p_instance);
        hal_timer_channel3_event_callback(p_timer);
    }

    if(itflag & LL_TIMER_INTSTAT_BLEPULSE1)
    {
        ll_timer_clear_blepulse1_flag_it(p_timer->p_instance);
        hal_timer_blepulse1_event_callback(p_timer);
    }

    if(itflag & LL_TIMER_INTSTAT_BLEPULSE2)
    {
        ll_timer_clear_blepulse2_flag_it(p_timer->p_instance);
        hal_timer_blepulse2_event_callback(p_timer);
    }

    return;
}

__WEAK hal_status_t hal_timer_start(timer_handle_t *p_timer)
{
    hal_status_t status = HAL_OK;

    if (HAL_TIMER_STATE_READY == p_timer->state)
    {
        p_timer->state = HAL_TIMER_STATE_BUSY;

        __HAL_TIMER_DISABLE_ALL_IT(p_timer);
        __HAL_TIMER_ENABLE(p_timer);
    }
    else
    {
        status = HAL_BUSY;
    }

    return status;
}

__WEAK hal_status_t hal_timer_stop(timer_handle_t *p_timer)
{
    hal_status_t status = HAL_OK;

    if ( 0U == ll_timer_is_enabled_it(p_timer->p_instance))
    {
        if (HAL_TIMER_STATE_BUSY == p_timer->state)
        {
            p_timer->state = HAL_TIMER_STATE_READY;
            __HAL_TIMER_DISABLE(p_timer);
        }
        else
        {
            status = HAL_ERROR;
        }
    }
    else
    {
        status = HAL_ERROR;
    }
    return status;
}

__WEAK hal_status_t hal_timer_start_it(timer_handle_t *p_timer)
{
    hal_status_t status = HAL_OK;

    if (HAL_TIMER_STATE_READY == p_timer->state)
    {
        p_timer->state = HAL_TIMER_STATE_BUSY;
        __HAL_TIMER_ENABLE_ALL_IT(p_timer);
        __HAL_TIMER_ENABLE(p_timer);
    }
    else
    {
        status = HAL_BUSY;
    }

    return status;
}

__WEAK hal_status_t hal_timer_stop_it(timer_handle_t *p_timer)
{
    hal_status_t status = HAL_OK;

    if (HAL_TIMER_STATE_BUSY == p_timer->state)
    {
        if (1U == ll_timer_is_enabled_it(p_timer->p_instance))
        {
            p_timer->state = HAL_TIMER_STATE_READY;

            __HAL_TIMER_DISABLE(p_timer);
            __HAL_TIMER_DISABLE_ALL_IT(p_timer);
        }
        else
        {
            status = HAL_ERROR;
        }
    }
    else
    {
        status = HAL_ERROR;
    }
    return status;
}

__WEAK void hal_timer_period_elapsed_callback(timer_handle_t *p_timer)
{
    UNUSED(p_timer);
    return;
}

__WEAK void hal_timer_channel0_event_callback(timer_handle_t *p_timer)
{
    UNUSED(p_timer);
    return;
}

__WEAK void hal_timer_channel1_event_callback(timer_handle_t *p_timer)
{
    UNUSED(p_timer);
    return;
}

__WEAK void hal_timer_channel2_event_callback(timer_handle_t *p_timer)
{
    UNUSED(p_timer);
    return;
}

__WEAK void hal_timer_channel3_event_callback(timer_handle_t *p_timer)
{
    UNUSED(p_timer);
    return;
}

__WEAK void hal_timer_blepulse1_event_callback(timer_handle_t *p_timer)
{
    UNUSED(p_timer);
    return;
}

__WEAK void hal_timer_blepulse2_event_callback(timer_handle_t *p_timer)
{
    UNUSED(p_timer);
    return;
}

__WEAK hal_timer_state_t hal_timer_get_state(const timer_handle_t *p_timer)
{
    return p_timer->state;
}

__WEAK hal_status_t hal_timer_set_config(const timer_handle_t *p_timer, const timer_init_t *p_structure)
{
    hal_status_t status = HAL_OK;

    if (HAL_TIMER_STATE_READY == p_timer->state)
    {
        ll_timer_init_t timer_init;
        //lint -e934 Taking address of near auto variable is necessary
        memcpy(&timer_init, p_structure, sizeof(ll_timer_init_t));
        //lint -e934 Taking address of near auto variable is necessary
        ll_timer_init(p_timer->p_instance, &timer_init);
    }
    else
    {
        status = HAL_BUSY;
    }

    return status;
}

__WEAK uint32_t hal_timer_get_channel0_val(const timer_handle_t *p_timer)
{
    return ll_timer_get_channel0_val(p_timer->p_instance);
}

__WEAK uint32_t hal_timer_get_channel1_val(const timer_handle_t *p_timer)
{
    return ll_timer_get_channel1_val(p_timer->p_instance);
}

__WEAK uint32_t hal_timer_get_channel2_val(const timer_handle_t *p_timer)
{
    return ll_timer_get_channel2_val(p_timer->p_instance);
}

__WEAK uint32_t hal_timer_get_channel3_val(const timer_handle_t *p_timer)
{
    return ll_timer_get_channel3_val(p_timer->p_instance);
}

#endif


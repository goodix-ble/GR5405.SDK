/**
  ****************************************************************************************
  * @file    hal_dual_tim.c
  * @author  BLE Driver Team
  * @brief   DUAL TIM HAL module driver.
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

#ifdef HAL_DUAL_TIMER_MODULE_ENABLED

#include "hal_dual_tim.h"

#ifdef HAL_CLOCK_UNIFORM_CONTROL
#include "hal_clock.h"
#endif

void ll_dual_timer_deinit(dual_timer_regs_t *DUAL_TIMERx);
void ll_dual_timer_init(dual_timer_regs_t *DUAL_TIMERx, const ll_dual_timer_init_t *p_dual_timer_init);

/**
  * @brief  Set DUAL_TIMERx registers to their reset values.
  * @param  DUAL_TIMERx Timer instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: DUAL_TIMERx registers are de-initialized
  *          - ERROR: invalid DUAL_TIMERx instance
  */
void ll_dual_timer_deinit(dual_timer_regs_t *DUAL_TIMERx)
{
    /* Check the parameters */
    gr_assert_param(IS_DUAL_TIM_ALL_INSTANCE(DUAL_TIMERx));

    LL_DUAL_TIMER_WriteReg(DUAL_TIMERx, IO_INIT_SET, 0);
    LL_DUAL_TIMER_WriteReg(DUAL_TIMERx, IO_ACT_CTRL, 0);
    LL_DUAL_TIMER_WriteReg(DUAL_TIMERx, CTRL, 0);
    LL_DUAL_TIMER_WriteReg(DUAL_TIMERx, RELOAD, 0);
    LL_DUAL_TIMER_WriteReg(DUAL_TIMERx, BG_LOAD, 0);
    LL_DUAL_TIMER_WriteReg(DUAL_TIMERx, COUNT_A1IO, 0xFFFFFFFFU);
    LL_DUAL_TIMER_WriteReg(DUAL_TIMERx, COUNT_A2IO, 0xFFFFFFFFU);
    LL_DUAL_TIMER_WriteReg(DUAL_TIMERx, COUNT_B1IO, 0xFFFFFFFFU);
    LL_DUAL_TIMER_WriteReg(DUAL_TIMERx, COUNT_B2IO, 0xFFFFFFFFU);
    LL_DUAL_TIMER_WriteReg(DUAL_TIMERx, COUNT_C1IO, 0xFFFFFFFFU);
    LL_DUAL_TIMER_WriteReg(DUAL_TIMERx, COUNT_C2IO, 0xFFFFFFFFU);
    LL_DUAL_TIMER_WriteReg(DUAL_TIMERx, TP_LOAD, 0);
    LL_DUAL_TIMER_WriteReg(DUAL_TIMERx, PERIOD_COUNT, 0);
    LL_DUAL_TIMER_WriteReg(DUAL_TIMERx, INTCLR, 1);
    LL_DUAL_TIMER_WriteReg(DUAL_TIMERx, IO_BLE_INTCLR, 0xFFF);
}

/**
  * @brief  Configure the DUAL_TIMERx time base unit.
  * @param  DUAL_TIMERx Timer instance
  * @param  p_dual_tim_init pointer to a @ref ll_dual_timer_init_t structure (DUAL_TIMERx time base unit configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: DUAL_TIMERx registers are de-initialized
  *          - ERROR: not applicable
  */
void ll_dual_timer_init(dual_timer_regs_t *DUAL_TIMERx, const ll_dual_timer_init_t *p_dual_timer_init)
{
    /* Check the parameters */
    gr_assert_param(IS_DUAL_TIM_ALL_INSTANCE(DUAL_TIMERx));

    ll_dual_timer_set_auto_reload(DUAL_TIMERx, p_dual_timer_init->auto_reload);
    ll_dual_timer_set_counter_mode(DUAL_TIMERx, p_dual_timer_init->counter_mode);
    ll_dual_timer_set_prescaler(DUAL_TIMERx, p_dual_timer_init->prescaler);
    ll_dual_timer_set_counter_size(DUAL_TIMERx, p_dual_timer_init->counter_size);
}

__WEAK hal_status_t hal_dual_timer_base_init(dual_timer_handle_t *p_dual_timer)
{
    hal_status_t status              = HAL_OK;
    ll_dual_timer_init_t dual_tim_init;

    dual_tim_init.prescaler   = LL_DUAL_TIMER_PRESCALER_DIV0;
    dual_tim_init.counter_size = LL_DUAL_TIMER_COUNTERSIZE_32;
    dual_tim_init.counter_mode = LL_DUAL_TIMER_PERIODIC_MODE;
    dual_tim_init.auto_reload  = SystemCoreClock - 1U;

    /* Check the parameters */
    gr_assert_param(IS_DUAL_TIM_ALL_INSTANCE(p_dual_timer->p_instance));
    gr_assert_param(IS_DUAL_TIMER_PRESCALER(p_dual_timer->init.prescaler));
    gr_assert_param(IS_DUAL_TIMER_COUNTERMODE(p_dual_timer->init.counter_mode));

    if (HAL_DUAL_TIMER_STATE_RESET == p_dual_timer->state)
    {
        /* init the low level hardware : GPIO, CLOCK */
#ifdef HAL_CLOCK_UNIFORM_CONTROL
        //lint -e923 Cast from pointer to unsigned int is necessary and safe
        hal_clock_enable_module((uint32_t)p_dual_timer->p_instance);
#endif
        hal_dual_timer_base_msp_init(p_dual_timer);
    }

    p_dual_timer->state = HAL_DUAL_TIMER_STATE_BUSY;

    /* Configure DUAL_TIM Clock Prescaler and Clock Mode */
    dual_tim_init.prescaler  = p_dual_timer->init.prescaler;
    dual_tim_init.auto_reload = p_dual_timer->init.auto_reload;
    //lint -e934 Taking address of near auto variable is necessary
    ll_dual_timer_init(p_dual_timer->p_instance, &dual_tim_init);
    if (DUAL_TIMER_COUNTERMODE_ONESHOT == p_dual_timer->init.counter_mode)
    {
        ll_dual_timer_enable_oneshot(p_dual_timer->p_instance);
    }
    else
    {
        ll_dual_timer_disable_oneshot(p_dual_timer->p_instance);
    }

    /* Initialize the DUAL_TIM state */
    p_dual_timer->state = HAL_DUAL_TIMER_STATE_READY;

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_dual_timer_base_deinit(dual_timer_handle_t *p_dual_timer)
{
    /* Disable the DUAL_TIM Peripheral Clock */
    ll_dual_timer_deinit(p_dual_timer->p_instance);

    /* DeInit the low level hardware: GPIO, CLOCK... */
    hal_dual_timer_base_msp_deinit(p_dual_timer);

#ifdef HAL_CLOCK_UNIFORM_CONTROL
    //lint -e923 Cast from pointer to unsigned int is necessary and safe
    hal_clock_disable_module((uint32_t)p_dual_timer->p_instance);
#endif

    /* Initialize the DUAL_TIM state */
    p_dual_timer->state = HAL_DUAL_TIMER_STATE_RESET;

    return HAL_OK;
}

__WEAK void hal_dual_timer_base_msp_init(dual_timer_handle_t *p_dual_timer)
{
	  UNUSED(p_dual_timer);
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
            the hal_dual_timer_base_msp_init can be implemented in the user file
    */
}

__WEAK void hal_dual_timer_base_msp_deinit(dual_timer_handle_t *p_dual_timer)
{
	  UNUSED(p_dual_timer);
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
            the hal_dual_timer_base_msp_deinit can be implemented in the user file
    */
}

__WEAK void hal_dual_timer_irq_handler(dual_timer_handle_t *p_dual_timer)
{
    uint32_t itflag = ll_dual_timer_get_it_flag(p_dual_timer->p_instance);

    if( (itflag & LL_DUAL_TIMER_INTSTAT_COUNTDONE) == LL_DUAL_TIMER_INTSTAT_COUNTDONE )
    {
        __HAL_DUAL_TIMER_CLEAR_FLAG_IT(p_dual_timer);
        /* Reset the dual timer in oneshot mode */
        if (DUAL_TIMER_COUNTERMODE_ONESHOT == p_dual_timer->init.counter_mode)
        {
            p_dual_timer->state = HAL_DUAL_TIMER_STATE_READY;
            __HAL_DUAL_TIMER_DISABLE(p_dual_timer);
            __HAL_DUAL_TIMER_DISABLE_IT(p_dual_timer);
            uint32_t cur_reload_val = ll_dual_timer_get_auto_reload(p_dual_timer->p_instance);
            ll_dual_timer_set_auto_reload(p_dual_timer->p_instance, cur_reload_val);
        }
        hal_dual_timer_period_elapsed_callback(p_dual_timer);
    }

    if( (itflag & LL_DUAL_TIMER_INTSTAT_ACT_START) == LL_DUAL_TIMER_INTSTAT_ACT_START )
    {
        ll_dual_timer_clear_act_start_flag_it(p_dual_timer->p_instance);
        hal_dual_timer_act_start_event_callback(p_dual_timer);
    }

    if( (itflag & LL_DUAL_TIMER_INTSTAT_IOA_ACT_C1) == LL_DUAL_TIMER_INTSTAT_IOA_ACT_C1 )
    {
        ll_dual_timer_clear_ioa_act_c1_flag_it(p_dual_timer->p_instance);
        hal_dual_timer_ioa_act_c1_event_callback(p_dual_timer);
    }

    if( (itflag & LL_DUAL_TIMER_INTSTAT_IOA_ACT_C2) == LL_DUAL_TIMER_INTSTAT_IOA_ACT_C2 )
    {
        ll_dual_timer_clear_ioa_act_c2_flag_it(p_dual_timer->p_instance);
        hal_dual_timer_ioa_act_c2_event_callback(p_dual_timer);
    }

    if( (itflag & LL_DUAL_TIMER_INTSTAT_ACT_PERIOD) == LL_DUAL_TIMER_INTSTAT_ACT_PERIOD )
    {
        ll_dual_timer_clear_act_period_flag_it(p_dual_timer->p_instance);
        hal_dual_timer_act_period_event_callback(p_dual_timer);
    }

    if( (itflag & LL_DUAL_TIMER_INTSTAT_ACT_STOP) == LL_DUAL_TIMER_INTSTAT_ACT_STOP )
    {
        ll_dual_timer_clear_act_stop_flag_it(p_dual_timer->p_instance);
        /* Clear dual timer enbale bit and reset value while using period count mode */
        if ( DUAL_TIMER_COUNTERMODE_LOOP == p_dual_timer->init.counter_mode)
        {
           if ( 0U != ll_dual_timer_get_period_count(p_dual_timer->p_instance) )
           {
              p_dual_timer->state = HAL_DUAL_TIMER_STATE_READY;
              __HAL_DUAL_TIMER_DISABLE(p_dual_timer);
              __HAL_DUAL_TIMER_DISABLE_IT(p_dual_timer);
              uint32_t cur_reload_val = ll_dual_timer_get_auto_reload(p_dual_timer->p_instance);
              ll_dual_timer_set_auto_reload(p_dual_timer->p_instance, cur_reload_val);
           }
        }
        hal_dual_timer_act_stop_event_callback(p_dual_timer);
    }

    if( (itflag & LL_DUAL_TIMER_INTSTAT_IOB_ACT_C1) == LL_DUAL_TIMER_INTSTAT_IOB_ACT_C1 )
    {
        ll_dual_timer_clear_iob_act_c1_flag_it(p_dual_timer->p_instance);
        hal_dual_timer_iob_act_c1_event_callback(p_dual_timer);
    }

    if( (itflag & LL_DUAL_TIMER_INTSTAT_IOB_ACT_C2) == LL_DUAL_TIMER_INTSTAT_IOB_ACT_C2 )
    {
        ll_dual_timer_clear_iob_act_c2_flag_it(p_dual_timer->p_instance);
        hal_dual_timer_iob_act_c2_event_callback(p_dual_timer);
    }

    if( (itflag & LL_DUAL_TIMER_INTSTAT_IOC_ACT_C1) == LL_DUAL_TIMER_INTSTAT_IOC_ACT_C1 )
    {
        ll_dual_timer_clear_ioc_act_c1_flag_it(p_dual_timer->p_instance);
        hal_dual_timer_ioc_act_c1_event_callback(p_dual_timer);
    }

    if ( (itflag & LL_DUAL_TIMER_INTSTAT_IOC_ACT_C2) == LL_DUAL_TIMER_INTSTAT_IOC_ACT_C2 )
    {
        ll_dual_timer_clear_ioc_act_c2_flag_it(p_dual_timer->p_instance);
        hal_dual_timer_ioc_act_c2_event_callback(p_dual_timer);
    }

    if( (itflag & LL_DUAL_TIMER_INTSTAT_BLEPULSE1) == LL_DUAL_TIMER_INTSTAT_BLEPULSE1 )
    {
        ll_dual_timer_clear_ble_pulse1_flag_it(p_dual_timer->p_instance);
        hal_dual_timer_ble_pulse1_event_callback(p_dual_timer);
    }

    if( (itflag & LL_DUAL_TIMER_INTSTAT_BLEPULSE2) == LL_DUAL_TIMER_INTSTAT_BLEPULSE2 )
    {
        ll_dual_timer_clear_ble_pulse2_flag_it(p_dual_timer->p_instance);
        hal_dual_timer_ble_pulse2_event_callback(p_dual_timer);
    }

    return;
}

__WEAK hal_status_t hal_dual_timer_base_start(dual_timer_handle_t *p_dual_timer)
{
    hal_status_t status = HAL_OK;

    if (HAL_DUAL_TIMER_STATE_READY == p_dual_timer->state)
    {
        p_dual_timer->state = HAL_DUAL_TIMER_STATE_BUSY;
        __HAL_DUAL_TIMER_DISABLE_IT(p_dual_timer);
        __HAL_DUAL_TIMER_ENABLE(p_dual_timer);
    }
    else
    {
        status = HAL_BUSY;
    }

    return status;
}

__WEAK hal_status_t hal_dual_timer_base_stop(dual_timer_handle_t *p_dual_timer)
{
    hal_status_t status = HAL_ERROR;

    if ( HAL_DUAL_TIMER_STATE_BUSY == p_dual_timer->state )
    {
        if ( 0U == ll_dual_timer_is_enabled_it(p_dual_timer->p_instance) )
        {
            p_dual_timer->state = HAL_DUAL_TIMER_STATE_READY;
            __HAL_DUAL_TIMER_DISABLE(p_dual_timer);
            status = HAL_OK;
        }
    }
    return status;
}

__WEAK hal_status_t hal_dual_timer_base_start_it(dual_timer_handle_t *p_dual_timer)
{
    hal_status_t status = HAL_OK;

    if (HAL_DUAL_TIMER_STATE_READY == p_dual_timer->state)
    {
        p_dual_timer->state = HAL_DUAL_TIMER_STATE_BUSY;
        __HAL_DUAL_TIMER_ENABLE_IT(p_dual_timer);

        __HAL_DUAL_TIMER_ENABLE(p_dual_timer);
    }
    else
    {
        status = HAL_BUSY;
    }

    return status;
}

__WEAK hal_status_t hal_dual_timer_base_stop_it(dual_timer_handle_t *p_dual_timer)
{
    hal_status_t status = HAL_ERROR;
    if ( HAL_DUAL_TIMER_STATE_BUSY == p_dual_timer->state )
    {
        if ( 1U == ll_dual_timer_is_enabled_it(p_dual_timer->p_instance) )
        {
            p_dual_timer->state = HAL_DUAL_TIMER_STATE_READY;
            __HAL_DUAL_TIMER_DISABLE(p_dual_timer);
            __HAL_DUAL_TIMER_DISABLE_IT(p_dual_timer);
            status = HAL_OK;
        }
    }
    return status;
}

__WEAK void hal_dual_timer_period_elapsed_callback(dual_timer_handle_t *p_dual_timer)
{
		UNUSED(p_dual_timer);
    return;
}

__WEAK void hal_dual_timer_act_start_event_callback(dual_timer_handle_t *p_dual_timer)
{
		UNUSED(p_dual_timer);
    return;
}

__WEAK void hal_dual_timer_ioa_act_c1_event_callback(dual_timer_handle_t *p_dual_timer)
{
		UNUSED(p_dual_timer);
    return;
}

__WEAK void hal_dual_timer_ioa_act_c2_event_callback(dual_timer_handle_t *p_dual_timer)
{
	  UNUSED(p_dual_timer);
    return;
}

__WEAK void hal_dual_timer_act_period_event_callback(dual_timer_handle_t *p_dual_timer)
{
		UNUSED(p_dual_timer);
    return;
}

__WEAK void hal_dual_timer_act_stop_event_callback(dual_timer_handle_t *p_dual_timer)
{
		UNUSED(p_dual_timer);
    return;
}

__WEAK void hal_dual_timer_iob_act_c1_event_callback(dual_timer_handle_t *p_dual_timer)
{
		UNUSED(p_dual_timer);
    return;
}

__WEAK void hal_dual_timer_iob_act_c2_event_callback(dual_timer_handle_t *p_dual_timer)
{
		UNUSED(p_dual_timer);
    return;
}

__WEAK void hal_dual_timer_ioc_act_c1_event_callback(dual_timer_handle_t *p_dual_timer)
{
		UNUSED(p_dual_timer);
    return;
}

__WEAK void hal_dual_timer_ioc_act_c2_event_callback(dual_timer_handle_t *p_dual_timer)
{
		UNUSED(p_dual_timer);
    return;
}

__WEAK void hal_dual_timer_ble_pulse1_event_callback(dual_timer_handle_t *p_dual_timer)
{
		UNUSED(p_dual_timer);
    return;
}

__WEAK void hal_dual_timer_ble_pulse2_event_callback(dual_timer_handle_t *p_dual_timer)
{
		UNUSED(p_dual_timer);
    return;
}

__WEAK hal_dual_timer_state_t hal_dual_timer_get_state(const dual_timer_handle_t *p_dual_timer)
{
    return p_dual_timer->state;
}

__WEAK hal_status_t hal_dual_timer_set_config(dual_timer_handle_t *p_dual_timer, const dual_timer_init_t *p_structure)
{
    hal_status_t status = HAL_OK;

    p_dual_timer->init.counter_mode = p_structure->counter_mode;
    p_dual_timer->init.auto_reload = p_structure->auto_reload;
    p_dual_timer->init.prescaler = p_structure->prescaler;
    if (HAL_DUAL_TIMER_STATE_READY == p_dual_timer->state)
    {
        ll_dual_timer_set_prescaler(p_dual_timer->p_instance, p_dual_timer->init.prescaler);
        ll_dual_timer_set_auto_reload(p_dual_timer->p_instance, p_dual_timer->init.auto_reload);
        if (DUAL_TIMER_COUNTERMODE_ONESHOT == p_structure->counter_mode)
        {
            ll_dual_timer_enable_oneshot(p_dual_timer->p_instance);
        }
        else
        {
            ll_dual_timer_set_counter_mode(p_dual_timer->p_instance, LL_DUAL_TIMER_PERIODIC_MODE);
            ll_dual_timer_disable_oneshot(p_dual_timer->p_instance);
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    return status;
}

__WEAK hal_status_t hal_dual_timer_set_background_reload(dual_timer_handle_t *p_dual_timer, uint32_t reload_value)
{
    p_dual_timer->init.auto_reload = reload_value;
    ll_dual_timer_set_background_reload(p_dual_timer->p_instance, reload_value);
    return HAL_OK;
}

__WEAK hal_status_t hal_dual_timer_set_onetime_reload(const dual_timer_handle_t *p_dual_timer, uint32_t reload_value)
{
    ll_dual_timer_set_onetime_reload(p_dual_timer->p_instance, reload_value);
    return HAL_OK;
}

__WEAK hal_status_t hal_dual_timer_set_period_count(const dual_timer_handle_t *p_dual_timer, uint32_t count_value)
{
    hal_status_t ret = HAL_ERROR;

    if ((p_dual_timer->init.counter_mode == DUAL_TIMER_COUNTERMODE_LOOP) && (count_value >= 2U))
    {
        ll_dual_timer_enable_act_stop_it(p_dual_timer->p_instance);
        ll_dual_timer_set_period_count(p_dual_timer->p_instance, count_value);
        ret = HAL_OK;
    }
    return ret;
}

__WEAK hal_status_t hal_dual_timer_io_crtl_config(const dual_timer_handle_t *p_dual_timer, const dual_timer_io_ctrl_cfg_t *p_structure)
{
    hal_status_t status = HAL_OK;

    /* Check the parameters */
    gr_assert_param(IS_DUAL_TIMER_IO_INIT_STATE(p_structure->io_init_state));
    gr_assert_param(IS_DUAL_TIMER_IO_ACT_MODE(p_structure->start_act_mode));
    gr_assert_param(IS_DUAL_TIMER_IO_ACT_MODE(p_structure->period_act_mode));
    gr_assert_param(IS_DUAL_TIMER_IO_ACT_MODE(p_structure->stop_act_mode));
    gr_assert_param(IS_DUAL_TIMER_IO_ACT_MODE(p_structure->count1_act_mode));
    gr_assert_param(IS_DUAL_TIMER_IO_ACT_MODE(p_structure->count2_act_mode));

    if (HAL_DUAL_TIMER_STATE_READY == p_dual_timer->state)
    {
        switch (p_structure->channel)
        {
            case HAL_DUAL_TIMER_CHANNEL_A:
                /* Set io initial state */
                ll_dual_timer_set_ioa_action_init(p_dual_timer->p_instance, p_structure->io_init_state);
                /* Set count1 value, must larger than 0 */
                if (p_structure->count1_value != 0x0U)
                {
                    ll_dual_timer_set_ioa_count1(p_dual_timer->p_instance, p_structure->count1_value);
                }
                /* Set count2 value, must larger than 0 */
                if (p_structure->count2_value != 0x0U)
                {
                    ll_dual_timer_set_ioa_count2(p_dual_timer->p_instance, p_structure->count2_value);
                }
                /* Set io action mode */
                ll_dual_timer_set_ioa_action_start(p_dual_timer->p_instance, p_structure->start_act_mode);
                ll_dual_timer_set_ioa_action_period(p_dual_timer->p_instance, p_structure->period_act_mode);
                ll_dual_timer_set_ioa_action_stop(p_dual_timer->p_instance, p_structure->stop_act_mode);
                ll_dual_timer_set_ioa_action_count1(p_dual_timer->p_instance, p_structure->count1_act_mode);
                ll_dual_timer_set_ioa_action_count2(p_dual_timer->p_instance, p_structure->count2_act_mode);
                /* Set interrupt when start action */
                if (p_structure->start_act_it == (uint32_t)ENABLE)
                {
                    ll_dual_timer_enable_act_start_it(p_dual_timer->p_instance);
                }
                else /* (p_structure->start_act_it == (uint32_t)DISABLE) */
                {
                    ll_dual_timer_disable_act_start_it(p_dual_timer->p_instance);
                }
                /* Set interrupt when period action */
                if (p_structure->period_act_it == (uint32_t)ENABLE)
                {
                    ll_dual_timer_enable_act_period_it(p_dual_timer->p_instance);
                }
                else /* (p_structure->period_act_it == (uint32_t)DISABLE) */
                {
                    ll_dual_timer_disable_act_period_it(p_dual_timer->p_instance);
                }
                /* Set interrupt when stop action */
                if (p_structure->stop_act_it == (uint32_t)ENABLE)
                {
                    ll_dual_timer_enable_act_stop_it(p_dual_timer->p_instance);
                }
                else /*(p_structure->stop_act_it == (uint32_t)DISABLE) */
                {
                    ll_dual_timer_disable_act_stop_it(p_dual_timer->p_instance);
                }
                /* Set interrupt when count1 action */
                if (p_structure->count1_act_it == (uint32_t)ENABLE)
                {
                    ll_dual_timer_enable_ioa_act_c1_it(p_dual_timer->p_instance);
                }
                else /* (p_structure->count1_act_it == (uint32_t)DISABLE) */
                {
                    ll_dual_timer_disable_ioa_act_c1_it(p_dual_timer->p_instance);
                }
                /* Set interrupt when count2 action */
                if (p_structure->count2_act_it == (uint32_t)ENABLE)
                {
                    ll_dual_timer_enable_ioa_act_c2_it(p_dual_timer->p_instance);
                }
                else /* (p_structure->count2_act_it == (uint32_t)DISABLE) */
                {
                    ll_dual_timer_disable_ioa_act_c2_it(p_dual_timer->p_instance);
                }
                /* Enable io ctrl for channel A */
                ll_dual_timer_enable_ioa_ctrl(p_dual_timer->p_instance);
                break;
            case HAL_DUAL_TIMER_CHANNEL_B:
                /* Set io initial state */
                ll_dual_timer_set_iob_action_init(p_dual_timer->p_instance, p_structure->io_init_state);
                /* Set count1 value, must larger than 0 */
                if (p_structure->count1_value != 0x0U)
                {
                    ll_dual_timer_set_iob_count1(p_dual_timer->p_instance, p_structure->count1_value);
                }
                /* Set count2 value, must larger than 0 */
                if (p_structure->count2_value != 0x0U)
                {
                    ll_dual_timer_set_iob_count2(p_dual_timer->p_instance, p_structure->count2_value);
                }
                /* Set io action mode */
                ll_dual_timer_set_iob_action_start(p_dual_timer->p_instance, p_structure->start_act_mode);
                ll_dual_timer_set_iob_action_period(p_dual_timer->p_instance, p_structure->period_act_mode);
                ll_dual_timer_set_iob_action_stop(p_dual_timer->p_instance, p_structure->stop_act_mode);
                ll_dual_timer_set_iob_action_count1(p_dual_timer->p_instance, p_structure->count1_act_mode);
                ll_dual_timer_set_iob_action_count2(p_dual_timer->p_instance, p_structure->count2_act_mode);
                /* Set interrupt when start action */
                if (p_structure->start_act_it == (uint32_t)ENABLE)
                {
                    ll_dual_timer_enable_act_start_it(p_dual_timer->p_instance);
                }
                else /* (p_structure->start_act_it == (uint32_t)DISABLE) */
                {
                    ll_dual_timer_disable_act_start_it(p_dual_timer->p_instance);
                }
                /* Set interrupt when period action */
                if (p_structure->period_act_it == (uint32_t)ENABLE)
                {
                    ll_dual_timer_enable_act_period_it(p_dual_timer->p_instance);
                }
                else /* (p_structure->period_act_it == (uint32_t)DISABLE) */
                {
                    ll_dual_timer_disable_act_period_it(p_dual_timer->p_instance);
                }
                /* Set interrupt when stop action */
                if (p_structure->stop_act_it == (uint32_t)ENABLE)
                {
                    ll_dual_timer_enable_act_stop_it(p_dual_timer->p_instance);
                }
                else /* (p_structure->stop_act_it == (uint32_t)DISABLE) */
                {
                    ll_dual_timer_disable_act_stop_it(p_dual_timer->p_instance);
                }
                /* Set interrupt when count1 action */
                if (p_structure->count1_act_it == (uint32_t)ENABLE)
                {
                    ll_dual_timer_enable_iob_act_c1_it(p_dual_timer->p_instance);
                }
                else /* (p_structure->count1_act_it == (uint32_t)DISABLE) */
                {
                    ll_dual_timer_disable_iob_act_c1_it(p_dual_timer->p_instance);
                }
                /* Set interrupt when count2 action */
                if (p_structure->count2_act_it == (uint32_t)ENABLE)
                {
                    ll_dual_timer_enable_iob_act_c2_it(p_dual_timer->p_instance);
                }
                else /* (p_structure->count2_act_it == (uint32_t)DISABLE)  */
                {
                    ll_dual_timer_disable_iob_act_c2_it(p_dual_timer->p_instance);
                }
                /* Enable io ctrl for channel B */
                ll_dual_timer_enable_iob_ctrl(p_dual_timer->p_instance);
                break;
            case HAL_DUAL_TIMER_CHANNEL_C:
                /* Set io initial state */
                ll_dual_timer_set_ioc_action_init(p_dual_timer->p_instance, p_structure->io_init_state);
                /* Set count1 value, must larger than 0 */
                if (p_structure->count1_value != 0x0U)
                {
                    ll_dual_timer_set_ioc_count1(p_dual_timer->p_instance, p_structure->count1_value);
                }
                /* Set count2 value, must larger than 0 */
                if (p_structure->count2_value != 0x0U)
                {
                    ll_dual_timer_set_ioc_count2(p_dual_timer->p_instance, p_structure->count2_value);
                }
                /* Set io action mode */
                ll_dual_timer_set_ioc_action_start(p_dual_timer->p_instance, p_structure->start_act_mode);
                ll_dual_timer_set_ioc_action_period(p_dual_timer->p_instance, p_structure->period_act_mode);
                ll_dual_timer_set_ioc_action_stop(p_dual_timer->p_instance, p_structure->stop_act_mode);
                ll_dual_timer_set_ioc_action_count1(p_dual_timer->p_instance, p_structure->count1_act_mode);
                ll_dual_timer_set_ioc_action_count2(p_dual_timer->p_instance, p_structure->count2_act_mode);
                /* Set interrupt when start action */
                if (p_structure->start_act_it == (uint32_t)ENABLE)
                {
                    ll_dual_timer_enable_act_start_it(p_dual_timer->p_instance);
                }
                else /* (p_structure->start_act_it == (uint32_t)DISABLE) */
                {
                    ll_dual_timer_disable_act_start_it(p_dual_timer->p_instance);
                }
                /* Set interrupt when period action */
                if (p_structure->period_act_it == (uint32_t)ENABLE)
                {
                    ll_dual_timer_enable_act_period_it(p_dual_timer->p_instance);
                }
                else /* (p_structure->period_act_it == (uint32_t)DISABLE) */
                {
                    ll_dual_timer_disable_act_period_it(p_dual_timer->p_instance);
                }
                /* Set interrupt when stop action */
                if (p_structure->stop_act_it == (uint32_t)ENABLE)
                {
                    ll_dual_timer_enable_act_stop_it(p_dual_timer->p_instance);
                }
                else /* (p_structure->stop_act_it == (uint32_t)DISABLE) */
                {
                    ll_dual_timer_disable_act_stop_it(p_dual_timer->p_instance);
                }
                /* Set interrupt when count1 action */
                if (p_structure->count1_act_it == (uint32_t)ENABLE)
                {
                    ll_dual_timer_enable_ioc_act_c1_it(p_dual_timer->p_instance);
                }
                else /* (p_structure->count1_act_it == (uint32_t)DISABLE) */
                {
                    ll_dual_timer_disable_ioc_act_c1_it(p_dual_timer->p_instance);
                }
                /* Set interrupt when count2 action */
                if (p_structure->count2_act_it == (uint32_t)ENABLE)
                {
                    ll_dual_timer_enable_ioc_act_c2_it(p_dual_timer->p_instance);
                }
                else /* (p_structure->count2_act_it == (uint32_t)DISABLE) */
                {
                    ll_dual_timer_disable_ioc_act_c2_it(p_dual_timer->p_instance);
                }
                /* Enable io ctrl for channel C */
                ll_dual_timer_enable_ioc_ctrl(p_dual_timer->p_instance);
                break;
            default:
                status = HAL_ERROR;
                break;
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    return status;
}

#endif


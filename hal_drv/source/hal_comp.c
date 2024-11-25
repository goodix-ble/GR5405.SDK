/**
  ****************************************************************************************
  * @file    hal_comp.c
  * @author  BLE Driver Team
  * @brief   COMP HAL module driver.
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

#ifdef HAL_COMP_MODULE_ENABLED

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
__WEAK void ll_comp_deinit(void)
{
    /* Reset configuration */
    ll_comp_set_remove_cycle(0x0); //set glitch_remove_cycle to default value

    /* Reset configuration */
    //lint -e923 Cast from pointer to unsigned int is necessary
    LL_COMP_WriteReg(AON_PMU, COMP_REG_0, 0x00000040U);
    LL_COMP_WriteReg(AON_PMU, COMP_REG_1, 0x00000088U);

}

__WEAK void ll_comp_init(const ll_comp_init_t *p_comp_init)
{
    /* Check the parameters */
    gr_assert_param(IS_LL_COMP_INPUT(p_comp_init->input_source));
    gr_assert_param(IS_LL_COMP_REF(p_comp_init->ref_source));
    /* ------------------------- Configure COMP ---------------- */
    ll_comp_set_input_src(p_comp_init->input_source);
    ll_comp_set_ref_src(p_comp_init->ref_source);

    if (LL_COMP_REF_SRC_VBAT == p_comp_init->ref_source)
    {
        ll_comp_set_vbatt_lvl(p_comp_init->ref_value);
    }
    if (LL_COMP_REF_SRC_VREF == p_comp_init->ref_source)
    {
        ll_comp_set_vref_lvl(p_comp_init->ref_value);
    }
    if (LL_COMP_HYST_POSITIVE & p_comp_init->hyst)
    {
        ll_comp_positive_hysteresis(p_comp_init->hyst);
    }
    if (LL_COMP_HYST_NEGATIVE & p_comp_init->hyst)
    {
        ll_comp_negative_hysteresis(p_comp_init->hyst);
    }
    ll_comp_set_current(0xC); // set icomp_ctrl_3_0 to 0xC
    ll_comp_cascres_half_high(0x0); //set cascres_half to 0x0
    ll_comp_set_remove_cycle(0x7); //set glitch_remove_cycle to 0x7
}

__WEAK hal_status_t hal_comp_init(comp_handle_t *p_comp)
{
    hal_status_t   status = HAL_OK;

    if (HAL_COMP_STATE_RESET == p_comp->state)
    {
        /* init the low level hardware : MSIO, NVIC */
        hal_comp_msp_init(p_comp);
    }

    /* Configure COMP peripheral */
    ll_comp_init(&p_comp->init);

    /* Set COMP error code to none */
    p_comp->error_code = HAL_COMP_ERROR_NONE;

    /* Initialize the ADC state */
    p_comp->state = HAL_COMP_STATE_READY;

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_comp_deinit(comp_handle_t *p_comp)
{
    /* Reset COMP Peripheral */
    ll_comp_deinit();

    /* DeInit the low level hardware: GPIO, NVIC... */
    hal_comp_msp_deinit(p_comp);

    /* Set COMP error code to none */
    p_comp->error_code = HAL_COMP_ERROR_NONE;

    /* Initialize the COMP state */
    p_comp->state = HAL_COMP_STATE_RESET;

    return HAL_OK;
}

__WEAK void hal_comp_msp_init(comp_handle_t *p_comp)
{
    UNUSED(p_comp);
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
              the hal_comp_msp_init can be implemented in the user file
     */
}

__WEAK void hal_comp_msp_deinit(comp_handle_t *p_comp)
{
    UNUSED(p_comp);
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
              the hal_comp_msp_deinit can be implemented in the user file
     */
}

__WEAK hal_status_t hal_comp_start(comp_handle_t *p_comp)
{
    hal_status_t status = HAL_OK;

    if (HAL_COMP_STATE_READY == p_comp->state)
    {
        p_comp->error_code = HAL_COMP_ERROR_NONE;

        /* Update COMP state */
        p_comp->state = HAL_COMP_STATE_BUSY;

        ll_comp_clear_rising_triger_flag_it();
        ll_comp_clear_falling_triger_flag_it();
        if(p_comp->init.edge == COMP_WAKEUP_EDGE_BOTH)
        {
            ll_comp_enable_falling_wakeup();
            ll_comp_enable_rising_wakeup();
        }
        else if(p_comp->init.edge == COMP_WAKEUP_EDGE_FALLING)
        {
            ll_comp_enable_falling_wakeup();
            ll_comp_disable_rising_wakeup();
        }
        else /* p_comp->init.edge == COMP_WAKEUP_EDGE_RISING */
        {
            ll_comp_enable_rising_wakeup();
            ll_comp_disable_falling_wakeup();
        }

        /* Enable the comparator. */
        ll_comp_enable();
    }
    else
    {
        status = HAL_BUSY;
    }
    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_comp_stop(comp_handle_t *p_comp)
{
    hal_status_t status = HAL_OK;

    hal_comp_state_t l_state = p_comp->state;
    if ((HAL_COMP_STATE_READY == l_state) ||
        (HAL_COMP_STATE_BUSY == l_state))
    {
        p_comp->error_code = HAL_COMP_ERROR_NONE;

        /* Disable the comparator. */
        ll_comp_disable();

        p_comp->state = HAL_COMP_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK void hal_comp_rising_trigger_callback(comp_handle_t *p_comp)
{
    UNUSED(p_comp);
    return;
}

__WEAK void hal_comp_falling_trigger_callback(comp_handle_t *p_comp)
{
    UNUSED(p_comp);
    return;
}

__WEAK void hal_comp_irq_handler(comp_handle_t *p_comp)
{
    if(ll_comp_is_rising_triger_flag_it())
    {
        /* Clear COMP pending bit */
        ll_comp_clear_rising_triger_flag_it();

        if ((HAL_COMP_STATE_BUSY == p_comp->state) && ( (p_comp->init.edge == COMP_WAKEUP_EDGE_BOTH) || (p_comp->init.edge == COMP_WAKEUP_EDGE_RISING)))
        {
            /* Change state of COMP */
            //p_comp->state = HAL_COMP_STATE_READY;

            hal_comp_rising_trigger_callback(p_comp);
        }
    }
    if(ll_comp_is_falling_triger_flag_it())
    {
        /* Clear COMP pending bit */
        ll_comp_clear_falling_triger_flag_it();

        if ((HAL_COMP_STATE_BUSY == p_comp->state) && ((p_comp->init.edge == COMP_WAKEUP_EDGE_BOTH) || (p_comp->init.edge == COMP_WAKEUP_EDGE_FALLING)))
        {
            /* Change state of COMP */
            //p_comp->state = HAL_COMP_STATE_READY;

            hal_comp_falling_trigger_callback(p_comp);
        }
    }
    return;
}

__WEAK hal_comp_state_t hal_comp_get_state(const comp_handle_t *p_comp)
{
    /* Return COMP handle state */
    return p_comp->state;
}

__WEAK uint32_t hal_comp_get_error(const comp_handle_t *p_comp)
{
    return p_comp->error_code;
}

__WEAK hal_status_t hal_comp_suspend_reg(comp_handle_t *p_comp)
{
    UNUSED(p_comp);
    return HAL_OK;
}

__WEAK hal_status_t hal_comp_resume_reg(comp_handle_t *p_comp)
{
    UNUSED(p_comp);
    return HAL_OK;
}

#endif /* HAL_ADC_MODULE_ENABLED */

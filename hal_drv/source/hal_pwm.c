/**
  ****************************************************************************************
  * @file    hal_pwm.c
  * @author  BLE Driver Team
  * @brief   PWM HAL module driver.
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

#ifdef HAL_PWM_MODULE_ENABLED

#include "hal_pwm.h"

#ifdef HAL_CLOCK_MODULE
#include "hal_cgc.h"
#endif
#ifdef HAL_CLOCK_UNIFORM_CONTROL
#include "hal_clock.h"
#endif
static void pwm_set_device_state(pwm_handle_t *p_pwm, hal_pwm_state_t state);
static void pwm_dma_transmit_cplt(dma_handle_t *p_dma);
static void pwm_dma_error(dma_handle_t *p_dma);

extern void hal_pwm_channel_a_error_callback(pwm_handle_t *p_pwm);
extern void hal_pwm_channel_b_error_callback(pwm_handle_t *p_pwm);
extern void hal_pwm_channel_c_error_callback(pwm_handle_t *p_pwm);
extern void hal_pwm_coding_done_callback(pwm_handle_t *p_pwm);
extern void hal_pwm_coding_load_callback(pwm_handle_t *p_pwm);

/**
  * @brief  Set PWMx registers to their reset values.
  * @param  PWMx PWM instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: PWMx registers are de-initialized
  *          - ERROR: invalid PWMx instance
  */
__WEAK void ll_pwm_deinit(pwm_regs_t *PWMx)
{
    /* Check the parameters */
    gr_assert_param(IS_PWM_ALL_INSTANCE(PWMx));

    LL_PWM_WriteReg(PWMx, MODE, 0);
    LL_PWM_WriteReg(PWMx, UPDATE, 0);
    LL_PWM_WriteReg(PWMx, PRD, 0x64);
    LL_PWM_WriteReg(PWMx, CMPA0, 0);
    LL_PWM_WriteReg(PWMx, CMPA1, 0);
    LL_PWM_WriteReg(PWMx, CMPB0, 0);
    LL_PWM_WriteReg(PWMx, CMPB1, 0);
    LL_PWM_WriteReg(PWMx, CMPC0, 0);
    LL_PWM_WriteReg(PWMx, CMPC1, 0);
    LL_PWM_WriteReg(PWMx, AQCTRL, 0);
    LL_PWM_WriteReg(PWMx, BRPRD, 0);
    LL_PWM_WriteReg(PWMx, HOLD, 0);
    LL_PWM_WriteReg(PWMx, PRD_CYCLES, 0);
    //lint -e923 Cast from pointer to unsigned int is necessary
    if(PWMx == PWM0)
    {
        LL_PWM_WriteReg(PWMx, WAIT_TIME, 0);
        LL_PWM_WriteReg(PWMx, DATA_WIDTH_VALID, 0x1F);
        LL_PWM_WriteReg(PWMx, CODING_DATA, 0);
        LL_PWM_WriteReg(PWMx, CLR_CODING_STATUS, 0x1F);
    }
}

/**
  * @brief  Configure the PWMx time base unit.
  * @param  PWMx PWM instance
  * @param  p_pwm_init pointer to a @ref ll_pwm_init_t structure (PWMx time base unit configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: PWMx registers are de-initialized
  *          - ERROR: not applicable
  */
__WEAK void ll_pwm_init(pwm_regs_t *PWMx, const ll_pwm_init_t *p_pwm_init)
{
    ll_pwm_mode_t cur_mode = p_pwm_init->ll_mode;
    /* Check the parameters */
    gr_assert_param(IS_PWM_ALL_INSTANCE(PWMx));

    ll_pwm_set_mode(PWMx, cur_mode);
    ll_pwm_set_prd_cycles(PWMx, p_pwm_init->ll_prd_cycles);

    if(cur_mode == (ll_pwm_mode_t)LL_PWM_CODING_MODE)
    {
        ll_pwm_set_prescaler(PWMx, p_pwm_init->coding_mode_cfg.ll_period);
        if(PWMx == PWM0)
        {
            ll_pwm_set_waiting_time(PWMx, p_pwm_init->coding_mode_cfg.ll_waiting_time);
            ll_pwm_set_data_width_valid(PWMx, p_pwm_init->coding_mode_cfg.ll_data_width_valid);
            ll_pwm_coding_channel_select(PWMx, p_pwm_init->coding_mode_cfg.ll_coding_channel_select);
            ll_pwm_set_waiting_time_level_a(PWMx, p_pwm_init->coding_mode_cfg.ll_channel_a.ll_waiting_time_lvl);
            ll_pwm_set_waiting_time_level_b(PWMx, p_pwm_init->coding_mode_cfg.ll_channel_b.ll_waiting_time_lvl);
            ll_pwm_set_waiting_time_level_c(PWMx, p_pwm_init->coding_mode_cfg.ll_channel_c.ll_waiting_time_lvl);

            if (LL_PWM_DRIVEPOLARITY_POSITIVE == p_pwm_init->coding_mode_cfg.ll_channel_a.ll_drive_polarity)
            {
                ll_pwm_enable_positive_drive_channel_a(PWMx);
            }
            else
            {
                ll_pwm_disable_positive_drive_channel_a(PWMx);
            }

            if (LL_PWM_DRIVEPOLARITY_POSITIVE == p_pwm_init->coding_mode_cfg.ll_channel_b.ll_drive_polarity)
            {
                ll_pwm_enable_positive_drive_channel_b(PWMx);
            }
            else
            {
                ll_pwm_disable_positive_drive_channel_b(PWMx);
            }

            if (LL_PWM_DRIVEPOLARITY_POSITIVE == p_pwm_init->coding_mode_cfg.ll_channel_c.ll_drive_polarity)
            {
                ll_pwm_enable_positive_drive_channel_c(PWMx);
            }
            else
            {
                ll_pwm_disable_positive_drive_channel_c(PWMx);
            }

            ll_pwm_set_compare_a0(PWMx, p_pwm_init->coding_mode_cfg.ll_channel_a.ll_comp0);
            ll_pwm_set_compare_a1(PWMx, p_pwm_init->coding_mode_cfg.ll_channel_a.ll_comp1);
            ll_pwm_set_compare_b0(PWMx, p_pwm_init->coding_mode_cfg.ll_channel_b.ll_comp0);
            ll_pwm_set_compare_b1(PWMx, p_pwm_init->coding_mode_cfg.ll_channel_b.ll_comp1);
            ll_pwm_set_compare_c0(PWMx, p_pwm_init->coding_mode_cfg.ll_channel_c.ll_comp0);
            ll_pwm_set_compare_c1(PWMx, p_pwm_init->coding_mode_cfg.ll_channel_c.ll_comp1);
        }
    }
    else
    {
        ll_pwm_set_prescaler(PWMx, p_pwm_init->none_coding_mode_cfg.prescaler);
        ll_pwm_set_breath_prescaler(PWMx, p_pwm_init->none_coding_mode_cfg.bprescaler);
        ll_pwm_set_hold_prescaler(PWMx, p_pwm_init->none_coding_mode_cfg.hprescaler);
        ll_pwm_set_breath_stop_level(PWMx, p_pwm_init->none_coding_mode_cfg.breathstop_lvl);
        ll_pwm_set_flicker_stop_level_a(PWMx, p_pwm_init->none_coding_mode_cfg.channel_a.flickerstop_lvl);
        ll_pwm_set_flicker_stop_level_b(PWMx, p_pwm_init->none_coding_mode_cfg.channel_b.flickerstop_lvl);
        ll_pwm_set_flicker_stop_level_c(PWMx, p_pwm_init->none_coding_mode_cfg.channel_c.flickerstop_lvl);

        if (LL_PWM_DRIVEPOLARITY_POSITIVE == p_pwm_init->none_coding_mode_cfg.channel_a.drive_polarity)
        {
            ll_pwm_enable_positive_drive_channel_a(PWMx);
        }
        else
        {
            ll_pwm_disable_positive_drive_channel_a(PWMx);
        }

        if (LL_PWM_DRIVEPOLARITY_POSITIVE == p_pwm_init->none_coding_mode_cfg.channel_b.drive_polarity)
        {
            ll_pwm_enable_positive_drive_channel_b(PWMx);
        }
        else
        {
            ll_pwm_disable_positive_drive_channel_b(PWMx);
        }

        if (LL_PWM_DRIVEPOLARITY_POSITIVE == p_pwm_init->none_coding_mode_cfg.channel_c.drive_polarity)
        {
            ll_pwm_enable_positive_drive_channel_c(PWMx);
        }
        else
        {
            ll_pwm_disable_positive_drive_channel_c(PWMx);
        }

        if(p_pwm_init->none_coding_mode_cfg.align == LL_PWM_EDGE_ALIGNED)
        {
            ll_pwm_set_compare_a0(PWMx, 0);
            ll_pwm_set_compare_a1(PWMx, p_pwm_init->none_coding_mode_cfg.prescaler * p_pwm_init->none_coding_mode_cfg.channel_a.duty / 100U);
            ll_pwm_set_compare_b0(PWMx, 0);
            ll_pwm_set_compare_b1(PWMx, p_pwm_init->none_coding_mode_cfg.prescaler * p_pwm_init->none_coding_mode_cfg.channel_b.duty / 100U);
            ll_pwm_set_compare_c0(PWMx, 0);
            ll_pwm_set_compare_c1(PWMx, p_pwm_init->none_coding_mode_cfg.prescaler * p_pwm_init->none_coding_mode_cfg.channel_c.duty / 100U);
        }
        else /* (p_pwm_init->none_coding_mode_cfg.align == LL_PWM_CENTER_ALIGNED) */
        {
            uint8_t max_duty = (p_pwm_init->none_coding_mode_cfg.channel_a.duty >= p_pwm_init->none_coding_mode_cfg.channel_b.duty) ?
                       p_pwm_init->none_coding_mode_cfg.channel_a.duty : p_pwm_init->none_coding_mode_cfg.channel_b.duty;
            max_duty = (p_pwm_init->none_coding_mode_cfg.channel_c.duty >= max_duty) ? p_pwm_init->none_coding_mode_cfg.channel_c.duty : max_duty;

            if(max_duty > p_pwm_init->none_coding_mode_cfg.channel_a.duty)
            {
                ll_pwm_set_compare_a0(PWMx, (p_pwm_init->none_coding_mode_cfg.prescaler * max_duty / 100U / 2U) -
                                      (p_pwm_init->none_coding_mode_cfg.prescaler * p_pwm_init->none_coding_mode_cfg.channel_a.duty / 100U / 2U));
                ll_pwm_set_compare_a1(PWMx, (p_pwm_init->none_coding_mode_cfg.prescaler * max_duty / 100U / 2U) +
                                      (p_pwm_init->none_coding_mode_cfg.prescaler * p_pwm_init->none_coding_mode_cfg.channel_a.duty / 100U / 2U));
            }
            else
            {
                ll_pwm_set_compare_a0(PWMx, 0);
                ll_pwm_set_compare_a1(PWMx, p_pwm_init->none_coding_mode_cfg.prescaler * p_pwm_init->none_coding_mode_cfg.channel_a.duty / 100U);
            }

            if(max_duty > p_pwm_init->none_coding_mode_cfg.channel_b.duty)
            {
                ll_pwm_set_compare_b0(PWMx, (p_pwm_init->none_coding_mode_cfg.prescaler * max_duty / 100U / 2U) -
                                      (p_pwm_init->none_coding_mode_cfg.prescaler * p_pwm_init->none_coding_mode_cfg.channel_b.duty / 100U / 2U));
                ll_pwm_set_compare_b1(PWMx, (p_pwm_init->none_coding_mode_cfg.prescaler * max_duty / 100U / 2U) +
                                      (p_pwm_init->none_coding_mode_cfg.prescaler * p_pwm_init->none_coding_mode_cfg.channel_b.duty / 100U / 2U));
            }
            else
            {
                ll_pwm_set_compare_b0(PWMx, 0);
                ll_pwm_set_compare_b1(PWMx, p_pwm_init->none_coding_mode_cfg.prescaler * p_pwm_init->none_coding_mode_cfg.channel_b.duty / 100U);
            }

            if(max_duty > p_pwm_init->none_coding_mode_cfg.channel_c.duty)
            {
                ll_pwm_set_compare_c0(PWMx, (p_pwm_init->none_coding_mode_cfg.prescaler * max_duty / 100U / 2U) -
                                      (p_pwm_init->none_coding_mode_cfg.prescaler * p_pwm_init->none_coding_mode_cfg.channel_c.duty / 100U / 2U));
                ll_pwm_set_compare_c1(PWMx, (p_pwm_init->none_coding_mode_cfg.prescaler * max_duty / 100U / 2U) +
                                      (p_pwm_init->none_coding_mode_cfg.prescaler * p_pwm_init->none_coding_mode_cfg.channel_c.duty / 100U / 2U));
            }
            else
            {
                ll_pwm_set_compare_c0(PWMx, 0);
                ll_pwm_set_compare_c1(PWMx, p_pwm_init->none_coding_mode_cfg.prescaler * p_pwm_init->none_coding_mode_cfg.channel_c.duty / 100U);
            }
        }
        ll_pwm_set_action_event_cmp_a0(PWMx, LL_PWM_ACTIONEVENT_SET);
        ll_pwm_set_action_event_cmp_a1(PWMx, LL_PWM_ACTIONEVENT_CLEAR);
        ll_pwm_set_action_event_cmp_b0(PWMx, LL_PWM_ACTIONEVENT_SET);
        ll_pwm_set_action_event_cmp_b1(PWMx, LL_PWM_ACTIONEVENT_CLEAR);
        ll_pwm_set_action_event_cmp_c0(PWMx, LL_PWM_ACTIONEVENT_SET);
        ll_pwm_set_action_event_cmp_c1(PWMx, LL_PWM_ACTIONEVENT_CLEAR);
    }
}

static void save_stop_level_cfg(pwm_handle_t *p_pwm)
{
    const pwm_regs_t *p_pwm_reg = p_pwm->p_instance;

    p_pwm->retention[11] = READ_REG(p_pwm_reg->MODE);
}

static void resume_stop_level_cfg(const pwm_handle_t *p_pwm)
{
    pwm_regs_t *p_pwm_reg = p_pwm->p_instance;

    MODIFY_REG(p_pwm_reg->MODE, PWM_MODE_FLICKER_PAUSE_LEVEL_A, (p_pwm->retention[11] & PWM_MODE_FLICKER_PAUSE_LEVEL_A_MSK));
    MODIFY_REG(p_pwm_reg->MODE, PWM_MODE_FLICKER_PAUSE_LEVEL_B, (p_pwm->retention[11] & PWM_MODE_FLICKER_PAUSE_LEVEL_B_MSK));
    MODIFY_REG(p_pwm_reg->MODE, PWM_MODE_FLICKER_PAUSE_LEVEL_C, (p_pwm->retention[11] & PWM_MODE_FLICKER_PAUSE_LEVEL_C_MSK));
    MODIFY_REG(p_pwm_reg->MODE, PWM_MODE_BREATH_PAUSE_LEVEL, (p_pwm->retention[11] & PWM_MODE_BREATH_PAUSE_LEVEL_MSK));
}

static void pwm_dma_transmit_cplt(dma_handle_t *p_dma)
{
    UNUSED(p_dma);
}

static void pwm_dma_error(dma_handle_t *p_dma)
{
    UNUSED(p_dma);
}

__WEAK hal_status_t hal_pwm_init(pwm_handle_t *p_pwm)
{
    hal_status_t  status   = HAL_OK;
    ll_pwm_init_t pwm_init = LL_PWM_DEFAULT_CONFIG;
    p_pwm->p_hal_pwm_update_freq = NULL;

    /* Check the parameters */
    gr_assert_param(IS_PWM_ALL_INSTANCE(p_pwm->p_instance));
    gr_assert_param(IS_PWM_MODE(p_pwm->init.mode));
    gr_assert_param(IS_PWM_ALIGNMENT_MODE(p_pwm->init.align));
    gr_assert_param(IS_PWM_STOP_LVL(p_pwm->init.bstoplvl));
    gr_assert_param(IS_PWM_DRIVEPOLARITY(p_pwm->init.channel_a.drive_polarity));
    gr_assert_param(IS_PWM_DRIVEPOLARITY(p_pwm->init.channel_b.drive_polarity));
    gr_assert_param(IS_PWM_DRIVEPOLARITY(p_pwm->init.channel_c.drive_polarity));
    gr_assert_param(IS_PWM_STOP_LVL(p_pwm->init.channel_a.fstoplvl));
    gr_assert_param(IS_PWM_STOP_LVL(p_pwm->init.channel_b.fstoplvl));
    gr_assert_param(IS_PWM_STOP_LVL(p_pwm->init.channel_c.fstoplvl));
    gr_assert_param(IS_PWM_WAITING_TIME_LVL(p_pwm->init.channel_a.waiting_time_lvl));
    gr_assert_param(IS_PWM_WAITING_TIME_LVL(p_pwm->init.channel_b.waiting_time_lvl));
    gr_assert_param(IS_PWM_WAITING_TIME_LVL(p_pwm->init.channel_c.waiting_time_lvl));
    if ((p_pwm->init.mode == PWM_BREATH_MODE) && ((0U == p_pwm->init.none_coding_mode_cfg.bperiod) || (0U == p_pwm->init.none_coding_mode_cfg.hperiod)))
    {
        status = HAL_ERROR;
        goto EXIT;
    }

    if (HAL_PWM_STATE_RESET == p_pwm->state)
    {
#ifdef HAL_CLOCK_UNIFORM_CONTROL
        //lint -e923 Cast from pointer to unsigned int is necessary
        hal_clock_enable_module((uint32_t)p_pwm->p_instance);
#endif
#ifdef HAL_CLOCK_MODULE
        /* Enable Clock for Serial blocks and Automatic turn off Serial blocks clock during WFI. */
        ll_cgc_disable_force_off_serial_hclk();
        ll_cgc_disable_wfi_off_serial_hclk();

        /* Enable PWM Clock and Automatic turn off PWM clock during WFI. */
        if (p_pwm->p_instance == PWM0)
        {
            ll_cgc_disable_force_off_pwm0_hclk();
            ll_cgc_disable_pwm0_slp_wfi();
        }
        else /* (p_pwm->p_instance == PWM1) */
        {
            ll_cgc_disable_force_off_pwm1_hclk();
            ll_cgc_disable_pwm1_slp_wfi();
        }
#endif
        /* init the low level hardware : GPIO, CLOCK */
        hal_pwm_msp_init(p_pwm);
    }

    pwm_set_device_state(p_pwm, HAL_PWM_STATE_BUSY);

    /* Update SystemCoreClock */
    SystemCoreUpdateClock();

    /* Configure PWM Clock Prescaler and Clock Mode */
    pwm_init.ll_mode                                          = (ll_pwm_mode_t)p_pwm->init.mode;
    pwm_init.ll_prd_cycles                                    = p_pwm->init.prd_cycles;

    pwm_init.none_coding_mode_cfg.align                       = p_pwm->init.none_coding_mode_cfg.align;
    pwm_init.none_coding_mode_cfg.prescaler                   = SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq;
    pwm_init.none_coding_mode_cfg.bprescaler                  = SystemCoreClock / 1000U * p_pwm->init.none_coding_mode_cfg.bperiod;
    pwm_init.none_coding_mode_cfg.hprescaler                  = SystemCoreClock / 1000U * p_pwm->init.none_coding_mode_cfg.hperiod;
    pwm_init.none_coding_mode_cfg.breathstop_lvl              = p_pwm->init.none_coding_mode_cfg.bstoplvl;
    pwm_init.none_coding_mode_cfg.channel_a.duty              = p_pwm->init.none_coding_mode_cfg.channel_a.duty;
    pwm_init.none_coding_mode_cfg.channel_a.drive_polarity    = p_pwm->init.none_coding_mode_cfg.channel_a.drive_polarity;
    pwm_init.none_coding_mode_cfg.channel_a.flickerstop_lvl   = p_pwm->init.none_coding_mode_cfg.channel_a.fstoplvl;
    pwm_init.none_coding_mode_cfg.channel_b.duty              = p_pwm->init.none_coding_mode_cfg.channel_b.duty;
    pwm_init.none_coding_mode_cfg.channel_b.drive_polarity    = p_pwm->init.none_coding_mode_cfg.channel_b.drive_polarity;
    pwm_init.none_coding_mode_cfg.channel_b.flickerstop_lvl   = p_pwm->init.none_coding_mode_cfg.channel_b.fstoplvl;
    pwm_init.none_coding_mode_cfg.channel_c.duty              = p_pwm->init.none_coding_mode_cfg.channel_c.duty;
    pwm_init.none_coding_mode_cfg.channel_c.drive_polarity    = p_pwm->init.none_coding_mode_cfg.channel_c.drive_polarity;
    pwm_init.none_coding_mode_cfg.channel_c.flickerstop_lvl   = p_pwm->init.none_coding_mode_cfg.channel_c.fstoplvl;

    pwm_init.coding_mode_cfg.ll_period                        = p_pwm->init.coding_mode_cfg.period;
    pwm_init.coding_mode_cfg.ll_waiting_time                  = p_pwm->init.coding_mode_cfg.waiting_time;
    pwm_init.coding_mode_cfg.ll_data_width_valid              = p_pwm->init.coding_mode_cfg.data_width_valid;
    pwm_init.coding_mode_cfg.ll_coding_channel_select         = p_pwm->init.coding_mode_cfg.coding_channel_select;
    pwm_init.coding_mode_cfg.ll_channel_a.ll_comp0            = p_pwm->init.coding_mode_cfg.channel_a.comp0;
    pwm_init.coding_mode_cfg.ll_channel_a.ll_comp1            = p_pwm->init.coding_mode_cfg.channel_a.comp1;
    pwm_init.coding_mode_cfg.ll_channel_a.ll_drive_polarity   = p_pwm->init.coding_mode_cfg.channel_a.drive_polarity;
    pwm_init.coding_mode_cfg.ll_channel_a.ll_waiting_time_lvl = p_pwm->init.coding_mode_cfg.channel_a.waiting_time_lvl;
    pwm_init.coding_mode_cfg.ll_channel_b.ll_comp0            = p_pwm->init.coding_mode_cfg.channel_b.comp0;
    pwm_init.coding_mode_cfg.ll_channel_b.ll_comp1            = p_pwm->init.coding_mode_cfg.channel_b.comp1;
    pwm_init.coding_mode_cfg.ll_channel_b.ll_drive_polarity   = p_pwm->init.coding_mode_cfg.channel_b.drive_polarity;
    pwm_init.coding_mode_cfg.ll_channel_b.ll_waiting_time_lvl = p_pwm->init.coding_mode_cfg.channel_b.waiting_time_lvl;
    pwm_init.coding_mode_cfg.ll_channel_c.ll_comp0            = p_pwm->init.coding_mode_cfg.channel_c.comp0;
    pwm_init.coding_mode_cfg.ll_channel_c.ll_comp1            = p_pwm->init.coding_mode_cfg.channel_c.comp1;
    pwm_init.coding_mode_cfg.ll_channel_c.ll_drive_polarity   = p_pwm->init.coding_mode_cfg.channel_c.drive_polarity;
    pwm_init.coding_mode_cfg.ll_channel_c.ll_waiting_time_lvl = p_pwm->init.coding_mode_cfg.channel_c.waiting_time_lvl;

    ll_pwm_init(p_pwm->p_instance, &pwm_init);


    if(p_pwm->init.mode == PWM_FLICKER_MODE)
    {
        if (0U == ((uint32_t)p_pwm->active_channel & (uint32_t)HAL_PWM_ACTIVE_CHANNEL_A))
        {
            ll_pwm_set_action_event_cmp_a0(p_pwm->p_instance, LL_PWM_ACTIONEVENT_CLEAR);
            ll_pwm_set_action_event_cmp_a1(p_pwm->p_instance, LL_PWM_ACTIONEVENT_NONE);
        }
        if (0U == ((uint32_t)p_pwm->active_channel & (uint32_t)HAL_PWM_ACTIVE_CHANNEL_B))
        {
            ll_pwm_set_action_event_cmp_b0(p_pwm->p_instance, LL_PWM_ACTIONEVENT_CLEAR);
            ll_pwm_set_action_event_cmp_b1(p_pwm->p_instance, LL_PWM_ACTIONEVENT_NONE);
        }
        if (0U == ((uint32_t)p_pwm->active_channel & (uint32_t)HAL_PWM_ACTIVE_CHANNEL_C))
        {
            ll_pwm_set_action_event_cmp_c0(p_pwm->p_instance, LL_PWM_ACTIONEVENT_CLEAR);
            ll_pwm_set_action_event_cmp_c1(p_pwm->p_instance, LL_PWM_ACTIONEVENT_NONE);
        }
    }

    /* Initialize the PWM state */
    pwm_set_device_state(p_pwm, HAL_PWM_STATE_READY);
EXIT:
    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pwm_deinit(pwm_handle_t *p_pwm)
{
#ifdef HAL_CLOCK_MODULE
    /* Enable PWM Clock for operate the register. */
    if (p_pwm->p_instance == PWM0)
    {
        ll_cgc_disable_force_off_pwm0_hclk();
    }
    else /* (p_pwm->p_instance == PWM1) */
    {
        ll_cgc_disable_force_off_pwm1_hclk();
    }
#endif
    /* Disable the PWM Peripheral Clock */
    ll_pwm_deinit(p_pwm->p_instance);

    /* Disable the PWM peripheral */
    __HAL_PWM_DISABLE(p_pwm);

    /* DeInit the low level hardware: GPIO, CLOCK... */
    hal_pwm_msp_deinit(p_pwm);
#ifdef HAL_CLOCK_UNIFORM_CONTROL
    //lint -e923 Cast from pointer to unsigned int is necessary
     hal_clock_disable_module((uint32_t)p_pwm->p_instance);
#endif
#ifdef HAL_CLOCK_MODULE
    /* Enable PWM Clock and Automatic turn off PWM clock during WFI. */
    if (p_pwm->p_instance == PWM0)
    {
        ll_cgc_enable_force_off_pwm0_hclk();
        ll_cgc_enable_pwm0_slp_wfi();
    }
    else /* (p_pwm->p_instance == PWM1) */
    {
        ll_cgc_enable_force_off_pwm1_hclk();
        ll_cgc_enable_pwm1_slp_wfi();
    }

    GLOBAL_EXCEPTION_DISABLE();
    if(((LL_CGC_FRC_SERIALS_HCLK2 & ll_cgc_get_force_off_hclk_2()) == LL_CGC_FRC_SERIALS_HCLK2))
    {
        /* Disable Clock for Serial blocks  */
        ll_cgc_enable_force_off_serial_hclk();
    }

    if(((LL_CGC_MCU_PERIPH_SERIALS_SLP_OFF & ll_cgc_get_wfi_off_hclk_3()) == LL_CGC_MCU_PERIPH_SERIALS_SLP_OFF))
    {
        /* Disable Clock for Serial blocks  */
        ll_cgc_enable_wfi_off_serial_hclk();
    }
    GLOBAL_EXCEPTION_ENABLE();
#endif
    /* Initialize the PWM state */
    pwm_set_device_state(p_pwm, HAL_PWM_STATE_RESET);

    return HAL_OK;
}

__WEAK void hal_pwm_msp_init(pwm_handle_t *p_pwm)
{
    UNUSED(p_pwm);
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
            the hal_pwm_msp_init can be implemented in the user file
    */
}

__WEAK void hal_pwm_msp_deinit(pwm_handle_t *p_pwm)
{
    UNUSED(p_pwm);
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
            the hal_pwm_msp_deinit can be implemented in the user file
    */
}

__WEAK void hal_pwm_suspend_reg(pwm_handle_t *p_pwm)
{
#ifndef HAL_HW_RES_SUSP_PWM
    const pwm_regs_t *p_pwm_reg = p_pwm->p_instance;

    p_pwm->retention[0] = READ_REG(p_pwm_reg->MODE);
    p_pwm->retention[1] = READ_REG(p_pwm_reg->PRD);
    p_pwm->retention[2] = READ_REG(p_pwm_reg->BRPRD);
    p_pwm->retention[3] = READ_REG(p_pwm_reg->HOLD);
    p_pwm->retention[4] = READ_REG(p_pwm_reg->CMPA0);
    p_pwm->retention[5] = READ_REG(p_pwm_reg->CMPA1);
    p_pwm->retention[6] = READ_REG(p_pwm_reg->CMPB0);
    p_pwm->retention[7] = READ_REG(p_pwm_reg->CMPB1);
    p_pwm->retention[8] = READ_REG(p_pwm_reg->CMPC0);
    p_pwm->retention[9] = READ_REG(p_pwm_reg->CMPC1);
    p_pwm->retention[10] = READ_REG(p_pwm_reg->AQCTRL);

    if (p_pwm->p_instance == PWM0)
    {
        p_pwm->retention[12] = READ_REG(p_pwm_reg->PRD_CYCLES);
        p_pwm->retention[13] = READ_REG(p_pwm_reg->WAIT_TIME);
        p_pwm->retention[14] = READ_REG(p_pwm_reg->DATA_WIDTH_VALID);
    }
#else
    UNUSED(p_pwm);
#endif
}

__WEAK void hal_pwm_resume_reg(const pwm_handle_t *p_pwm)
{
#ifndef HAL_HW_RES_SUSP_PWM
    pwm_regs_t *p_pwm_reg = p_pwm->p_instance;

    WRITE_REG(p_pwm_reg->MODE, p_pwm->retention[0]);
    WRITE_REG(p_pwm_reg->PRD, p_pwm->retention[1]);
    WRITE_REG(p_pwm_reg->BRPRD, p_pwm->retention[2]);
    MODIFY_REG(p_pwm_reg->HOLD, PWM_HOLD_HOLD, (p_pwm->retention[3] & PWM_HOLD_HOLD_MSK));
    WRITE_REG(p_pwm_reg->CMPA0, p_pwm->retention[4]);
    WRITE_REG(p_pwm_reg->CMPA1, p_pwm->retention[5]);
    WRITE_REG(p_pwm_reg->CMPB0, p_pwm->retention[6]);
    WRITE_REG(p_pwm_reg->CMPB1, p_pwm->retention[7]);
    WRITE_REG(p_pwm_reg->CMPC0, p_pwm->retention[8]);
    WRITE_REG(p_pwm_reg->CMPC1, p_pwm->retention[9]);
    MODIFY_REG(p_pwm_reg->AQCTRL, PWM_AQCTRL_A0, (p_pwm->retention[10] & PWM_AQCTRL_A0_MSK));
    MODIFY_REG(p_pwm_reg->AQCTRL, PWM_AQCTRL_A1, (p_pwm->retention[10] & PWM_AQCTRL_A1_MSK));
    MODIFY_REG(p_pwm_reg->AQCTRL, PWM_AQCTRL_B0, (p_pwm->retention[10] & PWM_AQCTRL_B0_MSK));
    MODIFY_REG(p_pwm_reg->AQCTRL, PWM_AQCTRL_B1, (p_pwm->retention[10] & PWM_AQCTRL_B1_MSK));
    MODIFY_REG(p_pwm_reg->AQCTRL, PWM_AQCTRL_C0, (p_pwm->retention[10] & PWM_AQCTRL_C0_MSK));
    MODIFY_REG(p_pwm_reg->AQCTRL, PWM_AQCTRL_C1, (p_pwm->retention[10] & PWM_AQCTRL_C1_MSK));

    if (p_pwm->p_instance == PWM0)
    {
        WRITE_REG(p_pwm_reg->PRD_CYCLES, p_pwm->retention[12]);
        WRITE_REG(p_pwm_reg->WAIT_TIME, p_pwm->retention[13]);
        WRITE_REG(p_pwm_reg->DATA_WIDTH_VALID, p_pwm->retention[14]);
    }
#else
    UNUSED(p_pwm);
#endif
}

#ifdef HAL_PM_ENABLE
__WEAK hal_pm_status_t hal_pm_pwm_suspend(pwm_handle_t *p_pwm)
{
    hal_pm_status_t ret;
    hal_pwm_state_t state;
    state = hal_pwm_get_state(p_pwm);
    if ((state != HAL_PWM_STATE_READY) && (state != HAL_PWM_STATE_RESET))
    {
        ret = HAL_PM_ACTIVE;
    }
    else
    {
        hal_pwm_suspend_reg(p_pwm);
        ret = HAL_PM_SLEEP;
    }
    return ret;
}

__WEAK void hal_pm_pwm_resume(const pwm_handle_t *p_pwm)
{
    hal_pwm_resume_reg(p_pwm);
}
#endif /* HAL_PM_ENABLE */

__WEAK hal_status_t hal_pwm_start(pwm_handle_t *p_pwm)
{
    hal_status_t status    = HAL_OK;

    if (HAL_PWM_STATE_READY == p_pwm->state)
    {
        /* Update PWM state */
        pwm_set_device_state(p_pwm, HAL_PWM_STATE_BUSY);

        if(0U != p_pwm->retention[11])
        {
            resume_stop_level_cfg(p_pwm);
        }

        /* Sync-all update enable */
        ll_pwm_enable_update_all(p_pwm->p_instance);

        __HAL_PWM_ENABLE(p_pwm);
    }
    else
    {
        status = HAL_BUSY;
    }

    return status;
}

__WEAK hal_status_t hal_pwm_stop(pwm_handle_t *p_pwm)
{
    hal_status_t status    = HAL_OK;

    if (HAL_PWM_STATE_BUSY != p_pwm->state)
    {
        status = HAL_ERROR;
    }
    else
    {
        /* Sync-all update disable */
        ll_pwm_disable_update_all(p_pwm->p_instance);

        __HAL_PWM_DISABLE(p_pwm);

        save_stop_level_cfg(p_pwm);

        /* Change the PWM state */
        pwm_set_device_state(p_pwm, HAL_PWM_STATE_READY);
    }

    return status;
}

__WEAK hal_status_t hal_pwm_resume(pwm_handle_t *p_pwm)
{
    hal_status_t status    = HAL_OK;

    if (HAL_PWM_STATE_READY == p_pwm->state)
    {
        /* Update PWM state */
        pwm_set_device_state(p_pwm, HAL_PWM_STATE_BUSY);

        if(0U != p_pwm->retention[11])
        {
            resume_stop_level_cfg(p_pwm);
        }

        /* Sync-all update enable */
        ll_pwm_enable_update_all(p_pwm->p_instance);

        ll_pwm_disable_pause(p_pwm->p_instance);
    }
    else
    {
        status = HAL_BUSY;
    }

    return status;
}

__WEAK hal_status_t hal_pwm_pause(pwm_handle_t *p_pwm)
{
    hal_status_t status    = HAL_OK;

    if (HAL_PWM_STATE_BUSY != p_pwm->state)
    {
        status = HAL_ERROR;
    }
    else
    {
        /* Sync-all update disable */
        ll_pwm_disable_update_all(p_pwm->p_instance);

        ll_pwm_enable_pause(p_pwm->p_instance);

        save_stop_level_cfg(p_pwm);

        /* Change the PWM state */
        pwm_set_device_state(p_pwm, HAL_PWM_STATE_READY);
    }

    return status;
}

__WEAK hal_status_t hal_pwm_update_freq(pwm_handle_t *p_pwm, uint32_t freq)
{
    hal_status_t status    = HAL_OK;

    if (HAL_PWM_STATE_RESET == p_pwm->state)
    {
        status = HAL_ERROR;
    }
    else
    {
        /* Update SystemCoreClock */
        SystemCoreUpdateClock();

        /* Check the freq value */
        if ((0U == freq) || ((SystemCoreClock >> 1U) < freq))
        {
            status = HAL_ERROR;
            goto EXIT;
        }

        p_pwm->init.none_coding_mode_cfg.freq = freq;

        if(p_pwm->init.none_coding_mode_cfg.align == PWM_ALIGNED_EDGE)
        {
            ll_pwm_set_compare_a0(p_pwm->p_instance, 0);
            ll_pwm_set_compare_a1(p_pwm->p_instance, SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq * p_pwm->init.none_coding_mode_cfg.channel_a.duty / 100U);
            ll_pwm_set_compare_b0(p_pwm->p_instance, 0);
            ll_pwm_set_compare_b1(p_pwm->p_instance, SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq * p_pwm->init.none_coding_mode_cfg.channel_b.duty / 100U);
            ll_pwm_set_compare_c0(p_pwm->p_instance, 0);
            ll_pwm_set_compare_c1(p_pwm->p_instance, SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq * p_pwm->init.none_coding_mode_cfg.channel_c.duty / 100U);
        }
        else /* (p_pwm->init.none_coding_mode_cfg.align == PWM_ALIGNED_CENTER) */
        {
            uint8_t max_duty = (p_pwm->init.none_coding_mode_cfg.channel_a.duty >= p_pwm->init.none_coding_mode_cfg.channel_b.duty) ?
                       p_pwm->init.none_coding_mode_cfg.channel_a.duty : p_pwm->init.none_coding_mode_cfg.channel_b.duty;
            max_duty = (p_pwm->init.none_coding_mode_cfg.channel_c.duty >= max_duty) ?
                       p_pwm->init.none_coding_mode_cfg.channel_c.duty : max_duty;

            if(max_duty > p_pwm->init.none_coding_mode_cfg.channel_a.duty)
            {
                ll_pwm_set_compare_a0(p_pwm->p_instance, (SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq * max_duty / 100U / 2U) -
                                      ((SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq) * p_pwm->init.none_coding_mode_cfg.channel_a.duty / 100U / 2U));
                ll_pwm_set_compare_a1(p_pwm->p_instance, (SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq * max_duty / 100U / 2U) +
                                      ((SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq) * p_pwm->init.none_coding_mode_cfg.channel_a.duty / 100U / 2U));
            }
            else
            {
                ll_pwm_set_compare_a0(p_pwm->p_instance, 0);
                ll_pwm_set_compare_a1(p_pwm->p_instance, (SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq) * p_pwm->init.none_coding_mode_cfg.channel_a.duty / 100U);
            }

            if(max_duty > p_pwm->init.none_coding_mode_cfg.channel_b.duty)
            {
                ll_pwm_set_compare_b0(p_pwm->p_instance, ((SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq) * max_duty / 100U / 2U) -
                                      ((SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq) * p_pwm->init.none_coding_mode_cfg.channel_b.duty / 100U / 2U));
                ll_pwm_set_compare_b1(p_pwm->p_instance, (SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq * max_duty / 100U / 2U) +
                                      ((SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq) * p_pwm->init.none_coding_mode_cfg.channel_b.duty / 100U / 2U));
            }
            else
            {
                ll_pwm_set_compare_b0(p_pwm->p_instance, 0);
                ll_pwm_set_compare_b1(p_pwm->p_instance, SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq * p_pwm->init.none_coding_mode_cfg.channel_b.duty / 100U);
            }

            if(max_duty > p_pwm->init.none_coding_mode_cfg.channel_c.duty)
            {
                ll_pwm_set_compare_c0(p_pwm->p_instance, (SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq * max_duty / 100U / 2U) -
                                      ((SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq) * p_pwm->init.none_coding_mode_cfg.channel_c.duty / 100U / 2U));
                ll_pwm_set_compare_c1(p_pwm->p_instance, (SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq * max_duty / 100U / 2U) +
                                      ((SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq) * p_pwm->init.none_coding_mode_cfg.channel_c.duty / 100U / 2U));
            }
            else
            {
                ll_pwm_set_compare_c0(p_pwm->p_instance, 0);
                ll_pwm_set_compare_c1(p_pwm->p_instance, SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq * p_pwm->init.none_coding_mode_cfg.channel_c.duty / 100U);
            }
        }

        ll_pwm_set_prescaler(p_pwm->p_instance, SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq);
    }
    EXIT:
    return status;
}

__WEAK hal_status_t hal_pwm_config_channel(pwm_handle_t *p_pwm, const pwm_none_coding_channel_init_t *p_config, hal_pwm_active_channel_t channel)
{
    hal_status_t status    = HAL_OK;

    if ( (HAL_PWM_STATE_RESET == p_pwm->state) || (100U < p_config->duty))
    {
        status = HAL_ERROR;
    }
    else
    {
        /* Update Channal Config */
        if ((uint32_t)channel & (uint32_t)HAL_PWM_ACTIVE_CHANNEL_A)
        {
            p_pwm->init.none_coding_mode_cfg.channel_a.duty = p_config->duty;
            ll_pwm_set_action_event_cmp_a0(p_pwm->p_instance, LL_PWM_ACTIONEVENT_SET);
            ll_pwm_set_action_event_cmp_a1(p_pwm->p_instance, LL_PWM_ACTIONEVENT_CLEAR);
            if (LL_PWM_DRIVEPOLARITY_POSITIVE == p_config->drive_polarity)
            {
                ll_pwm_enable_positive_drive_channel_a(p_pwm->p_instance);
            }
            else
            {
                ll_pwm_disable_positive_drive_channel_a(p_pwm->p_instance);
            }
            ll_pwm_set_flicker_stop_level_a(p_pwm->p_instance, p_config->fstoplvl);
        }
        if ((uint32_t)channel & (uint32_t)HAL_PWM_ACTIVE_CHANNEL_B)
        {
            p_pwm->init.none_coding_mode_cfg.channel_b.duty = p_config->duty;
            ll_pwm_set_action_event_cmp_b0(p_pwm->p_instance, LL_PWM_ACTIONEVENT_SET);
            ll_pwm_set_action_event_cmp_b1(p_pwm->p_instance, LL_PWM_ACTIONEVENT_CLEAR);
            if (LL_PWM_DRIVEPOLARITY_POSITIVE == p_config->drive_polarity)
            {
                ll_pwm_enable_positive_drive_channel_b(p_pwm->p_instance);
            }
            else
            {
                ll_pwm_disable_positive_drive_channel_b(p_pwm->p_instance);
            }
            ll_pwm_set_flicker_stop_level_b(p_pwm->p_instance, p_config->fstoplvl);
        }
        if ((uint32_t)channel & (uint32_t)HAL_PWM_ACTIVE_CHANNEL_C)
        {
            p_pwm->init.none_coding_mode_cfg.channel_c.duty = p_config->duty;
            ll_pwm_set_action_event_cmp_c0(p_pwm->p_instance, LL_PWM_ACTIONEVENT_SET);
            ll_pwm_set_action_event_cmp_c1(p_pwm->p_instance, LL_PWM_ACTIONEVENT_CLEAR);
            if (LL_PWM_DRIVEPOLARITY_POSITIVE == p_config->drive_polarity)
            {
                ll_pwm_enable_positive_drive_channel_c(p_pwm->p_instance);
            }
            else
            {
                ll_pwm_disable_positive_drive_channel_c(p_pwm->p_instance);
            }
            ll_pwm_set_flicker_stop_level_c(p_pwm->p_instance, p_config->fstoplvl);
        }

        /* Update SystemCoreClock */
        SystemCoreUpdateClock();

        /* Update Compare Registers */
        if (p_pwm->init.none_coding_mode_cfg.align == PWM_ALIGNED_EDGE)
        {
            if ((uint32_t)channel & (uint32_t)HAL_PWM_ACTIVE_CHANNEL_A)
            {
                ll_pwm_set_compare_a0(p_pwm->p_instance, 0);
                ll_pwm_set_compare_a1(p_pwm->p_instance, SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq * p_pwm->init.none_coding_mode_cfg.channel_a.duty / 100U);
            }
            if ((uint32_t)channel & (uint32_t)HAL_PWM_ACTIVE_CHANNEL_B)
            {
                ll_pwm_set_compare_b0(p_pwm->p_instance, 0);
                ll_pwm_set_compare_b1(p_pwm->p_instance, SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq * p_pwm->init.none_coding_mode_cfg.channel_b.duty / 100U);
            }
            if ((uint32_t)channel & (uint32_t)HAL_PWM_ACTIVE_CHANNEL_C)
            {
                ll_pwm_set_compare_c0(p_pwm->p_instance, 0);
                ll_pwm_set_compare_c1(p_pwm->p_instance, SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq * p_pwm->init.none_coding_mode_cfg.channel_c.duty / 100U);
            }
        }
        else /* (p_pwm->init.none_coding_mode_cfg.align == PWM_ALIGNED_CENTER)*/
        {
            uint8_t max_duty = (p_pwm->init.none_coding_mode_cfg.channel_a.duty >= p_pwm->init.none_coding_mode_cfg.channel_b.duty) ?
                       p_pwm->init.none_coding_mode_cfg.channel_a.duty : p_pwm->init.none_coding_mode_cfg.channel_b.duty;
            max_duty = (p_pwm->init.none_coding_mode_cfg.channel_c.duty >= max_duty) ? p_pwm->init.none_coding_mode_cfg.channel_c.duty : max_duty;

            if(max_duty > p_pwm->init.none_coding_mode_cfg.channel_a.duty)
            {
                ll_pwm_set_compare_a0(p_pwm->p_instance, (SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq * max_duty / 100U / 2U) -
                                      (SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq * p_pwm->init.none_coding_mode_cfg.channel_a.duty / 100U / 2U));
                ll_pwm_set_compare_a1(p_pwm->p_instance, (SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq * max_duty / 100U / 2U) +
                                      (SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq * p_pwm->init.none_coding_mode_cfg.channel_a.duty / 100U / 2U));
            }
            else
            {
                ll_pwm_set_compare_a0(p_pwm->p_instance, 0);
                ll_pwm_set_compare_a1(p_pwm->p_instance, SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq * p_pwm->init.none_coding_mode_cfg.channel_a.duty / 100U);
            }

           if(max_duty > p_pwm->init.none_coding_mode_cfg.channel_b.duty)
            {
                ll_pwm_set_compare_b0(p_pwm->p_instance, (SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq * max_duty / 100U / 2U) -
                                      (SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq * p_pwm->init.none_coding_mode_cfg.channel_b.duty / 100U / 2U));
                ll_pwm_set_compare_b1(p_pwm->p_instance, (SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq * max_duty / 100U / 2U) +
                                      (SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq * p_pwm->init.none_coding_mode_cfg.channel_b.duty / 100U / 2U));
            }
            else
            {
                ll_pwm_set_compare_b0(p_pwm->p_instance, 0);
                ll_pwm_set_compare_b1(p_pwm->p_instance, SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq * p_pwm->init.none_coding_mode_cfg.channel_b.duty / 100U);
            }

            if(max_duty > p_pwm->init.none_coding_mode_cfg.channel_c.duty)
            {
                ll_pwm_set_compare_c0(p_pwm->p_instance, (SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq * max_duty / 100U / 2U) -
                                      (SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq * p_pwm->init.none_coding_mode_cfg.channel_c.duty / 100U / 2U));
                ll_pwm_set_compare_c1(p_pwm->p_instance, (SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq * max_duty / 100U / 2U) +
                                      (SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq * p_pwm->init.none_coding_mode_cfg.channel_c.duty / 100U / 2U));
            }
            else
            {
                ll_pwm_set_compare_c0(p_pwm->p_instance, 0);
                ll_pwm_set_compare_c1(p_pwm->p_instance, SystemCoreClock / p_pwm->init.none_coding_mode_cfg.freq * p_pwm->init.none_coding_mode_cfg.channel_c.duty / 100U);
            }
        }

        save_stop_level_cfg(p_pwm);
    }

    return status;
}

__WEAK hal_status_t hal_pwm_inactive_channel(const pwm_handle_t *p_pwm, hal_pwm_active_channel_t channel)
{
    hal_status_t status    = HAL_OK;

    if (HAL_PWM_STATE_RESET == p_pwm->state)
    {
        status = HAL_ERROR;
    }
    else
    {
        if ((uint32_t)channel & (uint32_t)HAL_PWM_ACTIVE_CHANNEL_A)
        {
            ll_pwm_set_action_event_cmp_a0(p_pwm->p_instance, LL_PWM_ACTIONEVENT_CLEAR);
            ll_pwm_set_action_event_cmp_a1(p_pwm->p_instance, LL_PWM_ACTIONEVENT_NONE);
        }

        if ((uint32_t)channel & (uint32_t)HAL_PWM_ACTIVE_CHANNEL_B)
        {
            ll_pwm_set_action_event_cmp_b0(p_pwm->p_instance, LL_PWM_ACTIONEVENT_CLEAR);
            ll_pwm_set_action_event_cmp_b1(p_pwm->p_instance, LL_PWM_ACTIONEVENT_NONE);
        }

        if ((uint32_t)channel & (uint32_t)HAL_PWM_ACTIVE_CHANNEL_C)
        {
            ll_pwm_set_action_event_cmp_c0(p_pwm->p_instance, LL_PWM_ACTIONEVENT_CLEAR);
            ll_pwm_set_action_event_cmp_c1(p_pwm->p_instance, LL_PWM_ACTIONEVENT_NONE);
        }
    }

    return status;
}

hal_status_t hal_pwm_set_coding_data_in_one_channel(const pwm_handle_t *p_pwm, uint32_t coding_data)
{
    hal_status_t status    = HAL_OK;

    if (HAL_PWM_STATE_RESET == p_pwm->state)
    {
        status = HAL_ERROR;
    }
    else
    {
        /* Update SystemCoreClock */
        SystemCoreUpdateClock();

        ll_pwm_set_coding_data(p_pwm->p_instance, coding_data);
    }

    return status;
}

hal_status_t hal_pwm_set_coding_data_in_three_channels(const pwm_handle_t *p_pwm, uint32_t coding_data0, uint32_t coding_data1, uint32_t coding_data2)
{
    hal_status_t status    = HAL_OK;

    if (HAL_PWM_STATE_RESET == p_pwm->state)
    {
        status = HAL_ERROR;
    }
    else
    {
        /* Update SystemCoreClock */
        SystemCoreUpdateClock();

        ll_pwm_set_coding_data(p_pwm->p_instance, coding_data0);
        ll_pwm_set_coding_data(p_pwm->p_instance, coding_data1);
        ll_pwm_set_coding_data(p_pwm->p_instance, coding_data2);
    }

    return status;
}

hal_status_t hal_pwm_start_coding_with_dma(pwm_handle_t *p_pwm, uint32_t *p_data, uint16_t size)
{
    hal_status_t status    = HAL_OK;

    if (HAL_PWM_STATE_RESET == p_pwm->state)
    {
        status = HAL_ERROR;
    }
    else
    {
        pwm_set_device_state(p_pwm, HAL_PWM_STATE_BUSY);

        if(p_pwm->init.coding_mode_cfg.coding_channel_select == PWM_CODING_CHANNEL_A)
        {
            ll_dma_set_source_burst_length(p_pwm->p_dma->p_instance, p_pwm->p_dma->channel , LL_DMA_SRC_BURST_LENGTH_1);
            ll_dma_set_destination_burst_length(p_pwm->p_dma->p_instance, p_pwm->p_dma->channel, LL_DMA_DST_BURST_LENGTH_1);
        }
        else
        {
            ll_dma_set_source_burst_length(p_pwm->p_dma->p_instance, p_pwm->p_dma->channel , LL_DMA_SRC_BURST_LENGTH_4);
            ll_dma_set_destination_burst_length(p_pwm->p_dma->p_instance, p_pwm->p_dma->channel, LL_DMA_DST_BURST_LENGTH_4);
        }

        p_pwm->p_dma->xfer_tfr_callback = pwm_dma_transmit_cplt;
        p_pwm->p_dma->xfer_error_callback = pwm_dma_error;
        p_pwm->p_dma->xfer_abort_callback = NULL;

        status = hal_dma_start_it(p_pwm->p_dma, (uint32_t )p_data, ll_pwm_dma_get_register_address(p_pwm->p_instance), size);

        ll_pwm_set_dma_enable(p_pwm->p_instance);
        __HAL_PWM_ENABLE(p_pwm);
    }

    return status;
}

hal_status_t hal_pwm_start_coding_in_one_channel(pwm_handle_t *p_pwm, uint32_t coding_data)
{
    hal_status_t status    = HAL_OK;

    if (HAL_PWM_STATE_RESET == p_pwm->state)
    {
        status = HAL_ERROR;
    }
    else
    {
        pwm_set_device_state(p_pwm, HAL_PWM_STATE_BUSY);

        /* Update SystemCoreClock */
        SystemCoreUpdateClock();

        ll_pwm_set_coding_data(p_pwm->p_instance, coding_data);
        __HAL_PWM_ENABLE(p_pwm);
    }

    return status;
}

hal_status_t hal_pwm_start_coding_in_three_channels(pwm_handle_t *p_pwm, uint32_t coding_data0, uint32_t coding_data1, uint32_t coding_data2)
{
    hal_status_t status    = HAL_OK;

    if (HAL_PWM_STATE_RESET == p_pwm->state)
    {
        status = HAL_ERROR;
    }
    else
    {
        pwm_set_device_state(p_pwm, HAL_PWM_STATE_BUSY);

        /* Update SystemCoreClock */
        SystemCoreUpdateClock();

        ll_pwm_set_coding_data(p_pwm->p_instance, coding_data0);
        ll_pwm_set_coding_data(p_pwm->p_instance, coding_data1);
        ll_pwm_set_coding_data(p_pwm->p_instance, coding_data2);
        __HAL_PWM_ENABLE(p_pwm);
    }

    return status;
}

__WEAK void hal_pwm_channel_a_error_callback(pwm_handle_t *p_pwm)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_pwm);
}

__WEAK void hal_pwm_channel_b_error_callback(pwm_handle_t *p_pwm)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_pwm);
}

__WEAK void hal_pwm_channel_c_error_callback(pwm_handle_t *p_pwm)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_pwm);
}

__WEAK void hal_pwm_coding_done_callback(pwm_handle_t *p_pwm)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_pwm);
}

__WEAK void hal_pwm_coding_load_callback(pwm_handle_t *p_pwm)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_pwm);
}

__WEAK void hal_pwm_irq_handler(pwm_handle_t *p_pwm)
{
    uint32_t coding_status = ll_pwm_get_coding_status(p_pwm->p_instance);

    if ((coding_status & PWM_CODING_STATUS_CODING_A_ERROR) == (PWM_CODING_STATUS_CODING_A_ERROR))
    {
        ll_pwm_clr_coding_a_error_status(p_pwm->p_instance);
        hal_pwm_channel_a_error_callback(p_pwm);
    }
    if ((coding_status & PWM_CODING_STATUS_CODING_B_ERROR) == (PWM_CODING_STATUS_CODING_B_ERROR))
    {
        ll_pwm_clr_coding_b_error_status(p_pwm->p_instance);
        hal_pwm_channel_b_error_callback(p_pwm);
    }
    if ((coding_status & PWM_CODING_STATUS_CODING_C_ERROR) == (PWM_CODING_STATUS_CODING_C_ERROR))
    {
        ll_pwm_clr_coding_c_error_status(p_pwm->p_instance);
        hal_pwm_channel_c_error_callback(p_pwm);
    }
    if ((coding_status & PWM_CODING_STATUS_CODING_DONE) == (PWM_CODING_STATUS_CODING_DONE))
    {
        ll_pwm_clr_coding_done_status(p_pwm->p_instance);

        /* Sync-all update disable */
        ll_pwm_disable_update_all(p_pwm->p_instance);

        ll_pwm_set_dma_disable(p_pwm->p_instance);
        __HAL_PWM_DISABLE(p_pwm);

        /* Reset DMA PWM request */
        if(HAL_OK == hal_dma_abort_it(p_pwm->p_dma))
        {
            pwm_set_device_state(p_pwm, HAL_PWM_STATE_READY);
        }

        hal_pwm_coding_done_callback(p_pwm);
    }
    if ((coding_status & PWM_CODING_STATUS_CODING_LOAD) == (PWM_CODING_STATUS_CODING_LOAD))
    {
        ll_pwm_clr_coding_load_status(p_pwm->p_instance);
        hal_pwm_coding_load_callback(p_pwm);
    }
}

__WEAK hal_pwm_state_t hal_pwm_get_state(const pwm_handle_t *p_pwm)
{
    return p_pwm->state;
}

static void pwm_set_device_state(pwm_handle_t *p_pwm, hal_pwm_state_t state)
{
    p_pwm->state = state;
}

#endif /* HAL_PWM_MODULE_ENABLED */


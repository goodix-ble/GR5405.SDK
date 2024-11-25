/**
  ****************************************************************************************
  * @file    hal_efuse.c
  * @author  BLE Driver Team
  * @brief   EFUSE HAL module driver.
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

#ifdef HAL_EFUSE_MODULE_ENABLED
#include "hal_def.h"
#include "hal_efuse.h"
#include "hal_clock.h"
#include "hal_cgc.h"
#include <string.h>

#define EFUSE_TIMEOUT_RETRY         400000

//static hal_status_t efuse_wait_flag_state_until_timeout(efuse_handle_t *p_efuse, uint32_t flag, flag_status_t state);

__WEAK hal_status_t hal_efuse_init(efuse_handle_t *p_efuse)
{
    hal_status_t status = HAL_OK;

    if (HAL_EFUSE_STATE_RESET == p_efuse->state)
    {
        /* Enable security blocks clock and Automatic turn off security blocks clock during WFI. */
        ll_cgc_disable_force_off_secu_hclk();
        ll_cgc_disable_wfi_off_secu_hclk();

        ll_cgc_disable_force_off_efuse_hclk();
        ll_cgc_disable_force_off_secu_div4_pclk();
        ll_cgc_disable_wfi_off_efuse_hclk();
        ll_cgc_disable_wfi_off_secu_div4_hclk();

        /* init the low level hardware : GPIO, CLOCK */
        hal_efuse_msp_init(p_efuse);
    }

    p_efuse->state = HAL_EFUSE_STATE_BUSY;

    /* Open VDD. VDD must be enble when read/write efuse */
    ll_efuse_enable_power(p_efuse->p_instance);

    /* Set EFUSE error code to none */
    p_efuse->error_code = HAL_EFUSE_ERROR_NONE;

    /* Initialize the EFUSE state */
    p_efuse->state = HAL_EFUSE_STATE_READY;

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_efuse_deinit(efuse_handle_t *p_efuse)
{
    /* Enable EFUSE Clock for operate the register. */
    ll_cgc_disable_force_off_secu_hclk();

    ll_cgc_disable_force_off_efuse_hclk();
    ll_cgc_disable_force_off_secu_div4_pclk();

    /* DeInit the low level hardware: GPIO, CLOCK... */
    hal_efuse_msp_deinit(p_efuse);

    ll_efuse_disable_power(p_efuse->p_instance);

    ll_cgc_enable_force_off_efuse_hclk();
    ll_cgc_enable_wfi_off_efuse_hclk();

    GLOBAL_EXCEPTION_DISABLE();
    if (ll_cgc_get_force_off_secu() == LL_CGC_MCU_SECU_FRC_OFF_HCLK)
    {
        ll_cgc_enable_force_off_secu_hclk();
        ll_cgc_enable_force_off_secu_div4_pclk();
    }
    if (ll_cgc_get_slp_off_secu() == LL_CGC_MCU_SECU_FRC_OFF_WFI_HCLK)
    {
        ll_cgc_enable_wfi_off_secu_hclk();
        ll_cgc_enable_wfi_off_secu_div4_hclk();
    }
    GLOBAL_EXCEPTION_ENABLE();

    /* Set EFUSE error code to none */
    p_efuse->error_code = HAL_EFUSE_ERROR_NONE;

    /* Initialize the EFUSE state */
    p_efuse->state = HAL_EFUSE_STATE_RESET;

    return HAL_OK;
}

__WEAK void hal_efuse_msp_init(efuse_handle_t *p_efuse)
{
    UNUSED(p_efuse);
    /* Prevent unused argument(s) compilation warning */
    return;
}

__WEAK void hal_efuse_msp_deinit(efuse_handle_t *p_efuse)
{
    UNUSED(p_efuse);
    /* Prevent unused argument(s) compilation warning */
    return;
}

__WEAK hal_status_t hal_efuse_write(efuse_handle_t *p_efuse, uint32_t word_offset, const uint32_t *p_data, uint32_t nword)
{
    hal_status_t status = HAL_OK;
    //lint -e923 Cast from pointer to unsigned int is necessary
    volatile uint32_t *p_origin = (volatile uint32_t *)(EFUSE_STORAGE_BASE);
    volatile uint32_t *pefuse;
    uint32_t count = 0;

    if (((word_offset + nword) > 0x80U) || (NULL == p_data) || (0U == nword))
    {
        p_efuse->error_code |= HAL_EFUSE_ERROR_INVALID_PARAM;
        status = HAL_ERROR;
        goto EXIT;
    }

    pefuse = &p_origin[word_offset];

    if (HAL_EFUSE_STATE_READY == p_efuse->state)
    {
        p_efuse->error_code = HAL_EFUSE_ERROR_NONE;
        p_efuse->state = HAL_EFUSE_STATE_BUSY;

        /* Set program time, and config PGENB to 0 */
        ll_efuse_set_tpro(p_efuse->p_instance, 0x50);
        ll_efuse_disable_pgenb(p_efuse->p_instance);

        /* Open VDD and VDDQ by HW */
        //ll_efuse_set_controller_power_timing(p_efuse->p_instance, 0x10, 0x13, 0x16);
        /* Enable EFUSE HW auto power control */
        ll_efuse_enable_controller_power_en(p_efuse->p_instance);
        /* EFUSE HW auto power control begin */
        ll_efuse_enable_controller_power_begin(p_efuse->p_instance);
        while(ll_efuse_is_controller_power_flag(p_efuse->p_instance, LL_EFUSE_PWR_CTL_EN_DONE))
        {
        }

        /* Wait for more than 1ms */
        for (count = 0; count < 8000U; count++)
        {
            __asm("nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n");
        }

        for (count = 0; count < nword; count++)
        {
            pefuse[count] = p_data[count];
            while(ll_efuse_is_active_flag(p_efuse->p_instance, LL_EFUSE_WRITE_DONE) == 0U)
            {
            }
        }

        /* EFUSE HW auto power control stop */
        ll_efuse_enable_controller_power_stop(p_efuse->p_instance);
        while(ll_efuse_is_controller_power_flag(p_efuse->p_instance, LL_EFUSE_PWR_CTL_DIS_DONE))
        {
        }

        /* Disable EFUSE HW auto power control */
        ll_efuse_disable_controller_power(p_efuse->p_instance);

        /* Config PGENB to 1 */
        ll_efuse_enable_pgenb(p_efuse->p_instance);

        p_efuse->state = HAL_EFUSE_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }
    EXIT:
    return status;
}

__WEAK hal_status_t hal_efuse_read(efuse_handle_t *p_efuse, uint32_t word_offset, uint32_t *p_data, uint32_t nword)
{
    hal_status_t status = HAL_OK;
    //lint -e923 Cast from pointer to unsigned int is necessary
    const volatile uint32_t *p_origin = (volatile uint32_t *)(EFUSE_STORAGE_BASE);
    const volatile uint32_t *pefuse;
    uint32_t count = 0;

    if (((word_offset + nword) > EFUSE_ZONE_SIZE_WORD) || (NULL == p_data) || (0U == nword))
    {
        p_efuse->error_code |= HAL_EFUSE_ERROR_INVALID_PARAM;
        status = HAL_ERROR;
        goto EXIT;
    }

    pefuse = &p_origin[word_offset];

    if (HAL_EFUSE_STATE_READY == p_efuse->state)
    {
        p_efuse->error_code = HAL_EFUSE_ERROR_NONE;
        p_efuse->state = HAL_EFUSE_STATE_BUSY;

        /* EFUSE reading need close HW auto power control and VDD = 1, PEGNB = 1*/
        ll_efuse_disable_controller_power(p_efuse->p_instance);

        ll_efuse_enable_pgenb(p_efuse->p_instance);

        for (count = 0; count < nword; count++)
        {
            p_data[count] = pefuse[count];
        }

        p_efuse->state = HAL_EFUSE_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }
EXIT:
    return status;
}

__WEAK hal_status_t hal_efuse_initial_value_check(efuse_handle_t *p_efuse)
{
    hal_status_t status = HAL_OK;
    uint32_t flag = 0, retry = EFUSE_TIMEOUT_RETRY;

    if (HAL_EFUSE_STATE_READY == p_efuse->state)
    {
        p_efuse->error_code = HAL_EFUSE_ERROR_NONE;
        p_efuse->state      = HAL_EFUSE_STATE_BUSY;

        ll_efuse_set_operation(p_efuse->p_instance, LL_EFUSE_INIT_CHECK);
        do {
            if (0U == retry--)
            {
                status = HAL_ERROR;
                p_efuse->error_code = HAL_EFUSE_ERROR_TIMEOUT;
                break;
            }
            flag = p_efuse->p_instance->STAT;
        } while((flag & EFUSE_FLAG_INIT_CHECK_DONE) != EFUSE_FLAG_INIT_CHECK_DONE);

        if (HAL_OK == status)
        {
            if ( (flag & EFUSE_FLAG_INIT_CHECK_SUCCESS) != EFUSE_FLAG_INIT_CHECK_SUCCESS)
            {
                status = HAL_ERROR;
            }
        }

        p_efuse->state = HAL_EFUSE_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    return status;
}

#endif /* HAL_EFUSE_MODULE_ENABLED */


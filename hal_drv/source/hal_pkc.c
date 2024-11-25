/**
  ****************************************************************************************
  * @file    gr5xx_hal_pkc.c
  * @author  BLE Driver Team
  * @brief   PKC HAL module driver.
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

//lint -e9058 [advisory] tag unused
#ifdef HAL_PKC_MODULE_ENABLED

#include "hal_def.h"
#include "hal_pkc.h"
#include "ll_misc.h"
#include <string.h>

#ifdef HAL_CLOCK_UNIFORM_CONTROL
#include "hal_clock.h"
#endif

typedef struct _pkc_sw_operation_init
{
    uint32_t op_mode;
    uint32_t enable_it;
    uint32_t ptrA;
    uint32_t ptrB;
    uint32_t ptrC;
    uint32_t ptrP;
    uint32_t timeout;
} pkc_sw_operation_init_t;

typedef struct _ecc_point_multi_init
{
    uint32_t enable_it;
    pkc_ecc_point_multi_t *p_input;
    ecc_point_t *p_result;
    uint32_t timeout;
} ecc_point_multi_init_t;

static hal_status_t pkc_wait_flag_state_until_timeout(pkc_handle_t *p_pkc, uint32_t flag, flag_status_t state, uint32_t timeout);
static void pkc_read_spram(pkc_handle_t *p_pkc, uint32_t *p_data, uint32_t ptr);
static void pkc_write_spram(pkc_handle_t *p_pkc, uint32_t *p_data, uint32_t ptr);
static uint32_t pkc_compare_data(uint32_t *p_data_a, uint32_t *p_data_b, uint32_t data_bits);
static hal_status_t pkc_modular_compare_operation(pkc_handle_t *p_pkc, uint32_t *p_data_a, uint32_t *p_data_b, uint32_t data_bits);
static uint32_t pkc_find_msb(uint32_t *p_data, uint32_t data_bits);
static uint32_t pkc_get_bit(uint32_t *p_data, uint32_t ofs, uint32_t data_bits);
static int32_t pkc_compare_const(uint32_t *p_data, uint32_t Const, uint32_t data_bits);
static uint32_t ecc_is_infinite_point(ecc_point_t *p_Piont);
static hal_status_t pkc_software_operation(pkc_handle_t *p_pkc, pkc_sw_operation_init_t *p_input);
static hal_status_t ecc_point_multiply(pkc_handle_t *p_pkc, ecc_point_multi_init_t *p_input);

__WEAK hal_status_t hal_pkc_init(pkc_handle_t *p_pkc)
{
    hal_status_t   status    = HAL_ERROR;

    /* Check the PKC handle allocation */
    if (NULL == p_pkc)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    gr_assert_param(IS_PKC_ALL_INSTANCE(p_pkc->p_instance));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    if (HAL_PKC_STATE_RESET == p_pkc->state)
    {
#ifdef HAL_CLOCK_UNIFORM_CONTROL
        //lint -e923 Cast from pointer to unsigned int is necessary
        hal_clock_enable_module((uint32_t)p_pkc->p_instance);
#endif
        /* Enable security blocks clock and Automatic turn off security blocks clock during WFI. */
//        ll_cgc_disable_force_off_secu_hclk();
//        ll_cgc_disable_wfi_off_secu_hclk();

//        ll_cgc_disable_force_off_pkc_hclk();
//        ll_cgc_disable_force_off_secu_div4_pclk();
//        ll_cgc_disable_wfi_off_pkc_hclk();
//        ll_cgc_disable_wfi_off_secu_div4_hclk();

        /* Init the low level hardware : CLOCK, NVIC */
        hal_pkc_msp_init(p_pkc);
    }

    /* Enable PKC */
    __HAL_PKC_ENABLE(p_pkc);

    /* Set PKC error code to none */
    p_pkc->error_code = HAL_PKC_ERROR_NONE;

    /* Initialize the PKC state */
    p_pkc->state = HAL_PKC_STATE_READY;

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_deinit(pkc_handle_t *p_pkc)
{
    /* Check the PKC handle allocation */
    if (NULL == p_pkc)
    {
        return HAL_ERROR;
    }

    /* Set PKC registers to their reset values */
    ll_pkc_enable(p_pkc->p_instance);
    ll_pkc_disable_reset(p_pkc->p_instance);
    ll_pkc_enable_reset(p_pkc->p_instance);
    ll_pkc_disable(p_pkc->p_instance);

    /* DeInit the low level hardware: CLOCK, NVIC... */
    hal_pkc_msp_deinit(p_pkc);

#ifdef HAL_CLOCK_UNIFORM_CONTROL
    //lint -e923 Cast from pointer to unsigned int is necessary
    hal_clock_disable_module((uint32_t)p_pkc->p_instance);
#endif
//    ll_cgc_enable_force_off_pkc_hclk();
//    ll_cgc_enable_wfi_off_pkc_hclk();

//    GLOBAL_EXCEPTION_DISABLE();
//    if (ll_cgc_get_force_off_secu() == LL_CGC_MCU_SECU_FRC_OFF_HCLK)
//    {
//        ll_cgc_enable_force_off_secu_hclk();
//        ll_cgc_enable_force_off_secu_div4_pclk();
//    }
//    if (ll_cgc_get_slp_off_secu() == LL_CGC_MCU_SECU_FRC_OFF_WFI_HCLK)
//    {
//        ll_cgc_enable_wfi_off_secu_hclk();
//        ll_cgc_enable_wfi_off_secu_div4_hclk();
//    }
//    GLOBAL_EXCEPTION_ENABLE();

    /* Set PKC error code to none */
    p_pkc->error_code = HAL_PKC_ERROR_NONE;

    /* Initialize the PKC state */
    p_pkc->state = HAL_PKC_STATE_RESET;

    return HAL_OK;
}

__WEAK void hal_pkc_msp_init(pkc_handle_t *p_pkc)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_pkc);
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
            the hal_pkc_msp_init can be implemented in the user file
     */
}

__WEAK void hal_pkc_msp_deinit(pkc_handle_t *p_pkc)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_pkc);
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
            the hal_pkc_msp_deinit can be implemented in the user file
     */
}

__WEAK void hal_pkc_irq_handler(pkc_handle_t *p_pkc)
{
    uint32_t itsource = READ_REG(p_pkc->p_instance->INT_STAT);
    uint32_t op_mode, out_ptr = 0U;

    if (itsource & PKC_IT_ERR)
    {
        __HAL_PKC_CLEAR_FLAG_IT(p_pkc, PKC_IT_ERR);

        if (PKC_OPERATION_MODE_CMP == ll_pkc_get_operation_mode(p_pkc->p_instance))
        {
            out_ptr = ll_pkc_get_mas_c_pointer(p_pkc->p_instance);
            pkc_read_spram(p_pkc, p_pkc->p_result, out_ptr);
            if (pkc_compare_data(p_pkc->p_result, p_pkc->p_P, p_pkc->init.data_bits) >= 1U)
            {
                ll_pkc_disable_software_start(p_pkc->p_instance);
                ll_pkc_enable_software_start(p_pkc->p_instance);
            }
            else
            {
                __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE);

                if (HAL_PKC_STATE_BUSY == p_pkc->state)
                {
                    /* Change state of PKC */
                    p_pkc->state = HAL_PKC_STATE_READY;

                    /* Error callback */
                    hal_pkc_done_callback(p_pkc);
                }
            }
        }
        else
        {
            /* Disable all the PKC Interrupts */
            __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_ERR | PKC_IT_OVF | PKC_IT_DONE);

            /* Set error code */
            p_pkc->error_code |= HAL_PKC_ERROR_TRANSFER;

            /* Change state of PKC */
            p_pkc->state = HAL_PKC_STATE_READY;

            /* Error callback */
            hal_pkc_error_callback(p_pkc);
        }
    }

    if (itsource & PKC_IT_OVF)
    {
        __HAL_PKC_CLEAR_FLAG_IT(p_pkc, PKC_IT_OVF);

        __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_OVF);

        /* Set error code */
        p_pkc->error_code |= HAL_PKC_ERROR_OVERFLOW;

        /* Error callback */
        hal_pkc_overflow_callback(p_pkc);
    }

    if (itsource & PKC_IT_DONE)
    {
        __HAL_PKC_CLEAR_FLAG_IT(p_pkc, PKC_IT_DONE);

        if (ll_pkc_is_enabled_software(p_pkc->p_instance))
        {
            bool temp0 = (PKC_OPERATION_MODE_LSHIFT != ll_pkc_get_operation_mode(p_pkc->p_instance));
            bool temp1 = (PKC_OPERATION_MODE_CMP != ll_pkc_get_operation_mode(p_pkc->p_instance));
            bool temp2 = (PKC_OPERATION_MODE_INVER != ll_pkc_get_operation_mode(p_pkc->p_instance));

            if (temp0 && temp1 && temp2)
            {
                __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE);

                op_mode = ll_pkc_get_operation_mode(p_pkc->p_instance);
                switch(op_mode)
                {
                case PKC_OPERATION_MODE_MULTI:
                    out_ptr = ll_pkc_get_mm_c_pointer(p_pkc->p_instance);
                    break;
                case PKC_OPERATION_MODE_ADD:
                case PKC_OPERATION_MODE_SUB:
                    out_ptr = ll_pkc_get_mas_c_pointer(p_pkc->p_instance);
                    break;
                case PKC_OPERATION_MODE_BIGMULTI:
                    out_ptr = ll_pkc_get_bm_c_pointer(p_pkc->p_instance);
                    break;
                case PKC_OPERATION_MODE_BIGADD:
                    out_ptr = ll_pkc_get_ba_c_pointer(p_pkc->p_instance);
                    break;
                default:
                    /* Nothing to do */
                    break;
                }
                pkc_read_spram(p_pkc, p_pkc->p_result, out_ptr);

                if (HAL_PKC_STATE_BUSY == p_pkc->state)
                {
                    /* Change state of PKC */
                    p_pkc->state = HAL_PKC_STATE_READY;

                    /* Error callback */
                    hal_pkc_done_callback(p_pkc);
                }
            }
            else if (PKC_OPERATION_MODE_LSHIFT == ll_pkc_get_operation_mode(p_pkc->p_instance))
            {
                p_pkc->shift_count--;
                if (0U < p_pkc->shift_count)
                {
                    ll_pkc_disable_software_start(p_pkc->p_instance);
                    ll_pkc_enable_software_start(p_pkc->p_instance);
                }
                else
                {
                    __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE);

                    out_ptr = ll_pkc_get_mas_c_pointer(p_pkc->p_instance);
                    pkc_read_spram(p_pkc, p_pkc->p_result, out_ptr);

                    if (HAL_PKC_STATE_BUSY == p_pkc->state)
                    {
                        /* Change state of PKC */
                        p_pkc->state = HAL_PKC_STATE_READY;

                        /* Error callback */
                        hal_pkc_done_callback(p_pkc);
                    }
                }
            }
            else if (PKC_OPERATION_MODE_CMP == ll_pkc_get_operation_mode(p_pkc->p_instance))
            {
                out_ptr = ll_pkc_get_mas_c_pointer(p_pkc->p_instance);
                pkc_read_spram(p_pkc, p_pkc->p_result, out_ptr);
                if (pkc_compare_data(p_pkc->p_result, p_pkc->p_P, p_pkc->init.data_bits) >= 1U)
                {
                    ll_pkc_disable_software_start(p_pkc->p_instance);
                    ll_pkc_enable_software_start(p_pkc->p_instance);
                }
                else
                {
                    __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE);

                    if (HAL_PKC_STATE_BUSY == p_pkc->state)
                    {
                        /* Change state of PKC */
                        p_pkc->state = HAL_PKC_STATE_READY;

                        /* Error callback */
                        hal_pkc_done_callback(p_pkc);
                    }
                }
            }
            else if (PKC_OPERATION_MODE_INVER == ll_pkc_get_operation_mode(p_pkc->p_instance))
            {
                out_ptr        = ll_pkc_get_mi_x1_pointer(p_pkc->p_instance);
                uint32_t temp_k = ll_pkc_get_mik_output(p_pkc->p_instance);
                pkc_read_spram(p_pkc, p_pkc->p_result, out_ptr);
                uint32_t constp = ll_pkc_get_constp(p_pkc->p_instance);

                pkc_sw_operation_init_t pkc_sw_init;
                uint32_t temp[64] = {0};
                uint32_t t_k = (p_pkc->init.data_bits << 1U) - temp_k;
                uint32_t t_pos = ((p_pkc->init.data_bits >> 5U) - 1U) - (t_k / 32U);
                uint32_t t_bit_offset = t_k % 32U;
                temp[t_pos] |= (1U << t_bit_offset);

                pkc_sw_init.ptrC = 0U;
                pkc_sw_init.ptrA = 1U * (p_pkc->init.data_bits >> 5U);
                pkc_sw_init.ptrB = 2U * (p_pkc->init.data_bits >> 5U);
                pkc_sw_init.ptrP = 3U * (p_pkc->init.data_bits >> 5U);

                __HAL_PKC_RESET(p_pkc);

                pkc_write_spram(p_pkc, p_pkc->p_result, pkc_sw_init.ptrA);  // input A
                //lint -e934 Taking address of near auto variable is necessary
                pkc_write_spram(p_pkc, temp, pkc_sw_init.ptrB);             // input B
                pkc_write_spram(p_pkc, p_pkc->p_P, pkc_sw_init.ptrP);       // input P
                ll_pkc_set_constp(p_pkc->p_instance, constp);
                ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

                pkc_sw_init.op_mode   = PKC_OPERATION_MODE_MULTI;
                pkc_sw_init.enable_it = 0U;
                pkc_sw_init.timeout   = HAL_PKC_TIMEOUT_DEFAULT_VALUE;
                hal_status_t status = pkc_software_operation(p_pkc, &pkc_sw_init);
                if (HAL_OK != status)
                {
                   
                }
                pkc_read_spram(p_pkc, p_pkc->p_result,  pkc_sw_init.ptrC);

                __HAL_PKC_RESET(p_pkc);

                temp[t_pos] = 0x00000000U;
                temp[(p_pkc->init.data_bits >> 5U)-1U] = 0x00000001U;
                pkc_write_spram(p_pkc, p_pkc->p_result, pkc_sw_init.ptrA);  // input A
                pkc_write_spram(p_pkc, temp, pkc_sw_init.ptrB);             // input B
                pkc_write_spram(p_pkc, p_pkc->p_P, pkc_sw_init.ptrP);       // input P
                ll_pkc_set_constp(p_pkc->p_instance, constp);
                ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);
                status = pkc_software_operation(p_pkc, &pkc_sw_init);
                if (HAL_OK != status)
                {
                   
                }
                pkc_read_spram(p_pkc, p_pkc->p_result,  pkc_sw_init.ptrC);

                if (HAL_PKC_STATE_BUSY == p_pkc->state)
                {
                    /* Change state of PKC */
                    p_pkc->state = HAL_PKC_STATE_READY;

                    /* Error callback */
                    hal_pkc_done_callback(p_pkc);
                }
            }
            else
            {
                /* Nothing to do */
            }
        }
        else
        {
            __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE);
            //lint -e9087 Cast a pointer to a different object type is necessary
            pkc_read_spram(p_pkc, ((ecc_point_t *)p_pkc->p_result)->X, 0x48U);
            //lint -e9087 Cast a pointer to a different object type is necessary
            pkc_read_spram(p_pkc, ((ecc_point_t *)p_pkc->p_result)->Y, 0x50U);

            if (HAL_PKC_STATE_BUSY == p_pkc->state)
            {
                /* Change state of PKC */
                p_pkc->state = HAL_PKC_STATE_READY;

                /* Error callback */
                hal_pkc_done_callback(p_pkc);
            }
        }
    }
}

__WEAK hal_status_t hal_pkc_rsa_modular_exponent(pkc_handle_t *p_pkc, pkc_rsa_modular_exponent_t *p_input, uint32_t timeout)
{
    hal_status_t            status    = HAL_OK;
    pkc_sw_operation_init_t pkc_sw_init = {0};

    if ((NULL == p_pkc) || (NULL == p_input))
    {
        return HAL_ERROR;
    }

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));


    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        uint32_t i = 0U;
        uint32_t op_out_ptr = 0U;
        uint32_t op_ina_ptr = 1U * (p_pkc->init.data_bits >> 5U);
        uint32_t op_inb_ptr = 2U * (p_pkc->init.data_bits >> 5U);
        uint32_t op_inp_ptr = 3U * (p_pkc->init.data_bits >> 5U);

        do {
            /* Step 1. Find most significant 1 in input b[] */
            i = pkc_find_msb(p_input->p_B, p_pkc->init.data_bits);

            if (1U >= i)
            {
                if (0U == p_input->p_B[(p_pkc->init.data_bits >> 5U) - 1U])
                {
                    memset(p_pkc->p_result, 0, p_pkc->init.data_bits >> 3U);
                    ((uint32_t *)p_pkc->p_result)[(p_pkc->init.data_bits >> 5U) - 1U] = 1U;
                }
                else
                {
                    memcpy(p_pkc->p_result, p_input->p_A, p_pkc->init.data_bits >> 3U);
                }
                break;
            }
            /* Begins hardware computation */
            /* Step 2. Enable PKC and load data */
            __HAL_PKC_RESET(p_pkc);
            __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

            pkc_write_spram(p_pkc, p_input->p_A,  op_ina_ptr);   // input A
            pkc_write_spram(p_pkc, p_input->p_P_R2, op_inb_ptr); // R^2 mod P
            pkc_write_spram(p_pkc, p_input->p_P,  op_inp_ptr);   // input P
            ll_pkc_set_constp(p_pkc->p_instance, p_input->ConstP);
            ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

            /* Step 3. Calculate MM(A, R^2), store in op_out_ptr */
            pkc_sw_init.op_mode   = PKC_OPERATION_MODE_MULTI;
            pkc_sw_init.enable_it = 0U;
            pkc_sw_init.ptrA      = op_ina_ptr;
            pkc_sw_init.ptrB      = op_inb_ptr;
            pkc_sw_init.ptrC      = op_out_ptr;
            pkc_sw_init.ptrP      = op_inp_ptr;
            pkc_sw_init.timeout   = timeout;
            status = pkc_software_operation(p_pkc, &pkc_sw_init);
            if (HAL_OK != status)
            {
                break;
            }

            pkc_read_spram(p_pkc, p_pkc->p_result,  op_out_ptr);
            pkc_write_spram(p_pkc, p_pkc->p_result, op_ina_ptr);

            /* Step 4. if(getbit == 0){x = x^2 } else {x = x^2 * a;} */
            pkc_sw_init.ptrA = op_out_ptr;
            for (i--; i > 0U; i--)
            {
                pkc_sw_init.ptrB = op_out_ptr;
                status = pkc_software_operation(p_pkc, &pkc_sw_init);
                if (1U == pkc_get_bit(p_input->p_B, i - 1U, p_pkc->init.data_bits))
                {
                    pkc_sw_init.ptrB = op_ina_ptr;
                    status = pkc_software_operation(p_pkc, &pkc_sw_init);
                    if (HAL_OK != status)
                    {
                        break;
                    }
                }
            }
            if (HAL_OK != status)
            {
                break;
            }

            /* Step 5. Calculate out = MM(x,1) */
            memset(p_pkc->p_result, 0, p_pkc->init.data_bits >> 3U);
            ((uint32_t *)p_pkc->p_result)[(p_pkc->init.data_bits >> 5U) - 1U] = 1U;
            pkc_write_spram(p_pkc, p_pkc->p_result, op_inb_ptr);
            pkc_sw_init.ptrB = op_inb_ptr;
            status = pkc_software_operation(p_pkc, &pkc_sw_init);
            if (HAL_OK != status)
            {
                break;
            }

            pkc_read_spram(p_pkc, p_pkc->p_result,  op_out_ptr);
        } while(0);

        p_pkc->state = HAL_PKC_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_ecc_point_multi(pkc_handle_t *p_pkc, pkc_ecc_point_multi_t *p_input, uint32_t timeout)
{
    hal_status_t           status    = HAL_OK;
    ecc_point_multi_init_t ecc_point_multi_init = {0};

    if ((NULL == p_pkc) || (NULL == p_input))
    {
        return HAL_ERROR;
    }

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        do {
            __HAL_PKC_ENABLE(p_pkc);
            __HAL_PKC_RESET(p_pkc);
            __HAL_PKC_DISABLE(p_pkc);
            __HAL_PKC_ENABLE(p_pkc);

            if (NULL == p_input->p_ecc_point)
            {
                p_input->p_ecc_point = &p_pkc->init.p_ecc_curve->G;
            }

            if (ecc_is_infinite_point(p_input->p_ecc_point))
            {
                memset(p_pkc->p_result, 0, sizeof(ecc_point_t));
                break;
            }

            if (pkc_compare_const(p_input->p_K, 0U, p_pkc->init.data_bits) == 0)
            {
                memset(p_pkc->p_result, 0, sizeof(ecc_point_t));
                break;
            }

            if (pkc_compare_const(p_input->p_K, 1U, p_pkc->init.data_bits) == 0)
            {
                memcpy(p_pkc->p_result, p_input->p_ecc_point, sizeof(ecc_point_t));
                break;
            }

            ecc_point_multi_init.enable_it = 0U;
            ecc_point_multi_init.p_input   = p_input;
            ecc_point_multi_init.p_result  = p_pkc->p_result;
            ecc_point_multi_init.timeout   = timeout;
            status = ecc_point_multiply(p_pkc, &ecc_point_multi_init);

            pkc_read_spram(p_pkc, ((ecc_point_t *)p_pkc->p_result)->X, 0x48U);
            pkc_read_spram(p_pkc, ((ecc_point_t *)p_pkc->p_result)->Y, 0x50U);
        } while(0);

        p_pkc->state = HAL_PKC_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_ecc_point_multi_it(pkc_handle_t *p_pkc, pkc_ecc_point_multi_t *p_input)
{
    hal_status_t           status    = HAL_OK;
    ecc_point_multi_init_t ecc_point_multi_init = {0};
    uint32_t               callback_flag = 0U;

    if ((NULL == p_pkc) || (NULL == p_input))
    {
        return HAL_ERROR;
    }

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        do {
            __HAL_PKC_ENABLE(p_pkc);
            __HAL_PKC_RESET(p_pkc);
            __HAL_PKC_DISABLE(p_pkc);
            __HAL_PKC_ENABLE(p_pkc);

            if (NULL == p_input->p_ecc_point)
            {
                p_input->p_ecc_point = &p_pkc->init.p_ecc_curve->G;
            }
            else if (ecc_is_infinite_point(p_input->p_ecc_point))
            {
                memset(p_pkc->p_result, 0, sizeof(ecc_point_t));
                callback_flag = 1U;
                break;
            }
            else
            {
                /* Nothing to do */
            }

            if (0 == pkc_compare_const(p_input->p_K, 0U, p_pkc->init.data_bits))
            {
                memset(p_pkc->p_result, 0, sizeof(ecc_point_t));
                callback_flag = 1U;
                break;
            }

            if (0 == pkc_compare_const(p_input->p_K, 1U, p_pkc->init.data_bits))
            {
                memcpy(p_pkc->p_result, p_input->p_ecc_point, sizeof(ecc_point_t));
                callback_flag = 1U;
                break;
            }

            ecc_point_multi_init.enable_it   = 1U;
            ecc_point_multi_init.p_input     = p_input;
            ecc_point_multi_init.p_result    = p_pkc->p_result;
            status = ecc_point_multiply(p_pkc, &ecc_point_multi_init);
        } while(0);

        if (callback_flag)
        {
            /* Update PKC state */
            p_pkc->state = HAL_PKC_STATE_READY;

            hal_pkc_done_callback(p_pkc);
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_modular_add(pkc_handle_t *p_pkc, pkc_modular_add_t *p_input, uint32_t timeout)
{
    hal_status_t            status    = HAL_OK;
    pkc_sw_operation_init_t pkc_sw_init = {0};

    if ((NULL == p_pkc) || (NULL == p_input))
    {
        return HAL_ERROR;
    }

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        __HAL_PKC_RESET(p_pkc);
        __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

        while (pkc_compare_data(p_input->p_A, p_input->p_P, p_pkc->init.data_bits) >= 1U)
        {
            status = pkc_modular_compare_operation(p_pkc, p_input->p_A, p_input->p_P, p_pkc->init.data_bits);
            if(HAL_OK != status)
            {
            
            }
        }

        __HAL_PKC_RESET(p_pkc);
        while (pkc_compare_data(p_input->p_B, p_input->p_P, p_pkc->init.data_bits) >= 1U)
        {
            status = pkc_modular_compare_operation(p_pkc, p_input->p_B, p_input->p_P, p_pkc->init.data_bits);
            if(HAL_OK != status)
            {
            
            }
        }

        pkc_sw_init.ptrC = 0U;
        pkc_sw_init.ptrA = 1U * (p_pkc->init.data_bits >> 5U);
        pkc_sw_init.ptrB = 2U * (p_pkc->init.data_bits >> 5U);
        pkc_sw_init.ptrP = 3U * (p_pkc->init.data_bits >> 5U);

        __HAL_PKC_RESET(p_pkc);

        pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
        pkc_write_spram(p_pkc, p_input->p_B, pkc_sw_init.ptrB);   // input B
        pkc_write_spram(p_pkc, p_input->p_P, pkc_sw_init.ptrP);   // input P
        ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

        pkc_sw_init.op_mode   = PKC_OPERATION_MODE_ADD;
        pkc_sw_init.enable_it = 0U;
        pkc_sw_init.timeout   = timeout;
        status = pkc_software_operation(p_pkc, &pkc_sw_init);

        pkc_read_spram(p_pkc, p_pkc->p_result,  pkc_sw_init.ptrC);

        p_pkc->state = HAL_PKC_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_modular_add_it(pkc_handle_t *p_pkc, pkc_modular_add_t *p_input)
{
    hal_status_t            status    = HAL_OK;
    pkc_sw_operation_init_t pkc_sw_init = {0};

    if ((NULL == p_pkc) || (NULL == p_input))
    {
        return HAL_ERROR;
    }

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        __HAL_PKC_RESET(p_pkc);
        __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

        while (pkc_compare_data(p_input->p_A, p_input->p_P, p_pkc->init.data_bits) >= 1U)
        {
            status = pkc_modular_compare_operation(p_pkc, p_input->p_A, p_input->p_P, p_pkc->init.data_bits);
            if(HAL_OK != status)
            {
            }
        }

        __HAL_PKC_RESET(p_pkc);
        while (pkc_compare_data(p_input->p_B, p_input->p_P, p_pkc->init.data_bits) >= 1U)
        {
            status = pkc_modular_compare_operation(p_pkc, p_input->p_B, p_input->p_P, p_pkc->init.data_bits);
            if(HAL_OK != status)
            {
            }
        }


        pkc_sw_init.ptrC = 0U;
        pkc_sw_init.ptrA = 1U * (p_pkc->init.data_bits >> 5U);
        pkc_sw_init.ptrB = 2U * (p_pkc->init.data_bits >> 5U);
        pkc_sw_init.ptrP = 3U * (p_pkc->init.data_bits >> 5U);

        __HAL_PKC_RESET(p_pkc);

        pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
        pkc_write_spram(p_pkc, p_input->p_B, pkc_sw_init.ptrB);   // input B
        pkc_write_spram(p_pkc, p_input->p_P, pkc_sw_init.ptrP);   // input P
        ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

        pkc_sw_init.op_mode   = PKC_OPERATION_MODE_ADD;
        pkc_sw_init.enable_it = 1U;
        status = pkc_software_operation(p_pkc, &pkc_sw_init);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_modular_sub(pkc_handle_t *p_pkc, pkc_modular_sub_t *p_input, uint32_t timeout)
{
    hal_status_t            status    = HAL_OK;
    pkc_sw_operation_init_t pkc_sw_init = {0};

    if ((NULL == p_pkc) || (NULL == p_input))
    {
        return HAL_ERROR;
    }

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        __HAL_PKC_RESET(p_pkc);
        __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

        while (pkc_compare_data(p_input->p_A, p_input->p_P, p_pkc->init.data_bits) >= 1U)
        {
            status = pkc_modular_compare_operation(p_pkc, p_input->p_A, p_input->p_P, p_pkc->init.data_bits);
            if(HAL_OK != status)
            {
            }
        }

        __HAL_PKC_RESET(p_pkc);
        while (pkc_compare_data(p_input->p_B, p_input->p_P, p_pkc->init.data_bits) >= 1U)
        {
            status = pkc_modular_compare_operation(p_pkc, p_input->p_B, p_input->p_P, p_pkc->init.data_bits);
            if(HAL_OK != status)
            {
            }
        }

        pkc_sw_init.ptrC = 0U;
        pkc_sw_init.ptrA = 1U * (p_pkc->init.data_bits >> 5U);
        pkc_sw_init.ptrB = 2U * (p_pkc->init.data_bits >> 5U);
        pkc_sw_init.ptrP = 3U * (p_pkc->init.data_bits >> 5U);

        __HAL_PKC_RESET(p_pkc);

        pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
        pkc_write_spram(p_pkc, p_input->p_B, pkc_sw_init.ptrB);   // input B
        pkc_write_spram(p_pkc, p_input->p_P, pkc_sw_init.ptrP);   // input P
        ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

        pkc_sw_init.op_mode   = PKC_OPERATION_MODE_SUB;
        pkc_sw_init.enable_it = 0U;
        pkc_sw_init.timeout   = timeout;
        status = pkc_software_operation(p_pkc, &pkc_sw_init);

        pkc_read_spram(p_pkc, p_pkc->p_result,  pkc_sw_init.ptrC);

        p_pkc->state = HAL_PKC_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_modular_sub_it(pkc_handle_t *p_pkc, pkc_modular_sub_t *p_input)
{
    hal_status_t            status    = HAL_OK;
    pkc_sw_operation_init_t pkc_sw_init = {0};

    if ((NULL == p_pkc) || (NULL == p_input))
    {
        return HAL_ERROR;
    }

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        __HAL_PKC_RESET(p_pkc);
        __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

        while (pkc_compare_data(p_input->p_A, p_input->p_P, p_pkc->init.data_bits) >= 1U)
        {
            status = pkc_modular_compare_operation(p_pkc, p_input->p_A, p_input->p_P, p_pkc->init.data_bits);
            if(HAL_OK != status)
            {
            }
        }

        __HAL_PKC_RESET(p_pkc);
        while (pkc_compare_data(p_input->p_B, p_input->p_P, p_pkc->init.data_bits) >= 1U)
        {
            status = pkc_modular_compare_operation(p_pkc, p_input->p_B, p_input->p_P, p_pkc->init.data_bits);
            if(HAL_OK != status)
            {
            }
        }

        pkc_sw_init.ptrC = 0U;
        pkc_sw_init.ptrA = 1U * (p_pkc->init.data_bits >> 5U);
        pkc_sw_init.ptrB = 2U * (p_pkc->init.data_bits >> 5U);
        pkc_sw_init.ptrP = 3U * (p_pkc->init.data_bits >> 5U);

        __HAL_PKC_RESET(p_pkc);

        pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
        pkc_write_spram(p_pkc, p_input->p_B, pkc_sw_init.ptrB);   // input B
        pkc_write_spram(p_pkc, p_input->p_P, pkc_sw_init.ptrP);   // input P
        ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

        pkc_sw_init.op_mode   = PKC_OPERATION_MODE_SUB;
        pkc_sw_init.enable_it = 1U;
        status = pkc_software_operation(p_pkc, &pkc_sw_init);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_modular_left_shift(pkc_handle_t *p_pkc, pkc_modular_shift_t *p_input, uint32_t timeout)
{
    hal_status_t            status    = HAL_OK;
    pkc_sw_operation_init_t pkc_sw_init = {0};

    if ((NULL == p_pkc) || (NULL == p_input))
    {
        return HAL_ERROR;
    }

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        __HAL_PKC_RESET(p_pkc);
        __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

        while (pkc_compare_data(p_input->p_A, p_input->p_P, p_pkc->init.data_bits) >= 1U)
        {
           status = pkc_modular_compare_operation(p_pkc, p_input->p_A, p_input->p_P, p_pkc->init.data_bits);
            if(HAL_OK != status)
            {
            }
        }

        pkc_sw_init.ptrC = 0U;
        pkc_sw_init.ptrA = 0U;
        pkc_sw_init.ptrP = 1U * (p_pkc->init.data_bits >> 5U);
        pkc_sw_init.ptrB = 0xFFFFFFFFU;

        __HAL_PKC_RESET(p_pkc);

        pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
        pkc_write_spram(p_pkc, p_input->p_P, pkc_sw_init.ptrP);   // input P
        ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

        pkc_sw_init.op_mode   = PKC_OPERATION_MODE_LSHIFT;
        pkc_sw_init.enable_it = 0U;
        pkc_sw_init.timeout   = timeout;
        status = pkc_software_operation(p_pkc, &pkc_sw_init);
        for (uint32_t i = 1; i < p_input->shift_bits; i++)
        {
            ll_pkc_enable_software(p_pkc->p_instance);
            ll_pkc_disable_software_start(p_pkc->p_instance);
            ll_pkc_enable_software_start(p_pkc->p_instance);
            status = pkc_wait_flag_state_until_timeout(p_pkc, PKC_FLAG_BUSY, RESET, timeout);
        }

        pkc_read_spram(p_pkc, p_pkc->p_result,  pkc_sw_init.ptrC);

        p_pkc->state = HAL_PKC_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_modular_left_shift_it(pkc_handle_t *p_pkc, pkc_modular_shift_t *p_input)
{
    hal_status_t            status    = HAL_OK;
    pkc_sw_operation_init_t pkc_sw_init = {0};

    if ((NULL == p_pkc) || (NULL == p_input))
    {
        return HAL_ERROR;
    }

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        __HAL_PKC_RESET(p_pkc);
        __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

        while (pkc_compare_data(p_input->p_A, p_input->p_P, p_pkc->init.data_bits) >= 1U)
        {
            status = pkc_modular_compare_operation(p_pkc, p_input->p_A, p_input->p_P, p_pkc->init.data_bits);
            if(HAL_OK != status)
            {}
        }

        pkc_sw_init.ptrC = 0U;
        pkc_sw_init.ptrA = 0U;
        pkc_sw_init.ptrP = 1U * (p_pkc->init.data_bits >> 5U);
        pkc_sw_init.ptrB = 0xFFFFFFFFU;

        __HAL_PKC_RESET(p_pkc);

        p_pkc->shift_count = p_input->shift_bits;
        pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
        pkc_write_spram(p_pkc, p_input->p_P, pkc_sw_init.ptrP);   // input P
        ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

        pkc_sw_init.op_mode   = PKC_OPERATION_MODE_LSHIFT;
        pkc_sw_init.enable_it = 1U;
        status = pkc_software_operation(p_pkc, &pkc_sw_init);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_modular_compare(pkc_handle_t *p_pkc, pkc_modular_compare_t *p_input, uint32_t timeout)
{
    hal_status_t            status    = HAL_OK;
    pkc_sw_operation_init_t pkc_sw_init = {0};

    if ((NULL == p_pkc) || (NULL == p_input))
    {
        return HAL_ERROR;
    }

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        pkc_sw_init.ptrC = 0U;
        pkc_sw_init.ptrA = 0U;
        pkc_sw_init.ptrP = 1U * (p_pkc->init.data_bits >> 5U);
        pkc_sw_init.ptrB = 0xFFFFFFFFU;

        __HAL_PKC_RESET(p_pkc);
        __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

        pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
        pkc_write_spram(p_pkc, p_input->p_P, pkc_sw_init.ptrP);   // input P
        ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

        pkc_sw_init.op_mode   = PKC_OPERATION_MODE_CMP;
        pkc_sw_init.enable_it = 0U;
        pkc_sw_init.timeout  = timeout;
        status = pkc_software_operation(p_pkc, &pkc_sw_init);

        pkc_read_spram(p_pkc, p_pkc->p_result,  pkc_sw_init.ptrC);
        while (pkc_compare_data(p_pkc->p_result, p_input->p_P, p_pkc->init.data_bits) >= 1U)
        {
            ll_pkc_enable_software(p_pkc->p_instance);
            ll_pkc_disable_software_start(p_pkc->p_instance);
            ll_pkc_enable_software_start(p_pkc->p_instance);
            status = pkc_wait_flag_state_until_timeout(p_pkc, PKC_FLAG_BUSY, RESET, timeout);
            pkc_read_spram(p_pkc, p_pkc->p_result,  pkc_sw_init.ptrC);
        }

        p_pkc->state = HAL_PKC_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_modular_compare_it(pkc_handle_t *p_pkc, pkc_modular_compare_t *p_input)
{
    hal_status_t            status        = HAL_OK;
    pkc_sw_operation_init_t pkc_sw_init = {0};

    if ((NULL == p_pkc) || (NULL == p_input))
    {
        return HAL_ERROR;
    }

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        pkc_sw_init.ptrC = 0U;
        pkc_sw_init.ptrA = 0U;
        pkc_sw_init.ptrP = 1U * (p_pkc->init.data_bits >> 5U);
        pkc_sw_init.ptrB = 0xFFFFFFFFU;

        __HAL_PKC_RESET(p_pkc);
        __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

        p_pkc->p_P = p_input->p_P;
        pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
        pkc_write_spram(p_pkc, p_input->p_P, pkc_sw_init.ptrP);   // input P
        ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

        pkc_sw_init.op_mode   = PKC_OPERATION_MODE_CMP;
        pkc_sw_init.enable_it = 1U;
        status = pkc_software_operation(p_pkc, &pkc_sw_init);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_montgomery_multi(pkc_handle_t *p_pkc, pkc_montgomery_multi_t *p_input, uint32_t timeout)
{
    hal_status_t            status    = HAL_OK;
    pkc_sw_operation_init_t pkc_sw_init = {0};

    if ((NULL == p_pkc) || (NULL == p_input))
    {
        return HAL_ERROR;
    }

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        do {
            __HAL_PKC_RESET(p_pkc);
            __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

            while (pkc_compare_data(p_input->p_A, p_input->p_P, p_pkc->init.data_bits) >= 1U)
            {
                status = pkc_modular_compare_operation(p_pkc, p_input->p_A, p_input->p_P, p_pkc->init.data_bits);
                if(HAL_OK != status)
                {}
            }

            __HAL_PKC_RESET(p_pkc);
            while (pkc_compare_data(p_input->p_B, p_input->p_P, p_pkc->init.data_bits) >= 1U)
            {
                status = pkc_modular_compare_operation(p_pkc, p_input->p_B, p_input->p_P, p_pkc->init.data_bits);
                if(HAL_OK != status)
                {}
            }

            bool temp0 = (pkc_compare_const(p_input->p_A, 0U, p_pkc->init.data_bits) == 0);
            bool temp1 = (pkc_compare_const(p_input->p_B, 0U, p_pkc->init.data_bits) == 0);
            if (temp0 || temp1)
            {
                memset(p_pkc->p_result, 0, p_pkc->init.data_bits >> 3U);
                break;
            }

            pkc_sw_init.ptrC = 0U;
            pkc_sw_init.ptrA = 1U * (p_pkc->init.data_bits >> 5U);
            pkc_sw_init.ptrB = 2U * (p_pkc->init.data_bits >> 5U);
            pkc_sw_init.ptrP = 3U * (p_pkc->init.data_bits >> 5U);

            __HAL_PKC_RESET(p_pkc);

            pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
            pkc_write_spram(p_pkc, p_input->p_B, pkc_sw_init.ptrB);   // input B
            pkc_write_spram(p_pkc, p_input->p_P, pkc_sw_init.ptrP);   // input P
            ll_pkc_set_constp(p_pkc->p_instance, p_input->ConstP);
            ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

            pkc_sw_init.op_mode   = PKC_OPERATION_MODE_MULTI;
            pkc_sw_init.enable_it = 0U;
            pkc_sw_init.timeout   = timeout;
            status = pkc_software_operation(p_pkc, &pkc_sw_init);

            pkc_read_spram(p_pkc, p_pkc->p_result,  pkc_sw_init.ptrC);
        } while(0);

        p_pkc->state = HAL_PKC_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_montgomery_multi_it(pkc_handle_t *p_pkc, pkc_montgomery_multi_t *p_input)
{
    hal_status_t            status        = HAL_OK;
    pkc_sw_operation_init_t pkc_sw_init = {0};
    uint32_t                callback_flag = 0;

    if ((NULL == p_pkc) || (NULL == p_input))
    {
        return HAL_ERROR;
    }

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        do {
            __HAL_PKC_RESET(p_pkc);
            __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

            while (pkc_compare_data(p_input->p_A, p_input->p_P, p_pkc->init.data_bits) >= 1U)
            {
                status = pkc_modular_compare_operation(p_pkc, p_input->p_A, p_input->p_P, p_pkc->init.data_bits);
            }

            __HAL_PKC_RESET(p_pkc);
            while (pkc_compare_data(p_input->p_B, p_input->p_P, p_pkc->init.data_bits) >= 1U)
            {
                status = pkc_modular_compare_operation(p_pkc, p_input->p_B, p_input->p_P, p_pkc->init.data_bits);
            }
            if (HAL_OK != status)
            {
                break;
            }
            
            bool temp0 = (pkc_compare_const(p_input->p_A, 0, p_pkc->init.data_bits) == 0);
            bool temp1 = (pkc_compare_const(p_input->p_B, 0, p_pkc->init.data_bits) == 0);
            if (temp0 || temp1)
            {
                memset(p_pkc->p_result, 0, p_pkc->init.data_bits >> 3);
                callback_flag = 1U;
                break;
            }

            pkc_sw_init.ptrC = 0U;
            pkc_sw_init.ptrA = 1U * (p_pkc->init.data_bits >> 5U);
            pkc_sw_init.ptrB = 2U * (p_pkc->init.data_bits >> 5U);
            pkc_sw_init.ptrP = 3U * (p_pkc->init.data_bits >> 5U);

            __HAL_PKC_RESET(p_pkc);

            pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
            pkc_write_spram(p_pkc, p_input->p_B, pkc_sw_init.ptrB);   // input B
            pkc_write_spram(p_pkc, p_input->p_P, pkc_sw_init.ptrP);   // input P
            ll_pkc_set_constp(p_pkc->p_instance, p_input->ConstP);
            ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

            pkc_sw_init.op_mode   = PKC_OPERATION_MODE_MULTI;
            pkc_sw_init.enable_it = 1U;
            status = pkc_software_operation(p_pkc, &pkc_sw_init);
        } while(0);

        if (callback_flag)
        {
            /* Update PKC state */
            p_pkc->state = HAL_PKC_STATE_READY;

            hal_pkc_done_callback(p_pkc);
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_montgomery_inversion(pkc_handle_t *p_pkc, pkc_montgomery_inversion_t *p_input, uint32_t timeout)
{
    hal_status_t            status    = HAL_OK;
    pkc_sw_operation_init_t pkc_sw_init = {0};

    if ((NULL == p_pkc) || (NULL == p_input))
    {
        return HAL_ERROR;
    }

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        do {
            __HAL_PKC_RESET(p_pkc);
            __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);
            /* 1.Check the param. */
            while (pkc_compare_data(p_input->p_A, p_input->p_P, p_pkc->init.data_bits) >= 1U)
            {
                status = pkc_modular_compare_operation(p_pkc, p_input->p_A, p_input->p_P, p_pkc->init.data_bits);
            }

            if (0 == pkc_compare_const(p_input->p_A, 0, p_pkc->init.data_bits))
            {
                status = HAL_ERROR;
                p_pkc->error_code |= HAL_PKC_ERROR_INVALID_PARAM;
                break;
            }
            if (1U != (p_input->p_P[(p_pkc->init.data_bits >> 5U) - 1U] & 0x1U))
            {
                status = HAL_ERROR;
                p_pkc->error_code |= HAL_PKC_ERROR_INVALID_PARAM;
                break;
            }
            /* 2.Do Partial Mongtomery Inversion, Cal X = A^-1 * 2^k mod P, data_bits <= k <= 2*data_bits */
            uint32_t inX1[64]   = {0};
            uint32_t op_ina_ptr = 0U;
            uint32_t op_inp_ptr = 1U * (p_pkc->init.data_bits >> 5U);
            uint32_t op_x1_ptr  = 2U * (p_pkc->init.data_bits >> 5U);
            uint32_t op_x2_ptr  = 3U * (p_pkc->init.data_bits >> 5U);
            uint32_t op_tmp_ptr = 4U * (p_pkc->init.data_bits >> 5U);
            uint32_t temp_k = 0;    // k

            __HAL_PKC_RESET(p_pkc);

            pkc_write_spram(p_pkc, p_input->p_A, op_ina_ptr);   // input A
            pkc_write_spram(p_pkc, p_input->p_P, op_inp_ptr);   // input P
            pkc_write_spram(p_pkc, inX1, op_x2_ptr);
            inX1[(p_pkc->init.data_bits >> 5U)-1U] = 0x00000001U;
            pkc_write_spram(p_pkc, inX1, op_x1_ptr);

            ll_pkc_set_mi_x1_pointer(p_pkc->p_instance, op_x1_ptr);
            ll_pkc_set_mi_x2_pointer(p_pkc->p_instance, op_x2_ptr);
            ll_pkc_set_swmi_tmp_pointer(p_pkc->p_instance, op_tmp_ptr);
            ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

            pkc_sw_init.op_mode   = PKC_OPERATION_MODE_INVER;
            pkc_sw_init.enable_it = 0U;
            pkc_sw_init.timeout   = timeout;
            pkc_sw_init.ptrA      = op_ina_ptr;
            pkc_sw_init.ptrP      = op_inp_ptr;
            status = pkc_software_operation(p_pkc, &pkc_sw_init);
            if (HAL_OK != status)
            {
                break;
            }

            uint32_t p_A_temp[64] = {0};
            pkc_read_spram(p_pkc, p_pkc->p_result, op_x1_ptr);
            pkc_read_spram(p_pkc, p_A_temp, op_ina_ptr);
            temp_k = ll_pkc_get_mik_output(p_pkc->p_instance);

            if ((temp_k < p_pkc->init.data_bits) || (temp_k > (p_pkc->init.data_bits << 1U)))
            {
                temp_k = 0x0;
                status = HAL_ERROR;
                p_pkc->error_code |= HAL_PKC_ERROR_INVERSE_K;
                break;
            }

            if (p_A_temp[(p_pkc->init.data_bits >> 5U)-1U] != 1U)
            {
                temp_k = 0x0U;
                status = HAL_ERROR;
                p_pkc->error_code |= HAL_PKC_ERROR_IRREVERSIBLE;
                break;
            }
            /* 3. Cal T = MM(X,2^(2*data_bits-k)) = A^-1 * R mod P */
            uint32_t temp[64] = {0};
            uint32_t t_k = (p_pkc->init.data_bits << 1) - temp_k;
            uint32_t t_pos = (p_pkc->init.data_bits >> 5U) - 1U - t_k / 32U;
            uint32_t t_bit_offset = t_k % 32U;
            temp[t_pos] |= (1U << t_bit_offset);

            pkc_sw_init.ptrC = 0U;
            pkc_sw_init.ptrA = 1U * (p_pkc->init.data_bits >> 5U);
            pkc_sw_init.ptrB = 2U * (p_pkc->init.data_bits >> 5U);
            pkc_sw_init.ptrP = 3U * (p_pkc->init.data_bits >> 5U);

            __HAL_PKC_RESET(p_pkc);

            pkc_write_spram(p_pkc, p_pkc->p_result, pkc_sw_init.ptrA);  // input A
            pkc_write_spram(p_pkc, temp, pkc_sw_init.ptrB);             // input B
            pkc_write_spram(p_pkc, p_input->p_P, pkc_sw_init.ptrP);     // input P
            ll_pkc_set_constp(p_pkc->p_instance, p_input->ConstP);
            ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

            pkc_sw_init.op_mode   = PKC_OPERATION_MODE_MULTI;
            pkc_sw_init.enable_it = 0U;
            pkc_sw_init.timeout   = timeout;
            status = pkc_software_operation(p_pkc, &pkc_sw_init);
            pkc_read_spram(p_pkc, p_pkc->p_result,  pkc_sw_init.ptrC);

            __HAL_PKC_RESET(p_pkc);
            /* 4. Cal Result = MM(T,1) = A^-1 * 1 mode P */
            temp[t_pos] = 0x00000000U;
            temp[(p_pkc->init.data_bits >> 5)-1U] = 0x00000001U;
            pkc_write_spram(p_pkc, p_pkc->p_result, pkc_sw_init.ptrA);  // input A
            pkc_write_spram(p_pkc, temp, pkc_sw_init.ptrB);             // input B
            pkc_write_spram(p_pkc, p_input->p_P, pkc_sw_init.ptrP);     // input P
            ll_pkc_set_constp(p_pkc->p_instance, p_input->ConstP);
            ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);
            status = pkc_software_operation(p_pkc, &pkc_sw_init);
            pkc_read_spram(p_pkc, p_pkc->p_result,  pkc_sw_init.ptrC);
        } while(0);

        p_pkc->state = HAL_PKC_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_montgomery_inversion_it(pkc_handle_t *p_pkc, pkc_montgomery_inversion_t *p_input)
{
    hal_status_t            status    = HAL_OK;
    pkc_sw_operation_init_t pkc_sw_init = {0};

    if ((NULL == p_pkc) || (NULL == p_input))
    {
        return HAL_ERROR;
    }

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        do {
            __HAL_PKC_RESET(p_pkc);
            __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

            while (pkc_compare_data(p_input->p_A, p_input->p_P, p_pkc->init.data_bits) >= 1U)
            {
                status = pkc_modular_compare_operation(p_pkc, p_input->p_A, p_input->p_P, p_pkc->init.data_bits);
            }
            if (0 == pkc_compare_const(p_input->p_A, 0, p_pkc->init.data_bits))
            {
                status = HAL_ERROR;
                p_pkc->error_code |= HAL_PKC_ERROR_INVALID_PARAM;
                break;
            }
            if (1U != (p_input->p_P[(p_pkc->init.data_bits >> 5U) - 1U] & 0x1U))
            {
                status = HAL_ERROR;
                p_pkc->error_code |= HAL_PKC_ERROR_INVALID_PARAM;
                break;
            }

            uint32_t inX1[64]   = {0};
            uint32_t op_ina_ptr = 0U;
            uint32_t op_inp_ptr = 1U * (p_pkc->init.data_bits >> 5U);
            uint32_t op_x1_ptr  = 2U * (p_pkc->init.data_bits >> 5U);
            uint32_t op_x2_ptr  = 3U * (p_pkc->init.data_bits >> 5U);
            uint32_t op_tmp_ptr = 4U * (p_pkc->init.data_bits >> 5U);

            __HAL_PKC_RESET(p_pkc);

            p_pkc->p_P = p_input->p_P;
            pkc_write_spram(p_pkc, p_input->p_A, op_ina_ptr);   // input A
            pkc_write_spram(p_pkc, p_input->p_P, op_inp_ptr);   // input P
            pkc_write_spram(p_pkc, inX1, op_x2_ptr);
            inX1[(p_pkc->init.data_bits >> 5U) - 1U] = 1U;
            pkc_write_spram(p_pkc, inX1, op_x1_ptr);
            p_pkc->p_P = p_input->p_P;

            ll_pkc_set_mi_x1_pointer(p_pkc->p_instance, op_x1_ptr);
            ll_pkc_set_mi_x2_pointer(p_pkc->p_instance, op_x2_ptr);
            ll_pkc_set_swmi_tmp_pointer(p_pkc->p_instance, op_tmp_ptr);
            ll_pkc_set_constp(p_pkc->p_instance, p_input->ConstP);
            ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

            pkc_sw_init.op_mode   = PKC_OPERATION_MODE_INVER;
            pkc_sw_init.enable_it = 1U;
            pkc_sw_init.ptrA      = op_ina_ptr;
            pkc_sw_init.ptrP      = op_inp_ptr;
            status = pkc_software_operation(p_pkc, &pkc_sw_init);
        } while(0);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_big_number_multi(pkc_handle_t *p_pkc, pkc_big_number_multi_t *p_input, uint32_t timeout)
{
    hal_status_t            status    = HAL_OK;
    pkc_sw_operation_init_t pkc_sw_init = {0};

    if ((NULL == p_pkc) || (NULL == p_input))
    {
        return HAL_ERROR;
    }

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BIGMULTI_BITS_LENGTH(p_pkc->init.data_bits));

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        pkc_sw_init.ptrA = 0U;
        pkc_sw_init.ptrB = 1U * (p_pkc->init.data_bits >> 5U);
        pkc_sw_init.ptrC = 2U * (p_pkc->init.data_bits >> 5U);

        __HAL_PKC_RESET(p_pkc);
        __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

        pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
        pkc_write_spram(p_pkc, p_input->p_B, pkc_sw_init.ptrB);   // input B
        ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

        pkc_sw_init.op_mode   = PKC_OPERATION_MODE_BIGMULTI;
        pkc_sw_init.enable_it = 0U;
        pkc_sw_init.timeout   = timeout;
        status = pkc_software_operation(p_pkc, &pkc_sw_init);

        pkc_read_spram(p_pkc, p_pkc->p_result, pkc_sw_init.ptrC);

        p_pkc->state = HAL_PKC_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_big_number_multi_it(pkc_handle_t *p_pkc, pkc_big_number_multi_t *p_input)
{
    hal_status_t            status    = HAL_OK;
    pkc_sw_operation_init_t pkc_sw_init = {0};

    if ((NULL == p_pkc) || (NULL == p_input))
    {
        return HAL_ERROR;
    }

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BIGMULTI_BITS_LENGTH(p_pkc->init.data_bits));

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        pkc_sw_init.ptrA = 0U;
        pkc_sw_init.ptrB = 1U * (p_pkc->init.data_bits >> 5U);
        pkc_sw_init.ptrC = 2U * (p_pkc->init.data_bits >> 5U);

        __HAL_PKC_RESET(p_pkc);
        __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

        pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
        pkc_write_spram(p_pkc, p_input->p_B, pkc_sw_init.ptrB);   // input B
        ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

        pkc_sw_init.op_mode   = PKC_OPERATION_MODE_BIGMULTI;
        pkc_sw_init.enable_it = 1U;
        status = pkc_software_operation(p_pkc, &pkc_sw_init);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_big_number_add(pkc_handle_t *p_pkc, pkc_big_number_add_t *p_input, uint32_t timeout)
{
    hal_status_t            status    = HAL_OK;
    pkc_sw_operation_init_t pkc_sw_init = {0};

    if ((NULL == p_pkc) || (NULL == p_input))
    {
        return HAL_ERROR;
    }

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        pkc_sw_init.ptrA = 0U;
        pkc_sw_init.ptrB = 1U * (p_pkc->init.data_bits >> 5U);
        pkc_sw_init.ptrC = 2U * (p_pkc->init.data_bits >> 5U);

        __HAL_PKC_RESET(p_pkc);
        __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

        pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
        pkc_write_spram(p_pkc, p_input->p_B, pkc_sw_init.ptrB);   // input B
        ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

        pkc_sw_init.op_mode   = PKC_OPERATION_MODE_BIGADD;
        pkc_sw_init.enable_it = 0U;
        pkc_sw_init.timeout   = timeout;
        status = pkc_software_operation(p_pkc, &pkc_sw_init);

        pkc_read_spram(p_pkc, p_pkc->p_result, pkc_sw_init.ptrC);
        uint32_t it_status = READ_REG(p_pkc->p_instance->INT_STAT);
        if (it_status & PKC_IT_OVF)
        {
            /* Set error code */
            p_pkc->error_code |= HAL_PKC_ERROR_OVERFLOW;
        }

        p_pkc->state = HAL_PKC_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_big_number_add_it(pkc_handle_t *p_pkc, pkc_big_number_add_t *p_input)
{
    hal_status_t            status    = HAL_OK;
    pkc_sw_operation_init_t pkc_sw_init = {0};

    if ((NULL == p_pkc) || (NULL == p_input))
    {
        return HAL_ERROR;
    }

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        pkc_sw_init.ptrA = 0U;
        pkc_sw_init.ptrB = 1U * (p_pkc->init.data_bits >> 5U);
        pkc_sw_init.ptrC = 2U * (p_pkc->init.data_bits >> 5U);

        __HAL_PKC_RESET(p_pkc);
        __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

        pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
        pkc_write_spram(p_pkc, p_input->p_B, pkc_sw_init.ptrB);   // input B
        ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

        pkc_sw_init.op_mode   = PKC_OPERATION_MODE_BIGADD;
        pkc_sw_init.enable_it = 1U;
        status = pkc_software_operation(p_pkc, &pkc_sw_init);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK void hal_pkc_done_callback(pkc_handle_t *p_pkc)
{
    UNUSED(p_pkc);
}

__WEAK void hal_pkc_error_callback(pkc_handle_t *p_pkc)
{
    UNUSED(p_pkc);
}

__WEAK void hal_pkc_overflow_callback(pkc_handle_t *p_pkc)
{
    UNUSED(p_pkc);
}

__WEAK hal_pkc_state_t hal_pkc_get_state(pkc_handle_t *p_pkc)
{
    return p_pkc->state;
}

__WEAK uint32_t hal_pkc_get_error(pkc_handle_t *p_pkc)
{
    return p_pkc->error_code;
}

static hal_status_t pkc_wait_flag_state_until_timeout(pkc_handle_t *p_pkc,
                                                      uint32_t      flag,
                                                      flag_status_t state,
                                                      uint32_t      timeout)
{
    if((HAL_NEVER_TIMEOUT != timeout) && (HAL_MAX_DELAY < timeout))
    {
        return HAL_ERROR;
    }

    HAL_TIMEOUT_INIT();
    uint32_t tickstart = HAL_TIMEOUT_GET_TICK();
    /* Update SystemCoreClock */
    SystemCoreUpdateClock();
    uint32_t timeout_counter = ((SystemCoreClock / 1000U) * timeout);

    /* Wait until flag is in expected state */
    while ((__HAL_PKC_GET_FLAG(p_pkc, flag)) != state)
    {
        if(HAL_NEVER_TIMEOUT != timeout)
        {
            if((HAL_TIMEOUT_GET_TICK() - tickstart) > timeout_counter)
            {
                p_pkc->error_code |= HAL_PKC_ERROR_TIMEOUT;
                HAL_TIMEOUT_DEINIT();
                return HAL_TIMEOUT;
            }
        }
    }
    HAL_TIMEOUT_DEINIT();
    return HAL_OK;
}

static void pkc_read_spram(pkc_handle_t *p_pkc, uint32_t *p_data, uint32_t ptr)
{
    //lint -e9033  cast of composite expression is necessary
    uint32_t *preg         = (uint32_t *)(PKC_SPRAM_BASE_ADDR + (ptr << 2U));
    uint32_t data_word_len = p_pkc->init.data_bits >> 5U;

    if (PKC_OPERATION_MODE_BIGMULTI == ll_pkc_get_operation_mode(p_pkc->p_instance))
    {
        data_word_len <<= 1U;
    }

    for (uint32_t i = 0U; i < data_word_len; i++)
    {
        p_data[data_word_len - i - 1U] = preg[i];
    }
}

static void pkc_write_spram(pkc_handle_t *p_pkc, uint32_t *p_data, uint32_t ptr)
{
    //lint -e9033  cast of composite expression is necessary
    uint32_t *preg = (uint32_t *)(PKC_SPRAM_BASE_ADDR + (ptr << 2U));
    uint32_t data_word_len = p_pkc->init.data_bits >> 5U;

    for (uint32_t i = 0U; i < data_word_len; i++)
    {
        preg[i] = p_data[data_word_len - i - 1U];
    }
}

static hal_status_t pkc_modular_compare_operation(pkc_handle_t *p_pkc, uint32_t *p_data_a, uint32_t *p_data_b, uint32_t data_bits)
{
    pkc_sw_operation_init_t pkc_sw_init;
    hal_status_t status = HAL_OK;

    pkc_sw_init.ptrC = 0U;
    pkc_sw_init.ptrA = 0U;
    pkc_sw_init.ptrP = 1U * (p_pkc->init.data_bits >> 5U);
    pkc_sw_init.ptrB = 0xFFFFFFFFU;

    pkc_write_spram(p_pkc, p_data_a, pkc_sw_init.ptrA);   // input A
    pkc_write_spram(p_pkc, p_data_b, pkc_sw_init.ptrP);   // input P
    ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

    pkc_sw_init.op_mode   = PKC_OPERATION_MODE_CMP;
    pkc_sw_init.enable_it = 0U;
    pkc_sw_init.timeout  = HAL_PKC_TIMEOUT_DEFAULT_VALUE;
    status = pkc_software_operation(p_pkc, &pkc_sw_init);

    pkc_read_spram(p_pkc, p_data_a,  pkc_sw_init.ptrC);

    return status;
}

static uint32_t pkc_compare_data(uint32_t *p_data_a, uint32_t *p_data_b, uint32_t data_bits)
{
    uint32_t data_word_len = data_bits >> 5U;
    uint32_t  i, ret = 0U;

    for(i = 0U; i < data_word_len; i++)
    {
        if (p_data_a[i] > p_data_b[i])
        {
            return 2U;
        }
        else if (p_data_a[i] < p_data_b[i])
        {
            return 0U;
        }
        else
        {
            ret = 1U;
        }
    }

    return ret;
}

/*
    return value: 0, input data = 0
    return value: 1~DataBits, input data > 0
*/
static uint32_t pkc_find_msb(uint32_t *p_data, uint32_t data_bits)
{
    uint32_t data_word_len = data_bits >> 5U;
    uint32_t ret = 0U, i, j, k = 0U;

    for (i = 0U; i < data_word_len; i++)
    {
        if (0U != p_data[i])
        {
            for (j = 32U; j > 0U; j--)
            {
                if ((p_data[i] >> (j - 1U)) & 0x1U)
                {
                    ret = j + ((data_word_len - i - 1U) << 5U);
                    k = 1U;
                    break;
                }
            }
            if (k == 1U)
            {
                break;
            }
        }
    }

    return ret;
}

/* ofs: 0 ~ data_bits-1 */
static uint32_t pkc_get_bit(uint32_t *p_data, uint32_t ofs, uint32_t data_bits)
{
    uint32_t data_word_len = data_bits >> 5U;

    if (p_data[data_word_len - 1U - (ofs >> 5U)] & (1U << (ofs & 0x1FU)))
    {
        return 1U;
    }
    else
    {
        return 0U;
    }
}

static int32_t pkc_compare_const(uint32_t *p_data, uint32_t Const, uint32_t data_bits)
{
    uint32_t data_word_len = data_bits >> 5U;
    uint32_t i;

    for(i = 1; i < (data_word_len - 1U); i++)
    {
        if(p_data[i] > 0U)
        {
            return 1;
        }
    }

    if(p_data[0] > Const)
    {
        return 1;
    }
    else if(p_data[0] == Const)
    {
        return 0;
    }
    else
    {
        return -1;
    }
}

//ret : 0 --> not infinite point , 1--> is infinite point
static uint32_t ecc_is_infinite_point(ecc_point_t *p_Piont)
{
    for (uint32_t i = 0U; i < ECC_U32_LENGTH; i++)
    {
        if ((0U != p_Piont->X[i]) || (0U != p_Piont->Y[i]))
        {
            return 0U;
        }
    }

    return 1U;
}

static hal_status_t pkc_software_operation(pkc_handle_t *p_pkc, pkc_sw_operation_init_t *p_input)
{
    hal_status_t status    = HAL_OK;

    switch(p_input->op_mode)
    {
    case PKC_OPERATION_MODE_MULTI:
        ll_pkc_set_mm_a_pointer(p_pkc->p_instance, p_input->ptrA);
        ll_pkc_set_mm_b_pointer(p_pkc->p_instance, p_input->ptrB);
        ll_pkc_set_mm_p_pointer(p_pkc->p_instance, p_input->ptrP);
        ll_pkc_set_mm_c_pointer(p_pkc->p_instance, p_input->ptrC);
        break;
    case PKC_OPERATION_MODE_INVER:
        ll_pkc_set_mi_u_pointer(p_pkc->p_instance, p_input->ptrA);
        ll_pkc_set_mi_v_pointer(p_pkc->p_instance, p_input->ptrP);
        break;
    case PKC_OPERATION_MODE_CMP:
    case PKC_OPERATION_MODE_LSHIFT:
    case PKC_OPERATION_MODE_ADD:
    case PKC_OPERATION_MODE_SUB:
        ll_pkc_set_mas_a_pointer(p_pkc->p_instance, p_input->ptrA);
        ll_pkc_set_mas_b_pointer(p_pkc->p_instance, p_input->ptrB);
        ll_pkc_set_mas_p_pointer(p_pkc->p_instance, p_input->ptrP);
        ll_pkc_set_mas_c_pointer(p_pkc->p_instance, p_input->ptrC);
        break;
    case PKC_OPERATION_MODE_BIGMULTI:
        ll_pkc_set_bm_a_pointer(p_pkc->p_instance, p_input->ptrA);
        ll_pkc_set_bm_b_pointer(p_pkc->p_instance, p_input->ptrB);
        ll_pkc_set_bm_c_pointer(p_pkc->p_instance, p_input->ptrC);
        break;
    case PKC_OPERATION_MODE_BIGADD:
        ll_pkc_set_ba_a_pointer(p_pkc->p_instance, p_input->ptrA);
        ll_pkc_set_ba_b_pointer(p_pkc->p_instance, p_input->ptrB);
        ll_pkc_set_ba_c_pointer(p_pkc->p_instance, p_input->ptrC);
        break;
    default:
        status = HAL_ERROR;
        break;
    }

    if (HAL_OK == status)
    {
        /* Set operation mode */
        ll_pkc_set_operation_mode(p_pkc->p_instance, p_input->op_mode);
        /* Enable SW mode */
        ll_pkc_enable_software(p_pkc->p_instance);
        ll_pkc_disable_software_start(p_pkc->p_instance);
        if (PKC_SECURE_MODE_ENABLE == p_pkc->init.secure_mode)
        {
            ll_pkc_set_dummy_multiply_seed(p_pkc->p_instance, p_pkc->init.random_func());
            ll_pkc_set_random_clock_gating_seed(p_pkc->p_instance, p_pkc->init.random_func());

            ll_pkc_enable_random_clock_gating(p_pkc->p_instance);
        }

        if (PKC_OPERATION_MODE_BIGMULTI != p_input->op_mode)
        {
            ll_pkc_enable_dummy_multi(p_pkc->p_instance);
        }

        if (0U == p_input->enable_it)
        {
            ll_pkc_enable_software_start(p_pkc->p_instance);
            /* Wait for finish */
            status = pkc_wait_flag_state_until_timeout(p_pkc, PKC_FLAG_BUSY, RESET, p_input->timeout);

            if ((HAL_OK == status) && (PKC_OPERATION_MODE_CMP != p_input->op_mode))
            {
                if (p_pkc->p_instance->INT_STAT & PKC_INT_STAT_ERR_INT_FLAG)
                {
                    status = HAL_ERROR;
                }
            }
            ll_pkc_disable_software(p_pkc->p_instance);
        }
        else
        {
            __HAL_PKC_CLEAR_FLAG_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);
            __HAL_PKC_ENABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR);
            if (PKC_OPERATION_MODE_BIGADD == p_input->op_mode)
            {
                __HAL_PKC_ENABLE_IT(p_pkc, PKC_IT_OVF);
            }

            ll_pkc_enable_software_start(p_pkc->p_instance);
        }
    }

    return status;
}

static hal_status_t ecc_point_multiply(pkc_handle_t *p_pkc, ecc_point_multi_init_t *p_input)
{
    hal_status_t            status = HAL_OK;
    pkc_sw_operation_init_t pkc_sw_init;
    uint32_t                i;
    uint32_t                r_random[ECC_U32_LENGTH << 1U] = {0};
    uint32_t                gz_random[ECC_U32_LENGTH] = {0};
    uint32_t                tmpk[ECC_U32_LENGTH] = {0};

    /* clear SPRAM */
    //memset((uint8_t *)PKC_SPRAM_BASE_ADDR, 0x00, PKC_SPRAM_SIZE);
    for(uint32_t *ptr = (uint32_t*)PKC_SPRAM_BASE_ADDR;ptr!=(uint32_t*)(PKC_SPRAM_BASE_ADDR+PKC_SPRAM_SIZE);ptr++)
    {
        *ptr = 0U;
    }

    pkc_sw_init.ptrC = 0U;
    pkc_sw_init.ptrA = 0U;
    pkc_sw_init.ptrB = 2U * (p_pkc->init.data_bits >> 5U);

    if (PKC_SECURE_MODE_ENABLE == p_pkc->init.secure_mode)
    {
        uint32_t rg[ECC_U32_LENGTH] = {0};

        rg[ECC_U32_LENGTH - 1U] = p_pkc->init.random_func();
        rg[ECC_U32_LENGTH - 2U] = p_pkc->init.random_func();
        rg[ECC_U32_LENGTH - 2U] |= 0x80000000U;

        for (i = 0; i < ECC_U32_LENGTH; i++)
        {
            r_random[i] = p_pkc->init.random_func();
            r_random[i + ECC_U32_LENGTH] = p_pkc->init.random_func();
            gz_random[i] = p_pkc->init.random_func();
        }

        pkc_write_spram(p_pkc, rg, pkc_sw_init.ptrA);   // input A
        pkc_write_spram(p_pkc, p_pkc->init.p_ecc_curve->N, pkc_sw_init.ptrB);   // input B
        ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

        pkc_sw_init.op_mode   = PKC_OPERATION_MODE_BIGMULTI;
        pkc_sw_init.enable_it = 0U;
        pkc_sw_init.timeout   = HAL_PKC_TIMEOUT_DEFAULT_VALUE;
        status = pkc_software_operation(p_pkc, &pkc_sw_init);

        pkc_write_spram(p_pkc, p_input->p_input->p_K, pkc_sw_init.ptrB);
        ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits << 1);

        pkc_sw_init.op_mode   = PKC_OPERATION_MODE_BIGADD;
        status = pkc_software_operation(p_pkc, &pkc_sw_init);

        ll_pkc_set_dummy_multiply_seed(p_pkc->p_instance, p_pkc->init.random_func());
        ll_pkc_set_random_clock_gating_seed(p_pkc->p_instance, p_pkc->init.random_func());

        ll_pkc_enable_random_clock_gating(p_pkc->p_instance);
    }
    else
    {
        pkc_write_spram(p_pkc, p_input->p_input->p_K, pkc_sw_init.ptrA);
        r_random[0] = p_pkc->init.random_func();

        for (i = 0U; i < ECC_U32_LENGTH; i++)
        {
            r_random[i] = r_random[0] * (i + 1U);
            r_random[i + ECC_U32_LENGTH] = r_random[0] * (0xFABCD971U + 3U * i);
            gz_random[i] = r_random[0] * (0xDFE11111U + 17U * i);
        }
    }
    ll_pkc_enable_dummy_multi(p_pkc->p_instance);

    p_pkc->init.data_bits <<= 1U;
    pkc_write_spram(p_pkc, r_random, 0x10U);                          //write r_random
    p_pkc->init.data_bits >>= 1;
    pkc_write_spram(p_pkc, p_pkc->init.p_ecc_curve->P, 0x20U);           //write p
    pkc_write_spram(p_pkc, p_pkc->init.p_ecc_curve->PRSquare, 0x28U);    //write R^2 mod p
    pkc_write_spram(p_pkc, p_input->p_input->p_ecc_point->X, 0x30U);        //write Point's x axis
    pkc_write_spram(p_pkc, p_input->p_input->p_ecc_point->Y, 0x38U);        //write Point's y axis
    pkc_write_spram(p_pkc, gz_random, 0x40U);                         //write random
    pkc_write_spram(p_pkc, p_pkc->init.p_ecc_curve->A, 0xD0U);           //write a*R
    pkc_write_spram(p_pkc, p_pkc->init.p_ecc_curve->B, 0xD8U);           //write b*R

    /* Compute tmpk = R mod p = 0xFFFF...FFF - p + 1 = 0xFFFFFF...FF xor p + 1 */
    for (i = 0U; i < ECC_U32_LENGTH; i++)
    {
        tmpk[i] = 0xFFFFFFFFU ^ p_pkc->init.p_ecc_curve->P[i];
    }
    tmpk[ECC_U32_LENGTH - 1U]++;

    pkc_write_spram(p_pkc, tmpk, 0xC8U);                              //write R
    ll_pkc_set_constp(p_pkc->p_instance, p_pkc->init.p_ecc_curve->ConstP);
    ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

    if (0U == p_input->enable_it)
    {
        ll_pkc_enable_hardware_start(p_pkc->p_instance);
        /* Wait for finish */
        status = pkc_wait_flag_state_until_timeout(p_pkc, PKC_FLAG_BUSY, RESET, p_input->timeout);

        if ((HAL_OK == status))
        {
            if(p_pkc->p_instance->INT_STAT & PKC_INT_STAT_ERR_INT_FLAG)
            {
                status = HAL_ERROR;
            }
        }

        ll_pkc_disable_hardware_start(p_pkc->p_instance);
    }
    else
    {
        __HAL_PKC_CLEAR_FLAG_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);
        __HAL_PKC_ENABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR);

        ll_pkc_enable_hardware_start(p_pkc->p_instance);
    }

    return status;
}

#endif /* HAL_PKC_MODULE_ENABLED */

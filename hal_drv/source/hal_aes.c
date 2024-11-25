/**
  ****************************************************************************************
  * @file    gr5xx_hal_aes.c
  * @author  BLE Driver Team
  * @brief   AES HAL module driver.
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

//lint -e9058 [advisory] tag unused
#ifdef HAL_AES_MODULE_ENABLED
#include "hal_def.h"
#include "hal_aes.h"

#ifdef HAL_CLOCK_UNIFORM_CONTROL
#include "hal_clock.h"
#endif

static hal_status_t aes_wait_flag_state_until_timeout(aes_handle_t *p_aes, uint32_t flag, flag_status_t state, uint32_t timeout);
static hal_status_t aes_config(aes_handle_t *p_aes);
static void aes_read_data(aes_handle_t *p_aes, uint32_t *p_data);
static void aes_write_data(aes_handle_t *p_aes, uint32_t *p_data);
static hal_status_t aes_mcu_process(aes_handle_t *p_aes, uint32_t timeout);
static hal_status_t aes_dma_process(aes_handle_t *p_aes);

__WEAK hal_status_t hal_aes_init(aes_handle_t *p_aes)
{
    hal_status_t status = HAL_OK;

    /* Check the AES handle allocation */
    if (NULL == p_aes)
    {
        return HAL_ERROR;
    }

    GLOBAL_EXCEPTION_DISABLE();
    if (HAL_AES_STATE_RESET == p_aes->state)
    {
#ifdef HAL_CLOCK_UNIFORM_CONTROL
        //lint -e923 Cast from pointer to unsigned int is necessary
        hal_clock_enable_module((uint32_t)p_aes->p_instance);
#endif
        /* Enable security blocks clock and Automatic turn off security blocks clock during WFI. */
//        ll_cgc_disable_force_off_secu_hclk();
//        ll_cgc_disable_wfi_off_secu_hclk();

//        ll_cgc_disable_force_off_aes_hclk();
//        ll_cgc_disable_force_off_secu_div4_pclk();
//        ll_cgc_disable_wfi_off_aes_hclk();
//        ll_cgc_disable_wfi_off_secu_div4_hclk();

        /* Init the low level hardware : CLOCK, NVIC */
        hal_aes_msp_init(p_aes);
    }

    /* Enable AES */
    ll_aes_enable(p_aes->p_instance);

    /* Set AES error code to none */
    p_aes->error_code = HAL_AES_ERROR_NONE;

    /* Initialize the AES state */
    p_aes->state = HAL_AES_STATE_READY;
    GLOBAL_EXCEPTION_ENABLE();

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_aes_deinit(aes_handle_t *p_aes)
{
    hal_status_t status = HAL_OK;

    /* Check the AES handle allocation */
    if (NULL == p_aes)
    {
        return HAL_ERROR;
    }

    /* Set AES registers to their reset values */
    p_aes->p_instance->CTRL = 0;
    p_aes->p_instance->CFG = 0;
    p_aes->p_instance->INT = 1;

    GLOBAL_EXCEPTION_DISABLE();
#ifdef HAL_CLOCK_UNIFORM_CONTROL
    //lint -e923 Cast from pointer to unsigned int is necessary
    hal_clock_disable_module((uint32_t)p_aes->p_instance);
#endif
    /* Disable AES Clock */
//    ll_cgc_enable_force_off_aes_hclk();
//    ll_cgc_enable_wfi_off_aes_hclk();

    /* Disable Security Clock if all security peripherals are disabled */
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

    /* DeInit the low level hardware: CLOCK, NVIC... */
    hal_aes_msp_deinit(p_aes);

    /* Set AES error code to none */
    p_aes->error_code = HAL_AES_ERROR_NONE;

    /* Initialize the AES state */
    p_aes->state = HAL_AES_STATE_RESET;
    GLOBAL_EXCEPTION_ENABLE();

    /* Return function status */
    return status;
}

__WEAK void hal_aes_msp_init(aes_handle_t *p_aes)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_aes);
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
            the hal_aes_msp_init can be implemented in the user file
     */
}

__WEAK void hal_aes_msp_deinit(aes_handle_t *p_aes)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_aes);
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
            the hal_aes_msp_deinit can be implemented in the user file
     */
}

__WEAK hal_status_t hal_aes_ecb_encrypt(aes_handle_t *p_aes, uint32_t *p_plain_data, uint32_t number, uint32_t *p_cypher_data, uint32_t timeout)
{
    hal_status_t status = HAL_OK;

    /* Check the AES handle allocation */
    if (NULL == p_aes)
    {
        return HAL_ERROR;
    }

    if (HAL_AES_STATE_READY == p_aes->state)
    {
        /* Change state Busy */
        p_aes->state = HAL_AES_STATE_BUSY;

        p_aes->operation_mode       = AES_OPERATION_MODE_ENCRYPT;
        p_aes->chaining_mode        = AES_CHAININGMODE_ECB;
        p_aes->p_cryp_input_buffer  = p_plain_data;
        p_aes->p_cryp_output_buffer = p_cypher_data;
        p_aes->block_size           = number >> 4;
        p_aes->keyram_offset        = p_aes->init.key_slot;

        /* Configure AES registers */
        status = aes_config(p_aes);
        if (HAL_OK != status)
        {
            /* Set invalid param error code */
            p_aes->error_code |= HAL_AES_ERROR_INVALID_PARAM;

            return status;
        }

        /* AES MCU Process */
        status = aes_mcu_process(p_aes, ((timeout > 0U) ? timeout : 1U));
    }
    else
    {
        /* Set Busy error code */
        p_aes->error_code |= HAL_AES_ERROR_BUSY;
        status = HAL_ERROR;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_aes_ecb_decrypt(aes_handle_t *p_aes, uint32_t *p_cypher_data, uint32_t number, uint32_t *p_plain_data, uint32_t timeout)
{
    hal_status_t status = HAL_OK;

    /* Check the AES handle allocation */
    if (NULL == p_aes)
    {
        return HAL_ERROR;
    }

    if (HAL_AES_STATE_READY == p_aes->state)
    {
        /* Change state Busy */
        p_aes->state = HAL_AES_STATE_BUSY;

        p_aes->operation_mode       = AES_OPERATION_MODE_DECRYPT;
        p_aes->chaining_mode        = AES_CHAININGMODE_ECB;
        p_aes->p_cryp_input_buffer  = p_cypher_data;
        p_aes->p_cryp_output_buffer = p_plain_data;
        p_aes->block_size           = number >> 4;
        p_aes->keyram_offset        = p_aes->init.key_slot;

        /* Configure AES registers */
        status = aes_config(p_aes);
        if (HAL_OK != status)
        {
            /* Set invalid param error code */
            p_aes->error_code |= HAL_AES_ERROR_INVALID_PARAM;

            return status;
        }

        /* AES MCU Process */
        status = aes_mcu_process(p_aes, ((timeout > 0U) ? timeout : 1U));
    }
    else
    {
        /* Set Busy error code */
        p_aes->error_code |= HAL_AES_ERROR_BUSY;
        status = HAL_ERROR;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_aes_cbc_encrypt(aes_handle_t *p_aes, uint32_t *p_plain_data, uint32_t number, uint32_t *p_cypher_data, uint32_t timeout)
{
    hal_status_t status = HAL_OK;

    /* Check the AES handle allocation */
    if (NULL == p_aes)
    {
        return HAL_ERROR;
    }

    if (HAL_AES_STATE_READY == p_aes->state)
    {
        /* Change state Busy */
        p_aes->state = HAL_AES_STATE_BUSY;

        p_aes->operation_mode       = AES_OPERATION_MODE_ENCRYPT;
        p_aes->chaining_mode        = AES_CHAININGMODE_CBC;
        p_aes->p_cryp_input_buffer  = p_plain_data;
        p_aes->p_cryp_output_buffer = p_cypher_data;
        p_aes->block_size           = number >> 4;
        p_aes->keyram_offset        = p_aes->init.key_slot;

        /* Configure AES registers */
        status = aes_config(p_aes);
        if (HAL_OK != status)
        {
            /* Set invalid param error code */
            p_aes->error_code |= HAL_AES_ERROR_INVALID_PARAM;

            return status;
        }

        /* AES MCU Process */
        status = aes_mcu_process(p_aes, ((timeout > 0U) ? timeout : 1U));
    }
    else
    {
        /* Set Busy error code */
        p_aes->error_code |= HAL_AES_ERROR_BUSY;
        status = HAL_ERROR;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_aes_cbc_decrypt(aes_handle_t *p_aes, uint32_t *p_cypher_data, uint32_t number, uint32_t *p_plain_data, uint32_t timeout)
{
    hal_status_t status = HAL_OK;

    /* Check the AES handle allocation */
    if (NULL == p_aes)
    {
        return HAL_ERROR;
    }

    if (HAL_AES_STATE_READY == p_aes->state)
    {
        /* Change state Busy */
        p_aes->state = HAL_AES_STATE_BUSY;

        p_aes->operation_mode       = AES_OPERATION_MODE_DECRYPT;
        p_aes->chaining_mode        = AES_CHAININGMODE_CBC;
        p_aes->p_cryp_input_buffer  = p_cypher_data;
        p_aes->p_cryp_output_buffer = p_plain_data;
        p_aes->block_size           = number >> 4;
        p_aes->keyram_offset        = p_aes->init.key_slot;

        /* Configure AES registers */
        status = aes_config(p_aes);
        if (HAL_OK != status)
        {
            /* Set invalid param error code */
            p_aes->error_code |= HAL_AES_ERROR_INVALID_PARAM;

            return status;
        }

        /* AES MCU Process */
        status = aes_mcu_process(p_aes, ((timeout > 0U) ? timeout : 1U));
    }
    else
    {
        /* Set Busy error code */
        p_aes->error_code |= HAL_AES_ERROR_BUSY;
        status = HAL_ERROR;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_aes_ecb_encrypt_it(aes_handle_t *p_aes, uint32_t *p_plain_data, uint32_t number, uint32_t *p_cypher_data)
{
    hal_status_t status = HAL_OK;

    /* Check the AES handle allocation */
    if (NULL == p_aes)
    {
        return HAL_ERROR;
    }

    if (HAL_AES_STATE_READY == p_aes->state)
    {
        /* Change state Busy */
        p_aes->state = HAL_AES_STATE_BUSY;

        p_aes->operation_mode       = AES_OPERATION_MODE_ENCRYPT;
        p_aes->chaining_mode        = AES_CHAININGMODE_ECB;
        p_aes->p_cryp_input_buffer  = p_plain_data;
        p_aes->p_cryp_output_buffer = p_cypher_data;
        p_aes->block_size           = number >> 4;
        p_aes->keyram_offset        = p_aes->init.key_slot;

        /* Configure AES registers */
        status = aes_config(p_aes);
        if (HAL_OK != status)
        {
            /* Set invalid param error code */
            p_aes->error_code |= HAL_AES_ERROR_INVALID_PARAM;

            return status;
        }

        /* AES MCU Process */
        status = aes_mcu_process(p_aes, 0);
    }
    else
    {
        /* Set Busy error code */
        p_aes->error_code |= HAL_AES_ERROR_BUSY;
        status = HAL_ERROR;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_aes_ecb_decrypt_it(aes_handle_t *p_aes, uint32_t *p_cypher_data, uint32_t number, uint32_t *p_plain_data)
{
    hal_status_t status = HAL_OK;

    /* Check the AES handle allocation */
    if (NULL == p_aes)
    {
        return HAL_ERROR;
    }

    if (HAL_AES_STATE_READY == p_aes->state)
    {
        /* Change state Busy */
        p_aes->state = HAL_AES_STATE_BUSY;

        p_aes->operation_mode       = AES_OPERATION_MODE_DECRYPT;
        p_aes->chaining_mode        = AES_CHAININGMODE_ECB;
        p_aes->p_cryp_input_buffer  = p_cypher_data;
        p_aes->p_cryp_output_buffer = p_plain_data;
        p_aes->block_size           = number >> 4;
        p_aes->keyram_offset        = p_aes->init.key_slot;

        /* Configure AES registers */
        status = aes_config(p_aes);
        if (HAL_OK != status)
        {
            /* Set invalid param error code */
            p_aes->error_code |= HAL_AES_ERROR_INVALID_PARAM;

            return status;
        }

        /* AES MCU Process */
        status = aes_mcu_process(p_aes, 0);
    }
    else
    {
        /* Set Busy error code */
        p_aes->error_code |= HAL_AES_ERROR_BUSY;
        status = HAL_ERROR;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_aes_cbc_encrypt_it(aes_handle_t *p_aes, uint32_t *p_plain_data, uint32_t number, uint32_t *p_cypher_data)
{
    hal_status_t status = HAL_OK;

    /* Check the AES handle allocation */
    if (NULL == p_aes)
    {
        return HAL_ERROR;
    }

    if (HAL_AES_STATE_READY == p_aes->state)
    {
        /* Change state Busy */
        p_aes->state = HAL_AES_STATE_BUSY;

        p_aes->operation_mode       = AES_OPERATION_MODE_ENCRYPT;
        p_aes->chaining_mode        = AES_CHAININGMODE_CBC;
        p_aes->p_cryp_input_buffer  = p_plain_data;
        p_aes->p_cryp_output_buffer = p_cypher_data;
        p_aes->block_size           = number >> 4;
        p_aes->keyram_offset        = p_aes->init.key_slot;

        /* Configure AES registers */
        status = aes_config(p_aes);
        if (HAL_OK != status)
        {
            /* Set invalid param error code */
            p_aes->error_code |= HAL_AES_ERROR_INVALID_PARAM;

            return status;
        }

        /* AES MCU Process */
        status = aes_mcu_process(p_aes, 0);
    }
    else
    {
        /* Set Busy error code */
        p_aes->error_code |= HAL_AES_ERROR_BUSY;
        status = HAL_ERROR;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_aes_cbc_decrypt_it(aes_handle_t *p_aes, uint32_t *p_cypher_data, uint32_t number, uint32_t *p_plain_data)
{
    hal_status_t status = HAL_OK;

    /* Check the AES handle allocation */
    if (NULL == p_aes)
    {
        return HAL_ERROR;
    }

    if (HAL_AES_STATE_READY == p_aes->state)
    {
        /* Change state Busy */
        p_aes->state = HAL_AES_STATE_BUSY;

        p_aes->operation_mode       = AES_OPERATION_MODE_DECRYPT;
        p_aes->chaining_mode        = AES_CHAININGMODE_CBC;
        p_aes->p_cryp_input_buffer  = p_cypher_data;
        p_aes->p_cryp_output_buffer = p_plain_data;
        p_aes->block_size           = number >> 4;
        p_aes->keyram_offset        = p_aes->init.key_slot;

        /* Configure AES registers */
        status = aes_config(p_aes);
        if (HAL_OK != status)
        {
            /* Set invalid param error code */
            p_aes->error_code |= HAL_AES_ERROR_INVALID_PARAM;

            return status;
        }

        /* AES MCU Process */
        status = aes_mcu_process(p_aes, 0);
    }
    else
    {
        /* Set Busy error code */
        p_aes->error_code |= HAL_AES_ERROR_BUSY;
        status = HAL_ERROR;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_aes_ecb_encrypt_dma(aes_handle_t *p_aes, uint32_t *p_plain_data, uint32_t number, uint32_t *p_cypher_data)
{
    hal_status_t status = HAL_OK;

    /* Check the AES handle allocation */
    if (NULL == p_aes)
    {
        return HAL_ERROR;
    }

    if (HAL_AES_STATE_READY == p_aes->state)
    {
        /* Change state Busy */
        p_aes->state = HAL_AES_STATE_BUSY;

        p_aes->operation_mode       = AES_OPERATION_MODE_ENCRYPT;
        p_aes->chaining_mode        = AES_CHAININGMODE_ECB;
        p_aes->p_cryp_input_buffer  = p_plain_data;
        p_aes->p_cryp_output_buffer = p_cypher_data;
        p_aes->block_size           = number >> 4;

        /* Configure AES registers */
        status = aes_config(p_aes);
        if (HAL_OK != status)
        {
            /* Set invalid param error code */
            p_aes->error_code |= HAL_AES_ERROR_INVALID_PARAM;

            return status;
        }

        /* AES DMA Process */
        status = aes_dma_process(p_aes);
    }
    else
    {
        /* Set Busy error code */
        p_aes->error_code |= HAL_AES_ERROR_BUSY;
        status = HAL_ERROR;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_aes_ecb_decrypt_dma(aes_handle_t *p_aes, uint32_t *p_cypher_data, uint32_t number, uint32_t *p_plain_data)
{
    hal_status_t status = HAL_OK;

    /* Check the AES handle allocation */
    if (NULL == p_aes)
    {
        return HAL_ERROR;
    }

    if (HAL_AES_STATE_READY == p_aes->state)
    {
        /* Change state Busy */
        p_aes->state = HAL_AES_STATE_BUSY;

        p_aes->operation_mode       = AES_OPERATION_MODE_DECRYPT;
        p_aes->chaining_mode        = AES_CHAININGMODE_ECB;
        p_aes->p_cryp_input_buffer  = p_cypher_data;
        p_aes->p_cryp_output_buffer = p_plain_data;
        p_aes->block_size           = number >> 4;

        /* Configure AES registers */
        status = aes_config(p_aes);
        if (HAL_OK != status)
        {
            /* Set invalid param error code */
            p_aes->error_code |= HAL_AES_ERROR_INVALID_PARAM;
            return status;
        }

        /* AES DMA Process */
        status = aes_dma_process(p_aes);
    }
    else
    {
        /* Set Busy error code */
        p_aes->error_code |= HAL_AES_ERROR_BUSY;
        status = HAL_ERROR;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_aes_cbc_encrypt_dma(aes_handle_t *p_aes, uint32_t *p_plain_data, uint32_t number, uint32_t *p_cypher_data)
{
    hal_status_t status = HAL_OK;

    /* Check the AES handle allocation */
    if (NULL == p_aes)
    {
        return HAL_ERROR;
    }

    if (HAL_AES_STATE_READY == p_aes->state)
    {
        /* Change state Busy */
        p_aes->state = HAL_AES_STATE_BUSY;

        p_aes->operation_mode       = AES_OPERATION_MODE_ENCRYPT;
        p_aes->chaining_mode        = AES_CHAININGMODE_CBC;
        p_aes->p_cryp_input_buffer  = p_plain_data;
        p_aes->p_cryp_output_buffer = p_cypher_data;
        p_aes->block_size           = number >> 4;

        /* Configure AES registers */
        status = aes_config(p_aes);
        if (HAL_OK != status)
        {
            /* Set invalid param error code */
            p_aes->error_code |= HAL_AES_ERROR_INVALID_PARAM;

            return status;
        }

        /* AES DMA Process */
        status = aes_dma_process(p_aes);
    }
    else
    {
        /* Set Busy error code */
        p_aes->error_code |= HAL_AES_ERROR_BUSY;
        status = HAL_ERROR;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_aes_cbc_decrypt_dma(aes_handle_t *p_aes, uint32_t *p_cypher_data, uint32_t number, uint32_t *p_plain_data)
{
    hal_status_t status = HAL_OK;

    /* Check the AES handle allocation */
    if (NULL == p_aes)
    {
        return HAL_ERROR;
    }

    if (HAL_AES_STATE_READY == p_aes->state)
    {
        /* Change state Busy */
        p_aes->state = HAL_AES_STATE_BUSY;

        p_aes->operation_mode       = AES_OPERATION_MODE_DECRYPT;
        p_aes->chaining_mode        = AES_CHAININGMODE_CBC;
        p_aes->p_cryp_input_buffer  = p_cypher_data;
        p_aes->p_cryp_output_buffer = p_plain_data;
        p_aes->block_size           = number >> 4;

        /* Configure AES registers */
        status = aes_config(p_aes);
        if (HAL_OK != status)
        {
            /* Set invalid param error code */
            p_aes->error_code |= HAL_AES_ERROR_INVALID_PARAM;

            return status;
        }

        /* AES DMA Process */
        status = aes_dma_process(p_aes);
    }
    else
    {
        /* Set Busy error code */
        p_aes->error_code |= HAL_AES_ERROR_BUSY;
        status = HAL_ERROR;
    }

    /* Return function status */
    return status;
}

__WEAK void hal_aes_irq_handler(aes_handle_t *p_aes)
{
    __HAL_AES_CLEAR_FLAG_IT(p_aes, AES_IT_DONE);

    /* Check if DMA transfer error occurred */
    if (RESET != __HAL_AES_GET_FLAG(p_aes, AES_FLAG_DMA_ERR))
    {
        ll_aes_disable_dma_start(p_aes->p_instance);

        ll_aes_disable_it_done(p_aes->p_instance);

        if (HAL_AES_STATE_BUSY == p_aes->state)
        {
            p_aes->error_code |= HAL_AES_ERROR_TRANSFER;
            p_aes->state = HAL_AES_STATE_READY;
            hal_aes_error_callback(p_aes);
        }

        return;
    }

    /* AES DMA Mode done */
    if (RESET != __HAL_AES_GET_FLAG(p_aes, AES_FLAG_DMA_DONE))
    {
        ll_aes_disable_dma_start(p_aes->p_instance);

        ll_aes_disable_it_done(p_aes->p_instance);

        if (HAL_AES_STATE_BUSY == p_aes->state)
        {
            p_aes->state = HAL_AES_STATE_READY;
            hal_aes_done_callback(p_aes);
        }
    }

    /* AES IT Mode done */
    if (RESET != __HAL_AES_GET_FLAG(p_aes, AES_FLAG_DATAREADY))
    {
        if (LL_AES_KEYMODE_NORMAL == p_aes->init.key_mode)
        {
            /* MCU read AES output */
            aes_read_data(p_aes, p_aes->p_cryp_output_buffer);
            p_aes->p_cryp_output_buffer += AES_BLOCKSIZE_WORDS;
        }
        else if (LL_AES_KEYMODE_KEYWRAP == p_aes->init.key_mode)
        {
            /* Update output KEYRAM offset */
            p_aes->keyram_offset = p_aes->keyram_offset + AES_BLOCKSIZE_BYTES;
            ll_aes_set_keyram_offset(p_aes->p_instance, p_aes->keyram_offset);
        }
        else
        {
            /* Nothing to do */
        }

        p_aes->block_size--;
        ll_aes_disable_start(p_aes->p_instance);

        if (0U < p_aes->block_size)
        {
            aes_write_data(p_aes, p_aes->p_cryp_input_buffer);
            p_aes->p_cryp_input_buffer += AES_BLOCKSIZE_WORDS;

            ll_aes_enable_start(p_aes->p_instance);
        }
        else
        {
            ll_aes_disable_it_done(p_aes->p_instance);

            if (HAL_AES_STATE_BUSY == p_aes->state)
            {
                p_aes->state = HAL_AES_STATE_READY;
                hal_aes_done_callback(p_aes);
            }
        }
    }
}

__WEAK void hal_aes_done_callback(aes_handle_t *p_aes)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_aes);
    return;
}

__WEAK void hal_aes_error_callback(aes_handle_t *p_aes)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_aes);
    return;
}

__WEAK hal_aes_state_t hal_aes_get_state(aes_handle_t *p_aes)
{
    return p_aes->state;
}

__WEAK uint32_t hal_aes_get_error(aes_handle_t *p_aes)
{
    return p_aes->error_code;
}

static hal_status_t aes_config(aes_handle_t *p_aes)
{
    hal_status_t status = HAL_OK;

    /* Disable AES start */
    ll_aes_disable_start(p_aes->p_instance);
    ll_aes_disable_dma_start(p_aes->p_instance);

    do {
        /* Set AES endian */
        if (AES_ENDIAN_MODE_BIG == p_aes->init.endian_mode)
        {
            /* Set big endian */
            ll_aes_disable_little_endian(p_aes->p_instance);
        }
        else
        {
            /* Set small endian */
            ll_aes_enable_little_endian(p_aes->p_instance);
        }

        /* Set AES Operation Mode */
        if (AES_OPERATION_MODE_ENCRYPT == p_aes->operation_mode)
        {
            ll_aes_enable_encryption(p_aes->p_instance);
        }
        else
        {
            ll_aes_disable_encryption(p_aes->p_instance);
        }

        /* Set ECB or CBC */
        ll_aes_set_operation_mode(p_aes->p_instance, p_aes->chaining_mode);

        /* Set key mode */
        ll_aes_set_key_mode(p_aes->p_instance, p_aes->init.key_mode);

        if (LL_AES_KEYMODE_KEYWRAP == p_aes->init.key_mode)
        {
            /* Set output keyram offset */
            ll_aes_set_keyram_offset(p_aes->p_instance, p_aes->init.key_slot);
        }

        /* Set key size */
        ll_aes_set_key_size(p_aes->p_instance, p_aes->init.key_size);

        /* Set key type */
        ll_aes_set_key_type(p_aes->p_instance, p_aes->init.key_type);

        if (AES_KEY_TYPE_MCU == p_aes->init.key_type)
        {
            /* Fetch Key from MCU */
            if (NULL == p_aes->init.p_key)
            {
                status = HAL_ERROR;
                break;
            }
            switch (p_aes->init.key_size)
            {
                //lint -e825 -e9090 NO break in this case
                case AES_KEYSIZE_256BITS:
                    ll_aes_set_key_31_0   (p_aes->p_instance, p_aes->init.p_key[7]);
                    ll_aes_set_key_63_32  (p_aes->p_instance, p_aes->init.p_key[6]);
                //lint -e825 -e9090 NO break in this case
                case AES_KEYSIZE_192BITS:
                    ll_aes_set_key_95_64  (p_aes->p_instance, p_aes->init.p_key[5]);
                    ll_aes_set_key_127_96 (p_aes->p_instance, p_aes->init.p_key[4]);
                //lint -e825 -e9090 NO break in this case
                case AES_KEYSIZE_128BITS:
                    ll_aes_set_key_159_128(p_aes->p_instance, p_aes->init.p_key[3]);
                    ll_aes_set_key_191_160(p_aes->p_instance, p_aes->init.p_key[2]);
                    ll_aes_set_key_223_192(p_aes->p_instance, p_aes->init.p_key[1]);
                    ll_aes_set_key_255_224(p_aes->p_instance, p_aes->init.p_key[0]);
                    break;
                default:
                    /* Nothing to do */
                    break;
            }
        }
        else if (AES_KEY_TYPE_AHB == p_aes->init.key_type)
        {
            /* Fetch Key from AHB */
            ll_aes_set_key_address(p_aes->p_instance, p_aes->init.key_addr);
            ll_aes_enable_read_key(p_aes->p_instance);
            while(0U == ll_aes_is_action_flag_key_valid(p_aes->p_instance)){}
        }
        else if (AES_KEY_TYPE_KEYRAM == p_aes->init.key_type)
        {
            /* Fetch Key from KEYRAM */
            ll_aes_set_key_port_mask(p_aes->p_instance, p_aes->init.key_mask);
            ll_aes_set_key_address(p_aes->p_instance, p_aes->init.key_addr);
            ll_aes_enable_read_key(p_aes->p_instance);
            while(0U == ll_aes_is_action_flag_key_valid(p_aes->p_instance)){}
        }
        else 
        {
            /* Nothing to do */
        }

        /* Set initial vector in CBC mode */
        if (AES_CHAININGMODE_CBC == p_aes->chaining_mode)
        {
            if (NULL == p_aes->init.p_init_vector)
            {
                status = HAL_ERROR;
                break;
            }
            ll_aes_set_vector_127_96(p_aes->p_instance, p_aes->init.p_init_vector[0]);
            ll_aes_set_vector_95_64 (p_aes->p_instance, p_aes->init.p_init_vector[1]);
            ll_aes_set_vector_63_32 (p_aes->p_instance, p_aes->init.p_init_vector[2]);
            ll_aes_set_vector_31_0  (p_aes->p_instance, p_aes->init.p_init_vector[3]);
        }

        /* Set DPA Resistence */
        if (AES_DPA_MODE_ENABLE == p_aes->init.dpa_mode)
        {
            if (NULL == p_aes->init.p_seed)
            {
                status = HAL_ERROR;
                break;
            }
            ll_aes_enable_full_mask(p_aes->p_instance);
            ll_aes_set_seed_in (p_aes->p_instance, p_aes->init.p_seed[0]);
            ll_aes_set_seed_out(p_aes->p_instance, p_aes->init.p_seed[1]);
            ll_aes_set_seed_Imask(p_aes->p_instance, p_aes->init.p_seed[2]);
            ll_aes_set_seed_Osbox(p_aes->p_instance, p_aes->init.p_seed[3]);
        }
        else
        {
            ll_aes_disable_full_mask(p_aes->p_instance);
        }

        /* Disable interrupt and clear flag */
        p_aes->p_instance->INT = 1U;

    } while(0);

    return status;
}

static hal_status_t aes_wait_flag_state_until_timeout(aes_handle_t *p_aes, uint32_t flag, flag_status_t state, uint32_t timeout)
{
    hal_status_t ret = HAL_OK;
    if((HAL_NEVER_TIMEOUT != timeout) && (HAL_MAX_DELAY < timeout))
    {
        return HAL_ERROR;
    }

    HAL_TIMEOUT_INIT();
    //lint -e923 Cast from pointer to unsigned int is necessary
    uint32_t tickstart = HAL_TIMEOUT_GET_TICK();
    /* Update SystemCoreClock */
    SystemCoreUpdateClock();
    uint32_t timeout_counter = ((SystemCoreClock / 1000U) * timeout);

    /* Wait until flag is in expected state */
    while ((__HAL_AES_GET_FLAG(p_aes, flag)) != state)
    {
        if(HAL_NEVER_TIMEOUT != timeout)
        {
            //lint -e923 Cast from pointer to unsigned int is necessary
            if((HAL_TIMEOUT_GET_TICK() - tickstart) > timeout_counter)
            {
                ret = HAL_TIMEOUT;
                goto EXIT;
            }
        }
    }
EXIT:
    HAL_TIMEOUT_DEINIT();
    return ret;
}

static void aes_read_data(aes_handle_t *p_aes, uint32_t *p_data)
{
    p_data[0] = p_aes->p_instance->DATA_OUT0;
    p_data[1] = p_aes->p_instance->DATA_OUT1;
    p_data[2] = p_aes->p_instance->DATA_OUT2;
    p_data[3] = p_aes->p_instance->DATA_OUT3;
}

static void aes_write_data(aes_handle_t *p_aes, uint32_t *p_data)
{
    p_aes->p_instance->DATA_IN0 = p_data[0];
    p_aes->p_instance->DATA_IN1 = p_data[1];
    p_aes->p_instance->DATA_IN2 = p_data[2];
    p_aes->p_instance->DATA_IN3 = p_data[3];
}

static hal_status_t aes_mcu_process(aes_handle_t *p_aes, uint32_t timeout)
{
    hal_status_t status = HAL_OK;

    /* Set DPA Resistence */
    if (AES_DPA_MODE_ENABLE == p_aes->init.dpa_mode)
    {
        ll_aes_set_load_seed(p_aes->p_instance);
    }

     /* Set CBC First Block Flag */
    if (AES_CHAININGMODE_CBC == p_aes->chaining_mode)
    {
        ll_aes_clr_first_block(p_aes->p_instance);
        ll_aes_set_first_block(p_aes->p_instance);
    }

    if (0U < timeout)
    {
        /* MCU process in polling */
        while (p_aes->block_size)
        {
            /* MCU write AES input */
            aes_write_data(p_aes, p_aes->p_cryp_input_buffer);
            p_aes->p_cryp_input_buffer += AES_BLOCKSIZE_WORDS;

            /* AES start enable */
            ll_aes_enable_start(p_aes->p_instance);

            /* AES wait for completion */
            status = aes_wait_flag_state_until_timeout(p_aes, AES_FLAG_DATAREADY, SET, timeout);
            if (HAL_OK != status)
            {
                p_aes->error_code |= HAL_AES_ERROR_TIMEOUT;
                break;
            }

            if (LL_AES_KEYMODE_NORMAL == p_aes->init.key_mode)
            {
                /* MCU read AES output */
                aes_read_data(p_aes, p_aes->p_cryp_output_buffer);
                p_aes->p_cryp_output_buffer += AES_BLOCKSIZE_WORDS;
            }
            else if (LL_AES_KEYMODE_KEYWRAP == p_aes->init.key_mode)
            {
                /* Update output KEYRAM offset */
                p_aes->keyram_offset = p_aes->keyram_offset + AES_BLOCKSIZE_BYTES;
                ll_aes_set_keyram_offset(p_aes->p_instance, p_aes->keyram_offset);
            }
            else
            {
                /* Nothing to do */
            }

            p_aes->block_size--;

            /* AES start disable */
            ll_aes_disable_start(p_aes->p_instance);
        }

        /* Change state Ready */
        p_aes->state = HAL_AES_STATE_READY;
    }
    else
    {
        /* MCU process in IT */
        /* MCU write AES input */
        aes_write_data(p_aes, p_aes->p_cryp_input_buffer);
        p_aes->p_cryp_input_buffer += AES_BLOCKSIZE_WORDS;

        /* AES IT enable */
        ll_aes_enable_it_done(p_aes->p_instance);

        /* AES start enable */
        ll_aes_enable_start(p_aes->p_instance);
    }

    return status;
}

static hal_status_t aes_dma_process(aes_handle_t *p_aes)
{
    hal_status_t status = HAL_OK;

    /* Set DPA Resistence */
    if (AES_DPA_MODE_ENABLE == p_aes->init.dpa_mode)
    {
        ll_aes_set_load_seed(p_aes->p_instance);
    }

     /* Set CBC First Block Flag */
    if (AES_CHAININGMODE_CBC == p_aes->chaining_mode)
    {
        ll_aes_clr_first_block(p_aes->p_instance);
        ll_aes_set_first_block(p_aes->p_instance);
    }

    /* Set AES DMA Transfer Block Size */
    ll_aes_set_dma_transfer_block(p_aes->p_instance, p_aes->block_size);

    /* Set AES DMA input address */
    //lint -e923 Cast from pointer to unsigned int is necessary
    ll_aes_set_dma_read_address(p_aes->p_instance, (uint32_t)p_aes->p_cryp_input_buffer);

    /* Set AES DMA output address */
    //lint -e923 Cast from pointer to unsigned int is necessary
    ll_aes_set_dma_write_address(p_aes->p_instance, (uint32_t)p_aes->p_cryp_output_buffer);

    /* AES IT enable */
    ll_aes_enable_it_done(p_aes->p_instance);

    /* AES DMA start enable */
    ll_aes_enable_dma_start(p_aes->p_instance);

    return status;
}

#endif /* HAL_AES_MODULE_ENABLED */

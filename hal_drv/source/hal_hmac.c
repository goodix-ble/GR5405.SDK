/**
  ****************************************************************************************
  * @file    gr5xx_hal_hmac.c
  * @author  BLE Driver Team
  * @brief   HMAC HAL module driver.
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
#ifdef HAL_HMAC_MODULE_ENABLED
#include "hal_def.h"
#include "hal_hmac.h"
#include <string.h>

#ifdef HAL_CLOCK_UNIFORM_CONTROL
#include "hal_clock.h"
#endif

static hal_status_t hmac_wait_flag_state_until_timeout(hmac_handle_t *p_hmac, uint32_t flag, flag_status_t state, uint32_t timeout);
static void hmac_read_data(hmac_handle_t *p_hmac, uint32_t *p_data);
static void hmac_write_data(hmac_handle_t *p_hmac, uint32_t *p_data);
static hal_status_t hmac_mcu_process(hmac_handle_t *p_hmac, uint32_t timeout);
static hal_status_t hmac_dma_process(hmac_handle_t *p_hmac);
static hal_status_t hmac_config(hmac_handle_t *p_hmac);
static void hal_hmac_swap_endian(uint32_t *in, uint32_t len, uint32_t *out);
static uint32_t hal_hmac_preprocess_big_endian(uint8_t *in, uint32_t len, uint32_t *out, uint32_t mode);

__WEAK hal_status_t hal_hmac_init(hmac_handle_t *p_hmac)
{
    hal_status_t status = HAL_OK;

    if (HAL_HMAC_STATE_RESET == p_hmac->state)
    {
#ifdef HAL_CLOCK_UNIFORM_CONTROL
        //lint -e923 Cast from pointer to unsigned int is necessary
        hal_clock_enable_module((uint32_t)p_hmac->p_instance);
#endif

        /* Enable security blocks clock and Automatic turn off security blocks clock during WFI. */
//        ll_cgc_disable_force_off_secu_hclk();
//        ll_cgc_disable_wfi_off_secu_hclk();

//        ll_cgc_disable_force_off_hmac_hclk();
//        ll_cgc_disable_force_off_secu_div4_pclk();
//        ll_cgc_disable_wfi_off_hmac_hclk();
//        ll_cgc_disable_wfi_off_secu_div4_hclk();

        /* Init the low level hardware : CLOCK, NVIC */
        hal_hmac_msp_init(p_hmac);
    }

    /* Set HMAC error code to none */
    p_hmac->error_code = HAL_HMAC_ERROR_NONE;

    /* Initialize the HMAC state */
    p_hmac->state = HAL_HMAC_STATE_READY;

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_hmac_deinit(hmac_handle_t *p_hmac)
{
    hal_status_t status = HAL_OK;

    /* Check the HMAC handle allocation */
    if (NULL == p_hmac)
    {
        return HAL_ERROR;
    }

    /* Set HMAC registers to their reset values */
    p_hmac->p_instance->CTRL      = 0U;
    p_hmac->p_instance->CFG       = 0U;
    p_hmac->p_instance->INT       = 1U;

#ifdef HAL_CLOCK_UNIFORM_CONTROL
    //lint -e923 Cast from pointer to unsigned int is necessary
    hal_clock_disable_module((uint32_t)p_hmac->p_instance);
#endif
//    ll_cgc_enable_force_off_hmac_hclk();
//    ll_cgc_enable_wfi_off_hmac_hclk();

    /* Disable the HMAC Peripheral Clock */
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

    /* DeInit the low level hardware: CLOCK, NVIC... */
    hal_hmac_msp_deinit(p_hmac);

    /* Set HMAC error code to none */
    p_hmac->error_code = HAL_HMAC_ERROR_NONE;

    /* Initialize the HMAC state */
    p_hmac->state = HAL_HMAC_STATE_RESET;

    /* Return function status */
    return status;
}

__WEAK void hal_hmac_msp_init(hmac_handle_t *p_hmac)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_hmac);
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
            the hal_hmac_msp_init can be implemented in the user file
     */
}

__WEAK void hal_hmac_msp_deinit(hmac_handle_t *p_hmac)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_hmac);
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
            the hal_hmac_msp_deinit can be implemented in the user file
     */
}

__WEAK hal_status_t hal_hmac_sha256_digest(hmac_handle_t *p_hmac, uint32_t *p_message, uint32_t number, uint32_t *p_digest, uint32_t timeout)
{
    hal_status_t status = HAL_OK;
    uint32_t input_rem[32] = { 0 };
    uint32_t padded_block_size = 0;
    uint32_t process_block_size = number / HMAC_BLOCKSIZE_BYTES;  //512 bits per block

    /* Check the HMAC handle allocation */
    if ((NULL == p_message) || (NULL == p_digest))
    {
        return HAL_ERROR;
    }

    if (HAL_HMAC_STATE_READY == p_hmac->state)
    {
        /* Change state Busy */
        p_hmac->state = HAL_HMAC_STATE_BUSY;

        /* Configure HMAC registers */
        status = hmac_config(p_hmac);
        if (HAL_OK != status)
        {
            /* Set invalid param error code */
            p_hmac->error_code |= HAL_HMAC_ERROR_INVALID_PARAM;

            return status;
        }

        /* Input data preprocessing */
        //lint -e934 Taking address of near auto variable is necessary
        padded_block_size = hal_hmac_preprocess_big_endian((uint8_t *)p_message, number, input_rem, p_hmac->init.mode);
        hal_hmac_swap_endian(input_rem, 32U, input_rem);

        p_hmac->p_message     = p_message;
        p_hmac->p_digest      = NULL;
        p_hmac->block_size    = process_block_size;
        p_hmac->is_last_trans = 0U;

        /* HMAC MCU Process */
        if (process_block_size > 0U)
        {
            status = hmac_mcu_process(p_hmac, timeout);
            if (HAL_OK != status)
            {
                return status;
            }
        }

        /* Calculate the last block */
        p_hmac->p_message     = input_rem;
        p_hmac->p_digest      = p_digest;
        p_hmac->block_size    = padded_block_size;
        p_hmac->is_last_trans = 1U;

        /* HMAC MCU Process */
        status = hmac_mcu_process(p_hmac, timeout);
    }
    else
    {
        /* Set Busy error code */
        p_hmac->error_code |= HAL_HMAC_ERROR_BUSY;
        status = HAL_ERROR;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_hmac_sha256_digest_it(hmac_handle_t *p_hmac, uint32_t *p_message, uint32_t number, uint32_t *p_digest)
{
    hal_status_t status = HAL_OK;
    uint32_t input_rem[32] = { 0 };
    uint32_t padded_block_size = 0;
    uint32_t process_block_size = number / HMAC_BLOCKSIZE_BYTES;  //512 bits per block

    /* Check the HMAC handle allocation */
    if ((NULL == p_message) || (NULL == p_digest))
    {
        return HAL_ERROR;
    }

    if (HAL_HMAC_STATE_READY == p_hmac->state)
    {
        /* Change state Busy */
        p_hmac->state = HAL_HMAC_STATE_BUSY;

        /* Configure HMAC registers */
        status = hmac_config(p_hmac);
        if (HAL_OK != status)
        {
            /* Set invalid param error code */
            p_hmac->error_code |= HAL_HMAC_ERROR_INVALID_PARAM;

            return status;
        }

        /* Input data preprocessing */
        padded_block_size = hal_hmac_preprocess_big_endian((uint8_t *)p_message, number, input_rem, p_hmac->init.mode);
        hal_hmac_swap_endian(input_rem, 32, input_rem);

        p_hmac->p_message     = p_message;
        p_hmac->p_digest      = NULL;
        p_hmac->block_size    = process_block_size;
        p_hmac->is_last_trans = 0U;

        /* HMAC MCU Process */
        if (process_block_size > 0U)
        {
            p_hmac->it_flag = 0U;
            status = hmac_mcu_process(p_hmac, 0U);
            if (HAL_OK != status)
            {
                return status;
            }
            while(0U == p_hmac->it_flag){}
        }

        /* Calculate the last block */
        p_hmac->p_message     = input_rem;
        p_hmac->p_digest      = p_digest;
        p_hmac->block_size    = padded_block_size;
        p_hmac->is_last_trans = 1U;

        /* HMAC MCU Process */
        status = hmac_mcu_process(p_hmac, 0U);
    }
    else
    {
        /* Set Busy error code */
        p_hmac->error_code |= HAL_HMAC_ERROR_BUSY;
        status = HAL_ERROR;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_hmac_sha256_digest_dma(hmac_handle_t *p_hmac, uint32_t *p_message, uint32_t number, uint32_t *p_digest)
{
    hal_status_t status = HAL_OK;
    uint32_t input_rem[32] = { 0 };
    uint32_t padded_block_size = 0U;
    uint32_t process_block_size = number / HMAC_BLOCKSIZE_BYTES;  //512 bits per block

    /* Check the HMAC handle allocation */
    if ((NULL == p_message) || (NULL == p_digest))
    {
        return HAL_ERROR;
    }

    if (HAL_HMAC_STATE_READY == p_hmac->state)
    {
        /* Change state Busy */
        p_hmac->state = HAL_HMAC_STATE_BUSY;

        /* Configure HMAC registers */
        status = hmac_config(p_hmac);
        if (HAL_OK != status)
        {
            /* Set invalid param error code */
            p_hmac->error_code |= HAL_HMAC_ERROR_INVALID_PARAM;

            return status;
        }

        /* Input data preprocessing */
        padded_block_size = hal_hmac_preprocess_big_endian((uint8_t *)p_message, number, input_rem, p_hmac->init.mode);
        hal_hmac_swap_endian(input_rem, 32, input_rem);

        p_hmac->p_message     = p_message;
        p_hmac->p_digest      = NULL;
        p_hmac->block_size    = process_block_size;
        p_hmac->is_last_trans = 0U;

        /* HMAC DMA Process */
        if (process_block_size > 0U)
        {
            p_hmac->it_flag = 0U;
            status = hmac_dma_process(p_hmac);
            if (HAL_OK != status)
            {
                return status;
            }
            while(0U == p_hmac->it_flag){}
        }

        /* Calculate the last block */
        p_hmac->p_message     = input_rem;
        p_hmac->p_digest      = p_digest;
        p_hmac->block_size    = padded_block_size;
        p_hmac->is_last_trans = 1U;

        /* HMAC DMA Process */
        status = hmac_dma_process(p_hmac);
    }
    else
    {
        /* Set Busy error code */
        p_hmac->error_code |= HAL_HMAC_ERROR_BUSY;
        status = HAL_ERROR;
    }

    /* Return function status */
    return status;
}

__WEAK void hal_hmac_irq_handler(hmac_handle_t *p_hmac)
{
    __HAL_HMAC_CLEAR_FLAG_IT(p_hmac, HMAC_IT_DONE);

    /* Check if DMA transfer error occurred */
    if (RESET != __HAL_HMAC_GET_FLAG(p_hmac, HMAC_FLAG_DMA_ERR))
    {
        /* Disable HMAC */
        ll_hmac_disable_dma_start(p_hmac->p_instance);
        ll_hmac_disable_it_done(p_hmac->p_instance);
        ll_hmac_disable(p_hmac->p_instance);

        if (HAL_HMAC_STATE_BUSY == p_hmac->state)
        {
            p_hmac->error_code |= HAL_HMAC_ERROR_TRANSFER;
            p_hmac->state = HAL_HMAC_STATE_READY;

            hal_hmac_error_callback(p_hmac);
        }

        return;
    }

    /* Done in SHA/HMAC mode */
    if ((RESET != __HAL_HMAC_GET_FLAG(p_hmac, HMAC_FLAG_DATAREADY_SHA)) || \
            (RESET != __HAL_HMAC_GET_FLAG(p_hmac, HMAC_FLAG_DATAREADY_HMAC)))
    {
        if (ll_hmac_is_enabled_dma_start(p_hmac->p_instance))
        {
            /* DMA mode */
            if (1U == p_hmac->is_last_trans)
            {
                /* Disable HMAC */
                ll_hmac_disable_dma_start(p_hmac->p_instance);
                ll_hmac_disable_it_done(p_hmac->p_instance);
                ll_hmac_disable(p_hmac->p_instance);

                /* HMAC Done Callback */
                if (HAL_HMAC_STATE_BUSY == p_hmac->state)
                {
                    p_hmac->state = HAL_HMAC_STATE_READY;
#ifdef HAL_HMAC_MODULE_STREAM_ENABLED
                    p_hmac->strm_done_flag = 1U;
#endif
                    hal_hmac_done_callback(p_hmac);
                }
            }
            else
            {
                if ((p_hmac->block_size > 0U) && (0U == p_hmac->is_last_trans))
                {
                    /* DMA start disable */
                    ll_hmac_disable_dma_start(p_hmac->p_instance);

                    /* Set DMA read and write address */
                    ll_hmac_set_dma_read_address(p_hmac->p_instance, (uint32_t)p_hmac->p_message);
                    ll_hmac_set_dma_write_address(p_hmac->p_instance, (uint32_t)p_hmac->p_digest);

                    /* DMA Single Transfer Max 512 Blocks */
                    if (p_hmac->block_size > HMAC_DMA_BLOCK_MAX)
                    {
                        ll_hmac_set_dma_transfer_block(p_hmac->p_instance, HMAC_DMA_BLOCK_MAX);
                        p_hmac->p_message += HMAC_DMA_BLOCK_MAX * HMAC_BLOCKSIZE_WORDS;
                        p_hmac->block_size -= HMAC_DMA_BLOCK_MAX;
                    }
                    else
                    {
                        ll_hmac_set_dma_transfer_block(p_hmac->p_instance, p_hmac->block_size);
                        p_hmac->p_message += p_hmac->block_size * HMAC_BLOCKSIZE_WORDS;
                        p_hmac->block_size -= p_hmac->block_size;
                    }

                    /* DMA start enable */
                    ll_hmac_enable_dma_start(p_hmac->p_instance);
                }
                else
                {
                    /* Non-last block done */
                    ll_hmac_disable_dma_start(p_hmac->p_instance);
                    ll_hmac_disable_it_done(p_hmac->p_instance);
                    p_hmac->it_flag = 1U;
                }
            }
        }
        else
        {
            /* MCU mode */
            p_hmac->block_size--;

            if (0U < p_hmac->block_size)
            {
                /* Enable last transfer for the last block */
                if ((1U == p_hmac->block_size) && (1U == p_hmac->is_last_trans))
                {
                    ll_hmac_enable_last_transfer(p_hmac->p_instance);
                }

                /* MCU write HMAC input */
                hmac_write_data(p_hmac, p_hmac->p_message);
                p_hmac->p_message += HMAC_BLOCKSIZE_WORDS;
            }
            else
            {
                if (1U == p_hmac->is_last_trans)
                {
                    /* MCU read HMAC input */
                    hmac_read_data(p_hmac, p_hmac->p_digest);

                    /* Disable HMAC */
                    ll_hmac_disable_it_done(p_hmac->p_instance);
                    ll_hmac_disable(p_hmac->p_instance);

                    /* HMAC Done Callback */
                    if (HAL_HMAC_STATE_BUSY == p_hmac->state)
                    {
                        p_hmac->state = HAL_HMAC_STATE_READY;
                        hal_hmac_done_callback(p_hmac);
                    }
                }
                else
                {
                    /* Non-last block done */
                    ll_hmac_disable_it_done(p_hmac->p_instance);
                    p_hmac->it_flag = 1U;
                }
            }
        }
    }
}

__WEAK void hal_hmac_done_callback(hmac_handle_t *p_hmac)
{
    UNUSED(p_hmac);
}

__WEAK void hal_hmac_error_callback(hmac_handle_t *p_hmac)
{
    UNUSED(p_hmac);
}

__WEAK hal_hmac_state_t hal_hmac_get_state(hmac_handle_t *p_hmac)
{
    return p_hmac->state;
}

__WEAK uint32_t hal_hmac_get_error(hmac_handle_t *p_hmac)
{
    return p_hmac->error_code;
}

static hal_status_t hmac_wait_flag_state_until_timeout(hmac_handle_t *p_hmac, uint32_t flag, flag_status_t state, uint32_t timeout)
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
    while ((__HAL_HMAC_GET_FLAG(p_hmac, flag)) != state)
    {
        if(HAL_NEVER_TIMEOUT != timeout)
        {
            if((HAL_TIMEOUT_GET_TICK() - tickstart) > timeout_counter)
            {
                p_hmac->error_code |= HAL_HMAC_ERROR_TIMEOUT;
                HAL_TIMEOUT_DEINIT();
                return HAL_TIMEOUT;
            }
        }
    }

    HAL_TIMEOUT_DEINIT();
    return HAL_OK;
}

static hal_status_t hmac_config(hmac_handle_t *p_hmac)
{
    hal_status_t status = HAL_OK;

    /* Disable HMAC start */
    ll_hmac_disable(p_hmac->p_instance);
    ll_hmac_disable_dma_start(p_hmac->p_instance);

    do {
        /* Set SHA or HMAC */
        if (HMAC_MODE_SHA == p_hmac->init.mode)
        {
            ll_hmac_enable_sha(p_hmac->p_instance);
        }
        else
        {
            ll_hmac_disable_sha(p_hmac->p_instance);

            /* Set HMAC key type */
            ll_hmac_set_key_type(p_hmac->p_instance, p_hmac->init.key_type);

            /* Set HMAC key */
            if (HAL_HMAC_KEYTYPE_MCU == p_hmac->init.key_type)
            {
                /* Fetch Key from MCU */
                if (NULL == p_hmac->init.p_key)
                {
                    status = HAL_ERROR;
                    break;
                }
                ll_hmac_set_key0(p_hmac->p_instance, p_hmac->init.p_key[0]);
                ll_hmac_set_key1(p_hmac->p_instance, p_hmac->init.p_key[1]);
                ll_hmac_set_key2(p_hmac->p_instance, p_hmac->init.p_key[2]);
                ll_hmac_set_key3(p_hmac->p_instance, p_hmac->init.p_key[3]);
                ll_hmac_set_key4(p_hmac->p_instance, p_hmac->init.p_key[4]);
                ll_hmac_set_key5(p_hmac->p_instance, p_hmac->init.p_key[5]);
                ll_hmac_set_key6(p_hmac->p_instance, p_hmac->init.p_key[6]);
                ll_hmac_set_key7(p_hmac->p_instance, p_hmac->init.p_key[7]);
            }
            else if (HAL_HMAC_KEYTYPE_AHB == p_hmac->init.key_type)
            {
                /* Fetch Key from AHB */
                ll_hmac_set_key_address(p_hmac->p_instance, p_hmac->init.key_addr);
                ll_hmac_enable_read_key(p_hmac->p_instance);
                while(0U == ll_hmac_is_action_flag_key_valid(p_hmac->p_instance)){}
            }
            else
            {
                /* Fetch Key from KEYRAM */
                ll_hmac_set_key_port_mask(p_hmac->p_instance, p_hmac->init.key_mask);
                ll_hmac_set_key_address(p_hmac->p_instance, p_hmac->init.key_addr);
                ll_hmac_enable_read_key(p_hmac->p_instance);
                while(0U == ll_hmac_is_action_flag_key_valid(p_hmac->p_instance)){}
            }
        }

        /* Set initial HASH in user HASH mode */
        if (NULL != p_hmac->init.p_user_hash)
        {
            ll_hmac_enable_user_hash(p_hmac->p_instance);
            ll_hmac_set_user_hash_255_224(p_hmac->p_instance, p_hmac->init.p_user_hash[0]);
            ll_hmac_set_user_hash_223_192(p_hmac->p_instance, p_hmac->init.p_user_hash[1]);
            ll_hmac_set_user_hash_191_160(p_hmac->p_instance, p_hmac->init.p_user_hash[2]);
            ll_hmac_set_user_hash_159_128(p_hmac->p_instance, p_hmac->init.p_user_hash[3]);
            ll_hmac_set_user_hash_127_96 (p_hmac->p_instance, p_hmac->init.p_user_hash[4]);
            ll_hmac_set_user_hash_95_64  (p_hmac->p_instance, p_hmac->init.p_user_hash[5]);
            ll_hmac_set_user_hash_63_32  (p_hmac->p_instance, p_hmac->init.p_user_hash[6]);
            ll_hmac_set_user_hash_31_0   (p_hmac->p_instance, p_hmac->init.p_user_hash[7]);
        }
        else
        {
            ll_hmac_disable_user_hash(p_hmac->p_instance);
        }

        /* Set DPA Resistence */
        if (HAL_HMAC_DPA_MODE_ENABLE == p_hmac->init.dpa_mode)
        {
            ll_hmac_enable_private(p_hmac->p_instance);
        }
        else
        {
            ll_hmac_disable_private(p_hmac->p_instance);
        }

        /* Set big endian */
        ll_hmac_disable_little_endian(p_hmac->p_instance);

        /* Disable interrupt and clear flag */
        p_hmac->p_instance->INT = 1U;

    } while(0);

    return status;
}

static void hmac_read_data(hmac_handle_t *p_hmac, uint32_t *p_data)
{
    for (uint32_t i = 0U; i < HMAC_DIGESTSIZE_WORDS; i++)
    {
        p_data[i] = p_hmac->p_instance->DATA_OUT;
    }
}

static void hmac_write_data(hmac_handle_t *p_hmac, uint32_t *p_data)
{
    for (uint32_t i = 0U; i < HMAC_BLOCKSIZE_WORDS; i++)
    {
        p_hmac->p_instance->DATA_IN = p_data[i];
    }
}

static hal_status_t hmac_mcu_process(hmac_handle_t *p_hmac, uint32_t timeout)
{
    hal_status_t status = HAL_OK;

    /* If there is a need of misalignment when MCU read and/or write data */
//    ll_hmac_set_dma_read_address(p_hmac->p_instance, (uint32_t)p_hmac->p_message & 0x3);
//    ll_hmac_set_dma_write_address(p_hmac->p_instance, (uint32_t)p_hmac->p_digest & 0x3);

    /* HMAC enable */
    if (0U == ll_hmac_is_enabled(p_hmac->p_instance))
    {
        ll_hmac_enable(p_hmac->p_instance);
    }

    /* Query SHA ready status in HMAC mode */
    if (HMAC_MODE_HMAC == p_hmac->init.mode)
    {
        status = hmac_wait_flag_state_until_timeout(p_hmac, HMAC_FLAG_DATAREADY_SHA, SET, HAL_HMAC_TIMEOUT_DEFAULT_VALUE);
        if (HAL_OK != status)
        {
            /* HMAC disable */
            ll_hmac_disable(p_hmac->p_instance);

            /* Change state Ready */
            p_hmac->state = HAL_HMAC_STATE_READY;

            return status;
        }
    }

    if (0U < timeout)
    {
        while (0U < p_hmac->block_size)
        {
            /* Enable last transfer for the last block */
            if ((1U == p_hmac->block_size) && (1U == p_hmac->is_last_trans))
            {
                ll_hmac_enable_last_transfer(p_hmac->p_instance);
            }

            /* MCU write HMAC input */
            hmac_write_data(p_hmac, p_hmac->p_message);
            p_hmac->p_message += HMAC_BLOCKSIZE_WORDS;

            /* HMAC wait for completion */
            if ((1U == p_hmac->block_size) && (1U == p_hmac->is_last_trans))
            {
                status = hmac_wait_flag_state_until_timeout(p_hmac, HMAC_FLAG_DATAREADY_HMAC, SET, timeout);
            }
            else
            {
                status = hmac_wait_flag_state_until_timeout(p_hmac, HMAC_FLAG_DATAREADY_SHA, SET, timeout);
            }

            if (HAL_OK != status)
            {
                break;
            }

            p_hmac->block_size--;
        }

        if (HAL_OK != status)
        {
            /* HMAC disable */
            ll_hmac_disable(p_hmac->p_instance);

            /* Change state Ready */
            p_hmac->state = HAL_HMAC_STATE_READY;
        }
        else if (1U == p_hmac->is_last_trans)
        {
            /* MCU read HMAC output */
            hmac_read_data(p_hmac, p_hmac->p_digest);

            /* HMAC disable */
            ll_hmac_disable(p_hmac->p_instance);

            /* Change state Ready */
            p_hmac->state = HAL_HMAC_STATE_READY;
        }
        else
        {
            /* Nothing to do */
        }
    }
    else
    {
        /* Enable interrupt */
        ll_hmac_enable_it_done(p_hmac->p_instance);
        __HAL_HMAC_CLEAR_FLAG_IT(p_hmac, HMAC_IT_DONE);

        /* Enable last transfer for the last block */
        if ((1U == p_hmac->block_size) && (1U == p_hmac->is_last_trans))
        {
            ll_hmac_enable_last_transfer(p_hmac->p_instance);
        }

        /* MCU write HMAC input */
        GLOBAL_EXCEPTION_DISABLE();
        hmac_write_data(p_hmac, p_hmac->p_message);
        p_hmac->p_message += HMAC_BLOCKSIZE_WORDS;
        GLOBAL_EXCEPTION_ENABLE();
    }

    return status;
}

static hal_status_t hmac_dma_process(hmac_handle_t *p_hmac)
{
    hal_status_t status = HAL_OK;

    /* Set DMA read and write address */
    ll_hmac_set_dma_read_address(p_hmac->p_instance, (uint32_t)p_hmac->p_message);
    ll_hmac_set_dma_write_address(p_hmac->p_instance, (uint32_t)p_hmac->p_digest);

    if ((p_hmac->block_size > 0U) && (0U == p_hmac->is_last_trans))
    {
        /* DMA Single Transfer Max 512 Blocks */
        if (p_hmac->block_size > HMAC_DMA_BLOCK_MAX)
        {
            ll_hmac_set_dma_transfer_block(p_hmac->p_instance, HMAC_DMA_BLOCK_MAX);
            p_hmac->p_message += HMAC_DMA_BLOCK_MAX * HMAC_BLOCKSIZE_WORDS;
            p_hmac->block_size -= HMAC_DMA_BLOCK_MAX;
        }
        else
        {
            ll_hmac_set_dma_transfer_block(p_hmac->p_instance, p_hmac->block_size);
            p_hmac->p_message += p_hmac->block_size * HMAC_BLOCKSIZE_WORDS;
            p_hmac->block_size -= p_hmac->block_size;
        }
    }
    else
    {
        ll_hmac_set_dma_transfer_block(p_hmac->p_instance, p_hmac->block_size);
        p_hmac->p_message += p_hmac->block_size * HMAC_BLOCKSIZE_WORDS;
        p_hmac->block_size -= p_hmac->block_size;
    }

    /* HMAC enable */
    if (0U == ll_hmac_is_enabled(p_hmac->p_instance))
    {
        ll_hmac_enable(p_hmac->p_instance);
    }

    /* Query SHA ready status in HMAC mode */
    if (HMAC_MODE_HMAC == p_hmac->init.mode)
    {
        status = hmac_wait_flag_state_until_timeout(p_hmac, HMAC_FLAG_DATAREADY_SHA, SET, HAL_HMAC_TIMEOUT_DEFAULT_VALUE);
        if (HAL_OK != status)
        {
            /* HMAC disable */
            ll_hmac_disable(p_hmac->p_instance);

            /* Change state Ready */
            p_hmac->state = HAL_HMAC_STATE_READY;

            return status;
        }
    }

    /* Enable interrupt */
    ll_hmac_enable_it_done(p_hmac->p_instance);
    __HAL_HMAC_CLEAR_FLAG_IT(p_hmac, HMAC_IT_DONE);

    /* DMA start */
    if (1U == p_hmac->is_last_trans)
    {
        __HAL_HMAC_DMA_START_LAST_TRF(p_hmac);
    }
    else
    {
        ll_hmac_enable_dma_start(p_hmac->p_instance);
    }

    return status;
}

static void hal_hmac_swap_endian(uint32_t *in, uint32_t len, uint32_t *out)
{
    for (uint32_t i = 0U; i < len; i++)
    {
        out[i] = ((in[i] & 0xFF000000U) >> 24U) + ((in[i] & 0xFF0000U) >> 8U)
                + ((in[i] & 0xFF00U) << 8U) + ((in[i] & 0xFFU) << 24U);
    }
}

static uint32_t hal_hmac_preprocess_big_endian(uint8_t *in, uint32_t len, uint32_t *out, uint32_t mode)
{
    uint32_t i = 0U;
    uint32_t j = 3U;
    uint32_t a = 0U;
    uint8_t val[4]; /* C0 don`t support byte access */
    uint32_t block_size = ((len % 64U) > 55U) ? 2U : 1U;
    uint32_t out_len = block_size * 16U;

    uint32_t temp = 1U << 24U;
    uint32_t m = len / 64U;

    memset(out, 0x0, sizeof(uint32_t) * out_len);
    memset(val, 0x0, sizeof(val));
    in += m * 64U;

    for (i = 0U; i < (len - (m * 64U)); i++)
    {
        if ((i % 4U) == 0U)
        {
            memcpy(&val[0], in, 4U);
        }
        if ((i % 4U) == 0U)
        {
            j = 0U;
            *out += ((uint32_t)(val[i%4U])) << 24U;
        }
        else if ((i % 4U) == 1U)
        {
            j = 1U;
            *out += ((uint32_t)(val[i%4U])) << 16U;
        }
        else if ((i % 4U) == 2U)
        {
            j = 2U;
            *out += ((uint32_t)(val[i%4U])) << 8U;
        }
        else
        {
            *out += val[i%4U];
            j = 3U;
            out++;
            a++;
        }
        in++;
    }

    if (j == 0U)
    {
        *out += 128U << 16U;
    }
    else if (j == 1U)
    {
        *out += 128U << 8U;
    }
    else if (j == 2U)
    {
        *out += 128U;
    }
    else
    {
        *out += temp * 128U;
    }

    i = (out_len - 1U) - a;

    out += i;

    if (HMAC_MODE_SHA == mode)
    {
        *out += 8U * len;
    }
    else if (HMAC_MODE_HMAC == mode)
    {
        *out += 8U * len + 512U;
    }
    else
    {
        /* Nothing to do */
    }

    return block_size;
}

#ifdef HAL_HMAC_MODULE_STREAM_ENABLED

static uint32_t hal_hmac_stream_preprocess_big_endian(hmac_handle_t *p_hmac, uint8_t *in, uint32_t len, uint32_t *out)
{
    uint32_t i = 0U;
    uint32_t j = 3U;
    uint32_t a = 0U;
    uint32_t m = 0U;
    uint8_t val[4]; /* C0 don`t support byte access */
    uint32_t temp = 0U;
    uint32_t block_size = (len % 64U > 55U) ? 2U : 1U;
    uint32_t out_len = block_size * 16U;

    temp = 1U << 24U;
    m = len / 64U;

    memset(out, 0x0, sizeof(uint32_t) * out_len);
    memset(val, 0x0, sizeof(val));
    in += m * 64U;

    for (i = 0U; i < (len - (m * 64U)); i++)
    {
        if ((i % 4U) == 0U)
        {
            memcpy(&val[0], in, 4);
        }
        if ((i % 4U) == 0U)
        {
            j = 0U;
            *out += ((uint32_t)(val[i%4U])) << 24U;
        }
        else if ((i % 4U) == 1U)
        {
            j = 1;
            *out += ((uint32_t)(val[i%4U])) << 16U;
        }
        else if ((i % 4U) == 2U)
        {
            j = 2U;
            *out += ((uint32_t)(val[i%4U])) << 8U;
        }
        else
        {
            *out += val[i%4U];
            j = 3U;
            out++;
            a++;
        }
        in++;
    }

    if (j == 0U)
    {
        *out += 128U << 16U;
    }
    else if (j == 1U)
    {
        *out += 128U << 8U;
    }
    else if (j == 2U)
    {
        *out += 128U;
    }
    else
    {
        *out += temp * 128U;
    }

    i = (out_len - 1U) - a;

    out += i;

    if (HMAC_MODE_SHA == p_hmac->init.mode)
    {
        *out += 8U * p_hmac->total;
    }
    else if (HMAC_MODE_HMAC == p_hmac->init.mode)
    {
        *out += (8U * p_hmac->total) + 512U;
    }
    else
    {
        /* Nothing to do */
    }

    return block_size;
}

__WEAK hal_status_t hal_hmac_sha256_start(hmac_handle_t *p_hmac)
{
    hal_status_t status = HAL_OK;

    /* Check the HMAC handle allocation */
    if (NULL == p_hmac)
    {
        return HAL_ERROR;
    }

    if (HAL_HMAC_STATE_READY == p_hmac->state)
    {
        /* Change state Busy */
        p_hmac->state = HAL_HMAC_STATE_BUSY;

        /* Configure HMAC registers */
        status = hmac_config(p_hmac);
        if (HAL_OK != status)
        {
            /* Set invalid param error code */
            p_hmac->error_code |= HAL_HMAC_ERROR_INVALID_PARAM;

            return status;
        }

        /* Clear data total size */
        p_hmac->total = 0U;

        /* Change state Ready */
        p_hmac->state = HAL_HMAC_STATE_READY;
    }
    else
    {
        /* Set Busy error code */
        p_hmac->error_code |= HAL_HMAC_ERROR_BUSY;
        status = HAL_ERROR;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_hmac_sha256_update(hmac_handle_t *p_hmac, uint32_t *p_message, uint32_t number)
{
    hal_status_t status = HAL_OK;
    uint32_t remain_data_num = 0U;
    uint8_t *msg_pos = NULL;
    uint32_t fill_count = 0U;

    /* Check the HMAC handle allocation */
    if (NULL == p_message)
    {
        return HAL_ERROR;
    }

    if (HAL_HMAC_STATE_READY == p_hmac->state)
    {
        /* Change state Busy */
        p_hmac->state = HAL_HMAC_STATE_BUSY;

        do {
            /* Current input pointer position */
            msg_pos = (uint8_t *)p_message;

            /* Check if last operation have */
            remain_data_num = p_hmac->total & 0x3FU;
            if (remain_data_num > 0U)
            {
                /* Get remaining hmac buffer space */
                fill_count = HMAC_BLOCKSIZE_BYTES - remain_data_num;

                /* Check if the buffer is exceeded */
                if (number < fill_count)
                {
                    memcpy(((uint8_t *)(p_hmac->hmac_buf) + remain_data_num), (uint8_t *)p_message, number);
                    p_hmac->total += number;
                    p_hmac->state = HAL_HMAC_STATE_READY;
                    break;
                }

                number -= fill_count;
                p_hmac->total += fill_count;

                /* Fill the remaining hmac buffer */
                memcpy(((uint8_t *)(p_hmac->hmac_buf) + remain_data_num), (uint8_t *)p_message, fill_count);
                msg_pos += fill_count;

                /* hmac buffer process */
                p_hmac->p_message     = p_hmac->hmac_buf;
                p_hmac->p_digest      = NULL;
                p_hmac->block_size    = 1U;
                p_hmac->is_last_trans = 0U;

                if (p_hmac->init.strm_dma_en == HAL_HMAC_STRM_DMA_DISABLE)
                {
                    /* MCU Mode */
                    status = hmac_mcu_process(p_hmac, HAL_HMAC_TIMEOUT_DEFAULT_VALUE);
                    if (HAL_OK != status)
                    {
                        break;
                    }
                }
                else
                {
                    /* DMA Mode */
                    p_hmac->it_flag = 0U;
                    status = hmac_dma_process(p_hmac);
                    if (HAL_OK != status)
                    {
                        break;
                    }
                    while(0U == p_hmac->it_flag){}
                }
            }

            p_hmac->total += number;

            /* Check if there are some data haven't been calculated */
            remain_data_num = (number & 0x3FU);
            if (remain_data_num > 0U)
            {
                memset(p_hmac->hmac_buf, 0x0, sizeof(p_hmac->hmac_buf));
                memcpy((uint8_t *)(p_hmac->hmac_buf), (uint8_t *)msg_pos + number - remain_data_num, remain_data_num);
            }

            /* Remaining input data process */
            //lint -e9087 Cast a pointer to a different object type is necessary
            p_hmac->p_message     = (uint32_t *)msg_pos;
            p_hmac->p_digest      = NULL;
            p_hmac->block_size    = number / HMAC_BLOCKSIZE_BYTES;
            p_hmac->is_last_trans = 0U;

            if (p_hmac->init.strm_dma_en == HAL_HMAC_STRM_DMA_DISABLE)
            {
                /* MCU Mode */
                status = hmac_mcu_process(p_hmac, HAL_HMAC_TIMEOUT_DEFAULT_VALUE);
                if (HAL_OK != status)
                {
                    break;
                }
            }
            else
            {
                if (p_hmac->block_size > 0U)
                {
                    /* DMA Mode */
                    p_hmac->it_flag = 0U;
                    status = hmac_dma_process(p_hmac);
                    if (HAL_OK != status)
                    {
                        break;
                    }
                    while(0U == p_hmac->it_flag){}
                }
            }

        } while( 0 );

        /* Change state Ready */
        p_hmac->state = HAL_HMAC_STATE_READY;
    }
    else
    {
        /* Set Busy error code */
        p_hmac->error_code |= HAL_HMAC_ERROR_BUSY;
        status = HAL_ERROR;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_hmac_sha256_finish(hmac_handle_t *p_hmac, uint32_t *p_digest)
{
    hal_status_t status = HAL_OK;
    uint32_t remain_data_num = 0;
    uint32_t input_rem[32] = { 0 };
    uint32_t padded_block_size = 0;

    /* Check the HMAC handle allocation */
    if ((NULL == p_hmac) || (NULL == p_digest))
    {
        return HAL_ERROR;
    }

    if (HAL_HMAC_STATE_READY == p_hmac->state)
    {
        /* Change state Busy */
        p_hmac->state = HAL_HMAC_STATE_BUSY;

        /* Check remaining data num */
        remain_data_num = p_hmac->total & 0x3FU;
        memset(((uint8_t *)(p_hmac->hmac_buf) + remain_data_num), 0x0, HMAC_BLOCKSIZE_BYTES - remain_data_num);

        /* Last block data preprocessing */
        padded_block_size = hal_hmac_stream_preprocess_big_endian(p_hmac, (uint8_t *)(p_hmac->hmac_buf), remain_data_num, input_rem);
        //lint -e934 Taking address of near auto variable is necessary
        hal_hmac_swap_endian(input_rem, 32U, input_rem);

        /* Last block process */
        p_hmac->p_message     = input_rem;
        p_hmac->p_digest      = p_digest;
        p_hmac->block_size    = padded_block_size;
        p_hmac->is_last_trans = 1U;

        if (p_hmac->init.strm_dma_en == HAL_HMAC_STRM_DMA_DISABLE)
        {
            /* MCU Mode */
            status = hmac_mcu_process(p_hmac, HAL_HMAC_TIMEOUT_DEFAULT_VALUE);
        }
        else
        {
            /* DMA Mode */
            p_hmac->strm_done_flag = 0U;
            status = hmac_dma_process(p_hmac);
            if (HAL_OK != status)
            {
                return status;
            }
            while(0U == p_hmac->strm_done_flag){}
        }
    }
    else
    {
        /* Set Busy error code */
        p_hmac->error_code |= HAL_HMAC_ERROR_BUSY;
        status = HAL_ERROR;
    }

    /* Return function status */
    return status;
}

#endif /* HAL_HMAC_MODULE_STREAM_ENABLED */

#endif /* HAL_HMAC_MODULE_ENABLED */


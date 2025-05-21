/**
  ****************************************************************************************
  * @file    hal_adc.c
  * @author  BLE Driver Team
  * @brief   ADC HAL module driver.
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

#include <math.h>
#include "gr5x.h"

#ifdef HAL_ADC_MODULE_ENABLED
#include "hal_def.h"

#ifdef HAL_CLOCK_MODULE
#include "hal_cgc.h"
#endif

#include "hal_adc.h"
#include "grx_sys.h"

/*
 * DEFINE
 *****************************************************************************************
 */
#define ADC_UNUSED_CONV_LENGTH         (3U)    /**< Invalid calibrated buffer. */
#define ADC_USED_CONV_LENGTH           (16U)   /**< Used calibrated buffer.    */
#define ADC_DGL_CALI_MODE              (0xFFU)   /**< Use single method to calibrate ADC.    */
#define ADC_DIFF_CALI_MODE             (0x00U)   /**< Use differential method to calibrate ADC.    */

typedef struct _adc_param_t
{
    uint16_t offset;     /* Offset based on interanl reference */
    uint16_t slope;      /* Slope based on interanl reference  */
    uint16_t temp;       /* The ADC output code based on TEMP_REF temperature. */
    uint16_t temp_ref;   /* Chip temperature sensor reference temperature. E.g. Decimal 2618: 26.18буC */
    uint16_t vbat_div;   /* Chip internal resistance voltage ratio. E.g. Decimal 358:3.58 */
    uint8_t  cali_mode;  /* ADC calibration mode                      */
} adc_param_t;

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void adc_dma_cplt(dma_handle_t *p_dma);
static void adc_dma_error(dma_handle_t *p_dma);
static hal_status_t adc_wait_notempty_until_timeout(adc_handle_t *p_adc, uint32_t timeout);
static hal_status_t adc_try_swtoken_locked_until_timeout(adc_handle_t *p_adc, uint32_t timeout);
static error_status_t hal_adc_try_set_clock(uint32_t clk);

static void adc_set_device_state(adc_handle_t *p_adc ,hal_adc_state_t state);
static error_status_t hal_adc_get_slope_offset(adc_handle_t *p_adc, adc_param_t *p_param);
static uint16_t diff_calculate_offset(adc_handle_t *p_adc);
void filter_adc_data(uint32_t clock);
static uint32_t diff_int_vref   = 0;
static uint16_t diff_int_offset = 0;
static double   diff_ext_vref   = 0.0;
static uint16_t diff_ext_offset = 0;

#define IS_LL_ADC_INPUT(__INPUT__)          (((__INPUT__) == LL_ADC_INPUT_SRC_IO0) || \
                                             ((__INPUT__) == LL_ADC_INPUT_SRC_IO1) || \
                                             ((__INPUT__) == LL_ADC_INPUT_SRC_IO2) || \
                                             ((__INPUT__) == LL_ADC_INPUT_SRC_IO3) || \
                                             ((__INPUT__) == LL_ADC_INPUT_SRC_IO4) || \
                                             ((__INPUT__) == LL_ADC_INPUT_SRC_TMP) || \
                                             ((__INPUT__) == LL_ADC_INPUT_SRC_BAT) || \
                                             ((__INPUT__) == LL_ADC_INPUT_SRC_REF))

#define IS_LL_ADC_INPUT_MODE(__MODE__)      (((__MODE__) == LL_ADC_INPUT_SINGLE) || \
                                             ((__MODE__) == LL_ADC_INPUT_DIFFERENTIAL)

#define IS_LL_ADC_REF(__INPUT__)            (((__INPUT__) == LL_ADC_REF_SRC_BUF_INT) || \
                                             ((__INPUT__) == LL_ADC_REF_SRC_IO0)     || \
                                             ((__INPUT__) == LL_ADC_REF_SRC_IO1)     || \
                                             ((__INPUT__) == LL_ADC_REF_SRC_IO2)     || \
                                             ((__INPUT__) == LL_ADC_REF_SRC_IO3))

#define IS_LL_ADC_REF_VALUE(__VALUE__)      (((__VALUE__) >= LL_ADC_REF_VALUE_0P8) && \
                                             ((__VALUE__) <= LL_ADC_REF_VALUE_1P6))

/** @} */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup ADC_LL_Exported_Functions
  * @{
  */

/** @addtogroup ADC_LL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize ADC registers (Registers restored to their default values).
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: ADC registers are de-initialized
  *          - ERROR:   Wrong ADC Port
  */
__WEAK void ll_adc_deinit(void)
{
    /* Reset configuration */
    //lint -e923 Cast from pointer to unsigned int is necessary
    LL_ADC_WriteReg(AON_PMU, SNSADC_CFG, 0x0708070AU);

    /* Disable clock */
    ll_adc_disable_clock();
    //ll_adc_set_clock(LL_ADC_CLK_1P6);
}

/**
  * @brief  Initialize ADC registers according to the specified parameters in adc_init_t.
  * @param  p_adc_init pointer to a @ref adc_init_t structure
  *         that contains the ADC_InitStructuration information for the specified ADC peripheral.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: ADC registers are initialized according to adc_init_t content
  *          - ERROR:   Not applicable
  */
__WEAK void ll_adc_init(ll_adc_init_t *p_adc_init)
{
    /* Check the parameters */
    gr_assert_param(IS_LL_ADC_INPUT(p_adc_init->channel_p));
    gr_assert_param(IS_LL_ADC_INPUT(p_adc_init->channel_n));
    gr_assert_param(IS_LL_ADC_INPUT_MODE(p_adc_init->input_mode));
    gr_assert_param(IS_LL_ADC_REF(p_adc_init->ref_source));
    gr_assert_param(IS_LL_ADC_REF_VALUE(p_adc_init->ref_value));

    /* ------------------------- Configure ADC ---------------- */
    ll_adc_set_thresh(0x3F);//thresh cann't be zero, otherwise it will affect DMA enven if in polling mode.
    ll_adc_set_channelp(p_adc_init->channel_p);
    ll_adc_set_channeln(p_adc_init->channel_n);

    if ((LL_ADC_INPUT_SRC_TMP == p_adc_init->channel_p) || (LL_ADC_INPUT_SRC_TMP == p_adc_init->channel_n))
    {
        ll_adc_enable_temp();
    }
    if ((LL_ADC_INPUT_SRC_BAT == p_adc_init->channel_p) || (LL_ADC_INPUT_SRC_BAT == p_adc_init->channel_n))
    {
        ll_adc_enable_vbat();
    }
    ll_adc_set_input_mode(p_adc_init->input_mode);
    ll_adc_set_ref(p_adc_init->ref_source);
    if (LL_ADC_REF_SRC_BUF_INT == p_adc_init->ref_source)
    {
        ll_adc_set_ref_value(p_adc_init->ref_value);
    }
    //ll_adc_set_clock(p_adc_init->clock);
    ll_adc_disable_clock();
}

/**
  * @brief Set each @ref ll_adc_init_t field to default value.
  * @param p_adc_init pointer to a @ref ll_adc_init_t structure
  *                          whose fields will be set to default values.
  * @retval None
  */

__WEAK void ll_adc_struct_init(ll_adc_init_t *p_adc_init)
{
    /* Reset ADC init structure parameters values */
    p_adc_init->channel_p  = LL_ADC_INPUT_SRC_IO1;
    p_adc_init->channel_n  = LL_ADC_INPUT_SRC_IO1;
    p_adc_init->input_mode = LL_ADC_INPUT_DIFFERENTIAL;
    p_adc_init->ref_source = LL_ADC_REF_SRC_BUF_INT;
    p_adc_init->ref_value  = LL_ADC_REF_VALUE_0P8;
    p_adc_init->clock      = 0;
}


__WEAK hal_status_t hal_adc_init(adc_handle_t *p_adc)
{
    hal_status_t   status = HAL_OK;

    if (HAL_ADC_STATE_RESET == p_adc->state)
    {
#ifdef HAL_CLOCK_MODULE
        /* Enable ADC Clock and Automatic turn off ADC clock during WFI. */
        ll_cgc_disable_force_off_snsadc_hclk();
        ll_cgc_disable_wfi_off_snsadc_hclk();
#endif
        /* try to lock token by SW */
        status = adc_try_swtoken_locked_until_timeout(p_adc, 1000);
        if (HAL_OK != status)
        {
            p_adc->error_code = HAL_ADC_ERROR_TIMEOUT;

            return HAL_ERROR;
        }

        /* init the low level hardware : GPIO, CLOCK, NVIC, DMA */
        hal_adc_msp_init(p_adc);
    }

    /* Disable the ADC peripheral */
    __HAL_ADC_DISABLE(p_adc);

    /* Configure ADC peripheral */
    ll_adc_init(&p_adc->init);
    if (ERROR == hal_adc_try_set_clock(ADC_CLK_NONE))//confirm the adc clock is closed(help the later FIFO clearing goes well)
    {
        return HAL_ERROR;
    }
    /* Enable the ADC peripheral */
    __HAL_ADC_ENABLE(p_adc);

    /* Set ADC error code to none */
    p_adc->error_code = HAL_ADC_ERROR_NONE;

    /* Initialize the ADC state */
    adc_set_device_state(p_adc, HAL_ADC_STATE_READY);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_adc_deinit(adc_handle_t *p_adc)
{
#ifdef HAL_CLOCK_MODULE
    /* Enable ADC Clock for operate the register. */
    ll_cgc_disable_force_off_snsadc_hclk();
#endif

    /* Reset ADC Peripheral */
    ll_adc_deinit();
    if (ERROR == hal_adc_try_set_clock(ADC_CLK_NONE))//confirm the adc clock is closed
    {
        return HAL_ERROR;
    }

    /* DeInit the low level hardware: GPIO, CLOCK, NVIC... */
    hal_adc_msp_deinit(p_adc);

    /* Set ADC error code to none */
    p_adc->error_code = HAL_ADC_ERROR_NONE;

    /* Initialize the ADC state */
    adc_set_device_state(p_adc, HAL_ADC_STATE_RESET);

    /* Release the sw token */
    ll_adc_release_sw_token();
#ifdef HAL_CLOCK_MODULE
    /* Disable source Clock of ADC. */
    ll_cgc_enable_force_off_snsadc_hclk();
    ll_cgc_enable_wfi_off_snsadc_hclk();
#endif

    return HAL_OK;
}

__WEAK void hal_adc_msp_init(adc_handle_t *p_adc)
{
    UNUSED(p_adc);
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
              the hal_adc_msp_init can be implemented in the user file
     */
}

__WEAK void hal_adc_msp_deinit(adc_handle_t *p_adc)
{
    UNUSED(p_adc);
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
              the hal_adc_msp_deinit can be implemented in the user file
     */
}

__WEAK hal_status_t hal_adc_poll_for_conversion(adc_handle_t *p_adc, uint16_t *p_data, uint32_t length)
{
    hal_status_t status = HAL_OK;
    uint32_t     wcont  = length >> 1;
    uint16_t    *p_buf  = p_data;
    uint32_t     temp_data;

    uint32_t     fifo_cnt;

    if ((p_data == NULL) || (length == 0U))
    {
        return HAL_ERROR;
    }

    if (HAL_ADC_STATE_READY == p_adc->state)
    {
        p_adc->error_code = HAL_ADC_ERROR_NONE;

        /* Update ADC state */
        adc_set_device_state(p_adc, HAL_ADC_STATE_BUSY);

        /* Flush FIFO and then enable adc clock */
        __HAL_ADC_FLUSH_FIFO(p_adc);

        ll_adc_disable_dma_req();//disable DMA request, otherwise it will affect DMA mode.

        if (ERROR == hal_adc_try_set_clock(p_adc->init.clock))
        {
            return HAL_ERROR;
        }

        while (wcont)
        {
            status = adc_wait_notempty_until_timeout(p_adc, 1000);
            if (HAL_OK != status)
            {
                break;
            }

            fifo_cnt = ll_adc_get_fifo_count();
            fifo_cnt = (fifo_cnt > wcont) ? wcont : fifo_cnt;
            for (uint32_t i = 0; i < fifo_cnt; i++)
            {
                temp_data =  ll_adc_read_fifo();
                *p_buf++ = (uint16_t)(temp_data & 0x0000FFFFU);
                *p_buf++ = (uint16_t)((temp_data >> 16) & 0x0000FFFFU);
            }
            wcont -= fifo_cnt;
        }

        if ((0U == wcont) && (0U !=(length & 0x1U)))
        {
            if (HAL_OK == adc_wait_notempty_until_timeout(p_adc, 1000))
            {
                *p_buf = (uint16_t)(ll_adc_read_fifo() & (uint32_t)0x0000FFFFU);

            }
        }

        /* Disable ADC clock to stop conversion */
        if (ERROR == hal_adc_try_set_clock(ADC_CLK_NONE))
        {
            return HAL_ERROR;
        }

        adc_set_device_state(p_adc, HAL_ADC_STATE_READY);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_adc_start_dma(adc_handle_t *p_adc, uint16_t *p_data, uint32_t length)
{
    hal_status_t status = HAL_OK;

    if ((p_data == NULL) || (length == 0U))
    {
        p_adc->error_code |= HAL_ADC_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }

    if (HAL_ADC_STATE_READY == p_adc->state)
    {
        p_adc->error_code = HAL_ADC_ERROR_NONE;

        /* Update ADC state */
        adc_set_device_state(p_adc, HAL_ADC_STATE_BUSY);

        if ((DMA_SDATAALIGN_WORD) != p_adc->p_dma->init.src_data_alignment)
        {
            p_adc->error_code |= HAL_ADC_ERROR_INVALID_PARAM;
            status = HAL_ERROR;
        }

        if (HAL_OK == status)
        {
            /* Config FIFO thresh */
            ll_adc_set_thresh(16);

            /* clear dma request by disable dma req*/
            ll_adc_disable_dma_req();

            if(length < 16U)
            {
                ll_dma_set_source_burst_length(p_adc->p_dma->p_instance, p_adc->p_dma->channel, (LL_DMA_SRC_BURST_LENGTH_1));
            }
            else
            {
                ll_dma_set_source_burst_length(p_adc->p_dma->p_instance, p_adc->p_dma->channel, LL_DMA_SRC_BURST_LENGTH_8);
            }

            /* Configure counters and size of buffer */
            p_adc->buff_count = length;
            p_adc->buff_size  = p_adc->buff_count;
            p_adc->p_buffer   = p_data;

            /* Set the ADC DMA transfer complete callback */
            p_adc->p_dma->xfer_tfr_callback = adc_dma_cplt;

            /* Set the DMA error callback */
            p_adc->p_dma->xfer_error_callback = adc_dma_error;

            /* Clear the DMA abort callback */
            p_adc->p_dma->xfer_abort_callback = NULL;

            /* Flush FIFO*/
            __HAL_ADC_FLUSH_FIFO(p_adc);
            ll_adc_enable_dma_req();
            if (1u == p_adc->buff_count)
            {
                status = hal_dma_start_it(p_adc->p_dma, (uint32_t)&MCU_SUB->SENSE_ADC_FIFO, (uint32_t)p_adc->p_buffer, 1);
            }
            else
            {
                status = hal_dma_start_it(p_adc->p_dma, (uint32_t)&MCU_SUB->SENSE_ADC_FIFO, (uint32_t)p_adc->p_buffer, p_adc->buff_count >> 1);
            }
            /* Enable the ADC transmit DMA Channel */
            if (HAL_OK == status)
            {
                /*And then enable adc clock*/
                if (ERROR == hal_adc_try_set_clock(p_adc->init.clock))
                {
                    return HAL_ERROR;
                }
            }
            else
            {
                p_adc->error_code |= HAL_ADC_ERROR_DMA;

                /* Update ADC state */
                adc_set_device_state(p_adc, HAL_ADC_STATE_READY);
            }

        }
        else
        {
            /* Update ADC state */
            adc_set_device_state(p_adc, HAL_ADC_STATE_READY);
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}


__WEAK hal_status_t hal_adc_stop_dma(adc_handle_t *p_adc)
{
    hal_status_t status = HAL_OK;

    /* Check if the state is in busy states */
    if (HAL_ADC_STATE_BUSY == p_adc->state)
    {
        /* Disable ADC clock to stop conversion */
        if (ERROR == hal_adc_try_set_clock(ADC_CLK_NONE))
        {
            return HAL_ERROR;
        }

        if (HAL_DMA_STATE_BUSY == p_adc->p_dma->state)
        {
            /* Abort DMA channel */
            status = hal_dma_abort(p_adc->p_dma);
            if (HAL_OK != status)
            {
                p_adc->error_code |= HAL_ADC_ERROR_DMA;
            }
        }

        /* Update state */
        adc_set_device_state(p_adc, HAL_ADC_STATE_READY);
    }

    return status;
}

__WEAK void hal_adc_conv_cplt_callback(adc_handle_t *p_adc)
{
    UNUSED(p_adc);
    return;
}

__WEAK hal_adc_state_t hal_adc_get_state(adc_handle_t *p_adc)
{
    /* Return ADC handle state */
    return p_adc->state;
}

__WEAK uint32_t hal_adc_get_error(adc_handle_t *p_adc)
{
    return p_adc->error_code;
}

__WEAK void hal_adc_suspend_reg(adc_handle_t *p_adc)
{
    p_adc->retention[0] = READ_REG(AON_PMU->SNSADC_CFG);
    p_adc->retention[1] = READ_REG(MCU_SUB->SENSE_FF_THRESH);
    SET_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_EN_Msk);
}

__WEAK void hal_adc_resume_reg(adc_handle_t *p_adc)
{
    CLEAR_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_EN_Msk);
    WRITE_REG(AON_PMU->SNSADC_CFG,p_adc->retention[0]);
    WRITE_REG(MCU_SUB->SENSE_FF_THRESH, p_adc->retention[1]);
    SET_BITS(AON_PMU->SNSADC_CFG, AON_PMU_SNSADC_CFG_EN_Msk);
}

#ifdef HAL_PM_ENABLE
__WEAK hal_pm_status_t hal_pm_adc_suspend(adc_handle_t *p_adc)
{
    hal_adc_state_t state;
    state = hal_adc_get_state(p_adc);
    if ((state != HAL_ADC_STATE_READY) && (state != HAL_ADC_STATE_RESET))
    {
        return HAL_PM_ACTIVE;
    }
    else
    {
        hal_adc_suspend_reg(p_adc);
        return HAL_PM_SLEEP;
    }
}

__WEAK void hal_pm_adc_resume(adc_handle_t *p_adc)
{
    hal_adc_resume_reg(p_adc);
}
#endif /* HAL_PM_ENABLE */

__WEAK hal_status_t hal_adc_set_dma_threshold(adc_handle_t *p_adc, uint32_t threshold)
{
    hal_status_t status = HAL_OK;

    if (HAL_ADC_STATE_READY == p_adc->state)
    {
        /* Configure ADC FIFO Threshold */
        ll_adc_set_thresh(threshold);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK uint32_t hal_adc_get_dma_threshold(adc_handle_t *p_adc)
{
    return ll_adc_get_thresh();
}

static void adc_dma_cplt(dma_handle_t *p_dma)
{
    //lint -e9087 Cast a pointer to a different object type is necessary
    adc_handle_t *p_adc = (adc_handle_t *)(p_dma->p_parent);

    if (p_adc->buff_size & 0x1U)
    {
        if (HAL_OK == adc_wait_notempty_until_timeout(p_adc, 1000))
        {
            (p_adc->p_buffer[p_adc->buff_size - 1U]) = (uint16_t)(ll_adc_read_fifo() & 0xFFFFU);
        }
    }

    p_adc->buff_count = 0;
    /* Disable ADC clock */
    if (ERROR == hal_adc_try_set_clock(ADC_CLK_NONE))
    {
        return;
    }

    if (HAL_ADC_STATE_BUSY == p_adc->state)
    {
        /* Change state of ADC */
        adc_set_device_state(p_adc, HAL_ADC_STATE_READY);

        hal_adc_conv_cplt_callback(p_adc);
    }
}

static void adc_dma_error(dma_handle_t *p_dma)
{
    adc_handle_t *p_adc = (adc_handle_t *)p_dma->p_parent;

    p_adc->error_code  |= HAL_ADC_ERROR_DMA;

    /* Abort the ADC */
    if(HAL_OK == hal_adc_stop_dma(p_adc))
    {
        return;
    }
}

static hal_status_t adc_wait_notempty_until_timeout(adc_handle_t *p_adc, uint32_t timeout)
{
    if((HAL_NEVER_TIMEOUT != timeout) && (HAL_MAX_DELAY < timeout))
    {
        return HAL_ERROR;
    }

    HAL_TIMEOUT_INIT();
    uint32_t tickstart = HAL_TIMEOUT_GET_TICK();
    /* Update SystemCoreClock */
    SystemCoreUpdateClock();
    uint32_t timeout_counter = SystemCoreClock / 1000U * timeout;

    /* Wait until notempty flag is set */
    while (0U == (__HAL_ADC_GET_FLAG_NOTEMPTY(p_adc)))
    {
        if(HAL_NEVER_TIMEOUT != timeout)
        {
            if((HAL_TIMEOUT_GET_TICK() - tickstart) > timeout_counter)
            {
                adc_set_device_state(p_adc, HAL_ADC_STATE_ERROR);
                p_adc->error_code |= HAL_ADC_ERROR_TIMEOUT;
                HAL_TIMEOUT_DEINIT();
                return HAL_TIMEOUT;
            }
        }
    }

    HAL_TIMEOUT_DEINIT();
    return HAL_OK;
}

static hal_status_t adc_try_swtoken_locked_until_timeout(adc_handle_t *p_adc, uint32_t timeout)
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

    /* Wait until token is locked by SW */
    while (0U == (__HAL_ADC_TRY_SWTOKEN_LOCK(p_adc)))
    {
        if(HAL_NEVER_TIMEOUT != timeout)
        {
            if((HAL_TIMEOUT_GET_TICK() - tickstart) > timeout_counter)
            {
                adc_set_device_state(p_adc, HAL_ADC_STATE_ERROR);
                p_adc->error_code |= HAL_ADC_ERROR_TIMEOUT;
                HAL_TIMEOUT_DEINIT();
                return HAL_TIMEOUT;
            }
        }
    }

    HAL_TIMEOUT_DEINIT();
    return HAL_OK;
}

__WEAK void filter_adc_data(uint32_t clock)
{
    uint32_t clk_setting;
    uint32_t adc_fifo_read_retry_cnt;
    uint32_t wait_times;

    clk_setting = clock;
    if(clk_setting != LL_ADC_CLK_NONE)
    {
        adc_fifo_read_retry_cnt = 0U;
        if(clk_setting <= LL_ADC_CLK_4K)
        {
            wait_times = 1000000U;
        }
        else
        {
            wait_times = 1000U;
        }
        while(ll_adc_get_fifo_count() < 2U)
        {
            // WA: When some board won't get FIFO data
            // in low temperature at COLD BOOT
            if (adc_fifo_read_retry_cnt > wait_times)
            {
                ll_adc_disable();
                ll_adc_enable_mas_rst();
                ll_adc_disable_mas_rst();
                ll_adc_set_clock(LL_ADC_CLK_NONE);
                delay_us(5);
                ll_adc_enable();
                ll_adc_set_clock(clk_setting);
                return;
            }
            else
            {
                delay_us(1);
                adc_fifo_read_retry_cnt++;
            }
        }
        //Filtering 4 abnormal datas
        ll_adc_read_fifo(); /*lint !e534 MISRA exception. return value is useless */
        ll_adc_read_fifo(); /*lint !e534 MISRA exception. return value is useless */
    }
}

static error_status_t hal_adc_try_set_clock(uint32_t clk)
{
    uint32_t i;

    if(clk == ll_adc_get_clock())
    {
        return SUCCESS;
    }

    GLOBAL_EXCEPTION_DISABLE();
    ll_adc_set_clock(clk);
    filter_adc_data(clk);

    for(i=0U; i<0xFFFFU; i++)
    {
        if(clk == ll_adc_get_clock())
        {
            break;//break if adc clock setting is effective.
        }
        delay_us(1);
    }
    GLOBAL_EXCEPTION_ENABLE();

    if(i == 0xFFFFU)
    {
        return ERROR;//return if adc clock setting is always invalid.
    }
    else
    {
        return SUCCESS;
    }
}

/**
 ****************************************************************************************
 * @brief  Set the snsadc bit of g_devices_state.
 ****************************************************************************************
 */
static void adc_set_device_state(adc_handle_t *p_adc ,hal_adc_state_t state)
{
    p_adc->state = state;
}

/**
 ****************************************************************************************
 * @brief  resume snsadc regs before using if just waked up from sleep /only need to clear sleep_flag if on initial or deinitial state.
init_flag=true: use in init function or deinit function
init_flag=fault: not use in init function or deinit function
 ****************************************************************************************
 */
void hal_adc_voltage_intern(adc_handle_t *p_adc, uint16_t *p_inbuf, double *p_outbuf, uint32_t buflen)
{
    double cslope = 0.0, coffset = 0.0;
    adc_param_t adc_param = {0};
    //lint -e934 Taking address of near auto variable is necessary
    if(hal_adc_get_slope_offset(p_adc, &adc_param) != SUCCESS)
    {
        return;
    }
    else
    {
        coffset = (double)adc_param.offset;
        cslope = (-1.0) * (double)adc_param.slope;
    }

    if (ADC_INPUT_SINGLE == p_adc->init.input_mode)
    {
        for (uint32_t i = 0; i < buflen; i++)
        {
            p_outbuf[i] = ((double)p_inbuf[i] - coffset) / cslope;
        }
    }
    else
    {
        if (diff_int_vref != p_adc->init.ref_value)
        {
            diff_int_vref = p_adc->init.ref_value;
            diff_int_offset = diff_calculate_offset(p_adc);
        }
        coffset = (double)diff_int_offset;
        for (uint32_t i = 0; i < buflen; i++)
        {
            p_outbuf[i] = (coffset - ((double)p_inbuf[i] * 2.0)) / cslope;
        }
    }

    return;
}

void hal_adc_voltage_extern(adc_handle_t *p_adc, double vref, uint16_t *p_inbuf, double *p_outbuf, uint32_t buflen)
{
    double cslope = 0.0, coffset = 0.0;
    adc_param_t adc_param = {0};
    //lint -e934 Taking address of near auto variable is necessary
    if(hal_adc_get_slope_offset(p_adc, &adc_param) != SUCCESS)
    {
        return;
    }
    else
    {
        coffset = (double)adc_param.offset;
        cslope = (-1.0) * (double)adc_param.slope * 1.0f / vref;
    }

    if (ADC_INPUT_SINGLE == p_adc->init.input_mode)
    {
        for (uint32_t i = 0; i < buflen; i++)
        {
            p_outbuf[i] = ((double)p_inbuf[i] - coffset) / cslope;
        }
    }
    else
    {
        if (fabs(diff_ext_vref - vref) > 0.00001)
        {
            diff_ext_vref = vref;
            diff_ext_offset = diff_calculate_offset(p_adc);
        }
        coffset = (double)diff_ext_offset;
        for (uint32_t i = 0; i < buflen; i++)
        {
            p_outbuf[i] = (coffset - ((double)p_inbuf[i] * 2.0)) / cslope;
        }
    }

    return;
}

void hal_adc_temperature_conv(adc_handle_t *p_adc, uint16_t *p_inbuf, double *p_outbuf, uint32_t buflen)
{
    double adc_temp_slope = 0.0;
    double adc_temp_offset = 0.0;
    double adc_temp_ref = 0.0;
    adc_param_t adc_param = {0};
    //lint -e934 Taking address of near auto variable is necessary
    if(hal_adc_get_slope_offset(p_adc, &adc_param) != SUCCESS)
    {
        return;
    }
    else
    {
        adc_temp_offset = (double)adc_param.offset;
        adc_temp_slope = (-1.0) * (double)adc_param.slope;
        adc_temp_ref = (double)((double)adc_param.temp_ref/100.0);
    }

    for (uint32_t i = 0; i < buflen; i++)
    {
        p_outbuf[i] = ((((double)p_inbuf[i] - adc_temp_offset) / (adc_temp_slope*2.0)) / (-0.00195)) + adc_temp_ref;
    }
}

void hal_adc_vbat_conv(adc_handle_t *p_adc, uint16_t *p_inbuf, double *p_outbuf, uint32_t buflen)
{
    double adc_vbat_slope = 0.0;
    double adc_vbat_offset = 0.0;
    double adc_vbat_div = 0.0;
    adc_param_t adc_param = {0};

    if(hal_adc_get_slope_offset(p_adc, &adc_param) != SUCCESS)
    {
        return;
    }
    else
    {
        adc_vbat_offset = (double)adc_param.offset;
        adc_vbat_slope = (-1.0) * (double)adc_param.slope;
        adc_vbat_div = (double)adc_param.vbat_div;
    }

    if (ADC_DGL_CALI_MODE == adc_param.cali_mode)
    {
        for (uint32_t i = 0; i < buflen; i++)
        {
            p_outbuf[i] = (((double)p_inbuf[i] - adc_vbat_offset) / adc_vbat_slope) * (525.0 / 136.0);
        }
    }
    else if (ADC_DIFF_CALI_MODE == adc_param.cali_mode)
    {
        for (uint32_t i = 0; i < buflen; i++)
        {
            p_outbuf[i] = ((((double)p_inbuf[i] - adc_vbat_offset) / adc_vbat_slope) * adc_vbat_div) / 100.0;
        }
    }
    else
    {
     /* Nothing to do */
    }
}

static error_status_t hal_adc_get_slope_offset(adc_handle_t *p_adc, adc_param_t *p_param)
{
    adc_trim_info_t adc_trim;

    if(0x0U != sys_adc_trim_get(&adc_trim))
    {
        return ERROR;
    }

    p_param->cali_mode = adc_trim.cali_mode;
    p_param->vbat_div = adc_trim.adc_vbat_div;
    p_param->temp_ref = adc_trim.adc_temp_ref;
    if(ADC_REF_SRC_BUF_INT != p_adc->init.ref_source)
    {
        p_param->offset = adc_trim.offset_ext_1p0;
        p_param->slope  = adc_trim.slope_ext_1p0;
    }
    else
    {
        if (ADC_REF_VALUE_0P8 == p_adc->init.ref_value)
        {
            if(ADC_INPUT_SRC_TMP == p_adc->init.channel_n)
            {
                p_param->offset = adc_trim.adc_temp;
            }
            else
            {
                p_param->offset = adc_trim.offset_int_0p8;
            }
            p_param->slope  = adc_trim.slope_int_0p8;
        }
        else if (ADC_REF_VALUE_1P2 == p_adc->init.ref_value)
        {
            p_param->offset = adc_trim.offset_int_1p2;
            p_param->slope  = adc_trim.slope_int_1p2;
        }
        else if (ADC_REF_VALUE_1P6 == p_adc->init.ref_value)
        {
            p_param->offset = adc_trim.offset_int_1p6;
            p_param->slope  = adc_trim.slope_int_1p6;
        }
        else if (ADC_REF_VALUE_2P0 == p_adc->init.ref_value)
        {
            p_param->offset = adc_trim.offset_int_2p0;
            p_param->slope  = adc_trim.slope_int_2p0;
        }
        else
        {
            return ERROR;
        }
    }

    return SUCCESS;
}

static uint16_t diff_calculate_offset(adc_handle_t *p_adc)
{
    uint32_t sum = 0;
    uint16_t conversion[ADC_UNUSED_CONV_LENGTH + ADC_USED_CONV_LENGTH];

    uint32_t adc_channel_n = p_adc->init.channel_n;
    uint32_t adc_channel_p = p_adc->init.channel_p;
    p_adc->init.channel_n = ADC_INPUT_SRC_REF;
    p_adc->init.channel_p = ADC_INPUT_SRC_REF;

    if(HAL_OK != hal_adc_deinit(p_adc))
    {
        return 0;
    }
    if(HAL_OK != hal_adc_init(p_adc))
    {
        return 0;
    }
    //lint -e934 Taking address of near auto variable is necessary
    if(hal_adc_poll_for_conversion(p_adc, conversion, ADC_UNUSED_CONV_LENGTH + ADC_USED_CONV_LENGTH))
    {
        return 0;
    }
    for(uint16_t i = ADC_UNUSED_CONV_LENGTH; i < (ADC_UNUSED_CONV_LENGTH + ADC_USED_CONV_LENGTH); i++)
    {
        sum += conversion[i];
    }
    uint16_t conversion_avg = (uint16_t)(sum / ADC_USED_CONV_LENGTH);
    p_adc->init.channel_n = adc_channel_n;
    p_adc->init.channel_p = adc_channel_p;

    if(HAL_OK != hal_adc_deinit(p_adc))
    {
        return 0;
    }
    if(HAL_OK != hal_adc_init(p_adc))
    {
        return 0;
    }
    //lint -e734, [Info] (conversion_avg << 1) is right
    return (conversion_avg << 1);
}

#endif /* HAL_ADC_MODULE_ENABLED */


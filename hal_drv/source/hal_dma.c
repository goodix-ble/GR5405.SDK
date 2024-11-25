/**
  ****************************************************************************************
  * @file    hal_dma.c
  * @author  BLE Driver Team
  * @brief   DMA HAL module driver.
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

#ifdef HAL_DMA_MODULE_ENABLED

#include "hal_dma.h"

#ifdef HAL_CLOCK_MODULE
#include "hal_cgc.h"
#endif
#ifdef HAL_CLOCK_UNIFORM_CONTROL
#include "hal_clock.h"
#endif
/*
 * DEFINE
 *****************************************************************************************
 */
#define HAL_TIMEOUT_DMA_ABORT    ((uint32_t)1000U)  /* 1s  */
#define MAX_DMA_CHANNEL_NUM      HAL_DMA_MAX_CHANNEL
#ifndef DMA0
  #ifdef DMA
  #define DMA0 DMA
  #endif
#endif
/* Private function prototypes -----------------------------------------------*/
static void dma_set_device_state(dma_handle_t *p_dma, hal_dma_state_t state);
/* Private variables ---------------------------------------------------------*/
#if defined(HAL_CLOCK_MODULE) || defined(HAL_CLOCK_UNIFORM_CONTROL)
static uint8_t s_dma_channel_en = 0;
#endif
/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
__WEAK void ll_dma_deinit(dma_regs_t *DMAx, uint32_t channel)
{
    if (LL_DMA_CHANNEL_ALL == channel)
    {
        ll_dma_disable(DMAx);
        uint32_t dma_enable;
        do
        {
            dma_enable = ll_dma_is_enabled_channel(DMAx, LL_DMA_CHANNEL_0);
            dma_enable |= ll_dma_is_enabled_channel(DMAx, LL_DMA_CHANNEL_1);
            dma_enable |= ll_dma_is_enabled_channel(DMAx, LL_DMA_CHANNEL_2);
            dma_enable |= ll_dma_is_enabled_channel(DMAx, LL_DMA_CHANNEL_3);
            dma_enable |= ll_dma_is_enabled_channel(DMAx, LL_DMA_CHANNEL_4);
            #if (MAX_DMA_CHANNEL_NUM > 5)
            dma_enable |= ll_dma_is_enabled_channel(DMAx, LL_DMA_CHANNEL_5);
            #endif
            #if (MAX_DMA_CHANNEL_NUM > 6)
            dma_enable |= ll_dma_is_enabled_channel(DMAx, LL_DMA_CHANNEL_6);
            #endif
            #if (MAX_DMA_CHANNEL_NUM > 7)
            dma_enable |= ll_dma_is_enabled_channel(DMAx, LL_DMA_CHANNEL_7);
            #endif
            dma_enable |= ll_dma_is_enable(DMAx);
        }while(0U != dma_enable);
    }
    else
    {
        if (0U != ll_dma_is_enabled_channel(DMAx, channel))
        {
            /* Disable the channel */
            ll_dma_disable_channel(DMAx, channel);
            while(0U != ll_dma_is_enabled_channel(DMAx, channel)){}
        }
        /* Mask interrupt bits for DMAx_Channely */
        ll_dma_disable_it_tfr(DMAx, channel);
        ll_dma_disable_it_blk(DMAx, channel);
        ll_dma_disable_it_srct(DMAx, channel);
        ll_dma_disable_it_dstt(DMAx, channel);
        ll_dma_disable_it_err(DMAx, channel);

        /* Reset interrupt pending bits for DMAx_Channely */
        ll_dma_clear_flag_tfr(DMAx, channel);
        ll_dma_clear_flag_blk(DMAx, channel);
        ll_dma_clear_flag_srct(DMAx, channel);
        ll_dma_clear_flag_dstt(DMAx, channel);
        ll_dma_clear_flag_err(DMAx, channel);

        /* Reset DMAx_Channely CFGL register */
        LL_DMA_WriteReg(DMAx->CHANNEL[channel], CFG_LO, 0x00000e00U + (channel * 0x20U));

        /* Reset DMAx_Channely CFGH register */
        LL_DMA_WriteReg(DMAx->CHANNEL[channel], CFG_HI, 0x00000004U);

        /* Reset DMAx_Channely CTLL register */
        LL_DMA_WriteReg(DMAx->CHANNEL[channel], CTL_LO, 0U);

        /* Reset DMAx_Channely CTLH register */
        LL_DMA_WriteReg(DMAx->CHANNEL[channel], CTL_HI, 0U);

        /* Reset DMAx_Channely Source address register */
        LL_DMA_WriteReg(DMAx->CHANNEL[channel], SAR, 0U);

        /* Reset DMAx_Channely Destination address register */
        LL_DMA_WriteReg(DMAx->CHANNEL[channel], DAR, 0U);

//        if(!LL_DMA_ReadReg(DMAx->MISCELLANEOU, CH_EN))
//        {
//            ll_dma_disable(DMAx);
//        }
//locke comment out for bug -- one channel deinit after multiple channels inited
//then DMA disable,other channel would not work normally
    }
}

/**
  * @brief  Initialize the DMA HS choice according to the specified parameters.
  * @param  DMAx DMAx instance
  * @param  src_peripheral src_peripheral
  * @param  dst_peripheral dst_peripheral
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: DMA hs choice are initialized
  *          - ERROR: Error DMA instance
  */
#ifdef HAL_DMA_HS_CHOICE_ENABLED
void ll_dma_hs_choice(dma_regs_t *DMAx, uint32_t src_peripheral, uint32_t dst_peripheral)
{
    //lint -e923 Cast from pointer to unsigned int is necessary and safe
    if(DMA0 == DMAx)
    {
        //hs 1
        if ((LL_DMA_PERIPH_UART1_TX == dst_peripheral) || (LL_DMA_PERIPH_UART1_TX == src_peripheral)) {
            MODIFY_REG(MCU_SUB->DMA_0_HS_SEL, MCU_SUB_DMA_0_HS_SEL_HS1SEL, MCU_SUB_DMA_0_HS_SEL_HS1SEL);
        } else if((LL_DMA_PERIPH_PWM0 == dst_peripheral) || (LL_DMA_PERIPH_PWM0 == src_peripheral)){
            CLEAR_BITS(MCU_SUB->DMA_0_HS_SEL, MCU_SUB_DMA_0_HS_SEL_HS1SEL);
        } else{ /* Nothing to do */ }
        //hs 2
        if ((LL_DMA_PERIPH_UART1_RX == dst_peripheral) || (LL_DMA_PERIPH_UART1_RX == src_peripheral)) {
            MODIFY_REG(MCU_SUB->DMA_0_HS_SEL, MCU_SUB_DMA_0_HS_SEL_HS2SEL, MCU_SUB_DMA_0_HS_SEL_HS2SEL);
        } else if((LL_DMA_PERIPH_PWM1 == dst_peripheral) || (LL_DMA_PERIPH_PWM1 == src_peripheral)){
            CLEAR_BITS(MCU_SUB->DMA_0_HS_SEL, MCU_SUB_DMA_0_HS_SEL_HS2SEL);
        } else{ /* Nothing to do */ }
        //hs 10
        if ((LL_DMA_PERIPH_I2C0_TX == dst_peripheral) || (LL_DMA_PERIPH_I2C0_TX == src_peripheral)) {
            MODIFY_REG(MCU_SUB->DMA_0_HS_SEL, MCU_SUB_DMA_0_HS_SEL_HS10SEL, MCU_SUB_DMA_0_HS_SEL_HS10SEL);
        } else if((LL_DMA_PERIPH_SPIM2_TX == dst_peripheral) || (LL_DMA_PERIPH_SPIM2_TX == src_peripheral)){
            CLEAR_BITS(MCU_SUB->DMA_0_HS_SEL, MCU_SUB_DMA_0_HS_SEL_HS10SEL);
        } else{ /* Nothing to do */ }
        //hs 11
        if ((LL_DMA_PERIPH_I2C0_RX == dst_peripheral) || (LL_DMA_PERIPH_I2C0_RX == src_peripheral)) {
            MODIFY_REG(MCU_SUB->DMA_0_HS_SEL, MCU_SUB_DMA_0_HS_SEL_HS11SEL, MCU_SUB_DMA_0_HS_SEL_HS11SEL);
        } else if((LL_DMA_PERIPH_SPIM2_RX == dst_peripheral) || (LL_DMA_PERIPH_SPIM2_RX == src_peripheral)){
            CLEAR_BITS(MCU_SUB->DMA_0_HS_SEL, MCU_SUB_DMA_0_HS_SEL_HS11SEL);
        } else{ /* Nothing to do */ }
    }
    else
    {
        /* Nothing to do */
    }
}
#endif /* HAL_DMA_HS_CHOICE_ENABLED */

/**
  * @brief  Initialize the DMA registers according to the specified parameters in DMA_InitStruct.
  * @note   To convert DMAx_Channely instance to DMAx instance and Channely, use helper macros :
  *         @arg @ref __LL_DMA_GET_instance
  *         @arg @ref __LL_DMA_GET_CHANNEL
  * @param  DMAx DMAx instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_0
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *
  *         (*) value not defined in all devices
  * @param  p_dma_init pointer to a @ref ll_dma_init_t structure.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: DMA registers are initialized
  *          - ERROR: Not applicable
  */
__WEAK void ll_dma_init(dma_regs_t *DMAx, uint32_t channel, const ll_dma_init_t *p_dma_init)
{
    if (ll_dma_is_enable(DMAx))
    {
        /* Disable DMA channel that has been enabled.*/
        if (ll_dma_is_enabled_channel(DMAx, channel))
        {
            ll_dma_disable_channel(DMAx, channel);
            while(ll_dma_is_enabled_channel(DMAx, channel)){}
        }
    }
    else
    {
        ll_dma_enable(DMAx);
    }

    /* Reset DMAx_Channely CFGL register */
    LL_DMA_WriteReg(DMAx->CHANNEL[channel], CFG_LO, 0x00000e00U + (channel * 0x20U));

    /* Reset DMAx_Channely CFGH register */
    LL_DMA_WriteReg(DMAx->CHANNEL[channel], CFG_HI, 0x00000004U);

    /* Reset DMAx_Channely CTLL register */
    LL_DMA_WriteReg(DMAx->CHANNEL[channel], CTL_LO, 0U);

    /* Reset DMAx_Channely CTLH register */
    LL_DMA_WriteReg(DMAx->CHANNEL[channel], CTL_HI, 0U);

    /*---------------------------- DMAx CTLL Configuration ------------------------
    * Configure DMAx_Channely: data transfer mode, peripheral and memory increment mode,
    *                          data size alignment and burst length:
    * - Direction:      CTLL_TT_FC bit
    * - SrcIncMode   :  CTLL_SINC bit
    * - DstIncMode   :  CTLL_DINC bit
    * - SrcDataWitch :  CTLL_SRC_TR_WIDTH bits
    * - dst_data_width :  CTLL_DST_TR_WIDTH bits
    * - src_gather_en   SRC_GATHER_EN bit
    * - dst_scatter_en  DST_SCATTER_EN bit
    * - llp_src_en      LLP_SRC_EN bit
    * - llp_dst_en      LLP_DST_EN bit
    */
    ll_dma_config_transfer(DMAx, channel, p_dma_init->direction | \
                           p_dma_init->src_increment_mode       | \
                           p_dma_init->dst_increment_mode       | \
                           p_dma_init->src_data_width           | \
                           p_dma_init->dst_data_width           | \
                           LL_DMA_SRC_BURST_LENGTH_1            | \
                           LL_DMA_DST_BURST_LENGTH_1);

    /*-------------------------- DMAx CFGL Configuration -------------------------
     * Configure Channel handshaking interface with parameters :
     * - src_handshaking: HS_SEL_SRC bits
     * - dst_handshaking: HS_SEL_DST bits
     */
    ll_dma_select_handshaking(DMAx, channel, LL_DMA_SHANDSHAKING_HW, LL_DMA_DHANDSHAKING_HW);

    /*-------------------------- DMAx CFGL Configuration -------------------------
     * Configure Channel max amba burst length with parameters :
     * - beats: MAX_ABRST bits
     */
    ll_dma_set_max_amba_burst(DMAx, channel, 0U);

    /*-------------------------- DMAx SAR Configuration -------------------------
     * Configure the source base address with parameter :
     * - SrcAddress: SAR[31:0] bits
     */
    ll_dma_set_source_address(DMAx, channel, p_dma_init->src_address);

    /*-------------------------- DMAx DAR Configuration -------------------------
     * Configure the destination base address with parameter :
     * - DstAddress: DAR[31:0] bits
     */
    ll_dma_set_destination_address(DMAx, channel, p_dma_init->dst_address);

    /*--------------------------- DMAx CTLH Configuration -----------------------
     * Configure the number of data to transfer with parameter :
     * - block_size: BLOCK_TS bits
     */
    ll_dma_set_block_size(DMAx, channel, p_dma_init->block_size);

    /*--------------------------- DMA_GS_SEL Configuration -----------------------
     * Configure the HS choice with parameter :
     * - DMAx:dma install
     * - src_peripheral: SRC_PER bits
     * - dst_peripheral: DEST_PER bits
     */
#ifdef HAL_DMA_HS_CHOICE_ENABLED
    ll_dma_hs_choice(DMAx, p_dma_init->src_peripheral, p_dma_init->dst_peripheral);
#endif
    /*--------------------------- DMAx CFGH Configuration -----------------------
     * Configure the source peripheral with parameter :
     * - src_peripheral: SRC_PER bits
     */
    ll_dma_set_source_peripheral(DMAx, channel, p_dma_init->src_peripheral);

    /*--------------------------- DMAx CFGH Configuration -----------------------
     * Configure the destination peripheral with parameter :
     * - dst_peripheral: DEST_PER bits
     */
    ll_dma_set_destination_peripheral(DMAx, channel, p_dma_init->dst_peripheral);

    /*--------------------------- DMAx CFGL Configuration -----------------------
     * Configure the priority of channel with parameter :
     * - priority: CH_PRIOR bits
     */
    ll_dma_set_channel_priority_level(DMAx, channel, p_dma_init->priority);
}

/**
  * @brief  Set each @ref ll_dma_init_t field to default value.
  * @param  p_dma_init Pointer to a @ref ll_dma_init_t structure.
  * @retval None
  */
__WEAK void ll_dma_struct_init(ll_dma_init_t *p_dma_init)
{
    /* Set DMA_InitStruct fields to default values */
    p_dma_init->src_address        = (uint32_t)0x00000000U;
    p_dma_init->dst_address        = (uint32_t)0x00000000U;
    p_dma_init->direction          = LL_DMA_DIRECTION_MEMORY_TO_MEMORY;
    p_dma_init->src_increment_mode = LL_DMA_SRC_NO_CHANGE;
    p_dma_init->dst_increment_mode = LL_DMA_DST_NO_CHANGE;
    p_dma_init->src_data_width     = LL_DMA_SDATAALIGN_BYTE;
    p_dma_init->dst_data_width     = LL_DMA_DDATAALIGN_BYTE;
    p_dma_init->block_size         = (uint32_t)0x00000000U;
    p_dma_init->src_peripheral     = (uint32_t)0x00000000U;
    p_dma_init->dst_peripheral     = (uint32_t)0x00000000U;
    p_dma_init->priority           = LL_DMA_PRIORITY_0;
    return;
}

static void dma_set_device_state(dma_handle_t *p_dma, hal_dma_state_t state)
{
    p_dma->state = state;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
__WEAK hal_status_t hal_dma_init(dma_handle_t *p_dma)
{
    ll_dma_init_t dma_init;

    /* Check the parameters */
    gr_assert_param(IS_DMA_ALL_P_INSTANCE(p_dma->p_instance));
    gr_assert_param(IS_DMA_ALL_INSTANCE(p_dma->channel));
    gr_assert_param(IS_DMA_ALL_REQUEST(p_dma->init.src_request));
    gr_assert_param(IS_DMA_ALL_REQUEST(p_dma->init.dst_request));
    gr_assert_param(IS_DMA_DIRECTION(p_dma->init.direction));
    gr_assert_param(IS_DMA_SOURCE_INC_STATE(p_dma->init.src_increment));
    gr_assert_param(IS_DMA_DESTINATION_INC_STATE(p_dma->init.dst_increment));
    gr_assert_param(IS_DMA_SOURCE_DATA_SIZE(p_dma->init.src_data_alignment));
    gr_assert_param(IS_DMA_DESTINATION_DATA_SIZE(p_dma->init.dst_data_alignment));
    gr_assert_param(IS_DMA_PRIORITY(p_dma->init.priority));

    if (HAL_DMA_STATE_RESET == p_dma->state)
    {
#ifdef HAL_CLOCK_MODULE
        /* Enable Clock for Serial blocks and Automatic turn off Serial blocks clock during WFI. */
        //lint -e923 Cast from pointer to unsigned int is necessary
        if (p_dma->p_instance == DMA0)
        {
            GLOBAL_EXCEPTION_DISABLE();
            if(0U == s_dma_channel_en)
            {
                ll_cgc_disable_force_off_dma0_hclk();
                ll_cgc_disable_wfi_off_dma_hclk();
            }
            s_dma_channel_en |= (uint8_t)(0x1UL << p_dma->channel);
            GLOBAL_EXCEPTION_ENABLE();
        }
#endif

#ifdef HAL_CLOCK_UNIFORM_CONTROL
        /* Enable Clock for Serial blocks and Automatic turn off Serial blocks clock during WFI. */
        GLOBAL_EXCEPTION_DISABLE();
        if(0U == s_dma_channel_en)
        {
            hal_clock_enable_module((uint32_t)p_dma->p_instance);
        }
        s_dma_channel_en |= (uint8_t)(0x1UL << p_dma->channel);
        GLOBAL_EXCEPTION_ENABLE();
#endif
    }

    /* Change DMA peripheral state */
    dma_set_device_state(p_dma, HAL_DMA_STATE_BUSY);

    /* Set each dma_init_t field to default value. */
    //lint -e934 Taking address of near auto variable is necessary
    ll_dma_struct_init(&dma_init);

    /* Prepare the DMA Channel configuration */
    dma_init.src_peripheral     = p_dma->init.src_request;
    dma_init.dst_peripheral     = p_dma->init.dst_request;
    dma_init.src_increment_mode = p_dma->init.src_increment;
    dma_init.dst_increment_mode = p_dma->init.dst_increment;
    dma_init.src_data_width     = p_dma->init.src_data_alignment;
    dma_init.dst_data_width     = p_dma->init.dst_data_alignment;
    dma_init.direction          = p_dma->init.direction;
    dma_init.priority           = p_dma->init.priority;

    /* Initialize the DMA registers according to the specified parameters in dma_init_t. */
    //lint -e934 Taking address of near auto variable is necessary
    ll_dma_init(p_dma->p_instance, p_dma->channel, &dma_init);

    /* Initialise the error code */
    p_dma->error_code = HAL_DMA_ERROR_NONE;

    /* Initialize the DMA state*/
    dma_set_device_state(p_dma, HAL_DMA_STATE_READY);

    return HAL_OK;
}

__WEAK hal_status_t hal_dma_deinit(dma_handle_t *p_dma)
{
#ifdef HAL_CLOCK_MODULE
    /* Enable DMA Clock for operate the register. */
    if (p_dma->p_instance == DMA0)
    {
        ll_cgc_disable_force_off_dma0_hclk();
    }
#endif

    ll_dma_deinit(p_dma->p_instance, p_dma->channel);

    /* Initialise the error code */
    p_dma->error_code = HAL_DMA_ERROR_NONE;

    /* Initialize the DMA state */
    dma_set_device_state(p_dma, HAL_DMA_STATE_RESET);

    /* Disable source Clock of DMA. */
    if (p_dma->p_instance == DMA0)
    {
#ifdef HAL_CLOCK_MODULE
        GLOBAL_EXCEPTION_DISABLE();
        s_dma_channel_en &= ~(uint8_t)(0x1UL << p_dma->channel);
        if (0U == s_dma_channel_en)
        {
            ll_cgc_enable_force_off_dma0_hclk();
            ll_cgc_enable_wfi_off_dma_hclk();
        }
        GLOBAL_EXCEPTION_ENABLE();
#endif
#ifdef HAL_CLOCK_UNIFORM_CONTROL
        GLOBAL_EXCEPTION_DISABLE();
        s_dma_channel_en &= ~(uint8_t)(0x1UL << p_dma->channel);
        if (0U == s_dma_channel_en)
        {
            hal_clock_disable_module((uint32_t)p_dma->p_instance);
        }
        GLOBAL_EXCEPTION_ENABLE();

#endif
    }

    return HAL_OK;
}

__WEAK hal_status_t hal_dma_start(dma_handle_t *p_dma, uint32_t src_address, uint32_t dst_address, uint32_t data_length)
{
    hal_status_t status = HAL_OK;

    if(HAL_DMA_STATE_READY == p_dma->state)
    {
        /* Change DMA peripheral state */
        dma_set_device_state(p_dma, HAL_DMA_STATE_BUSY);

        /* Check the parameters */
        gr_assert_param(IS_DMA_BUFFER_SIZE(data_length));

        /* Disbale DMA channel lock */
        ll_dma_disable_channel_lock(p_dma->p_instance, p_dma->channel);

        /* Disbale DMA bus lock */
        ll_dma_disable_bus_lock(p_dma->p_instance, p_dma->channel);

        /* Configure the source, destination address and the data length */
        ll_dma_config_address(p_dma->p_instance, p_dma->channel, src_address, dst_address, p_dma->init.direction);
        ll_dma_set_block_size(p_dma->p_instance, p_dma->channel, data_length);

        /* Enable the channle */
        ll_dma_enable_channel(p_dma->p_instance, p_dma->channel);
    }
    else
    {
        status = HAL_BUSY;
    }

    return status;
}

__WEAK hal_status_t hal_dma_start_lock(dma_handle_t *p_dma, uint32_t src_address, uint32_t dst_address, uint32_t data_length, dma_lock_config *lock_config)
{
    hal_status_t status = HAL_OK;

    if(HAL_DMA_STATE_READY == p_dma->state)
    {
        /* Change DMA peripheral state */
        dma_set_device_state(p_dma, HAL_DMA_STATE_BUSY);

        /* Check the parameters */
        gr_assert_param(IS_DMA_BUFFER_SIZE(data_length));

        /* Config DMA Channel lock config */
        if(DMA_HAL_CHANNEL_LOCK_ENABLE == lock_config->channel_lock_en)
        {
            ll_dma_enable_channel_lock(p_dma->p_instance, p_dma->channel);
            ll_dma_set_channel_lock_level(p_dma->p_instance, p_dma->channel,(uint32_t)lock_config->channel_lock_level);
        }
        else
        {
            ll_dma_disable_channel_lock(p_dma->p_instance, p_dma->channel);
            ll_dma_set_channel_lock_level(p_dma->p_instance, p_dma->channel, DMA_HAL_CHANNEL_LOCK_TFR);
        }

        /* Config DMA Bus lock config */
        if(DMA_HAL_BUS_LOCK_ENABLE == lock_config->bus_lock_en)
        {
            ll_dma_enable_bus_lock(p_dma->p_instance, p_dma->channel);
            ll_dma_set_bus_lock_level(p_dma->p_instance, p_dma->channel,(uint32_t)lock_config->bus_lock_level);
        }
        else
        {
            ll_dma_disable_bus_lock(p_dma->p_instance, p_dma->channel);
            ll_dma_set_bus_lock_level(p_dma->p_instance, p_dma->channel, DMA_HAL_BUS_LOCK_TFR);
        }

        /* Configure the source, destination address and the data length */
        ll_dma_config_address(p_dma->p_instance, p_dma->channel, src_address, dst_address, p_dma->init.direction);
        ll_dma_set_block_size(p_dma->p_instance, p_dma->channel, data_length);

        /* Enable the channle */
        ll_dma_enable_channel(p_dma->p_instance, p_dma->channel);
    }
    else
    {
        status = HAL_BUSY;
    }

    return status;
}

/* _dc means disable channel in default */
__WEAK hal_status_t hal_dma_start_it_dc(dma_handle_t *p_dma, uint32_t src_address, uint32_t dst_address, uint32_t data_length)
{
    hal_status_t status = HAL_OK;

    if (0U == data_length)
    {
        p_dma->error_code |= HAL_DMA_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }

    if(HAL_DMA_STATE_READY == p_dma->state)
    {
        /* Change DMA peripheral state */
        dma_set_device_state(p_dma, HAL_DMA_STATE_BUSY);

        /* Check the parameters */
        gr_assert_param(IS_DMA_BUFFER_SIZE(data_length));

        /* Disable DMA channel interrupt */
        ll_dma_disable_it(p_dma->p_instance, p_dma->channel);

        /* Disbale DMA channel lock */
        ll_dma_disable_channel_lock(p_dma->p_instance, p_dma->channel);

        /* Disbale DMA bus lock */
        ll_dma_disable_bus_lock(p_dma->p_instance, p_dma->channel);

        /* Configure the source, destination address and the data length */
        ll_dma_config_address(p_dma->p_instance, p_dma->channel, src_address, dst_address, p_dma->init.direction);
        ll_dma_set_block_size(p_dma->p_instance, p_dma->channel, data_length);

        /* Enable the transfer complete interrupt */
        ll_dma_clear_flag_tfr(p_dma->p_instance, p_dma->channel);
        ll_dma_enable_it_tfr(p_dma->p_instance, p_dma->channel);

        /* Enable the block complete interrupt */
        ll_dma_clear_flag_blk(p_dma->p_instance, p_dma->channel);
        ll_dma_enable_it_blk(p_dma->p_instance, p_dma->channel);

        /* Enable the transfer Error interrupt */
        ll_dma_clear_flag_err(p_dma->p_instance, p_dma->channel);
        ll_dma_enable_it_err(p_dma->p_instance, p_dma->channel);

        /* Enable DMA channel interrupt */
        ll_dma_enable_it(p_dma->p_instance, p_dma->channel);

        /* Not Enable the channle */
        //ll_dma_enable_channel(p_dma->p_instance, p_dma->channel);
    }
    else
    {
        /* Remain BUSY */
        status = HAL_BUSY;
    }
    return status;
}

__WEAK hal_status_t hal_dma_start_it(dma_handle_t *p_dma, uint32_t src_address, uint32_t dst_address, uint32_t data_length)
{
    hal_status_t status = hal_dma_start_it_dc(p_dma, src_address, dst_address, data_length);

    if(HAL_OK == status) {
        /* Enable the channle */
        ll_dma_enable_channel(p_dma->p_instance, p_dma->channel);
    }

    return status;
}

__WEAK hal_status_t hal_dma_start_it_lock(dma_handle_t *p_dma, uint32_t src_address, uint32_t dst_address, uint32_t data_length, dma_lock_config *lock_config)
{
    hal_status_t status = HAL_OK;

    if (0U == data_length)
    {
        p_dma->error_code |= HAL_DMA_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }

    if(HAL_DMA_STATE_READY == p_dma->state)
    {
        /* Change DMA peripheral state */
        dma_set_device_state(p_dma, HAL_DMA_STATE_BUSY);

        /* Check the parameters */
        gr_assert_param(IS_DMA_BUFFER_SIZE(data_length));

        /* Disable DMA channel interrupt */
        ll_dma_disable_it(p_dma->p_instance, p_dma->channel);

        /* Disbale DMA channel lock */
        ll_dma_disable_channel_lock(p_dma->p_instance, p_dma->channel);

        /* Disbale DMA bus lock */
        ll_dma_disable_bus_lock(p_dma->p_instance, p_dma->channel);

        /* Config DMA Channel lock config */
        if(DMA_HAL_CHANNEL_LOCK_ENABLE == lock_config->channel_lock_en)
        {
            ll_dma_enable_channel_lock(p_dma->p_instance, p_dma->channel);
            ll_dma_set_channel_lock_level(p_dma->p_instance, p_dma->channel,(uint32_t)lock_config->channel_lock_level);
        }
        else
        {
            ll_dma_disable_channel_lock(p_dma->p_instance, p_dma->channel);
            ll_dma_set_channel_lock_level(p_dma->p_instance, p_dma->channel, DMA_HAL_CHANNEL_LOCK_TFR);
        }

        /* Config DMA Bus lock config */
        if(DMA_HAL_BUS_LOCK_ENABLE == lock_config->bus_lock_en)
        {
            ll_dma_enable_bus_lock(p_dma->p_instance, p_dma->channel);
            ll_dma_set_bus_lock_level(p_dma->p_instance, p_dma->channel,(uint32_t)lock_config->bus_lock_level);
        }
        else
        {
            ll_dma_disable_bus_lock(p_dma->p_instance, p_dma->channel);
            ll_dma_set_bus_lock_level(p_dma->p_instance, p_dma->channel, DMA_HAL_BUS_LOCK_TFR);
        }

        /* Configure the source, destination address and the data length */
        ll_dma_config_address(p_dma->p_instance, p_dma->channel, src_address, dst_address, p_dma->init.direction);
        ll_dma_set_block_size(p_dma->p_instance, p_dma->channel, data_length);

        /* Enable the transfer complete interrupt */
        ll_dma_clear_flag_tfr(p_dma->p_instance, p_dma->channel);
        ll_dma_enable_it_tfr(p_dma->p_instance, p_dma->channel);

        /* Enable the block complete interrupt */
        ll_dma_clear_flag_blk(p_dma->p_instance, p_dma->channel);
        ll_dma_enable_it_blk(p_dma->p_instance, p_dma->channel);

        /* Enable the transfer Error interrupt */
        ll_dma_clear_flag_err(p_dma->p_instance, p_dma->channel);
        ll_dma_enable_it_err(p_dma->p_instance, p_dma->channel);

        /* Enable DMA channel interrupt */
        ll_dma_enable_it(p_dma->p_instance, p_dma->channel);

        /* Enable the channle */
        ll_dma_enable_channel(p_dma->p_instance, p_dma->channel);
    }
    else
    {
        /* Remain BUSY */
        status = HAL_BUSY;
    }
    return status;
}


__WEAK hal_status_t hal_dma_abort(dma_handle_t *p_dma)
{
    hal_status_t status = HAL_OK;
    HAL_TIMEOUT_INIT();
    uint32_t tickstart = HAL_TIMEOUT_GET_TICK();
    /* Update SystemCoreClock */
    SystemCoreUpdateClock();

    uint32_t timeout_counter = ((SystemCoreClock / 1000U) * HAL_TIMEOUT_DMA_ABORT);

    /* Suspend the channel */
    ll_dma_suspend_channel(p_dma->p_instance, p_dma->channel);

    while (0U == ll_dma_is_empty_fifo(p_dma->p_instance, p_dma->channel))
    {
        /* Check for the Timeout */
        if ((HAL_TIMEOUT_GET_TICK() - tickstart) > timeout_counter)
        {
            /* Update error code */
            p_dma->error_code |= HAL_DMA_ERROR_TIMEOUT;

            /* Change the DMA state */
            dma_set_device_state(p_dma, HAL_DMA_STATE_TIMEOUT);

            status = HAL_TIMEOUT;
            goto EXIT;
        }
    }

    /* Disable the channel */
    ll_dma_disable_channel(p_dma->p_instance, p_dma->channel);

    /* Check if the DMA Channel is effectively disabled */
    while (0U != ll_dma_is_enabled_channel(p_dma->p_instance, p_dma->channel))
    {
        /* Check for the Timeout */
        if ((HAL_TIMEOUT_GET_TICK() - tickstart) > timeout_counter)
        {
            /* Update error code */
            p_dma->error_code |= HAL_DMA_ERROR_TIMEOUT;

            /* Change the DMA state */
            dma_set_device_state(p_dma, HAL_DMA_STATE_TIMEOUT);

            status = HAL_TIMEOUT;
            goto EXIT;
        }
    }

    /* Resume the channel */
    ll_dma_resume_channel(p_dma->p_instance, p_dma->channel);

    /* Change the DMA state*/
    dma_set_device_state(p_dma, HAL_DMA_STATE_READY);

EXIT:
    HAL_TIMEOUT_DEINIT();
    return status;
}

__WEAK hal_status_t hal_dma_abort_it(dma_handle_t *p_dma)
{
    hal_status_t status = HAL_OK;

    if (HAL_DMA_STATE_READY == p_dma->state)
    {
        /* Call User Abort callback */
        if (NULL != p_dma->xfer_abort_callback)
        {
            p_dma->xfer_abort_callback(p_dma);
        }
    }
    else if (HAL_DMA_STATE_BUSY != p_dma->state)
    {
        if(HAL_DMA_STATE_ERROR != p_dma->state)
        {
            /* no transfer ongoing */
            p_dma->error_code |= HAL_DMA_ERROR_NO_XFER;
            status = HAL_ERROR;
        }
    }
    else
    {
        /* Disable the channle */
        ll_dma_disable_channel(p_dma->p_instance, p_dma->channel);
        while (0U != ll_dma_is_enabled_channel(p_dma->p_instance, p_dma->channel)){}

        /* Mask interrupt bits for DMAx_Channely */
        ll_dma_disable_it_tfr(p_dma->p_instance, p_dma->channel);
        ll_dma_disable_it_blk(p_dma->p_instance, p_dma->channel);
        ll_dma_disable_it_srct(p_dma->p_instance, p_dma->channel);
        ll_dma_disable_it_dstt(p_dma->p_instance, p_dma->channel);
        ll_dma_disable_it_err(p_dma->p_instance, p_dma->channel);

        /* Reset interrupt pending bits for DMAx_Channely */
        ll_dma_clear_flag_tfr(p_dma->p_instance, p_dma->channel);
        ll_dma_clear_flag_blk(p_dma->p_instance, p_dma->channel);
        ll_dma_clear_flag_srct(p_dma->p_instance, p_dma->channel);
        ll_dma_clear_flag_dstt(p_dma->p_instance, p_dma->channel);
        ll_dma_clear_flag_err(p_dma->p_instance, p_dma->channel);

        /* Disable DMA channel interrupt */
        ll_dma_disable_it(p_dma->p_instance, p_dma->channel);

        /* Change the DMA state */
        dma_set_device_state(p_dma, HAL_DMA_STATE_READY);

        /* Call User Abort callback */
        if (NULL != p_dma->xfer_abort_callback)
        {
            p_dma->xfer_abort_callback(p_dma);
        }
    }
    return status;
}

__WEAK hal_status_t hal_dma_poll_for_transfer(dma_handle_t *p_dma, uint32_t timeout)
{
    hal_status_t status = HAL_OK;
    if((HAL_NEVER_TIMEOUT != timeout) && (HAL_MAX_DELAY < timeout))
    {
        return HAL_ERROR;
    }

    if (HAL_DMA_STATE_BUSY != p_dma->state)
    {
        /* no transfer ongoing */
        p_dma->error_code = HAL_DMA_ERROR_NO_XFER;
        return HAL_ERROR;
    }

    HAL_TIMEOUT_INIT();
    uint32_t tickstart = HAL_TIMEOUT_GET_TICK();
    /* Update SystemCoreClock */
    SystemCoreUpdateClock();
    uint32_t timeout_counter = ((SystemCoreClock / 1000U) * timeout);

    while (0U == ll_dma_is_active_flag_rtfr(p_dma->p_instance, p_dma->channel))
    {
        if (0U != ll_dma_is_active_flag_rerr(p_dma->p_instance, p_dma->channel))
        {
            /* Clear the transfer error flags */
            ll_dma_clear_flag_err(p_dma->p_instance, p_dma->channel);
            /* Update error code */
            SET_BITS(p_dma->error_code, HAL_DMA_ERROR_TE);

            /* Change the DMA state */
            dma_set_device_state(p_dma, HAL_DMA_STATE_ERROR);

            status = HAL_TIMEOUT;
            goto EXIT;
        }

        /* Check for the Timeout */
        if(HAL_NEVER_TIMEOUT != timeout)
        {
            if((HAL_TIMEOUT_GET_TICK() - tickstart) > timeout_counter)
            {
                /* Update error code */
                SET_BITS(p_dma->error_code, HAL_DMA_ERROR_TIMEOUT);

                /* Change the DMA state */
                dma_set_device_state(p_dma, HAL_DMA_STATE_TIMEOUT);

                status = HAL_TIMEOUT;
                goto EXIT;
            }
        }
    }

    /* Clear the transfer complete flag */
    ll_dma_clear_flag_tfr(p_dma->p_instance, p_dma->channel);

    /* Clear the block flag */
    ll_dma_clear_flag_blk(p_dma->p_instance, p_dma->channel);

    /* The selected Channelx EN bit is cleared (DMA is disabled and all transfers are complete) */
    dma_set_device_state(p_dma, HAL_DMA_STATE_READY);

EXIT:
    HAL_TIMEOUT_DEINIT();

    return status;
}

__WEAK void hal_dma_irq_handler(dma_handle_t *p_dma)
{
    if (HAL_DMA_STATE_RESET == p_dma->state)
    {
        return;
    }

    /* Transfer Error Interrupt management ***************************************/
    if (0U != ll_dma_is_active_flag_err(p_dma->p_instance, p_dma->channel))
    {
        if (0U != ll_dma_is_enable_it_err(p_dma->p_instance, p_dma->channel))
        {

            /* Disable the transfer error interrupt */
            ll_dma_disable_it_err(p_dma->p_instance, p_dma->channel);

            /* Clear the transfer error flag */
            ll_dma_clear_flag_err(p_dma->p_instance, p_dma->channel);

            /* Update error code */
            p_dma->error_code |= HAL_DMA_ERROR_TE;

            /* Change the DMA state */
            dma_set_device_state(p_dma, HAL_DMA_STATE_ERROR);

            if (NULL != p_dma->xfer_error_callback)
            {
                /* Transfer error callback */
                p_dma->xfer_error_callback(p_dma);
            }
        }
    }

    /* Transfer Complete Interrupt management ******************************/
    if (0U != ll_dma_is_active_flag_tfr(p_dma->p_instance, p_dma->channel))
    {
        if (0U != ll_dma_is_enable_it_tfr(p_dma->p_instance, p_dma->channel))
        {
            /* Disable the transfer Complete interrupt */
            ll_dma_disable_it_tfr(p_dma->p_instance, p_dma->channel);

            /* Clear the transfer Complete flag */
            ll_dma_clear_flag_tfr(p_dma->p_instance, p_dma->channel);

            /* Clear the transfer Complete flag */
            ll_dma_clear_flag_tfr(p_dma->p_instance, p_dma->channel);

            /* Update error code */
            p_dma->error_code |= HAL_DMA_ERROR_NONE;

            /* Change the DMA state */
            dma_set_device_state(p_dma, HAL_DMA_STATE_READY);

            if (NULL != p_dma->xfer_tfr_callback)
            {
                /* Transfer Complete callback */
                p_dma->xfer_tfr_callback(p_dma);
            }
        }
    }

    /* Block Complete Interrupt management ******************************/
    if (0U != ll_dma_is_active_flag_blk(p_dma->p_instance, p_dma->channel))
    {
        if (0U != ll_dma_is_enable_it_blk(p_dma->p_instance, p_dma->channel))
        {
            /* Disable the block interrupt */
            ll_dma_disable_it_blk(p_dma->p_instance, p_dma->channel);

            /* Clear the block flag */
            ll_dma_clear_flag_blk(p_dma->p_instance, p_dma->channel);

            /* Update error code */
            p_dma->error_code |= HAL_DMA_ERROR_NONE;

            if (NULL != p_dma->xfer_blk_callback)
            {
                /* Transfer Complete callback */
                p_dma->xfer_blk_callback(p_dma);
            }
        }
    }
}

__WEAK hal_status_t hal_dma_register_callback(dma_handle_t * p_dma, hal_dma_callback_id_t id, void (* callback)( dma_handle_t *p_dma))
{
    hal_status_t status = HAL_OK;

    if (HAL_DMA_STATE_READY == p_dma->state)
    {
        switch (id)
        {
        case  HAL_DMA_XFER_TFR_CB_ID:
            p_dma->xfer_tfr_callback = callback;
            break;

        case  HAL_DMA_XFER_BLK_CB_ID:
            p_dma->xfer_blk_callback = callback;
            break;

        case  HAL_DMA_XFER_ERROR_CB_ID:
            p_dma->xfer_error_callback = callback;
            break;

        case  HAL_DMA_XFER_ABORT_CB_ID:
            p_dma->xfer_abort_callback = callback;
            break;

        default:
            status = HAL_ERROR;
            break;
        }
    }
    else
    {
        status = HAL_ERROR;
    }

    return status;
}

__WEAK hal_status_t hal_dma_unregister_callback(dma_handle_t *p_dma, hal_dma_callback_id_t id)
{
    hal_status_t status = HAL_OK;

    if (HAL_DMA_STATE_READY == p_dma->state)
    {
        switch (id)
        {
        case  HAL_DMA_XFER_TFR_CB_ID:
            p_dma->xfer_tfr_callback = NULL;
            break;

        case  HAL_DMA_XFER_BLK_CB_ID:
            p_dma->xfer_blk_callback = NULL;
            break;

        case  HAL_DMA_XFER_ERROR_CB_ID:
            p_dma->xfer_error_callback = NULL;
            break;

        case  HAL_DMA_XFER_ABORT_CB_ID:
            p_dma->xfer_abort_callback = NULL;
            break;

        case   HAL_DMA_XFER_ALL_CB_ID:
            p_dma->xfer_tfr_callback   = NULL;
            p_dma->xfer_blk_callback   = NULL;
            p_dma->xfer_error_callback = NULL;
            p_dma->xfer_abort_callback = NULL;
            break;

        default:
            status = HAL_ERROR;
            break;
        }
    }
    else
    {
        status = HAL_ERROR;
    }

    return status;
}

__WEAK hal_dma_state_t hal_dma_get_state(dma_handle_t *p_dma)
{
    return p_dma->state;
}

__WEAK uint32_t hal_dma_get_error(dma_handle_t *p_dma)
{
    return p_dma->error_code;
}

__WEAK void hal_dma_suspend_reg(dma_handle_t *p_dma)
{
#ifndef HAL_HW_RES_SUSP_DMA
    p_dma->retention[0] = READ_REG(p_dma->p_instance->MISCELLANEOU.CFG);
    p_dma->retention[1] = READ_REG(p_dma->p_instance->CHANNEL[p_dma->channel].CTL_LO);
    p_dma->retention[2] = READ_REG(p_dma->p_instance->CHANNEL[p_dma->channel].CTL_HI);
    p_dma->retention[3] = READ_REG(p_dma->p_instance->CHANNEL[p_dma->channel].CFG_LO);
    p_dma->retention[4] = READ_REG(p_dma->p_instance->CHANNEL[p_dma->channel].CFG_HI);
#else
    UNUSED(p_dma);
#endif
}

__WEAK void hal_dma_resume_reg(dma_handle_t *p_dma)
{
#ifndef HAL_HW_RES_SUSP_DMA
    WRITE_REG(p_dma->p_instance->MISCELLANEOU.CFG, p_dma->retention[0]);
    WRITE_REG(p_dma->p_instance->CHANNEL[p_dma->channel].CTL_LO, p_dma->retention[1]);
    WRITE_REG(p_dma->p_instance->CHANNEL[p_dma->channel].CTL_HI, p_dma->retention[2]);
    WRITE_REG(p_dma->p_instance->CHANNEL[p_dma->channel].CFG_LO, p_dma->retention[3]);
    WRITE_REG(p_dma->p_instance->CHANNEL[p_dma->channel].CFG_HI, p_dma->retention[4]);
#else
    UNUSED(p_dma);
#endif
}

#ifdef HAL_PM_ENABLE
__WEAK hal_pm_status_t hal_pm_dma_suspend(dma_handle_t *p_dma)
{
    hal_dma_state_t state;
    state = hal_dma_get_state(p_dma);
    if ((state != HAL_DMA_STATE_RESET) && (state != HAL_DMA_STATE_READY))
    {
        return HAL_PM_ACTIVE;
    }
    else
    {
        hal_dma_suspend_reg(p_dma);
        return HAL_PM_SLEEP;
    }
}

__WEAK void hal_pm_dma_resume(dma_handle_t *p_dma)
{
    hal_dma_resume_reg(p_dma);
}
#endif /* HAL_PM_ENABLE */

__WEAK void hal_dma_suspend_channel(dma_handle_t *p_dma, uint32_t channel)
{
    ll_dma_suspend_channel(p_dma->p_instance, channel);
}

__WEAK void hal_dma_resume_channel(dma_handle_t *p_dma, uint32_t channel)
{
    ll_dma_resume_channel(p_dma->p_instance, channel);
}

#endif


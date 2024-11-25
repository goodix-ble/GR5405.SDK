/**
  ****************************************************************************************
  * @file    hal_spi.c
  * @author  BLE Driver Team
  * @brief   SPI HAL module driver.
  ****************************************************************************************
  * @attention
  #####Copyright (c) 2020 GOODIX
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

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6100100)
#else
#pragma O2
#endif

//lint -e9026 [advisory] Function-like macro defined
#ifdef  USE_GLOBLE_DISABLE
#define HAL_GLOBAL_EXCEPTION_DISABLE()  GLOBAL_EXCEPTION_DISABLE()
#define HAL_GLOBAL_EXCEPTION_ENABLE()   GLOBAL_EXCEPTION_ENABLE()
#else
#define HAL_GLOBAL_EXCEPTION_DISABLE()  ((void)0U)
#define HAL_GLOBAL_EXCEPTION_ENABLE()   ((void)0U)
#endif

#ifdef HAL_SPI_MODULE_ENABLED
#include "hal_spi.h"

#ifdef HAL_CLOCK_MODULE
#include "hal_cgc.h"
#endif

typedef struct {
    uint32_t data_size;                         /* @ref DATA_SIZE */
    uint32_t transfer_direction;                /* transfer direction */
    uint32_t transfer_length;                   /* used in std mode */
} spi_config_t;

/* Don't Change the Default Value */
#define SPI_CONFIG_DEFAULT_VAL()                              \
    {                                                         \
        .data_size            = SPI_DATASIZE_8BIT,            \
        .transfer_direction   = SPI_DIRECTION_FULL_DUPLEX,    \
        .transfer_length      = 0U,                           \
    }

#define SPI_SOFT_CS_ASSERT(handle, state) \
            if(SPI_SOFT_CS_MAGIC_NUMBER == p_spi->soft_cs_magic) {  \
                hal_spi_soft_cs_assert(handle, state);              \
            }

#define SPI_SOFT_CS_DEASSERT(handle, state) \
            if(SPI_SOFT_CS_MAGIC_NUMBER == p_spi->soft_cs_magic) {  \
                hal_spi_soft_cs_deassert(handle, state);            \
            }

#define SPI_DMA_SG_LLP_DEFAULT_VAL()          \
    {                                          \
        .scatter_config = {DMA_DST_SCATTER_DISABLE,0,0},                        \
        .gather_config  = {DMA_SRC_GATHER_DISABLE,0,0},                         \
        .llp_config     = {DMA_LLP_SRC_DISABLE,0,DMA_LLP_DST_DISABLE,0,NULL}    \
    }

#define SPI_SET_DMA_XFER_TR_WIDTH_AND_MSIZE(dma, SRC_TR_WIDTH, DST_TR_WIDTH, SRC_MSIZE, DST_MSIZE)              \
    do {                                                                                                    \
        ll_dma_set_source_width(dma->p_instance, dma->channel, SRC_TR_WIDTH);           \
        ll_dma_set_destination_width(dma->p_instance, dma->channel, DST_TR_WIDTH);      \
        ll_dma_set_source_burst_length(dma->p_instance, dma->channel, SRC_MSIZE);       \
        ll_dma_set_destination_burst_length(dma->p_instance, dma->channel, DST_MSIZE);  \
    } while(0)



/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void spi_dma_rx_cplt(dma_handle_t *p_dma);
static void spi_dma_tx_cplt(dma_handle_t *p_dma);
static void spi_dma_error(dma_handle_t *p_dma);
static void spi_dma_abort_cplt(dma_handle_t *p_dma);
static hal_status_t spi_wait_flag_state_until_timeout(spi_handle_t *p_spi, uint32_t flag, flag_status_t state, uint32_t timeout);
static void spi_config(spi_handle_t *p_spi, spi_config_t * p_config);
static void spi_send_8bit(spi_handle_t *p_spi);
static void spi_send_16bit(spi_handle_t *p_spi);
static void spi_send_32bit(spi_handle_t *p_spi);
static void spi_receive_8bit(spi_handle_t *p_spi);
static void spi_receive_16bit(spi_handle_t *p_spi);
static void spi_receive_32bit(spi_handle_t *p_spi);
static void spi_send_receive_8bit(spi_handle_t *p_spi);
static void spi_send_receive_16bit(spi_handle_t *p_spi);
static void spi_send_receive_32bit(spi_handle_t *p_spi);
static hal_status_t spi_transmit(spi_handle_t *p_spi, uint32_t timeout);
static hal_status_t spi_receive(spi_handle_t *p_spi, uint32_t timeout);
static hal_status_t spi_transmit_receive(spi_handle_t *p_spi, uint32_t timeout);
static hal_status_t spi_read_eeprom(spi_handle_t *p_spi, uint32_t timeout);
static void spi_update_dma_configure(spi_handle_t *p_spi, hal_spi_state_t xfer_state);
static void spi_set_device_state(spi_handle_t *p_spi, hal_spi_state_t state);

/**
  * @brief  Set spim registers to their reset values.
  * @param  SPIx SSI instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: spi registers are de-initialized
  *          - ERROR: invalid spi instance
  */
__WEAK void ll_spim_deinit(spi_regs_t *SPIx)
{
    /* Check the parameters */
    gr_assert_param(IS_SPI_ALL_INSTANCE(SPIx));

    WRITE_REG(SPIx->SSI_EN, 0U);
    WRITE_REG(SPIx->CTRL0, LL_SPI_DATASIZE_32BIT);
    WRITE_REG(SPIx->CTRL1, 0U);
    WRITE_REG(SPIx->MW_CTRL, 0U);
    WRITE_REG(SPIx->S_EN, 0U);
    WRITE_REG(SPIx->BAUD, 0U);
    WRITE_REG(SPIx->INT_MASK, 0U);
    WRITE_REG(SPIx->DMA_CTRL, 0U);
}

/**
  * @brief  Set the fields of the spim unit configuration data structure
  *         to their default values.
  * @param  p_spi_init pointer to a @ref ll_spim_init_t structure (spim unit configuration data structure)
  * @retval None
  */
__WEAK void ll_spim_struct_init(ll_spim_init_t *p_spi_init)
{
    /* Update SystemCoreClock */
    SystemCoreUpdateClock();
    /* Set the default configuration */
    p_spi_init->transfer_direction  = LL_SPI_FULL_DUPLEX;
    p_spi_init->data_size           = LL_SPI_DATASIZE_8BIT;
    p_spi_init->clock_polarity      = LL_SPI_SCPOL_LOW;
    p_spi_init->clock_phase         = LL_SPI_SCPHA_1EDGE;
    p_spi_init->slave_select        = LL_SPI_SLAVE0;
    p_spi_init->baud_rate           = 32;
    p_spi_init->rx_sample_delay     = 0U;
}

/**
  * @brief  Configure the spim unit.
  * @param  SPIx SSI instance
  * @param  p_spi_init pointer to a @ref ll_spim_init_t structure (spim unit configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: spi registers are de-initialized
  *          - ERROR: not applicable
  */
__WEAK void ll_spim_init(spi_regs_t *SPIx, const ll_spim_init_t *p_spi_init)
{
    /* Check the parameters */
    gr_assert_param(IS_SPI_ALL_INSTANCE(SPIx));

    if (!ll_spi_is_enabled(SPIx))
    {
        ll_spi_set_baud_rate_prescaler(SPIx, p_spi_init->baud_rate);
        ll_spi_disable_ss_toggle(SPIx);
        ll_spi_enable_ss(SPIx, p_spi_init->slave_select);
        ll_spi_set_data_size(SPIx, p_spi_init->data_size);
        ll_spi_set_clock_polarity(SPIx, p_spi_init->clock_polarity);
        ll_spi_set_clock_phase(SPIx, p_spi_init->clock_phase);
        ll_spi_set_rx_sample_delay(SPIx, p_spi_init->rx_sample_delay);
        ll_spi_set_transfer_direction(SPIx, p_spi_init->transfer_direction);
        ll_spi_disable_it(SPIx, LL_SPI_IM_MST | LL_SPI_IM_RXF | LL_SPI_IM_RXO | LL_SPI_IM_RXU | LL_SPI_IM_TXO | LL_SPI_IM_TXE);
    }
}

/**
  * @brief  Set spis registers to their reset values.
  * @param  SPIx SSI instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: spi registers are de-initialized
  *          - ERROR: invalid spi instance
  */
__WEAK void ll_spis_deinit(spi_regs_t *SPIx)
{
    /* Check the parameters */
    gr_assert_param(IS_SPI_ALL_INSTANCE(SPIx));

    WRITE_REG(SPIx->SSI_EN, 0U);
    WRITE_REG(SPIx->CTRL0, LL_SPI_DATASIZE_32BIT);
    WRITE_REG(SPIx->MW_CTRL, 0U);
    WRITE_REG(SPIx->INT_MASK, 0U);
    WRITE_REG(SPIx->DMA_CTRL, 0U);
}

/**
  * @brief  Set the fields of the spis unit configuration data structure
  *         to their default values.
  * @param  p_spi_init pointer to a @ref ll_spis_init_t structure (spis unit configuration data structure)
  * @retval None
  */
__WEAK void ll_spis_struct_init(ll_spis_init_t *p_spi_init)
{
    /* Set the default configuration */
    p_spi_init->data_size           = LL_SPI_DATASIZE_8BIT;
    p_spi_init->clock_polarity      = LL_SPI_SCPOL_LOW;
    p_spi_init->clock_phase         = LL_SPI_SCPHA_1EDGE;
}

/**
  * @brief  Configure the spis unit.
  * @param  SPIx SSI instance
  * @param  p_spi_init pointer to a @ref ll_spis_init_t structure (spis unit configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: spi registers are de-initialized
  *          - ERROR: not applicable
  */
__WEAK void ll_spis_init(spi_regs_t *SPIx, const ll_spis_init_t *p_spi_init)
{
    /* Check the parameters */
    gr_assert_param(IS_SPI_ALL_INSTANCE(SPIx));

    if (!ll_spi_is_enabled(SPIx))
    {
        ll_spi_disable_ss_toggle(SPIx);
        ll_spi_enable_slave_out(SPIx);
        ll_spi_set_data_size(SPIx, p_spi_init->data_size);
        ll_spi_set_clock_polarity(SPIx, p_spi_init->clock_polarity);
        ll_spi_set_clock_phase(SPIx, p_spi_init->clock_phase);
        ll_spi_disable_it(SPIx, LL_SPI_IM_RXF | LL_SPI_IM_RXO | LL_SPI_IM_RXU | LL_SPI_IM_TXO | LL_SPI_IM_TXE);
    }
}


__WEAK hal_status_t hal_spi_init(spi_handle_t *p_spi)
{
    hal_status_t   status    = HAL_ERROR;
    ll_spim_init_t spim_init = LL_SPIM_DEFAULT_CONFIG;
    ll_spis_init_t spis_init = LL_SPIS_DEFAULT_CONFIG;

    /* Check the parameters */
    gr_assert_param(IS_SPI_ALL_INSTANCE(p_spi->p_instance));
    //lint -e923 Cast from pointer to unsigned int is necessary
    if (SPIM == p_spi->p_instance)
    {
        gr_assert_param(IS_SPI_DATASIZE(p_spi->init.data_size));
        gr_assert_param(IS_SPI_CPOL(p_spi->init.clock_polarity));
        gr_assert_param(IS_SPI_CPHA(p_spi->init.clock_phase));
        gr_assert_param(IS_SPI_BAUDRATE_PRESCALER(p_spi->init.baudrate_prescaler));
        gr_assert_param(IS_SPI_TIMODE(p_spi->init.ti_mode));
        gr_assert_param(IS_SPI_SLAVE(p_spi->init.slave_select));
        gr_assert_param(IS_SPI_RX_SAMPLE_DLY(p_spi->init.rx_sample_delay));
    }
    else
    {
        gr_assert_param(IS_SPI_DATASIZE(p_spi->init.data_size));
        gr_assert_param(IS_SPI_CPOL(p_spi->init.clock_polarity));
        gr_assert_param(IS_SPI_CPHA(p_spi->init.clock_phase));
    }

    if (HAL_SPI_STATE_RESET == p_spi->state)
    {
#ifdef HAL_CLOCK_MODULE
        /* Enable Clock for Serial blocks and Automatic turn off Serial blocks clock during WFI. */
        ll_cgc_disable_force_off_serial_hclk();
        ll_cgc_disable_wfi_off_serial_hclk();

        /* Enable SPIM/SPIS Clock */
        if(p_spi->p_instance == SPIM)
        {
            ll_cgc_disable_force_off_spim_hclk();
            ll_cgc_disable_spi_m_slp_wfi();
        }
        else if(p_spi->p_instance == SPIS)
        {
            ll_cgc_disable_force_off_spis_hclk();
            ll_cgc_disable_spi_s_slp_wfi();
        }
        else
        {
            /* Nothing to do */
        }
#endif

        /* init the low level hardware : GPIO, CLOCK, NVIC, DMA */
        hal_spi_msp_init(p_spi);

        /* Configure the default timeout for the SPI memory access */
        hal_spi_set_timeout(p_spi, HAL_SPI_TIMEOUT_DEFAULT_VALUE);
    }

    /* Configure SPI FIFO Threshold */
    ll_spi_set_tx_fifo_threshold(p_spi->p_instance, 0U);
    ll_spi_set_rx_fifo_threshold(p_spi->p_instance, 0U);
    ll_spi_set_dma_tx_fifo_threshold(p_spi->p_instance, 4U);
    ll_spi_set_dma_rx_fifo_threshold(p_spi->p_instance, 0U);

    /* Wait till BUSY flag reset */
    status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, p_spi->timeout);

    if (HAL_OK == status)
    {
        /* Disable the SPI peripheral */
        __HAL_SPI_DISABLE(p_spi);
        /* Configure SPI Clock Prescaler and Clock Mode */
        if (SPIM == p_spi->p_instance)
        {
            spim_init.data_size      = p_spi->init.data_size;
            spim_init.clock_polarity = p_spi->init.clock_polarity;
            spim_init.clock_phase    = p_spi->init.clock_phase;
            spim_init.slave_select   = p_spi->init.slave_select;
            spim_init.baud_rate      = p_spi->init.baudrate_prescaler;
            spim_init.rx_sample_delay= p_spi->init.rx_sample_delay;
            //lint -e934 Taking address of near auto variable is necessary
            ll_spim_init(p_spi->p_instance, &spim_init);
        }
        else
        {
            spis_init.data_size = p_spi->init.data_size;
            spis_init.clock_polarity = p_spi->init.clock_polarity;
            spis_init.clock_phase = p_spi->init.clock_phase;
            ll_spis_init(p_spi->p_instance, &spis_init);
        }
        ll_spi_set_standard(p_spi->p_instance, p_spi->init.ti_mode);

        /* Enable the SPI peripheral */
        __HAL_SPI_ENABLE(p_spi);

        /* Set write/read fifo interface */
        if (p_spi->init.data_size <= SPI_DATASIZE_8BIT)
        {
            p_spi->write_fifo = spi_send_8bit;
            p_spi->read_fifo = spi_receive_8bit;
            p_spi->read_write_fifo = spi_send_receive_8bit;
        }
        else if (p_spi->init.data_size <= SPI_DATASIZE_16BIT)
        {
            p_spi->write_fifo = spi_send_16bit;
            p_spi->read_fifo = spi_receive_16bit;
            p_spi->read_write_fifo = spi_send_receive_16bit;
        }
        else
        {
            p_spi->write_fifo = spi_send_32bit;
            p_spi->read_fifo = spi_receive_32bit;
            p_spi->read_write_fifo = spi_send_receive_32bit;
        }

        /* Set SPI error code to none */
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Initialize the SPI state */
        spi_set_device_state(p_spi,HAL_SPI_STATE_READY);
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_spi_deinit(spi_handle_t *p_spi)
{
    if (p_spi->state != HAL_SPI_STATE_RESET)
    {
        /* Disable the SPI Peripheral Clock */
        if (SPIM == p_spi->p_instance)
        {
            ll_spim_deinit(p_spi->p_instance);
        }
        else
        {
            ll_spis_deinit(p_spi->p_instance);
        }

        /* DeInit the low level hardware: GPIO, CLOCK, NVIC... */
        hal_spi_msp_deinit(p_spi);

#ifdef HAL_CLOCK_MODULE
        /* Disable SPIM/SPIS Clock */
        if(p_spi->p_instance == SPIM)
        {
            ll_cgc_enable_force_off_spim_hclk();
            ll_cgc_enable_spi_m_slp_wfi();
        }
        else if(p_spi->p_instance == SPIS)
        {
            ll_cgc_enable_force_off_spis_hclk();
            ll_cgc_enable_spi_s_slp_wfi();
        }
        else
        {
            /*Do nothing */
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

        /* Set SPI error code to none */
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Initialize the SPI state */
        spi_set_device_state(p_spi,HAL_SPI_STATE_RESET);
    }

    return HAL_OK;
}

__WEAK void hal_spi_soft_cs_assert(spi_handle_t *p_spi, uint32_t state) {

    /* implement if controlling the CS Signal by software
     * assert the cs signal line
     */
    UNUSED(p_spi);
    UNUSED(state);
    return;
}

__WEAK void hal_spi_soft_cs_deassert(spi_handle_t *p_spi, uint32_t state) {
    /* implement if controlling the CS Signal by software
     * de-assert the cs signal line
     */
    UNUSED(p_spi);
    UNUSED(state);
    return;
}

__WEAK void hal_spi_msp_init(spi_handle_t *p_spi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_spi);
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
              the hal_spi_msp_init can be implemented in the user file
     */
}

__WEAK void hal_spi_msp_deinit(spi_handle_t *p_spi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_spi);
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
              the hal_spi_msp_deinit can be implemented in the user file
     */
}

__WEAK void hal_spi_irq_handler(spi_handle_t *p_spi)
{
    uint32_t itsource = READ_REG(p_spi->p_instance->INT_STAT);

    if (itsource & (SPI_IT_MST | SPI_IT_RXO | SPI_IT_RXU | SPI_IT_TXO))
    {
        ll_spi_clear_flag_all(p_spi->p_instance);

        /* Disable all the SPI Interrupts */
        __HAL_SPI_DISABLE_IT(p_spi, (SPI_IT_MST | SPI_IT_RXF | SPI_IT_RXO | SPI_IT_RXU | SPI_IT_TXO | SPI_IT_TXE));

        /* Set error code */
        p_spi->error_code |= HAL_SPI_ERROR_TRANSFER;

        SPI_SOFT_CS_DEASSERT(p_spi, CS_END_STATE_XFER_ERR);

        if (0U != p_spi->p_instance->DMA_CTRL)
        {
            /* Disable the DMA transfer by clearing the DMAEN bit in the SSI DMAC register */
            CLEAR_REG(p_spi->p_instance->DMA_CTRL);

            /* Abort DMA channel */
            p_spi->p_dmatx->xfer_abort_callback = spi_dma_abort_cplt;
            p_spi->p_dmarx->xfer_abort_callback = spi_dma_abort_cplt;
            hal_dma_abort_it(p_spi->p_dmatx); /*lint !e534 MISRA exception. Ignoring return value */
            hal_dma_abort_it(p_spi->p_dmarx); /*lint !e534 MISRA exception. Ignoring return value */
        }
        else
        {
            /* Change state of SPI */
            spi_set_device_state(p_spi,HAL_SPI_STATE_READY);

            /* Error callback */
            hal_spi_error_callback(p_spi);
        }
    }

    if (itsource & SPI_IT_RXF)
    {
        p_spi->read_fifo(p_spi);

        if (0U == p_spi->rx_xfer_count)
        {
            /* All data have been received for the transfer */
            /* Disable the SPI RX Interrupt */
            __HAL_SPI_DISABLE_IT(p_spi, SPI_IT_RXF);

            if (HAL_SPI_STATE_BUSY_RX == p_spi->state)
            {
                /* Change state of SPI */
                spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
                while(ll_spi_is_active_flag(p_spi->p_instance, LL_SPI_SR_BUSY)){}
                SPI_SOFT_CS_DEASSERT(p_spi, CS_END_STATE_RX_CPLT);
                hal_spi_rx_cplt_callback(p_spi);
            }
            else if (HAL_SPI_STATE_BUSY_TX_RX == p_spi->state)
            {
                /* Change state of SPI */
                spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
                while(ll_spi_is_active_flag(p_spi->p_instance, LL_SPI_SR_BUSY)){}
                SPI_SOFT_CS_DEASSERT(p_spi, CS_END_STATE_TX_RX_CPLT);
                hal_spi_tx_rx_cplt_callback(p_spi);
            }
            else if (HAL_SPI_STATE_ABORT == p_spi->state)
            {
                /* Change state of SPI */
                spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
                __HAL_SPI_DISABLE(p_spi);
                __HAL_SPI_ENABLE(p_spi);
                while(ll_spi_is_active_flag(p_spi->p_instance, LL_SPI_SR_BUSY)){}
                SPI_SOFT_CS_DEASSERT(p_spi, CS_END_STATE_RX_ABORT_CPLT);
                hal_spi_abort_cplt_callback(p_spi);
            }
            else
            {
                /* Do nothing */
            }
        }
    }

    if (itsource & SPI_IT_TXE)
    {
        if (0U == p_spi->tx_xfer_count)
        {
            /* All data have been sended for the transfer */
            /* Disable the SPI TX Interrupt */
            __HAL_SPI_DISABLE_IT(p_spi, SPI_IT_TXE);

            if (HAL_SPI_STATE_BUSY_TX == p_spi->state)
            {
                /* Change state of SPI */
                spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
                while(ll_spi_is_active_flag(p_spi->p_instance, LL_SPI_SR_BUSY)){}
                SPI_SOFT_CS_DEASSERT(p_spi, CS_END_STATE_TX_CPLT);
                hal_spi_tx_cplt_callback(p_spi);
            }
            else if (HAL_SPI_STATE_ABORT == p_spi->state)
            {
                /* Change state of SPI */
                spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
                __HAL_SPI_DISABLE(p_spi);
                __HAL_SPI_ENABLE(p_spi);
                while(ll_spi_is_active_flag(p_spi->p_instance, LL_SPI_SR_BUSY)){}
                SPI_SOFT_CS_DEASSERT(p_spi, CS_END_STATE_TX_ABORT_CPLT);
                hal_spi_abort_cplt_callback(p_spi);
            }
            else
            {
                /* Do nothing */
            }
        }
        else
        {
            p_spi->write_fifo(p_spi);

            if (0U == p_spi->tx_xfer_count)
            {
                ll_spi_set_tx_fifo_threshold(p_spi->p_instance, 0U);
            }
        }
    }
}

/*
 ************************************************************************************************
 * Polling API family :
 *      hal_spi_transmit          : for TX only
 *      hal_spi_receive           : for RX only
 *      hal_spi_transmit_receive  : for ful-dulplex
 *      hal_spi_read_eeprom       : for read eeprom/flash
 ************************************************************************************************
 */

__WEAK hal_status_t hal_spi_transmit(spi_handle_t *p_spi, uint8_t *p_data, uint32_t length, uint32_t timeout)
{
    hal_status_t status   = HAL_OK;
    spi_config_t s_config = SPI_CONFIG_DEFAULT_VAL();

    if ((NULL == p_data) || (0U == length))
    {
        p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
        return  HAL_ERROR;
    }

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Update SPI state */
        spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY);

        /* Wait till BUSY flag reset */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, timeout);

        if (HAL_OK == status)
        {
            /* Call the configuration function */
            s_config.data_size          = p_spi->init.data_size;
            s_config.transfer_length    = length;
            s_config.transfer_direction = SPI_DIRECTION_SIMPLEX_TX;
            spi_config(p_spi, &s_config);

            /* Configure counters and size of the handle */
            p_spi->tx_xfer_count = length;
            p_spi->tx_xfer_size  = length;
            p_spi->p_tx_buffer   = p_data;

            p_spi->rx_xfer_count = 0U;
            p_spi->rx_xfer_size  = 0U;
            p_spi->p_rx_buffer   = NULL;

            spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY_TX);

            HAL_GLOBAL_EXCEPTION_DISABLE();
            SPI_SOFT_CS_ASSERT(p_spi,   CS_STA_STATE_POLL_TX);
            status = spi_transmit(p_spi, timeout);
            SPI_SOFT_CS_DEASSERT(p_spi, CS_END_STATE_POLL_TX);
            HAL_GLOBAL_EXCEPTION_ENABLE();
        }

        spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_spi_receive(spi_handle_t *p_spi, uint8_t *p_data, uint32_t length, uint32_t timeout)
{
    hal_status_t status   = HAL_OK;
    spi_config_t s_config = SPI_CONFIG_DEFAULT_VAL();

    if ((NULL == p_data) || (0U == length))
    {
        p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
        return  HAL_ERROR;
    }

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Update SPI state */
        spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY);

        /* Wait till BUSY flag reset */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, timeout);

        if (HAL_OK == status)
        {
            /* Call the configuration function */
            s_config.data_size          = p_spi->init.data_size;
            s_config.transfer_length    = length;
            s_config.transfer_direction = SPI_DIRECTION_SIMPLEX_RX;
            spi_config(p_spi, &s_config);

            /* Configure counters and size of the handle */
            p_spi->rx_xfer_count = length;
            p_spi->rx_xfer_size  = length;
            p_spi->p_rx_buffer   = p_data;

            p_spi->tx_xfer_count = 0U;
            p_spi->tx_xfer_size  = 0U;
            p_spi->p_tx_buffer   = NULL;

            spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY_RX);

            HAL_GLOBAL_EXCEPTION_DISABLE();
            SPI_SOFT_CS_ASSERT(p_spi,    CS_STA_STATE_POLL_RX);
            /* Send a dummy byte to start transfer */
            if (SPIM == p_spi->p_instance)
            {
                p_spi->p_instance->DATA = 0xFFFFFFFFU;
            }
            status = spi_receive(p_spi, timeout);
            SPI_SOFT_CS_DEASSERT(p_spi,   CS_END_STATE_POLL_RX);
            HAL_GLOBAL_EXCEPTION_ENABLE();
        }

        spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_spi_transmit_receive(spi_handle_t *p_spi, uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t length, uint32_t timeout)
{
    hal_status_t status   = HAL_OK;
    spi_config_t s_config = SPI_CONFIG_DEFAULT_VAL();

    if ((NULL == p_tx_data) || (NULL == p_rx_data) || (0U == length))
    {
        p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
        return  HAL_ERROR;
    }

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Update SPI state */
        spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY);

        /* Wait till BUSY flag reset */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, timeout);

        if (HAL_OK == status)
        {
            /* Call the configuration function */
            s_config.data_size          = p_spi->init.data_size;
            s_config.transfer_length    = length;
            s_config.transfer_direction = SPI_DIRECTION_FULL_DUPLEX;
            spi_config(p_spi, &s_config);

            /* Configure counters and size of the handle */
            p_spi->rx_xfer_count = length;
            p_spi->rx_xfer_size  = length;
            p_spi->p_rx_buffer   = p_rx_data;

            p_spi->tx_xfer_count = length;
            p_spi->tx_xfer_size  = length;
            p_spi->p_tx_buffer   = p_tx_data;

            spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY_TX_RX);

            HAL_GLOBAL_EXCEPTION_DISABLE();
            SPI_SOFT_CS_ASSERT(p_spi,   CS_STA_STATE_POLL_TX_RX);
            status = spi_transmit_receive(p_spi, timeout);
            SPI_SOFT_CS_DEASSERT(p_spi, CS_END_STATE_POLL_TX_RX);
            HAL_GLOBAL_EXCEPTION_ENABLE();
        }

        spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_spi_read_eeprom(spi_handle_t *p_spi,
                                        uint8_t      *p_tx_data,
                                        uint8_t      *p_rx_data,
                                        uint32_t      tx_number_data,
                                        uint32_t      rx_number_data,
                                        uint32_t      timeout)
{
    hal_status_t status   = HAL_OK;
    spi_config_t s_config = SPI_CONFIG_DEFAULT_VAL();

    /* SPIS can't in this mode */
    if (SPIS == p_spi->p_instance)
    {
        p_spi->error_code = HAL_SPI_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Update SPI state */
        spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY);

        /* Wait till BUSY flag reset */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, timeout);

        if (HAL_OK == status)
        {
            /* Call the configuration function */
            s_config.data_size          = p_spi->init.data_size;
            s_config.transfer_length    = rx_number_data;
            s_config.transfer_direction = SPI_DIRECTION_READ_EEPROM;

            spi_config(p_spi, &s_config);

            /* Configure counters and size of the handle */
            p_spi->rx_xfer_count = rx_number_data;
            p_spi->rx_xfer_size  = rx_number_data;
            p_spi->p_rx_buffer   = p_rx_data;

            p_spi->tx_xfer_count = tx_number_data;
            p_spi->tx_xfer_size  = tx_number_data;
            p_spi->p_tx_buffer   = p_tx_data;

            spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY_TX_RX);

            HAL_GLOBAL_EXCEPTION_DISABLE();
            SPI_SOFT_CS_ASSERT(p_spi,   CS_STA_STATE_POLL_EEPREAD);
            status = spi_read_eeprom(p_spi, timeout);
            SPI_SOFT_CS_DEASSERT(p_spi, CS_END_STATE_POLL_EEPREAD);
            HAL_GLOBAL_EXCEPTION_ENABLE();
        }

        spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

/*
 ************************************************************************************************
 * Interrupt API family :
 *      hal_spi_transmit_it          : for TX only
 *      hal_spi_receive_it           : for RX only
 *      hal_spi_transmit_receive_it  : for ful-dulplex
 *      hal_spi_read_eeprom_it       : for read eeprom/flash
 ************************************************************************************************
 */
__WEAK hal_status_t hal_spi_transmit_it(spi_handle_t *p_spi, uint8_t *p_data, uint32_t length)
{
    hal_status_t status   = HAL_OK;
    spi_config_t s_config = SPI_CONFIG_DEFAULT_VAL();

    if ((NULL == p_data) || (0U == length))
    {
        p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
        return  HAL_ERROR;
    }

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Update SPI state */
        spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY);

        /* Wait till BUSY flag reset */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, p_spi->timeout);

        if (HAL_OK == status)
        {
            /* Call the configuration function */
            s_config.data_size          = p_spi->init.data_size;
            s_config.transfer_length    = length;
            s_config.transfer_direction = SPI_DIRECTION_SIMPLEX_TX;
            spi_config(p_spi, &s_config);

            /* Configure counters and size of the handle */
            p_spi->tx_xfer_count = length;
            p_spi->tx_xfer_size  = length;
            p_spi->p_tx_buffer   = p_data;

            p_spi->rx_xfer_count = 0U;
            p_spi->rx_xfer_size  = 0U;
            p_spi->p_rx_buffer   = NULL;

            /* Update SPI state */
            spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY_TX);

            ll_spi_set_tx_fifo_threshold(p_spi->p_instance, 4U);

            SPI_SOFT_CS_ASSERT(p_spi,   CS_STA_STATE_IT_TX);
            __HAL_SPI_ENABLE_IT(p_spi, SPI_IT_TXE);
        }
        else
        {
            /* Update SPI state */
            spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_spi_receive_it(spi_handle_t *p_spi, uint8_t *p_data, uint32_t length)
{
    hal_status_t status   = HAL_OK;
    spi_config_t s_config = SPI_CONFIG_DEFAULT_VAL();

    if ((NULL == p_data) || (0U == length))
    {
        p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
        return  HAL_ERROR;
    }

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Update SPI state */
        spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY);

        /* Wait till BUSY flag reset */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, p_spi->timeout);

        if (HAL_OK == status)
        {
            /* Call the configuration function */
            s_config.data_size          = p_spi->init.data_size;
            s_config.transfer_length    = length;
            s_config.transfer_direction = SPI_DIRECTION_SIMPLEX_RX;
            spi_config(p_spi, &s_config);

            /* Configure counters and size of the handle */
            p_spi->rx_xfer_count = length;
            p_spi->rx_xfer_size  = length;
            p_spi->p_rx_buffer   = p_data;

            p_spi->tx_xfer_count = 0U;
            p_spi->tx_xfer_size  = 0U;
            p_spi->p_tx_buffer   = NULL;

            /* Update SPI state */
            spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY_RX);

            SPI_SOFT_CS_ASSERT(p_spi,   CS_STA_STATE_IT_RX);
            /* Send a dummy byte to start transfer */
            __HAL_SPI_ENABLE_IT(p_spi, SPI_IT_RXF | SPI_IT_RXO);
            if (SPIM == p_spi->p_instance)
            {
                p_spi->p_instance->DATA = 0xFFFFFFFFU;
            }
        }
        else
        {
            /* Update SPI state */
            spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_spi_transmit_receive_it(spi_handle_t *p_spi, uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t length)
{
    hal_status_t status   = HAL_OK;
    spi_config_t s_config = SPI_CONFIG_DEFAULT_VAL();

    if ((NULL == p_tx_data) || (NULL == p_rx_data) || (0U == length))
    {
        p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
        return  HAL_ERROR;
    }

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Update SPI state */
        spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY);

        /* Wait till BUSY flag reset */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, p_spi->timeout);

        if (HAL_OK == status)
        {
            /* Call the configuration function */
            s_config.data_size          = p_spi->init.data_size;
            s_config.transfer_length    = length;
            s_config.transfer_direction = SPI_DIRECTION_FULL_DUPLEX;
            spi_config(p_spi, &s_config);

            /* Configure counters and size of the handle */
            p_spi->rx_xfer_count = length;
            p_spi->rx_xfer_size  = length;
            p_spi->p_rx_buffer   = p_rx_data;

            p_spi->tx_xfer_count = length;
            p_spi->tx_xfer_size  = length;
            p_spi->p_tx_buffer   = p_tx_data;

            /* Update SPI state */
            spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY_TX_RX);

            SPI_SOFT_CS_ASSERT(p_spi,   CS_STA_STATE_IT_TX_RX);
            ll_spi_set_tx_fifo_threshold(p_spi->p_instance, 4U);
            __HAL_SPI_ENABLE_IT(p_spi, SPI_IT_RXF | SPI_IT_RXO | SPI_IT_TXE);
        }
        else
        {
            /* Update SPI state */
            spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_spi_read_eeprom_it(spi_handle_t *p_spi, uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t tx_number_data, uint32_t rx_number_data)
{
    hal_status_t status   = HAL_OK;
    spi_config_t s_config = SPI_CONFIG_DEFAULT_VAL();

    /* SPIS can't in this mode */
    if ((NULL == p_tx_data) || (NULL == p_rx_data) || (SPIS == p_spi->p_instance))
    {
        p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
        return  HAL_ERROR;
    }

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Update SPI state */
        spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY);

        /* Wait till BUSY flag reset */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, p_spi->timeout);

        if (HAL_OK == status)
        {
            /* Call the configuration function */
            s_config.data_size          = p_spi->init.data_size;
            s_config.transfer_length    = rx_number_data;
            s_config.transfer_direction = SPI_DIRECTION_READ_EEPROM;
            spi_config(p_spi, &s_config);

            /* Configure counters and size of the handle */
            p_spi->rx_xfer_count = rx_number_data;
            p_spi->rx_xfer_size  = rx_number_data;
            p_spi->p_rx_buffer   = p_rx_data;

            p_spi->tx_xfer_count = tx_number_data;
            p_spi->tx_xfer_size  = tx_number_data;
            p_spi->p_tx_buffer   = p_tx_data;

            /* Update SPI state */
            spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY_TX_RX);

            SPI_SOFT_CS_ASSERT(p_spi,   CS_STA_STATE_IT_EEPREAD);
            ll_spi_set_tx_fifo_threshold(p_spi->p_instance, 4U);
            __HAL_SPI_ENABLE_IT(p_spi, SPI_IT_RXF | SPI_IT_RXO | SPI_IT_TXE);
        }
        else
        {
            /* Update SPI state */
            spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

/*
 ************************************************************************************************
 * DMA API family :
 *      hal_spi_transmit_dma          : for TX only
 *      hal_spi_receive_dma           : for RX only
 *      hal_spi_transmit_receive_dma  : for ful-dulplex
 *      hal_spi_read_eeprom_dma       : for read eeprom/flash
 ************************************************************************************************
 */

__WEAK hal_status_t hal_spi_transmit_dma(spi_handle_t *p_spi, uint8_t *p_data, uint32_t length)
{
    hal_status_t status   = HAL_OK;
    spi_config_t s_config = SPI_CONFIG_DEFAULT_VAL();

    if(NULL == p_data)
    {
        p_spi->error_code = HAL_SPI_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Update SPI state */
        spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY);

        /* Wait till BUSY flag reset */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, p_spi->timeout);

        if (HAL_OK == status)
        {
            if (DMA_SDATAALIGN_BYTE == p_spi->p_dmatx->init.src_data_alignment)
            {
                p_spi->tx_xfer_count = length;
                s_config.data_size  = SPI_DATASIZE_8BIT;
            }
            else if (DMA_SDATAALIGN_HALFWORD == p_spi->p_dmatx->init.src_data_alignment)
            {
                if (0U != (length % 2U))
                {
                    p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_spi->tx_xfer_count = (length >> 1U);
                    s_config.data_size  = SPI_DATASIZE_16BIT;
                }
            }
            else if (DMA_SDATAALIGN_WORD == p_spi->p_dmatx->init.src_data_alignment)
            {
                if (0U != (length % 4U))
                {
                    p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_spi->tx_xfer_count = (length >> 2U);
                    s_config.data_size  = SPI_DATASIZE_32BIT;
                }
            }
            else
            {
                /* Do nothing */
            }

            if (HAL_OK == status)
            {
                /* Call the configuration function */
                s_config.transfer_length    = length;
                s_config.transfer_direction = SPI_DIRECTION_SIMPLEX_TX;
                spi_config(p_spi, &s_config);

                /* Configure counters and size of the handle */
                p_spi->tx_xfer_size = p_spi->tx_xfer_count;
                p_spi->p_tx_buffer = p_data;

                /* Update DMA configure to avoid the mis-matched parameters from upper layer */
                spi_update_dma_configure(p_spi, HAL_SPI_STATE_BUSY_TX);

                /* Enable the SPI transmit DMA Channel */
                status = hal_dma_start_it(p_spi->p_dmatx, (uint32_t)p_data, (uint32_t)&p_spi->p_instance->DATA, p_spi->tx_xfer_size);

                if (HAL_OK == status)
                {
                    SPI_SOFT_CS_ASSERT(p_spi,   CS_STA_STATE_DMA_TX);
                    __HAL_SPI_ENABLE_DMATX(p_spi);
                }
                else
                {
                    p_spi->error_code |= HAL_SPI_ERROR_DMA;

                    /* Update SPI state */
                    spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
                }
            }
            else
            {
                /* Update SPI state */
                spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
            }
        }
        else
        {
            /* Update SPI state */
            spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_spi_receive_dma(spi_handle_t *p_spi, uint8_t *p_data, uint32_t length)
{
    hal_status_t status   = HAL_OK;
    spi_config_t s_config = SPI_CONFIG_DEFAULT_VAL();

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Update SPI state */
        spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY);

        /* Wait till BUSY flag reset */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, p_spi->timeout);

        if (HAL_OK == status)
        {
            if (DMA_SDATAALIGN_BYTE == p_spi->p_dmarx->init.src_data_alignment)
            {
                p_spi->rx_xfer_count = length;
                s_config.data_size   = SPI_DATASIZE_8BIT;
            }
            else if (DMA_SDATAALIGN_HALFWORD == p_spi->p_dmarx->init.src_data_alignment)
            {
                if (0U != (length % 2U))
                {
                    p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_spi->rx_xfer_count = (length >> 1U);
                    s_config.data_size   = SPI_DATASIZE_16BIT;
                }
            }
            else if (DMA_SDATAALIGN_WORD == p_spi->p_dmarx->init.src_data_alignment)
            {
                if (0U != (length % 4U))
                {
                    p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_spi->rx_xfer_count = (length >> 2U);
                    s_config.data_size   = SPI_DATASIZE_32BIT;
                }
            }
            else
            {
                /* Do nothing */
            }

            if (HAL_OK == status)
            {
                /* Call the configuration function */
                s_config.transfer_length    = length;
                s_config.transfer_direction = SPI_DIRECTION_SIMPLEX_RX;
                spi_config(p_spi, &s_config);

                /* Configure counters and size of the handle */
                p_spi->rx_xfer_size = p_spi->rx_xfer_count;
                p_spi->p_rx_buffer = p_data;

                /* Update DMA configure to avoid the mis-matched parameters from upper layer */
                spi_update_dma_configure(p_spi, HAL_SPI_STATE_BUSY_RX);

                /* Enable the SPI transmit DMA Channel */
                status = hal_dma_start_it(p_spi->p_dmarx, (uint32_t)&p_spi->p_instance->DATA, (uint32_t)p_data, p_spi->rx_xfer_size);

                if (HAL_OK == status)
                {
                    SPI_SOFT_CS_ASSERT(p_spi,   CS_STA_STATE_DMA_RX);
                    /* Send a dummy byte to start transfer */
                    __HAL_SPI_ENABLE_DMARX(p_spi);
                    if (SPIM == p_spi->p_instance)
                    {
                        p_spi->p_instance->DATA = 0xFFFFFFFFU;
                    }
                }
                else
                {
                    p_spi->error_code |= HAL_SPI_ERROR_DMA;

                    /* Update SPI state */
                    spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
                }
            }
            else
            {
                /* Update SPI state */
                spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
            }
        }
        else
        {
            /* Update SPI state */
            spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_spi_transmit_receive_dma(spi_handle_t *p_spi, uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t length)
{
    hal_status_t status   = HAL_OK;
    spi_config_t s_config = SPI_CONFIG_DEFAULT_VAL();

    if ((NULL == p_tx_data) || (NULL == p_rx_data) || (0U == length))
    {
        p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
        return  HAL_ERROR;
    }

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Update SPI state */
        spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY);

        /* Wait till BUSY flag reset */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, p_spi->timeout);

        if (HAL_OK == status)
        {
            if (DMA_SDATAALIGN_BYTE == p_spi->p_dmatx->init.src_data_alignment)
            {
                p_spi->tx_xfer_count = length;
            }
            else if (DMA_SDATAALIGN_HALFWORD == p_spi->p_dmatx->init.src_data_alignment)
            {
                if (0U != (length % 2U))
                {
                    p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_spi->tx_xfer_count = (length >> 1U);
                }
            }
            else if (DMA_SDATAALIGN_WORD == p_spi->p_dmatx->init.src_data_alignment)
            {
                if (0U != (length % 4U))
                {
                    p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_spi->tx_xfer_count = (length >> 2U);
                }
            }
            else
            {
                /* Do nothing */
            }

            if (DMA_SDATAALIGN_BYTE == p_spi->p_dmarx->init.src_data_alignment)
            {
                p_spi->rx_xfer_count = length;
                s_config.data_size   = SPI_DATASIZE_8BIT;
            }
            else if (DMA_SDATAALIGN_HALFWORD == p_spi->p_dmarx->init.src_data_alignment)
            {
                if (0U != (length % 2U))
                {
                    p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_spi->rx_xfer_count = (length >> 1U);
                    s_config.data_size   = SPI_DATASIZE_16BIT;
                }
            }
            else if (DMA_SDATAALIGN_WORD == p_spi->p_dmarx->init.src_data_alignment)
            {
                if (0U != (length % 4U))
                {
                    p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_spi->rx_xfer_count = (length >> 2U);
                    s_config.data_size   = SPI_DATASIZE_32BIT;
                }
            }
            else
            {
                /* Do nothing */
            }

            if (HAL_OK == status)
            {
                /* Call the configuration function */
                s_config.transfer_length    = length;
                s_config.transfer_direction = SPI_DIRECTION_FULL_DUPLEX;
                spi_config(p_spi, &s_config);

                /* Configure counters and size of the handle */
                p_spi->tx_xfer_size = p_spi->tx_xfer_count;
                p_spi->p_tx_buffer  = p_tx_data;
                p_spi->rx_xfer_size = p_spi->rx_xfer_count;
                p_spi->p_rx_buffer  = p_rx_data;

                /* Update DMA configure to avoid the mis-matched parameters from upper layer */
                spi_update_dma_configure(p_spi, HAL_SPI_STATE_BUSY_TX_RX);

                /* Enable the SPI transmit DMA Channel */
                status = hal_dma_start_it(p_spi->p_dmatx, (uint32_t)p_tx_data, (uint32_t)&p_spi->p_instance->DATA, p_spi->tx_xfer_size);
                status = hal_dma_start_it(p_spi->p_dmarx, (uint32_t)&p_spi->p_instance->DATA, (uint32_t)p_rx_data, p_spi->rx_xfer_size);

                if (HAL_OK == status)
                {
                    SPI_SOFT_CS_ASSERT(p_spi,   CS_STA_STATE_DMA_TX_RX);
                    __HAL_SPI_ENABLE_DMARX(p_spi);
                    __HAL_SPI_ENABLE_DMATX(p_spi);
                }
                else
                {
                    p_spi->error_code |= HAL_SPI_ERROR_DMA;

                    /* Update SPI state */
                    spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
                }
            }
            else
            {
                /* Update SPI state */
                spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
            }
        }
        else
        {
            /* Update SPI state */
            spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}


__WEAK hal_status_t hal_spi_read_eeprom_dma(spi_handle_t *p_spi, uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t tx_number_data, uint32_t rx_number_data)
{
    hal_status_t status   = HAL_OK;
    spi_config_t s_config = SPI_CONFIG_DEFAULT_VAL();

    /* SPIS can't in this mode */
    if ((SPIS == p_spi->p_instance) || (NULL == p_tx_data) || (NULL == p_rx_data))
    {
        p_spi->error_code = HAL_SPI_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Update SPI state */
        spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY);

        /* Wait till BUSY flag reset */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, p_spi->timeout);

        if (HAL_OK == status)
        {
            if (DMA_SDATAALIGN_BYTE == p_spi->p_dmatx->init.src_data_alignment)
            {
                p_spi->tx_xfer_count = tx_number_data;
            }
            else if (DMA_SDATAALIGN_HALFWORD == p_spi->p_dmatx->init.src_data_alignment)
            {
                if (0U != (tx_number_data % 2U))
                {
                    p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_spi->tx_xfer_count = (tx_number_data >> 1U);
                }
            }
            else if (DMA_SDATAALIGN_WORD == p_spi->p_dmatx->init.src_data_alignment)
            {
                if (0U != (tx_number_data % 4U))
                {
                    p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_spi->tx_xfer_count = (tx_number_data >> 2U);
                }
            }
            else
            {
                /* Do nothing */
            }

            if (DMA_SDATAALIGN_BYTE == p_spi->p_dmarx->init.src_data_alignment)
            {
                p_spi->rx_xfer_count = rx_number_data;
                s_config.data_size   = SPI_DATASIZE_8BIT;
            }
            else if (DMA_SDATAALIGN_HALFWORD == p_spi->p_dmarx->init.src_data_alignment)
            {
                if (0U != (rx_number_data % 2U))
                {
                    p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_spi->rx_xfer_count = (rx_number_data >> 1U);
                    s_config.data_size   = SPI_DATASIZE_16BIT;
                }
            }
            else if (DMA_SDATAALIGN_WORD == p_spi->p_dmarx->init.src_data_alignment)
            {
                if (0U != (rx_number_data % 4U))
                {
                    p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_spi->rx_xfer_count = (rx_number_data >> 2U);
                    s_config.data_size   = SPI_DATASIZE_32BIT;
                }
            }
            else
            {
                /* Do nothing */
            }

            if (HAL_OK == status)
            {
                /* Call the configuration function */
                s_config.transfer_length    = rx_number_data;
                s_config.transfer_direction = SPI_DIRECTION_READ_EEPROM;
                spi_config(p_spi, &s_config);

                /* Configure counters and size of the handle */
                p_spi->tx_xfer_size = p_spi->tx_xfer_count;
                p_spi->p_tx_buffer  = p_tx_data;
                p_spi->rx_xfer_size = p_spi->rx_xfer_count;
                p_spi->p_rx_buffer  = p_rx_data;

                /* Update DMA configure to avoid the mis-matched parameters from upper layer */
                spi_update_dma_configure(p_spi, HAL_SPI_STATE_BUSY_TX_RX);

                /* Enable the SPI transmit DMA Channel */
                status = hal_dma_start_it(p_spi->p_dmatx, (uint32_t)p_tx_data, (uint32_t)&p_spi->p_instance->DATA, p_spi->tx_xfer_size);
                status = hal_dma_start_it(p_spi->p_dmarx, (uint32_t)&p_spi->p_instance->DATA, (uint32_t)p_rx_data, p_spi->rx_xfer_size);

                if (HAL_OK == status)
                {
                    SPI_SOFT_CS_ASSERT(p_spi,   CS_STA_STATE_DMA_EEPREAD);
                    __HAL_SPI_ENABLE_DMARX(p_spi);
                    __HAL_SPI_ENABLE_DMATX(p_spi);
                }
                else
                {
                    p_spi->error_code |= HAL_SPI_ERROR_DMA;

                    /* Update SPI state */
                    spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
                }
            }
            else
            {
                /* Update SPI state */
                spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
            }
        }
        else
        {
            /* Update SPI state */
            spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}


__WEAK hal_status_t hal_spi_transmit_with_ia(spi_handle_t *p_spi, uint8_t inst, uint32_t addr, uint8_t *p_data, uint32_t length, uint32_t timeout)
{
    hal_status_t status   = HAL_OK;
    spi_config_t s_config = SPI_CONFIG_DEFAULT_VAL();
    uint32_t inst_addr = (addr & 0x00ffffffU) | ((uint32_t)inst << 24U);

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Update SPI state */
        spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY);

        /* Wait till BUSY flag reset */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, timeout);

        if (HAL_OK == status)
        {
            /* Call the configuration function */
            s_config.data_size          = p_spi->init.data_size;
            s_config.transfer_length    = length;
            s_config.transfer_direction = SPI_DIRECTION_SIMPLEX_TX;
            spi_config(p_spi, &s_config);

            /* Configure counters and size of the handle */
            p_spi->tx_xfer_count = length;
            p_spi->tx_xfer_size  = length;
            p_spi->p_tx_buffer   = p_data;

            p_spi->rx_xfer_count = 0U;
            p_spi->rx_xfer_size  = 0U;
            p_spi->p_rx_buffer   = NULL;

            spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY_TX);

            HAL_GLOBAL_EXCEPTION_DISABLE();
            SPI_SOFT_CS_ASSERT(p_spi,   CS_STA_STATE_POLL_TX);
            p_spi->p_instance->DATA = ((inst_addr >> 24UL ) & 0xffU);
            p_spi->p_instance->DATA = ((inst_addr >> 16UL ) & 0xffU);
            p_spi->p_instance->DATA = ((inst_addr >>  8UL ) & 0xffU);
            p_spi->p_instance->DATA = ((inst_addr >>  0UL ) & 0xffU);
            status = spi_transmit(p_spi, timeout);
            SPI_SOFT_CS_DEASSERT(p_spi, CS_END_STATE_POLL_TX);
            HAL_GLOBAL_EXCEPTION_ENABLE();
        }

        spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}


__WEAK hal_status_t hal_spi_transmit_dma_with_ia(spi_handle_t *p_spi, uint8_t inst, uint32_t addr, uint8_t *p_data, uint32_t length)
{
    hal_status_t status   = HAL_OK;
    spi_config_t s_config = SPI_CONFIG_DEFAULT_VAL();

    uint32_t inst_addr = (addr & 0x00ffffffU) | ((uint32_t)inst << 24U);

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Update SPI state */
        spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY);

        /* Wait till BUSY flag reset */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, p_spi->timeout);

        if (HAL_OK == status)
        {
            if (DMA_SDATAALIGN_BYTE == p_spi->p_dmatx->init.src_data_alignment)
            {
                p_spi->tx_xfer_count = length;
                s_config.data_size   = SPI_DATASIZE_8BIT;
            }
            else if (DMA_SDATAALIGN_HALFWORD == p_spi->p_dmatx->init.src_data_alignment)
            {
                if (0U != (length % 2U))
                {
                    p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_spi->tx_xfer_count = (length >> 1U);
                    s_config.data_size  = SPI_DATASIZE_16BIT;
                }
            }
            else if (DMA_SDATAALIGN_WORD == p_spi->p_dmatx->init.src_data_alignment)
            {
                if (0U != (length % 4U))
                {
                    p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_spi->tx_xfer_count = (length >> 2U);
                    s_config.data_size  = SPI_DATASIZE_32BIT;
                }
            }
            else
            {
                /* Do nothing */
            }

            if (HAL_OK == status)
            {
                /* Call the configuration function */
                s_config.transfer_length    = length;
                s_config.transfer_direction = SPI_DIRECTION_SIMPLEX_TX;
                spi_config(p_spi, &s_config);

                /* Configure counters and size of the handle */
                p_spi->tx_xfer_size = p_spi->tx_xfer_count;
                p_spi->p_tx_buffer  = p_data;

                /* Update DMA configure to avoid the mis-matched parameters from upper layer */
                spi_update_dma_configure(p_spi, HAL_SPI_STATE_BUSY_TX);

                /* Enable the SPI transmit DMA Channel */
                status = hal_dma_start_it(p_spi->p_dmatx, (uint32_t)p_data, (uint32_t)&p_spi->p_instance->DATA, p_spi->tx_xfer_size);

                if (HAL_OK == status)
                {
                    SPI_SOFT_CS_ASSERT(p_spi,   CS_STA_STATE_DMA_TX);
                    if(s_config.data_size == SPI_DATASIZE_8BIT) {
                        p_spi->p_instance->DATA =((inst_addr >> 24UL ) & 0xffU);
                        p_spi->p_instance->DATA =((inst_addr >> 16UL ) & 0xffU);
                        p_spi->p_instance->DATA =((inst_addr >>  8UL ) & 0xffU);
                        p_spi->p_instance->DATA =((inst_addr >>  0UL ) & 0xffU);
                    } else if(s_config.data_size == SPI_DATASIZE_16BIT) {
                        p_spi->p_instance->DATA = ((inst_addr >> 16UL ) & 0xffffU);
                        p_spi->p_instance->DATA = ((inst_addr >>  0UL ) & 0xffffU);
                    } else if(s_config.data_size == SPI_DATASIZE_32BIT) {
                        p_spi->p_instance->DATA = inst_addr;
                    }
                    else
                    {
                        /* Do nothing */
                    }
                    __HAL_SPI_ENABLE_DMATX(p_spi);
                }
                else
                {
                    p_spi->error_code |= HAL_SPI_ERROR_DMA;

                    /* Update SPI state */
                    spi_set_device_state(p_spi, HAL_SPI_STATE_READY);

                }
            }
            else
            {
                /* Update SPI state */
                spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
            }
        }
        else
        {
            /* Update SPI state */
            spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_spi_transmit_with_ia_32addr(spi_handle_t *p_spi, uint8_t inst, uint32_t addr, uint8_t *p_data, uint32_t length, uint32_t timeout)
{
    hal_status_t status   = HAL_OK;
    spi_config_t s_config = SPI_CONFIG_DEFAULT_VAL();

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Update SPI state */
        spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY);

        /* Wait till BUSY flag reset */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, timeout);

        if (HAL_OK == status)
        {
            /* Call the configuration function */
            s_config.data_size          = SPI_DATASIZE_8BIT;
            s_config.transfer_length    = length;
            s_config.transfer_direction = SPI_DIRECTION_SIMPLEX_TX;
            spi_config(p_spi, &s_config);

            /* Configure counters and size of the handle */
            p_spi->tx_xfer_count = length;
            p_spi->tx_xfer_size  = length;
            p_spi->p_tx_buffer   = p_data;

            p_spi->rx_xfer_count = 0U;
            p_spi->rx_xfer_size  = 0U;
            p_spi->p_rx_buffer   = NULL;

            spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY_TX);

            HAL_GLOBAL_EXCEPTION_DISABLE();
            SPI_SOFT_CS_ASSERT(p_spi,   CS_STA_STATE_POLL_TX);
            p_spi->p_instance->DATA = inst;
            p_spi->p_instance->DATA = ((addr >> 24U ) & 0xffU);
            p_spi->p_instance->DATA = ((addr >> 16U ) & 0xffU);
            p_spi->p_instance->DATA = ((addr >>  8U ) & 0xffU);
            p_spi->p_instance->DATA = ((addr >>  0U ) & 0xffU);
            while ((__HAL_SPI_GET_FLAG(p_spi, SPI_FLAG_TFE)) != SET){}
            while ((__HAL_SPI_GET_FLAG(p_spi, SPI_FLAG_BUSY)) != RESET){}
            __HAL_SPI_DISABLE(p_spi);
            ll_spi_set_data_size(p_spi->p_instance, p_spi->init.data_size);
            __HAL_SPI_ENABLE(p_spi);
            status = spi_transmit(p_spi, timeout);
            SPI_SOFT_CS_DEASSERT(p_spi, CS_END_STATE_POLL_TX);
            HAL_GLOBAL_EXCEPTION_ENABLE();
        }

        spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_spi_transmit_dma_with_ia_32addr(spi_handle_t *p_spi, uint8_t inst, uint32_t addr, uint8_t *p_data, uint32_t length)
{
    hal_status_t status   = HAL_OK;
    spi_config_t s_config = SPI_CONFIG_DEFAULT_VAL();
    uint32_t data_size = SPI_DATASIZE_8BIT;

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Update SPI state */
        spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY);

        /* Wait till BUSY flag reset */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, p_spi->timeout);

        if (HAL_OK == status)
        {
            if (DMA_SDATAALIGN_BYTE == p_spi->p_dmatx->init.src_data_alignment)
            {
                p_spi->tx_xfer_count = length;
                data_size   = SPI_DATASIZE_8BIT;
            }
            else if (DMA_SDATAALIGN_HALFWORD == p_spi->p_dmatx->init.src_data_alignment)
            {
                if (0U != (length % 2U))
                {
                    p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_spi->tx_xfer_count = (length >> 1U);
                    data_size  = SPI_DATASIZE_16BIT;
                }
            }
            else if (DMA_SDATAALIGN_WORD == p_spi->p_dmatx->init.src_data_alignment)
            {
                if (0U != (length % 4U))
                {
                    p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_spi->tx_xfer_count = (length >> 2U);
                    data_size  = SPI_DATASIZE_32BIT;
                }
            }
            else
            {
                /* Do nothing */
            }

            if (HAL_OK == status)
            {
                /* Call the configuration function */
                s_config.transfer_length    = length;
                s_config.data_size  = SPI_DATASIZE_8BIT;
                s_config.transfer_direction = SPI_DIRECTION_SIMPLEX_TX;
                spi_config(p_spi, &s_config);

                /* Configure counters and size of the handle */
                p_spi->tx_xfer_size = p_spi->tx_xfer_count;
                p_spi->p_tx_buffer  = p_data;

                /* Update DMA configure to avoid the mis-matched parameters from upper layer */
                spi_update_dma_configure(p_spi, HAL_SPI_STATE_BUSY_TX);

                /* Enable the SPI transmit DMA Channel */
                status = hal_dma_start_it(p_spi->p_dmatx, (uint32_t)p_data, (uint32_t)&p_spi->p_instance->DATA, p_spi->tx_xfer_size);

                if (HAL_OK == status)
                {
                    SPI_SOFT_CS_ASSERT(p_spi,   CS_STA_STATE_DMA_TX);
                    p_spi->p_instance->DATA = inst;
                    p_spi->p_instance->DATA = ((addr >> 24U ) & 0xffU);
                    p_spi->p_instance->DATA = ((addr >> 16U ) & 0xffU);
                    p_spi->p_instance->DATA = ((addr >>  8U ) & 0xffU);
                    p_spi->p_instance->DATA = ((addr >>  0U ) & 0xffU);
                    while ((__HAL_SPI_GET_FLAG(p_spi, SPI_FLAG_TFE)) != SET){}
                    while ((__HAL_SPI_GET_FLAG(p_spi, SPI_FLAG_BUSY)) != RESET){}
                    __HAL_SPI_DISABLE(p_spi);
                    ll_spi_set_data_size(p_spi->p_instance, data_size);
                    __HAL_SPI_ENABLE(p_spi);
                    __HAL_SPI_ENABLE_DMATX(p_spi);
                }
                else
                {
                    p_spi->error_code |= HAL_SPI_ERROR_DMA;

                    /* Update SPI state */
                    spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
                }
            }
            else
            {
                /* Update SPI state */
                spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
            }
        }
        else
        {
            /* Update SPI state */
            spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK void hal_spi_error_callback(spi_handle_t *p_spi)
{
    UNUSED(p_spi);
}

__WEAK void hal_spi_abort_cplt_callback(spi_handle_t *p_spi)
{
    UNUSED(p_spi);
}

__WEAK void hal_spi_rx_cplt_callback(spi_handle_t *p_spi)
{
    UNUSED(p_spi);
}

__WEAK void hal_spi_tx_cplt_callback(spi_handle_t *p_spi)
{
    UNUSED(p_spi);
}

__WEAK void hal_spi_tx_rx_cplt_callback(spi_handle_t *p_spi)
{
    UNUSED(p_spi);
}

__WEAK hal_spi_state_t hal_spi_get_state(spi_handle_t *p_spi)
{
    /* Return SPI handle state */
    return p_spi->state;
}

__WEAK uint32_t hal_spi_get_error(spi_handle_t *p_spi)
{
    return p_spi->error_code;
}

__WEAK void hal_spi_suspend_reg(spi_handle_t *p_spi)
{
    spi_regs_t *p_ssi_regs = p_spi->p_instance;

    p_spi->retention[0] = READ_REG(p_ssi_regs->TX_FIFO_TL);
    p_spi->retention[1] = READ_REG(p_ssi_regs->RX_FIFO_TL);
    p_spi->retention[2] = READ_REG(p_ssi_regs->DMA_TX_DL);
    p_spi->retention[3] = READ_REG(p_ssi_regs->DMA_RX_DL);

    p_spi->retention[4] = READ_REG(p_ssi_regs->BAUD);
    p_spi->retention[5] = READ_REG(p_ssi_regs->S_EN);
    p_spi->retention[6] = READ_REG(p_ssi_regs->INT_MASK);

    p_spi->retention[7] = READ_REG(p_ssi_regs->CTRL0);
    p_spi->retention[8] = READ_REG(p_ssi_regs->RX_SAMPLE_DLY);
}

__WEAK void hal_spi_resume_reg(spi_handle_t *p_spi)
{
    spi_regs_t *p_ssi_regs = p_spi->p_instance;

    WRITE_REG(p_ssi_regs->TX_FIFO_TL, p_spi->retention[0]);
    WRITE_REG(p_ssi_regs->RX_FIFO_TL, p_spi->retention[1]);
    WRITE_REG(p_ssi_regs->DMA_TX_DL, p_spi->retention[2]);
    WRITE_REG(p_ssi_regs->DMA_RX_DL, p_spi->retention[3]);
    CLEAR_BITS(p_ssi_regs->SSI_EN, SPI_SSI_EN);
    if (SPIM == p_ssi_regs)
    {
        WRITE_REG(p_ssi_regs->BAUD, p_spi->retention[4]);
        WRITE_REG(p_ssi_regs->S_EN, p_spi->retention[5]);
    }
    WRITE_REG(p_ssi_regs->CTRL0, p_spi->retention[7]);
    WRITE_REG(p_ssi_regs->RX_SAMPLE_DLY, p_spi->retention[8]);
    WRITE_REG(p_ssi_regs->INT_MASK, p_spi->retention[6]);
    SET_BITS(p_ssi_regs->SSI_EN, SPI_SSI_EN);
}

#ifdef HAL_PM_ENABLE
__WEAK hal_pm_status_t hal_pm_spi_suspend(spi_handle_t *p_spi)
{
    hal_spi_state_t state;
    state = hal_spi_get_state(p_spi);
    if ((state != HAL_SPI_STATE_READY) && (state != HAL_SPI_STATE_RESET))
    {
        return HAL_PM_ACTIVE;
    }
    else
    {
        hal_spi_suspend_reg(p_spi);
        return HAL_PM_SLEEP;
    }
}

__WEAK void hal_pm_spi_resume(spi_handle_t *p_spi)
{
    hal_spi_resume_reg(p_spi);
}
#endif /* HAL_PM_ENABLE */

__WEAK hal_status_t hal_spi_abort(spi_handle_t *p_spi)
{
    hal_status_t status    = HAL_OK;
    uint32_t     tmp_dmac;

    /* Check if the state is in one of the busy states */
    if (((uint32_t)(p_spi->state) & 0x2U))
    {
        if (0U != p_spi->p_instance->DMA_CTRL)
        {
            tmp_dmac = p_spi->p_instance->DMA_CTRL;

            /* Abort DMA channel */
            if (tmp_dmac & SPI_DMA_CTRL_TX_DMA_EN)
            {
                status = hal_dma_abort(p_spi->p_dmatx);
            }
            if (tmp_dmac & SPI_DMA_CTRL_RX_DMA_EN)
            {
                status = hal_dma_abort(p_spi->p_dmarx);
            }
            if (HAL_OK != status)
            {
                p_spi->error_code |= HAL_SPI_ERROR_DMA;
            }

            /* Disable the DMA transfer by clearing the DMAEN bit in the SSI DMAC register */
            CLEAR_REG(p_spi->p_instance->DMA_CTRL);

            /* Flush FIFO */
            __HAL_SPI_DISABLE(p_spi);
            __HAL_SPI_ENABLE(p_spi);
        }
        else
        {
            /* Disable all interrupts */
            CLEAR_REG(p_spi->p_instance->INT_MASK);

            /* Flush FIFO */
            __HAL_SPI_DISABLE(p_spi);
            __HAL_SPI_ENABLE(p_spi);
        }

        /* Update state */
        spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
    }

    return status;
}

__WEAK hal_status_t hal_spi_abort_it(spi_handle_t *p_spi)
{
    hal_status_t status = HAL_OK;
    uint32_t     tmp_dmac;

    /* Check if the state is in one of the busy states */
    if (((uint32_t)(p_spi->state) & 0x2U))
    {
        /* Disable all interrupts */
        CLEAR_REG(p_spi->p_instance->INT_MASK);

        if (0U != p_spi->p_instance->DMA_CTRL)
        {
            tmp_dmac = p_spi->p_instance->DMA_CTRL;
            /* Disable the DMA transfer by clearing the DMAEN bit in the SSI DMAC register */
            CLEAR_REG(p_spi->p_instance->DMA_CTRL);

            /* Update SPI state */
            spi_set_device_state(p_spi, HAL_SPI_STATE_ABORT);
            /* Abort DMA channel */
            if (tmp_dmac & SPI_DMA_CTRL_TX_DMA_EN)
            {
                p_spi->p_dmatx->xfer_abort_callback = spi_dma_abort_cplt;
                status = hal_dma_abort_it(p_spi->p_dmatx);
            }
            if (tmp_dmac & SPI_DMA_CTRL_RX_DMA_EN)
            {
                p_spi->p_dmarx->xfer_abort_callback = spi_dma_abort_cplt;
                status = hal_dma_abort_it(p_spi->p_dmarx);
            }
        }
        else if (((uint32_t)(p_spi->state) & 0x10U))
        {
            /* Update SPI state */
            spi_set_device_state(p_spi, HAL_SPI_STATE_ABORT);
            p_spi->tx_xfer_count = 0U;
            __HAL_SPI_ENABLE_IT(p_spi, SPI_IT_TXE);
        }
        else if (HAL_SPI_STATE_BUSY_RX == p_spi->state)
        {
            /* Update SPI state */
            spi_set_device_state(p_spi, HAL_SPI_STATE_ABORT);
            p_spi->rx_xfer_count = 0U;
            __HAL_SPI_ENABLE_IT(p_spi, SPI_IT_RXF);
        }
        else
        {
            /* DO nothing */
        }
    }

    return status;
}

__WEAK void hal_spi_set_timeout(spi_handle_t *p_spi, uint32_t timeout)
{
    p_spi->timeout = timeout;
}

__WEAK hal_status_t hal_spi_set_tx_fifo_threshold(spi_handle_t *p_spi, uint32_t threshold)
{
    hal_status_t status = HAL_OK;

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        /* Configure SPI FIFO Threshold */
        ll_spi_set_tx_fifo_threshold(p_spi->p_instance, threshold);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_spi_set_rx_fifo_threshold(spi_handle_t *p_spi, uint32_t threshold)
{
    hal_status_t status = HAL_OK;

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        /* Configure SPI FIFO Threshold */
        ll_spi_set_rx_fifo_threshold(p_spi->p_instance, threshold);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK uint32_t hal_spi_get_tx_fifo_threshold(spi_handle_t *p_spi)
{
    return ll_spi_get_tx_fifo_threshold(p_spi->p_instance);
}

__WEAK uint32_t hal_spi_get_rx_fifo_threshold(spi_handle_t *p_spi)
{
    return ll_spi_get_rx_fifo_threshold(p_spi->p_instance);
}

static void spi_dma_rx_cplt(dma_handle_t *p_dma)
{
    //lint -e9087 Cast a pointer to a different object type is necessary
    spi_handle_t* p_spi = ( spi_handle_t* )p_dma->p_parent;
    p_spi->rx_xfer_count = 0U;
    __HAL_SPI_DISABLE_DMARX(p_spi);

    if (HAL_SPI_STATE_BUSY_RX == p_spi->state)
    {
        /* Change state of SPI */
        spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
        SPI_SOFT_CS_DEASSERT(p_spi, CS_END_STATE_RX_CPLT);
        hal_spi_rx_cplt_callback(p_spi);
    }
    if (HAL_SPI_STATE_BUSY_TX_RX == p_spi->state)
    {
        /* Change state of SPI */
        spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
        SPI_SOFT_CS_DEASSERT(p_spi, CS_END_STATE_TX_RX_CPLT);
        hal_spi_tx_rx_cplt_callback(p_spi);
    }
}

static void spi_dma_tx_cplt(dma_handle_t *p_dma)
{
    //lint -e9087 Cast a pointer to a different object type is necessary
    spi_handle_t* p_spi = ( spi_handle_t* )p_dma->p_parent;
    p_spi->tx_xfer_count = 0U;
    __HAL_SPI_DISABLE_DMATX(p_spi);

    __HAL_SPI_ENABLE_IT(p_spi, SPI_IT_TXE);
}

static void spi_dma_error(dma_handle_t *p_dma)
{
    spi_handle_t* p_spi = ( spi_handle_t* )p_dma->p_parent;

    p_spi->error_code  |= HAL_SPI_ERROR_DMA;

    /* Abort the SPI */
    hal_spi_abort_it(p_spi); /*lint !e534. Ignoring return value of function */
}

static void spi_dma_abort_cplt(dma_handle_t *p_dma)
{
    //lint -e9087 Cast a pointer to a different object type is necessary
    spi_handle_t* p_spi = ( spi_handle_t* )p_dma->p_parent;

    p_spi->rx_xfer_count = 0U;
    p_spi->tx_xfer_count = 0U;

    __HAL_SPI_DISABLE(p_spi);
    __HAL_SPI_ENABLE(p_spi);

    if (HAL_SPI_STATE_ABORT == p_spi->state)
    {
        /* Change state of SPI */
        spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
        SPI_SOFT_CS_DEASSERT(p_spi, CS_END_STATE_ABORT_CPLT);
        hal_spi_abort_cplt_callback(p_spi);
    }
    else
    {
        /* DMA Abort called due to a transfer error interrupt */
        /* Change state of SPI */
        spi_set_device_state(p_spi, HAL_SPI_STATE_READY);
        /* Error callback */
        SPI_SOFT_CS_DEASSERT(p_spi, CS_END_STATE_XFER_ERR);
        hal_spi_error_callback(p_spi);
    }
}

static hal_status_t spi_wait_flag_state_until_timeout(spi_handle_t *p_spi,
                                                      uint32_t      flag,
                                                      flag_status_t state,
                                                      uint32_t      timeout)
{
    hal_status_t status = HAL_OK;
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
    while ((__HAL_SPI_GET_FLAG(p_spi, flag)) != state)
    {
        if(HAL_NEVER_TIMEOUT != timeout)
        {
            if((HAL_TIMEOUT_GET_TICK() - tickstart) > timeout_counter)
            {
                spi_set_device_state(p_spi, HAL_SPI_STATE_ERROR);
                p_spi->error_code |= HAL_SPI_ERROR_TIMEOUT;
                status = HAL_TIMEOUT;
                goto EXIT;
            }
        }
    }
EXIT:
    HAL_TIMEOUT_DEINIT();
    return status;
}

static void spi_config(spi_handle_t *p_spi, spi_config_t * p_config)
{
    gr_assert_param(IS_SPI_DIRECTION(p_config->transfer_direction));

    __HAL_SPI_DISABLE(p_spi);

    ll_spi_set_data_size(p_spi->p_instance, p_config->data_size);

    ll_spi_set_transfer_direction(p_spi->p_instance, p_config->transfer_direction);
    if ((SPI_DIRECTION_SIMPLEX_RX  == p_config->transfer_direction) ||
        (SPI_DIRECTION_READ_EEPROM == p_config->transfer_direction))
    {
        if (SPI_DATASIZE_8BIT >= p_config->data_size)
        {
            ll_spi_set_receive_size(p_spi->p_instance, p_config->transfer_length - 1U);
        }
        else if (SPI_DATASIZE_16BIT >= p_config->data_size)
        {
            ll_spi_set_receive_size(p_spi->p_instance, (p_config->transfer_length >> 1U) - 1U);
        }
        else
        {
            ll_spi_set_receive_size(p_spi->p_instance, (p_config->transfer_length >> 2U) - 1U);
        }
    }
    ll_spi_enable_ss(p_spi->p_instance, p_spi->init.slave_select);
    __HAL_SPI_ENABLE(p_spi);
}

static void spi_send_8bit(spi_handle_t *p_spi)
{
    while (__HAL_SPI_GET_FLAG(p_spi, SPI_FLAG_TFNF))
    {
        if(0U == p_spi->tx_xfer_count)
        {
            break;
        }
        p_spi->p_instance->DATA = *p_spi->p_tx_buffer++;
        p_spi->tx_xfer_count--;
    }
}

static void spi_send_16bit(spi_handle_t *p_spi)
{
    while (__HAL_SPI_GET_FLAG(p_spi, SPI_FLAG_TFNF))
    {
        if(0U == p_spi->tx_xfer_count)
        {
            break;
        }
        p_spi->p_instance->DATA = *(uint16_t *)p_spi->p_tx_buffer;
        p_spi->p_tx_buffer += 2U;
        p_spi->tx_xfer_count -= 2U;
    }
}

static void spi_send_32bit(spi_handle_t *p_spi)
{
    while (__HAL_SPI_GET_FLAG(p_spi, SPI_FLAG_TFNF))
    {
        if(0U == p_spi->tx_xfer_count)
        {
            break;
        }
        p_spi->p_instance->DATA = *(uint32_t *)p_spi->p_tx_buffer;
        p_spi->p_tx_buffer += 4U;
        p_spi->tx_xfer_count -= 4U;
    }
}

static void spi_receive_8bit(spi_handle_t *p_spi)
{
    while (__HAL_SPI_GET_FLAG(p_spi, SPI_FLAG_RFNE))
    {
        if(0U == p_spi->rx_xfer_count)
        {
            break;
        }
        *(p_spi->p_rx_buffer++) =(uint8_t)p_spi->p_instance->DATA;
        p_spi->rx_xfer_count--;
    }
}

static void spi_receive_16bit(spi_handle_t *p_spi)
{
    while (__HAL_SPI_GET_FLAG(p_spi, SPI_FLAG_RFNE))
    {
        if(0U == p_spi->rx_xfer_count)
        {
            break;
        }
        *(uint16_t *)p_spi->p_rx_buffer = (uint16_t)(p_spi->p_instance->DATA);
        p_spi->p_rx_buffer += 2U;
        p_spi->rx_xfer_count -= 2U;
    }
}

static void spi_receive_32bit(spi_handle_t *p_spi)
{
    while (__HAL_SPI_GET_FLAG(p_spi, SPI_FLAG_RFNE))
    {
        if(0U == p_spi->rx_xfer_count)
        {
            break;
        }
        *(uint32_t *)p_spi->p_rx_buffer = p_spi->p_instance->DATA;
        p_spi->p_rx_buffer += 4U;
        p_spi->rx_xfer_count -= 4U;
    }
}

static void spi_send_receive_8bit(spi_handle_t *p_spi)
{
    while (__HAL_SPI_GET_FLAG(p_spi, SPI_FLAG_TFNF))
    {
        if(0U == p_spi->tx_xfer_count)
        {
            break;
        }
        uint32_t rx_xfer_count = p_spi->rx_xfer_count;
        uint32_t tx_xfer_count = p_spi->tx_xfer_count;
        if((rx_xfer_count - tx_xfer_count) >= 8U)
        {
            break;
        }
        p_spi->p_instance->DATA = *(p_spi->p_tx_buffer++);
        p_spi->tx_xfer_count--;
    }
    while (__HAL_SPI_GET_FLAG(p_spi, SPI_FLAG_RFNE))
    {
        if(0U == p_spi->rx_xfer_count)
        {
            break;
        }
        *p_spi->p_rx_buffer++ = (uint8_t)p_spi->p_instance->DATA;
        p_spi->rx_xfer_count--;
    }
}

static void spi_send_receive_16bit(spi_handle_t *p_spi)
{
    while (__HAL_SPI_GET_FLAG(p_spi, SPI_FLAG_TFNF))
    {
        if(0U == p_spi->tx_xfer_count)
        {
            break;
        }
        uint32_t rx_xfer_count = p_spi->rx_xfer_count;
        uint32_t tx_xfer_count = p_spi->tx_xfer_count;
        if((rx_xfer_count - tx_xfer_count) >= 16U)
        {
            break;
        }
        p_spi->p_instance->DATA = *(uint16_t *)p_spi->p_tx_buffer;
        p_spi->p_tx_buffer += 2U;
        p_spi->tx_xfer_count -= 2U;
    }
    while (__HAL_SPI_GET_FLAG(p_spi, SPI_FLAG_RFNE))
    {
        if(0U == p_spi->rx_xfer_count)
        {
            break;
        }
        *(uint16_t *)p_spi->p_rx_buffer = (uint16_t)(p_spi->p_instance->DATA);
        p_spi->p_rx_buffer += 2U;
        p_spi->rx_xfer_count -= 2U;
    }
}

static void spi_send_receive_32bit(spi_handle_t *p_spi)
{
    while (__HAL_SPI_GET_FLAG(p_spi, SPI_FLAG_TFNF))
    {
        if(0U == p_spi->tx_xfer_count)
        {
            break;
        }
        uint32_t rx_xfer_count = p_spi->rx_xfer_count;
        uint32_t tx_xfer_count = p_spi->tx_xfer_count;
        if((rx_xfer_count - tx_xfer_count) >= 32U)
        {
            break;
        }
        p_spi->p_instance->DATA = *(uint32_t *)p_spi->p_tx_buffer;
        p_spi->p_tx_buffer += 4U;
        p_spi->tx_xfer_count -= 4U;
    }
    while (__HAL_SPI_GET_FLAG(p_spi, SPI_FLAG_RFNE))
    {
        if(0U == p_spi->rx_xfer_count)
        {
            break;
        }
        *(uint32_t *)p_spi->p_rx_buffer = p_spi->p_instance->DATA;
        p_spi->p_rx_buffer += 4U;
        p_spi->rx_xfer_count -= 4U;
    }
}

static hal_status_t spi_transmit(spi_handle_t *p_spi, uint32_t timeout)
{
    if((HAL_NEVER_TIMEOUT != timeout) && (HAL_MAX_DELAY < timeout))
    {
        return HAL_ERROR;
    }

    hal_status_t status    = HAL_OK;

    HAL_TIMEOUT_INIT();
    uint32_t tickstart = HAL_TIMEOUT_GET_TICK();
    /* Update SystemCoreClock */
    SystemCoreUpdateClock();
    uint32_t timeout_counter = ((SystemCoreClock / 1000U) * timeout);

    while (0U < p_spi->tx_xfer_count)
    {
        p_spi->write_fifo(p_spi);
        if(HAL_NEVER_TIMEOUT != timeout)
        {
            if((HAL_TIMEOUT_GET_TICK() - tickstart) > timeout_counter)
            {
                status = HAL_TIMEOUT;
                break;
            }
        }
    }

    HAL_TIMEOUT_DEINIT();

    if (HAL_OK == status)
    {
        /* Wait until TFE flag is set then to check BUSY flag */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_TFE, SET, timeout);
        /* Wait until BUSY flag is reset to go back in idle state */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, timeout);
    }

    return status;
}

static hal_status_t spi_receive(spi_handle_t *p_spi, uint32_t timeout)
{
    if((HAL_NEVER_TIMEOUT != timeout) && (HAL_MAX_DELAY < timeout))
    {
        return HAL_ERROR;
    }

    hal_status_t status    = HAL_OK;

    HAL_TIMEOUT_INIT();
    uint32_t tickstart = HAL_TIMEOUT_GET_TICK();
    /* Update SystemCoreClock */
    SystemCoreUpdateClock();
    uint32_t timeout_counter = (SystemCoreClock / 1000U * timeout);

    while (0U < p_spi->rx_xfer_count)
    {
        p_spi->read_fifo(p_spi);
        if(HAL_NEVER_TIMEOUT != timeout)
        {
            if((HAL_TIMEOUT_GET_TICK() - tickstart) > timeout_counter)
            {
                status = HAL_TIMEOUT;
                break;
            }
        }
    }

    HAL_TIMEOUT_DEINIT();

    if (HAL_OK == status)
    {
        /* Wait until BUSY flag is reset to go back in idle state */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, timeout);
    }

    return status;
}

static hal_status_t spi_transmit_receive(spi_handle_t *p_spi, uint32_t timeout)
{
    if((HAL_NEVER_TIMEOUT != timeout) && (HAL_MAX_DELAY < timeout))
    {
        return HAL_ERROR;
    }

    hal_status_t status    = HAL_OK;

    HAL_TIMEOUT_INIT();
    uint32_t tickstart = HAL_TIMEOUT_GET_TICK();
    /* Update SystemCoreClock */
    SystemCoreUpdateClock();
    uint32_t timeout_counter = ((SystemCoreClock / 1000U) * timeout);

    while (1)
    {
        uint32_t rx_xfer_count = p_spi->rx_xfer_count;
        uint32_t tx_xfer_count = p_spi->tx_xfer_count;

        if((0U == rx_xfer_count) && (0U == tx_xfer_count))
        {
            break;
        }

        p_spi->read_write_fifo(p_spi);

        if(HAL_NEVER_TIMEOUT != timeout)
        {
            if((HAL_TIMEOUT_GET_TICK() - tickstart) > timeout_counter)
            {
                status = HAL_TIMEOUT;
                break;
            }
        }
    }

    HAL_TIMEOUT_DEINIT();

    if (HAL_OK == status)
    {
        /* Wait until BUSY flag is reset to go back in idle state */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, timeout);
    }

    return status;
}

static hal_status_t spi_read_eeprom(spi_handle_t *p_spi, uint32_t timeout)
{
    if((HAL_NEVER_TIMEOUT != timeout) && (HAL_MAX_DELAY < timeout))
    {
        return HAL_ERROR;
    }

    hal_status_t status    = HAL_OK;

    HAL_TIMEOUT_INIT();
    uint32_t tickstart = HAL_TIMEOUT_GET_TICK();
    /* Update SystemCoreClock */
    SystemCoreUpdateClock();
    uint32_t timeout_counter = ((SystemCoreClock / 1000U) * timeout);

    while (0U < p_spi->tx_xfer_count)
    {
        p_spi->write_fifo(p_spi);

        if(HAL_NEVER_TIMEOUT != timeout)
        {
            if((HAL_TIMEOUT_GET_TICK() - tickstart) > timeout_counter)
            {
                status = HAL_TIMEOUT;
                goto EXIT;
            }
        }

        p_spi->rx_xfer_size  = p_spi->rx_xfer_count;
        while (0U < p_spi->rx_xfer_count)
        {
            p_spi->read_fifo(p_spi);

            if(HAL_NEVER_TIMEOUT != timeout)
            {
                if((HAL_TIMEOUT_GET_TICK() - tickstart) > timeout_counter)
                {
                    status = HAL_TIMEOUT;
                    goto EXIT;
                }
            }
        }
    }

EXIT:
    if (HAL_OK == status)
    {
        /* Wait until BUSY flag is reset to go back in idle state */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, timeout);
    }

    HAL_TIMEOUT_DEINIT();
    return status;
}


/**
 ****************************************************************************************
 * @brief  Update DMA configure to avoid the mis-matched parameters from upper layer
 * @param[in]   p_spi: Pointer to a spi instance
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 ****************************************************************************************
 */
static void spi_update_dma_configure(spi_handle_t *p_spi, hal_spi_state_t xfer_state)
{
    switch(xfer_state)
    {
        case HAL_SPI_STATE_BUSY_TX_RX:
        {
            /* Update SPI state */
            spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY_TX_RX);

            /* Set the SPI DMA transfer complete callback */
            p_spi->p_dmatx->xfer_tfr_callback = spi_dma_tx_cplt;

            /* Set the DMA error callback */
            p_spi->p_dmatx->xfer_error_callback = spi_dma_error;

            /* Clear the DMA abort callback */
            p_spi->p_dmatx->xfer_abort_callback = NULL;

            /* Configure the direction of the DMA */
            p_spi->p_dmatx->init.direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;

            /* Configure the source address increment */
            ll_dma_set_source_increment_mode(p_spi->p_dmatx->p_instance, p_spi->p_dmatx->channel, LL_DMA_SRC_INCREMENT);

            /* Configure the destination address increment */
            ll_dma_set_destination_increment_mode(p_spi->p_dmatx->p_instance, p_spi->p_dmatx->channel, LL_DMA_DST_NO_CHANGE);


            /* Set the SPI DMA transfer complete callback */
            p_spi->p_dmarx->xfer_tfr_callback = spi_dma_rx_cplt;

            /* Set the DMA error callback */
            p_spi->p_dmarx->xfer_error_callback = spi_dma_error;

            /* Clear the DMA abort callback */
            p_spi->p_dmarx->xfer_abort_callback = NULL;

            /* Configure the direction of the DMA */
            p_spi->p_dmarx->init.direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;

            /* Configure the source address increment */
            ll_dma_set_source_increment_mode(p_spi->p_dmarx->p_instance, p_spi->p_dmarx->channel, LL_DMA_SRC_NO_CHANGE);

            /* Configure the destination address increment */
            ll_dma_set_destination_increment_mode(p_spi->p_dmarx->p_instance, p_spi->p_dmarx->channel, LL_DMA_DST_INCREMENT);


            /* Configure the dma src & dst peripheral */
            if(DMA0 == p_spi->p_dmatx->p_instance) {
                if (SPIM == p_spi->p_instance)
                {
                    ll_dma_set_destination_peripheral(p_spi->p_dmatx->p_instance, p_spi->p_dmatx->channel, LL_DMA0_PERIPH_SPIM_TX);
                }
                else
                {
                    ll_dma_set_destination_peripheral(p_spi->p_dmatx->p_instance, p_spi->p_dmatx->channel, LL_DMA0_PERIPH_SPIS_TX);
                }
            }

            if(DMA0 == p_spi->p_dmarx->p_instance) {
                if (SPIM == p_spi->p_instance)
                {
                    ll_dma_set_source_peripheral(p_spi->p_dmarx->p_instance, p_spi->p_dmarx->channel , LL_DMA0_PERIPH_SPIM_RX);
                }
                else
                {
                    ll_dma_set_source_peripheral(p_spi->p_dmarx->p_instance, p_spi->p_dmarx->channel , LL_DMA0_PERIPH_SPIS_RX);
                }
            }
        }
        break;

        case HAL_SPI_STATE_BUSY_TX:
        {
            /* Update SPI state */
            spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY_TX);

            /* Set the SPI DMA transfer complete callback */
            p_spi->p_dmatx->xfer_tfr_callback = spi_dma_tx_cplt;

            /* Set the DMA error callback */
            p_spi->p_dmatx->xfer_error_callback = spi_dma_error;

            /* Clear the DMA abort callback */
            p_spi->p_dmatx->xfer_abort_callback = NULL;


            /* Configure the direction of the DMA */
            p_spi->p_dmatx->init.direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;

            /* Configure the source address increment */
            ll_dma_set_source_increment_mode(p_spi->p_dmatx->p_instance, p_spi->p_dmatx->channel, LL_DMA_SRC_INCREMENT);

            /* Configure the destination address increment */
            ll_dma_set_destination_increment_mode(p_spi->p_dmatx->p_instance, p_spi->p_dmatx->channel, LL_DMA_DST_NO_CHANGE);

            /* Configure the dma dst peripheral */
            if(DMA0 == p_spi->p_dmatx->p_instance) {
                if (SPIM == p_spi->p_instance)
                {
                    ll_dma_set_destination_peripheral(p_spi->p_dmatx->p_instance, p_spi->p_dmatx->channel, LL_DMA0_PERIPH_SPIM_TX);
                }
                else
                {
                    ll_dma_set_destination_peripheral(p_spi->p_dmatx->p_instance, p_spi->p_dmatx->channel, LL_DMA0_PERIPH_SPIS_TX);
                }
            }
        }
        break;

        case HAL_SPI_STATE_BUSY_RX:
        {
            /* Update SPI state */
            spi_set_device_state(p_spi, HAL_SPI_STATE_BUSY_RX);

            /* Set the SPI DMA transfer complete callback */
            p_spi->p_dmarx->xfer_tfr_callback = spi_dma_rx_cplt;

            /* Set the DMA error callback */
            p_spi->p_dmarx->xfer_error_callback = spi_dma_error;

            /* Clear the DMA abort callback */
            p_spi->p_dmarx->xfer_abort_callback = NULL;

            /* Configure the direction of the DMA */
            p_spi->p_dmarx->init.direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;

            /* Configure the source address increment */
            ll_dma_set_source_increment_mode(p_spi->p_dmarx->p_instance, p_spi->p_dmarx->channel, LL_DMA_SRC_NO_CHANGE);

            /* Configure the destination address increment */
            ll_dma_set_destination_increment_mode(p_spi->p_dmarx->p_instance, p_spi->p_dmarx->channel, LL_DMA_DST_INCREMENT);

            /* Configure the dma src peripheral */
            if(DMA0 == p_spi->p_dmarx->p_instance) {
                if (SPIM == p_spi->p_instance)
                {
                    ll_dma_set_source_peripheral(p_spi->p_dmarx->p_instance, p_spi->p_dmarx->channel , LL_DMA0_PERIPH_SPIM_RX);
                }
                else
                {
                    ll_dma_set_source_peripheral(p_spi->p_dmarx->p_instance, p_spi->p_dmarx->channel , LL_DMA0_PERIPH_SPIS_RX);
                }
            }
        }
        break;

        default:
            /* Nothing to do */
            break;
    }
}

/**
 ****************************************************************************************
 * @brief  Set the spi bit of g_devices_state.
 ****************************************************************************************
 */
static void spi_set_device_state(spi_handle_t *p_spi, hal_spi_state_t state)
{
    p_spi->state = state;
}

#endif /* HAL_SPI_MODULE_ENABLED */

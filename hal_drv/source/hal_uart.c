/**
  ****************************************************************************************
  * @file    hal_uart.c
  * @author  BLE Driver Team
  * @brief   UART HAL module driver.
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

#ifdef HAL_UART_MODULE_ENABLED
#include "hal_uart.h"

#ifdef HAL_CLOCK_MODULE
#include "hal_cgc.h"
#endif
#ifdef HAL_CLOCK_UNIFORM_CONTROL
#include "hal_clock.h"
#endif
/** @addtogroup HAL_DRIVER
  * @{
  */



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup UART_Private_Constants UART Private Constants
  * @{
  */

/** @} */

/* Private variables ---------------------------------------------------------*/
/** @defgroup UART_TX_RX_Flags
  * @{
  */
typedef enum
{
    TX = 0x00u,
    RX,
    TX_RX
}hal_uart_flag_t;
/** @} */

/* Private macros ------------------------------------------------------------*/
/** @defgroup UART_Flags     UART Status Flags
  *        Elements values convention: 0xXXXX
  *           - 0xXXXX  : Flag mask in the USR register
  * @{
  */
#define UART_FLAG_LINE_TEMT                 LL_UART_LSR_TEMT    /*!< Transmitter empty flag */

#define UART_FLAG_FIFO_RFF                  LL_UART_USR_RFF     /*!< Rx FIFO Full flag */
#define UART_FLAG_FIFO_RFNE                 LL_UART_USR_RFNE    /*!< Rx FIFO Not Empty flag */
#define UART_FLAG_FIFO_TFE                  LL_UART_USR_TFE     /*!< Tx FIFO Empty flag */
#define UART_FLAG_FIFO_TFNF                 LL_UART_USR_TFNF    /*!< Tx FIFO Not Full flag */
/** @} */

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @addtogroup UART_Private_Functions
  * @{
  */
__STATIC_INLINE void uart_end_tx_transfer(uart_handle_t *p_uart);
__STATIC_INLINE void uart_end_rx_transfer(uart_handle_t *p_uart);
__STATIC_INLINE void uart_stop_dma_tx(uart_handle_t *p_uart);
__STATIC_INLINE void uart_stop_dma_rx(uart_handle_t *p_uart);

static void uart_dma_transmit_cplt(dma_handle_t *p_dma);
static void uart_dma_receive_cplt(dma_handle_t *p_dma);

static void uart_dma_error(dma_handle_t *p_dma);
static void uart_dma_abort_on_error(dma_handle_t *p_dma);

static void uart_dma_tx_abort_callback(dma_handle_t *p_dma);
static void uart_dma_rx_abort_callback(dma_handle_t *p_dma);
static void uart_dma_tx_only_abort_callback(dma_handle_t *p_dma);
static void uart_dma_rx_only_abort_callback(dma_handle_t *p_dma);

static void uart_transmit_it(uart_handle_t *p_uart);
static void uart_receive_it(uart_handle_t *p_uart, flag_status_t cto_status);

static hal_status_t uart_wait_line_flag_until_timeout(uart_handle_t *p_uart, uint32_t flag, flag_status_t status, uint32_t timeout);
static hal_status_t uart_wait_fifo_flag_until_timeout(uart_handle_t *p_uart, uint32_t flag, flag_status_t status, uint32_t timeout);

static void uart_set_device_state(uart_handle_t *p_uart, hal_uart_flag_t tx_rx_flag, hal_uart_state_t state);



/* __BAUDRATE__ The maximum Baud Rate is derived from the maximum clock available
 *              divided by the smallest oversampling used on the UART (i.e. 8)    */
#define IS_LL_UART_BAUDRATE(__BAUDRATE__) ((__BAUDRATE__) <= 9000000U)

#define IS_LL_UART_PARITY(__VALUE__) (((__VALUE__) == LL_UART_PARITY_NONE) \
                                    || ((__VALUE__) == LL_UART_PARITY_EVEN) \
                                    || ((__VALUE__) == LL_UART_PARITY_ODD)) \
                                    || ((__VALUE__) == LL_UART_PARITY_SP0)) \
                                    || ((__VALUE__) == LL_UART_PARITY_SP1))

#define IS_LL_UART_DATABITS(__VALUE__) (((__VALUE__) == LL_UART_DATABITS_5B) \
                                       || ((__VALUE__) == LL_UART_DATABITS_6B) \
                                       || ((__VALUE__) == LL_UART_DATABITS_7B)) \
                                       || ((__VALUE__) == LL_UART_DATABITS_8B))

#define IS_LL_UART_STOPBITS(__VALUE__) (((__VALUE__) == LL_UART_STOPBITS_1) \
                                      || ((__VALUE__) == LL_UART_STOPBITS_1_5) \
                                      || ((__VALUE__) == LL_UART_STOPBITS_2))

#define IS_LL_UART_HWCONTROL(__VALUE__) (((__VALUE__) == LL_UART_HWCONTROL_NONE) \
                                       || ((__VALUE__) == LL_UART_HWCONTROL_RTS_CTS))



/** @} */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup UART_LL_Exported_Functions
  * @{
  */

/** @addtogroup UART_LL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize UART registers (Registers restored to their default values).
  * @param  UARTx UART instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: UART registers are de-initialized
  *          - ERROR: UART registers are not de-initialized
  */
void ll_uart_deinit(uart_regs_t *UARTx)
{
    /* Check the parameters */
    gr_assert_param(IS_UART_ALL_INSTANCE(UARTx));

    ll_uart_reset(UARTx);

    return ;
}

/**
  * @brief  Initialize UART registers according to the specified
  *         parameters in UART_InitStruct.
  * @note   As some bits in UART configuration registers can only be written when the UART is disabled (UART_CR1_UE bit =0),
  *         UART IP should be in disabled state prior calling this function. Otherwise, ERROR result will be returned.
  * @note   Baud rate value stored in p_uart_init BaudRate field, should be valid (different from 0).
  * @param  UARTx UART instance
  * @param  p_uart_init pointer to a ll_uart_init_t structure
  *         that contains the configuration information for the specified UART peripheral.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: UART registers are initialized according to p_uart_init content
  *          - ERROR: Problem occurred during UART Registers initialization
  */
void ll_uart_init(uart_regs_t *UARTx, const ll_uart_init_t *p_uart_init)
{
    /* Check the parameters */
    gr_assert_param(IS_UART_ALL_INSTANCE(UARTx));
    gr_assert_param(IS_LL_UART_BAUDRATE(p_uart_init->baud_rate));
    gr_assert_param(IS_LL_UART_DATABITS(p_uart_init->data_bits));
    gr_assert_param(IS_LL_UART_STOPBITS(p_uart_init->stop_bits));
    gr_assert_param(IS_LL_UART_PARITY(p_uart_init->parity));
    gr_assert_param(IS_LL_UART_HWCONTROL(p_uart_init->hw_flow_ctrl));

    ll_uart_config_character(UARTx, p_uart_init->data_bits, p_uart_init->parity, p_uart_init->stop_bits);

    extern uint32_t GetSerialClock(void);
    ll_uart_set_baud_rate(UARTx, GetSerialClock(), p_uart_init->baud_rate);

    ll_uart_set_hw_flow_ctrl(UARTx, p_uart_init->hw_flow_ctrl);

    /* Flush FIFO */
    ll_uart_flush_tx_fifo(UARTx);
    ll_uart_flush_rx_fifo(UARTx);

    /* Enable FIFO and set threshold */
    ll_uart_enable_fifo(UARTx);
    ll_uart_set_tx_fifo_threshold(UARTx, LL_UART_TX_FIFO_TH_CHAR_2);
    ll_uart_set_rx_fifo_threshold(UARTx, LL_UART_RX_FIFO_TH_CHAR_1);

    return;
}

/**
  * @brief Set each @ref ll_uart_init_t field to default value.
  * @param p_uart_init pointer to a @ref ll_uart_init_t structure
  *                          whose fields will be set to default values.
  * @retval None
  */

void ll_uart_struct_init(ll_uart_init_t *p_uart_init)
{
    /* Set UART_InitStruct fields to default values */
    p_uart_init->baud_rate    = 9600U;
    p_uart_init->data_bits    = LL_UART_DATABITS_8B;
    p_uart_init->stop_bits    = LL_UART_STOPBITS_1;
    p_uart_init->parity       = LL_UART_PARITY_NONE ;
    p_uart_init->hw_flow_ctrl = LL_UART_HWCONTROL_NONE;
}

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/

/** @defgroup UART_Exported_Functions UART Exported Functions
  * @{
  */

/** @defgroup UART_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions
  * @{
  */

__WEAK hal_status_t hal_uart_init(uart_handle_t *p_uart)
{
    ll_uart_init_t InitStruct;

#if defined(HAL_PARAMS_CHECK)
    /* Check the parameters */
    gr_assert_param(IS_UART_ALL_INSTANCE(p_uart->p_instance));
#endif

#if defined(HAL_STATUS_CHECK)
    if (HAL_UART_STATE_RESET == p_uart->tx_state)
    {
#endif
        if ( HAL_UART_STATE_RESET == p_uart->rx_state )
        {
    #ifdef HAL_CLOCK_UNIFORM_CONTROL
           //lint -e923 Cast from pointer to unsigned int is necessary and safe
           hal_clock_enable_module((uint32_t)p_uart->p_instance);
    #endif

    #ifdef HAL_CLOCK_MODULE
            /* Enable Clock for Serial blocks and Automatic turn off Serial blocks clock during WFI. */
            ll_cgc_disable_force_off_serial_hclk();
            ll_cgc_disable_wfi_off_serial_hclk();

            /* Enable UARTx Clock */
            //lint -e923 Cast from pointer to unsigned int is necessary
            if(p_uart->p_instance == UART0)
            {
                ll_cgc_disable_force_off_uart0_hclk();
                ll_cgc_disable_uart0_slp_wfi();
            }
            //lint -e923 Cast from pointer to unsigned int is necessary
            else if(p_uart->p_instance == UART1)
            {
                ll_cgc_disable_force_off_uart1_hclk();
                ll_cgc_disable_uart1_slp_wfi();
            }
            else
            {
                return HAL_ERROR;
            }
    #endif
            /* init the low level hardware : GPIO, CLOCK */
            hal_uart_msp_init(p_uart);
        }
#if defined(HAL_STATUS_CHECK)
    }
#endif

#if defined(HAL_STATUS_CHECK)
    uart_set_device_state(p_uart, TX_RX, HAL_UART_STATE_BUSY);
#endif

    /* Set the UART Communication parameters */
    InitStruct.baud_rate    = p_uart->init.baud_rate;
    InitStruct.data_bits    = p_uart->init.data_bits;
    InitStruct.parity       = p_uart->init.parity;
    InitStruct.stop_bits    = p_uart->init.stop_bits;
    InitStruct.hw_flow_ctrl = p_uart->init.hw_flow_ctrl;
    //lint -e934 Taking address of near auto variable is necessary
    ll_uart_init(p_uart->p_instance, &InitStruct);

#if defined(HAL_STATUS_CHECK)
    /* Initialize the UART ErrorCode */
    p_uart->error_code = HAL_UART_ERROR_NONE;
#endif

#if defined(HAL_STATUS_CHECK)
    /* Initialize the UART State */
    uart_set_device_state(p_uart, TX_RX, HAL_UART_STATE_READY);
#endif

    return HAL_OK;
}

__WEAK hal_status_t hal_uart_deinit(uart_handle_t *p_uart)
{
#if defined(HAL_PARAMS_CHECK)
    /* Check the parameters */
    gr_assert_param(IS_UART_ALL_INSTANCE(p_uart->p_instance));
#endif

#if defined(HAL_STATUS_CHECK)
    hal_uart_state_t tx_state = p_uart->tx_state;
    hal_uart_state_t rx_state = p_uart->rx_state;
    if ((tx_state != HAL_UART_STATE_RESET) || (rx_state != HAL_UART_STATE_RESET))
    {
#endif

#if defined(HAL_STATUS_CHECK)
        uart_set_device_state(p_uart, TX_RX, HAL_UART_STATE_BUSY);
#endif

        /* Check TEMT to make sure all data was sent out */
        ll_uart_disable_break_sending(p_uart->p_instance);

        //lint -e534
        uart_wait_line_flag_until_timeout(p_uart, UART_FLAG_LINE_TEMT, RESET, HAL_UART_TIMEOUT_DEFAULT_VALUE);


        /* Set the UART registers to default value */
        ll_uart_deinit(p_uart->p_instance);

        /* DeInit the low level hardware */
        hal_uart_msp_deinit(p_uart);
#ifdef HAL_CLOCK_UNIFORM_CONTROL
        hal_clock_disable_module((uint32_t)p_uart->p_instance);
#endif
#ifdef HAL_CLOCK_MODULE
        /* Disable UARTx Clock */
        //lint -e923 Cast from pointer to unsigned int is necessary
        if(p_uart->p_instance == UART0)
        {
            ll_cgc_enable_force_off_uart0_hclk();
            ll_cgc_enable_uart0_slp_wfi();
        }
        else /* UART1 */
        {
            ll_cgc_enable_force_off_uart1_hclk();
            ll_cgc_enable_uart1_slp_wfi();
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
#if defined(HAL_STATUS_CHECK)
        p_uart->error_code = HAL_UART_ERROR_NONE;
#endif

#if defined(HAL_STATUS_CHECK)
        uart_set_device_state(p_uart, TX_RX, HAL_UART_STATE_RESET);
#endif

#if defined(HAL_STATUS_CHECK)
    }
#endif

    return HAL_OK;
}

__WEAK void hal_uart_msp_init(uart_handle_t *p_uart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_uart);
}

__WEAK void hal_uart_msp_deinit(uart_handle_t *p_uart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_uart);
}

/** @} */

/** @defgroup UART_Exported_Functions_Group2 IO operation functions
  * @brief UART Transmit/Receive functions
  * @{
  */

__WEAK hal_status_t hal_uart_transmit(uart_handle_t *p_uart, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
#if defined(HAL_PARAMS_CHECK)
    if ((NULL == p_data) || (0U == size))
    {
        p_uart->error_code |= HAL_UART_ERROR_INVALID_PARAM;
        return  HAL_ERROR;
    }
#endif

#if defined(HAL_STATUS_CHECK)
    /* Check that a Tx process is not already ongoing */
    if (HAL_UART_STATE_READY == p_uart->tx_state)
    {
#endif

#if defined(HAL_STATUS_CHECK)
        p_uart->error_code = HAL_UART_ERROR_NONE;
#endif

#if defined(HAL_STATUS_CHECK)
        uart_set_device_state(p_uart, TX, HAL_UART_STATE_BUSY_TX);
#endif

        p_uart->tx_xfer_size  = size;

        while (0U < size)
        {
            if (HAL_OK != uart_wait_fifo_flag_until_timeout(p_uart, UART_FLAG_FIFO_TFNF, RESET, timeout))
            {
                return HAL_TIMEOUT;
            }
            size--;
            ll_uart_transmit_data8(p_uart->p_instance, *p_data++);
        }

        if (HAL_OK != uart_wait_line_flag_until_timeout(p_uart, UART_FLAG_LINE_TEMT, RESET, timeout))
        {
            return HAL_TIMEOUT;
        }

#if defined(HAL_STATUS_CHECK)
        /* At end of Tx process, restore p_uart->gState to Ready */
        uart_set_device_state(p_uart, TX, HAL_UART_STATE_READY);
#endif
        return HAL_OK;
#if defined(HAL_STATUS_CHECK)
    }
    else
    {
        return HAL_BUSY;
    }
#endif
}

__WEAK hal_status_t hal_uart_receive(uart_handle_t *p_uart, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
#if defined(HAL_PARAMS_CHECK)
    if ((NULL == p_data) || (0U == size))
    {
        p_uart->error_code |= HAL_UART_ERROR_INVALID_PARAM;
        return  HAL_ERROR;
    }
#endif

#if defined(HAL_STATUS_CHECK)
    /* Check that a Rx process is not already ongoing */
    if (HAL_UART_STATE_READY == p_uart->rx_state)
    {
#endif

#if defined(HAL_STATUS_CHECK)
        p_uart->error_code = HAL_UART_ERROR_NONE;
#endif

#if defined(HAL_STATUS_CHECK)
        uart_set_device_state(p_uart, RX, HAL_UART_STATE_BUSY_RX);
#endif

        p_uart->rx_xfer_size  = size;
        p_uart->rx_xfer_count = size;

        /* as long as data have to be received */
        while (0U < p_uart->rx_xfer_count)
        {
            if (HAL_OK != uart_wait_fifo_flag_until_timeout(p_uart, UART_FLAG_FIFO_RFNE, RESET, timeout))
            {
                return HAL_TIMEOUT;
            }
            p_uart->rx_xfer_count--;
            *p_data++ = ll_uart_receive_data8(p_uart->p_instance);
        }

#if defined(HAL_STATUS_CHECK)
        /* At end of Rx process, restore p_uart->RxState to Ready */
        uart_set_device_state(p_uart, RX, HAL_UART_STATE_READY);
#endif
        return HAL_OK;
#if defined(HAL_STATUS_CHECK)
    }
    else
    {
        return HAL_BUSY;
    }
#endif
}

__WEAK hal_status_t hal_uart_transmit_it(uart_handle_t *p_uart, uint8_t *p_data, uint16_t size)
{
#if defined(HAL_PARAMS_CHECK)
    if ((NULL == p_data) || (0U == size))
    {
        p_uart->error_code |= HAL_UART_ERROR_INVALID_PARAM;
        return  HAL_ERROR;
    }
#endif

#if defined(HAL_STATUS_CHECK)
    /* Check that a Tx process is not already ongoing */
    if (HAL_UART_STATE_READY == p_uart->tx_state)
    {
#endif

        p_uart->p_tx_buffer   = p_data;
        p_uart->tx_xfer_size  = size;
        p_uart->tx_xfer_count = size;

#if defined(HAL_STATUS_CHECK)
        p_uart->error_code = HAL_UART_ERROR_NONE;
#endif

#if defined(HAL_STATUS_CHECK)
        uart_set_device_state(p_uart, TX, HAL_UART_STATE_BUSY_TX);
#endif

        ll_uart_set_tx_fifo_threshold(p_uart->p_instance, LL_UART_TX_FIFO_TH_HALF_FULL);

        /* Note : The UART interrupt transfer must be enabled after unlocking current process
                to avoid the risk of UART interrupt handle execution before current
                process unlock */

        /* Enable the UART Transmit Data Register Empty Interrupt */
        __HAL_UART_ENABLE_IT(p_uart, UART_IT_THRE);

        return HAL_OK;
#if defined(HAL_STATUS_CHECK)
    }
    else
    {
        return HAL_BUSY;
    }
#endif
}

__WEAK hal_status_t hal_uart_receive_it(uart_handle_t *p_uart, uint8_t *p_data, uint16_t size)
{
#if defined(HAL_PARAMS_CHECK)
    if ((NULL == p_data) || (0U == size))
    {
        p_uart->error_code |= HAL_UART_ERROR_INVALID_PARAM;
        return  HAL_ERROR;
    }
#endif

#if defined(HAL_STATUS_CHECK)
    /* Check that a Rx process is not already ongoing */
    if (HAL_UART_STATE_READY == p_uart->rx_state)
    {
#endif

        p_uart->p_rx_buffer   = p_data;
        p_uart->rx_xfer_size  = size;
        p_uart->rx_xfer_count = size;

#if defined(HAL_STATUS_CHECK)
        p_uart->error_code = HAL_UART_ERROR_NONE;
#endif

#if defined(HAL_STATUS_CHECK)
        uart_set_device_state(p_uart, RX, HAL_UART_STATE_BUSY_RX);
#endif

        ll_uart_set_rx_fifo_threshold(p_uart->p_instance, LL_UART_RX_FIFO_TH_HALF_FULL);

        /* If Overrun error occurs. */
        if(ll_uart_get_line_status_flag(p_uart->p_instance) & HAL_UART_ERROR_OE)
        {
            ll_uart_clear_receive_data8(p_uart->p_instance); //Clear TIMEOUT Interrupt
            ll_uart_flush_rx_fifo(p_uart->p_instance);
        }

        /* Note : The UART interrupt transfer must be enabled after unlocking current process
                to avoid the risk of UART interrupt handle execution before current
                process unlock */

        /* Enable the UART Line Status Interrupt */
        /* Enable the UART Received Data Available and Character Timeout Interrupts */
        __HAL_UART_ENABLE_IT(p_uart, UART_IT_RLS | UART_IT_RDA);

        return HAL_OK;
#if defined(HAL_STATUS_CHECK)
    }
    else
    {
        return HAL_BUSY;
    }
#endif
}

__WEAK hal_status_t hal_uart_transmit_dma(uart_handle_t *p_uart, uint8_t *p_data, uint16_t size)
{
    hal_status_t status   = HAL_OK;

#if defined(HAL_PARAMS_CHECK)
    /* Check if UART p_instance supports continuous communication using DMA */
    gr_assert_param(IS_UART_ALL_INSTANCE(p_uart->p_instance));

    if ((NULL == p_data) || (0U == size) || (NULL == p_uart->p_dmatx))
    {
        p_uart->error_code |= HAL_UART_ERROR_INVALID_PARAM;
        return  HAL_ERROR;
    }
#endif

#if defined(HAL_STATUS_CHECK)
    /* Check that a Tx process is not already ongoing */
    if (HAL_UART_STATE_READY == p_uart->tx_state)
    {
#endif

        /* Set UART TX FIFO Threshold and DMA Burst Length for DMA transfer */
        ll_uart_set_tx_fifo_threshold(p_uart->p_instance, LL_UART_TX_FIFO_TH_CHAR_2);
        ll_dma_set_destination_burst_length(p_uart->p_dmatx->p_instance, (uint32_t)p_uart->p_dmatx->channel, LL_DMA_DST_BURST_LENGTH_4);
        p_uart->p_tx_buffer   = p_data;
        p_uart->tx_xfer_size  = size;
        p_uart->tx_xfer_count = size;

#if defined(HAL_STATUS_CHECK)
        p_uart->error_code  = HAL_UART_ERROR_NONE;
#endif

#if defined(HAL_STATUS_CHECK)
        uart_set_device_state(p_uart, TX, HAL_UART_STATE_BUSY_TX);
#endif
        p_uart->dma_tx_mode = ENABLE;

        /* Set the UART DMA transfer complete callback */
        p_uart->p_dmatx->xfer_tfr_callback = uart_dma_transmit_cplt;

        /* Set the DMA error callback */
        p_uart->p_dmatx->xfer_error_callback = uart_dma_error;

        /* Set the DMA abort callback */
        p_uart->p_dmatx->xfer_abort_callback = NULL;

        /* Note : The UART DMA transfer must be enabled after unlocking current process
                to avoid the risk of UART/DMA interrupt handle execution before current
                process unlock */

        /* Enable the UART transmit DMA channel */
        status = hal_dma_start_it(p_uart->p_dmatx, (uint32_t)p_uart->p_tx_buffer, ll_uart_dma_get_register_address(p_uart->p_instance), size);

#if defined(HAL_STATUS_CHECK)
        if (HAL_OK != status)
        {
            p_uart->error_code |= HAL_UART_ERROR_DMA;
#endif
#if defined(HAL_STATUS_CHECK)
            /* Update UART state */
            uart_set_device_state(p_uart, TX, HAL_UART_STATE_READY);
#endif
#if defined(HAL_STATUS_CHECK)
        }
    }
    else
    {
        status = HAL_BUSY;
    }
#endif
    return status;
}

__WEAK hal_status_t hal_uart_receive_dma(uart_handle_t *p_uart, uint8_t *p_data, uint16_t size)
{
    hal_status_t status   = HAL_OK;

#if defined(HAL_PARAMS_CHECK)
    /* Check if UART p_instance supports continuous communication using DMA */
    gr_assert_param(IS_UART_ALL_INSTANCE(p_uart->p_instance));

    if ((NULL == p_data) || (0U == size) || (NULL == p_uart->p_dmarx))
    {
        p_uart->error_code |= HAL_UART_ERROR_INVALID_PARAM;
        return  HAL_ERROR;
    }
#endif

    /* Check that a Rx process is not already ongoing */
#if defined(HAL_STATUS_CHECK)
    if (HAL_UART_STATE_READY == p_uart->rx_state)
    {
#endif

        /* Set UART RX FIFO Threshold and DMA Burst Length for DMA transfer */
        if (UART_RECEIVER_TIMEOUT_DISABLE == p_uart->init.rx_timeout_mode)
        {
            ll_uart_set_rx_fifo_threshold(p_uart->p_instance, LL_UART_RX_FIFO_TH_CHAR_1);
            ll_dma_set_source_burst_length(p_uart->p_dmarx->p_instance, (uint32_t)p_uart->p_dmarx->channel , LL_DMA_SRC_BURST_LENGTH_1);
        }
        else
        {
            ll_uart_set_rx_fifo_threshold(p_uart->p_instance, LL_UART_RX_FIFO_TH_HALF_FULL);
            ll_dma_set_source_burst_length(p_uart->p_dmarx->p_instance, (uint32_t)p_uart->p_dmarx->channel , LL_DMA_SRC_BURST_LENGTH_4);
        }

        p_uart->p_rx_buffer   = p_data;
        p_uart->rx_xfer_size  = size;
        p_uart->rx_xfer_count = size;

#if defined(HAL_STATUS_CHECK)
        p_uart->error_code  = HAL_UART_ERROR_NONE;
#endif

#if defined(HAL_STATUS_CHECK)
        uart_set_device_state(p_uart, RX, HAL_UART_STATE_BUSY_RX);
#endif
        p_uart->dma_rx_mode = ENABLE;

        /* Set the UART DMA transfer complete callback */
        p_uart->p_dmarx->xfer_tfr_callback = uart_dma_receive_cplt;

        /* Set the DMA error callback */
        p_uart->p_dmarx->xfer_error_callback = uart_dma_error;

        /* Set the DMA abort callback */
        p_uart->p_dmarx->xfer_abort_callback = NULL;

        /* If Overrun error occurs. */
        if(ll_uart_get_line_status_flag(p_uart->p_instance) & HAL_UART_ERROR_OE)
        {
            ll_uart_clear_receive_data8(p_uart->p_instance); //Clear TIMEOUT Interrupt
            ll_uart_flush_rx_fifo(p_uart->p_instance);
        }

        /* Note : The UART DMA transfer must be enabled after unlocking current process
                to avoid the risk of UART/DMA interrupt handle execution before current
                process unlock */

        /* Enable the DMA channel */
        status = hal_dma_start_it(p_uart->p_dmarx, ll_uart_dma_get_register_address(p_uart->p_instance), (uint32_t)p_uart->p_rx_buffer, size);

        if (HAL_OK == status)
        {
            /* Enable RLS Interrupt */
            __HAL_UART_ENABLE_IT(p_uart, UART_IT_RLS);
            if (UART_RECEIVER_TIMEOUT_ENABLE == p_uart->init.rx_timeout_mode)
            {
                /* Enable Received Data Available Interrupt and Character Timeout Interrupt */
                __HAL_UART_ENABLE_IT(p_uart, UART_IT_RDA);
            }
        }
        else
        {
#if defined(HAL_STATUS_CHECK)
            p_uart->error_code |= HAL_UART_ERROR_DMA;
#endif

#if defined(HAL_STATUS_CHECK)
            /* Update UART state */
            uart_set_device_state(p_uart, RX, HAL_UART_STATE_READY);
#endif
        }
#if defined(HAL_STATUS_CHECK)
    }
    else
    {
        status = HAL_BUSY;
    }
#endif
    return status;
}

__WEAK hal_status_t hal_uart_dma_pause(uart_handle_t *p_uart)
{
#if defined(HAL_STATUS_CHECK)
    if (HAL_UART_STATE_BUSY_TX == p_uart->tx_state)
    {
#endif
        if(ENABLE == p_uart->dma_tx_mode)
        {
            /* Suspend the DMA channel to disable the UART DMA Tx request */
            ll_dma_suspend_channel(p_uart->p_dmatx->p_instance, (uint32_t)p_uart->p_dmatx->channel);
        }
#if defined(HAL_STATUS_CHECK)
    }
#endif

#if defined(HAL_STATUS_CHECK)
    if (HAL_UART_STATE_BUSY_RX == p_uart->rx_state)
    {
#endif
        if(ENABLE == p_uart->dma_rx_mode)
        {
            /* Disable RLS interrupt */
            __HAL_UART_DISABLE_IT(p_uart, UART_IT_RLS);
            /* Disable RDA interrupt */
            __HAL_UART_DISABLE_IT(p_uart, UART_IT_RDA);

            /* Suspend the DMA channel to disable the UART DMA Rx request */
            ll_dma_suspend_channel(p_uart->p_dmarx->p_instance, (uint32_t)p_uart->p_dmarx->channel);
        }
#if defined(HAL_STATUS_CHECK)
    }
#endif

    return HAL_OK;
}

__WEAK hal_status_t hal_uart_dma_resume(uart_handle_t *p_uart)
{
#if defined(HAL_STATUS_CHECK)
    if (HAL_UART_STATE_BUSY_TX == p_uart->tx_state)
    {
#endif
        /* Enable the UART DMA Tx request */
        ll_dma_resume_channel(p_uart->p_dmatx->p_instance, (uint32_t)p_uart->p_dmatx->channel);
#if defined(HAL_STATUS_CHECK)
    }
#endif

#if defined(HAL_STATUS_CHECK)
    if (HAL_UART_STATE_BUSY_RX == p_uart->rx_state)
    {
#endif
        /* Clear receive line error status */
        ll_uart_clear_line_status_flag(p_uart->p_instance);

        /* Enable the UART DMA Rx request */
        ll_dma_resume_channel(p_uart->p_dmarx->p_instance, (uint32_t)p_uart->p_dmarx->channel);

        /* Enable RLS interrupt */
        __HAL_UART_ENABLE_IT(p_uart, UART_IT_RLS);
        /* Enable RDA interrupt */
        __HAL_UART_ENABLE_IT(p_uart, UART_IT_RDA);
#if defined(HAL_STATUS_CHECK)
    }
#endif

    return HAL_OK;
}

__WEAK hal_status_t hal_uart_dma_stop(uart_handle_t *p_uart)
{
    /* The Lock is not implemented on this API to allow the user application
       to call the HAL UART API under callbacks hal_uart_tx_cplt_callback() / hal_uart_rx_cplt_callback():
       indeed, when hal_dma_abort() API is called, the DMA TX/RX Transfer complete interrupt is generated
       if the DMA transfer interruption occurs at the middle or at the end of the stream and the
       corresponding call back is executed. */

    /* Stop UART DMA Tx request if ongoing */
    uart_stop_dma_tx(p_uart);

    /* Stop UART DMA Rx request if ongoing */
    uart_stop_dma_rx(p_uart);

    return HAL_OK;
}

__WEAK hal_status_t hal_uart_abort(uart_handle_t *p_uart)
{
    /* Disable THRE, RLS, RDA interrupts */
    __HAL_UART_DISABLE_IT(p_uart, UART_IT_THRE | UART_IT_RLS | UART_IT_RDA);

    /* Disable the UART DMA Tx request if enabled */
    if (ENABLE == p_uart->dma_tx_mode)
    {
        /* Abort the UART DMA Tx channel : use blocking DMA Abort API (no callback) */
        if (NULL != p_uart->p_dmatx)
        {
            /* Set the UART DMA Abort callback to Null.
                No call back execution at end of DMA abort procedure */
            p_uart->p_dmatx->xfer_abort_callback = NULL;

            hal_dma_abort(p_uart->p_dmatx);
        }
        p_uart->dma_tx_mode = DISABLE;
    }

    /* Disable the UART DMA Rx request if enabled */
    if (ENABLE == p_uart->dma_rx_mode)
    {
        /* Abort the UART DMA Rx channel : use blocking DMA Abort API (no callback) */
        if (NULL != p_uart->p_dmarx)
        {
            /* Set the UART DMA Abort callback to Null.
                No call back execution at end of DMA abort procedure */
            p_uart->p_dmarx->xfer_abort_callback = NULL;

            hal_dma_abort(p_uart->p_dmarx);
        }
        p_uart->dma_rx_mode = DISABLE;
    }

    /* Reset Tx and Rx transfer counters */
    p_uart->tx_xfer_count = 0U;
    p_uart->rx_xfer_count = 0U;

    /* Clear the Error flags in the LSR register */
    ll_uart_clear_line_status_flag(p_uart->p_instance);

    /* Flush the whole TX FIFO and Discard the received data */
    __HAL_UART_SEND_REQ(p_uart, UART_TXRXDATA_FLUSH_REQUEST);

#if defined(HAL_STATUS_CHECK)
    /* Restore p_uart->gState and p_uart->RxState to Ready */
    uart_set_device_state(p_uart, TX_RX, HAL_UART_STATE_READY);
#endif

#if defined(HAL_STATUS_CHECK)
    /* Reset Handle ErrorCode to No Error */
    p_uart->error_code = HAL_UART_ERROR_NONE;
#endif

    return HAL_OK;
}

__WEAK hal_status_t hal_uart_abort_transmit(uart_handle_t *p_uart)
{
    /* Disable THRE interrupt */
    __HAL_UART_DISABLE_IT(p_uart, UART_IT_THRE);

    /* Disable the UART DMA Tx request if enabled */
    if (ENABLE == p_uart->dma_tx_mode)
    {
        /* Abort the UART DMA Tx channel : use blocking DMA Abort API (no callback) */
        if (NULL != p_uart->p_dmatx)
        {
            /* Set the UART DMA Abort callback to Null.
                No call back execution at end of DMA abort procedure */
            p_uart->p_dmatx->xfer_abort_callback = NULL;

            hal_dma_abort(p_uart->p_dmatx);
        }
        p_uart->dma_tx_mode = DISABLE;
    }

    /* Reset Tx transfer counter */
    p_uart->tx_xfer_count = 0U;

    /* Flush the whole TX FIFO */
    __HAL_UART_SEND_REQ(p_uart, UART_TXDATA_FLUSH_REQUEST);

#if defined(HAL_STATUS_CHECK)
    /* Restore p_uart->gState to Ready */
    uart_set_device_state(p_uart, TX, HAL_UART_STATE_READY);
#endif

    return HAL_OK;
}

__WEAK hal_status_t hal_uart_abort_receive(uart_handle_t *p_uart)
{
    /* Disable RLS, RDA interrupts */
    __HAL_UART_DISABLE_IT(p_uart, UART_IT_RLS | UART_IT_RDA);

    /* Disable the UART DMA Rx request if enabled */
    if (ENABLE == p_uart->dma_rx_mode)
    {
        /* Abort the UART DMA Rx channel : use blocking DMA Abort API (no callback) */
        if (NULL != p_uart->p_dmarx)
        {
            /* Set the UART DMA Abort callback to Null.
                No call back execution at end of DMA abort procedure */
            p_uart->p_dmarx->xfer_abort_callback = NULL;

            hal_dma_abort(p_uart->p_dmarx);
        }
        p_uart->dma_rx_mode = DISABLE;
    }

    /* Reset Rx transfer counter */
    p_uart->rx_xfer_count = 0U;

    /* Clear the Error flags in the LSR register */
    ll_uart_clear_line_status_flag(p_uart->p_instance);

    /* Discard the received data */
    __HAL_UART_SEND_REQ(p_uart, UART_RXDATA_FLUSH_REQUEST);

#if defined(HAL_STATUS_CHECK)
    /* Restore p_uart->RxState to Ready */
    uart_set_device_state(p_uart, RX, HAL_UART_STATE_READY);
#endif
    return HAL_OK;
}

__WEAK hal_status_t hal_uart_abort_it(uart_handle_t *p_uart)
{
    uint32_t abortcplt = 1U;

    /* Disable THRE, RLS, RDA interrupts */
    __HAL_UART_DISABLE_IT(p_uart, UART_IT_THRE | UART_IT_RLS | UART_IT_RDA);

    /* Disable the UART DMA Tx request if enabled */
    if (ENABLE == p_uart->dma_tx_mode)
    {
        /* Abort the UART DMA Tx channel : use non blocking DMA Abort API (callback) */
        if (NULL != p_uart->p_dmatx)
        {
            /* UART Tx DMA Abort callback will lead to call hal_uart_abort_cplt_callback()
               at end of DMA abort procedure */
            p_uart->p_dmatx->xfer_abort_callback = uart_dma_tx_abort_callback;

            /* Abort DMA TX */
            if (HAL_OK != hal_dma_abort_it(p_uart->p_dmatx))
            {
                p_uart->p_dmatx->xfer_abort_callback = NULL;
            }
            else
            {
                abortcplt = 0U;
            }
        }
    }

    /* Disable the UART DMA Rx request if enabled */
    if (ENABLE == p_uart->dma_rx_mode)
    {
        /* Abort the UART DMA Rx channel : use non blocking DMA Abort API (callback) */
        if (NULL != p_uart->p_dmarx)
        {
            /* UART Rx DMA Abort callback will lead to call hal_uart_abort_cplt_callback()
               at end of DMA abort procedure */
            p_uart->p_dmarx->xfer_abort_callback = uart_dma_rx_abort_callback;

            /* Abort DMA RX */
            if (HAL_OK != hal_dma_abort_it(p_uart->p_dmarx))
            {
                p_uart->p_dmarx->xfer_abort_callback = NULL;
                abortcplt = 1U;
            }
            else
            {
                abortcplt = 0U;
            }
        }
    }

    /* if no DMA abort complete callback execution is required => call user Abort Complete callback */
    if (1U == abortcplt)
    {
        /* Reset Tx and Rx transfer counters */
        p_uart->tx_xfer_count = 0U;
        p_uart->rx_xfer_count = 0U;

#if defined(HAL_STATUS_CHECK)
        /* Reset errorCode */
        p_uart->error_code = HAL_UART_ERROR_NONE;
#endif

        /* Clear the Error flags in the LSR register */
        ll_uart_clear_line_status_flag(p_uart->p_instance);

        /* Flush the whole TX FIFO and Discard the received data */
        __HAL_UART_SEND_REQ(p_uart, UART_TXRXDATA_FLUSH_REQUEST);

        /* Restore p_uart->gState and p_uart->RxState to Ready */
#if defined(HAL_STATUS_CHECK)
        uart_set_device_state(p_uart, TX_RX, HAL_UART_STATE_READY);
#endif

        /* As no DMA to be aborted, call directly user Abort complete callback */
        hal_uart_abort_cplt_callback(p_uart);

        p_uart->dma_tx_mode = DISABLE;
        p_uart->dma_rx_mode = DISABLE;
    }
    return HAL_OK;
}

__WEAK hal_status_t hal_uart_abort_transmit_it(uart_handle_t *p_uart)
{
    /* Disable THRE interrupt */
    __HAL_UART_DISABLE_IT(p_uart, UART_IT_THRE);

    /* Disable the UART DMA Tx request if enabled */
    if (ENABLE == p_uart->dma_tx_mode)
    {
        /* Abort the UART DMA Tx channel : use non blocking DMA Abort API (callback) */
        if (NULL != p_uart->p_dmatx)
        {
            /* Set the UART DMA Abort callback :
                will lead to call hal_uart_abort_cplt_callback() at end of DMA abort procedure */
            p_uart->p_dmatx->xfer_abort_callback = uart_dma_tx_only_abort_callback;

            /* Abort DMA TX */
            if (HAL_OK != hal_dma_abort_it(p_uart->p_dmatx))
            {
                /* Call Directly p_uart->p_dmatx->XferAbortCallback function in case of error */
                p_uart->p_dmatx->xfer_abort_callback(p_uart->p_dmatx);
            }
        }
        else
        {
            /* Reset Tx transfer counter */
            p_uart->tx_xfer_count = 0U;

#if defined(HAL_STATUS_CHECK)
            /* Restore p_uart->gState to Ready */
            uart_set_device_state(p_uart, TX, HAL_UART_STATE_READY);
#endif
            /* As no DMA to be aborted, call directly user Abort complete callback */
            hal_uart_abort_tx_cplt_callback(p_uart);
        }
        p_uart->dma_tx_mode = DISABLE;
    }
    else
    {
        /* Reset Tx transfer counter */
        p_uart->tx_xfer_count = 0U;

        /* Flush the whole TX FIFO */
        __HAL_UART_SEND_REQ(p_uart, UART_TXDATA_FLUSH_REQUEST);

#if defined(HAL_STATUS_CHECK)
        /* Restore p_uart->gState to Ready */
        uart_set_device_state(p_uart, TX, HAL_UART_STATE_READY);
#endif

        /* As no DMA to be aborted, call directly user Abort complete callback */
        hal_uart_abort_tx_cplt_callback(p_uart);
    }

    return HAL_OK;
}

__WEAK hal_status_t hal_uart_abort_receive_it(uart_handle_t *p_uart)
{
    /* Disable RLS, RDA interrupts */
    __HAL_UART_DISABLE_IT(p_uart, UART_IT_RLS | UART_IT_RDA);

    /* Disable the UART DMA Rx request if enabled */
    if (ENABLE == p_uart->dma_rx_mode)
    {
        /* Abort the UART DMA Rx channel : use non blocking DMA Abort API (callback) */
        if (NULL != p_uart->p_dmarx)
        {
            /* Set the UART DMA Abort callback :
                will lead to call hal_uart_abort_cplt_callback() at end of DMA abort procedure */
            p_uart->p_dmarx->xfer_abort_callback = uart_dma_rx_only_abort_callback;

            /* Abort DMA RX */
            if (HAL_OK != hal_dma_abort_it(p_uart->p_dmarx))
            {
                /* Call Directly p_uart->p_dmarx->XferAbortCallback function in case of error */
                p_uart->p_dmarx->xfer_abort_callback(p_uart->p_dmarx);
            }
        }
        else
        {
            /* Reset Rx transfer counter */
            p_uart->rx_xfer_count = 0U;

            /* Clear the Error flags in the LSR register */
            ll_uart_clear_line_status_flag(p_uart->p_instance);

            /* Discard the received data */
            __HAL_UART_SEND_REQ(p_uart, UART_RXDATA_FLUSH_REQUEST);

#if defined(HAL_STATUS_CHECK)
            /* Restore p_uart->RxState to Ready */
            uart_set_device_state(p_uart, RX, HAL_UART_STATE_READY);
#endif

            /* As no DMA to be aborted, call directly user Abort complete callback */
            hal_uart_abort_rx_cplt_callback(p_uart);
        }
        p_uart->dma_rx_mode = DISABLE;
    }
    else
    {
        /* Reset Rx transfer counter */
        p_uart->rx_xfer_count = 0U;

        /* Clear the Error flags in the LSR register */
        ll_uart_clear_line_status_flag(p_uart->p_instance);

#if defined(HAL_STATUS_CHECK)
        /* Restore p_uart->RxState to Ready */
        uart_set_device_state(p_uart, RX, HAL_UART_STATE_READY);
#endif

        /* As no DMA to be aborted, call directly user Abort complete callback */
        hal_uart_abort_rx_cplt_callback(p_uart);
    }

    return HAL_OK;
}

__WEAK void hal_uart_irq_handler(uart_handle_t *p_uart)
{
    uart_regs_t *p_instance = p_uart->p_instance;
    uint32_t isrflag      = ll_uart_get_it_flag(p_instance);
    uint32_t linestat     = 0;
    flag_status_t ctostat = RESET;

    switch (isrflag)
    {
    /* UART transmit hold register empty */
    case LL_UART_IIR_THRE:
        uart_transmit_it(p_uart);
        break;

    /* UART receive data avialble */
    case LL_UART_IIR_RDA:
        uart_receive_it(p_uart, RESET);
        break;

    /* UART character timeout */
    case LL_UART_IIR_CTO:
        /* In rx_timeout_mode, rx timeout will also be treated as receive complete. */
        if (UART_RECEIVER_TIMEOUT_ENABLE == p_uart->init.rx_timeout_mode)
        {
            ctostat = SET;
        }
        uart_receive_it(p_uart, ctostat);
        break;

    /* UART receive line error */
    case LL_UART_IIR_RLS:
        linestat = ll_uart_get_line_status_flag(p_instance);
#if defined(HAL_STATUS_CHECK)
        p_uart->error_code |= (linestat & UART_LINE_ERROR_MASK);
#endif

        if (ll_uart_is_active_flag_rfne(p_instance))
        {
            /* Method 1: call uart_receive_it() to continue receive error data,
               Method 2: do nothing, user need to decide whether flush RX FIFO
               or continue to receive data */
            uart_receive_it(p_uart, RESET);
        }

        /* Call UART Error Call back function if need be ----------------------------*/
        /* If Overrun error occurs, or if any error occurs in DMA mode reception,
            consider error as blocking */
        if (p_uart->error_code & HAL_UART_ERROR_OE)
        {
            /* Blocking error : transfer is aborted
                Set the UART state ready to be able to start again the process,
                Disable Rx Interrupts, and disable Rx DMA request, if ongoing */
            uart_end_rx_transfer(p_uart);

            /* Disable the UART DMA Rx request if enabled */
            if (ENABLE == p_uart->dma_rx_mode)
            {
                /* Abort the UART DMA Rx channel */
                if (NULL != p_uart->p_dmarx)
                {
                    /* Set the UART DMA Abort callback :
                        will lead to call hal_uart_error_callback() at end of DMA abort procedure */
                    p_uart->p_dmarx->xfer_abort_callback = uart_dma_abort_on_error;

                    /* Abort DMA RX */
                    if (HAL_OK != hal_dma_abort_it(p_uart->p_dmarx))
                    {
                        /* Call Directly p_uart->p_dmarx->XferAbortCallback function in case of error */
                        p_uart->p_dmarx->xfer_abort_callback(p_uart->p_dmarx);
                    }
                }
                else
                {
                    /* Call user error callback */
                    hal_uart_error_callback(p_uart);
                }
                p_uart->dma_rx_mode = DISABLE;
            }
            else
            {
                /* Call user error callback */
                hal_uart_error_callback(p_uart);
            }
        }
        else
        {
            /* Non Blocking error : transfer could go on.
                Error is notified to user through user error callback */
            hal_uart_error_callback(p_uart);
#if defined(HAL_STATUS_CHECK)
            p_uart->error_code = HAL_UART_ERROR_NONE;
#endif
        }
        break;

    default:
        /* Do nothing */
        break;
    }
    return;
}

__WEAK void hal_uart_tx_cplt_callback(uart_handle_t *p_uart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_uart);
}

__WEAK void hal_uart_rx_cplt_callback(uart_handle_t *p_uart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_uart);
}

__WEAK void hal_uart_error_callback(uart_handle_t *p_uart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_uart);
}

__WEAK void hal_uart_abort_cplt_callback (uart_handle_t *p_uart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_uart);
}

__WEAK void hal_uart_abort_tx_cplt_callback (uart_handle_t *p_uart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_uart);
}

__WEAK void hal_uart_abort_rx_cplt_callback (uart_handle_t *p_uart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_uart);
}

/** @} */


/** @defgroup UART_Exported_Functions_Group3 Peripheral Control and State functions
 *  @brief   UART Peripheral State functions
  * @{
  */


__WEAK hal_uart_state_t hal_uart_get_state(uart_handle_t *p_uart)
{
    hal_uart_state_t state;
    hal_uart_state_t tx_state = p_uart->tx_state;
    hal_uart_state_t rx_state = p_uart->rx_state;
    if((tx_state == HAL_UART_STATE_RESET) && (rx_state ==  HAL_UART_STATE_RESET))
    {
        state = HAL_UART_STATE_RESET;
    }
    else if((tx_state == HAL_UART_STATE_READY) && (rx_state ==  HAL_UART_STATE_READY))
    {
        state = HAL_UART_STATE_READY;
    }
    else
    {
        state = HAL_UART_STATE_BUSY;
    }
    return state;
}

__WEAK uint32_t hal_uart_get_error(uart_handle_t *p_uart)
{
    return p_uart->error_code;
}


__WEAK void hal_uart_suspend_reg(uart_handle_t *p_uart)
{
#ifndef HAL_HW_RES_SUSP_UART
    uart_regs_t *p_uart_reg = p_uart->p_instance;

    p_uart->retention[0] = READ_REG(p_uart_reg->LCR);
    SET_BITS(p_uart_reg->LCR, UART_LCR_DLAB);
    p_uart->retention[1] = READ_REG(p_uart_reg->RBR_DLL_THR.DLL);
    p_uart->retention[2] = READ_REG(p_uart_reg->DLH_IER.DLH);
    CLEAR_BITS(p_uart_reg->LCR, UART_LCR_DLAB);
    p_uart->retention[3] = READ_REG(p_uart_reg->DLF);
    p_uart->retention[4] = READ_REG(p_uart_reg->MCR);
    p_uart->retention[5] = READ_REG(p_uart_reg->SFE);
    p_uart->retention[6] = READ_REG(p_uart_reg->STET);
    p_uart->retention[7] = READ_REG(p_uart_reg->SRT);
#else
    UNUSED(p_uart);
#endif
    return ;
}

__WEAK void hal_uart_resume_reg(uart_handle_t *p_uart)
{
#ifndef HAL_HW_RES_SUSP_UART
    uart_regs_t *p_uart_reg = p_uart->p_instance;

    WRITE_REG(p_uart_reg->LCR, p_uart->retention[0]);
    SET_BITS(p_uart->p_instance->LCR, UART_LCR_DLAB);
    WRITE_REG(p_uart_reg->RBR_DLL_THR.DLL, p_uart->retention[1]);
    WRITE_REG(p_uart_reg->DLH_IER.DLH, p_uart->retention[2]);
    CLEAR_BITS(p_uart->p_instance->LCR, UART_LCR_DLAB);
    WRITE_REG(p_uart_reg->DLF, p_uart->retention[3]);
    WRITE_REG(p_uart_reg->MCR, p_uart->retention[4]);
    WRITE_REG(p_uart_reg->SRR, UART_SRR_RFR | UART_SRR_XFR);
    WRITE_REG(p_uart_reg->SFE, p_uart->retention[5]);
    WRITE_REG(p_uart_reg->STET, p_uart->retention[6]);
    WRITE_REG(p_uart_reg->SRT, p_uart->retention[7]);
#else
    UNUSED(p_uart);
#endif
    return ;
}

#ifdef HAL_PM_ENABLE
__WEAK hal_pm_status_t hal_pm_uart_suspend(uart_handle_t *p_uart)
{
    hal_uart_state_t state = hal_uart_get_state(p_uart);
    if ((state != HAL_UART_STATE_RESET) && (state != HAL_UART_STATE_READY))
    {
        return HAL_PM_ACTIVE;
    }
    else
    {
        hal_uart_suspend_reg(p_uart);
        return HAL_PM_SLEEP;
    }
}

__WEAK void hal_pm_uart_resume(uart_handle_t *p_uart)
{
    hal_uart_resume_reg(p_uart);
}
#endif /* HAL_PM_ENABLE */


/** @} */

/** @} */

/** @defgroup UART_Private_Functions UART Private Functions
  * @{
  */

/**
  * @brief  Handle UART Receive Line Communication Timeout.
  * @param p_uart UART handle.
  * @param  Flag Specifies the UART line flag to check
  * @param  Status Flag status (SET or RESET)
  * @param  Tickstart Tick start value
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
static hal_status_t uart_wait_line_flag_until_timeout(uart_handle_t *p_uart,
                                               uint32_t       flag,
                                               flag_status_t  status,
                                               uint32_t       timeout)
{
    hal_status_t ret = HAL_OK;
    if((HAL_NEVER_TIMEOUT != timeout) && (HAL_MAX_DELAY < timeout))
    {
        return HAL_ERROR;
    }

    HAL_TIMEOUT_INIT();
    uint32_t tickstart = HAL_TIMEOUT_GET_TICK();
    /* Update SystemCoreClock */
    SystemCoreUpdateClock();
    uint32_t timeout_counter = ((SystemCoreClock / 1000U) * timeout);

    /* Wait until flag is set */
    while ((READ_BITS((p_uart->p_instance->LSR), flag)) == (uint32_t)status)
    {
        if(HAL_NEVER_TIMEOUT != timeout)
        {
            if((HAL_TIMEOUT_GET_TICK() - tickstart) > timeout_counter)
            {
                /* Disable THRE, RLS, RDA interrupts */
                __HAL_UART_DISABLE_IT(p_uart, UART_IT_THRE | UART_IT_RLS | UART_IT_RDA);
                #if defined(HAL_STATUS_CHECK)
                uart_set_device_state(p_uart, TX_RX, HAL_UART_STATE_READY);
                #endif
                ret = HAL_TIMEOUT;
                goto EXIT;
            }
        }
    }
EXIT:
    HAL_TIMEOUT_DEINIT();
    return ret;
}

/**
  * @brief  Handle UART FIFO Communication Timeout.
  * @param p_uart UART handle.
  * @param  Flag Specifies the UART FIFO flag to check
  * @param  Status Flag status (SET or RESET)
  * @param  Tickstart Tick start value
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
static hal_status_t uart_wait_fifo_flag_until_timeout(uart_handle_t *p_uart,
                                               uint32_t       flag,
                                               flag_status_t  status,
                                               uint32_t       timeout)
{
    hal_status_t ret = HAL_OK;
    if((HAL_NEVER_TIMEOUT != timeout) && (HAL_MAX_DELAY < timeout))
    {
        return HAL_ERROR;
    }

    HAL_TIMEOUT_INIT();
    uint32_t tickstart = HAL_TIMEOUT_GET_TICK();
    /* Update SystemCoreClock */
    SystemCoreUpdateClock();
    uint32_t timeout_counter = ((SystemCoreClock / 1000U) * timeout);

    /* Wait until flag is set */
    while (READ_BITS((p_uart->p_instance->USR), flag) == (uint32_t)status)
    {
        if(HAL_NEVER_TIMEOUT != timeout)
        {
            if((HAL_TIMEOUT_GET_TICK() - tickstart) > timeout_counter)
            {
                /* Disable THRE, RLS, RDA interrupts */
                __HAL_UART_DISABLE_IT(p_uart, UART_IT_THRE | UART_IT_RLS | UART_IT_RDA);
#if defined(HAL_STATUS_CHECK)
                uart_set_device_state(p_uart, TX_RX, HAL_UART_STATE_READY);
#endif
                ret = HAL_TIMEOUT;
                goto EXIT;
            }
        }
    }
EXIT:
    HAL_TIMEOUT_DEINIT();
    return ret;
}


__STATIC_INLINE void uart_stop_dma_tx(uart_handle_t *p_uart)
{
#if defined(HAL_STATUS_CHECK)
    /* Stop UART DMA Tx request if ongoing */
    if (HAL_UART_STATE_BUSY_TX == p_uart->tx_state)
    {
#endif
        if(ENABLE == p_uart->dma_tx_mode)
        {
            /* Abort the UART DMA Tx channel */
            if (NULL != p_uart->p_dmatx)
            {
                hal_dma_abort(p_uart->p_dmatx);
            }

            p_uart->dma_tx_mode = DISABLE;
            uart_end_tx_transfer(p_uart);
        }
#if defined(HAL_STATUS_CHECK)
    }
#endif
}

__STATIC_INLINE void uart_stop_dma_rx(uart_handle_t *p_uart)
{
#if defined(HAL_STATUS_CHECK)
    /* Stop UART DMA Rx request if ongoing */
    if (HAL_UART_STATE_BUSY_RX == p_uart->rx_state)
    {
#endif
        if(ENABLE == p_uart->dma_rx_mode)
        {
            /* Abort the UART DMA Rx channel */
            if (NULL != p_uart->p_dmarx)
            {
                hal_dma_abort(p_uart->p_dmarx);
            }

            p_uart->dma_rx_mode = DISABLE;
            uart_end_rx_transfer(p_uart);
        }
#if defined(HAL_STATUS_CHECK)
    }
#endif
}

/**
  * @brief  End ongoing Tx transfer on UART peripheral (following error detection or Transmit completion).
  * @param  p_uart UART handle.
  * @retval None
  */
__STATIC_INLINE void uart_end_tx_transfer(uart_handle_t *p_uart)
{
    /* Disable THRE interrupt */
    __HAL_UART_DISABLE_IT(p_uart, UART_IT_THRE);

#if defined(HAL_STATUS_CHECK)
    /* At end of Tx process, restore p_uart->gState to Ready */
    uart_set_device_state(p_uart, TX, HAL_UART_STATE_READY);
#endif
}


/**
  * @brief  End ongoing Rx transfer on UART peripheral (following error detection or Reception completion).
  * @param  p_uart UART handle.
  * @retval None
  */
__STATIC_INLINE void uart_end_rx_transfer(uart_handle_t *p_uart)
{
    /* Disable RLS, RDA interrupts */
    __HAL_UART_DISABLE_IT(p_uart, UART_IT_RLS | UART_IT_RDA);

#if defined(HAL_STATUS_CHECK)
    /* At end of Rx process, restore p_uart->RxState to Ready */
    uart_set_device_state(p_uart, RX, HAL_UART_STATE_READY);
#endif
}

/**
  * @brief DMA UART transmit process complete callback.
  * @param p_dma DMA handle.
  * @retval None
  */
static void uart_dma_transmit_cplt(dma_handle_t *p_dma)
{
    //lint -e9087 Cast a pointer to a different object type is necessary
    uart_handle_t *p_uart = (uart_handle_t*)(p_dma->p_parent);

    p_uart->tx_xfer_count = 0U;

    /* Disable the DMA transfer mode */
    p_uart->dma_tx_mode = DISABLE;

    /* Set UART TX FIFO Threshold to EMPTY to make sure all data were sent out */
    ll_uart_set_tx_fifo_threshold(p_uart->p_instance, LL_UART_TX_FIFO_TH_EMPTY);

    /* Enable the UART THRE interrupt, wait all data was sent out */
    __HAL_UART_ENABLE_IT(p_uart, UART_IT_THRE);

}


/**
  * @brief DMA UART receive process complete callback.
  * @param p_dma DMA handle.
  * @retval None
  */
static void uart_dma_receive_cplt(dma_handle_t *p_dma)
{
    //lint -e9087 Cast a pointer to a different object type is necessary
    uart_handle_t *p_uart = (uart_handle_t*)(p_dma->p_parent);

    p_uart->rx_xfer_count = 0U;

    /* Disable RLS interrupt */
    __HAL_UART_DISABLE_IT(p_uart, UART_IT_RLS);
    if (UART_RECEIVER_TIMEOUT_ENABLE == p_uart->init.rx_timeout_mode)
    {
        /* Disable Received Data Available Interrupt and Character Timeout Interrupt */
        __HAL_UART_DISABLE_IT(p_uart, UART_IT_RDA);
    }

    /* Disable the DMA transfer mode */
    p_uart->dma_rx_mode = DISABLE;

#if defined(HAL_STATUS_CHECK)
    /* At end of Rx process, restore p_uart->RxState to Ready */
    uart_set_device_state(p_uart, RX, HAL_UART_STATE_READY);
#endif

    hal_uart_rx_cplt_callback(p_uart);
}

/**
  * @brief DMA UART communication error callback.
  * @param p_dma DMA handle.
  * @retval None
  */
static void uart_dma_error(dma_handle_t *p_dma)
{
    //lint -e9087 Cast a pointer to a different object type is necessary
    uart_handle_t *p_uart = (uart_handle_t*)(p_dma->p_parent);

#if defined(HAL_STATUS_CHECK)
    /* Stop UART DMA Tx request if ongoing */
    if (HAL_UART_STATE_BUSY_TX == p_uart->tx_state)
    {
#endif
        if(ENABLE == p_uart->dma_tx_mode)
        {
            p_uart->tx_xfer_count = 0U;
            uart_end_tx_transfer(p_uart);
        }
#if defined(HAL_STATUS_CHECK)
    }
#endif

#if defined(HAL_STATUS_CHECK)
    /* Stop UART DMA Rx request if ongoing */
    if (HAL_UART_STATE_BUSY_RX == p_uart->rx_state)
    {
#endif
        if(ENABLE == p_uart->dma_rx_mode)
        {
            p_uart->rx_xfer_count = 0U;
            uart_end_rx_transfer(p_uart);
        }
#if defined(HAL_STATUS_CHECK)
    }
#endif

#if defined(HAL_STATUS_CHECK)
    p_uart->error_code |= HAL_UART_ERROR_DMA;
#endif
    hal_uart_error_callback(p_uart);
}

/**
  * @brief  DMA UART communication abort callback, when initiated by HAL services on Error
  *         (To be called at end of DMA Abort procedure following error occurrence).
  * @param  p_dma DMA handle.
  * @retval None
  */
static void uart_dma_abort_on_error(dma_handle_t *p_dma)
{
    //lint -e9087 Cast a pointer to a different object type is necessary
    uart_handle_t *p_uart = (uart_handle_t*)(p_dma->p_parent);
    p_uart->rx_xfer_count = 0U;
    p_uart->tx_xfer_count = 0U;

    hal_uart_error_callback(p_uart);
}

/**
  * @brief  DMA UART Tx communication abort callback, when initiated by user
  *         (To be called at end of DMA Tx Abort procedure following user abort request).
  * @note   When this callback is executed, User Abort complete call back is called only if no
  *         Abort still ongoing for Rx DMA Handle.
  * @param  p_dma DMA handle.
  * @retval None
  */
static void uart_dma_tx_abort_callback(dma_handle_t *p_dma)
{
    uart_handle_t *p_uart = (uart_handle_t* )(p_dma->p_parent);

    p_uart->p_dmatx->xfer_abort_callback = NULL;

    /* Check if an Abort process is still ongoing */
    if (NULL != p_uart->p_dmarx)
    {
        if (NULL != p_uart->p_dmarx->xfer_abort_callback)
        {
            return;
        }
    }

    /* No Abort process still ongoing : All DMA channels are aborted, call user Abort Complete callback */
    p_uart->tx_xfer_count = 0U;
    p_uart->rx_xfer_count = 0U;

#if defined(HAL_STATUS_CHECK)
    /* Reset errorCode */
    p_uart->error_code = HAL_UART_ERROR_NONE;
#endif
    /* Clear the Error flags in the LSR register */
    ll_uart_clear_line_status_flag(p_uart->p_instance);

    /* Flush the whole TX FIFO (if needed) */
    __HAL_UART_SEND_REQ(p_uart, UART_TXDATA_FLUSH_REQUEST);

#if defined(HAL_STATUS_CHECK)
    /* Restore p_uart->gState and p_uart->RxState to Ready */
    uart_set_device_state(p_uart, TX_RX, HAL_UART_STATE_READY);
#endif

    /* Call user Abort complete callback */
    hal_uart_abort_cplt_callback(p_uart);
}


/**
  * @brief  DMA UART Rx communication abort callback, when initiated by user
  *         (To be called at end of DMA Rx Abort procedure following user abort request).
  * @note   When this callback is executed, User Abort complete call back is called only if no
  *         Abort still ongoing for Tx DMA Handle.
  * @param  p_dma DMA handle.
  * @retval None
  */
static void uart_dma_rx_abort_callback(dma_handle_t *p_dma)
{
    //lint -e9087 Cast a pointer to a different object type is necessary
    uart_handle_t *p_uart = (uart_handle_t* )(p_dma->p_parent);

    p_uart->p_dmarx->xfer_abort_callback = NULL;

    /* Check if an Abort process is still ongoing */
    if (NULL != p_uart->p_dmatx)
    {
        if (NULL != p_uart->p_dmatx->xfer_abort_callback)
        {
            return;
        }
    }

    /* No Abort process still ongoing : All DMA channels are aborted, call user Abort Complete callback */
    p_uart->tx_xfer_count = 0U;
    p_uart->rx_xfer_count = 0U;

#if defined(HAL_STATUS_CHECK)
    /* Reset errorCode */
    p_uart->error_code = HAL_UART_ERROR_NONE;
#endif

    /* Clear the Error flags in the LSR register */
    ll_uart_clear_line_status_flag(p_uart->p_instance);

    /* Discard the received data */
    __HAL_UART_SEND_REQ(p_uart, UART_RXDATA_FLUSH_REQUEST);

#if defined(HAL_STATUS_CHECK)
    /* Restore p_uart->gState and p_uart->RxState to Ready */
    uart_set_device_state(p_uart, TX_RX, HAL_UART_STATE_READY);
#endif

    /* Call user Abort complete callback */
    hal_uart_abort_cplt_callback(p_uart);
}


/**
  * @brief  DMA UART Tx communication abort callback, when initiated by user by a call to
  *         hal_uart_abort_transmit_it API (Abort only Tx transfer)
  *         (This callback is executed at end of DMA Tx Abort procedure following user abort request,
  *         and leads to user Tx Abort Complete callback execution).
  * @param  p_dma DMA handle.
  * @retval None
  */
static void uart_dma_tx_only_abort_callback(dma_handle_t *p_dma)
{
    //lint -e9087 Cast a pointer to a different object type is necessary
    uart_handle_t *p_uart = (uart_handle_t*)(p_dma->p_parent);

    p_uart->tx_xfer_count = 0U;

    /* Flush the whole TX FIFO (if needed) */
    __HAL_UART_SEND_REQ(p_uart, UART_TXDATA_FLUSH_REQUEST);

#if defined(HAL_STATUS_CHECK)
    /* Restore p_uart->gState to Ready */
    uart_set_device_state(p_uart, TX, HAL_UART_STATE_READY);
#endif

    /* Call user Abort complete callback */
    hal_uart_abort_tx_cplt_callback(p_uart);
}

/**
  * @brief  DMA UART Rx communication abort callback, when initiated by user by a call to
  *         hal_uart_abort_receive_it API (Abort only Rx transfer)
  *         (This callback is executed at end of DMA Rx Abort procedure following user abort request,
  *         and leads to user Rx Abort Complete callback execution).
  * @param  p_dma DMA handle.
  * @retval None
  */
static void uart_dma_rx_only_abort_callback(dma_handle_t *p_dma)
{
    //lint -e9087 Cast a pointer to a different object type is necessary
    uart_handle_t *p_uart = ( uart_handle_t* )((dma_handle_t* )p_dma)->p_parent;

    p_uart->rx_xfer_count = 0U;

    /* Clear the Error flags in the LSR register */
    ll_uart_clear_line_status_flag(p_uart->p_instance);

    /* Discard the received data */
    __HAL_UART_SEND_REQ(p_uart, UART_RXDATA_FLUSH_REQUEST);

#if defined(HAL_STATUS_CHECK)
    /* Restore p_uart->RxState to Ready */
    uart_set_device_state(p_uart, RX, HAL_UART_STATE_READY);
#endif

    /* Call user Abort complete callback */
    hal_uart_abort_rx_cplt_callback(p_uart);
}

/**
  * @brief  Send an amount of data in interrupt mode.
  * @note   Function is called under interruption only, once
  *         interruptions have been enabled by hal_uart_transmit_it().
  * @param  p_uart UART handle.
  * @retval HAL status
  */
static void uart_transmit_it(uart_handle_t *p_uart)
{
    uint32_t curxfercnt = UART_TXFIFO_SIZE - ll_uart_get_tx_fifo_level(p_uart->p_instance);

#if defined(HAL_STATUS_CHECK)
    /* Check that a Tx process is ongoing */
    if (HAL_UART_STATE_BUSY_TX == p_uart->tx_state)
    {
#endif
        if (0U == p_uart->tx_xfer_count)
        {
            if (HAL_OK != uart_wait_line_flag_until_timeout(p_uart, UART_FLAG_LINE_TEMT, RESET, HAL_UART_TIMEOUT_DEFAULT_VALUE))
            {
                return ;
            }

            uart_end_tx_transfer(p_uart);
            hal_uart_tx_cplt_callback(p_uart);
        }
        else
        {
            while ((0U != curxfercnt))
            {
                if((0U == p_uart->tx_xfer_count))
                {
                    break;
                }
                ll_uart_transmit_data8(p_uart->p_instance, *p_uart->p_tx_buffer++);
                curxfercnt--;
                p_uart->tx_xfer_count--;
            }

            if (0U == p_uart->tx_xfer_count)
            {
                /* Set UART TX FIFO Threshold to EMPTY to make sure all data were sent out */
                ll_uart_set_tx_fifo_threshold(p_uart->p_instance, LL_UART_TX_FIFO_TH_EMPTY);
            }
        }
        return ;
#if defined(HAL_STATUS_CHECK)
    }
    else
    {
        uart_end_tx_transfer(p_uart);
        return ;
    }
#endif
}

/**
  * @brief  Receive an amount of data in interrupt mode.
  * @note   Function is called under interruption only, once
  *         interruptions have been enabled by hal_uart_receive_it()
  * @param  p_uart UART handle.
  * @param  cto_status Character Timeout status.
  * @retval HAL status
  */
static void uart_receive_it(uart_handle_t *p_uart, flag_status_t cto_status)
{
    uint32_t curxfercnt = ll_uart_get_rx_fifo_level(p_uart->p_instance);

    if ((ENABLE == p_uart->dma_rx_mode) && ((it_status_t)RESET == (it_status_t)cto_status))
    {
        return ;
    }

#if defined(HAL_STATUS_CHECK)
    /* Check that a Rx process is ongoing */
    if (HAL_UART_STATE_BUSY_RX == p_uart->rx_state)
    {
#endif
        if (((it_status_t)RESET == (it_status_t)cto_status) && (1U < curxfercnt))
        {
            curxfercnt--;
        }

        if (ENABLE == p_uart->dma_rx_mode)
        {
            uint16_t count = (uint16_t)ll_dma_get_block_size(p_uart->p_dmarx->p_instance, (uint32_t)p_uart->p_dmarx->channel);
            p_uart->p_rx_buffer += count;
            p_uart->rx_xfer_count -= count;
        }

        //lint -e9007
        while ((0U != curxfercnt) && (0U != p_uart->rx_xfer_count))
        {
            *p_uart->p_rx_buffer++ = ll_uart_receive_data8(p_uart->p_instance);
            curxfercnt--;
            p_uart->rx_xfer_count--;
        }

        if (((uint32_t)0U == p_uart->rx_xfer_count) || ((flag_status_t)SET == (flag_status_t)cto_status))
        {
            if (ENABLE == p_uart->dma_rx_mode)
            {
                uart_stop_dma_rx(p_uart);
            }
            else
            {
                uart_end_rx_transfer(p_uart);
            }

            hal_uart_rx_cplt_callback(p_uart);
        }

        return ;
#if defined(HAL_STATUS_CHECK)
    }
    else
    {
        return ;
    }
#endif
}

/**
 ****************************************************************************************
 * @brief  Set the uart bit of g_devices_state.
 ****************************************************************************************
 */
static void uart_set_device_state(uart_handle_t *p_uart, hal_uart_flag_t tx_rx_flag, hal_uart_state_t state)
{
    if(tx_rx_flag == TX)
    {
        p_uart->tx_state = state;
    }
    else if(tx_rx_flag == RX)
    {
        p_uart->rx_state = state;
    }
    else
    {
        p_uart->tx_state = state;
        p_uart->rx_state = state;
    }
}

/**
 ****************************************************************************************
 * @brief  Get device number (PERIPH_DEVICE_NUM_UART0~PERIPH_DEVICE_NUM_UART5) from uart_handle
 ****************************************************************************************
 */
/** @} */

#endif /* HAL_UART_MODULE_ENABLED */

/** @} */

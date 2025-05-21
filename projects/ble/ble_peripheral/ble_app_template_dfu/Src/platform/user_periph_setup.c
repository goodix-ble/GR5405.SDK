/**
 *****************************************************************************************
 *
 * @file user_periph_setup.c
 *
 * @brief  User Periph Init Function Implementation.
 *
 *****************************************************************************************
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
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "user_periph_setup.h"
#include "gr_includes.h"
#include "hal_flash.h"
#include "custom_config.h"
#include "user_app.h"
#include "app_log.h"
#include "app_assert.h"
#include "dfu_port.h"
#include "board_SK.h"
#include "app_uart_dma.h"
#include "app_timer.h"

/*
 * DEFINE
 *****************************************************************************************
 */
#define DFU_UART_RX_BUFF_SIZE  0x400
#define DFU_UART_TX_BUFF_SIZE  0x400

/*
 * NOTE:
 * To avoid UART RX overflow, DMA mode can be used.
 * In DMA mode, a double-buffer mechanism is employed, where DMA only stops and immediately restarts when switching buffers.
 */
#define DFU_UART_DMA_MODE_EN   1  // 1: DMA mode, 0: interrupt mode

#if DFU_UART_DMA_MODE_EN
#define DFU_UART_TIMER_PERIOD  10 // Unit: ms
#endif

#if defined(SOC_GR5515) || defined(SOC_GR5X25) || defined(SOC_GR5526)
#define DFU_FW_SAVE_ADDR       (FLASH_START_ADDR + 0x60000)
#else
#define DFU_FW_SAVE_ADDR       (FLASH_START_ADDR + 0x40000)
#endif

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**@brief Bluetooth device address. */
static const uint8_t  s_bd_addr[SYS_BD_ADDR_LEN] = {0x20, 0xaa, 0xcf, 0x3e, 0xcb, 0xea};
static app_uart_params_t s_dfu_uart_param;
static uint8_t s_dfu_uart_tx_buffer[DFU_UART_TX_BUFF_SIZE];
static uint8_t s_dfu_uart_rx_buffer[DFU_UART_RX_BUFF_SIZE];
#if DFU_UART_DMA_MODE_EN
static uint8_t s_dfu_uart_rx_buffer_alt[DFU_UART_RX_BUFF_SIZE];
static app_timer_id_t s_dfu_uart_timer_id;
static uint8_t *s_active_rx_buffer = s_dfu_uart_rx_buffer;
#endif

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
static void dfu_program_start_callback(void);
static void dfu_programing_callback(uint8_t pro);
static void dfu_program_end_callback(uint8_t status);
#if DFU_UART_DMA_MODE_EN
static void dfu_uart_rx_callback(uint8_t *p_data, uint32_t size);
#endif

static dfu_pro_callback_t dfu_pro_call =
{
    .dfu_program_start_callback = dfu_program_start_callback,
    .dfu_programing_callback = dfu_programing_callback,
    .dfu_program_end_callback = dfu_program_end_callback,
};

static void dfu_program_start_callback(void)
{
    APP_LOG_DEBUG("Dfu start program");
}

static void dfu_programing_callback(uint8_t pro)
{
    APP_LOG_DEBUG("Dfu programing---%d%%", pro);
}

static void dfu_program_end_callback(uint8_t status)
{
    APP_LOG_DEBUG("Dfu program end");
    if (0x01 == status)
    {
        APP_LOG_DEBUG("status: successful");
    }
    else
    {
        APP_LOG_DEBUG("status: error");
    }
}

static void dfu_uart_evt_handler(app_uart_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case APP_UART_EVT_TX_CPLT:
            break;

        case APP_UART_EVT_RX_DATA:
        #if !DFU_UART_DMA_MODE_EN
            dfu_uart_receive_data_process(s_dfu_uart_rx_buffer, p_evt->data.size);
            app_uart_receive_async(APP_UART1_ID, s_dfu_uart_rx_buffer, DFU_UART_RX_BUFF_SIZE);
        #endif
            break;

        case APP_UART_EVT_ERROR:
            APP_LOG_WARNING("UART EVT ERR:%x", p_evt->data.error_code);
        #if DFU_UART_DMA_MODE_EN
            s_active_rx_buffer = s_dfu_uart_rx_buffer;
            app_uart_dma_receive_async(APP_UART1_ID, s_active_rx_buffer, DFU_UART_RX_BUFF_SIZE);
        #else
            app_uart_receive_async(APP_UART1_ID, s_dfu_uart_rx_buffer, DFU_UART_RX_BUFF_SIZE);
        #endif
            break;

        default:
            break;
    }
}

#if DFU_UART_DMA_MODE_EN
/**
 * @brief Stops and restarts the UART DMA reception and switches the RX buffer.
 *
 * This function stops the current UART DMA reception, switches the RX buffer to the alternate buffer,
 * and starts a new DMA reception.
 *
 * @param[in] p_uart_params Pointer to the UART parameters structure containing the UART configuration.
 *
 * @return The size of the data received before stopping the DMA reception.
 *
 * @note To avoid UART RX FIFO overflow, DMA restart needs to be fast and cannot be interrupted.
 *       The interrupt is disabled for about 66us at SYSTEMCLOCK 64MHz.
 *
 */
SECTION_RAM_CODE static uint32_t uart_restart_dma_rx(app_uart_params_t *p_uart_params)
{
    uint16_t ret = !APP_DRV_SUCCESS;
    hal_status_t status = HAL_ERROR;
    uart_handle_t *p_uart = &(p_uart_params->uart_dev.handle);
    uint32_t rx_size = 0;

    GLOBAL_EXCEPTION_DISABLE();
    /* Stop UART DMA Rx request if ongoing */
    if ((HAL_UART_STATE_BUSY_RX == p_uart->rx_state) && (ENABLE == p_uart->dma_rx_mode))
    {
        /* Suspend the DMA channel to disable the UART DMA Rx request */
        ll_dma_suspend_channel(p_uart->p_dmarx->p_instance, p_uart->p_dmarx->channel);
        while (!ll_dma_is_empty_fifo(p_uart->p_dmarx->p_instance, p_uart->p_dmarx->channel)){}
        rx_size = ll_dma_get_block_size(p_uart->p_dmarx->p_instance, p_uart->p_dmarx->channel);
        status = hal_dma_abort(p_uart->p_dmarx);
        p_uart->dma_rx_mode = DISABLE;
        p_uart->rx_state = HAL_UART_STATE_READY;
        // Switch the active RX buffer between the primary and alternate buffers
        s_active_rx_buffer = (s_active_rx_buffer == s_dfu_uart_rx_buffer) ? s_dfu_uart_rx_buffer_alt : s_dfu_uart_rx_buffer;
        ret = app_uart_dma_receive_async(p_uart_params->id, s_active_rx_buffer, DFU_UART_RX_BUFF_SIZE);
    }
    GLOBAL_EXCEPTION_ENABLE();

    APP_ASSERT_CHECK(HAL_OK == status);
    APP_ASSERT_CHECK(APP_DRV_SUCCESS == ret);
    return rx_size;
}

/**
 * @brief Checks if UART RX data is available.
 *
 * @param[in] p_uart_params Pointer to the UART parameters structure, which contains
 *                          the DMA configuration for the RX channel.
 *
 * @return true  If there is data available in the UART RX buffer.
 * @return false If there is no data available in the UART RX buffer.
 */
static bool uart_rx_data_available(app_uart_params_t *p_uart_params)
{
    if (ll_dma_get_block_size(p_uart_params->dma_cfg.rx_dma_instance, p_uart_params->dma_cfg.rx_dma_channel))
    {
        return true;
    }
    return false;
}

/**
 * @brief DFU UART RX Timer handler.
 *
 * This function is triggered when the DFU app timer timeout occurs. It checks if there
 * is data available in the UART RX buffer and call dfu_uart_rx_callback.
 *
 * @param[in] p_arg Unused parameter.
 */
static void dfu_uart_rx_timeout_handler(void *p_arg)
{
    if (uart_rx_data_available(&s_dfu_uart_param))
    {
        uint32_t rx_size = uart_restart_dma_rx(&s_dfu_uart_param);
        uint8_t *p_rx_buffer = (s_active_rx_buffer == s_dfu_uart_rx_buffer) ? s_dfu_uart_rx_buffer_alt : s_dfu_uart_rx_buffer;
        dfu_uart_rx_callback(p_rx_buffer, rx_size);
    }
}

/**
 * @brief Starts the UART rx and initializes a timer to check regularly whether data is received.
 *
 */
void dfu_uart_rx_start(void)
{
    app_uart_dma_receive_async(APP_UART1_ID, s_active_rx_buffer, DFU_UART_RX_BUFF_SIZE);
    app_timer_create(&s_dfu_uart_timer_id, ATIMER_REPEAT, dfu_uart_rx_timeout_handler);
    app_timer_start(s_dfu_uart_timer_id, DFU_UART_TIMER_PERIOD, NULL);
}

/**
 * @brief Stops the DFU UART reception.
 *
 * @note To go to sleep, stop the UART asynchronous reception.
 */
void dfu_uart_rx_stop(void)
{
    app_timer_delete(&s_dfu_uart_timer_id);
    app_uart_abort_receive(APP_UART1_ID);
}

/**
 * @brief UART RX callback function.
 *
 * This function is called when data is received over UART.
 *
 * @param[in] p_data Pointer to the buffer containing the received data.
 * @param[in] size   Size of the received data in bytes.
 */
static void dfu_uart_rx_callback(uint8_t *p_data, uint32_t size)
{
    dfu_uart_receive_data_process(p_data, size);
}
#endif //DFU_UART_DMA_MODE_EN

static void dfu_uart_init(void)
{
    app_uart_tx_buf_t uart_buffer;

    uart_buffer.tx_buf       = s_dfu_uart_tx_buffer;
    uart_buffer.tx_buf_size  = DFU_UART_TX_BUFF_SIZE;

    s_dfu_uart_param.id                   = APP_UART1_ID;
    s_dfu_uart_param.init.baud_rate       = APP_UART1_BAUDRATE;
    s_dfu_uart_param.init.data_bits       = UART_DATABITS_8;
    s_dfu_uart_param.init.stop_bits       = UART_STOPBITS_1;
    s_dfu_uart_param.init.parity          = UART_PARITY_NONE;
    s_dfu_uart_param.init.hw_flow_ctrl    = UART_HWCONTROL_NONE;
#if DFU_UART_DMA_MODE_EN
    s_dfu_uart_param.init.rx_timeout_mode = UART_RECEIVER_TIMEOUT_DISABLE;
#else
    s_dfu_uart_param.init.rx_timeout_mode = UART_RECEIVER_TIMEOUT_ENABLE;
#endif
    s_dfu_uart_param.pin_cfg.rx.type      = APP_UART1_RX_IO_TYPE;
    s_dfu_uart_param.pin_cfg.rx.pin       = APP_UART1_RX_PIN;
    s_dfu_uart_param.pin_cfg.rx.mux       = APP_UART1_RX_PINMUX;
    s_dfu_uart_param.pin_cfg.rx.pull      = APP_UART1_RX_PULL;
    s_dfu_uart_param.pin_cfg.tx.type      = APP_UART1_TX_IO_TYPE;
    s_dfu_uart_param.pin_cfg.tx.pin       = APP_UART1_TX_PIN;
    s_dfu_uart_param.pin_cfg.tx.mux       = APP_UART1_TX_PINMUX;
    s_dfu_uart_param.pin_cfg.tx.pull      = APP_UART1_TX_PULL;

    app_uart_init(&s_dfu_uart_param, dfu_uart_evt_handler, &uart_buffer);

#if DFU_UART_DMA_MODE_EN
    s_dfu_uart_param.dma_cfg.tx_dma_instance = DMA0;
    s_dfu_uart_param.dma_cfg.rx_dma_instance = DMA0;
    s_dfu_uart_param.dma_cfg.tx_dma_channel  = DMA_Channel0;
    s_dfu_uart_param.dma_cfg.rx_dma_channel  = DMA_Channel1;
    app_uart_dma_init(&s_dfu_uart_param);
    dfu_uart_rx_start();
#else
    app_uart_receive_async(APP_UART1_ID, s_dfu_uart_rx_buffer, DFU_UART_RX_BUFF_SIZE);
#endif
}

static void uart_send_data(uint8_t *data, uint16_t size)
{
#if DFU_UART_DMA_MODE_EN
    app_uart_dma_transmit_async(APP_UART1_ID, data, size);
#else
    app_uart_transmit_async(APP_UART1_ID, data, size);
#endif
}


/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void app_periph_init(void)
{
    SYS_SET_BD_ADDR(s_bd_addr);
    dfu_uart_init();
    board_init();
    pwr_mgmt_mode_set(PMR_MGMT_ACTIVE_MODE);
    dfu_port_init(uart_send_data, DFU_FW_SAVE_ADDR, &dfu_pro_call);
}

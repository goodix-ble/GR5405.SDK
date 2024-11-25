/**
 *****************************************************************************************
 *
 * @file main.c
 *
 * @brief main function Implementation.
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
#include "user_app.h"
#include "user_periph_setup.h"
#include "gr_includes.h"
#include "scatter_common.h"
#include "flash_scatter_config.h"
#include "custom_config.h"
#include "patch.h"
#include "app_log.h"
#include "app_uart2lin.h"
#include "app_queue.h"
#include "gr5405_sys_sdk.h"

/*
 * DEFINE
 *****************************************************************************************
 */
#define CMD_LIN_SEND_MAC              (0x02U)
#define CMD_LIN_GET_DILIN_IO          (0x03U)
#define CMD_LIN_GET_RSSI              (0x04U)
#define CMD_LIN_CLOSE_SWD             (0x05U)
#define CMD_LIN_OPEN_SWD              (0x06U)

#define DATA_LEN_CMD_LIN_SEND_MAC     (7U)
#define DATA_LEN_CMD_LIN_GET_DILIN_IO (1U)
#define DATA_LEN_CMD_LIN_GET_RSSI     (1U)
#define DATA_LEN_CMD_LIN_CLOSE_SWD    (1U)
#define DATA_LEN_CMD_LIN_OPEN_SWD     (1U)

#define MAX_DATA_LEN_LIN              (8U)

#define MAX_NUM_RSSI_CACHE            (2U)

typedef struct
{
    uint8_t          rssi;             /**< RSSI. */
} gwcp_rssi_element_t;

#ifdef SOC_GR5405
#ifdef DK_ANCHOR_BOARD_A
#define DILIN_IO_PIN_TYPE        APP_IO_TYPE_AON
#define DILIN_IO_1_PIN           APP_IO_PIN_5
#define DILIN_IO_2_PIN           APP_IO_PIN_6
#define DILIN_IO_3_PIN           APP_IO_PIN_7
#define VPUP_CTRL_PIN_TYPE       APP_IO_TYPE_AON
#define VPUP_CTRL_PIN            APP_IO_PIN_2
#define LIN_UART_ID              APP_UART_ID_1
#define LIN_TX_PIN_TYPE          APP_IO_TYPE_AON
#define LIN_TX_PIN_MUX           APP_IO_MUX_11
#define LIN_TX_PIN               APP_IO_PIN_1
#define LIN_RX_PIN_TYPE          APP_IO_TYPE_AON
#define LIN_RX_PIN_MUX           APP_IO_MUX_12
#define LIN_RX_PIN               APP_IO_PIN_0
#define LIN_SLP_PIN_TYPE         APP_IO_TYPE_AON
#define LIN_SLP_PIN_PIN          APP_IO_PIN_4
#else
#define DILIN_IO_PIN_TYPE        APP_IO_TYPE_MSIO
#define DILIN_IO_1_PIN           APP_IO_PIN_3
#define DILIN_IO_2_PIN           APP_IO_PIN_4
#define DILIN_IO_3_PIN           APP_IO_PIN_5
#define LIN_UART_ID              APP_UART_ID_1
#define LIN_TX_PIN_TYPE          APP_IO_TYPE_AON
#define LIN_TX_PIN_MUX           APP_IO_MUX_11
#define LIN_TX_PIN               APP_IO_PIN_3
#define LIN_RX_PIN_TYPE          APP_IO_TYPE_AON
#define LIN_RX_PIN_MUX           APP_IO_MUX_12
#define LIN_RX_PIN               APP_IO_PIN_2
#endif
#else
#define DILIN_IO_PIN_TYPE        APP_IO_TYPE_GPIOA
#define DILIN_IO_1_PIN           APP_IO_PIN_5
#define DILIN_IO_2_PIN           APP_IO_PIN_6
#define DILIN_IO_3_PIN           APP_IO_PIN_7
#define LIN_UART_ID              APP_UART_ID_0
#define LIN_TX_PIN_TYPE          APP_IO_TYPE_GPIOA
#define LIN_TX_PIN_MUX           APP_IO_MUX_1
#define LIN_TX_PIN               APP_IO_PIN_11
#define LIN_RX_PIN_TYPE          APP_IO_TYPE_GPIOA
#define LIN_RX_PIN_MUX           APP_IO_MUX_1
#define LIN_RX_PIN               APP_IO_PIN_10
#endif
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t s_uart2lin_rx_data[MAX_DATA_LEN_LIN] = {0};

static app_queue_t s_rssi_cache_queue;
static uint8_t s_rssi_queue_buffer[MAX_NUM_RSSI_CACHE];

static uint8_t s_swd_op_result = 1;

/*
 * LOCAL FUNCTION
 *****************************************************************************************
 */
static void dilin_io_init(void)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    io_init.pull = APP_IO_NOPULL;
    io_init.mode = APP_IO_MODE_INPUT;
    io_init.pin  = DILIN_IO_1_PIN | DILIN_IO_2_PIN | DILIN_IO_3_PIN;
    io_init.mux  = APP_IO_MUX;

    uint16_t ret = app_io_init(DILIN_IO_PIN_TYPE, &io_init);
    if (ret != APP_DRV_SUCCESS)
    {
        APP_LOG_ERROR("DILIN_IO init failed");
    }

#ifdef DK_ANCHOR_BOARD_A
    io_init.mode = APP_IO_MODE_OUTPUT;
    io_init.pin = VPUP_CTRL_PIN;
    ret = app_io_init(VPUP_CTRL_PIN_TYPE, &io_init);
    if (ret != APP_DRV_SUCCESS)
    {
        APP_LOG_ERROR("VPUP_CTRL_IO init failed");
    }
    app_io_write_pin(VPUP_CTRL_PIN_TYPE, VPUP_CTRL_PIN, APP_IO_PIN_SET);
#endif
}

// io_bitmap : x x x x x DILIN_1_IO_STATE DILIN_2_IO_STATE DILIN_3_IO_STATE
static uint8_t get_dilin_io(void)
{
    uint8_t io_bitmap = 0;

    app_io_pin_state_t pin_val;
    uint8_t temp = 0;

    // DILIN_1
    pin_val = app_io_read_pin(DILIN_IO_PIN_TYPE, DILIN_IO_1_PIN);
    temp = (pin_val == APP_IO_PIN_SET) ? 1 : 0;
    io_bitmap = io_bitmap | (temp << 2);

    // DILIN_2
    pin_val = app_io_read_pin(DILIN_IO_PIN_TYPE, DILIN_IO_2_PIN);
    temp = (pin_val == APP_IO_PIN_SET) ? 1 : 0;
    io_bitmap = io_bitmap | (temp << 1);

    // DILIN_3
    pin_val = app_io_read_pin(DILIN_IO_PIN_TYPE, DILIN_IO_3_PIN);
    temp = (pin_val == APP_IO_PIN_SET) ? 1 : 0;
    io_bitmap = io_bitmap | temp;

    return io_bitmap;
}

static void rssi_cache_queue_init(void)
{
    if (SDK_SUCCESS != app_queue_init(&s_rssi_cache_queue, s_rssi_queue_buffer, MAX_NUM_RSSI_CACHE / sizeof(gwcp_rssi_element_t), sizeof(gwcp_rssi_element_t)))
    {
        APP_LOG_ERROR("rssi queue init fail!!");
    }
}

static uint8_t rssi_cache_queue_pop(void)
{
    gwcp_rssi_element_t rssi_element;
    if (SDK_SUCCESS != app_queue_pop(&s_rssi_cache_queue, &rssi_element))
    {
        // There is currently no RSSI report.
        return 0x7F;
    }
    else
    {
        return rssi_element.rssi;
    }
}

static void uart2lin_config(void)
{
    app_uart2lin_init_t uart2lin_config;

    uart2lin_config.uart_id = LIN_UART_ID;
    uart2lin_config.rx_response_timeout = 10; // Unit: ms.
    uart2lin_config.baudrate = APP_UART2LIN_BAUD_19200;
    uart2lin_config.lin_mode = APP_UART2LIN_MODE_SLAVE;
    // When using software timers, it will increase the time for LIN response, but it can save one hardware timer.
    uart2lin_config.timer_type = APP_UART2LIN_SOFTWARE_TIMER;
    uart2lin_config.checksum_type = APP_UART2LIN_CHECK_ENHANCE_SUPPORT;
    uart2lin_config.brk_len = APP_UART2LIN_BRK_LEN_13;
    uart2lin_config.pin_cfg.tx.pin = LIN_TX_PIN;
    uart2lin_config.pin_cfg.tx.mux = LIN_TX_PIN_MUX;
    uart2lin_config.pin_cfg.tx.type = LIN_TX_PIN_TYPE;
    uart2lin_config.pin_cfg.tx.pull = APP_IO_NOPULL;
    uart2lin_config.pin_cfg.rx.pin = LIN_RX_PIN;
    uart2lin_config.pin_cfg.rx.mux = LIN_RX_PIN_MUX;
    uart2lin_config.pin_cfg.rx.type = LIN_RX_PIN_TYPE;
    uart2lin_config.pin_cfg.rx.pull = APP_IO_NOPULL;
    uart2lin_config.rx_dma_instance = DMA0;
    uart2lin_config.rx_dma_channel = DMA_Channel1;

    APP_LOG_INFO("LIN BAUDRATE: %d", uart2lin_config.baudrate);
    APP_LOG_INFO("LIN CHECKSUM TYPE: %s", (uart2lin_config.checksum_type == APP_UART2LIN_CHECK_ENHANCE_SUPPORT) ? "ENHANCE" : "CALSSIC");

    if (!app_uart2lin_init(&uart2lin_config))
    {
        APP_LOG_INFO("uart2lin init fail!!!");
    }

#ifdef DK_ANCHOR_BOARD_A
    // LIN transceiver SLP pin pulled high.
    app_io_init_t slp_io_init = APP_IO_DEFAULT_CONFIG;
    slp_io_init.pull = APP_IO_PULLUP;
    slp_io_init.mode = APP_IO_MODE_OUTPUT;
    slp_io_init.pin  = LIN_SLP_PIN_PIN;
    slp_io_init.mux  = APP_IO_MUX;
    if (APP_DRV_SUCCESS != app_io_init(LIN_SLP_PIN_TYPE, &slp_io_init))
    {
        APP_LOG_ERROR("SLP pin init failed");
    }
    app_io_write_pin(LIN_SLP_PIN_TYPE, LIN_SLP_PIN_PIN, APP_IO_PIN_SET);
#endif

    APP_LOG_INFO("uart2lin init success!");

    (void)app_uart2lin_rx_head();
}

/*
 * CALLBACK FUNCTION
 *****************************************************************************************
 */
void app_uart2lin_rx_head_cb(app_uart2lin_env_t *p_uart2lin)
{
    /*
    * Note: In the rx_head_cb function, log printing should not be performed, as it will block data transmission.
    * If logging for debugging purposes needs to be added,
    * the bsp_uart_send function in board_SK.c should be modified from app_uart_transmit_sync(APP_UART_ID, p_data, length, 1000)
    * to app_uart_transmit_async(APP_UART_ID, p_data, length), thus changing it to an asynchronous printing mode.
    */
    switch (p_uart2lin->rx_pid)
    {
        case CMD_LIN_SEND_MAC:
            {
                (void)app_uart2lin_rx_response(s_uart2lin_rx_data, DATA_LEN_CMD_LIN_SEND_MAC);
            }
            break;
        case CMD_LIN_GET_DILIN_IO:
            {
                (void)app_uart2lin_rx_response(s_uart2lin_rx_data, DATA_LEN_CMD_LIN_GET_DILIN_IO);
                uint8_t dilin_io_state = get_dilin_io();
                (void)app_uart2lin_tx_response(&dilin_io_state, DATA_LEN_CMD_LIN_GET_DILIN_IO);
            }
            break;
        case CMD_LIN_GET_RSSI:
            {
                (void)app_uart2lin_rx_response(s_uart2lin_rx_data, DATA_LEN_CMD_LIN_GET_RSSI);
                uint8_t rssi_data = rssi_cache_queue_pop();
                (void)app_uart2lin_tx_response(&rssi_data, DATA_LEN_CMD_LIN_GET_RSSI);
            }
            break;
        case CMD_LIN_CLOSE_SWD:
            {
                (void)app_uart2lin_rx_response(s_uart2lin_rx_data, DATA_LEN_CMD_LIN_CLOSE_SWD);
                // Close SWD
                sys_swd_disable();
                (void)app_uart2lin_tx_response(&s_swd_op_result, DATA_LEN_CMD_LIN_CLOSE_SWD);
            }
            break;
        case CMD_LIN_OPEN_SWD:
            {
                (void)app_uart2lin_rx_response(s_uart2lin_rx_data, DATA_LEN_CMD_LIN_OPEN_SWD);
                // Open SWD
                sys_swd_enable();
                (void)app_uart2lin_tx_response(&s_swd_op_result, DATA_LEN_CMD_LIN_OPEN_SWD);
            }
            break;
        default:
            break;
    }
}

void app_uart2lin_tx_response_cb(app_uart2lin_env_t *p_uart2lin)
{
    (void)app_uart2lin_rx_head();
}

void app_uart2lin_rx_response_cb(app_uart2lin_env_t *p_uart2lin)
{
    APP_LOG_INFO("rx response success: ");
    APP_LOG_HEX_DUMP(s_uart2lin_rx_data, p_uart2lin->rx_len);
    if (CMD_LIN_SEND_MAC == p_uart2lin->rx_pid)
    {
        ble_start_scan_test(s_uart2lin_rx_data[0], &s_uart2lin_rx_data[1]);
    }
    (void)app_uart2lin_rx_head();
}

void app_uart2lin_error_cb(app_uart2lin_env_t *p_uart2lin)
{
    APP_LOG_ERROR("uart2lin error , error_code = 0x%x", p_uart2lin->error_code);
    (void)app_uart2lin_rx_head();
}

/*
 * GLOBAL FUNCTION
 *****************************************************************************************
 */
/**@brief Stack global variables for Bluetooth protocol stack. */
STACK_HEAP_INIT(heaps_table);

bool rssi_cache_queue_push(uint8_t rssi)
{
    sdk_err_t err = 0;
    GLOBAL_EXCEPTION_DISABLE();
    gwcp_rssi_element_t rssi_element;
    // Ensure only the latest RSSI data in the queue.
    (void) app_queue_pop(&s_rssi_cache_queue, &rssi_element);

    rssi_element.rssi = rssi;
    err = app_queue_push(&s_rssi_cache_queue, &rssi_element);
    GLOBAL_EXCEPTION_ENABLE();
    return (SDK_SUCCESS == err);
}

int main (void)
{
    // Initialize user peripherals.
    app_periph_init();

    dilin_io_init();
    rssi_cache_queue_init();
    uart2lin_config();

    // Initialize ble stack.
    ble_stack_init(ble_evt_handler, &heaps_table);

    // loop
    while (1)
    {
    }
}

/**
 *****************************************************************************************
 *
 * @file main.c
 *
 * @brief main function Implementation.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2024 GOODIX
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
#include "scatter_common.h"
#include "flash_scatter_config.h"
#include "custom_config.h"
#include "patch.h"
#include "app_log.h"
#include "app_assert.h"
#include "app_uart2lin.h"

/*
 * DEFINE
 *****************************************************************************************
 */
#define MASTER_NODE_ID                (0x01U)
#define SLAVE1_NODE_ID                (0x02U)

#define CMD_LIN_MASTER_TO_SLAVE1      (0x02U)
#define CMD_LIN_SLAVE1_TO_MASTER      (0x12U)

#define MAX_DATA_LEN_LIN              (8U)

// The values of DATA_LEN_LIN_MASTER_TO_SLAVE1 and DATA_LEN_LIN_SLAVE1_TO_MASTER are less than or equal to 8 (the maximum length of LIN data segments).
#define DATA_LEN_LIN_MASTER_TO_SLAVE1 (8U)
#define DATA_LEN_LIN_SLAVE1_TO_MASTER (6U)

#define LIN_TRANS_CYCLE               (10)  // ms

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t s_uart2lin_tx_data[MAX_DATA_LEN_LIN] = {0};
static uint8_t s_uart2lin_rx_data[MAX_DATA_LEN_LIN] = {0};
static uint8_t s_local_node_id = 0;
/*
 * LOCAL FUNCTION
 *****************************************************************************************
 */
static uint8_t get_node_id(void)
{
    uint8_t node_id = 0;

    app_io_pin_state_t pin_val;
    uint8_t temp = 0;
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    io_init.pull = APP_IO_PULLUP;
    io_init.mode = APP_IO_MODE_INPUT;
    io_init.pin  = APP_IO_PIN_3 | APP_IO_PIN_4 | APP_IO_PIN_5;
    io_init.mux  = APP_IO_MUX;

    uint16_t ret = app_io_init(APP_IO_TYPE_MSIO, &io_init);
    if (ret != APP_DRV_SUCCESS)
    {
        APP_LOG_DEBUG("APP_GPIO_PIN345 init failed");
    }

    pin_val = app_io_read_pin(APP_IO_TYPE_MSIO, APP_IO_PIN_3);
    temp = (pin_val == APP_IO_PIN_SET) ? 0 : 1;
    node_id = node_id | (temp << 2);

    pin_val = app_io_read_pin(APP_IO_TYPE_MSIO, APP_IO_PIN_4);
    temp = (pin_val == APP_IO_PIN_SET) ? 0 : 1;
    node_id = node_id | (temp << 1);

    pin_val = app_io_read_pin(APP_IO_TYPE_MSIO, APP_IO_PIN_5);
    temp = (pin_val == APP_IO_PIN_SET) ? 0 : 1;
    node_id = node_id | temp;

    return node_id;
}

static void uart2lin_config(void)
{
    app_uart2lin_init_t uart2lin_config;

    uart2lin_config.uart_id = APP_UART_ID_1;
    uart2lin_config.rx_response_timeout = 10; // Unit: ms.
    uart2lin_config.baudrate = APP_UART2LIN_BAUD_20000;
    uart2lin_config.lin_mode = (MASTER_NODE_ID == s_local_node_id) ? APP_UART2LIN_MODE_MASTER : APP_UART2LIN_MODE_SLAVE;
    // When using software timers, it will increase the time for LIN response, but it can save one hardware timer.
    uart2lin_config.timer_type = APP_UART2LIN_SOFTWARE_TIMER;
    uart2lin_config.checksum_type = APP_UART2LIN_CHECK_ENHANCE_SUPPORT;
    uart2lin_config.brk_len = APP_UART2LIN_BRK_LEN_13;
    uart2lin_config.pin_cfg.tx.pin = APP_IO_PIN_3;
    uart2lin_config.pin_cfg.tx.mux = APP_IO_MUX_11;
    uart2lin_config.pin_cfg.tx.type = APP_IO_TYPE_AON;
    uart2lin_config.pin_cfg.tx.pull = APP_IO_NOPULL;
    uart2lin_config.pin_cfg.rx.pin = APP_IO_PIN_2;
    uart2lin_config.pin_cfg.rx.mux = APP_IO_MUX_12;
    uart2lin_config.pin_cfg.rx.type = APP_IO_TYPE_AON;
    uart2lin_config.pin_cfg.rx.pull = APP_IO_NOPULL;
    uart2lin_config.rx_dma_instance = DMA0;
    uart2lin_config.rx_dma_channel = DMA_Channel1;

    if (!app_uart2lin_init(&uart2lin_config))
    {
        APP_LOG_INFO("uart2lin init fail!!!");
        APP_ASSERT_CHECK(false);
    }

    if (!app_uart2lin_rx_head())
    {
        APP_LOG_INFO("uart2lin rx fail!!!");
        APP_ASSERT_CHECK(false);
    }
}

static void uart2lin_master_send_head(void)
{
    static uint8_t s_cmd_flag = 0;

    if (0 == s_cmd_flag)
    {
        s_cmd_flag = 1;
        (void)app_uart2lin_tx_head(CMD_LIN_MASTER_TO_SLAVE1);
    }
    else
    {
        s_cmd_flag = 0;
        (void)app_uart2lin_tx_head(CMD_LIN_SLAVE1_TO_MASTER);
    }
}

static void uart2lin_prepare_data(void)
{
    for (uint8_t i = 0; i < MAX_DATA_LEN_LIN; i++)
    {
        s_uart2lin_tx_data[i] = i;
    }
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
        case CMD_LIN_MASTER_TO_SLAVE1:
            {
                (void)app_uart2lin_rx_response(s_uart2lin_rx_data, DATA_LEN_LIN_MASTER_TO_SLAVE1);
                if (MASTER_NODE_ID == s_local_node_id)
                {
                    (void)app_uart2lin_tx_response(s_uart2lin_tx_data, DATA_LEN_LIN_MASTER_TO_SLAVE1);
                }
            }
            break;
        case CMD_LIN_SLAVE1_TO_MASTER:
            {
                (void)app_uart2lin_rx_response(s_uart2lin_rx_data, DATA_LEN_LIN_SLAVE1_TO_MASTER);
                if (SLAVE1_NODE_ID == s_local_node_id)
                {
                    (void)app_uart2lin_tx_response(s_uart2lin_tx_data, DATA_LEN_LIN_SLAVE1_TO_MASTER);
                }
            }
            break;
        default:
            break;
    }
}

void app_uart2lin_tx_response_cb(app_uart2lin_env_t *p_uart2lin)
{
    (void)app_uart2lin_rx_head();
    APP_LOG_INFO("tx response success, pid = 0x%x", p_uart2lin->rx_pid);
}

void app_uart2lin_rx_response_cb(app_uart2lin_env_t *p_uart2lin)
{
    (void)app_uart2lin_rx_head();
    APP_LOG_INFO("rx response success: ");
    APP_LOG_HEX_DUMP(s_uart2lin_rx_data, p_uart2lin->rx_len);
}

void app_uart2lin_error_cb(app_uart2lin_env_t *p_uart2lin)
{
    app_uart2lin_force_rx_head();
    APP_LOG_ERROR("uart2lin error , error_code = 0x%x", p_uart2lin->error_code);
}

/*
 * GLOBAL FUNCTION
 *****************************************************************************************
 */
int main (void)
{
    // Initialize user peripherals.
    app_periph_init();

    s_local_node_id = get_node_id();
    APP_LOG_INFO("s_local_node_id = 0x%x", s_local_node_id);
    uart2lin_prepare_data();
    uart2lin_config();

    // loop
    while (1)
    {
        if (MASTER_NODE_ID == s_local_node_id)
        {
            uart2lin_master_send_head();
            delay_ms(LIN_TRANS_CYCLE);
        }
    }
}

/**
 *****************************************************************************************
 *
 * @file swd_ctrl.c
 *
 * @brief SWD function control.
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

#include "custom_config.h"
#include "swd_ctrl.h"
#include "grx_sys.h"
#if SWD_CTRL_DEBUG_ENABLE
#include "app_log.h"
#endif

#define MAX_RETRIES 10  // Maximum retries of pin level reads

static const char pattern1[PATTERN_LENGTH] = PATTERN1;
static const char pattern2[PATTERN_LENGTH] = PATTERN2;

static void uart_pin_init(void)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    io_init.pin  = SWD_CTRL_UART_RX_PIN;
    io_init.pull = APP_IO_NOPULL;
    io_init.mode = APP_IO_MODE_MUX;
    io_init.mux  = SWD_CTRL_UART_RX_PINMUX;
    app_io_init(SWD_CTRL_UART_RX_IO_TYPE, &io_init);
}

static void gpio_pin_init(void)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    io_init.pin  = SWD_CTRL_UART_RX_PIN;
    io_init.pull = APP_IO_PULLDOWN;
    io_init.mode = APP_IO_MODE_INPUT;
    io_init.mux  = APP_IO_MUX;
    app_io_init(SWD_CTRL_UART_RX_IO_TYPE, &io_init);
}

/**
 *****************************************************************************************
 * @brief If the UART is connected, the RX pin is high level when idle
 *
 *****************************************************************************************
 */
static bool uart_connection_detect(void)
{
    gpio_pin_init();
    for (uint32_t count = 0; count < MAX_RETRIES; count++)
    {
        if (APP_IO_PIN_SET == app_io_read_pin(SWD_CTRL_UART_RX_IO_TYPE, SWD_CTRL_UART_RX_PIN))
        {
            uart_pin_init();
            return true;
        }
    }
    uart_pin_init();
    return false;
}

__WEAK void swd_unlock_wdt_refresh(void)
{
    return;
}

void swd_unlock_process(void)
{
    if (sys_swd_is_enabled())
    {
        SWD_CTRL_LOG("SWD is unlock");
        return;
    }
    SWD_CTRL_LOG("SWD is locked");

#if !APP_LOG_ENABLE || (APP_LOG_PORT != 0)
    // if APP_LOG_ENABLE and APP_LOG_PORT = 0, bsp_uart_init has been called
    bsp_uart_init();
#endif
    if (uart_connection_detect())
    {
        SWD_CTRL_LOG("To RX pattern1");
        bool wait_pattern2 = false;
        char uart_rx_buf[PATTERN_LENGTH*2];
        memset(uart_rx_buf, 0 ,sizeof(uart_rx_buf));
        app_uart_receive_sync(SWD_CTRL_UART_ID, (uint8_t *)uart_rx_buf, sizeof(uart_rx_buf), RX_PATTERN1_TIMEOUT);
        SWD_CTRL_LOG("RX data:");
        SWD_CTRL_LOG_HEX_DUMP(uart_rx_buf, sizeof(uart_rx_buf));
        // The position of PATTERN in rx buffer is uncertain
        for (uint32_t i = 0; i <= (sizeof(uart_rx_buf) - PATTERN_LENGTH); i++)
        {
            if (0 == memcmp(&uart_rx_buf[i], pattern1, PATTERN_LENGTH))
            {
                // Get pattern1, unlock SWD
                SWD_CTRL_LOG("Get pattern1, unlock SWD");
                sys_swd_enable();
                wait_pattern2 = true;
                break;
            }
        }

        while (wait_pattern2)
        {
            swd_unlock_wdt_refresh();
            SWD_CTRL_LOG("To RX pattern2");
            memset(uart_rx_buf, 0 ,sizeof(uart_rx_buf));
            app_uart_receive_sync(SWD_CTRL_UART_ID, (uint8_t *)uart_rx_buf, sizeof(uart_rx_buf), RX_PATTERN1_TIMEOUT);
            SWD_CTRL_LOG("RX data:");
            SWD_CTRL_LOG_HEX_DUMP(uart_rx_buf, sizeof(uart_rx_buf));
            for (uint32_t i = 0; i <= (sizeof(uart_rx_buf) - PATTERN_LENGTH); i++)
            {
                if (0 == memcmp(&uart_rx_buf[i], pattern2, PATTERN_LENGTH))
                {
                    // Get pattern2
                    SWD_CTRL_LOG("Get pattern2");
                #if !APP_LOG_ENABLE || (APP_LOG_PORT != 0)
                    // if APP_LOG_ENABLE and APP_LOG_PORT = 0, uart deinit by other modules
                    app_uart_deinit(SWD_CTRL_UART_ID);
                #endif
                    return;
                }
            }
        }
    }
}

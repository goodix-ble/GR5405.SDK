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
#include <stdio.h>
#include <string.h>
#include "app_log.h"
#include "app_spi.h"
#include "board_SK.h"
#include "app_spi2can.h"

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static app_spi2can_init_t spi2can_init;
static volatile bool spi2can_tx_done_flag = false;
static uint8_t test_can_tx_data[64] = {0};
uint8_t test_can_tx_len[15] = {1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

/*
 * STATIC FUNCTION DEFINITIONS
 ****************************************************************************************
 */
static void spi2can_init_test(void)
{
    spi2can_init.can_mode                 = APP_SPI2CAN_MODE_CAN_FD;
    spi2can_init.brs_mode                 = APP_SPI2CAN_MODE_RATE_SWITCH_ENABLE;
    spi2can_init.retrans_mode             = APP_SPI2CAN_MODE_AUTO_RETRANS_ENABLE;

    spi2can_init.pin_cfg.cs_pin.type      = APP_SPI2CAN_CS_IO_TYPE;
    spi2can_init.pin_cfg.cs_pin.pin       = APP_SPI2CAN_CS_PIN;
    spi2can_init.pin_cfg.cs_pin.mux       = APP_SPI2CAN_CS_PINMUX;
    spi2can_init.pin_cfg.clk_pin.type     = APP_SPI2CAN_CLK_IO_TYPE;
    spi2can_init.pin_cfg.clk_pin.pin      = APP_SPI2CAN_CLK_PIN;
    spi2can_init.pin_cfg.clk_pin.mux      = APP_SPI2CAN_CLK_PINMUX;
    spi2can_init.pin_cfg.mosi_pin.type    = APP_SPI2CAN_MOSI_IO_TYPE;
    spi2can_init.pin_cfg.mosi_pin.pin     = APP_SPI2CAN_MOSI_PIN;
    spi2can_init.pin_cfg.mosi_pin.mux     = APP_SPI2CAN_MOSI_PINMUX;
    spi2can_init.pin_cfg.miso_pin.type    = APP_SPI2CAN_MISO_IO_TYPE;
    spi2can_init.pin_cfg.miso_pin.pin     = APP_SPI2CAN_MISO_PIN;
    spi2can_init.pin_cfg.miso_pin.mux     = APP_SPI2CAN_MISO_PINMUX;
    spi2can_init.pin_cfg.wakeup_pin.type  = APP_SPI2CAN_WKRQ_IO_TYPE;
    spi2can_init.pin_cfg.wakeup_pin.pin   = APP_SPI2CAN_WKRQ_PIN;
    spi2can_init.pin_cfg.wakeup_pin.mux   = APP_SPI2CAN_WKRQ_PINMUX;
    spi2can_init.pin_cfg.int_pin.type     = APP_SPI2CAN_INT_IO_TYPE;
    spi2can_init.pin_cfg.int_pin.pin      = APP_SPI2CAN_INT_PIN;
    spi2can_init.pin_cfg.int_pin.mux      = APP_SPI2CAN_INT_PINMUX;
    spi2can_init.pin_cfg.rst_pin.type     = APP_SPI2CAN_RST_IO_TYPE;
    spi2can_init.pin_cfg.rst_pin.pin      = APP_SPI2CAN_RST_PIN;
    spi2can_init.pin_cfg.rst_pin.mux      = APP_SPI2CAN_RST_PINMUX;

    spi2can_init.normal_bit_baudrate      = APP_SPI2CAN_BAUD_1000K;
    spi2can_init.data_bit_baudrate        = APP_SPI2CAN_BAUD_2000K;

    if (!app_spi2can_init(&spi2can_init))
    {
        APP_LOG_INFO("APP SPI2CAN INIT FAIL!");
    }
}

static void spi2can_transmit_polling_test(void)
{
    for (uint8_t i = 0; i < 64; i++)
    {
        test_can_tx_data[i] = i;
    }

    uint32_t can_id = 0x55;

    for (uint8_t i = 0; i < 15; i++)
    {
        app_spi2can_transmit_polling(can_id + i, test_can_tx_data, test_can_tx_len[i], 100);
        delay_ms(10);
    }
}

static void spi2can_transmit_it_test(void)
{
    for (uint8_t i = 0; i < 64; i++)
    {
        test_can_tx_data[i] = i;
    }

    uint32_t can_id = 0x55;

    for (uint8_t i = 0; i < 15; i++)
    {
        spi2can_tx_done_flag = false;
        app_spi2can_transmit_it(can_id + i, test_can_tx_data, test_can_tx_len[i]);
        while(!spi2can_tx_done_flag)
        {
            // Nothing to do.
        }
        delay_ms(10);
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void app_spi2can_rx_cplt_cb(uint32_t id, uint8_t * p_data, uint8_t size)
{
    APP_LOG_INFO("CAN RX DONE ID = 0x%x.", id);
    APP_LOG_HEX_DUMP(p_data, size);
}

void app_spi2can_tx_cplt_cb(void)
{
    spi2can_tx_done_flag = true;
    APP_LOG_INFO("CAN TX DONE.");
}

void app_spi2can_busoff_restore_cb(void)
{
    APP_LOG_INFO("CAN RESTORE FROM BUSOFF.");
}

void app_spi2can_error_cb(uint32_t error_code)
{
    APP_LOG_INFO("CAN ERROR_CODE = 0x%x", error_code);
}

int main(void)
{
    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*              SPI2CAN tcan4x5x  Demo.               *\r\n");
    printf("******************************************************\r\n");

    spi2can_init_test();
    spi2can_transmit_polling_test();
    spi2can_transmit_it_test();

    while (1)
    {
    }
}

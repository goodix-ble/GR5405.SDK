/**
 ****************************************************************************************
 *
 * @file    app_spi2can.c
 * @author  BSP Team
 * @brief   C file for SPI2CAN.
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2025 GOODIX
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
#include <string.h>
#include "tcan4550_port.h"
#include "app_spi2can.h"
#include "app_gpiote.h"
#include "app_spi.h"
#include "hal_delay.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define SPI_MODULE_CLOCK_PRESCALER             (4U)    /* The SPI CLOCK Freq = Peripheral CLK (64M) / SPI_MODULE_CLOCK_PRESCALER */
#define SPI_MODULE_SOFT_CS_MODE_ENABLE         (1U)    /* Enable SOFT CS MODE */
#define SPI_MODULE_SOFT_CS_MODE_DISABLE        (0U)    /* Disable SOFT CS MODE */

#if SPI_MODULE_CLOCK_PRESCALER == 2u
    #define RX_SAMPLE_DELAY             (1U)
#else
    #define RX_SAMPLE_DELAY             (0U)
#endif

#define SPI2CAN_BAUD_PARM_BRP_INDEX     (0U)
#define SPI2CAN_BAUD_PARM_SEG1_INDEX    (1U)
#define SPI2CAN_BAUD_PARM_SEG2_INDEX    (2U)

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const uint16_t s_can_baud_parmm_list[][3] =
{
 /* brp  seg1  seg2 */
    {10  ,28   ,4},   /* Baud = 125K,  SamplePoint = 87.5% */
    {4   ,28   ,4},   /* Baud = 250K,  SamplePoint = 87.5% */
    {2   ,35   ,5},   /* Baud = 500K,  SamplePoint = 87.5% */
    {1   ,30   ,10},  /* Baud = 1000K, SamplePoint = 75% */
    {1   ,15   ,5},   /* Baud = 2000K, SamplePoint = 75% */
    {1   ,7    ,3},   /* Baud = 4000K, SamplePoint = 70% */
    {1   ,6    ,2},   /* Baud = 5000K, SamplePoint = 75% */
    {1   ,4    ,1},   /* Baud = 8000K, SamplePoint = 80% */
};

static app_spi2can_env_t s_spi2can_env;

static app_spi_params_t s_spi_params =
{
    .id = APP_SPI_ID_MASTER,
    .init = {
        .data_size       = SPI_DATASIZE_8BIT,
        .clock_polarity  = SPI_POLARITY_LOW,
        .clock_phase     = SPI_PHASE_1EDGE,
        .baudrate_prescaler = SPI_MODULE_CLOCK_PRESCALER,
        .ti_mode         = SPI_TIMODE_DISABLE,
        .slave_select    = SPI_SLAVE_SELECT_0,
#ifndef APP_SPI_GR551X_LEGACY
        .rx_sample_delay = RX_SAMPLE_DELAY,
#endif
    },
    .is_soft_cs = SPI_MODULE_SOFT_CS_MODE_DISABLE,
};

/*
 * STATIC FUNCTION DEFINITIONS
 ****************************************************************************************
 */
static void spi_event_callback(app_spi_evt_t *p_evt);
static void wakeup_irq_pin_event_handler(app_io_evt_t *p_evt);
static void int_irq_pin_event_handler(app_io_evt_t *p_evt);

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
__WEAK void app_spi2can_rx_cplt_cb(uint32_t id, uint8_t *p_data, uint8_t size)
{
    UNUSED(id);
    UNUSED(p_data);
    UNUSED(size);
}

__WEAK void app_spi2can_tx_cplt_cb(void)
{
}

__WEAK void app_spi2can_error_cb(uint32_t error_code)
{
    UNUSED(error_code);
}

__WEAK void app_spi2can_busoff_restore_cb(void)
{
}

bool app_spi2can_init(app_spi2can_init_t *p_spi2can_init)
{
    if (NULL == p_spi2can_init)
    {
        return false;
    }

    tcan4550_set_debug_log_level(TCAN_DEBUG_CLOSE);

    /* CS pin */
    s_spi_params.pin_cfg.cs.type = p_spi2can_init->pin_cfg.cs_pin.type;
    s_spi_params.pin_cfg.cs.mux = p_spi2can_init->pin_cfg.cs_pin.mux;
    s_spi_params.pin_cfg.cs.pin = p_spi2can_init->pin_cfg.cs_pin.pin;
    s_spi_params.pin_cfg.cs.mode = APP_IO_MODE_MUX;
    s_spi_params.pin_cfg.cs.pull = APP_IO_PULLUP;
    s_spi_params.pin_cfg.cs.enable = APP_SPI_PIN_DISABLE;
    /* CLK pin */
    s_spi_params.pin_cfg.clk.type = p_spi2can_init->pin_cfg.clk_pin.type;
    s_spi_params.pin_cfg.clk.mux = p_spi2can_init->pin_cfg.clk_pin.mux;
    s_spi_params.pin_cfg.clk.pin = p_spi2can_init->pin_cfg.clk_pin.pin;
    s_spi_params.pin_cfg.clk.mode = APP_IO_MODE_MUX;
    s_spi_params.pin_cfg.clk.pull = APP_IO_PULLUP;
    s_spi_params.pin_cfg.clk.enable = APP_SPI_PIN_ENABLE;
    /* MOSI pin */
    s_spi_params.pin_cfg.mosi.type = p_spi2can_init->pin_cfg.mosi_pin.type;
    s_spi_params.pin_cfg.mosi.mux = p_spi2can_init->pin_cfg.mosi_pin.mux;
    s_spi_params.pin_cfg.mosi.pin = p_spi2can_init->pin_cfg.mosi_pin.pin;
    s_spi_params.pin_cfg.mosi.mode = APP_IO_MODE_MUX;
    s_spi_params.pin_cfg.mosi.pull = APP_IO_PULLUP;
    s_spi_params.pin_cfg.mosi.enable = APP_SPI_PIN_ENABLE;
    /* MISO pin */
    s_spi_params.pin_cfg.miso.type = p_spi2can_init->pin_cfg.miso_pin.type;
    s_spi_params.pin_cfg.miso.mux = p_spi2can_init->pin_cfg.miso_pin.mux;
    s_spi_params.pin_cfg.miso.pin = p_spi2can_init->pin_cfg.miso_pin.pin;
    s_spi_params.pin_cfg.miso.mode = APP_IO_MODE_MUX;
    s_spi_params.pin_cfg.miso.pull = APP_IO_NOPULL;
    s_spi_params.pin_cfg.miso.enable = APP_SPI_PIN_ENABLE;

    if (APP_DRV_SUCCESS != app_spi_init(&s_spi_params, spi_event_callback))
    {
        return false;
    }

    app_io_init_t functional_pin = {0};

    /* CS Pin */
    functional_pin.mode = APP_IO_MODE_OUTPUT;
    functional_pin.mux  = APP_IO_MUX;
    functional_pin.pin  = p_spi2can_init->pin_cfg.cs_pin.pin;
    functional_pin.pull = APP_IO_PULLUP;
    app_io_init(p_spi2can_init->pin_cfg.cs_pin.type, &functional_pin);
    app_io_write_pin(p_spi2can_init->pin_cfg.cs_pin.type, p_spi2can_init->pin_cfg.cs_pin.pin, APP_IO_PIN_SET);

    /* RST Pin */
    functional_pin.mode = APP_IO_MODE_OUTPUT;
    functional_pin.mux  = APP_IO_MUX;
    functional_pin.pin  = p_spi2can_init->pin_cfg.rst_pin.pin;
    functional_pin.pull = APP_IO_PULLDOWN;
    app_io_init(p_spi2can_init->pin_cfg.rst_pin.type, &functional_pin);

    app_gpiote_param_t irq_pin_cfg[2];

    /* WAKEUP Pin */
    irq_pin_cfg[0].type      = p_spi2can_init->pin_cfg.wakeup_pin.type;
    irq_pin_cfg[0].pin       = p_spi2can_init->pin_cfg.wakeup_pin.pin;
    irq_pin_cfg[0].mode      = APP_IO_MODE_IT_FALLING;
    irq_pin_cfg[0].pull      = APP_IO_PULLUP;
    irq_pin_cfg[0].io_evt_cb = wakeup_irq_pin_event_handler;

    /* INT Pin */
    irq_pin_cfg[1].type      = p_spi2can_init->pin_cfg.int_pin.type;
    irq_pin_cfg[1].pin       = p_spi2can_init->pin_cfg.int_pin.pin;
    irq_pin_cfg[1].mode      = APP_IO_MODE_IT_FALLING;
    irq_pin_cfg[1].pull      = APP_IO_PULLUP;
    irq_pin_cfg[1].io_evt_cb = int_irq_pin_event_handler;
    memcpy(&s_spi2can_env.int_pin, &p_spi2can_init->pin_cfg.int_pin, sizeof(app_spi2can_pin_t));

    app_gpiote_init(&irq_pin_cfg[0], 2);

    // disable nINT IRQ, enable when canfd init finished
    if (APP_IO_TYPE_AON == p_spi2can_init->pin_cfg.int_pin.type)
    {
        ll_aon_gpio_disable_it(p_spi2can_init->pin_cfg.int_pin.pin);
    }
    else
    {
        ll_gpio_disable_it(GPIO0, p_spi2can_init->pin_cfg.int_pin.pin);
    }

    /* Reset Device */
    app_io_write_pin(p_spi2can_init->pin_cfg.rst_pin.type, p_spi2can_init->pin_cfg.rst_pin.pin, APP_IO_PIN_SET);
    // Increase the delay to allow the CAN controller to correctly recognize the RST signal.
    delay_ms(10);
    app_io_write_pin(p_spi2can_init->pin_cfg.rst_pin.type, p_spi2can_init->pin_cfg.rst_pin.pin, APP_IO_PIN_RESET);
    // Increase the delay to perform subsequent initialization operations only after the CAN controller has completed the RST.
    delay_ms(10);

    /* Init TCAN4550 */
    tcan4550_init_t tcan4550_param;
    tcan4550_param.can_mode = (tcan4550_mode_t)p_spi2can_init->can_mode;
    tcan4550_param.brs_mode = (tcan4550_brs_mode_t)p_spi2can_init->brs_mode;
    tcan4550_param.retrans_mode = (tcan4550_retrans_mode_t)p_spi2can_init->brs_mode;
    tcan4550_param.normal_timing.brp = s_can_baud_parmm_list[p_spi2can_init->normal_bit_baudrate][SPI2CAN_BAUD_PARM_BRP_INDEX];
    tcan4550_param.normal_timing.seg1 = s_can_baud_parmm_list[p_spi2can_init->normal_bit_baudrate][SPI2CAN_BAUD_PARM_SEG1_INDEX];
    tcan4550_param.normal_timing.seg2 = s_can_baud_parmm_list[p_spi2can_init->normal_bit_baudrate][SPI2CAN_BAUD_PARM_SEG2_INDEX];
    tcan4550_param.data_timing.brp = s_can_baud_parmm_list[p_spi2can_init->data_bit_baudrate][SPI2CAN_BAUD_PARM_BRP_INDEX];
    tcan4550_param.data_timing.seg1 = s_can_baud_parmm_list[p_spi2can_init->data_bit_baudrate][SPI2CAN_BAUD_PARM_SEG1_INDEX];
    tcan4550_param.data_timing.seg2 = s_can_baud_parmm_list[p_spi2can_init->data_bit_baudrate][SPI2CAN_BAUD_PARM_SEG2_INDEX];
    tcan4550_param.can_cb.tx_cplt_callback = app_spi2can_tx_cplt_cb;
    tcan4550_param.can_cb.rx_cplt_callback = app_spi2can_rx_cplt_cb;
    tcan4550_param.can_cb.xfer_err_callback = app_spi2can_error_cb;
    tcan4550_param.can_cb.busoff_restore_calback = app_spi2can_busoff_restore_cb;

    if (tcan4550_init(&tcan4550_param))
    {
        if (APP_IO_TYPE_AON == p_spi2can_init->pin_cfg.int_pin.type)
        {
            NVIC_ClearPendingIRQ(AON_EXT_IRQn);
            ll_aon_gpio_enable_it(p_spi2can_init->pin_cfg.int_pin.pin);
        }
        else
        {
            NVIC_ClearPendingIRQ(EXT0_IRQn);
            ll_gpio_enable_it(GPIO0, p_spi2can_init->pin_cfg.int_pin.pin);
        }
        // Note: This API is called after the SPI and IO initialization,
        //       to clear interference interrupts generated during the SPI2CAN initialization process.
        tcan4550_clear_all_interrupts();

        return true;
    }
    else
    {
        return false;
    }
}

bool app_spi2can_deinit(void)
{
    return true;
}

void app_spi2can_set_standard_filter(app_spi2can_standard_filter_init_t *p_sft_init)
{
     //lint -e740 app_spi2can_standard_filter_init_t is compatible with tcan4550_standard_filter_init_t.
    tcan4550_set_standard_filter((tcan4550_standard_filter_init_t *)p_sft_init);
}

bool app_spi2can_transmit_polling(uint32_t can_id, uint8_t *p_data, uint8_t length, uint32_t timeout)
{
    return tcan4550_transmit_polling(can_id, p_data, length, timeout);
}

bool app_spi2can_transmit_it(uint32_t can_id, uint8_t *p_data, uint8_t length)
{
    return tcan4550_transmit_it(can_id, p_data, length);
}

app_spi2can_busoff_state_t app_spi2can_get_busoff_state(void)
{
    return (app_spi2can_busoff_state_t)tcan4550_get_busoff_state();
}

// Adapt the weak function in tcan_4550_port.
bool spi_recv_word_dis_irq_port(uint8_t op_code, uint16_t address, uint32_t * p_buff, uint32_t words)
{
    uint32_t rx_cnt = 0;

    ll_spi_disable(SPIM);
    ll_spi_set_data_size(SPIM, LL_SPI_DATASIZE_32BIT);
    ll_spi_set_transfer_direction(SPIM, LL_SPI_READ_EEPROM);
    ll_spi_set_receive_size(SPIM, words - 1);
    ll_spi_set_rx_sample_delay(SPIM, 0);
    ll_spi_enable(SPIM);

    uint32_t ia = (op_code << 24UL) | (address << 8UL) | (words & 0xFF);

    GLOBAL_EXCEPTION_DISABLE();

    SPIM->DATA = ia;

    while (rx_cnt < words)
    {
        if (ll_spi_is_active_flag(SPIM, SPI_STAT_RX_FIFO_NE))
        {
            p_buff[rx_cnt] = SPIM->DATA;
            rx_cnt++;
        }
    }

    GLOBAL_EXCEPTION_ENABLE();

    return true;
}

// Adapt the weak function in tcan_4550_port.
bool spi_recv_word_with_irq_port(uint8_t op_code, uint16_t address, uint32_t * p_buff, uint32_t words)
{
    uint32_t rx_cnt = 0;

    ll_spi_disable(SPIM);
    ll_spi_set_data_size(SPIM, LL_SPI_DATASIZE_32BIT);
    ll_spi_set_transfer_direction(SPIM, LL_SPI_READ_EEPROM);
    ll_spi_set_receive_size(SPIM, words - 1);
    ll_spi_set_rx_sample_delay(SPIM, 0);
    ll_spi_enable(SPIM);

    uint32_t ia = (op_code << 24UL) | (address << 8UL) | (words & 0xFF);

    SPIM->DATA = ia;

    while (rx_cnt < words)
    {
        if (ll_spi_is_active_flag(SPIM, SPI_STAT_RX_FIFO_NE))
        {
            p_buff[rx_cnt] = SPIM->DATA;
            rx_cnt++;
        }
    }

    return true;
}

// Adapt the weak function in tcan_4550_port.
bool spi_send_word_port(uint8_t op_code, uint16_t address, uint32_t * p_buff, uint32_t words)
{
    ll_spi_disable(SPIM);
    ll_spi_set_data_size(SPIM, LL_SPI_DATASIZE_32BIT);
    ll_spi_set_transfer_direction(SPIM, LL_SPI_SIMPLEX_TX);
    ll_spi_enable(SPIM);

    GLOBAL_EXCEPTION_DISABLE();

    uint32_t ia = (op_code << 24UL) | (address << 8UL) | (words & 0xFF);

    while (!ll_spi_is_active_flag(SPIM, SPI_STAT_TX_FIFO_EMPTY))
    {
        // Nothing to do.
    }

    SPIM->DATA = ia;
    for (uint32_t i = 0; i < words; i++)
    {
        while (!ll_spi_is_active_flag(SPIM, SPI_STAT_TX_FIFO_NF))
        {
            // Nothing to do.
        }
        SPIM->DATA = p_buff[i];
    }

    while (!ll_spi_is_active_flag(SPIM, SPI_STAT_TX_FIFO_EMPTY))
    {
        // Nothing to do.
    }
    while (ll_spi_is_active_flag(SPIM, SPI_STAT_SSI_BUSY))
    {
        // Nothing to do.
    }
    GLOBAL_EXCEPTION_ENABLE();

    return true;
}

// Adapt the weak function in tcan_4550_port.
bool spi_send_word_data_only_port(uint32_t * p_buff, uint32_t words)
{
    uint32_t i;

    ll_spi_disable(SPIM);
    ll_spi_set_data_size(SPIM, LL_SPI_DATASIZE_32BIT);
    ll_spi_set_transfer_direction(SPIM, LL_SPI_SIMPLEX_TX);
    ll_spi_enable(SPIM);

    GLOBAL_EXCEPTION_DISABLE();

    for (i = 0; i < words; i++)
    {
        while (!ll_spi_is_active_flag(SPIM, SPI_STAT_TX_FIFO_NF))
        {
            // Nothing to do.
        }
        SPIM->DATA = p_buff[i];
    }

    while (!ll_spi_is_active_flag(SPIM, SPI_STAT_TX_FIFO_EMPTY))
    {
        // Nothing to do.
    }
    while (ll_spi_is_active_flag(SPIM, SPI_STAT_SSI_BUSY))
    {
        // Nothing to do.
    }
    GLOBAL_EXCEPTION_ENABLE();

    return true;
}

// Adapt the weak function in tcan_4550_port.
void spi_cspin_assert_port(void)
{
    delay_us(1);
    app_io_write_pin(s_spi_params.pin_cfg.cs.type, s_spi_params.pin_cfg.cs.pin, APP_IO_PIN_SET);
    delay_us(2);
    app_io_write_pin(s_spi_params.pin_cfg.cs.type, s_spi_params.pin_cfg.cs.pin, APP_IO_PIN_RESET);
}

// Adapt the weak function in tcan_4550_port.
void spi_cspin_deassert_port(void)
{
    delay_us(1);
    app_io_write_pin(s_spi_params.pin_cfg.cs.type, s_spi_params.pin_cfg.cs.pin, APP_IO_PIN_SET);
}

// Adapt the weak function in tcan_4550_port.
bool spi2can_get_int_irq_status(void)
{
    app_io_pin_state_t pin_s = app_io_read_pin(s_spi2can_env.int_pin.type, s_spi2can_env.int_pin.pin);
    return (pin_s == APP_IO_PIN_SET);
}

/*
 * STATIC FUNCTION DEFINITIONS
 ****************************************************************************************
 */
static void spi_event_callback(app_spi_evt_t *p_evt)
{
    UNUSED(p_evt);
}

static void wakeup_irq_pin_event_handler(app_io_evt_t *p_evt)
{
    tcan4550_run_wakeup_irq_process();
}

static void int_irq_pin_event_handler(app_io_evt_t *p_evt)
{
    tcan4550_run_int_irq_process();
}

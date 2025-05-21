#include "app_uart2lin.h"
#include "app_uart_dma.h"
#include "app_log.h"
#include "app_assert.h"
#include "app_timer.h"
#include "app_dual_tim.h"
#include "ring_buffer.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define GWCP_DUAL_TIMER_TIMER_MS(X) (SystemCoreClock / 1000 *(X) - 1)

#define LIN_MASTER_REQUEST_FRAME    (0x3C)
#define LIN_SLAVE_RESPONSE_FRAME    (0x3D)

#define UART_RINGBUFFER_LEN         (128)

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static app_uart_params_t s_uart_params = {
    .dma_cfg = {
        .tx_dma_instance = NULL,
        .rx_dma_instance = NULL,
    },
    .init = {
        .data_bits = UART_DATABITS_8,
        .stop_bits = UART_STOPBITS_1,
        .parity    = UART_PARITY_NONE,
        .hw_flow_ctrl    = UART_HWCONTROL_NONE,
        .rx_timeout_mode = UART_RECEIVER_TIMEOUT_DISABLE,
    },
};

static app_uart2lin_env_t s_uart2lin_env =
{
    .is_timeout = false,
    .is_timer_active = false,
    .init_state = APP_UART2LIN_INIT_NO_READY,
    .tx_type = APP_UART2LIN_TX_IDLE,
    .rx_type = APP_UART2LIN_RX_IDLE,
};

static uint8_t s_tx_buffer[1] = {0};

// UART's RX buffer is used by app_uart driver.
static uint8_t s_receive_buffer[APP_UART2LIN_FRAME_LEN] = {0};

// UART2LIN RX ring_buffer
static uint8_t s_ring_buffer[UART_RINGBUFFER_LEN] = {0};
static ring_buffer_t s_ring_buffer_handle;

// UART2LIN RX interrupt lock
static volatile bool s_uart_rx_lock = false;

// UART2LIN software timer instance
static app_timer_id_t s_uart2lin_timer_id = {0};

// UART2LIN hardware timer instance
static app_dual_tim_params_t s_dual_tim = {
    .id = APP_DUAL_TIM_ID_1,
    .init = {
        .prescaler    = DUAL_TIMER_PRESCALER_DIV0,
        .counter_mode = DUAL_TIMER_COUNTERMODE_ONESHOT,
        .auto_reload  = 63999999,
    },
};

/*
 * STATIC FUNCTION DEFINITIONS
 ****************************************************************************************
 */
static void uart2lin_software_timer_cb(void* p_arg)
{
    GLOBAL_EXCEPTION_DISABLE();

    s_uart2lin_env.is_timer_active = false;
    s_uart2lin_env.is_timeout = true;
    s_uart2lin_env.rx_type = APP_UART2LIN_RX_HEAD;
    s_uart2lin_env.tx_type = APP_UART2LIN_TX_IDLE;
    s_uart2lin_env.error_code |= APP_UART2LIN_ERROR_TIMEOUT;

    if (HAL_UART_STATE_READY != s_uart_params.uart_dev.handle.rx_state)
    {
        (void)app_uart_abort_receive(s_uart_params.id);
    }

    app_uart2lin_error_cb(&s_uart2lin_env);
    s_uart2lin_env.error_code = APP_UART2LIN_ERROR_NONE;

    GLOBAL_EXCEPTION_ENABLE();
}

static void uart2lin_hardware_timer_cb(app_dual_tim_evt_t *p_evt)
{
    LOCAL_INT_DISABLE(BLE_IRQn);

    s_uart2lin_env.is_timer_active = false;
    s_uart2lin_env.is_timeout = true;
    s_uart2lin_env.rx_type = APP_UART2LIN_RX_HEAD;
    s_uart2lin_env.tx_type = APP_UART2LIN_TX_IDLE;
    s_uart2lin_env.error_code |= APP_UART2LIN_ERROR_TIMEOUT;

    if (HAL_UART_STATE_READY != s_uart_params.uart_dev.handle.rx_state)
    {
        (void)app_uart_abort_receive(s_uart_params.id);
    }

    app_uart2lin_error_cb(&s_uart2lin_env);
    s_uart2lin_env.error_code = APP_UART2LIN_ERROR_NONE;

    LOCAL_INT_RESTORE();
}

static void uart2lin_timer_config(void)
{
    if (APP_UART2LIN_SOFTWARE_TIMER == s_uart2lin_env.timer_type)
    {
        if (SDK_SUCCESS != app_timer_create(&s_uart2lin_timer_id, ATIMER_ONE_SHOT, uart2lin_software_timer_cb))
        {
            //lint -e506 -e774 The passed constant value is intended to trigger an ASSERT directly.
            APP_ASSERT_CHECK(false);
        }
    }
    else
    {
        s_dual_tim.init.auto_reload = GWCP_DUAL_TIMER_TIMER_MS(s_uart2lin_env.rx_response_timeout);
        if (APP_DRV_SUCCESS != app_dual_tim_init(&s_dual_tim, uart2lin_hardware_timer_cb))
        {
            APP_ASSERT_CHECK(false);
        }
    }
}

static void uart2lin_timer_start(void)
{
    s_uart2lin_env.is_timer_active = true;
    if (APP_UART2LIN_SOFTWARE_TIMER == s_uart2lin_env.timer_type)
    {
        app_timer_stop(s_uart2lin_timer_id);
        //lint -e9080 The third argument of app_timer_start is a void pointer, and this parameter is not needed here.
        if (SDK_SUCCESS != app_timer_start(s_uart2lin_timer_id, s_uart2lin_env.rx_response_timeout, NULL))
        {
            s_uart2lin_env.is_timer_active = false;
        }
    }
    else
    {
        (void) app_dual_tim_stop(APP_DUAL_TIM_ID_1);
        app_dual_tim_set_params(&s_dual_tim, APP_DUAL_TIM_ID_1);
        if (SDK_SUCCESS != app_dual_tim_start(APP_DUAL_TIM_ID_1))
        {
            s_uart2lin_env.is_timer_active = false;
        }
    }
}

static void uart2lin_timer_stop(void)
{
    s_uart2lin_env.is_timer_active = false;
    if (APP_UART2LIN_SOFTWARE_TIMER == s_uart2lin_env.timer_type)
    {
        app_timer_stop(s_uart2lin_timer_id);
    }
    else
    {
        (void) app_dual_tim_stop(APP_DUAL_TIM_ID_1);
        ll_dual_timer_clear_flag_it(s_dual_tim.dual_tim_env.handle.p_instance);
    }
}

static uint8_t lin_get_std_checksum(uint8_t *p_data, uint8_t len)
{
    uint32_t sum = 0;

    for (uint8_t i = 0; i < len; i++)
    {
        sum = sum + p_data[i];
        if (sum > 0xff)
        {
            sum = (sum & 0xff) + 0x1;
        }
    }
    return 0xff - (uint8_t)sum;
}

static uint8_t lin_get_std_enhance_checksum(uint8_t pid, uint8_t *p_data, uint8_t len)
{
    uint32_t sum = pid;

    for (uint8_t i = 0; i < len; i++)
    {
        sum = sum + p_data[i];
        if (sum > 0xff)
        {
            sum = (sum & 0xff) + 0x1;
        }
    }
    return 0xff - (uint8_t)sum;
}

static uint8_t lin_get_pid_parity(uint8_t pid)
{
    uint8_t id = pid;
    uint8_t p0 = ((id >> 0) & 0x1) ^ ((id >> 1) & 0x1) ^ ((id >> 2) & 0x1) ^ ((id >> 4) & 0x1);
    uint8_t p1 = !(((id >> 1) & 0x1) ^ ((id >> 3) & 0x1) ^ ((id >> 4) & 0x1) ^ ((id >> 5) & 0x1));

    /* id + parity */
    id &= ~(0x3 << 6);
    id |= p0 << 6;
    id |= p1 << 7;
    return id;
}

static uint8_t lin_remove_pid_parity(uint8_t pid)
{
    return (pid & APP_UART2LIN_MAX_PID);
}

static uint8_t app_uart2lin_get_checksum(uint8_t pid, uint8_t *p_data, uint8_t length)
{
    uint8_t checksum = 0;

    if (LIN_MASTER_REQUEST_FRAME == pid || (LIN_SLAVE_RESPONSE_FRAME == pid)) // LIN diagnostic frame
    {
        checksum = lin_get_std_checksum(p_data, length);
    }
    else // LIN non-diagnostic frame
    {
        if (APP_UART2LIN_CHECK_ENHANCE_SUPPORT == s_uart2lin_env.checksum_type) // LIN Version >= LIN 2.0
        {
            checksum = lin_get_std_enhance_checksum(lin_get_pid_parity(pid), p_data, length);
        }
        else // LIN Version <= LIN 1.3
        {
            checksum = lin_get_std_checksum(p_data, length);
        }
    }

    return checksum;
}

static void hw_uart_transmit(app_uart2lin_tx_type_t tx_type)
{
    uint8_t hw_tx_count = 0;
    uint32_t sbf_time = 1 * 1000000 / s_uart_params.init.baud_rate * s_uart2lin_env.brk_len;
    uint32_t sbf_delimiter_time = 1 * 1000000 / s_uart_params.init.baud_rate;
    s_uart_params.uart_dev.handle.tx_state = HAL_UART_STATE_BUSY_TX;
    s_uart2lin_env.tx_type = tx_type;

    if (APP_UART2LIN_MODE_MASTER == s_uart2lin_env.lin_mode && (APP_UART2LIN_TX_HEAD == tx_type))
    {
        // Generating the sbf field that creates the LIN frame header, specifically 13 - 16 bits of low level.
        ll_uart_set_bc(s_uart_params.uart_dev.handle.p_instance);
        delay_us(sbf_time);
    }

    GLOBAL_EXCEPTION_DISABLE();

    if (APP_UART2LIN_MODE_MASTER == s_uart2lin_env.lin_mode && (APP_UART2LIN_TX_HEAD == tx_type))
    {
        ll_uart_reset_bc(s_uart_params.uart_dev.handle.p_instance);
        delay_us(sbf_delimiter_time);
    }

    // Put all the data that needs to be sent into the fifo to ensure the continuity of the tx data.
    __HAL_UART_ENABLE_IT(&s_uart_params.uart_dev.handle, UART_IT_THRE);
    while (s_uart2lin_env.tx_len > hw_tx_count)
    {
        ll_uart_transmit_data8(s_uart_params.uart_dev.handle.p_instance, s_uart2lin_env.tx_buffer[hw_tx_count++]);
    }
    ll_uart_set_tx_fifo_threshold(s_uart_params.uart_dev.handle.p_instance, LL_UART_TX_FIFO_TH_EMPTY);

    uart2lin_timer_stop();
    uart2lin_timer_start();

    GLOBAL_EXCEPTION_ENABLE();
}

static void app_uart_callback(app_uart_evt_t *p_evt)
{
    if (APP_UART_EVT_RX_DATA == p_evt->type)
    {
        bool cur_lock_state = true;
        uint8_t rx_packet[APP_UART2LIN_FRAME_LEN] = {0};
        uint8_t std_checksum = 0;
        uint8_t data_len = 0;

        GLOBAL_EXCEPTION_DISABLE();
        cur_lock_state = s_uart_rx_lock;
        if (!s_uart_rx_lock)
        {
            s_uart_rx_lock = true;
        }
        GLOBAL_EXCEPTION_ENABLE();

        if (cur_lock_state)
        {
            // If interrupt nesting occurs, discard the received data. Ensure that the system can recover.
            return;
        }

        if (APP_UART2LIN_TX_RESPONSE == s_uart2lin_env.tx_type)
        {
            // Filter out loopback data from the transceiver.
            s_uart2lin_env.rx_type = APP_UART2LIN_RX_HEAD;
            s_uart2lin_env.tx_type = APP_UART2LIN_TX_IDLE;
            uart2lin_timer_stop();
            if (p_evt->data.size > s_uart2lin_env.tx_len)
            {
                ring_buffer_write(&s_ring_buffer_handle, s_receive_buffer + s_uart2lin_env.tx_len, p_evt->data.size - s_uart2lin_env.tx_len);
            }
            if (0 != memcmp(s_receive_buffer, s_uart2lin_env.tx_buffer, s_uart2lin_env.tx_len))
            {
                s_uart2lin_env.error_code |= APP_UART2LIN_ERROR_BIT_ERROR;
                app_uart2lin_error_cb(&s_uart2lin_env);
                s_uart2lin_env.error_code = APP_UART2LIN_ERROR_NONE;
            }
            else
            {
                app_uart2lin_tx_response_cb(&s_uart2lin_env);
            }
            goto EXIT;
        }

        ring_buffer_write(&s_ring_buffer_handle, s_receive_buffer, p_evt->data.size);

        while (ring_buffer_items_count_get(&s_ring_buffer_handle) && (APP_UART2LIN_RX_HEAD == s_uart2lin_env.rx_type))
        {
            uint8_t sync_frame_index = 0xFF;

            if (ring_buffer_items_count_get(&s_ring_buffer_handle) < APP_UART2LIN_HEAD_LEN)
            {
                s_uart2lin_env.error_code |= APP_UART2LIN_ERROR_INCOMPLETE_DATA;
                app_uart2lin_error_cb(&s_uart2lin_env);
                s_uart2lin_env.error_code = APP_UART2LIN_ERROR_NONE;
                goto EXIT;
            }

            for (uint8_t idx = 0; idx < APP_UART2LIN_HEAD_LEN; idx++)
            {
                uint32_t ring_buffer_index = 0;

                if (s_ring_buffer_handle.read_index + idx < s_ring_buffer_handle.buffer_size)
                {
                    ring_buffer_index = s_ring_buffer_handle.read_index + idx;
                }
                else
                {
                    ring_buffer_index = s_ring_buffer_handle.read_index + idx - s_ring_buffer_handle.buffer_size;
                }

                if (APP_UART2LIN_SYNC_FRAME == (*(s_ring_buffer_handle.p_buffer + ring_buffer_index)))
                {
                    sync_frame_index = idx;
                    break;
                }
            }

            // UART2LIN HEAD: 0x0 (break) + 0x55 (sync) + pid.
            switch (sync_frame_index)
            {
                case (APP_UART2LIN_SYNC_FIELD_INDEX):
                    {
                        // Contain: 0x0 (break) + 0x55 (sync) + pid.
                        ring_buffer_read(&s_ring_buffer_handle, rx_packet, APP_UART2LIN_HEAD_LEN);
                        // Received LIN head.
                        s_uart2lin_env.tx_type = APP_UART2LIN_TX_IDLE;
                        s_uart2lin_env.rx_pid = lin_remove_pid_parity(rx_packet[APP_UART2LIN_PID_INDEX]);
                        if (rx_packet[APP_UART2LIN_PID_INDEX] == lin_get_pid_parity(s_uart2lin_env.rx_pid))
                        {
                            uart2lin_timer_start();
                            s_uart2lin_env.rx_type = APP_UART2LIN_RX_RESPONSE;
                            s_uart2lin_env.is_timeout = false;
                            app_uart2lin_rx_head_cb(&s_uart2lin_env);
                        }
                        else
                        {
                            s_uart2lin_env.error_code |= APP_UART2LIN_ERROR_IDENTIFIER_PARITY;
                            app_uart2lin_error_cb(&s_uart2lin_env);
                            s_uart2lin_env.error_code = APP_UART2LIN_ERROR_NONE;
                        }
                    }
                    break;
                case (APP_UART2LIN_SYNC_FIELD_INDEX + 1):
                    {
                        // Contain: 0x0 (break) + 0x55 (sync).
                        ring_buffer_read(&s_ring_buffer_handle, rx_packet, APP_UART2LIN_HEAD_LEN - 2);
                        if (APP_DRV_SUCCESS != app_uart_dma_receive_async(s_uart_params.id, s_receive_buffer, APP_UART2LIN_HEAD_LEN - 2))
                        {
                            s_uart2lin_env.error_code |= APP_UART2LIN_ERROR_RX_HEAD_STOP;
                            app_uart2lin_error_cb(&s_uart2lin_env);
                            s_uart2lin_env.error_code = APP_UART2LIN_ERROR_NONE;
                        }
                    }
                    break;
                default:
                    {
                        // No sync data
                        uint8_t rx_fifo_level = ll_uart_get_rx_fifo_level(s_uart_params.uart_dev.handle.p_instance);
                        ring_buffer_read(&s_ring_buffer_handle, rx_packet, APP_UART2LIN_HEAD_LEN);
                        if (APP_UART2LIN_BREAK_FRAME == rx_packet[APP_UART2LIN_PID_INDEX])
                        {
                            ring_buffer_write(&s_ring_buffer_handle, rx_packet + APP_UART2LIN_PID_INDEX, 1);
                            if (APP_DRV_SUCCESS != app_uart_dma_receive_async(s_uart_params.id, s_receive_buffer, APP_UART2LIN_HEAD_LEN - 1))
                            {
                                s_uart2lin_env.error_code |= APP_UART2LIN_ERROR_RX_HEAD_STOP;
                                app_uart2lin_error_cb(&s_uart2lin_env);
                                s_uart2lin_env.error_code = APP_UART2LIN_ERROR_NONE;
                            }
                        }
                        else
                        {
                            if (APP_DRV_SUCCESS != app_uart_dma_receive_async(s_uart_params.id, s_receive_buffer, APP_UART2LIN_HEAD_LEN))
                            {
                                s_uart2lin_env.error_code |= APP_UART2LIN_ERROR_RX_HEAD_STOP;
                                app_uart2lin_error_cb(&s_uart2lin_env);
                                s_uart2lin_env.error_code = APP_UART2LIN_ERROR_NONE;
                            }
                        }
                    }
                    break;
            }
            goto EXIT;
        }

        // Received LIN response.
        data_len = ring_buffer_items_count_get(&s_ring_buffer_handle);
        APP_ASSERT_CHECK(data_len > 1);

        if (data_len > APP_UART2LIN_DATA_LEN + APP_UART2LIN_CHECKSUM_LEN)
        {
            data_len = APP_UART2LIN_DATA_LEN + APP_UART2LIN_CHECKSUM_LEN;
        }

        ring_buffer_read(&s_ring_buffer_handle, rx_packet, data_len);
        s_uart2lin_env.rx_type = APP_UART2LIN_RX_HEAD;

        std_checksum = app_uart2lin_get_checksum(s_uart2lin_env.rx_pid, rx_packet, data_len - 1);

        uart2lin_timer_stop();

        if (rx_packet[data_len - 1] == std_checksum)
        {
            s_uart2lin_env.rx_len = data_len - 1;
            memcpy(s_uart2lin_env.p_rx_buffer, rx_packet, s_uart2lin_env.rx_len);
            app_uart2lin_rx_response_cb(&s_uart2lin_env);
        }
        else
        {
            s_uart2lin_env.error_code |= APP_UART2LIN_ERROR_CHECKSUM;
            app_uart2lin_error_cb(&s_uart2lin_env);
            s_uart2lin_env.error_code = APP_UART2LIN_ERROR_NONE;
        }

EXIT:
        s_uart_rx_lock = false;
        return;
    }

    if (APP_UART_EVT_ERROR == p_evt->type)
    {
        if (HAL_UART_ERROR_OE == (s_uart_params.uart_dev.handle.error_code & HAL_UART_ERROR_OE))
        {
            s_uart2lin_env.error_code |= APP_UART2LIN_ERROR_BUFFER_OVER;
        }
        if (HAL_UART_ERROR_FE == (s_uart_params.uart_dev.handle.error_code & HAL_UART_ERROR_FE))
        {
            s_uart2lin_env.error_code |= APP_UART2LIN_ERROR_FRAME;
        }
        if (!s_uart2lin_env.is_timer_active)
        {
            s_uart2lin_env.error_code |= APP_UART2LIN_ERROR_RX_HEAD_STOP;
            app_uart2lin_error_cb(&s_uart2lin_env);
            s_uart2lin_env.error_code = APP_UART2LIN_ERROR_NONE;
        }
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
bool app_uart2lin_init(app_uart2lin_init_t *p_uart2lin_init)
{
    if ((NULL == p_uart2lin_init) || (NULL == p_uart2lin_init->rx_dma_instance))
    {
        return false;
    }

    //lint -e64 Type mismatch is necessary
    app_uart_tx_buf_t uart_buffer = {s_tx_buffer, sizeof(s_tx_buffer)};

    s_uart_params.id = p_uart2lin_init->uart_id;
    memcpy(&(s_uart_params.pin_cfg), &(p_uart2lin_init->pin_cfg), sizeof(app_uart_pin_cfg_t));
    s_uart_params.init.baud_rate = p_uart2lin_init->baudrate;
    s_uart_params.dma_cfg.rx_dma_instance = p_uart2lin_init->rx_dma_instance;
    s_uart_params.dma_cfg.rx_dma_channel = p_uart2lin_init->rx_dma_channel;

    if (!ring_buffer_init(&s_ring_buffer_handle, s_ring_buffer, UART_RINGBUFFER_LEN))
    {
        return false;
    }

    if (APP_DRV_SUCCESS != app_uart_init(&s_uart_params, app_uart_callback, &uart_buffer)
        || (APP_DRV_SUCCESS != app_uart_dma_init(&s_uart_params)))
    {
        return false;
    }

    // Interrupt Preemption Priority: BLE > DMA > UART = DUAL_TIMER
    if (APP_UART_ID_0 == s_uart_params.id)
    {
        NVIC_SetPriority(UART0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0));
    }
    else
    {
        NVIC_SetPriority(UART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0));
    }
    NVIC_SetPriority(DMA0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
    if (APP_UART2LIN_HARDWARE_TIMER == p_uart2lin_init->timer_type)
    {
        NVIC_SetPriority(DUAL_TIMER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 1));
    }

    s_uart2lin_env.lin_mode = p_uart2lin_init->lin_mode;
    s_uart2lin_env.timer_type = p_uart2lin_init->timer_type;
    s_uart2lin_env.brk_len = p_uart2lin_init->brk_len;
    s_uart2lin_env.checksum_type = p_uart2lin_init->checksum_type;
    s_uart2lin_env.init_state = APP_UART2LIN_INIT_READY;
    s_uart2lin_env.rx_type = APP_UART2LIN_RX_HEAD;

    // Timeout period = (data segment + checksum segment) * 40%
    // Because UART is using DMA for reception, it is not possible to detect whether data has been received during the reception process.
    // Therefore, the timeout period of the timer should be set after the data reception is completed.
    uint32_t uart2lin_min_rx_rsp_timeout = ((1 * 1000000  * 10 * (APP_UART2LIN_DATA_LEN + APP_UART2LIN_CHECKSUM_LEN)/ s_uart_params.init.baud_rate) * 7 / 5) / 1000;

    if (p_uart2lin_init->rx_response_timeout <= uart2lin_min_rx_rsp_timeout)
    {
        s_uart2lin_env.rx_response_timeout = uart2lin_min_rx_rsp_timeout;
    }
    else
    {
        s_uart2lin_env.rx_response_timeout = p_uart2lin_init->rx_response_timeout;
    }

    (void)app_uart_abort_receive(s_uart_params.id);

    uart2lin_timer_config();

    return true;
}

bool app_uart2lin_deinit(void)
{
    if (APP_UART2LIN_TX_IDLE != s_uart2lin_env.tx_type)
    {
        return false;
    }

    uint16_t ret_dma = 0;
    uint16_t ret_uart = 0;

    ret_dma = app_uart_dma_deinit(s_uart_params.id);
    ret_uart = app_uart_deinit(s_uart_params.id);

    if ((APP_DRV_SUCCESS != ret_dma) || (APP_DRV_SUCCESS != ret_uart))
    {
        return false;
    }

    s_uart2lin_env.init_state = APP_UART2LIN_INIT_NO_READY;
    s_uart2lin_env.rx_type = APP_UART2LIN_RX_IDLE;

    if (APP_UART2LIN_SOFTWARE_TIMER == s_uart2lin_env.timer_type)
    {
        (void)app_timer_delete(&s_uart2lin_timer_id);
    }
    else
    {
        (void)app_dual_tim_deinit(APP_DUAL_TIM_ID_1);
    }

    return true;
}

bool app_uart2lin_rx_head(void)
{
    // The break signal will be recognized as 0x0 by the UART, so in addition to the sync frame and PID, an extra bit needs to be received.
    // This extra bit will be automatically filtered during RX interrupt processing.
    if (APP_UART2LIN_INIT_READY == s_uart2lin_env.init_state
        && (APP_UART2LIN_TX_IDLE == s_uart2lin_env.tx_type)
        && (APP_DRV_SUCCESS == app_uart_dma_receive_async(s_uart_params.id, s_receive_buffer, APP_UART2LIN_HEAD_LEN)))
    {
        return true;
    }
    else
    {
        return false;
    }
}

void app_uart2lin_force_rx_head(void)
{
    // The break signal will be recognized as 0x0 by the UART, so in addition to the sync frame and PID, an extra bit needs to be received.
    // This extra bit will be automatically filtered during RX interrupt processing.
    if (APP_UART2LIN_INIT_READY == s_uart2lin_env.init_state)
    {
        LOCAL_INT_DISABLE(BLE_IRQn);
        uart2lin_timer_stop();
        ring_buffer_clean(&s_ring_buffer_handle);
        s_uart2lin_env.tx_type = APP_UART2LIN_TX_IDLE;
        s_uart2lin_env.rx_type = APP_UART2LIN_RX_HEAD;
        (void)app_uart_abort(s_uart_params.id);
        if (APP_DRV_SUCCESS != app_uart_dma_receive_async(s_uart_params.id, s_receive_buffer, APP_UART2LIN_HEAD_LEN))
        {
            APP_ASSERT_CHECK(false);
        }
        LOCAL_INT_RESTORE();
    }
}

bool app_uart2lin_rx_response(uint8_t *p_data, uint8_t size)
{
    if (NULL == p_data || (APP_UART2LIN_INIT_NO_READY == s_uart2lin_env.init_state) || (APP_UART2LIN_TX_IDLE != s_uart2lin_env.tx_type))
    {
        return false;
    }

    s_uart2lin_env.p_rx_buffer = p_data;
    return (APP_DRV_SUCCESS == app_uart_dma_receive_async(s_uart_params.id, s_receive_buffer, size + APP_UART2LIN_CHECKSUM_LEN));
}

bool app_uart2lin_tx_head(uint8_t pid)
{
    if (APP_UART2LIN_TX_IDLE != s_uart2lin_env.tx_type
        || (pid > APP_UART2LIN_MAX_PID)
        || (APP_UART2LIN_INIT_NO_READY == s_uart2lin_env.init_state)
        || (APP_UART2LIN_MODE_MASTER != s_uart2lin_env.lin_mode))
    {
        return false;
    }
    else
    {
        //lint -e778 The calculation here is valid.
        s_uart2lin_env.tx_buffer[APP_UART2LIN_SYNC_FIELD_INDEX - 1] = APP_UART2LIN_SYNC_FRAME;
        s_uart2lin_env.tx_buffer[APP_UART2LIN_PID_INDEX - 1] = lin_get_pid_parity(pid);
        s_uart2lin_env.tx_len = APP_UART2LIN_HEAD_LEN - 1;
        hw_uart_transmit(APP_UART2LIN_TX_HEAD);
        return true;
    }
}

bool app_uart2lin_tx_response(uint8_t *p_data, uint8_t size)
{
    //lint -e775 The size may be 0.
    if (APP_UART2LIN_TX_IDLE != s_uart2lin_env.tx_type
        || (s_uart2lin_env.is_timeout)
        || (NULL == p_data)
        || (size <= 0)
        || (size > APP_UART2LIN_DATA_LEN)
        || (APP_UART2LIN_INIT_NO_READY == s_uart2lin_env.init_state))
    {
        return false;
    }

    memset(s_uart2lin_env.tx_buffer, 0, APP_UART2LIN_DATA_LEN + APP_UART2LIN_CHECKSUM_LEN);
    memcpy(s_uart2lin_env.tx_buffer, p_data, size);
    s_uart2lin_env.tx_buffer[size] = app_uart2lin_get_checksum(s_uart2lin_env.rx_pid, p_data, size);
    s_uart2lin_env.tx_len = size + APP_UART2LIN_CHECKSUM_LEN;
    hw_uart_transmit(APP_UART2LIN_TX_RESPONSE);

    return true;
}

void app_uart2lin_abort_rx(void)
{
    uart2lin_timer_stop();
    ring_buffer_clean(&s_ring_buffer_handle);
    s_uart2lin_env.tx_type = APP_UART2LIN_TX_IDLE;
    s_uart2lin_env.rx_type = APP_UART2LIN_RX_HEAD;
    (void)app_uart_abort_receive(s_uart_params.id);
}

void app_uart2lin_abort_tx(void)
{
    s_uart2lin_env.tx_type = APP_UART2LIN_TX_IDLE;
}

void app_uart2lin_wakeup(uint32_t low_level_duration)
{
    LOCAL_INT_DISABLE(BLE_IRQn);
    if (APP_IO_TYPE_AON == s_uart_params.pin_cfg.tx.type)
    {
        ll_aon_gpio_set_pin_mux(s_uart_params.pin_cfg.tx.pin, IO_MUX_GPIO);
        ll_aon_gpio_set_pin_mode(s_uart_params.pin_cfg.tx.pin, LL_AON_GPIO_MODE_OUTPUT);
        ll_aon_gpio_reset_output_pin(s_uart_params.pin_cfg.tx.pin);
        delay_us(low_level_duration);
        ll_aon_gpio_set_pin_mux(s_uart_params.pin_cfg.tx.pin, s_uart_params.pin_cfg.tx.mux);
        ll_aon_gpio_set_pin_mode(s_uart_params.pin_cfg.tx.pin, LL_AON_GPIO_MODE_INOUT);
    }
    else if (APP_IO_TYPE_GPIOA == s_uart_params.pin_cfg.tx.type)
    {
        ll_gpio_set_pin_mux(GPIO0, s_uart_params.pin_cfg.tx.pin, IO_MUX_GPIO);
        ll_gpio_set_pin_mode(GPIO0, s_uart_params.pin_cfg.tx.pin, LL_GPIO_MODE_OUTPUT);
        ll_gpio_reset_output_pin(GPIO0, s_uart_params.pin_cfg.tx.pin);
        delay_us(low_level_duration);
        ll_gpio_set_pin_mux(GPIO0, s_uart_params.pin_cfg.tx.pin, s_uart_params.pin_cfg.tx.mux);
        ll_gpio_set_pin_mode(GPIO0, s_uart_params.pin_cfg.tx.pin, LL_GPIO_MODE_INOUT);
    }
    else if (APP_IO_TYPE_MSIO == s_uart_params.pin_cfg.tx.type)
    {
        ll_msio_set_pin_mux(MSIOA, s_uart_params.pin_cfg.tx.pin, IO_MUX_GPIO);
        ll_msio_set_pin_direction(MSIOA, s_uart_params.pin_cfg.tx.pin, LL_MSIO_DIRECTION_OUTPUT);
        ll_msio_reset_output_pin(MSIOA, s_uart_params.pin_cfg.tx.pin);
        delay_us(low_level_duration);
        ll_msio_set_pin_mux(MSIOA, s_uart_params.pin_cfg.tx.pin, s_uart_params.pin_cfg.tx.mux);
        ll_msio_set_pin_direction(MSIOA, s_uart_params.pin_cfg.tx.pin, LL_MSIO_DIRECTION_INOUT);
    }
    else
    {
        APP_ASSERT_CHECK(false);
    }
    LOCAL_INT_RESTORE();
}

__WEAK void app_uart2lin_rx_head_cb(app_uart2lin_env_t *p_uart2lin)
{
    UNUSED(p_uart2lin);
}

__WEAK void app_uart2lin_tx_response_cb(app_uart2lin_env_t *p_uart2lin)
{
    UNUSED(p_uart2lin);
}

__WEAK void app_uart2lin_rx_response_cb(app_uart2lin_env_t *p_uart2lin)
{
    UNUSED(p_uart2lin);
}

__WEAK void app_uart2lin_error_cb(app_uart2lin_env_t *p_uart2lin)
{
    UNUSED(p_uart2lin);
}

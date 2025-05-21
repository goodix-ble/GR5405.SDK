/**
 *****************************************************************************************
 *
 * @file ble_debug.c
 *
 * @brief  BLE Debug Implementation.
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
#include "ble_debug.h"
#include "ble_debug_server.h"
#include "hal_flash.h"
#include "utility.h"

/*
 * DEFINE
 *****************************************************************************************
 */
#define MAX_DUMP_SIZE                   256U  /**< Single dump MAX size. */
#define BUFFER_SIZE                     (MAX_DUMP_SIZE + 16U)  /**< TRX buffer size. */
#define REG(x)                          (*((volatile unsigned int*)(x)))
#define CMD_FRAME_HEADER_L              0x44U
#define CMD_FRAME_HEADER_H              0x47U
#define ATT_CODE_AND_HANDLE_LEN         3U

#define DBG_CMD_WRITE_RAM               0x11U
#define DBG_CMD_READ_RAM                0x12U
#define DBG_CMD_DUMP_FLASH              0x21U
#define DBG_CMD_OPERATE_REG             0x2CU
#define ALL_CMD_COUNT                   4U

/*
 * ENUMERATION
 *****************************************************************************************
 */
typedef enum
{
    CHECK_FRAME_L_STATE = 0x00,
    CHECK_FRAME_H_STATE,
    RECEIVE_CMD_TYPE_L_STATE,
    RECEIVE_CMD_TYPE_H_STATE,
    RECEIVE_LEN_L_STATE,
    RECEIVE_LEN_H_STATE,
    RECEIVE_DATA_STATE,
    RECEIVE_CHECK_SUM_L_STATE,
    RECEIVE_CHECK_SUM_H_STATE,
} parse_state_t;

enum
{
    ACK_SUCCESS = 0x01,
    ACK_ERROR,
};

/*
 * STRUCTURES
 *****************************************************************************************
 */
typedef struct
{
    uint16_t cmd_type;
    uint16_t data_len;
    uint8_t  data[BUFFER_SIZE];
    uint16_t check_sum;
} trx_frame_t;

typedef void(*cmd_handler_t)(trx_frame_t *p_frame);

typedef struct
{
    uint16_t      cmd;
    cmd_handler_t handler;
} cmd_handler_tab_t;

typedef struct
{
    bool          check_state;
    parse_state_t parse_state;
    uint8_t       cmd_pro_index;
    uint8_t       cmd_receive_flag;
    uint16_t      receive_data_count;
    uint16_t      ble_sent_len;
    uint16_t      all_send_len;
    uint16_t      ble_once_size;
    uint16_t      receive_check_sum;
} env_parse_t;

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static void ble_send_data(uint8_t *p_data, uint16_t length);
static void cmd_parse_reset(void);
static void ble_rx_process(uint8_t *p_data, uint16_t len);
static void ble_tx_cmpl_process(void);
static void frame_send(uint8_t *p_data, uint16_t len,uint16_t cmd_type);
static void ble_debug_cmd_ram_write(trx_frame_t *p_frame);
static void ble_debug_cmd_ram_read(trx_frame_t *p_frame);
static void ble_debug_cmd_flash_dump(trx_frame_t *p_frame);
static void ble_debug_cmd_reg_operate(trx_frame_t *p_frame);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t     s_debug_conn_index = BLE_GAP_INVALID_CONN_INDEX;
static trx_frame_t s_trx_frame;
static env_parse_t s_cmd_parse_env;

static const cmd_handler_tab_t cmd_event_handler_tab[ALL_CMD_COUNT] =
{
   { DBG_CMD_WRITE_RAM,    ble_debug_cmd_ram_write   },
   { DBG_CMD_READ_RAM,     ble_debug_cmd_ram_read    },
   { DBG_CMD_DUMP_FLASH,   ble_debug_cmd_flash_dump  },
   { DBG_CMD_OPERATE_REG,  ble_debug_cmd_reg_operate },
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void dbgs_evt_process(dbgs_evt_t *p_evt)
{
    if ((s_debug_conn_index != BLE_GAP_INVALID_CONN_INDEX) && (s_debug_conn_index != p_evt->conn_idx))
    {
        return;
    }

    switch (p_evt->evt_type)
    {
        case DBGS_EVT_TX_NOTIFICATION_ENABLED:
            cmd_parse_reset();
            break;

        case DBGS_EVT_RX_RECEIVE_DATA:
            ble_rx_process(p_evt->p_data, p_evt->length);
            break;

        case DBGS_EVT_NOTIFY_COMPLETE:
            ble_tx_cmpl_process();
            break;

        case DBGS_EVT_DISCONNECT:
            if (s_debug_conn_index == p_evt->conn_idx)
            {
                s_debug_conn_index = BLE_GAP_INVALID_CONN_INDEX;
            }
            break;

        case DBGS_EVT_TASK_ENTER:
            if (s_debug_conn_index == BLE_GAP_INVALID_CONN_INDEX)
            {
                s_debug_conn_index = p_evt->conn_idx;
                uint16_t mtu;
                if (SDK_SUCCESS == ble_gatt_mtu_get(p_evt->conn_idx, &mtu))
                {
                    s_cmd_parse_env.ble_once_size = mtu - ATT_CODE_AND_HANDLE_LEN;
                }
            }
            break;

        default:
            break;
    }
}

static void ble_send_data(uint8_t *p_data, uint16_t length)
{
    (void)dbgs_notify_tx_data(s_debug_conn_index, p_data, length);
}

static void ble_send_frame(uint8_t *p_data, uint16_t len)
{
    s_cmd_parse_env.all_send_len = len;

    if (len >= s_cmd_parse_env.ble_once_size)
    {
        s_cmd_parse_env.ble_sent_len = s_cmd_parse_env.ble_once_size;
    }
    else
    {
        s_cmd_parse_env.ble_sent_len = len;
    }

    ble_send_data(p_data, s_cmd_parse_env.ble_sent_len);
}

static void cmd_parse_reset(void)
{
    s_cmd_parse_env.parse_state = CHECK_FRAME_L_STATE;
    s_cmd_parse_env.cmd_receive_flag = 0;
    s_cmd_parse_env.receive_check_sum = 0;
    s_cmd_parse_env.receive_data_count = 0;
    s_cmd_parse_env.check_state = 0;
    s_cmd_parse_env.ble_once_size = BLE_ATT_MTU_DEFAULT - ATT_CODE_AND_HANDLE_LEN;
}

static uint32_t ram_read(uint32_t address, uint8_t *p_read_buf, uint16_t len)
{
    memcpy(p_read_buf, (void *)address, len);
    return len;
}

static uint32_t ram_write(uint32_t address, uint8_t *p_write_buf, uint16_t len)
{
    memcpy((void *)address, p_write_buf, len);
    return len;
}

static void ble_debug_cmd_ram_write(trx_frame_t *p_frame)
{
    uint32_t ret = 0;
    uint32_t addr = le32toh(&p_frame->data[0]);

    uint16_t len = ((p_frame->data[5] << 8) | (p_frame->data[4]));
    if (len > MAX_DUMP_SIZE)
    {
        p_frame->data[0] = ACK_ERROR;
    }
    else
    {
        ret = ram_write(addr, &(p_frame->data[6]), len);
        if (0U == ret)
        {
            p_frame->data[0] = ACK_ERROR;
        }
        else
        {
            p_frame->data[0] = ACK_SUCCESS;
        }
    }
    frame_send(p_frame->data, 1, p_frame->cmd_type);
}

__STATIC_FORCEINLINE void reg_write(uint32_t reg_addr, uint32_t reg_value)
{
    REG(reg_addr) = reg_value;
}

__STATIC_FORCEINLINE uint32_t reg_read(uint32_t reg_addr)
{
    return REG(reg_addr);
}

static void ble_debug_cmd_reg_operate(trx_frame_t *p_frame)
{
    uint8_t operate_type = p_frame->data[0];
    uint32_t reg_addr;
    uint32_t reg_value;
    uint8_t send_len = 0;

    if (0x00U == operate_type) // write
    {
        reg_addr = le32toh(&p_frame->data[1]);
        reg_value = le32toh(&p_frame->data[5]);
        reg_write(reg_addr, reg_value);
        send_len = 1;
        p_frame->data[0] = ACK_SUCCESS;
    }
    else if (0x01U == operate_type) // read
    {
        reg_addr = le32toh(&p_frame->data[1]);
        reg_value = reg_read(reg_addr);
        memcpy(&p_frame->data[1], &reg_value, 4);
        send_len = 5;
        p_frame->data[0] = ACK_SUCCESS;
    }
    else
    {
        p_frame->data[0] = ACK_ERROR;
    }

    frame_send(p_frame->data, send_len, p_frame->cmd_type);
}

static void ble_debug_cmd_ram_read(trx_frame_t *p_frame)
{
    uint32_t ret = 0;
    uint32_t i;
    uint32_t addr = le32toh(&p_frame->data[0]);
    uint16_t len = ((p_frame->data[5] << 8) | (p_frame->data[4]));

    if (len > MAX_DUMP_SIZE)
    {
        len = 0;
        p_frame->data[0] = ACK_ERROR;
    }
    else
    {
        for (i = 6U; i > 0; i--)
        {
            p_frame->data[i] = p_frame->data[i - 1U];
        }

        ret = ram_read(addr, &(p_frame->data[7]), len);
        if (ret)
        {
            p_frame->data[0] = ACK_SUCCESS;

        }
        else
        {
            p_frame->data[0] = ACK_ERROR;
        }
    }

    frame_send(p_frame->data, len + 7U, p_frame->cmd_type);
}

static void ble_debug_cmd_flash_dump(trx_frame_t *p_frame)
{
    uint32_t ret = 0;

    uint32_t addr = le32toh(&p_frame->data[1]);
    uint16_t len = ((p_frame->data[6] << 8U) | (p_frame->data[5]));
    if (len > MAX_DUMP_SIZE)
    {
        len = 0;
        p_frame->data[0] = ACK_ERROR;
    }
    else
    {
        ret = hal_flash_read(addr, &(p_frame->data[7]), len);
        if (ret)
        {
            p_frame->data[0] = ACK_SUCCESS;
        }
        else
        {
            p_frame->data[0] = ACK_ERROR;
        }
    }

    frame_send(p_frame->data, len + 7U, p_frame->cmd_type);
}

static void frame_send(uint8_t *p_data, uint16_t len, uint16_t cmd_type)
{
    uint16_t check_sum = 0;
    int32_t i = 0;

    for (i = len - 1; i >= 0; i--)
    {
        *(p_data + 6 + i) = *(p_data + i);
        check_sum += *(p_data + i);
    }

    *(p_data) = CMD_FRAME_HEADER_L;
    *(p_data + 1) = CMD_FRAME_HEADER_H;
    *(p_data + 2) = (uint8_t)cmd_type;
    *(p_data + 3) = cmd_type >> 8U;
    *(p_data + 4) = (uint8_t)len;
    *(p_data + 5) = len >> 8U;

    for (i = 2; i < 6; i++)
    {
        check_sum += *(p_data + i);
    }

    *(p_data + 6 + len) = (uint8_t)check_sum;
    *(p_data + 7 + len) = check_sum >> 8U;

    ble_send_frame(p_data, len + 8U);
}

static void cmd_check(void)
{
    uint32_t i = 0;

    for (i = 0; i < ALL_CMD_COUNT; i++)
    {
        if (cmd_event_handler_tab[i].cmd == s_trx_frame.cmd_type)
        {
            s_cmd_parse_env.cmd_pro_index = i;
            break;
        }
    }
    if (i < ALL_CMD_COUNT)
    {
        for (i = 0; i < s_trx_frame.data_len; i++)
        {
            s_cmd_parse_env.receive_check_sum += s_trx_frame.data[i];
        }

        if (s_cmd_parse_env.receive_check_sum == s_trx_frame.check_sum)
        {
            s_cmd_parse_env.check_state = true;
        }
        else
        {
            s_cmd_parse_env.check_state = false;
        }
        s_cmd_parse_env.cmd_receive_flag = 1U;
    }
    else
    {
        s_cmd_parse_env.cmd_receive_flag = 1U;
        s_cmd_parse_env.check_state = false;
    }

}

static void cmd_parse_process(uint8_t *p_data, uint16_t len)
{
    uint32_t i = 0;

    if (0U == s_cmd_parse_env.cmd_receive_flag)
    {
        for (i = 0; i < len; i++)
        {
            switch (s_cmd_parse_env.parse_state)
            {
            case CHECK_FRAME_L_STATE:
            {
                s_cmd_parse_env.receive_check_sum = 0;
                if (p_data[i] == CMD_FRAME_HEADER_L)
                {
                    s_cmd_parse_env.parse_state = CHECK_FRAME_H_STATE;
                }
            }
            break;

            case CHECK_FRAME_H_STATE:
            {
                if (p_data[i] == CMD_FRAME_HEADER_H)
                {
                    s_cmd_parse_env.parse_state = RECEIVE_CMD_TYPE_L_STATE;
                }
                else if (p_data[i] == CMD_FRAME_HEADER_L)
                {
                    s_cmd_parse_env.parse_state = CHECK_FRAME_H_STATE;
                }
                else
                {
                    s_cmd_parse_env.parse_state = CHECK_FRAME_L_STATE;
                }
            }
            break;

            case RECEIVE_CMD_TYPE_L_STATE:
            {
                s_trx_frame.cmd_type = p_data[i];
                s_cmd_parse_env.receive_check_sum += p_data[i];
                s_cmd_parse_env.parse_state = RECEIVE_CMD_TYPE_H_STATE;
            }
            break;

            case RECEIVE_CMD_TYPE_H_STATE:
            {
                s_trx_frame.cmd_type |= (p_data[i] << 8);
                s_cmd_parse_env.receive_check_sum += p_data[i];
                s_cmd_parse_env.parse_state = RECEIVE_LEN_L_STATE;
            }
            break;

            case RECEIVE_LEN_L_STATE:
            {
                s_trx_frame.data_len = p_data[i];
                s_cmd_parse_env.receive_check_sum += p_data[i];
                s_cmd_parse_env.parse_state = RECEIVE_LEN_H_STATE;
            }
            break;

            case RECEIVE_LEN_H_STATE:
            {
                s_trx_frame.data_len |= (p_data[i] << 8);
                s_cmd_parse_env.receive_check_sum += p_data[i];
                if ((0U != s_trx_frame.data_len) && (s_trx_frame.data_len < BUFFER_SIZE))
                {
                    s_cmd_parse_env.receive_data_count = 0;
                    s_cmd_parse_env.parse_state = RECEIVE_DATA_STATE;
                }
                else
                {
                    s_trx_frame.data[0] = ACK_ERROR;
                    frame_send(s_trx_frame.data, 1, s_trx_frame.cmd_type);
                    s_cmd_parse_env.parse_state = CHECK_FRAME_L_STATE;
                }
            }
            break;

            case RECEIVE_DATA_STATE:
            {
                s_trx_frame.data[s_cmd_parse_env.receive_data_count] = p_data[i];
                if (++s_cmd_parse_env.receive_data_count == s_trx_frame.data_len)
                {
                    s_cmd_parse_env.parse_state = RECEIVE_CHECK_SUM_L_STATE;
                }
            }
            break;

            case RECEIVE_CHECK_SUM_L_STATE:
            {
                s_trx_frame.check_sum = p_data[i];
                s_cmd_parse_env.parse_state = RECEIVE_CHECK_SUM_H_STATE;
            }
            break;

            case RECEIVE_CHECK_SUM_H_STATE:
            {
                s_trx_frame.check_sum |= (p_data[i] << 8);
                s_cmd_parse_env.parse_state = CHECK_FRAME_L_STATE;
                cmd_check();
            }
            break;

            default:
                s_cmd_parse_env.parse_state = CHECK_FRAME_L_STATE;
                break;
            }
        }
    }
}

static void ble_debug_schedule(void)
{
    if (s_cmd_parse_env.cmd_receive_flag == 1U)
    {
        if (s_cmd_parse_env.check_state == true)
        {
            cmd_event_handler_tab[s_cmd_parse_env.cmd_pro_index].handler(&s_trx_frame);
            s_cmd_parse_env.cmd_receive_flag = 0;
        }
        else
        {
            s_cmd_parse_env.cmd_receive_flag = 0;
            uint8_t ack = ACK_ERROR;
            frame_send(&ack, 1, s_trx_frame.cmd_type);
        }
    }
}

static void ble_rx_process(uint8_t *p_data, uint16_t len)
{
    cmd_parse_process(p_data, len);
    ble_debug_schedule();
}

static void ble_tx_cmpl_process(void)
{
    int remain = s_cmd_parse_env.all_send_len - s_cmd_parse_env.ble_sent_len;
    if (remain >= s_cmd_parse_env.ble_once_size)
    {
        ble_send_data(&s_trx_frame.data[s_cmd_parse_env.ble_sent_len], s_cmd_parse_env.ble_once_size);
        s_cmd_parse_env.ble_sent_len += s_cmd_parse_env.ble_once_size;
    }
    else if (remain > 0)
    {
        ble_send_data(&s_trx_frame.data[s_cmd_parse_env.ble_sent_len], remain);
        s_cmd_parse_env.ble_sent_len += remain;
    }
    else
    {
        // Nothing to to
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void ble_debug_service_init(void)
{
    dbgs_init_t dbgs_init;

    s_debug_conn_index = BLE_GAP_INVALID_CONN_INDEX;
    dbgs_init.evt_handler = dbgs_evt_process;
    dbgs_service_init(&dbgs_init);
}

void ble_debug_reg_write_tx_cccd_cb(ble_debug_write_tx_cccd_cb_t p_cb)
{
    dbgs_reg_write_tx_cccd_cb((dbgs_write_tx_cccd_cb_t)p_cb);
}

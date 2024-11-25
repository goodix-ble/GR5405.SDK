/**
 *****************************************************************************************
 *
 * @file user_app.c
 *
 * @brief User function Implementation.
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
#include "grx_sys.h"
#include "app_log.h"
#include "app_error.h"
#include "user_bat_s.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define SCAN_INTERVAL               15                 /**< Scan interval (uint:0.625ms, 12.5ms). */
#define SCAN_WINDOW                 15                 /**< Scan window (uint:0.625ms, 12.5ms). */
#define INIT_SCAN_INTERVAL          160                /**< Scan interval (uint:0.625ms, 100ms). */
#define INIT_SCAN_WINDOW            160                /**< Scan window (uint:0.625ms, 100ms). */
#define ADV_INTERVAL                160                /**< The fast advertising min interval (unit: 0.625ms, 100ms). */
#define CONNECTION_MIN_INTERVAL     80                 /**< The connection min interval (in units of 1.25 ms. This value corresponds to 100 ms). */
#define CONNECTION_MAX_INTERVAL     80                 /**< The connection max interval (in units of 1.25 ms. This value corresponds to 100 ms). */
#define CONNECTION_SLAVE_LATENCY    0                  /**< Slave latency. */
#define CONNECTION_MAX_TIMEOUT      500                /**< Link supervision timeout (in unit of 10ms. This value corresponds to 5s). */

#define ADV_NAME_OFFSET             0x05               /**< Adv name index. */

#define INVALID_IDX                 0xFF               /**< Invalid test phone index. */
#define TEST_PHONE_NAME1            "TEST_PHONE1"      /**< Adv name for test phone 1. */
#define TEST_PHONE_NAME2            "TEST_PHONE2"      /**< Adv name for test phone 2. */
#define TEST_PHONE_NAME3            "TEST_PHONE3"      /**< Adv name for test phone 3. */
#define TEST_PHONE_NAME4            "TEST_PHONE4"      /**< Adv name for test phone 4. */

#define CONN_ROLE_MASTER             0
#define CONN_ROLE_SLAVE              1
#define MAX_LINK_NUM                 4                 /**< Maxium link number to support. */
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const uint8_t s_adv_data_set[] =                  /**< Advertising data. */
{
    0x09,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'A', 'd', 'v', ' ', 'D', 'e', 'm', 'o',
};

ble_gap_adv_time_param_t adv_time_param =
{
    .duration     = 0,
    .max_adv_evt  = 0,
};

typedef struct
{
    uint8_t conn_idx;    /**< Connectio index. */
    bool conn_flag;      /**< Indicate whether current connection is exist or creating. */
    uint8_t role;        /**< Connectio role. */
} test_phone_info_t;

typedef struct
{
    bool creating_conn_flag;        /**< Indicate whether creating connection is ongoing. */
    uint8_t curr_test_phone_idx;    /**< Record the index for the test phone which is creating connection. */
    test_phone_info_t test_phone_info[MAX_LINK_NUM];
} conn_context_t;

static uint8_t conn_role[MAX_LINK_NUM];

static conn_context_t s_conn_env;

uint32_t s_total_conn_num;
uint32_t s_exc_disc_num;
uint32_t s_conn_timeout_num;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Initialize gap parameters.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile)parameters
 *          of the device including the device name, appearance, and the preferred connection parameters.
 *****************************************************************************************
 */
static void gap_params_init(void)
{
    for (uint8_t phone_idx = 0; phone_idx < MAX_LINK_NUM; phone_idx++)
    {
        s_conn_env.test_phone_info[phone_idx].conn_idx = INVALID_IDX;
        s_conn_env.test_phone_info[phone_idx].conn_flag = false;
    }
}

static void start_adv(void)
{
    sdk_err_t   error_code;
    ble_gap_adv_param_t adv_param;

    memset(&adv_param, 0x00, sizeof(ble_gap_adv_param_t));
    adv_param.adv_intv_max    = ADV_INTERVAL;
    adv_param.adv_intv_min    = ADV_INTERVAL;
    adv_param.adv_mode        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_param.chnl_map        = BLE_GAP_ADV_CHANNEL_37_38_39;
    adv_param.disc_mode       = BLE_GAP_DISC_MODE_GEN_DISCOVERABLE;
    adv_param.filter_pol      = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;

    error_code = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC, &adv_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, s_adv_data_set, sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_start(0, &adv_time_param);
    APP_ERROR_CHECK(error_code);
}

static void start_scan(void)
{
    sdk_err_t   error_code;

    // set scan parameter
    ble_gap_ext_scan_param_t scan_param;
    scan_param.type = BLE_GAP_EXT_SCAN_TYPE_OBSERVER;
    scan_param.prop = BLE_GAP_EXT_SCAN_PROP_PHY_1M_BIT;
    scan_param.dup_filt_pol = BLE_GAP_EXT_DUP_FILT_DIS;
    scan_param.scan_param_1m.scan_intv = SCAN_INTERVAL;
    scan_param.scan_param_1m.scan_wd= SCAN_WINDOW;
    scan_param.scan_param_coded.scan_intv = SCAN_INTERVAL;
    scan_param.scan_param_coded.scan_wd = SCAN_WINDOW;
    scan_param.duration= 0;
    scan_param.period= 0;

    error_code = ble_gap_ext_scan_param_set(BLE_GAP_OWN_ADDR_STATIC, &scan_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_scan_start();
    APP_ERROR_CHECK(error_code);
}

static uint16_t start_connect(uint8_t peer_addr_type, const uint8_t *peer_addr)
{
    ble_gap_ext_init_param_t ext_conn_param;
    sdk_err_t error_code;

    memset(&ext_conn_param, 0 , sizeof(ext_conn_param));
    ext_conn_param.type                            = BLE_GAP_INIT_TYPE_DIRECT_CONN_EST;
    ext_conn_param.prop                            = BLE_GAP_INIT_PROP_1M_BIT;
    ext_conn_param.conn_to                         = 500;

    ext_conn_param.scan_param_1m.scan_intv         = INIT_SCAN_INTERVAL;
    ext_conn_param.scan_param_1m.scan_wd           = INIT_SCAN_WINDOW;
    ext_conn_param.conn_param_1m.conn_intv_min     = CONNECTION_MIN_INTERVAL;
    ext_conn_param.conn_param_1m.conn_intv_max     = CONNECTION_MAX_INTERVAL;
    ext_conn_param.conn_param_1m.conn_latency      = CONNECTION_SLAVE_LATENCY;
    ext_conn_param.conn_param_1m.supervision_to    = CONNECTION_MAX_TIMEOUT;
    ext_conn_param.conn_param_1m.ce_len_min        = 8;
    ext_conn_param.conn_param_1m.ce_len_max        = 8;

    ext_conn_param.peer_addr.addr_type             = peer_addr_type;
    memcpy(ext_conn_param.peer_addr.gap_addr.addr, peer_addr, 6);

    error_code = ble_gap_ext_connect(BLE_GAP_OWN_ADDR_STATIC, &ext_conn_param);
    APP_ERROR_CHECK(error_code);

    s_total_conn_num++;
    return error_code;
}

static void app_adv_report_handler(const ble_gap_evt_adv_report_t *p_param)
{
    sdk_err_t error_code;

    if (!memcmp(&(p_param->data[ADV_NAME_OFFSET]),(uint8_t const *)TEST_PHONE_NAME1, strlen(TEST_PHONE_NAME1)))
    {
        //APP_LOG_INFO("scan to phone1");
        if ((s_conn_env.creating_conn_flag == false) && (s_conn_env.test_phone_info[0].conn_flag == false))
        {
            error_code = ble_gap_scan_stop();
            APP_ERROR_CHECK(error_code);

            APP_LOG_INFO("connect to phone1");
            error_code = start_connect(p_param->broadcaster_addr.addr_type, p_param->broadcaster_addr.gap_addr.addr);
            APP_ERROR_CHECK(error_code);

            s_conn_env.creating_conn_flag = true;
            s_conn_env.test_phone_info[0].conn_flag = true;
            s_conn_env.curr_test_phone_idx = 0;
        }
    }
    else if (!memcmp(&(p_param->data[ADV_NAME_OFFSET]),(uint8_t const *)TEST_PHONE_NAME2, strlen(TEST_PHONE_NAME2)))
    {
        if ((s_conn_env.creating_conn_flag == false) && (s_conn_env.test_phone_info[1].conn_flag == false))
        {
            error_code = ble_gap_scan_stop();
            APP_ERROR_CHECK(error_code);

            APP_LOG_INFO("connect to phone2");
            error_code = start_connect(p_param->broadcaster_addr.addr_type, p_param->broadcaster_addr.gap_addr.addr);
            APP_ERROR_CHECK(error_code);

            s_conn_env.creating_conn_flag = true;
            s_conn_env.test_phone_info[1].conn_flag = true;
            s_conn_env.curr_test_phone_idx = 1;
        }
    }
    #if 0
    else if (!memcmp(&(p_param->data[ADV_NAME_OFFSET]),(uint8_t const *)TEST_PHONE_NAME3, strlen(TEST_PHONE_NAME3)))
    {
        if ((s_conn_env.creating_conn_flag == false) && (s_conn_env.test_phone_info[2].conn_flag == false))
        {
            error_code = start_connect(p_param->broadcaster_addr.addr_type, p_param->broadcaster_addr.gap_addr.addr);
            APP_ERROR_CHECK(error_code);

            s_conn_env.creating_conn_flag = true;
            s_conn_env.test_phone_info[2].conn_flag = true;
            s_conn_env.curr_test_phone_idx = 2;
        }
    }
    else if (!memcmp(&(p_param->data[ADV_NAME_OFFSET]),(uint8_t const *)TEST_PHONE_NAME4, strlen(TEST_PHONE_NAME4)))
    {
        if ((s_conn_env.creating_conn_flag == false) && (s_conn_env.test_phone_info[3].conn_flag == false))
        {
            error_code = start_connect(p_param->broadcaster_addr.addr_type, p_param->broadcaster_addr.gap_addr.addr);
            APP_ERROR_CHECK(error_code);

            s_conn_env.creating_conn_flag = true;
            s_conn_env.test_phone_info[3].conn_flag = true;
            s_conn_env.curr_test_phone_idx = 3;
        }
    }
    #endif
}

static uint8_t get_phone_idx_by_conn_idx(uint8_t conn_idx)
{
    for (uint8_t i = 0; i < MAX_LINK_NUM; i++)
    {
        if (s_conn_env.test_phone_info[i].conn_idx == conn_idx)
        {
            return i;
        }
    }

    return INVALID_IDX;
}

static void app_disconnected_handler(uint8_t conn_idx, uint8_t reason, uint8_t role)
{
    APP_LOG_INFO("Disconnected, conn_idx = %d, reason = 0x%02X.", conn_idx, reason);

    if (conn_role[conn_idx] == CONN_ROLE_MASTER)
    {
        uint8_t phone_idx = get_phone_idx_by_conn_idx(conn_idx);
        if (phone_idx != INVALID_IDX)
        {
            s_conn_env.test_phone_info[phone_idx].conn_idx = INVALID_IDX;
            s_conn_env.test_phone_info[phone_idx].conn_flag = false;
        }
        else
        {
            APP_LOG_INFO("invalid phone idx.");
        }

        if (reason != 0xA3)
        {
            s_exc_disc_num++;
            APP_LOG_INFO("total_conn_num = %d, exc_disc_num = %d, s_conn_timeout_num = %d", s_total_conn_num, s_exc_disc_num, s_conn_timeout_num);
        }
    }
    else
    {
        if (reason != 0xA3)
        {
            APP_LOG_INFO("slave disc with reason = 0x%02X.", reason);
        }
    }

    user_bat_s_disconnect_handler(conn_idx, reason);
}

static void app_connected_handler(uint8_t conn_idx, uint8_t status, const ble_gap_evt_connected_t *p_param)
{
    sdk_err_t error_code;

    APP_LOG_INFO("Connected with the peer %02X:%02X:%02X:%02X:%02X:%02X, conn_idx = %d, status = 0x%02x.",
                 p_param->peer_addr.addr[5],
                 p_param->peer_addr.addr[4],
                 p_param->peer_addr.addr[3],
                 p_param->peer_addr.addr[2],
                 p_param->peer_addr.addr[1],
                 p_param->peer_addr.addr[0],
                 conn_idx, status);

    if (BLE_SUCCESS == status)
    {
        conn_role[conn_idx] = p_param->ll_role;

        if (conn_role[conn_idx] == CONN_ROLE_MASTER)
        {
            APP_LOG_INFO("phone_idx = %d, conn_idx = %d, status = 0x%02x.", s_conn_env.curr_test_phone_idx + 1, conn_idx, status);

            uint8_t phone_idx = s_conn_env.curr_test_phone_idx;
            s_conn_env.test_phone_info[phone_idx].conn_idx = conn_idx;
            s_conn_env.creating_conn_flag = false;

            error_code = ble_gap_scan_start();
            APP_ERROR_CHECK(error_code);
        }

        user_bat_s_connect_handler(conn_idx);
    }
    else
    {
        APP_LOG_INFO("connect fail, status = 0x%x, phone_idx = %d.", status, s_conn_env.curr_test_phone_idx + 1);
        uint8_t phone_idx = s_conn_env.curr_test_phone_idx;
        s_conn_env.test_phone_info[phone_idx].conn_idx = INVALID_IDX;
        s_conn_env.test_phone_info[phone_idx].conn_flag = false;
        s_conn_env.creating_conn_flag = false;

        error_code = ble_gap_scan_start();
        APP_ERROR_CHECK(error_code);
    }
    

}

static void app_connect_timeout_handler(void)
{
    sdk_err_t error_code;

    APP_LOG_INFO("connection timeout for 5s");
    uint8_t phone_idx = s_conn_env.curr_test_phone_idx;
    s_conn_env.test_phone_info[phone_idx].conn_idx = INVALID_IDX;
    s_conn_env.test_phone_info[phone_idx].conn_flag = false;
    s_conn_env.creating_conn_flag = false;
    s_conn_timeout_num++;
    APP_LOG_INFO("total_conn_num = %d, exc_disc_num = %d, s_conn_timeout_num = %d", s_total_conn_num, s_exc_disc_num, s_conn_timeout_num);

    error_code = ble_gap_scan_start();
    APP_ERROR_CHECK(error_code);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void ble_evt_handler(const ble_evt_t *p_evt)
{
    switch(p_evt->evt_id)
    {
        case BLE_COMMON_EVT_STACK_INIT:
            ble_app_init();
            break;

        case BLE_GAPM_EVT_ADV_START:
            APP_LOG_INFO("adv started, adv_idx = %d, status = 0x%02x", p_evt->evt.gapm_evt.index, p_evt->evt_status);
            break;

        case BLE_GAPM_EVT_ADV_STOP:
            APP_LOG_INFO("adv stopped, adv_idx = %d, status = 0x%02x", p_evt->evt.gapm_evt.index, p_evt->evt_status);
            APP_LOG_INFO("restart adv.");
            ble_gap_adv_start(0, &adv_time_param);
            break;

        case BLE_GAPM_EVT_CH_MAP_SET:
            break;

        case BLE_GAPM_EVT_WHITELIST_SET:
            break;

        case BLE_GAPM_EVT_PER_ADV_LIST_SET:
            break;

        case BLE_GAPM_EVT_PRIVACY_MODE_SET:
            break;

        case BLE_GAPM_EVT_LEPSM_REGISTER:
            break;

        case BLE_GAPM_EVT_LEPSM_UNREGISTER:
            break;

        case BLE_GAPM_EVT_SCAN_REQUEST:
            break;

        case BLE_GAPM_EVT_ADV_DATA_UPDATE:
            break;

        case BLE_GAPM_EVT_SCAN_START:
            APP_LOG_INFO("Scan started. status = 0X%02X", p_evt->evt_status);
            break;

        case BLE_GAPM_EVT_SCAN_STOP:
            APP_LOG_INFO("Scan stopped. status = 0X%02X", p_evt->evt_status);
            break;

        case BLE_GAPM_EVT_ADV_REPORT:
            app_adv_report_handler(&(p_evt->evt.gapm_evt.params.adv_report));
            break;

        case BLE_GAPM_EVT_SYNC_ESTABLISH:
            break;

        case BLE_GAPM_EVT_SYNC_STOP:
            break;

        case BLE_GAPM_EVT_SYNC_LOST:
            break;

        case BLE_GAPM_EVT_READ_RSLV_ADDR:
            break;

        case BLE_GAPC_EVT_PHY_UPDATED:
            break;

        case BLE_GAPM_EVT_DEV_INFO_GOT:
            break;

        case BLE_GAPC_EVT_CONNECTED:
            app_connected_handler(p_evt->evt.gapc_evt.index, p_evt->evt_status, &(p_evt->evt.gapc_evt.params.connected));
            break;

        case BLE_GAPC_EVT_DISCONNECTED:
            app_disconnected_handler(p_evt->evt.gapc_evt.index, p_evt->evt.gapc_evt.params.disconnected.reason, conn_role[p_evt->evt.gapc_evt.index]);
            break;

        case BLE_GAPC_EVT_CONN_PARAM_UPDATE_REQ:
            ble_gap_conn_param_update_reply(p_evt->evt.gapc_evt.index, true);
            break;

        case BLE_GAPC_EVT_CONN_PARAM_UPDATED:
            break;

        case BLE_GAPC_EVT_CONNECT_CANCLE:
            break;

        case BLE_GAPC_EVT_AUTO_CONN_TIMEOUT:
            app_connect_timeout_handler();
            break;

        case BLE_GAPC_EVT_PEER_NAME_GOT:
            break;

        case BLE_GAPC_EVT_CONN_INFO_GOT:
            break;

        case BLE_GAPC_EVT_PEER_INFO_GOT:
            break;

        case BLE_GAPC_EVT_DATA_LENGTH_UPDATED:
            break;

        case BLE_GATT_COMMON_EVT_MTU_EXCHANGE:
            break;

        case BLE_GATT_COMMON_EVT_PRF_REGISTER:
            break;

        case BLE_GATTS_EVT_READ_REQUEST:
            break;

        case BLE_GATTS_EVT_WRITE_REQUEST:
            break;

        case BLE_GATTS_EVT_PREP_WRITE_REQUEST:
            break;

        case BLE_GATTS_EVT_NTF_IND:
            break;

        case BLE_GATTS_EVT_CCCD_RECOVERY:
            break;

        case BLE_GATTC_EVT_SRVC_BROWSE:
            break;

        case BLE_GATTC_EVT_PRIMARY_SRVC_DISC:
            break;

        case BLE_GATTC_EVT_INCLUDE_SRVC_DISC:
            break;

        case BLE_GATTC_EVT_CHAR_DISC:
            break;

        case BLE_GATTC_EVT_CHAR_DESC_DISC:
            break;

        case BLE_GATTC_EVT_READ_RSP:
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            break;

        case BLE_GATTC_EVT_NTF_IND:
            if (BLE_GATT_INDICATION == p_evt->evt.gattc_evt.params.ntf_ind.type)
            {
                ble_gattc_indicate_cfm(p_evt->evt.gattc_evt.index, p_evt->evt.gattc_evt.params.ntf_ind.handle);
            }
            break;

        case BLE_SEC_EVT_LINK_ENC_REQUEST:
            break;

        case BLE_SEC_EVT_LINK_ENCRYPTED:
            break;

        case BLE_SEC_EVT_KEY_PRESS_NTF:
            break;

        case BLE_SEC_EVT_KEY_MISSING:
            break;

        case BLE_L2CAP_EVT_CONN_REQ:
            break;

        case BLE_L2CAP_EVT_CONN_IND:
            break;

        case BLE_L2CAP_EVT_ADD_CREDITS_IND:
            break;

        case BLE_L2CAP_EVT_DISCONNECTED:
            break;

        case BLE_L2CAP_EVT_SDU_RECV:
            break;

        case BLE_L2CAP_EVT_SDU_SEND:
            break;

        case BLE_L2CAP_EVT_ADD_CREDITS_CPLT:
            break;
    }
}

void ble_app_init(void)
{
    sdk_err_t         error_code;
    ble_gap_bdaddr_t  bd_addr;
    sdk_version_t     version;

    sys_sdk_verison_get(&version);
    APP_LOG_INFO("Goodix BLE SDK V%d.%d.%d (commit %x)",
                 version.major, version.minor, version.build, version.commit_id);

    error_code = ble_gap_addr_get(&bd_addr);
    APP_ERROR_CHECK(error_code);
    APP_LOG_INFO("Local Board %02X:%02X:%02X:%02X:%02X:%02X.",
                 bd_addr.gap_addr.addr[5],
                 bd_addr.gap_addr.addr[4],
                 bd_addr.gap_addr.addr[3],
                 bd_addr.gap_addr.addr[2],
                 bd_addr.gap_addr.addr[1],
                 bd_addr.gap_addr.addr[0]);
    APP_LOG_INFO("multiple role demo started.");

    gap_params_init();
    user_bat_s_init();
    start_scan();
    start_adv();
}

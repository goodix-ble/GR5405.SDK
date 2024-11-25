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

/*
 * DEFINES
 *****************************************************************************************
 */
#define SCAN_INTERVAL               15                 /**< Scan interval (uint:0.625ms, 12.5ms). */
#define SCAN_WINDOW                 15                 /**< Scan window (uint:0.625ms, 12.5ms). */

#define INIT_SCAN_INTERVAL          160                /**< Scan interval (uint:0.625ms, 100ms). */
#define INIT_SCAN_WINDOW            160                /**< Scan window (uint:0.625ms, 100ms). */

#define CONNECTION_MIN_INTERVAL     80                 /**< The connection min interval (in units of 1.25 ms. This value corresponds to 100 ms). */
#define CONNECTION_MAX_INTERVAL     80                 /**< The connection max interval (in units of 1.25 ms. This value corresponds to 100 ms). */
#define CONNECTION_SLAVE_LATENCY    0                  /**< Slave latency. */
#define CONNECTION_MAX_TIMEOUT      500                /**< Link supervision timeout (in unit of 10ms. This value corresponds to 5s). */

#define RPA_RENEW_TIME              150                /**< Rpa address renew time (150s). */
#define ADV_NAME_OFFSET             0x05               /**< Adv name index. */
#define TEST_PHONE_NAME            "TEST_PHONE"        /**< Adv name for test phone 1. */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t s_peer_addr_type = 0;
static uint8_t s_peer_addr[BLE_GAP_ADDR_LEN] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static bool s_scan_filter_flag = false;
static bool s_creating_conn_flag = false;

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
    sdk_err_t   error_code;

    // set security paramter
    ble_sec_param_t sec_param;
    sec_param.level = BLE_SEC_MODE1_LEVEL2;
    sec_param.io_cap = BLE_SEC_IO_NO_INPUT_NO_OUTPUT;
    sec_param.oob = false;
    sec_param.auth = BLE_SEC_AUTH_BOND;
    sec_param.key_size = 16;
    sec_param.ikey_dist = BLE_SEC_KDIST_ALL;
    sec_param.rkey_dist = BLE_SEC_KDIST_ALL;
    error_code = ble_sec_params_set(&sec_param);
    APP_ERROR_CHECK(error_code);

    // enable privacy
    error_code = ble_gap_privacy_params_set(RPA_RENEW_TIME, true);
    APP_ERROR_CHECK(error_code);
}

static void start_scan(void)
{
    sdk_err_t   error_code;
    ble_gap_ext_scan_param_t scan_param;

    if (s_scan_filter_flag)
    {
        APP_LOG_INFO("start to filter scan.");
        scan_param.type = BLE_GAP_EXT_SCAN_TYPE_SEL_OBSERVER;
    }
    else
    {
        APP_LOG_INFO("start to observer scan.");
        scan_param.type = BLE_GAP_EXT_SCAN_TYPE_OBSERVER;
    }

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

static void start_connect(uint8_t peer_addr_type, const uint8_t *peer_addr)
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
    memcpy(ext_conn_param.peer_addr.gap_addr.addr, peer_addr, BLE_GAP_ADDR_LEN);

    error_code = ble_gap_ext_connect(BLE_GAP_OWN_ADDR_STATIC, &ext_conn_param);
    APP_ERROR_CHECK(error_code);
}

static void app_adv_report_handler(const ble_gap_evt_adv_report_t *p_param)
{
    sdk_err_t error_code;

    if (s_scan_filter_flag)
    {
        if ((p_param->broadcaster_addr.addr_type == s_peer_addr_type)
            && !memcmp(p_param->broadcaster_addr.gap_addr.addr, s_peer_addr, BLE_GAP_ADDR_LEN))
        {
            // report rssi
            APP_LOG_INFO("rssi = %d", p_param->rssi);
            extern bool rssi_cache_queue_push(uint8_t rssi);
            rssi_cache_queue_push(p_param->rssi);
        }
    }
    else
    {
        if (!memcmp(&(p_param->data[ADV_NAME_OFFSET]),(uint8_t const *)TEST_PHONE_NAME, strlen(TEST_PHONE_NAME)))
        {
            if (!s_creating_conn_flag)
            {
                error_code = ble_gap_scan_stop();
                APP_ERROR_CHECK(error_code);

                APP_LOG_INFO("connect to test phone");
                start_connect(p_param->broadcaster_addr.addr_type, p_param->broadcaster_addr.gap_addr.addr);

                s_creating_conn_flag = true;
            }
        }
    }
}

static void restart_observer_scan(void)
{
    APP_LOG_INFO("restart observer scan");

    s_creating_conn_flag = false;
    s_scan_filter_flag = false;
    start_scan();
}

static void app_disconnected_handler(uint8_t conn_idx, uint8_t reason)
{
    APP_LOG_INFO("Disconnected, conn_idx = %d, reason = 0x%02X.", conn_idx, reason);

    if (BLE_LL_ERR_CON_TERM_BY_LOCAL_HOST == reason)
    {
        // start filter scan
        s_scan_filter_flag = true;
        start_scan();
    }
    else
    {
        // restart observer scan
        restart_observer_scan();
    }
}

static void app_connected_handler(uint8_t conn_idx, uint8_t status, const ble_gap_evt_connected_t *p_param)
{
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
        APP_LOG_INFO("conn_idx = %d, status = 0x%02x.", conn_idx, status);
        // start pair
        ble_sec_enc_start(conn_idx);
    }
    else
    {
        APP_LOG_INFO("connect fail, status = 0x%x", status);
        // restart observer scan
        restart_observer_scan();
    }
}

static void app_connect_timeout_handler(void)
{
    APP_LOG_INFO("connection timeout for 5s");
    // restart observer scan
    restart_observer_scan();
}

static void app_sec_rcv_enc_ind_handler(uint8_t conn_idx, uint8_t status)
{
    APP_LOG_INFO("link encrypted, conn_idx = %d, status = 0x%x", conn_idx, status);

    if (BLE_SUCCESS == status)
    {
        // disconnect
        ble_gap_disconnect(conn_idx);
    }
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
            break;

        case BLE_GAPM_EVT_ADV_STOP:
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
            app_disconnected_handler(p_evt->evt.gapc_evt.index, p_evt->evt.gapc_evt.params.disconnected.reason);
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
            APP_LOG_INFO("pair complete, auth = %d", p_evt->evt.sec_evt.params.enc_ind.auth);
            app_sec_rcv_enc_ind_handler(p_evt->evt.sec_evt.index, p_evt->evt_status);
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

        default:
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

    gap_params_init();
}

void ble_start_scan_test(uint8_t addr_type, uint8_t *test_addr)
{
    APP_LOG_INFO("start scan test, test_addr_type = %d", addr_type);

    APP_LOG_INFO("test addr: %02X:%02X:%02X:%02X:%02X:%02X.",
        test_addr[5], test_addr[4], test_addr[3], test_addr[2], test_addr[1], test_addr[0]);

    s_peer_addr_type = addr_type;
    memcpy(s_peer_addr, test_addr, BLE_GAP_ADDR_LEN);

    // get bond list
    ble_gap_bdaddr_t bond_addr[CFG_MAX_BOND_DEVS];
    ble_gap_bond_dev_list_t bond_list;
    bond_list.num = 0;
    bond_list.items = bond_addr;
    ble_gap_bond_devs_get(&bond_list, CFG_MAX_BOND_DEVS);

    APP_LOG_INFO("bond_num = %d", bond_list.num);

    for (uint8_t i = 0; i < bond_list.num; i++)
    {
        if ((bond_list.items[i].addr_type == addr_type)
            && !memcmp(test_addr, bond_list.items[i].gap_addr.addr, BLE_GAP_ADDR_LEN))
        {
            s_scan_filter_flag = true;
            break;
        }
    }

    start_scan();
}

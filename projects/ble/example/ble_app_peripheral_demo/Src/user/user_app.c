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
#define ADV_INTERVAL        160      /**< The fast advertising min interval (unit: 0.625ms, 100ms). */
#define ADV_1M_IDX          0        /**< 1M adv index. */
#define ADV_CODED_IDX       1        /**< Coded adv index. */
#define RPA_RENEW_TIME      150      /**< Rpa address renew time (150s). */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const uint8_t s_adv_data_set_1m[] =                  /**< Advertising data for 1M. */
{
    0x10,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'P', 'e', 'r', 'i', 'p', 'h', 'e', 'r', 'a', 'l', ' ', 'D', 'e', 'm', 'o',
};

static const uint8_t s_adv_data_set_coded[] =               /**< Advertising data for coded. */
{
    0x16,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'P', 'e', 'r', 'i', 'p', 'h', 'e', 'r', 'a', 'l', ' ', 'D', 'e', 'm', 'o', ' ', 'C', 'o', 'd', 'e', 'd'
};

ble_gap_adv_time_param_t s_adv_time_param =
{
    .duration = 0,
    .max_adv_evt = 0
};

static uint8_t local_irk[16] =
{
    0x00, 0x01, 0x02, 0x03,
    0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b,
    0x0c, 0x0d, 0x0e, 0x0f,
};

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

    // set local irk
    ble_gap_sec_key_t irk;
    memcpy(irk.key, local_irk, BLE_GAP_MAX_KEY_LEN);
    error_code = ble_gap_irk_set(&irk);
    APP_ERROR_CHECK(error_code);

    // set security paramter
    ble_sec_param_t sec_param;
    sec_param.level = BLE_SEC_MODE1_LEVEL2;
    sec_param.io_cap = BLE_SEC_IO_KEYBOARD_ONLY;
    sec_param.oob = false;
    sec_param.auth = BLE_SEC_AUTH_BOND | BLE_SEC_AUTH_MITM;
    sec_param.key_size = 16;
    sec_param.ikey_dist = BLE_SEC_KDIST_ALL;
    sec_param.rkey_dist = BLE_SEC_KDIST_ALL;
    error_code = ble_sec_params_set(&sec_param);
    APP_ERROR_CHECK(error_code);

    // enable privacy
    error_code = ble_gap_privacy_params_set(RPA_RENEW_TIME, true);
    APP_ERROR_CHECK(error_code);
}

static void start_legacy_adv(void)
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

    error_code = ble_gap_adv_param_set(ADV_1M_IDX, BLE_GAP_OWN_ADDR_GEN_RSLV, &adv_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(ADV_1M_IDX, BLE_GAP_ADV_DATA_TYPE_DATA, s_adv_data_set_1m, sizeof(s_adv_data_set_1m));
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_start(ADV_1M_IDX, &s_adv_time_param);
    APP_ERROR_CHECK(error_code);
}

static void start_extend_adv(void)
{
    sdk_err_t error_code;
    ble_gap_ext_adv_param_t adv_param;

    memset(&adv_param, 0x00, sizeof(ble_gap_ext_adv_param_t));
    adv_param.type                    = BLE_GAP_ADV_TYPE_EXTENDED;
    adv_param.disc_mode               = BLE_GAP_DISC_MODE_GEN_DISCOVERABLE;
    adv_param.prop                    = BLE_GAP_ADV_PROP_CONNECTABLE_BIT;
    adv_param.filter_pol              = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;
    adv_param.prim_cfg.adv_intv_min   = ADV_INTERVAL;
    adv_param.prim_cfg.adv_intv_max   = ADV_INTERVAL;
    adv_param.prim_cfg.chnl_map       = BLE_GAP_ADV_CHANNEL_37_38_39;
    adv_param.prim_cfg.phy            = BLE_GAP_PHY_CODED_VALUE;
    adv_param.second_cfg.phy          = BLE_GAP_PHY_CODED_VALUE;

    error_code = ble_gap_ext_adv_param_set(ADV_CODED_IDX, BLE_GAP_OWN_ADDR_GEN_RSLV, &adv_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(ADV_CODED_IDX, BLE_GAP_ADV_DATA_TYPE_DATA, s_adv_data_set_coded, sizeof(s_adv_data_set_coded));
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_start(ADV_CODED_IDX, &s_adv_time_param);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 * @brief Initialize services that will be used by the application.
 *****************************************************************************************
 */
static void app_disconnected_handler(uint8_t conn_idx, uint8_t reason)
{
    APP_LOG_INFO("Disconnected (0x%02X).", reason);
    user_bat_s_disconnect_handler(conn_idx, reason);
}

static void app_connected_handler(uint8_t conn_idx, const ble_gap_evt_connected_t *p_param)
{
    APP_LOG_INFO("Connected with the peer %02X:%02X:%02X:%02X:%02X:%02X.",
                 p_param->peer_addr.addr[5],
                 p_param->peer_addr.addr[4],
                 p_param->peer_addr.addr[3],
                 p_param->peer_addr.addr[2],
                 p_param->peer_addr.addr[1],
                 p_param->peer_addr.addr[0]);

    user_bat_s_connect_handler(conn_idx);
}

static void app_sec_rcv_enc_req_handler(uint8_t conn_idx, const ble_sec_evt_enc_req_t *p_enc_req)
{
    ble_sec_cfm_enc_t cfm_enc;
    uint32_t tk;

    if (NULL == p_enc_req)
    {
        return;
    }

    memset((uint8_t *)&cfm_enc, 0, sizeof(cfm_enc));

    switch (p_enc_req->req_type)
    {
        // User needs to decide whether to accept the pair request.
        case BLE_SEC_PAIR_REQ:
            cfm_enc.req_type = BLE_SEC_PAIR_REQ;
            cfm_enc.accept   = true;
            break;

        case BLE_SEC_TK_REQ:
            APP_LOG_INFO("Input pin code: 999999.");
            cfm_enc.req_type = BLE_SEC_TK_REQ;
            cfm_enc.accept   = true;
            tk = 999999;
            memset(cfm_enc.data.tk.key, 0, 16);
            cfm_enc.data.tk.key[0] = (uint8_t)((tk & 0x000000FF) >> 0);
            cfm_enc.data.tk.key[1] = (uint8_t)((tk & 0x0000FF00) >> 8);
            cfm_enc.data.tk.key[2] = (uint8_t)((tk & 0x00FF0000) >> 16);
            cfm_enc.data.tk.key[3] = (uint8_t)((tk & 0xFF000000) >> 24);
            break;

        default:
            break;
    }

    ble_sec_enc_cfm(conn_idx, &cfm_enc);
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
            if (p_evt->evt.gapm_evt.index == 0)
            {
                APP_LOG_INFO("1M adverting started, adv_idx = %d, status = 0x%02x", p_evt->evt.gapm_evt.index, p_evt->evt_status);
            }
            else if (p_evt->evt.gapm_evt.index == 1)
            {
                APP_LOG_INFO("Coded adverting started, adv_idx = %d, status = 0x%02x", p_evt->evt.gapm_evt.index, p_evt->evt_status);
            }
            break;

        case BLE_GAPM_EVT_ADV_STOP:
            if (p_evt->evt.gapm_evt.index == ADV_1M_IDX)
            {
                APP_LOG_INFO("1M Adverting stopped, adv_idx = %d, status = 0x%02x", p_evt->evt.gapm_evt.index, p_evt->evt_status);
                APP_LOG_INFO("1M Adverting restart.");
                ble_gap_adv_start(ADV_1M_IDX, &s_adv_time_param);
            }
            else
            {
                APP_LOG_INFO("Coded Adverting stopped, adv_idx = %d, status = 0x%02x", p_evt->evt.gapm_evt.index, p_evt->evt_status);
                APP_LOG_INFO("Coded Adverting restart.");
                ble_gap_adv_start(ADV_CODED_IDX, &s_adv_time_param);
            }
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
            break;

        case BLE_GAPM_EVT_SCAN_STOP:
            break;

        case BLE_GAPM_EVT_ADV_REPORT:
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
            app_connected_handler(p_evt->evt.gapc_evt.index, &(p_evt->evt.gapc_evt.params.connected));
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
            app_sec_rcv_enc_req_handler(p_evt->evt.sec_evt.index, &(p_evt->evt.sec_evt.params.enc_req));
            break;

        case BLE_SEC_EVT_LINK_ENCRYPTED:
            if (BLE_SUCCESS == p_evt->evt_status)
            {
                APP_LOG_INFO("Link has been successfully encrypted.");
            }
            else
            {
                APP_LOG_INFO("Pairing failed for error 0x%x.", p_evt->evt_status);
            }
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
    APP_LOG_INFO("Peripheral Demo started.");

    gap_params_init();
    user_bat_s_init();

    start_legacy_adv();
    start_extend_adv();
}

/**
 *****************************************************************************************
 *
 * @file user_app.c
 *
 * @brief User function Implementation.
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
#include "user_app.h"
#include "user_periph_setup.h"
#include "grx_sys.h"
#include "app_log.h"
#include "utility.h"
#include "app_error.h"
#include "ble_error.h"
#include "ble_debug.h"

/*
 * DEFINES
 *****************************************************************************************
 */

/**@brief Gapm config data. */
#define DEVICE_NAME                       "Goodix_Debug"   /**< Device Name which will be set in GAP. */

#define APP_ADV_FAST_MIN_INTERVAL          32               /**< The fast advertising min interval (in units of 0.625 ms). */
#define APP_ADV_FAST_MAX_INTERVAL          48               /**< The fast advertising max interval (in units of 0.625 ms). */
#define APP_ADV_SLOW_MIN_INTERVAL          160              /**< The slow advertising min interval (in units of 0.625 ms). */
#define APP_ADV_SLOW_MAX_INTERVAL          400              /**< The slow advertising max interval (in units of 0.625 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS         0                /**< The advertising timeout in units of seconds. */
#define MIN_CONN_INTERVAL                  320              /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                  520              /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                      0                /**< Slave latency. */
#define CONN_SUP_TIMEOUT                   4000             /**< Connection supervisory timeout (4 seconds). */


/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static ble_gap_adv_param_t      s_gap_adv_param;            /**< Advertising parameters for legacy advertising. */
static ble_gap_adv_time_param_t s_gap_adv_time_param;       /**< Advertising time parameter. */

static const uint8_t s_adv_data_set[] =                 /**< Advertising data. */
{
    0x0D,   // Length of this data
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'D', 'e', 'b', 'u', 'g'
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

    ble_gap_pair_enable(false);

    s_gap_adv_param.adv_intv_max  = APP_ADV_SLOW_MAX_INTERVAL;
    s_gap_adv_param.adv_intv_min  = APP_ADV_FAST_MIN_INTERVAL;
    s_gap_adv_param.adv_mode      = BLE_GAP_ADV_TYPE_ADV_IND;
    s_gap_adv_param.chnl_map      = BLE_GAP_ADV_CHANNEL_37_38_39;
    s_gap_adv_param.disc_mode     = BLE_GAP_DISC_MODE_GEN_DISCOVERABLE;
    s_gap_adv_param.filter_pol    = BLE_GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;

    uint8_t dev_name[32];
    uint16_t dev_name_len = 32;
    error_code = ble_gap_device_name_get(dev_name, &dev_name_len);
    APP_ERROR_CHECK(error_code);

    if (!strcmp((const char *)dev_name, BLE_GAP_DEVNAME_DEFAULT))
    {
        // Set the default Device Name.
        error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_NOAUTH, (uint8_t const *)DEVICE_NAME, strlen(DEVICE_NAME));
        APP_ERROR_CHECK(error_code);
    }
    else
    {
        // Set the Device Name is writable from the peer.
        error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_NOAUTH, NULL, 0);
        APP_ERROR_CHECK(error_code);
    }

    error_code = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC, &s_gap_adv_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, s_adv_data_set, sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);

    s_gap_adv_time_param.duration     = 0;
    s_gap_adv_time_param.max_adv_evt  = 0;
}

/**
 *****************************************************************************************
 * @brief Initialize BLE security configuration.
 *****************************************************************************************
 */
static void app_sec_init(void)
{
    sdk_err_t    error_code;

    ble_gap_pair_enable(true);

    //set the default security parameters.
    ble_sec_param_t sec_param =
    {
        .level     = BLE_SEC_MODE1_LEVEL3,
        .io_cap    = BLE_SEC_IO_DISPLAY_ONLY,
        .oob       = false,
        .auth      = BLE_SEC_AUTH_BOND | BLE_SEC_AUTH_MITM | BLE_SEC_AUTH_SEC_CON,
        .key_size  = 16,
        .ikey_dist = BLE_SEC_KDIST_ALL,
        .rkey_dist = BLE_SEC_KDIST_ALL,
    };

    error_code = ble_sec_params_set(&sec_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_privacy_params_set(150, true);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 * @brief Initialize ble_connect module.
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Start advertising.
 *****************************************************************************************
 */
static void app_adv_start(void)
{
    sdk_err_t error_code;

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 * @brief Print device mac address.
 *****************************************************************************************
 */
static void dev_mac_info_print(void)
{
    sdk_err_t         error_code;
    ble_gap_bdaddr_t  bd_addr;

    error_code = ble_gap_addr_get(&bd_addr);
    APP_ERROR_CHECK(error_code);
    APP_LOG_INFO("Local Board %02X:%02X:%02X:%02X:%02X:%02X.",
                 bd_addr.gap_addr.addr[5],
                 bd_addr.gap_addr.addr[4],
                 bd_addr.gap_addr.addr[3],
                 bd_addr.gap_addr.addr[2],
                 bd_addr.gap_addr.addr[1],
                 bd_addr.gap_addr.addr[0]);
}

static void app_debug_write_tx_cccd_cb(uint8_t conn_idx)
{
    APP_LOG_INFO("app_debug_write_tx_cccd_cb, conn_idx = %d", conn_idx);
}

/**
 *****************************************************************************************
 * @brief Initialize services.
 *****************************************************************************************
 */
static void services_init(void)
{
    ble_debug_service_init();
    ble_debug_reg_write_tx_cccd_cb(app_debug_write_tx_cccd_cb);
}

static void app_sec_rcv_enc_req_handler(uint8_t conn_idx, const ble_sec_evt_enc_req_t *p_enc_req)
{
    ble_sec_cfm_enc_t cfm_enc;
    uint32_t tk;
    if (NULL == p_enc_req)
    {
        return;
    }
    memset((uint8_t *)&cfm_enc, 0, sizeof(ble_sec_cfm_enc_t));

    switch (p_enc_req->req_type)
    {
        // user need to decide whether to accept the pair request
        case BLE_SEC_PAIR_REQ:
            cfm_enc.req_type = BLE_SEC_PAIR_REQ;
            cfm_enc.accept   = true;
            break;

        // user need to input the password
        case BLE_SEC_TK_REQ:
            cfm_enc.req_type = BLE_SEC_TK_REQ;
            cfm_enc.accept   = true;
            tk = 123456;
            APP_LOG_INFO("Please Input pin code: %d.", tk);
            memset(cfm_enc.data.tk.key, 0, 16);
            htole32(&cfm_enc.data.tk.key[0], tk);
            break;

        default:
            break;
    }

    ble_sec_enc_cfm(conn_idx, &cfm_enc);
}

static void app_disconnected_handler(uint8_t conn_idx, uint8_t reason)
{
    sdk_err_t error_code;
    APP_LOG_INFO("Disconnected (0x%02X).", reason);

    error_code = ble_gap_adv_start(conn_idx, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
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
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void ble_evt_handler(const ble_evt_t *p_evt)
{
    switch (p_evt->evt_id)
    {
        case BLE_COMMON_EVT_STACK_INIT:
            ble_app_init();
            break;

        case BLE_GAPC_EVT_CONN_PARAM_UPDATE_REQ:
            ble_gap_conn_param_update_reply(p_evt->evt.gapc_evt.index, true);
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

        case BLE_GAPC_EVT_CONNECTED:
            app_connected_handler(p_evt->evt.gapc_evt.index, &(p_evt->evt.gapc_evt.params.connected));
            break;

        case BLE_GAPC_EVT_DISCONNECTED:
            app_disconnected_handler(p_evt->evt.gapc_evt.index, p_evt->evt.gapc_evt.params.disconnected.reason);
            break;
    }
}

void ble_app_init(void)
{
    sdk_version_t     version;

    sys_sdk_verison_get(&version);
    APP_LOG_INFO("Goodix BLE SDK V%d.%d.%d (commit %x)",
                version.major, version.minor, version.build, version.commit_id);
    APP_LOG_INFO("BLE debug example started.");
    dev_mac_info_print();
    services_init();
    gap_params_init();
    app_sec_init();
    app_adv_start();
}

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
#define SCAN_INTERVAL         15                         /**< Scan interval (uint:0.625ms, 12.5ms). */
#define SCAN_WINDOW           15                         /**< Scan window (uint:0.625ms, 12.5ms). */
#define RPA_RENEW_TIME        150                        /**< Rpa address renew time (150s). */
#define PEER_ADV_NAME_1M      "Peripheral Demo"          /**< Device name for peer device. */
#define PEER_ADV_NAME_CODED   "Peripheral Demo Coded"    /**< Device name for peer device. */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

/**< Identity address for peer device. */
static uint8_t peer_iden_addr[BLE_GAP_ADDR_LEN] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};

/**< Irk for peer device. */
static uint8_t peer_irk[BLE_GAP_MAX_KEY_LEN] =
{
    0x00, 0x01, 0x02, 0x03,
    0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b,
    0x0c, 0x0d, 0x0e, 0x0f,
};

/**< Irk for peer device. */
static uint8_t local_irk[BLE_GAP_MAX_KEY_LEN] =
{
    0x10, 0x11, 0x12, 0x13,
    0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1a, 0x1b,
    0x1c, 0x1d, 0x1e, 0x1f,
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
    sdk_err_t error_code;

    // add white list
    ble_gap_white_list_t white_list;
    error_code = ble_gap_whitelist_get(&white_list);
    APP_ERROR_CHECK(error_code);

    if (white_list.num == 0)
    {
        white_list.num = 1;
        white_list.items[0].addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;
        memcpy(white_list.items[0].gap_addr.addr, peer_iden_addr, BLE_GAP_ADDR_LEN);
        error_code = ble_gap_whitelist_add(&white_list);
        APP_ERROR_CHECK(error_code);
    }

    // add rpa list
    ble_gap_ral_dev_info_t rpa_info;
    ble_gap_ral_dev_list_t rpa_list;
    rpa_list.num = 0;
    rpa_list.items = &rpa_info;
    error_code = ble_gap_rpa_list_get(&rpa_list, 1);
    APP_ERROR_CHECK(error_code);

    if (rpa_list.num == 0)
    {
        rpa_info.bd_addr.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;
        memcpy(rpa_info.bd_addr.gap_addr.addr, peer_iden_addr, BLE_GAP_ADDR_LEN);
        rpa_info.priv_mode = BLE_GAP_PRIVACY_MODE_NETWORK;
        memcpy(rpa_info.peer_irk, peer_irk, BLE_GAP_MAX_KEY_LEN);
        memcpy(rpa_info.local_irk, local_irk, BLE_GAP_MAX_KEY_LEN);

        rpa_list.num = 1;
        rpa_list.offset = 0;

        extern uint16_t ble_gap_set_ral_list(const ble_gap_ral_dev_list_t *ral_list);
        error_code = ble_gap_set_ral_list(&rpa_list);
        APP_ERROR_CHECK(error_code);
    }

    // set local irk
    ble_gap_sec_key_t irk;
    memcpy(irk.key, local_irk, BLE_GAP_MAX_KEY_LEN);
    error_code = ble_gap_irk_set(&irk);
    APP_ERROR_CHECK(error_code);

    // enable privacy
    error_code = ble_gap_privacy_params_set(RPA_RENEW_TIME, true);
    APP_ERROR_CHECK(error_code);
}

static void start_scan(void)
{
    sdk_err_t   error_code;

    // set scan parameter
    ble_gap_ext_scan_param_t scan_param;
    scan_param.type = BLE_GAP_EXT_SCAN_TYPE_SEL_OBSERVER;
    scan_param.prop = BLE_GAP_EXT_SCAN_PROP_PHY_1M_BIT | \
                      BLE_GAP_EXT_SCAN_PROP_PHY_CODED_BIT | \
                      BLE_GAP_EXT_SCAN_PROP_ACTIVE_1M_BIT | \
                      BLE_GAP_EXT_SCAN_PROP_ACTIVE_CODED_BIT;
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

static void app_adv_report_handler(const ble_gap_evt_adv_report_t *p_param)
{
    if ((p_param->broadcaster_addr.addr_type == BLE_GAP_ADDR_TYPE_PUBLIC)
        && !memcmp(p_param->broadcaster_addr.gap_addr.addr, peer_iden_addr, BLE_GAP_ADDR_LEN))
    {
        if (!memcmp(&(p_param->data[5]),(uint8_t const *)PEER_ADV_NAME_CODED, strlen(PEER_ADV_NAME_CODED)))
        {
            APP_LOG_INFO("scan coded adv, adv_addr: %02X:%02X:%02X:%02X:%02X:%02X.",
                 peer_iden_addr[5],
                 peer_iden_addr[4],
                 peer_iden_addr[3],
                 peer_iden_addr[2],
                 peer_iden_addr[1],
                 peer_iden_addr[0]);
        }
        else if (!memcmp(&(p_param->data[5]), (uint8_t const *)PEER_ADV_NAME_1M, strlen(PEER_ADV_NAME_1M)))
        {
            APP_LOG_INFO("scan 1M adv, adv_addr: %02X:%02X:%02X:%02X:%02X:%02X.",
                 peer_iden_addr[5],
                 peer_iden_addr[4],
                 peer_iden_addr[3],
                 peer_iden_addr[2],
                 peer_iden_addr[1],
                 peer_iden_addr[0]);
        }
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
            APP_LOG_INFO("Scan started. status = 0x%02X\n", p_evt->evt_status);
            break;

        case BLE_GAPM_EVT_SCAN_STOP:
            APP_LOG_INFO("Scan stopped. status = 0x%02X\n", p_evt->evt_status);
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
            break;

        case BLE_GAPC_EVT_DISCONNECTED:
            break;

        case BLE_GAPC_EVT_CONN_PARAM_UPDATE_REQ:
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
    APP_LOG_INFO("Observer demo application started.");

    gap_params_init();
    start_scan();
}

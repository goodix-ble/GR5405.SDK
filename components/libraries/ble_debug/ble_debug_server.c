/**
 ****************************************************************************************
 *
 * @file ble_debug_server.c
 *
 * @brief BLE Debug Server Implementation.
 *
 ****************************************************************************************
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
 ****************************************************************************************
 */
#include "ble_debug_server.h"
#include "ble_prf_types.h"
#include "ble_prf_utils.h"
#include "utility.h"

/*
 * DEFINES
 ****************************************************************************************
 */
/**@brief Proprietary UUIDs. */
#define DBG_SERVICE_UUID         {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80, 0x0A, 0x46, 0x44, 0xD3, 0x01, 0x04, 0xED, 0xA6}
#define DBG_SERVICE_TX_UUID      {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80, 0x0A, 0x46, 0x44, 0xD3, 0x02, 0x04, 0xED, 0xA6}
#define DBG_SERVICE_RX_UUID      {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80, 0x0A, 0x46, 0x44, 0xD3, 0x03, 0x04, 0xED, 0xA6}
#define DBG_SERVICE_CTRL_UUID    {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80, 0x0A, 0x46, 0x44, 0xD3, 0x04, 0x04, 0xED, 0xA6}

/**@brief Macros for conversion of 128bit to 16bit UUID. */
#define ATT_128_PRIMARY_SERVICE     BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DECL_PRIMARY_SERVICE)
#define ATT_128_CHARACTERISTIC      BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DECL_CHARACTERISTIC)
#define ATT_128_CLIENT_CHAR_CFG     BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DESC_CLIENT_CHAR_CFG)

/*
 * ENUMERATIONS
 ****************************************************************************************
 */
/**@brief DBG Service Attributes Indexes. */
enum dbgs_attr_idx_tag
{
    DBGS_IDX_SVC,

    DBGS_IDX_TX_CHAR,
    DBGS_IDX_TX_VAL,
    DBGS_IDX_TX_CFG,
    DBGS_IDX_RX_CHAR,
    DBGS_IDX_RX_VAL,
    DBGS_IDX_CTRL_PT_CHAR,
    DBGS_IDX_CTRL_PT_VAL,
    DBGS_IDX_CTRL_PT_CFG,

    DBGS_IDX_NB,
};

/*
 * STRUCT DEFINE
 ****************************************************************************************
 */
struct dbgs_env_t
{
    dbgs_init_t             dbgs_init;
    uint16_t                tx_ntf_cfg[CFG_MAX_CONNECTIONS];
    uint16_t                ctrl_pt_ind_cfg[CFG_MAX_CONNECTIONS];
    uint16_t                start_hdl;
    ble_gatts_create_db_t   dbgs_att_db;
};

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
static uint16_t          s_char_mask = 0x1ff;
static const uint8_t     s_dbgs_svc_uuid[] = {BLE_UUID_DBG_SERVICE};
static struct dbgs_env_t s_dbgs_env;

/**@brief Full DBGS Database Description - Used to add attributes into the database. */
static const ble_gatts_attm_desc_128_t s_dbgs_att_db[DBGS_IDX_NB] = {
    // DBG service
    [DBGS_IDX_SVC] = {ATT_128_PRIMARY_SERVICE, BLE_GATTS_READ_PERM_UNSEC, 0, 0},

    // DBG TX Characteristic Declaration
    [DBGS_IDX_TX_CHAR] = {ATT_128_CHARACTERISTIC,BLE_GATTS_READ_PERM_UNSEC, 0, 0},
    // DBG TX Characteristic Value
    [DBGS_IDX_TX_VAL]  = {DBG_SERVICE_TX_UUID,
                          BLE_GATTS_NOTIFY_PERM(BLE_GATTS_AUTH),
                          (BLE_GATTS_ATT_VAL_LOC_USER | BLE_GATTS_ATT_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128)),
                          DBGS_MAX_DATA_LEN},
    // DBG TX Characteristic - Client Characteristic Configuration Descriptor
    [DBGS_IDX_TX_CFG]  = {ATT_128_CLIENT_CHAR_CFG,
                          BLE_GATTS_READ_PERM_UNSEC| BLE_GATTS_WRITE_REQ_PERM(BLE_GATTS_AUTH),
                          0,
                          0},

    // DBG RX Characteristic Declaration
    [DBGS_IDX_RX_CHAR] = {ATT_128_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},

    // DBG RX Characteristic Value
    [DBGS_IDX_RX_VAL]  = {DBG_SERVICE_RX_UUID,
                          BLE_GATTS_WRITE_CMD_PERM(BLE_GATTS_AUTH),
                          (BLE_GATTS_ATT_VAL_LOC_USER | BLE_GATTS_ATT_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128)),
                          DBGS_MAX_DATA_LEN},

    // DBG Control Point Characteristic Declaration
    [DBGS_IDX_CTRL_PT_CHAR] = {ATT_128_CHARACTERISTIC, BLE_GATTS_READ_PERM_UNSEC, 0, 0},

    // DBG Control Point Characteristic Value
    [DBGS_IDX_CTRL_PT_VAL]  = {DBG_SERVICE_CTRL_UUID,
                            BLE_GATTS_WRITE_CMD_PERM(BLE_GATTS_AUTH) | BLE_GATTS_INDICATE_PERM(BLE_GATTS_AUTH),
                            (BLE_GATTS_ATT_VAL_LOC_USER | BLE_GATTS_ATT_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128)),
                            sizeof(uint32_t)},
    // DBG Control Point Characteristic - Client Characteristic Configuration Descriptor
    [DBGS_IDX_CTRL_PT_CFG]  = {ATT_128_CLIENT_CHAR_CFG,
                          BLE_GATTS_READ_PERM_UNSEC| BLE_GATTS_WRITE_REQ_PERM(BLE_GATTS_AUTH),
                          0,
                          0},
};

static dbgs_write_tx_cccd_cb_t s_write_tx_cccd_cb = NULL;

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Handles reception of the attribute info request message.
 *
 * @param[in] conn_idx: Connection index
 * @param[in] p_param:  The parameters of the read request.
 *****************************************************************************************
 */
static void dbgs_read_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_read_t *p_param)
{
    ble_gatts_read_cfm_t    cfm;
    uint16_t                handle     = p_param->handle;
    uint8_t                 tab_index  = 0;

    tab_index = prf_find_idx_by_handle(handle, s_dbgs_env.start_hdl, DBGS_IDX_NB, (uint8_t*)&s_char_mask);

    cfm.handle = handle;
    cfm.status = BLE_SUCCESS;

    switch (tab_index)
    {
        case DBGS_IDX_TX_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value = (uint8_t *)(&s_dbgs_env.tx_ntf_cfg[conn_idx]);
            break;

        case DBGS_IDX_CTRL_PT_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value = (uint8_t *)(&s_dbgs_env.ctrl_pt_ind_cfg[conn_idx]);
            break;

        default:
            cfm.length = 0;
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
    }

    ble_gatts_read_cfm(conn_idx, &cfm);
}

/**
 *****************************************************************************************
 * @brief Handles control point cmd.
 *
 * @param[in] conn_idx: Connection index
 * @param[in] p_data: Pointer to the cmd data.
 * @param[in] length: Length of the cmd data.
 *****************************************************************************************
 */
static void dbgs_control_point_handler(uint8_t conn_idx, uint8_t *p_data, uint16_t length)
{
    dbgs_evt_t event;

    event.conn_idx = conn_idx;
    event.evt_type = DBGS_EVT_INVALID;

    if (le32toh(p_data) == DBGS_CTRL_PT_OP_DBG_ENTER)
    {
         event.evt_type = DBGS_EVT_TASK_ENTER;
    }

    if ((s_dbgs_env.dbgs_init.evt_handler != NULL) && (event.evt_type != DBGS_EVT_INVALID))
    {
        s_dbgs_env.dbgs_init.evt_handler(&event);
    }
}

/**
 *****************************************************************************************
 * @brief Handles reception of the write request.
 *
 * @param[in] conn_idx: Connection index
 * @param[in] p_param: Pointer to the parameters of the write request.
 *****************************************************************************************
 */
static void dbgs_write_att_evt_handler(uint8_t conn_idx, const ble_gatts_evt_write_t *p_param)
{
    ble_gatts_write_cfm_t   cfm;
    uint16_t                handle     = p_param->handle;
    uint8_t                 tab_index  = 0;
    uint16_t                cccd_value = 0;
    dbgs_evt_t              event;

    tab_index = prf_find_idx_by_handle(handle, s_dbgs_env.start_hdl, DBGS_IDX_NB, (uint8_t*)&s_char_mask);

    cfm.handle = handle;
    cfm.status = BLE_SUCCESS;

    switch (tab_index)
    {
        case DBGS_IDX_RX_VAL:
            if (s_dbgs_env.dbgs_init.evt_handler != NULL)
            {
                event.conn_idx = conn_idx;
                event.evt_type = DBGS_EVT_RX_RECEIVE_DATA;
                event.p_data = (uint8_t*)p_param->value;
                event.length = p_param->length;
                s_dbgs_env.dbgs_init.evt_handler(&event);
            }
            break;

        case DBGS_IDX_TX_CFG:
            cccd_value = le16toh(&p_param->value[0]);
            if (s_dbgs_env.dbgs_init.evt_handler != NULL)
            {
                event.conn_idx = conn_idx;
                event.evt_type = (cccd_value == PRF_CLI_START_NTF) ?\
                                    DBGS_EVT_TX_NOTIFICATION_ENABLED :
                                    DBGS_EVT_TX_NOTIFICATION_DISABLED;
                s_dbgs_env.dbgs_init.evt_handler(&event);
            }
            s_dbgs_env.tx_ntf_cfg[conn_idx] = cccd_value;

            if (s_write_tx_cccd_cb != NULL)
            {
                s_write_tx_cccd_cb(conn_idx);
            }
            break;

        case DBGS_IDX_CTRL_PT_CFG:
            cccd_value = le16toh(&p_param->value[0]);
            if (s_dbgs_env.dbgs_init.evt_handler != NULL)
            {
                event.conn_idx = conn_idx;
                event.evt_type = (cccd_value == PRF_CLI_START_IND) ?\
                                    DBGS_EVT_CTRL_PT_INDICATION_ENABLED :
                                    DBGS_EVT_CTRL_PT_INDICATION_DISABLED;

                s_dbgs_env.dbgs_init.evt_handler(&event);
            }
            s_dbgs_env.ctrl_pt_ind_cfg[conn_idx] = cccd_value;
            break;

        case DBGS_IDX_CTRL_PT_VAL:
            dbgs_control_point_handler(conn_idx, p_param->value, p_param->length);
            break;

        default:
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
    }

    (void)ble_gatts_write_cfm(conn_idx, &cfm);
}

/**
 *****************************************************************************************
 * @brief Handles reception of the cccd recover request.
 *
 * @param[in]: conn_idx:   Connection index
 * @param[in]: handle:     The handle of cccd attribute.
 * @param[in]: cccd_value: The value of cccd attribute.
 *****************************************************************************************
 */
static void dbgs_cccd_set_evt_handler(uint8_t conn_idx, uint16_t handle, uint16_t cccd_value)
{
    uint8_t tab_index = 0;
    dbgs_evt_t event;

    if (!prf_is_cccd_value_valid(cccd_value))
    {
        return;
    }

    tab_index = prf_find_idx_by_handle(handle, s_dbgs_env.start_hdl, DBGS_IDX_NB, (uint8_t*)&s_char_mask);

    switch (tab_index)
    {
        case DBGS_IDX_TX_CFG:
            if (s_dbgs_env.dbgs_init.evt_handler != NULL)
            {
                event.conn_idx = conn_idx;
                event.evt_type = (cccd_value == PRF_CLI_START_NTF) ?\
                                    DBGS_EVT_TX_NOTIFICATION_ENABLED :
                                    DBGS_EVT_TX_NOTIFICATION_DISABLED;
                s_dbgs_env.dbgs_init.evt_handler(&event);
            }
            s_dbgs_env.tx_ntf_cfg[conn_idx] = cccd_value;
            break;

        case DBGS_IDX_CTRL_PT_CFG:
            if (s_dbgs_env.dbgs_init.evt_handler != NULL)
            {
                event.conn_idx = conn_idx;
                event.evt_type = (cccd_value == PRF_CLI_START_IND) ?\
                                    DBGS_EVT_CTRL_PT_INDICATION_ENABLED :
                                    DBGS_EVT_CTRL_PT_INDICATION_DISABLED;
                s_dbgs_env.dbgs_init.evt_handler(&event);
            }
            s_dbgs_env.ctrl_pt_ind_cfg[conn_idx] = cccd_value;
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Handles the notification complete event.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] status: Event status.
 * @param[in] p_ntf_ind: Pointer to the notification.
 *****************************************************************************************
 */
static void dbgs_ntf_cplt_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gatts_evt_ntf_ind_t *p_ntf_ind)
{
    if (s_dbgs_env.dbgs_init.evt_handler != NULL)
    {
        dbgs_evt_t event;

        event.conn_idx = conn_idx;
        if (status == BLE_SUCCESS)
        {
            if (p_ntf_ind->type == BLE_GATT_NOTIFICATION)
            {
                event.evt_type = DBGS_EVT_NOTIFY_COMPLETE;
                s_dbgs_env.dbgs_init.evt_handler(&event);
            }
        }
    }
}

/**
 *****************************************************************************************
 * @brief Handles of the BLE event for DBG service.
 *
 * @param[in] p_evt: Pointer to the BLE event.
 *****************************************************************************************
 */
static void dbgs_ble_evt_handler(const ble_evt_t *p_evt)
{
    if (NULL == p_evt)
    {
        return;
    }

    switch (p_evt->evt_id)
    {
        case BLE_GATTS_EVT_READ_REQUEST:
            dbgs_read_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.read_req);
            break;

        case BLE_GATTS_EVT_WRITE_REQUEST:
            dbgs_write_att_evt_handler(p_evt->evt.gatts_evt.index, &p_evt->evt.gatts_evt.params.write_req);
            break;

        case BLE_GATTS_EVT_NTF_IND:
            dbgs_ntf_cplt_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt_status, &p_evt->evt.gatts_evt.params.ntf_ind_sended);
            break;

        case BLE_GATTS_EVT_CCCD_RECOVERY:
            dbgs_cccd_set_evt_handler(p_evt->evt.gatts_evt.index, p_evt->evt.gatts_evt.params.cccd_recovery.handle, p_evt->evt.gatts_evt.params.cccd_recovery.cccd_val);
            break;

        case BLE_GAPC_EVT_DISCONNECTED:
        {
            dbgs_evt_t event;
            event.evt_type = DBGS_EVT_DISCONNECT;
            event.conn_idx = p_evt->evt.gatts_evt.index;
            s_dbgs_env.dbgs_init.evt_handler(&event);
            break;
        }

        default:
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
sdk_err_t dbgs_notify_tx_data(uint8_t conn_idx, uint8_t *p_data, uint16_t length)
{
    if (NULL == p_data)
    {
        return SDK_ERR_POINTER_NULL;
    }

    sdk_err_t error_code = SDK_ERR_NTF_DISABLED;
    ble_gatts_noti_ind_t send_cmd;
    if (s_dbgs_env.tx_ntf_cfg[conn_idx] == PRF_CLI_START_NTF)
    {
        // Fill in the parameter structure
        send_cmd.type = BLE_GATT_NOTIFICATION;
        send_cmd.handle = prf_find_handle_by_idx(DBGS_IDX_TX_VAL, s_dbgs_env.start_hdl, (uint8_t *)&s_char_mask);
        // pack measured value in database
        send_cmd.length = length;
        send_cmd.value = p_data;
        // send notification to peer device
        error_code = ble_gatts_noti_ind(conn_idx, &send_cmd);
    }
    return error_code;
}

sdk_err_t dbgs_service_init(dbgs_init_t *p_dbgs_init)
{
    if (NULL == p_dbgs_init)
    {
        return SDK_ERR_POINTER_NULL;
    }

    s_dbgs_env.dbgs_init.evt_handler = p_dbgs_init->evt_handler;

    memset(&s_dbgs_env.dbgs_att_db, 0, sizeof(ble_gatts_create_db_t));

    s_dbgs_env.start_hdl = PRF_INVALID_HANDLE;
    s_dbgs_env.dbgs_att_db.shdl                  = &s_dbgs_env.start_hdl;
    s_dbgs_env.dbgs_att_db.uuid                  = s_dbgs_svc_uuid;
    s_dbgs_env.dbgs_att_db.attr_tab_cfg          = (uint8_t *)&s_char_mask;
    s_dbgs_env.dbgs_att_db.max_nb_attr           = DBGS_IDX_NB;
    s_dbgs_env.dbgs_att_db.srvc_perm             = BLE_GATTS_SRVC_UUID_TYPE_SET(BLE_GATTS_UUID_TYPE_128);
    s_dbgs_env.dbgs_att_db.attr_tab_type         = BLE_GATTS_SERVICE_TABLE_TYPE_128;
    s_dbgs_env.dbgs_att_db.attr_tab.attr_tab_128 = s_dbgs_att_db;

    return ble_gatts_prf_add(&s_dbgs_env.dbgs_att_db, dbgs_ble_evt_handler);
}

uint16_t dbgs_service_get_start_handle(void)
{
    return s_dbgs_env.start_hdl;
}

void dbgs_reg_write_tx_cccd_cb(dbgs_write_tx_cccd_cb_t p_cb)
{
    s_write_tx_cccd_cb = p_cb;
}

/**
 *****************************************************************************************
 *
 * @file bat_c.c
 *
 * @brief Battery Service Client Implementation.
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
#include "bat_c.h"
#include "ble_prf_utils.h"
#include "utility.h"
#include <string.h>

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */
/**@brief Battery Service environment variable. */
struct bat_c_env_t
{
    bat_c_handles_t     handles;            /**< Handles of BAT characteristics which will be got for peer. */
    bat_c_evt_handler_t evt_handler;        /**< Handler of BAT Client event  */
};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct bat_c_env_t s_bat_c_env;     /**< Battery Service Client environment variable. */
static uint8_t            s_target_uuid[2]   = {LO_U16(BLE_ATT_SVC_BATTERY_SERVICE), HI_U16(BLE_ATT_SVC_BATTERY_SERVICE)};
static ble_uuid_t         s_bat_service_uuid =
{
    .uuid_len = 2,
    .uuid     = s_target_uuid,
};
/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Excute Battery Service Client event handler.
 *
 * @param[in] p_evt: Pointer to Battery Service Client event structure.
 *****************************************************************************************
 */
void bat_c_evt_handler_excute(bat_c_evt_t *p_evt)
{
    if (NULL != s_bat_c_env.evt_handler && BAT_C_EVT_INVALID != p_evt->evt_type)
    {
        s_bat_c_env.evt_handler(p_evt);
    }
}

/**
 *****************************************************************************************
 * @brief This callback function will be called when receiving read response.
 *
 * @param[in] conn_idx:   The connection index.
 * @param[in] status:     The status of GATTC operation.
 * @param[in] p_read_rsp: The information of read response.
 *****************************************************************************************
 */
static void bat_c_att_read_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gattc_evt_read_t *p_read_rsp)
{
    bat_c_evt_t bat_c_evt;

    bat_c_evt.conn_idx = conn_idx;
    bat_c_evt.evt_type = BAT_C_EVT_INVALID;

    if (BLE_SUCCESS != status)
    {
        return;
    }

    if (p_read_rsp->value[0].handle == s_bat_c_env.handles.bat_bat_level_handle)
    {
        bat_c_evt.evt_type  = BAT_C_EVT_BAT_LEVE_RECEIVE;
        bat_c_evt.bat_level = p_read_rsp->value[0].p_value[0];
        bat_c_evt_handler_excute(&bat_c_evt);
    }
}

/**
 *****************************************************************************************
 * @brief This callback function will be called when receiving read response.
 *
 * @param[in] conn_idx:   The connection index.
 * @param[in] status:     The status of GATTC operation.
 * @param[in] handle:     The handle of attribute.
 *****************************************************************************************
 */
static void bat_c_att_write_evt_handler(uint8_t conn_idx, uint8_t status, uint16_t handle)
{
    bat_c_evt_t bat_c_evt;

    bat_c_evt.conn_idx = conn_idx;
    bat_c_evt.evt_type = BAT_C_EVT_INVALID;

    if (handle == s_bat_c_env.handles.bat_bat_level_cccd_handle)
    {
        bat_c_evt.evt_type = (BLE_SUCCESS == status) ?\
                             BAT_C_EVT_BAT_LEVEL_NTF_SET_SUCCESS :\
                             BAT_C_EVT_BAT_LEVEL_NTF_SET_ERR;
    }

    bat_c_evt_handler_excute(&bat_c_evt);
}

/**
 *****************************************************************************************
 * @brief This callback function will be called when receiving notification or indication.
 *
 * @param[in] conn_idx:  The connection index.
 * @param[in] status:    The status of GATTC operation.
 * @param[in] p_ntf_ind: The information of notification or indication.
 *****************************************************************************************
 */
static void bat_c_att_ntf_ind_evt_handler(uint8_t conn_idx, const ble_gattc_evt_ntf_ind_t *p_ntf_ind)
{
    bat_c_evt_t bat_c_evt;

    bat_c_evt.conn_idx = conn_idx;
    bat_c_evt.evt_type = BAT_C_EVT_INVALID;

    if (p_ntf_ind->handle == s_bat_c_env.handles.bat_bat_level_handle)
    {
        bat_c_evt.evt_type  = BAT_C_EVT_BAT_LEVE_RECEIVE;
        bat_c_evt.bat_level = p_ntf_ind->p_value[0];
        bat_c_evt_handler_excute(&bat_c_evt);
    }
}

/**
 *****************************************************************************************
 * @brief This callback function will be called when receiving browse service indication.
 *
 * @param[in] conn_idx:      The connection index.
 * @param[in] status:        The status of GATTC operation.
 * @param[in] p_browse_srvc: The information of service browse.
 *****************************************************************************************
 */
static void bat_c_srvc_browse_evt_handler(uint8_t conn_idx, uint8_t status, const ble_gattc_evt_browse_srvc_t *p_browse_srvc)
{
    bat_c_evt_t  bat_c_evt;
    uint16_t     uuid_disc;
    uint16_t     handle_disc;

    bat_c_evt.conn_idx = conn_idx;
    bat_c_evt.evt_type = BAT_C_EVT_DISCOVERY_FAIL;

    if(BLE_GATT_ERR_BROWSE_NO_ANY_MORE == status)
    {
        return;
    }

    if (BLE_SUCCESS == status)
    {
        uuid_disc = p_browse_srvc->uuid[0] | p_browse_srvc->uuid[1] << 8;

        if (BLE_ATT_SVC_BATTERY_SERVICE == uuid_disc)
        {
            s_bat_c_env.handles.bat_srvc_start_handle = p_browse_srvc->start_hdl;
            s_bat_c_env.handles.bat_srvc_end_handle   = p_browse_srvc->end_hdl;

            for (uint32_t i = 0; i < (p_browse_srvc->end_hdl - p_browse_srvc->start_hdl); i++)
            {
                uuid_disc   = p_browse_srvc->info[i].attr.uuid[0] | p_browse_srvc->info[i].attr.uuid[1] << 8;
                handle_disc = p_browse_srvc->start_hdl + i + 1;

                if (BLE_GATTC_BROWSE_ATTR_VAL == p_browse_srvc->info[i].attr_type)
                {
                    if (BLE_ATT_CHAR_BATTERY_LEVEL == uuid_disc)
                    {
                        s_bat_c_env.handles.bat_bat_level_handle = handle_disc;
                    }
                    else if (BLE_ATT_CHAR_TEMPERATURE_MEAS == uuid_disc)
                    {
                        s_bat_c_env.handles.bat_bat_temp_handle = handle_disc;
                    }
                }
                else if (BLE_GATTC_BROWSE_ATTR_DESC == p_browse_srvc->info[i].attr_type)
                {
                    if (BLE_ATT_DESC_CLIENT_CHAR_CFG == uuid_disc)
                    {
                        s_bat_c_env.handles.bat_bat_level_cccd_handle = handle_disc;
                    }
                    else if (BLE_ATT_DESC_CHAR_PRES_FORMAT == uuid_disc)
                    {
                        s_bat_c_env.handles.bat_bat_level_pres_handle = handle_disc;
                    }
                }
                else if (BLE_GATTC_BROWSE_NONE == p_browse_srvc->info[i].attr_type)
                {
                    break;
                }
            }

            bat_c_evt.evt_type = BAT_C_EVT_DISCOVERY_COMPLETE;
        }
    }

    bat_c_evt_handler_excute(&bat_c_evt);
}

static void bat_c_ble_evt_handler(const ble_evt_t *p_evt)
{
    if (NULL == p_evt)
    {
        return;
    }

    switch (p_evt->evt_id)
    {
        case BLE_GATTC_EVT_SRVC_BROWSE:
            bat_c_srvc_browse_evt_handler(p_evt->evt.gattc_evt.index, p_evt->evt_status, &p_evt->evt.gattc_evt.params.srvc_browse);
            break;

        case BLE_GATTC_EVT_READ_RSP:
            bat_c_att_read_evt_handler(p_evt->evt.gattc_evt.index, p_evt->evt_status, &p_evt->evt.gattc_evt.params.read_rsp);
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            bat_c_att_write_evt_handler(p_evt->evt.gattc_evt.index, p_evt->evt_status, p_evt->evt.gattc_evt.params.write_rsp.handle);
            break;

        case BLE_GATTC_EVT_NTF_IND:
            bat_c_att_ntf_ind_evt_handler(p_evt->evt.gattc_evt.index, &p_evt->evt.gattc_evt.params.ntf_ind);
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t bat_client_init(bat_c_evt_handler_t evt_handler)
{
    if (NULL == evt_handler)
    {
        return SDK_ERR_POINTER_NULL;
    }

    memset(&s_bat_c_env, 0, sizeof(s_bat_c_env));
    s_bat_c_env.evt_handler = evt_handler;

    return ble_gattc_prf_add(&s_bat_service_uuid, bat_c_ble_evt_handler);
}

sdk_err_t bat_c_disc_srvc_start(uint8_t conn_idx)
{
    return ble_gattc_services_browse(conn_idx, &s_bat_service_uuid);
}

sdk_err_t bat_c_bat_level_notify_set(uint8_t conn_idx, bool is_enable)
{
    uint16_t ntf_value = is_enable ? PRF_CLI_START_NTF : PRF_CLI_STOP_NTFIND;

    if (BLE_ATT_INVALID_HDL == s_bat_c_env.handles.bat_bat_level_cccd_handle)
    {
        return SDK_ERR_INVALID_HANDLE;
    }

    return ble_gattc_write(conn_idx, s_bat_c_env.handles.bat_bat_level_cccd_handle, 0, 2, (uint8_t *)&ntf_value);
}

sdk_err_t bat_c_bat_level_read(uint8_t conn_idx)
{
    if (BLE_ATT_INVALID_HDL == s_bat_c_env.handles.bat_bat_level_handle)
    {
        return SDK_ERR_INVALID_HANDLE;
    }

    return  ble_gattc_read(conn_idx, s_bat_c_env.handles.bat_bat_level_handle, 0);
}

sdk_err_t bat_c_bat_temp_write(uint8_t conn_idx, uint8_t* p_value)
{
    if (BLE_ATT_INVALID_HDL == s_bat_c_env.handles.bat_bat_temp_handle)
    {
        return SDK_ERR_INVALID_HANDLE;
    }

    return  ble_gattc_write_no_resp(conn_idx, false, s_bat_c_env.handles.bat_bat_temp_handle, 1, p_value);
}

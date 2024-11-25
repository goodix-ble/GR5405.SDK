/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "ble_prf_utils.h"
#include "user_bat_c.h"
#include "utility.h"
#include "app_timer.h"
#include "app_error.h"
#include "app_log.h"
#include <string.h>

#define BATTERY_WRITE_TEMP_INTERVAL  3000   // 3s

static app_timer_id_t s_bat_timer_id;
static uint8_t s_temp_value;

static void user_bat_c_timer_cb(void *p_arg)
{
    APP_LOG_INFO("send write cmd, temp_value = %d\n", s_temp_value);
    bat_c_bat_temp_write(0, &s_temp_value);
    s_temp_value++;
}

static void user_bat_c_timer_init(void)
{
    sdk_err_t error_code;

    error_code = app_timer_create(&s_bat_timer_id, ATIMER_REPEAT, user_bat_c_timer_cb);
    APP_ERROR_CHECK(error_code);
}

void user_bat_c_evt_handler(bat_c_evt_t *p_evt)
{
    switch(p_evt->evt_type)
    {
        case BAT_C_EVT_DISCOVERY_COMPLETE:
            APP_LOG_INFO("discover service complete.");
            APP_LOG_INFO("start to enable notify.");
            bat_c_bat_level_notify_set(p_evt->conn_idx, true);
            break;

        case BAT_C_EVT_BAT_LEVEL_NTF_SET_SUCCESS:
            APP_LOG_INFO("enable notify complete.");
            app_timer_start(s_bat_timer_id, BATTERY_WRITE_TEMP_INTERVAL, NULL);
            break;

        case BAT_C_EVT_BAT_LEVE_RECEIVE:
            APP_LOG_INFO("receive notity, level_value = %d", p_evt->bat_level);
            break;

        case BAT_C_EVT_DISCOVERY_FAIL:
            APP_LOG_INFO("discover service fail.");
            break;

        case BAT_C_EVT_BAT_LEVEL_NTF_SET_ERR:
            APP_LOG_INFO("set notify enable fail.");

        default:
            break;
    }
}

void user_bat_c_connect_handler(uint8_t conn_idx)
{

}

void user_bat_c_disconnect_handler(uint8_t conn_idx, uint8_t reason)
{
    app_timer_stop(s_bat_timer_id);
}

void user_bat_c_init(void)
{
    user_bat_c_timer_init();
    bat_client_init(user_bat_c_evt_handler);
}

void user_bat_c_disc_srvc_start(uint8_t conn_idx)
{
    bat_c_disc_srvc_start(conn_idx);
}

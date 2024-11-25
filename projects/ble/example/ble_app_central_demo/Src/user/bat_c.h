/**
 *****************************************************************************************
 *
 * @file bat_c.h
 *
 * @brief Battery Service Client API
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

/**
 * @addtogroup BLE_SRV BLE Services
 * @{
 */

/**
 * @defgroup BLE_SDK_BAS_C Battery Service Client (BAS_C)
 * @{
 * @brief Battery Service Client module.
 *
 * @details The Battery Service Client contains the APIs and types, which can be used by the
 *          application to discovery of Battery Service of peer and interact with it.
 *
 *          The application must provide an event handler to register, then call \ref bas_client_init().
 *          After Battery Service Client discoveries peer Battery Service, application can call
 *          \ref bas_c_bat_level_notify_set() and \ref bas_c_bat_level_read() to get battery
 *          data from peer.
 */

#ifndef __BAT_C_H__
#define __BAT_C_H__

#include "gr_includes.h"
#include "ble_prf_types.h"
#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup BAT_C_MACRO Defines
 * @{
 */
#define BAT_C_CONNECTION_MAX                10       /**< Maximum number of BAT Client connections. */
/** @} */

/**
 * @defgroup BAT_C_ENUM Enumerations
 * @{
 */
/**@brief Battery Service Client event type. */
typedef enum
{
    BAT_C_EVT_INVALID,                      /**< BAT Client invalid event. */
    BAT_C_EVT_DISCOVERY_COMPLETE,           /**< BAT Client has found BAS service and its characteristics. */
    BAT_C_EVT_DISCOVERY_FAIL,               /**< BAT Client found BAS service failed because of invalid operation or no found at the peer. */
    BAT_C_EVT_BAT_LEVEL_NTF_SET_SUCCESS,    /**< BAT Client has enabled Notification of Battery Level characteristics. */
    BAT_C_EVT_BAT_LEVEL_NTF_SET_ERR,        /**< Error occured when BAT Client set Notification of Battery Level characteristics. */
    BAT_C_EVT_BAT_LEVE_RECEIVE,             /**< BAT Client has received Battery Level value (Read or Notification from peer). */
} bat_c_evt_type_t;
/** @} */

/**
 * @defgroup BAT_C_STRUCT Structures
 * @{
 */
/**@brief Handles on the connected peer device needed to interact with it. */
typedef struct
{
    uint16_t bat_srvc_start_handle;      /**< BAT Service start handle. */
    uint16_t bat_srvc_end_handle;        /**< BAT Service end handle. */
    uint16_t bat_bat_level_handle;       /**< BAT Battery Level characteristic Value handle which has been got from peer. */
    uint16_t bat_bat_level_cccd_handle;  /**< BAT CCCD handle of Battery Level characteristic which has been got from peer. */
    uint16_t bat_bat_level_pres_handle;  /**< BAT Presentation Format Descriptor handle of Battery Level characteristic which has been got from peer. */
    uint16_t bat_bat_temp_handle;        /**< BAT Service temperature handle. */
} bat_c_handles_t;

/**@brief Battery Service Client event. */
typedef struct
{
    uint8_t          conn_idx;            /**< The connection index. */
    bat_c_evt_type_t evt_type;            /**< BAT Client event type. */
    uint8_t          bat_level;           /**< Battery level. */
} bat_c_evt_t;
/** @} */

/**
 * @defgroup BAT_C_TYPEDEF Typedefs
 * @{
 */
/**@brief  Battery Service Client event handler type. */
typedef void (*bat_c_evt_handler_t)(bat_c_evt_t *p_evt);
/** @} */

/**
 * @defgroup BAT_C_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Register BAT Client event handler.
 *
 * @param[in] evt_handler: Battery Service Client event handler.
 *
 * @return Result of initialization.
 *****************************************************************************************
 */
sdk_err_t bat_client_init(bat_c_evt_handler_t evt_handler);

/**
 *****************************************************************************************
 * @brief Discovery Battery Service on peer.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t bat_c_disc_srvc_start(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Enable or disable peer Battery Level characteristic notify.
 *
 * @param[in] conn_idx:  Index of connection.
 * @param[in] is_enable: true or false.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t bat_c_bat_level_notify_set(uint8_t conn_idx, bool is_enable);

/**
 *****************************************************************************************
 * @brief Read Battery Level characteristic value.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t bat_c_bat_level_read(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Read Battery Level characteristic value.
 *
 * @param[in] conn_idx: Index of connection.
 * @param[in] p_value: Pointer to the value.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t bat_c_bat_temp_write(uint8_t conn_idx, uint8_t* p_value);
/** @} */

#endif
/** @} */
/** @} */

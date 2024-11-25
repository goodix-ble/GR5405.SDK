/**
 ****************************************************************************************
 *
 * @file ble_prf.h
 *
 * @brief BLE PRF API
 *
 ****************************************************************************************
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
 * @addtogroup BLE
 * @{
 */

/**
  @addtogroup BLE_PRF Profile
  @{
  @brief  Definitions and prototypes for the profile interface.
 */

#ifndef __BLE_PRF_H__
#define __BLE_PRF_H__

#include "ble_error.h"
#include "ble_att.h"
#include "ble_gatts.h"
#include "ble_gattc.h"
#include "ble_gatt.h"
#include "ble_gapc.h"
#include "ble_event.h"

/**
  @addtogroup BLE_PRF_COMMON Profile Common
  @{
  @brief  Definitions and prototypes for Profile Common interface.
 */

/** @addtogroup BLE_PRF_MANAGER_TYPEDEFS Typedefs
 * @{ */
/**
****************************************************************************************
* @brief  Initialization of the Profile module.
* @note   This function performs all the initializations of the Profile module, and it will be automatically called after the ble_server_prf_add() or ble_client_prf_add() function.
*         - Creation of database (if it's a service) and ble_gatts_srvc_db_create should be called.
*         - Allocation of profile-required memory.
*
* @retval status code to know if profile initialization succeeds or not.
 ****************************************************************************************
 */
typedef uint8_t (*prf_init_func_t)(void);

/**
****************************************************************************************
* @brief Handles Connection creation. There is no need to recovery CCCD because stack will do that.
*
* @param[in]  conn_idx:         Connection index.
* @param[in]  p_peer_bd_addr:   The pointer for the peer identity address.
 ****************************************************************************************
 */
typedef void (*prf_on_connect_func_t)(uint8_t conn_idx, const ble_gap_bdaddr_t *p_peer_bd_addr);

/**
****************************************************************************************
* @brief Handles Disconnection. There is no need to recovery CCCD because stack will do that.
*
* @param[in]  conn_idx:     Connection index.
* @param[in]  reason:       Disconnection reason.
 ****************************************************************************************
 */
typedef void (*prf_on_disconnect_func_t)(uint8_t conn_idx, uint8_t reason);

 /** @addtogroup BLE_PRF_MANAGER_STRUCTURES Structures
 * @{ */
 /**
 * @brief Profile manager callbacks.
 */
/** @} */
typedef struct
{
    prf_init_func_t          init;              /**< Initialization callback. See @ref prf_init_func_t. */
    prf_on_connect_func_t    on_connect;        /**< Connection callback. See @ref prf_on_connect_func_t. */
    prf_on_disconnect_func_t on_disconnect;     /**< Disconnection callback. See @ref prf_on_disconnect_func_t. */
} ble_prf_manager_cbs_t;

/** @} */
/** @} */


/**
  @addtogroup BLE_PRF_SERVER Profile Server
  @{
  @brief  Definitions and prototypes for Profile Server interface.
 */

/** @addtogroup BLE_PRF_SERVER_STRUCTURES Structures
 * @{ */
/**
 * @brief GATT read request struct.
 */
typedef struct
{
    uint16_t handle;                       /**< Handle of the attribute to be read. */
} gatts_read_req_cb_t;

/**
 * @brief GATT write request struct.
 */
typedef struct
{
    uint16_t handle;                       /**< Handle of the attribute to be written. */
    uint16_t offset;                       /**< Offset at which the data has to be written. */
    uint16_t length;                       /**< Data length to be written. */
    uint8_t  value[__ARRAY_EMPTY];         /**< Data to be written to characteristic value. */
} gatts_write_req_cb_t;

/**
 * @brief GATT prepare write request struct.
 */
typedef struct
{
    uint16_t handle;                       /**< Handle of the attribute for whose value is requested. */
} gatts_prep_write_req_cb_t;

/**
 * @brief GATTS Operation Complete event structure.
 */
typedef struct
{
    ble_gatt_evt_type_t type;               /**< Notification or indication event type. */
    uint16_t            handle;             /**< Handle of the write operation, or notification/indication operation. */
} ble_gatts_ntf_ind_t;

/**
 * @brief Profile server register information structure.
 */
typedef struct
{
    uint16_t max_connection_nb;              /**< Maximum connections the profile supports. */
    ble_prf_manager_cbs_t* manager_cbs;      /**< Profile manager callbacks. */
    ble_evt_handler_t gatts_evt_handler;     /**< GATT server event handler in relation to the specific profile. */
} prf_server_info_t;

/** @} */

/** @addtogroup BLE_PRF_FUNCTIONS Functions
* @{ */

/**
 ****************************************************************************************
 * @brief Add a server profile by providing its detailed information..
 *
 * @param[in] p_gatts_db:     Pointer to the prf_info. See @ref ble_gatts_create_db_t.
 * @param[in] evt_handler:    Pointer to ble events handler.
 *
 * @retval ::SDK_SUCCESS:           The profile info is recorded successfully, and the database will be created in profile initialization callback function.
 * @retval ::SDK_ERR_POINTER_NULL:  The parameter p_gatts_db or evt_handler is NULL.
 * @retval ::SDK_ERR_NO_RESOURCES:  The profile number is up to the maximum number the system can support.
 ****************************************************************************************
 */
sdk_err_t ble_gatts_prf_add(ble_gatts_create_db_t *p_gatts_db, ble_evt_handler_t evt_handler);

/** @} */
/** @} */


/**
  @addtogroup BLE_PRF_CLIENT Profile Client
  @{
  @brief  Definitions and prototypes for Profile Client interface.
 */

/**
  @addtogroup BLE_PRF_CLIENT_FUNCTIONS Functions
  @{
  @brief  Definitions and prototypes for Profile Client interface.
 */
/**
 ****************************************************************************************
 * @brief Add a client profile by providing its detail information.
 *
 * @param[in]  p_uuid:       Pointer to the target service uuid. See @ref ble_uuid_t.
 * @param[out] evt_handler:  Pointer to ble events handler..
 *
 * @retval ::SDK_SUCCESS:          The profile info is recorded successfully, and the profile ENV will be initialized in profile initialization callback function.
 * @retval ::SDK_ERR_POINTER_NULL: The parameter p_uuid or evt_handler is NULL, or input parameters that prf_info points to are invalid.
 * @retval ::SDK_ERR_NO_RESOURCES: The profile number is up to the maximum number the system can support.
 ****************************************************************************************
 */
uint16_t ble_gattc_prf_add(ble_uuid_t *p_uuid,  ble_evt_handler_t evt_handler);

/** @} */
/** @} */

#endif

/** @} */
/** @} */

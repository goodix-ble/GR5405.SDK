/**
 ****************************************************************************************
 *
 * @file ble_debug_server.h
 *
 * @brief BLE debug Service API
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

/**
 * @addtogroup BLE_SRV BLE Services
 * @{
 * @brief Definitions and prototypes for the BLE Service interface.
 */

/**
 * @defgroup BLE_SDK DBG Service (DBGS)
 * @{
 * @brief Definitions and prototypes for the DBGS interface.
 *
 * @details The DBG Service is a customized service with Tx, Rx and Control Point
 *          characteristics. After @ref dbgs_init_t variable is initialized, the
 *          developer shall call @ref dbgs_service_init() to add the DBG
 *          Service and RX, TX, Control Point characteristic to the BLE stack database.
 *
 *          This module also provides \ref dbgs_notify_tx_data() function to the application
 *          to send data to peer.
 */

#ifndef __BLE_DEBUG_SERVER_H__
#define __BLE_DEBUG_SERVER_H__

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "gr_includes.h"

/**
 * @defgroup DBGS_MACRO Defines
 * @{
 */
#define DBGS_MAX_DATA_LEN           244                                                  /**< Maximum length of DBGS characteristic. */
#define DBGS_VERSION                0x02                                                 /**< The DBG VERSION. */
#define BLE_UUID_DBG_SERVICE        0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                                    0x0A, 0x46, 0x44, 0xD3, 0x01, 0x04, 0xED, 0xA6       /**< The UUID of DBG Service for setting advertising data. */
/** @} */

/**
 * @defgroup DBGS_ENUM Enumerations
 * @{
 */
/**@brief DBG Service Control Point Operation Code.*/
typedef enum
{
    DBGS_CTRL_PT_OP_RESERVED,                   /**< Reserved for future use. */
    DBGS_CTRL_PT_OP_DBG_ENTER = 0x474f4f44,     /**< DBGS task enter Operation Code.*/
    DBGS_CTRL_PT_OP_RSP_CODE = 0x10,            /**< Response code. */
} dbgs_ctrl_pt_op_code_t;

/**@brief DBG Service event type. */
typedef enum
{
    DBGS_EVT_INVALID,                           /**< Invalid event for DBG service. */
    DBGS_EVT_TX_NOTIFICATION_ENABLED,           /**< TX notification enable event for DBG service. */
    DBGS_EVT_TX_NOTIFICATION_DISABLED,          /**< TX notification disable event for DBG service. */
    DBGS_EVT_CTRL_PT_INDICATION_ENABLED,        /**< Control point indication enable event for DBG service. */
    DBGS_EVT_CTRL_PT_INDICATION_DISABLED,       /**< Control point indication disable event for DBG service. */
    DBGS_EVT_RX_RECEIVE_DATA,                   /**< RX data event for DBG service. */
    DBGS_EVT_NOTIFY_COMPLETE,                   /**< Notify complete event for DBG service. */
    DBGS_EVT_TASK_ENTER,                        /**< Set task enter event for DBG service. */
    DBGS_EVT_DISCONNECT,                        /**< Link disconnected. */
} dbgs_evt_type_t;
/** @} */

/**
 * @defgroup DBGS_STRUCT Structures
 * @{
 */
/**@brief DBG Service event. */
typedef struct
{
    dbgs_evt_type_t evt_type;                   /**< The DBGS event. */
    uint8_t         conn_idx;                   /**< Index of connection. */
    uint8_t         *p_data;                    /**< Pointer to data. */
    uint16_t        length;                     /**< Length of data. */
} dbgs_evt_t;
/** @} */

/**
 * @defgroup DBGS_TYPEDEF Typedefs
 * @{
 */
/**@brief DBG Service event handler type. */
typedef void (*dbgs_evt_handler_t)(dbgs_evt_t *p_evt);

/**@brief DBG Service write tx cccd callback type. */
typedef void (*dbgs_write_tx_cccd_cb_t)(uint8_t conn_idx);
/** @} */

/**
 * @defgroup DBGS_STRUCT Structures
 * @{
 */
/**@brief DBG Service initialization variable. */
typedef struct
{
    dbgs_evt_handler_t evt_handler;          /**< Handler to handle DBGS event. */
} dbgs_init_t;
/** @} */


/**
 * @defgroup DBGS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Add a DBG Service instance in the DB
 *
 * @param[in] p_dbgs_init: Pointer to DBG Service environment variable
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t dbgs_service_init(dbgs_init_t *p_dbgs_init);

/**
 *****************************************************************************************
 * @brief Send data to peer device
 *
 * @param[in] conn_idx: Connection index
 * @param[in] p_data:   The Pointer of sending value
 * @param[in] length:   The length of sending value
 *
 * @return Result of notify and indicate value
 *****************************************************************************************
 */
sdk_err_t dbgs_notify_tx_data(uint8_t conn_idx, uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Provide the interface for other modules to obtain the DBG service start handle.
 *
 * @return The DBG service start handle.
 *****************************************************************************************
 */
uint16_t dbgs_service_get_start_handle(void);
/** @} */

/**
 *****************************************************************************************
 * @brief Register callback for write tx cccd.
 *
 *****************************************************************************************
 */
void dbgs_reg_write_tx_cccd_cb(dbgs_write_tx_cccd_cb_t p_cb);

#endif

/** @} */
/** @} */

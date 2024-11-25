/**
 ****************************************************************************************
 *
 * @file ble_gapc.h
 *
 * @brief BLE GAPC API
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
 * @brief Definitions and prototypes for the BLE SDK interface.
 */

/**
 * @addtogroup BLE_GAP Generic Access Profile (GAP)
 * @{
 * @brief Definitions and prototypes for the GAP interface.
 */
 
/**
 * @defgroup BLE_GAPC Generic Access Profile (GAP) Connection Control
 * @{
 * @brief Definitions and prototypes for the GAP Connection Control interface.
 */
#ifndef __BLE_GAPC_H__
#define __BLE_GAPC_H__

#include "ble_error.h"
#include "gr5405_sys_cfg.h"
#include <stdint.h>         // Standard Integer
#include <string.h>
#include <stdbool.h>

/**
 * @defgroup  BLE_GAPC_DEFINES Defines
 * @{
 */
#define BLE_GAP_CHNL_MAP_LEN         0x05 /**< The length of channel map. */
#define BLE_GAP_FEATS_LEN            0x08 /**< The length of features. */
#define BLE_GAP_ADDR_LEN             0x06 /**< The length of address. */
#define BLE_GAP_INVALID_CONN_INDEX   0xFF /**< Invalid connection index. */

/// CTE length (in number of 8us periods)
#define BLE_GAP_MIN_CTE_LEN          0x02 /**< The minimum CTE length. */
#define BLE_GAP_MAX_CTE_LEN          0x14 /**< The maximum CTE length. */

/// CTE count
#define BLE_GAP_MIN_CTE_CNT          0x01 /**< The minimum CTE count. */
#define BLE_GAP_MAX_CTE_CNT          0x10 /**< The maximum CTE count. */

#define BLE_GAP_MIN_IQ_SAMPLE_NUM    0x09 /**< The minimum IQ sample number. */
#define BLE_GAP_MAX_IQ_SAMPLE_NUM    0x52 /**< The maximum IQ sample number. */

/** @} */

/**
 * @defgroup BLE_SDK_GAP_ENUM Enumerations
 * @{
 */

/** @brief The identity address type */
typedef enum
{
    BLE_GAP_ADDR_TYPE_PUBLIC = 0,      /**< Public (identity) address.*/
    BLE_GAP_ADDR_TYPE_RANDOM_STATIC,   /**< Random static (identity) address. */
} ble_gap_addr_type_t;

/** @brief The phy options */
typedef enum
{
    BLE_GAP_PHY_OPT_NO_CODING = 0, /**< The Host has no preferred coding when transmitting on the LE Coded PHY. */
    BLE_GAP_PHY_OPT_S2_CODING,     /**< The Host prefers that S=2 coding be used when transmitting on the LE Coded PHY. */
    BLE_GAP_PHY_OPT_S8_CODING,     /**< The Host prefers that S=8 coding be used when transmitting on the LE Coded PHY. */
} ble_gap_phy_options_t;

/** @brief The operation code used to get connection info */
typedef enum
{
    BLE_GAP_GET_CON_RSSI = 0,        /**< Get connection RSSI info. */
    BLE_GAP_GET_CON_CHANNEL_MAP,     /**< Get connection channel map. */
    BLE_GAP_GET_PHY,                 /**< Get connection PHY. */
} ble_gap_get_conn_info_op_t;

/**@brief The operation code used to get peer device info. */
typedef enum
{
    BLE_GAP_GET_PEER_VERSION = 0,    /**< Get peer device version info. */
    BLE_GAP_GET_PEER_FEATURES        /**< Get peer device features info. */
} ble_gap_get_peer_info_op_t;

/** @brief Device role of LL layer type */
typedef enum
{
    BLE_GAP_LL_ROLE_MASTER = 0,                  /**< Master role. */
    BLE_GAP_LL_ROLE_SLAVE  = 1,                  /**< Slave role. */
} ble_gap_ll_role_type_t;

/**
 * @brief Operation code used to set param(s).
 */
typedef enum
{
    BLE_GAP_OPCODE_CHNL_MAP_SET,            /**< Set Channel Map. */
    BLE_GAP_OPCODE_WHITELIST_SET,           /**< Set white list. */
    BLE_GAP_OPCODE_PER_ADV_LIST_SET,        /**< Set periodic advertising list. */
    BLE_GAP_OPCODE_PRIVACY_MODE_SET,        /**< Set privacy mode for peer device. */
} ble_gap_param_set_op_id_t;

/**
 * @brief The specified reason for terminating a connection.
 */
typedef enum
{
    BLE_GAP_HCI_AUTHENTICATION_FAILURE                          = 0x05, /**< Authentication Failure. */
    BLE_GAP_HCI_REMOTE_USER_TERMINATED_CONNECTION               = 0x13, /**< Remote User Terminated Connection. */
    BLE_GAP_HCI_REMOTE_DEV_TERMINATION_DUE_TO_LOW_RESOURCES     = 0x14, /**< Remote Device Terminated Connection due to Low Resources. */
    BLE_GAP_HCI_REMOTE_DEV_TERMINATION_DUE_TO_POWER_OFF         = 0x15, /**< Remote Device Terminated Connection due to Power Off. */
    BLE_GAP_HCI_UNSUPPORTED_REMOTE_FEATURE                      = 0x1A, /**< Unsupported Remote Feature. */
    BLE_GAP_HCI_PAIRING_WITH_UNIT_KEY_UNSUPPORTED               = 0X29, /**< Pairing With Unit Key Not Supported. */
    BLE_GAP_HCI_CONN_INTERVAL_UNACCEPTABLE                      = 0x3B, /**< Unacceptable Connection Parameters. */
} ble_gap_disconn_reason_t;

/**
 * @brief Operation code used for LEPSM manager.
 */
typedef enum
{
    BLE_GAP_OPCODE_LEPSM_REGISTER,      /**< LEPSM register operation. */
    BLE_GAP_OPCODE_LEPSM_UNREGISTER,    /**< LEPSM unregister operation. */
} ble_gap_psm_manager_op_id_t;

/** @brief GAP Device inforamtion write indication. */
typedef enum
{
    BLE_GAPC_DEV_NAME,              /* Device name type*/
    BLE_GAPC_DEV_APPEARANCE,        /* Device Appearance Icon type*/
} ble_gap_dev_info_type_t;

/**
 * @brief Type of constant tone extension.
 */
typedef enum
{
    BLE_GAP_CTE_TYPE_AOA         = 0x01 << 0,   /**< Allow AoA Constant Tone Extension Response. */
    BLE_GAP_CTE_TYPE_AOD_1US     = 0x01 << 1,   /**< Allow AoD Constant Tone Extension Response with 1us slots. */
    BLE_GAP_CTE_TYPE_AOD_2US     = 0x01 << 2,   /**< Allow AoD Constant Tone Extension Response with 2us slots. */
} ble_gap_cte_type_t;

/**
 * @brief Type of switching and sampling slots 
 */
typedef enum
{
    BLE_GAP_SLOT_1US = 0x01,     /**< Switching and sampling slots are 1us each. */
    BLE_GAP_SLOT_2US,            /**< Switching and sampling slots are 2us each. */
} ble_gap_switching_sampling_type_t;

/**
 * @brief Status of IQ report packet
 */
typedef enum
{
    BLE_GAP_CRC_OK,                      /**< CRC was correct. */
    BLE_GAP_CRC_ERR1,                    /**< CRC was incorrect and the Length and CTETime fields of the packet were used to determine sampling points. */
    BLE_GAP_CRC_ERR2,                    /**< CRC was incorrect but the Controller has determined the position and length of the Constant Tone Extension in some other way. */
    BLE_GAP_INSUFFI_RESOURCE = 0xFF     /**< Insufficient resources to sample (data_channel_idx, cte_type, and slot_dur invalid). */
} ble_gap_iq_report_status_t;

/**
 * @brief Phy for power control management 
 */
 typedef enum
{
    BLE_GAP_PHY_1M       = 0x01,        /**< LE 1M PHY. */
    BLE_GAP_PHY_2M       = 0x02,        /**< LE 2M PHY. */
    BLE_GAP_PHY_CODED_S8 = 0x03,        /**< LE Coded PHY with S=8 data coding. */
    BLE_GAP_PHY_CODED_S2 = 0x04         /**< LE Coded PHY with S=2 data coding. */
} ble_gap_phy_type_t;

/**
 * @brief Transmit power change reporting reason.
 */
typedef enum
{
    BLE_GAP_PWR_LOCAL_TX_CHG   = 0x00, /**< Local transmit power changed. */
    BLE_GAP_PWR_REMOTE_TX_CHG  = 0x01, /**< Remote transmit power changed. */
} ble_gap_tx_pwr_change_report_reason_t;

/**
 * @brief Transmit Power level flag.
 */
typedef enum
{
    BLE_GAP_PWR_MID_LVL  = 0x00, /**< Transmit power level is between minimum and max level. */
    BLE_GAP_PWR_MIN_LVL  = 0x01, /**< Transmit power level is at minimum level. */
    BLE_GAP_PWR_MAX_LVL  = 0x02  /**< Transmit power level is at maximum level. */
} ble_gap_pwr_lvl_flag_t;

/// Path Loss zones. HCI:7.8.118
typedef enum
{
    BLE_GAP_PATH_LOSS_LOW           = 0x00, /**< Entered Low zone. */
    BLE_GAP_PATH_LOSS_MID           = 0x01, /**< Entered Middle zone. */
    BLE_GAP_PATH_LOSS_HIGH          = 0x02, /**< Entered High zone. */
} ble_gap_path_loss_zone_t;

/** @} */


/**
 * @defgroup BLE_GAPC_STRUCT Structures
 * @{
 */

/** @brief The struct of address. */
typedef struct
{
    uint8_t  addr[BLE_GAP_ADDR_LEN]; /**< 6-byte array address value. */
} ble_gap_addr_t;

/** @brief The struct of broadcast address with broadcast type. */
typedef struct
{
    ble_gap_addr_t gap_addr;     /**< Device BD Address. */
    uint8_t        addr_type;    /**< Address type of the device: 0=public/1=random. please @ref ble_gap_addr_type_t. */
} ble_gap_bdaddr_t;

/** @brief Sync established indication. */
typedef struct
{
    uint8_t           phy;           /**< PHY on which synchronization has been established. @see gap_phy_type. */
    uint16_t          intv;          /**< Periodic advertising interval (in unit of 1.25ms, min is 7.5ms). */
    uint8_t           adv_sid;       /**< Advertising SID. */
    uint8_t           clk_acc;       /**< Advertiser clock accuracy. @see enum gapm_clk_acc. */
    ble_gap_bdaddr_t  bd_addr;       /**< Advertiser address. */
    uint16_t          sync_hdl;      /**< Sync handle. */
    uint16_t          serv_data;     /**< Service data. */
    bool              report_flag;   /**< Report Flag. */
} ble_gap_sync_established_ind_t;


/** @brief APP receives the extended advertising report indication info struct. */
typedef struct
{
    uint8_t           adv_type;              /**< Advertising type. @see enum gap_adv_report_type_t. */
    uint8_t           adv_info;              /**< Bit field providing information about the received report. @see enum gap_adv_report_info_t. */
    ble_gap_bdaddr_t  broadcaster_addr;      /**< Broadcaster device address. */
    ble_gap_bdaddr_t  direct_addr;           /**< Target address (in case of a directed advertising report). */
    int8_t            tx_pwr;                /**< TX power (in dBm). */
    int8_t            rssi;                  /**< RSSI (between -127 and +20 dBm). */
    uint8_t           phy_prim;              /**< Primary PHY on which advertising report has been received. */
    uint8_t           phy_second;            /**< Secondary PHY on which advertising report has been received. */
    uint8_t           adv_sid;               /**< Advertising SID , valid only for periodic advertising report. */
    uint16_t          period_adv_intv;       /**< Periodic advertising interval (in unit of 1.25ms, min is 7.5ms), valid only for periodic advertising report. */
    uint8_t           per_sync_idx;          /**< Periodic syncronization index, valid only for periodic advertising report. */
    uint16_t          length;                /**< Report length. */
    uint8_t           data[__ARRAY_EMPTY];   /**< Report. */
} ble_gap_ext_adv_report_ind_t;

/** @brief Connection parameter used to update connection parameters. */
typedef struct
{
    uint16_t interval;           /**< Connection interval. Range: 0x0006 to 0x0C80. Unit: 1.25 ms. Time range: 7.5 ms to 4 s. */
    uint16_t latency;            /**< Latency for the connection in number of connection events. Range: 0x0000 to 0x01F3. */
    uint16_t sup_timeout;        /**< Supervision timeout for the LE link. Range: 0x000A to 0x0C80, unit: 10 ms, time range: 100 ms to 32 s. */
} ble_gap_conn_update_cmp_t;

/** @brief The parameter of connection. */
typedef  struct
{
     uint16_t interval_min;  /**< Minimum value for the connection interval. This shall be less than or equal to Conn_Interval_Max.
                                  Range: 0x0006 to 0x0C80, unit: 1.25 ms, time range: 7.5 ms to 4 s*/
     uint16_t interval_max;  /**< Maximum value for the connection interval. This shall be greater than or equal to Conn_Interval_Min.
                                  Range: 0x0006 to 0x0C80, unit: 1.25 ms, time range: 7.5 ms to 4 s.*/
     uint16_t slave_latency; /**< Slave latency for the connection in number of connection events. Range: 0x0000 to 0x01F3. */
     uint16_t sup_timeout;   /**< Supervision timeout for the LE link. Range: 0x000A to 0x0C80, unit: 10 ms, time range: 100 ms to 32 s. */
} ble_gap_conn_param_t;

/** @brief The parameter of update connection. */
typedef  struct
{
     uint16_t interval_min;  /**< Minimum value for the connection interval. This shall be less than or equal to Conn_Interval_Max.
                                  Range: 0x0006 to 0x0C80, unit: 1.25 ms, time range: 7.5 ms to 4 s*/
     uint16_t interval_max;  /**< Maximum value for the connection interval. This shall be greater than or equal to Conn_Interval_Min.
                                  Range: 0x0006 to 0x0C80, unit: 1.25 ms, time range: 7.5 ms to 4 s.*/
     uint16_t slave_latency; /**< Slave latency for the connection in number of connection events. Range: 0x0000 to 0x01F3. */
     uint16_t sup_timeout;   /**< Supervision timeout for the LE link. range: 0x000A to 0x0C80, unit: 10 ms, Time range: 100 ms to 32 s. */
     uint16_t ce_len;        /**< The length of connection event needed for this LE connection. Range: 0x0002 to 0xFFFF, unit: 0.625 ms, time Range: 1.25 ms to 40.9 s.
                                  recommended value: 0x0002 for 1M phy, 0x0006 for coded phy*/
} ble_gap_conn_update_param_t;

/** @brief  Connection complete info. */
typedef struct
{
    uint16_t               conhdl;             /**< Connection_Handle. Range: 0x0000-0x0EFF (all other values reserved for future use). */
    uint16_t               con_interval;       /**< Connection interval. Range: 0x0006 to 0x0C80, unit: 1.25 ms, time range: 7.5 ms to 4 s. */
    uint16_t               con_latency;        /**< Latency for the connection in number of connection events. Range: 0x0000 to 0x01F3. */
    uint16_t               sup_to;             /**< Connection supervision timeout. Range: 0x000A to 0x0C80, unit: 10 ms, time range: 100 ms to 32 s. */
    uint8_t                clk_accuracy;       /**< Clock accuracy (0x00: 500 ppm, 0x01: 250 ppm, 0x02: 150 ppm, 0x03: 100 ppm, 0x04: 75 ppm,
                                                    0x05:50 ppm, 0x06:30 ppm, 0x07:20 ppm, others: reserved for future use). */
    uint8_t                peer_addr_type;     /**< Peer address type(0x00: Public Device Address, 0x01 : Random Device Address, others: reserved for future use). */
    ble_gap_addr_t         peer_addr;          /**< Peer BT address. */
    ble_gap_ll_role_type_t ll_role;            /**< Device Role of LL Layer. */
} gap_conn_cmp_t;

/** @brief  Channel map structure. */
typedef struct
{
    uint8_t map[BLE_GAP_CHNL_MAP_LEN];     /**< This parameter contains 37 1-bit fields. The nth bit (n is in the range of 0 to 36) contains the value for the link layer channel index n.
                                                Channel n is unused = 0, channel n is used = 1. The most significant bits are reserved for future use.*/
} ble_gap_chnl_map_t;

/** @brief PHY info. */
typedef struct
{
    uint8_t tx_phy; /**< LE PHY for data transmission. @see ble_gap_phy_type_t. */
    uint8_t rx_phy; /**< LE PHY for data reception. @see ble_gap_phy_type_t. */
} ble_gap_le_phy_ind_t;

/** @brief Connection info. */
typedef union
{
    int8_t               rssi;              /**< RSSI. */
    ble_gap_chnl_map_t   chnl_map;          /**< channel map. */
    ble_gap_le_phy_ind_t phy;               /**< PHY indicaiton. */
    uint8_t              chan_sel_algo;     /**< Chanel Selection algorithm, 0x00: LE Channel Selection Algorithm #1 is used.
                                                 0x01: LE Channel Selection Algorithm #2 is used.\n 0x02-0xFF: reserved. */
} ble_gap_conn_info_t;

/** @brief The info of connecting operation. */
typedef struct
{
    uint8_t             opcode;  /**< Operation code. See @ref ble_gap_get_conn_info_op_t. */
    ble_gap_conn_info_t info;    /**< Connection info. */
} ble_gap_conn_info_param_t;

/** @brief Peer version info. */
typedef struct
{
    uint16_t compid;        /**<Manufacturer name. */
    uint16_t lmp_subvers;   /**< LMP subversion. */
    uint8_t  lmp_vers;      /**< LMP version. */
} ble_gap_peer_version_ind_t;


/** @brief LE features info. */
typedef struct
{
    uint8_t features[BLE_GAP_FEATS_LEN]; /**< 8-byte array for LE features\n 
                                          Feature Setting field's bit mapping to Controller Features (0: not support, 1: support) \n
                                                          |Bit position       | Link Layer Feature|
                                                          |-------------|-----------------|
                                                          |0                    | LE Encryption|
                                                          |1                    |Connection Parameters Request Procedure| 
                                                          |2                    |Extended Reject Indication|
                                                          |3                    | Slave-initiated Features Exchange | 
                                                          |4                    |LE Ping | 
                                                          |5                    |LE Data Packet Length Extension | 
                                                          |6                    |LL Privacy |  
                                                          |7                    |Extended Scanner Filter Policies | 
                                                          |8                    |LE 2M PHY|  
                                                          |9                    | Stable Modulation Index - Transmitter | 
                                                          |10                   | Stable Modulation Index - Receiver |
                                                          |11                   |LE Coded PHY | 
                                                          |12                   |LE Extended Advertising| 
                                                          |13                   | LE Periodic Advertising| 
                                                          |14                   | Channel Selection Algorithm #2| 
                                                          |15                   |LE Power Class 1|
                                                          |16                   |Minimum Number of Used Channels Procedure|
                                                          |17                   |Connection CTE Request|
                                                          |18                   |Connection CTE Response|
                                                          |19                   |Connectionless CTE Transmitter|
                                                          |20                   |Connectionless CTE Receiver|
                                                          |21                   |Antenna Switching During CTE Transmission(AoD)|
                                                          |22                   |Antenna Switching During CTE Reception(AoA)|
                                                          |23                   |Receiving Constant Tone Extensions|
                                                          |24                   |Periodic Advertising Sync Transfer - Sender|
                                                          |25                   |Periodic Advertising Sync Transfer - Recipient|
                                                          |26                   |Sleep Clock Accuracy Updates|
                                                          |27                   |Remote Public Key Validation|
                                                          |33                   |LE Power Control Request|
                                                          |34                   |LE Power Change Indication|
                                                          |35                   |LE Path Loss Monitoring|

                                                          |All other values |Reserved for Future Use|*/
} ble_gap_peer_features_ind_t;

/** @brief LE peer info. */
typedef union
{
    ble_gap_peer_version_ind_t  peer_version;   /**< Version info. */
    ble_gap_peer_features_ind_t peer_features;  /**< Features info. */
} ble_gap_peer_info_t;

/** @brief Get peer info operation struct. */
typedef struct
{
    uint8_t             opcode;         /**< Operation code. See @ref ble_gap_get_peer_info_op_t. */
    ble_gap_peer_info_t peer_info;      /**< Peer info. */
} ble_gap_peer_info_param_t;

/** @brief Supported data length size Indication. */
typedef struct
{
    uint16_t max_tx_octets; /**<  The maximum number of payload octets in TX. */
    uint16_t max_tx_time;   /**<  The maximum time that the local Controller will take to TX. */
    uint16_t max_rx_octets; /**<  The maximum number of payload octets in RX. */
    uint16_t max_rx_time;   /**<  The maximum time that the local Controller will take to RX. */
} ble_gap_le_pkt_size_ind_t;

/**@brief The Structure for BLE Connection Arrangement. */
typedef struct
{
    uint16_t conn_idx;     /**< Connection Index. */
    uint32_t interval;     /**< Connection Interval (in 312.5 us). */
    uint32_t offset;       /**< Connection Offset (in 312.5 us). */
    uint32_t duration;     /**< Connection Duration (in 312.5 us). */
} ble_gap_con_plan_tag_t;

/** @brief Set preference slave event duration */
typedef struct
{
    uint16_t duration; /**< Preferred event duration. */
    uint8_t  single_tx; /**< Slave transmits a single packet per connection event (False/True). */
} ble_gap_set_pref_slave_evt_dur_param_t;

/** @brief GAP Device name struct. */
typedef struct
{
    uint16_t length;                /**< Device name length. */
    uint8_t  value[__ARRAY_EMPTY];  /**< Device name data. */
} ble_gap_dev_name_ind_t;

/** @brief Device information data struct. */
typedef union
{
    ble_gap_dev_name_ind_t dev_name;    /**< Device name. see @ref ble_gap_dev_name_ind_t. */
    uint16_t               appearance;  /**< Device appearance */
} ble_gapc_set_dev_info_t;


/** @brief GAP Device inforamtion write indication. */
typedef struct
{
    ble_gap_dev_info_type_t info_type; /**< Device info type. see @ref ble_gap_dev_info_type_t. */
    ble_gapc_set_dev_info_t info;      /**< Device info data. see @ref ble_gap_cte_type_t. */
} gapc_set_dev_info_ind_t;

/**
 * @brief Default periodic advertising synchronization transfer parameters
 */
typedef struct
{
    uint8_t   mode;                     /**< @see gap_per_adv_sync_info_rec_mode. */
    uint16_t  skip;                     /**< Number of periodic advertising that can be skipped after a successful receive.
                                             Maximum authorized value is 499. */
    uint16_t  sync_to;                  /**< Synchronization timeout for the periodic advertising (in unit of 10 ms between 100 ms and 163.84s). */
    uint8_t   cte_type;                 /**< Type of Constant Tone Extension device should sync on (@see enum gap_sync_cte_type). */
} ble_gap_per_sync_trans_param_t;

/**
 * @brief Connectionless IQ Report info
 */
typedef struct
{
    uint8_t  channel_idx;                         /**< The index of the channel on which the packet was received, range 0x00 to 0x24. */
    int16_t  rssi;                                /**< RSSI units: 0.1 dBm, range -1270 to +200. */
    uint8_t  rssi_antenna_id;                     /**< RSSI antenna ID. */
    uint8_t  cte_type;                            /**< CTE type (0: GAP_CET_AOA | 1: GAP_CET_AOD_1US | 2: GAP_CET_AOD_2US), @see enum ble_gap_cte_type_t. */
    uint8_t  slot_dur;                            /**< Slot durations (1: GAP_SLOT_1US | 2: GAP_SLOT_2US), see @ref ble_gap_switching_sampling_type_t. */
    uint8_t  pkt_status;                          /**< Packet status, @see enum ble_gap_iq_report_status_t. */
    uint16_t pa_evt_cnt;                          /**< Periodic advertising event counter. */
    uint8_t  nb_samples;                          /**< Number of samples. 0x00: no samples provided (only permitted if pkt_status is 0xFF),
                                                       0x09 to 0x52: total number of sample pairs. */
    int8_t i_sample[BLE_GAP_MAX_IQ_SAMPLE_NUM];   /**< The list of i samples for the reported PDU. */
    int8_t q_sample[BLE_GAP_MAX_IQ_SAMPLE_NUM];   /**< The list of q samples for the reported PDU. */
} ble_gap_connless_iq_report_t;

/** @brief Set connection CTE transmit parameters info. */
typedef struct
{
    uint8_t cte_type;     /**< The type of cte, see @ref ble_gap_cte_type_t. */
    uint8_t num_antenna;  /**< The number of Antenna IDs in the pattern, range 0x02 to 0x4B. */
    uint8_t *antenna_id;  /**< List of Antenna IDs in the pattern. */
} ble_gap_set_conn_cte_trans_param_t;

/** @brief Set connection CTE receive parameters info. */
typedef struct
{
    bool    sampling_enable; /**< Wheter to sample IQ from the CTE. */
    uint8_t slot_durations;  /**< The slot for sample IQ from the CTE, see @ref ble_gap_switching_sampling_type_t. */
    uint8_t num_antenna;     /**< The number of Antenna IDs in the pattern, range 0x02 to 0x4B. */
    uint8_t *antenna_id;     /**< List of Antenna IDs in the pattern. */
} ble_gap_set_conn_cte_rcv_param_t;

/** @brief Set connection CTE Request enable info. */
typedef struct
{
    uint16_t cte_req_interval;    /**< Defines whether the cte request procedure is initiated only once or periodically.
                                       0x0000: initiate the Constant Tone Extension Request procedure once.
                                       0x0001 to 0xFFFF: requested interval for initiating the cte request procedure in number of connection events. */
    uint8_t  cte_req_len;         /**< Minimum length of the cte being requested in 8us units, range 0x02 to 0x14. */
    uint8_t  cte_req_type;        /**< The type for requested cte, see @ref ble_gap_cte_type_t. */
} ble_gap_set_conn_cte_req_enable_t;

/** @brief Connection IQ Report info. */
typedef struct
{
    uint8_t  rx_phy;                          /**< Rx PHY (0x01: 1M | 0x02: 2M), see @ref ble_gap_phy_type_t. */
    uint8_t  data_channel_idx;                /**< Data channel index, range 0x00 to 0x24. */
    int16_t  rssi;                            /**< RSSI units: 0.1 dBm, range -1270 to +200. */
    uint8_t  rssi_antenna_id;                 /**< RSSI antenna ID. */
    uint8_t  cte_type;                        /**< CTE type (0: GAP_CET_AOA | 1: GAP_CET_AOD_1US | 2: GAP_CET_AOD_2US), @see enum ble_gap_cte_type_t. */
    uint8_t  slot_dur;                        /**< Slot durations (1: GAP_SLOT_1US | 2: GAP_SLOT_2US), see @ref ble_gap_switching_sampling_type_t. */
    uint8_t  pkt_status;                      /**< Packet status, @see enum ble_gap_iq_report_status_t. */
    uint16_t con_evt_cnt;                     /**< Connection event counter. */
    uint8_t  nb_samples;                      /**< Number of samples. 0x00: no samples provided (only permitted if pkt_status is 0xFF),
                                                   0x09 to 0x52: total number of sample pairs. */
    int8_t i_sample[BLE_GAP_MAX_IQ_SAMPLE_NUM];   /**< The list of i samples for the reported PDU. */
    int8_t q_sample[BLE_GAP_MAX_IQ_SAMPLE_NUM];   /**< The list of q samples for the reported PDU. */
} ble_gap_conn_iq_report_t;

/** @brief Set path loss reporting parameter info. */
typedef struct
{
    uint8_t  high_thr;                  /**< High threshold for the path loss (dB). */
    uint8_t  high_hyst;                 /**< Hysteresis value for the high threshold (dB). */
    uint8_t  low_thr;                   /**< Low threshold for the path loss (dB). */
    uint8_t  low_hyst;                  /**< Hysteresis value for the low threshold (dB). */
    uint16_t min_conn_evt_num;          /**< Minimum time in number of connection events to be observed. */
} ble_gap_set_path_loss_report_param_t;

/**@brief PHY update event for @ref BLE_GAPC_EVT_PHY_UPDATED. */
typedef struct
{
    uint8_t     tx_phy;         /**< LE PHY for data transmission. @ref ble_gap_phy_type_t. */
    uint8_t     rx_phy;         /**< LE PHY for data reception. @ref ble_gap_phy_type_t. */
} ble_gap_evt_phy_update_t;

/** @brief  Connection complete event for @ref BLE_GAPC_EVT_CONNECTED. */
typedef struct
{
    uint16_t                conn_handle;            /**< Connection_Handle. Range: 0x0000-0x0EFF (all other values reserved for future use). */
    uint16_t                conn_interval;          /**< Connection interval. Range: 0x0006 to 0x0C80, unit: 1.25 ms, time range: 7.5 ms to 4 s. */
    uint16_t                slave_latency;          /**< Latency for the connection in number of connection events. Range: 0x0000 to 0x01F3. */
    uint16_t                sup_timeout;            /**< Connection supervision timeout. Range: 0x000A to 0x0C80, unit: 10 ms, time range: 100 ms to 32 s. */
    uint8_t                 clk_accuracy;           /**< Clock accuracy (0x00: 500 ppm, 0x01: 250 ppm, 0x02: 150 ppm, 0x03: 100 ppm, 0x04: 75 ppm, 0x05:50 ppm, 0x06:30 ppm, 0x07:20 ppm, others: reserved for future use). */
    uint8_t                 peer_addr_type;         /**< Peer address type(0x00: Public Device Address, 0x01 : Random Device Address, others: reserved for future use). */
    ble_gap_addr_t          peer_addr;              /**< Peer BT address. */
    ble_gap_ll_role_type_t  ll_role;                /**< Device Role of LL Layer. */
} ble_gap_evt_connected_t;

/**@brief Disconnection event for @ref BLE_GAPC_EVT_DISCONNECTED. */
typedef struct
{
    uint8_t reason;         /**< Hci error code. */
} ble_gap_evt_disconnected_t;

/** @brief  Name of peer device indication event for @ref BLE_GAPC_EVT_PEER_NAME_GOT. */
typedef struct
{
    ble_gap_addr_t  peer_addr;              /**< Peer device bd address. */
    uint8_t         addr_type;              /**< Peer device address type. */
    uint8_t         name_len;               /**< Peer device name length. */
    uint8_t        *name;                   /**< Peer device name. */
} ble_gap_evt_peer_name_get_t;

/** @brief Get peer info event for @ref BLE_GAPC_EVT_PEER_INFO_GOT. */
typedef struct
{
    uint8_t             opcode;         /**< Operation code. See @ref ble_gap_get_peer_info_op_t. */
    ble_gap_peer_info_t peer_info;      /**< Peer info. */
} ble_gap_evt_peer_info_t;

/** @brief Connection parameter updated event for @ref BLE_GAPC_EVT_CONN_PARAM_UPDATED. */
typedef struct
{
    uint16_t conn_interval;             /**< Connection interval. Range: 0x0006 to 0x0C80. Unit: 1.25 ms. Time range: 7.5 ms to 4 s. */
    uint16_t slave_latency;             /**< Latency for the connection in number of connection events. Range: 0x0000 to 0x01F3. */
    uint16_t sup_timeout;               /**< Supervision timeout for the LE link. Range: 0x000A to 0x0C80, unit: 10 ms, time range: 100 ms to 32 s. */
} ble_gap_evt_conn_param_updated_t;

/** @brief Connection parameter update request event for @ref BLE_GAPC_EVT_CONN_PARAM_UPDATE_REQ. */
typedef  struct
{
     uint16_t interval_min;  /**< Minimum value for the connection interval. This shall be less than or equal to Conn_Interval_Max.
                                  Range: 0x0006 to 0x0C80, unit: 1.25 ms, time range: 7.5 ms to 4 s*/
     uint16_t interval_max;  /**< Maximum value for the connection interval. This shall be greater than or equal to Conn_Interval_Min.
                                  Range: 0x0006 to 0x0C80, unit: 1.25 ms, time range: 7.5 ms to 4 s.*/
     uint16_t slave_latency; /**< Slave latency for the connection in number of connection events. Range: 0x0000 to 0x01F3. */
     uint16_t sup_timeout;   /**< Supervision timeout for the LE link. Range: 0x000A to 0x0C80, unit: 10 ms, time range: 100 ms to 32 s. */
} ble_gap_evt_conn_param_update_req_t;

/** @brief Get Connection info event for @ref BLE_GAPC_EVT_CONN_INFO_GOT. */
typedef struct
{
    uint8_t             opcode;     /**< Operation code. See @ref ble_gap_get_conn_info_op_t. */
    ble_gap_conn_info_t info;       /**< Connection info. */
} ble_gap_evt_conn_info_t;

/** @brief Data Length Updated event for @ref BLE_GAPC_EVT_DATA_LENGTH_UPDATED. */
typedef struct
{
    uint16_t max_tx_octets; /**<  The maximum number of payload octets in TX. */
    uint16_t max_tx_time;   /**<  The maximum time that the local Controller will take to TX. */
    uint16_t max_rx_octets; /**<  The maximum number of payload octets in RX. */
    uint16_t max_rx_time;   /**<  The maximum time that the local Controller will take to RX. */
} ble_gap_evt_data_length_t;

/** @brief Device Information set event for @ref BLE_GAPC_EVT_DEV_INFO_SET. */
typedef struct
{
    ble_gap_dev_info_type_t info_type; /**< Device info type. see @ref ble_gap_dev_info_type_t. */
    ble_gapc_set_dev_info_t info;      /**< Device info data. see @ref ble_gap_cte_type_t. */
} ble_gap_evt_dev_info_set_t;

/** @brief Connection IQ Report info event for @ref BLE_GAPC_EVT_CONNECT_IQ_REPORT. */
typedef struct
{
    uint8_t  rx_phy;                              /**< Rx PHY (0x01: 1M | 0x02: 2M), see @ref BLE_GAP_PHYS. */
    uint8_t  data_channel_idx;                    /**< Data channel index, range 0x00 to 0x24. */
    int16_t  rssi;                                /**< RSSI units: 0.1 dBm, range -1270 to +200. */
    uint8_t  rssi_antenna_id;                     /**< RSSI antenna ID. */
    uint8_t  cte_type;                            /**< CTE type (0: GAP_CET_AOA | 1: GAP_CET_AOD_1US | 2: GAP_CET_AOD_2US), @see enum ble_gap_cte_type_t. */
    uint8_t  slot_dur;                            /**< Slot durations (1: GAP_SLOT_1US | 2: GAP_SLOT_2US), see @ref ble_gap_switching_sampling_type_t. */
    uint8_t  pkt_status;                          /**< Packet status, @see enum ble_gap_iq_report_status_t. */
    uint16_t con_evt_cnt;                         /**< Connection event counter. */
    uint8_t  nb_samples;                          /**< Number of samples. 0x00: no samples provided (only permitted if pkt_status is 0xFF),
                                                       0x09 to 0x52: total number of sample pairs. */
    int8_t i_sample[BLE_GAP_MAX_IQ_SAMPLE_NUM];   /**< The list of i samples for the reported PDU. */
    int8_t q_sample[BLE_GAP_MAX_IQ_SAMPLE_NUM];   /**< The list of q samples for the reported PDU. */
} ble_gap_evt_conn_iq_report_t;

/** @brief Connectionless IQ Report info event for @ref BLE_GAPC_EVT_CONNECTLESS_IQ_REPORT. */
typedef struct
{
    uint8_t  channel_idx;                         /**< The index of the channel on which the packet was received, range 0x00 to 0x24. */
    int16_t  rssi;                                /**< RSSI units: 0.1 dBm, range -1270 to +200. */
    uint8_t  rssi_antenna_id;                     /**< RSSI antenna ID. */
    uint8_t  cte_type;                            /**< CTE type (0: GAP_CET_AOA | 1: GAP_CET_AOD_1US | 2: GAP_CET_AOD_2US), @see enum ble_gap_cte_type_t. */
    uint8_t  slot_dur;                            /**< Slot durations (1: GAP_SLOT_1US | 2: GAP_SLOT_2US), see @ref ble_gap_switching_sampling_type_t. */
    uint8_t  pkt_status;                          /**< Packet status, @see enum ble_gap_iq_report_status_t. */
    uint16_t pa_evt_cnt;                          /**< Periodic advertising event counter. */
    uint8_t  nb_samples;                          /**< Number of samples. 0x00: no samples provided (only permitted if pkt_status is 0xFF),
                                                       0x09 to 0x52: total number of sample pairs. */
    int8_t i_sample[BLE_GAP_MAX_IQ_SAMPLE_NUM];   /**< The list of i samples for the reported PDU. */
    int8_t q_sample[BLE_GAP_MAX_IQ_SAMPLE_NUM];   /**< The list of q samples for the reported PDU. */
} ble_gap_evt_connless_iq_report_t;

/** @brief Le event notification reporting info event for @ref BLE_GAPC_EVT_LE_EVT_NOTI_REPORT. */
typedef struct
{
    int8_t   rssi;              /**< Rsssi value (dB). */
    uint8_t  channel;           /**< Connection channel. */
    uint16_t event_counter;     /**< Connection event counter. */
    uint16_t anchor_ts_hus;     /**< Ble time stamp of the anchor point with this event_counter, unit 0.5us. */
    uint32_t anchor_ts_hs;      /**< ble time stamp of the anchor point with this event_counter, unit 312.5us. */
} ble_gap_evt_le_event_noti_report_t;

/**@brief BLE GAPC event structure. */
typedef struct
{
    uint8_t  index;                                                          /**< Index of connection. */
    union                                                                    /**< union alternative identified by evt_id in enclosing struct. */
    {
        ble_gap_evt_phy_update_t                 phy_update;                 /**< PHY update parameters. */
        ble_gap_evt_connected_t                  connected;                  /**< Connection parameters. */
        ble_gap_evt_disconnected_t               disconnected;               /**< Disconnection parameters. See @ref BLE_STACK_ERROR_CODES. */
        ble_gap_evt_peer_name_get_t              peer_name;                  /**< Peer device name indication parameters. */
        ble_gap_evt_peer_info_t                  peer_info;                  /**< Peer info indication parameters. */
        ble_gap_evt_conn_param_updated_t         conn_param_updated;         /**< Connection parameter updated parameters. */
        ble_gap_evt_conn_param_update_req_t      conn_param_update_req;      /**< Connection parameter update request parameters. */
        ble_gap_evt_conn_info_t                  conn_info;                  /**< Connection info parameters. */
        ble_gap_evt_data_length_t                data_length;                /**< Data Length Update parameter. */
        ble_gap_evt_dev_info_set_t               dev_info_ind;               /**< Device info parameters. */
        ble_gap_evt_conn_iq_report_t             conn_iq_report;             /**< Connection IQ Report info parameters. */
        ble_gap_evt_connless_iq_report_t         connless_iq_report;         /**< Connectionless IQ Report info parameters. */
        ble_gap_evt_le_event_noti_report_t       le_evt_noti_report;         /**< Le event notification reporting info parameters. */
    } params;                                                                /**< Event Parameters. */
} ble_gapc_evt_t;

/** @} */

/**
 * @defgroup BLE_GAPC_FUNCTION Functions
 * @{
 */
/**
 ****************************************************************************************
 * @brief Terminate an existing connection.
 *
 * @param[in] conn_idx: The index of connection.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
  */
uint16_t ble_gap_disconnect(uint8_t conn_idx);

/**
 ****************************************************************************************
 * @brief Terminate an existing connection with a specified reason.
 *
 * @param[in] conn_idx: The index of connection.
 * @param[in] reason: The specified reason.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
  */
uint16_t ble_gap_disconnect_with_reason(uint8_t conn_idx, ble_gap_disconn_reason_t reason);

/**
 ****************************************************************************************
 * @brief Change the Link Layer connection parameters of a connection.
 *
 * @param[in] conn_idx:     The index of connection.
 * @param[in] p_conn_param: The new connection param.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_conn_param_update(uint8_t conn_idx, const ble_gap_conn_update_param_t *p_conn_param);

/**
 *****************************************************************************************
 * @brief  Set the method for updating connection parameter.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] use_l2cap_flag: Preferred to use l2cap to update connection parameter.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 *****************************************************************************************
 */
uint16_t ble_gap_update_conn_param_method_set(uint8_t conn_idx, bool use_l2cap_flag);

/**
 *****************************************************************************************
 * @brief Set connection's Latency.
 * @note  The latency shall be set to X value by LLCP firstly, then uses this API to change the latency in [0, X].
 *
 * @param[in] conn_idx:     The index of connection.
 * @param[in] latency:      The latency of connection.
 *                               
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 *****************************************************************************************
 */
uint16_t ble_gap_latency_set(uint8_t conn_idx, uint16_t latency);

/**
 *****************************************************************************************
 * @brief Get connection's Latency.
 * @note  This function is used to get connection's Latency.
 *
 * @param[in] conn_idx:     The index of connection.
 * @param[in] latency:      Pointer to the latency of connection.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 *****************************************************************************************
 */
uint16_t ble_gap_latency_get(uint8_t conn_idx, uint16_t *latency);

/**
 *****************************************************************************************
 * @brief Consult BLE connection activity plan situation function.
 * @note  This function should be called when connection established and no periodic advertising exists.
 *
 * @param[out] p_act_num:        Pointer to the number of existing connection activities.
 * @param[out] p_conn_plan_arr:  Pointer to the global array that stores planned connection activities.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 *****************************************************************************************
 */
uint16_t ble_gap_con_plan_consult(uint8_t *p_act_num, ble_gap_con_plan_tag_t **p_conn_plan_arr);

/**
 ****************************************************************************************
 * @brief Connection param update reply to peer device.
 *
 * @param[in] conn_idx:      The index of connection.
 * @param[in] accept: True to accept connection parameters, false to reject.
 *
 * @retval ::SDK_SUCCESS: Operation is success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_conn_param_update_reply(uint8_t conn_idx, bool accept);

/**
 ****************************************************************************************
 * @brief The suggested maximum transmission packet size and maximum packet transmission time to be used for a given connection.
 *
 * @param[in] conn_idx:   The index of connection.
 * @param[in] tx_octects: Preferred maximum number of payload octets that the local Controller should include in a single Link Layer packet on this connection.
 *            Range 0x001B-0x00FB (all other values reserved for future use).
 * @param[in] tx_time:    Preferred maximum number of microseconds that the local Controller should use to transmit a single Link Layer packet on this connection.
 *            Range 0x0148-0x4290 (all other values reserved for future use).
 *
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_data_length_update(uint8_t conn_idx,  uint16_t  tx_octects , uint16_t tx_time);

/**
 ****************************************************************************************
 * @brief Set the PHY preferences for the connection identified by the connection index.
 *
 * @param[in] conn_idx:   The index of connection.
 * @param[in] tx_phys: A bit field that indicates the transmitter PHYs that the Host prefers the Controller to use (see @ref ble_gap_phy_type_t).
 * @param[in] rx_phys: A bit field that indicates the receiver PHYs that the Host prefers the Controller to use (see @ref ble_gap_phy_type_t).
 * @param[in] phy_opt: A bit field that allows the Host to specify options for PHYs (see @ref ble_gap_phy_options_t).
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_phy_update(uint8_t conn_idx, uint8_t tx_phys , uint8_t rx_phys, uint8_t phy_opt);

/**
 ****************************************************************************************
 * @brief Get the information of the connection.
 *
 * @param[in] conn_idx: The index of connection.
 * @param[in] opcode:   The operation code. See @ref ble_gap_get_conn_info_op_t.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_conn_info_get(uint8_t conn_idx, ble_gap_get_conn_info_op_t opcode);

/**
 ****************************************************************************************
 * @brief Get the information of the peer device.
 *
 * @param[in] conn_idx: The index of connection.
 * @param[in] opcode:   The operation code. See @ref ble_gap_get_peer_info_op_t.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_gap_peer_info_get(uint8_t conn_idx, ble_gap_get_peer_info_op_t opcode);

/**
 ****************************************************************************************
 * @brief Get BD address of the bonded device.
 *
 * @param[in] conn_idx:     The index of connection.
 * @param[in] p_peer_addr:  Pointer to the peer BD addrss
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_bond_dev_addr_get(uint8_t conn_idx, ble_gap_bdaddr_t *p_peer_addr);

/**
 ****************************************************************************************
 * @brief Set connection CTE transmit parameters.
 *
 * @param[in] conn_idx:  The index of connection.
 * @param[in] param:     Set connection CTE transmit parameters info, see @ref ble_gap_set_conn_cte_trans_param_t.
 *
 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_conn_cte_trans_param_set(uint8_t conn_idx, ble_gap_set_conn_cte_trans_param_t *param);

/**
 ****************************************************************************************
 * @brief Set connection CTE receive parameters.
 *
 * @param[in] conn_idx:  The index of connection.
 * @param[in] param:     Set connection CTE receive parameters info, see @ref ble_gap_set_conn_cte_rcv_param_t.

 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_conn_cte_recv_param_set(uint8_t conn_idx, ble_gap_set_conn_cte_rcv_param_t *param);

/**
 ****************************************************************************************
 * @brief Set connection CTE request enable.
 *
 * @param[in] conn_idx:     The index of connection.
 * @param[in] enable_flag:  Wheter to request the cte for the connection. If enable_flag is set to false, the param shall be NULL.
 * @param[in] param:        Set connection CTE request enable info, see @ref ble_gap_set_conn_cte_req_enable_t.

 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_conn_cte_req_enable_set(uint8_t conn_idx, bool enable_flag, ble_gap_set_conn_cte_req_enable_t *param);

/**
 ****************************************************************************************
 * @brief Set connection CTE response enable.
 *
 * @param[in] conn_idx:     The index of connection.
 * @param[in] enable_flag:  Wheter to response the cte req for the connection.

 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_conn_cte_rsp_enable_set(uint8_t conn_idx, bool enable_flag);

/**
 ****************************************************************************************
 * @brief Get BD address of the local device by the conidx.
 *
 * @param[in] conidx: The index of conncetion.
 * @param[in] p_addr: Pointer to the local BD addrss
 ****************************************************************************************
 */
void ble_gap_get_local_addr_by_conidx(uint8_t conidx, uint8_t *p_addr);

/**
 ****************************************************************************************
 * @brief Enable or disable the reporting of le event notification.
 * @note  This API is asynchronous.
 * @note  After invoke this api, the event @ref BLE_GAPC_EVT_LE_EVT_NOTI_REPORT will be called.
 *
 * @param[in] conn_idx:     The index of connection.
 * @param[in] enable_flag:  The enable flag for reporting of le event notification.

 * @retval ::SDK_SUCCESS: Operation is Success.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 ****************************************************************************************
 */
uint16_t ble_gap_le_event_noti_report_enable_set(uint8_t conn_idx, bool enable_flag);

/**
 ****************************************************************************************
 * @brief Get connection link number.

 * @retval ::Return the number of connection link.
 ****************************************************************************************
 */
uint8_t ble_gap_conn_link_num_get(void);

/** @} */
#endif
/** @} */
/** @} */
/** @} */

/**
 ****************************************************************************************
 *
 * @file ble_id.h
 *
 * @brief define ble message id
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
 * @addtogroup BLE_ID
 * @{
 * @brief Message ID and hci cmd/evt opcode defines.
 */

#ifndef __BLE_ID_H__
#define __BLE_ID_H__

#include <stdint.h>

/** @addtogroup BLE_ID_DEFINES Defines
 * @{ */

/// Message identifier index
#define MSG_ID(task, idx) (TASK_FIRST_MSG((TASK_ID_ ## task)) + idx)

/// Build the first message ID of a task. (in fact a ke_msg_id_t)
#define TASK_FIRST_MSG(task) ((uint16_t)((task) << 8))

/// Default message ID
#define KE_MSG_DEFAULT_HANDLER  (0xFFFF)

/// Number of L2CC TASK Deprecated messages
#define L2CC_NB_DEPRECATED_MSG   (4)

/**@} */

/** @addtogroup BLE_ID_ENUMS Enums
 * @{ */

enum hci_opcode
{
    HCI_NO_OPERATION_CMD_OPCODE                         = 0x0000,

    //Link Control Commands
    HCI_INQ_CMD_OPCODE                                  = 0x0401,
    HCI_INQ_CANCEL_CMD_OPCODE                           = 0x0402,
    HCI_PER_INQ_MODE_CMD_OPCODE                         = 0x0403,
    HCI_EXIT_PER_INQ_MODE_CMD_OPCODE                    = 0x0404,
    HCI_CREATE_CON_CMD_OPCODE                           = 0x0405,
    HCI_DISCONNECT_CMD_OPCODE                           = 0x0406,
    HCI_CREATE_CON_CANCEL_CMD_OPCODE                    = 0x0408,
    HCI_ACCEPT_CON_REQ_CMD_OPCODE                       = 0x0409,
    HCI_REJECT_CON_REQ_CMD_OPCODE                       = 0x040A,
    HCI_LK_REQ_REPLY_CMD_OPCODE                         = 0x040B,
    HCI_LK_REQ_NEG_REPLY_CMD_OPCODE                     = 0x040C,
    HCI_PIN_CODE_REQ_REPLY_CMD_OPCODE                   = 0x040D,
    HCI_PIN_CODE_REQ_NEG_REPLY_CMD_OPCODE               = 0x040E,
    HCI_CHG_CON_PKT_TYPE_CMD_OPCODE                     = 0x040F,
    HCI_AUTH_REQ_CMD_OPCODE                             = 0x0411,
    HCI_SET_CON_ENC_CMD_OPCODE                          = 0x0413,
    HCI_CHG_CON_LK_CMD_OPCODE                           = 0x0415,
    HCI_MASTER_LK_CMD_OPCODE                            = 0x0417,
    HCI_REM_NAME_REQ_CMD_OPCODE                         = 0x0419,
    HCI_REM_NAME_REQ_CANCEL_CMD_OPCODE                  = 0x041A,
    HCI_RD_REM_SUPP_FEATS_CMD_OPCODE                    = 0x041B,
    HCI_RD_REM_EXT_FEATS_CMD_OPCODE                     = 0x041C,
    HCI_RD_REM_VER_INFO_CMD_OPCODE                      = 0x041D,
    HCI_RD_CLK_OFF_CMD_OPCODE                           = 0x041F,
    HCI_RD_LMP_HDL_CMD_OPCODE                           = 0x0420,
    HCI_SETUP_SYNC_CON_CMD_OPCODE                       = 0x0428,
    HCI_ACCEPT_SYNC_CON_REQ_CMD_OPCODE                  = 0x0429,
    HCI_REJECT_SYNC_CON_REQ_CMD_OPCODE                  = 0x042A,
    HCI_IO_CAP_REQ_REPLY_CMD_OPCODE                     = 0x042B,
    HCI_USER_CFM_REQ_REPLY_CMD_OPCODE                   = 0x042C,
    HCI_USER_CFM_REQ_NEG_REPLY_CMD_OPCODE               = 0x042D,
    HCI_USER_PASSKEY_REQ_REPLY_CMD_OPCODE               = 0x042E,
    HCI_USER_PASSKEY_REQ_NEG_REPLY_CMD_OPCODE           = 0x042F,
    HCI_REM_OOB_DATA_REQ_REPLY_CMD_OPCODE               = 0x0430,
    HCI_REM_OOB_DATA_REQ_NEG_REPLY_CMD_OPCODE           = 0x0433,
    HCI_IO_CAP_REQ_NEG_REPLY_CMD_OPCODE                 = 0x0434,
    HCI_ENH_SETUP_SYNC_CON_CMD_OPCODE                   = 0x043D,
    HCI_ENH_ACCEPT_SYNC_CON_CMD_OPCODE                  = 0x043E,
    HCI_TRUNC_PAGE_CMD_OPCODE                           = 0x043F,
    HCI_TRUNC_PAGE_CAN_CMD_OPCODE                       = 0x0440,
    HCI_SET_CON_SLV_BCST_CMD_OPCODE                     = 0x0441,
    HCI_SET_CON_SLV_BCST_REC_CMD_OPCODE                 = 0x0442,
    HCI_START_SYNC_TRAIN_CMD_OPCODE                     = 0x0443,
    HCI_REC_SYNC_TRAIN_CMD_OPCODE                       = 0x0444,
    HCI_REM_OOB_EXT_DATA_REQ_REPLY_CMD_OPCODE           = 0x0445,

    //Link Policy Commands
    HCI_HOLD_MODE_CMD_OPCODE                            = 0x0801,
    HCI_SNIFF_MODE_CMD_OPCODE                           = 0x0803,
    HCI_EXIT_SNIFF_MODE_CMD_OPCODE                      = 0x0804,
    HCI_PARK_STATE_CMD_OPCODE                           = 0x0805,
    HCI_EXIT_PARK_STATE_CMD_OPCODE                      = 0x0806,
    HCI_QOS_SETUP_CMD_OPCODE                            = 0x0807,
    HCI_ROLE_DISCOVERY_CMD_OPCODE                       = 0x0809,
    HCI_SWITCH_ROLE_CMD_OPCODE                          = 0x080B,
    HCI_RD_LINK_POL_STG_CMD_OPCODE                      = 0x080C,
    HCI_WR_LINK_POL_STG_CMD_OPCODE                      = 0x080D,
    HCI_RD_DFT_LINK_POL_STG_CMD_OPCODE                  = 0x080E,
    HCI_WR_DFT_LINK_POL_STG_CMD_OPCODE                  = 0x080F,
    HCI_FLOW_SPEC_CMD_OPCODE                            = 0x0810,
    HCI_SNIFF_SUB_CMD_OPCODE                            = 0x0811,

    //Controller and Baseband Commands
    HCI_SET_EVT_MASK_CMD_OPCODE                         = 0x0C01,
    HCI_RESET_CMD_OPCODE                                = 0x0C03,
    HCI_SET_EVT_FILTER_CMD_OPCODE                       = 0x0C05,
    HCI_FLUSH_CMD_OPCODE                                = 0x0C08,
    HCI_RD_PIN_TYPE_CMD_OPCODE                          = 0x0C09,
    HCI_WR_PIN_TYPE_CMD_OPCODE                          = 0x0C0A,
    HCI_CREATE_NEW_UNIT_KEY_CMD_OPCODE                  = 0x0C0B,
    HCI_RD_STORED_LK_CMD_OPCODE                         = 0x0C0D,
    HCI_WR_STORED_LK_CMD_OPCODE                         = 0x0C11,
    HCI_DEL_STORED_LK_CMD_OPCODE                        = 0x0C12,
    HCI_WR_LOCAL_NAME_CMD_OPCODE                        = 0x0C13,
    HCI_RD_LOCAL_NAME_CMD_OPCODE                        = 0x0C14,
    HCI_RD_CON_ACCEPT_TO_CMD_OPCODE                     = 0x0C15,
    HCI_WR_CON_ACCEPT_TO_CMD_OPCODE                     = 0x0C16,
    HCI_RD_PAGE_TO_CMD_OPCODE                           = 0x0C17,
    HCI_WR_PAGE_TO_CMD_OPCODE                           = 0x0C18,
    HCI_RD_SCAN_EN_CMD_OPCODE                           = 0x0C19,
    HCI_WR_SCAN_EN_CMD_OPCODE                           = 0x0C1A,
    HCI_RD_PAGE_SCAN_ACT_CMD_OPCODE                     = 0x0C1B,
    HCI_WR_PAGE_SCAN_ACT_CMD_OPCODE                     = 0x0C1C,
    HCI_RD_INQ_SCAN_ACT_CMD_OPCODE                      = 0x0C1D,
    HCI_WR_INQ_SCAN_ACT_CMD_OPCODE                      = 0x0C1E,
    HCI_RD_AUTH_EN_CMD_OPCODE                           = 0x0C1F,
    HCI_WR_AUTH_EN_CMD_OPCODE                           = 0x0C20,
    HCI_RD_CLASS_OF_DEV_CMD_OPCODE                      = 0x0C23,
    HCI_WR_CLASS_OF_DEV_CMD_OPCODE                      = 0x0C24,
    HCI_RD_VOICE_STG_CMD_OPCODE                         = 0x0C25,
    HCI_WR_VOICE_STG_CMD_OPCODE                         = 0x0C26,
    HCI_RD_AUTO_FLUSH_TO_CMD_OPCODE                     = 0x0C27,
    HCI_WR_AUTO_FLUSH_TO_CMD_OPCODE                     = 0x0C28,
    HCI_RD_NB_BDCST_RETX_CMD_OPCODE                     = 0x0C29,
    HCI_WR_NB_BDCST_RETX_CMD_OPCODE                     = 0x0C2A,
    HCI_RD_HOLD_MODE_ACTIVITY_CMD_OPCODE                = 0x0C2B,
    HCI_WR_HOLD_MODE_ACTIVITY_CMD_OPCODE                = 0x0C2C,
    HCI_RD_TX_PWR_LVL_CMD_OPCODE                        = 0x0C2D,
    HCI_RD_SYNC_FLOW_CTRL_EN_CMD_OPCODE                 = 0x0C2E,
    HCI_WR_SYNC_FLOW_CTRL_EN_CMD_OPCODE                 = 0x0C2F,
    HCI_SET_CTRL_TO_HOST_FLOW_CTRL_CMD_OPCODE           = 0x0C31,
    HCI_HOST_BUF_SIZE_CMD_OPCODE                        = 0x0C33,
    HCI_HOST_NB_CMP_PKTS_CMD_OPCODE                     = 0x0C35,
    HCI_RD_LINK_SUPV_TO_CMD_OPCODE                      = 0x0C36,
    HCI_WR_LINK_SUPV_TO_CMD_OPCODE                      = 0x0C37,
    HCI_RD_NB_SUPP_IAC_CMD_OPCODE                       = 0x0C38,
    HCI_RD_CURR_IAC_LAP_CMD_OPCODE                      = 0x0C39,
    HCI_WR_CURR_IAC_LAP_CMD_OPCODE                      = 0x0C3A,
    HCI_SET_AFH_HOST_CH_CLASS_CMD_OPCODE                = 0x0C3F,
    HCI_RD_INQ_SCAN_TYPE_CMD_OPCODE                     = 0x0C42,
    HCI_WR_INQ_SCAN_TYPE_CMD_OPCODE                     = 0x0C43,
    HCI_RD_INQ_MODE_CMD_OPCODE                          = 0x0C44,
    HCI_WR_INQ_MODE_CMD_OPCODE                          = 0x0C45,
    HCI_RD_PAGE_SCAN_TYPE_CMD_OPCODE                    = 0x0C46,
    HCI_WR_PAGE_SCAN_TYPE_CMD_OPCODE                    = 0x0C47,
    HCI_RD_AFH_CH_ASSESS_MODE_CMD_OPCODE                = 0x0C48,
    HCI_WR_AFH_CH_ASSESS_MODE_CMD_OPCODE                = 0x0C49,
    HCI_RD_EXT_INQ_RSP_CMD_OPCODE                       = 0x0C51,
    HCI_WR_EXT_INQ_RSP_CMD_OPCODE                       = 0x0C52,
    HCI_REFRESH_ENC_KEY_CMD_OPCODE                      = 0x0C53,
    HCI_RD_SP_MODE_CMD_OPCODE                           = 0x0C55,
    HCI_WR_SP_MODE_CMD_OPCODE                           = 0x0C56,
    HCI_RD_LOC_OOB_DATA_CMD_OPCODE                      = 0x0C57,
    HCI_RD_INQ_RSP_TX_PWR_LVL_CMD_OPCODE                = 0x0C58,
    HCI_WR_INQ_TX_PWR_LVL_CMD_OPCODE                    = 0x0C59,
    HCI_RD_DFT_ERR_DATA_REP_CMD_OPCODE                  = 0x0C5A,
    HCI_WR_DFT_ERR_DATA_REP_CMD_OPCODE                  = 0x0C5B,
    HCI_ENH_FLUSH_CMD_OPCODE                            = 0x0C5F,
    HCI_SEND_KEYPRESS_NOTIF_CMD_OPCODE                  = 0x0C60,
    HCI_SET_EVT_MASK_PAGE_2_CMD_OPCODE                  = 0x0C63,
    HCI_RD_FLOW_CNTL_MODE_CMD_OPCODE                    = 0x0C66,
    HCI_WR_FLOW_CNTL_MODE_CMD_OPCODE                    = 0x0C67,
    HCI_RD_ENH_TX_PWR_LVL_CMD_OPCODE                    = 0x0C68,
    HCI_RD_LE_HOST_SUPP_CMD_OPCODE                      = 0x0C6C,
    HCI_WR_LE_HOST_SUPP_CMD_OPCODE                      = 0x0C6D,
    HCI_SET_MWS_CHANNEL_PARAMS_CMD_OPCODE               = 0x0C6E,
    HCI_SET_EXTERNAL_FRAME_CONFIG_CMD_OPCODE            = 0x0C6F,
    HCI_SET_MWS_SIGNALING_CMD_OPCODE                    = 0x0C70,
    HCI_SET_MWS_TRANSPORT_LAYER_CMD_OPCODE              = 0x0C71,
    HCI_SET_MWS_SCAN_FREQ_TABLE_CMD_OPCODE              = 0x0C72,
    HCI_SET_MWS_PATTERN_CONFIG_CMD_OPCODE               = 0x0C73,
    HCI_SET_RES_LT_ADDR_CMD_OPCODE                      = 0x0C74,
    HCI_DEL_RES_LT_ADDR_CMD_OPCODE                      = 0x0C75,
    HCI_SET_CON_SLV_BCST_DATA_CMD_OPCODE                = 0x0C76,
    HCI_RD_SYNC_TRAIN_PARAM_CMD_OPCODE                  = 0x0C77,
    HCI_WR_SYNC_TRAIN_PARAM_CMD_OPCODE                  = 0x0C78,
    HCI_RD_SEC_CON_HOST_SUPP_CMD_OPCODE                 = 0x0C79,
    HCI_WR_SEC_CON_HOST_SUPP_CMD_OPCODE                 = 0x0C7A,
    HCI_RD_AUTH_PAYL_TO_CMD_OPCODE                      = 0x0C7B,
    HCI_WR_AUTH_PAYL_TO_CMD_OPCODE                      = 0x0C7C,
    HCI_RD_LOC_OOB_EXT_DATA_CMD_OPCODE                  = 0x0C7D,
    HCI_RD_EXT_PAGE_TO_CMD_OPCODE                       = 0x0C7E,
    HCI_WR_EXT_PAGE_TO_CMD_OPCODE                       = 0x0C7F,
    HCI_RD_EXT_INQ_LEN_CMD_OPCODE                       = 0x0C80,
    HCI_WR_EXT_INQ_LEN_CMD_OPCODE                       = 0x0C81,
    HCI_SET_ECO_BASE_INTV_CMD_OPCODE                    = 0x0C82,
    HCI_CONFIG_DATA_PATH_CMD_OPCODE                     = 0x0C83,

    //Info Params
    HCI_RD_LOCAL_VER_INFO_CMD_OPCODE                    = 0x1001,
    HCI_RD_LOCAL_SUPP_CMDS_CMD_OPCODE                   = 0x1002,
    HCI_RD_LOCAL_SUPP_FEATS_CMD_OPCODE                  = 0x1003,
    HCI_RD_LOCAL_EXT_FEATS_CMD_OPCODE                   = 0x1004,
    HCI_RD_BUF_SIZE_CMD_OPCODE                          = 0x1005,
    HCI_RD_BD_ADDR_CMD_OPCODE                           = 0x1009,
    HCI_RD_DATA_BLOCK_SIZE_CMD_OPCODE                   = 0x100A,
    HCI_RD_LOCAL_SUPP_CODECS_CMD_OPCODE                 = 0x100B,
    HCI_RD_LOCAL_SP_OPT_CMD_OPCODE                      = 0x100C,
    HCI_RD_LOCAL_SUPP_CODECS_V2_CMD_OPCODE              = 0x100D,
    HCI_RD_LOCAL_SUPP_CODEC_CAP_CMD_OPCODE              = 0x100E,
    HCI_RD_LOCAL_SUPP_CTRL_DELAY_CMD_OPCODE             = 0x100F,

    //Status Params
    HCI_RD_FAIL_CONTACT_CNT_CMD_OPCODE                  = 0x1401,
    HCI_RST_FAIL_CONTACT_CNT_CMD_OPCODE                 = 0x1402,
    HCI_RD_LINK_QUAL_CMD_OPCODE                         = 0x1403,
    HCI_RD_RSSI_CMD_OPCODE                              = 0x1405,
    HCI_RD_AFH_CH_MAP_CMD_OPCODE                        = 0x1406,
    HCI_RD_CLK_CMD_OPCODE                               = 0x1407,
    HCI_RD_ENC_KEY_SIZE_CMD_OPCODE                      = 0x1408,
    HCI_GET_MWS_TRANSPORT_LAYER_CONFIG_CMD_OPCODE       = 0x140C,

    //Testing Commands
    HCI_RD_LOOPBACK_MODE_CMD_OPCODE                     = 0x1801,
    HCI_WR_LOOPBACK_MODE_CMD_OPCODE                     = 0x1802,
    HCI_EN_DUT_MODE_CMD_OPCODE                          = 0x1803,
    HCI_WR_SP_DBG_MODE_CMD_OPCODE                       = 0x1804,
    HCI_WR_SEC_CON_TEST_MODE_CMD_OPCODE                 = 0x180A,

    /// LE Commands Opcodes
    HCI_LE_SET_EVT_MASK_CMD_OPCODE                      = 0x2001,
    HCI_LE_RD_BUF_SIZE_CMD_OPCODE                       = 0x2002,
    HCI_LE_RD_LOCAL_SUPP_FEATS_CMD_OPCODE               = 0x2003,
    HCI_LE_SET_RAND_ADDR_CMD_OPCODE                     = 0x2005,
    HCI_LE_SET_ADV_PARAM_CMD_OPCODE                     = 0x2006,
    HCI_LE_RD_ADV_CHNL_TX_PW_CMD_OPCODE                 = 0x2007,
    HCI_LE_SET_ADV_DATA_CMD_OPCODE                      = 0x2008,
    HCI_LE_SET_SCAN_RSP_DATA_CMD_OPCODE                 = 0x2009,
    HCI_LE_SET_ADV_EN_CMD_OPCODE                        = 0x200A,
    HCI_LE_SET_SCAN_PARAM_CMD_OPCODE                    = 0x200B,
    HCI_LE_SET_SCAN_EN_CMD_OPCODE                       = 0x200C,
    HCI_LE_CREATE_CON_CMD_OPCODE                        = 0x200D,
    HCI_LE_CREATE_CON_CANCEL_CMD_OPCODE                 = 0x200E,
    HCI_LE_RD_WLST_SIZE_CMD_OPCODE                      = 0x200F,
    HCI_LE_CLEAR_WLST_CMD_OPCODE                        = 0x2010,
    HCI_LE_ADD_DEV_TO_WLST_CMD_OPCODE                   = 0x2011,
    HCI_LE_RMV_DEV_FROM_WLST_CMD_OPCODE                 = 0x2012,
    HCI_LE_CON_UPDATE_CMD_OPCODE                        = 0x2013,
    HCI_LE_SET_HOST_CH_CLASS_CMD_OPCODE                 = 0x2014,
    HCI_LE_RD_CHNL_MAP_CMD_OPCODE                       = 0x2015,
    HCI_LE_RD_REM_FEATS_CMD_OPCODE                      = 0x2016,
    HCI_LE_ENC_CMD_OPCODE                               = 0x2017,
    HCI_LE_RAND_CMD_OPCODE                              = 0x2018,
    HCI_LE_EN_ENC_CMD_OPCODE                            = 0x2019,
    HCI_LE_LTK_REQ_REPLY_CMD_OPCODE                     = 0x201A,
    HCI_LE_LTK_REQ_NEG_REPLY_CMD_OPCODE                 = 0x201B,
    HCI_LE_RD_SUPP_STATES_CMD_OPCODE                    = 0x201C,
    HCI_LE_RX_TEST_V1_CMD_OPCODE                        = 0x201D,
    HCI_LE_TX_TEST_V1_CMD_OPCODE                        = 0x201E,
    HCI_LE_TEST_END_CMD_OPCODE                          = 0x201F,
    HCI_LE_REM_CON_PARAM_REQ_REPLY_CMD_OPCODE           = 0x2020,
    HCI_LE_REM_CON_PARAM_REQ_NEG_REPLY_CMD_OPCODE       = 0x2021,
    HCI_LE_SET_DATA_LEN_CMD_OPCODE                      = 0x2022,
    HCI_LE_RD_SUGGTED_DFT_DATA_LEN_CMD_OPCODE           = 0x2023,
    HCI_LE_WR_SUGGTED_DFT_DATA_LEN_CMD_OPCODE           = 0x2024,
    HCI_LE_RD_LOC_P256_PUB_KEY_CMD_OPCODE               = 0x2025,
    HCI_LE_GEN_DHKEY_V1_CMD_OPCODE                      = 0x2026,
    HCI_LE_ADD_DEV_TO_RSLV_LIST_CMD_OPCODE              = 0x2027,
    HCI_LE_RMV_DEV_FROM_RSLV_LIST_CMD_OPCODE            = 0x2028,
    HCI_LE_CLEAR_RSLV_LIST_CMD_OPCODE                   = 0x2029,
    HCI_LE_RD_RSLV_LIST_SIZE_CMD_OPCODE                 = 0x202A,
    HCI_LE_RD_PEER_RSLV_ADDR_CMD_OPCODE                 = 0x202B,
    HCI_LE_RD_LOC_RSLV_ADDR_CMD_OPCODE                  = 0x202C,
    HCI_LE_SET_ADDR_RESOL_EN_CMD_OPCODE                 = 0x202D,
    HCI_LE_SET_RSLV_PRIV_ADDR_TO_CMD_OPCODE             = 0x202E,
    HCI_LE_RD_MAX_DATA_LEN_CMD_OPCODE                   = 0x202F,
    HCI_LE_RD_PHY_CMD_OPCODE                            = 0x2030,
    HCI_LE_SET_DFT_PHY_CMD_OPCODE                       = 0x2031,
    HCI_LE_SET_PHY_CMD_OPCODE                           = 0x2032,
    HCI_LE_RX_TEST_V2_CMD_OPCODE                        = 0x2033,
    HCI_LE_TX_TEST_V2_CMD_OPCODE                        = 0x2034,
    HCI_LE_SET_ADV_SET_RAND_ADDR_CMD_OPCODE             = 0x2035,
    HCI_LE_SET_EXT_ADV_PARAM_CMD_OPCODE                 = 0x2036,
    HCI_LE_SET_EXT_ADV_DATA_CMD_OPCODE                  = 0x2037,
    HCI_LE_SET_EXT_SCAN_RSP_DATA_CMD_OPCODE             = 0x2038,
    HCI_LE_SET_EXT_ADV_EN_CMD_OPCODE                    = 0x2039,
    HCI_LE_RD_MAX_ADV_DATA_LEN_CMD_OPCODE               = 0x203A,
    HCI_LE_RD_NB_SUPP_ADV_SETS_CMD_OPCODE               = 0x203B,
    HCI_LE_RMV_ADV_SET_CMD_OPCODE                       = 0x203C,
    HCI_LE_CLEAR_ADV_SETS_CMD_OPCODE                    = 0x203D,
    HCI_LE_SET_PER_ADV_PARAM_CMD_OPCODE                 = 0x203E,
    HCI_LE_SET_PER_ADV_DATA_CMD_OPCODE                  = 0x203F,
    HCI_LE_SET_PER_ADV_EN_CMD_OPCODE                    = 0x2040,
    HCI_LE_SET_EXT_SCAN_PARAM_CMD_OPCODE                = 0x2041,
    HCI_LE_SET_EXT_SCAN_EN_CMD_OPCODE                   = 0x2042,
    HCI_LE_EXT_CREATE_CON_CMD_OPCODE                    = 0x2043,
    HCI_LE_PER_ADV_CREATE_SYNC_CMD_OPCODE               = 0x2044,
    HCI_LE_PER_ADV_CREATE_SYNC_CANCEL_CMD_OPCODE        = 0x2045,
    HCI_LE_PER_ADV_TERM_SYNC_CMD_OPCODE                 = 0x2046,
    HCI_LE_ADD_DEV_TO_PER_ADV_LIST_CMD_OPCODE           = 0x2047,
    HCI_LE_RMV_DEV_FROM_PER_ADV_LIST_CMD_OPCODE         = 0x2048,
    HCI_LE_CLEAR_PER_ADV_LIST_CMD_OPCODE                = 0x2049,
    HCI_LE_RD_PER_ADV_LIST_SIZE_CMD_OPCODE              = 0x204A,
    HCI_LE_RD_TX_PWR_CMD_OPCODE                         = 0x204B,
    HCI_LE_RD_RF_PATH_COMP_CMD_OPCODE                   = 0x204C,
    HCI_LE_WR_RF_PATH_COMP_CMD_OPCODE                   = 0x204D,
    HCI_LE_SET_PRIV_MODE_CMD_OPCODE                     = 0x204E,
    HCI_LE_RX_TEST_V3_CMD_OPCODE                        = 0x204F,
    HCI_LE_TX_TEST_V3_CMD_OPCODE                        = 0x2050,
    HCI_LE_SET_CONLESS_CTE_TX_PARAM_CMD_OPCODE          = 0x2051,
    HCI_LE_SET_CONLESS_CTE_TX_EN_CMD_OPCODE             = 0x2052,
    HCI_LE_SET_CONLESS_IQ_SAMPL_EN_CMD_OPCODE           = 0x2053,
    HCI_LE_SET_CON_CTE_RX_PARAM_CMD_OPCODE              = 0x2054,
    HCI_LE_SET_CON_CTE_TX_PARAM_CMD_OPCODE              = 0x2055,
    HCI_LE_CON_CTE_REQ_EN_CMD_OPCODE                    = 0x2056,
    HCI_LE_CON_CTE_RSP_EN_CMD_OPCODE                    = 0x2057,
    HCI_LE_RD_ANTENNA_INF_CMD_OPCODE                    = 0x2058,
    HCI_LE_SET_PER_ADV_REC_EN_CMD_OPCODE                = 0x2059,
    HCI_LE_PER_ADV_SYNC_TRANSF_CMD_OPCODE               = 0x205A,
    HCI_LE_PER_ADV_SET_INFO_TRANSF_CMD_OPCODE           = 0x205B,
    HCI_LE_SET_PER_ADV_SYNC_TRANSF_PARAM_CMD_OPCODE     = 0x205C,
    HCI_LE_SET_DFT_PER_ADV_SYNC_TRANSF_PARAM_CMD_OPCODE = 0x205D,
    HCI_LE_GEN_DHKEY_V2_CMD_OPCODE                      = 0x205E,
    HCI_LE_MOD_SLEEP_CLK_ACC_CMD_OPCODE                 = 0x205F,
    HCI_LE_RD_BUF_SIZE_V2_CMD_OPCODE                    = 0x2060,
    HCI_LE_RD_ISO_TX_SYNC_CMD_OPCODE                    = 0x2061,
    HCI_LE_SET_CIG_PARAMS_CMD_OPCODE                    = 0x2062,
    HCI_LE_SET_CIG_PARAMS_TEST_CMD_OPCODE               = 0x2063,
    HCI_LE_CREATE_CIS_CMD_OPCODE                        = 0x2064,
    HCI_LE_REMOVE_CIG_CMD_OPCODE                        = 0x2065,
    HCI_LE_ACCEPT_CIS_REQ_CMD_OPCODE                    = 0x2066,
    HCI_LE_REJECT_CIS_REQ_CMD_OPCODE                    = 0x2067,
    HCI_LE_CREATE_BIG_CMD_OPCODE                        = 0x2068,
    HCI_LE_CREATE_BIG_TEST_CMD_OPCODE                   = 0x2069,
    HCI_LE_TERMINATE_BIG_CMD_OPCODE                     = 0x206A,
    HCI_LE_BIG_CREATE_SYNC_CMD_OPCODE                   = 0x206B,
    HCI_LE_BIG_TERMINATE_SYNC_CMD_OPCODE                = 0x206C,
    HCI_LE_REQ_PEER_SCA_CMD_OPCODE                      = 0x206D,
    HCI_LE_SETUP_ISO_DATA_PATH_CMD_OPCODE               = 0x206E,
    HCI_LE_REMOVE_ISO_DATA_PATH_CMD_OPCODE              = 0x206F,
    HCI_LE_ISO_TX_TEST_CMD_OPCODE                       = 0x2070,
    HCI_LE_ISO_RX_TEST_CMD_OPCODE                       = 0x2071,
    HCI_LE_ISO_READ_TEST_COUNTERS_CMD_OPCODE            = 0x2072,
    HCI_LE_ISO_TEST_END_CMD_OPCODE                      = 0x2073,
    HCI_LE_SET_HOST_FEATURE_CMD_OPCODE                  = 0x2074,
    HCI_LE_RD_ISO_LINK_QUALITY_CMD_OPCODE               = 0x2075,
    HCI_LE_ENH_RD_TX_PWR_LVL_CMD_OPCODE                 = 0x2076,
    HCI_LE_RD_REMOTE_TX_PWR_LVL_CMD_OPCODE              = 0x2077,
    HCI_LE_SET_PATH_LOSS_REP_PARAM_CMD_OPCODE           = 0x2078,
    HCI_LE_SET_PATH_LOSS_REP_EN_CMD_OPCODE              = 0x2079,
    HCI_LE_SET_TX_POWER_REP_EN_CMD_OPCODE               = 0x207A,
    HCI_LE_TX_TEST_V4_CMD_OPCODE                        = 0x207B,
    HCI_LE_SET_GDX_RANGING_PARAMS_CMD_OPCODE            = 0x20F9,
    HCI_LE_START_RANGING_CMD_OPCODE                     = 0x20FA,
    HCI_PUBLIC_ADDR_SET_CMD_OPCODE                      = 0x20FB,
    HCI_LE_TIME_SYNC_EN_CMD_OPCODE                      = 0x20FC,

    ///Debug commands - OGF = 0x3F (spec)
    HCI_DBG_RD_MEM_CMD_OPCODE                           = 0xFC01,
    HCI_DBG_WR_MEM_CMD_OPCODE                           = 0xFC02,
    HCI_DBG_DEL_PAR_CMD_OPCODE                          = 0xFC03,
    HCI_DBG_ID_FLASH_CMD_OPCODE                         = 0xFC05,
    HCI_DBG_ER_FLASH_CMD_OPCODE                         = 0xFC06,
    HCI_DBG_WR_FLASH_CMD_OPCODE                         = 0xFC07,
    HCI_DBG_RD_FLASH_CMD_OPCODE                         = 0xFC08,
    HCI_DBG_RD_PAR_CMD_OPCODE                           = 0xFC09,
    HCI_DBG_WR_PAR_CMD_OPCODE                           = 0xFC0A,
    HCI_DBG_WLAN_COEX_CMD_OPCODE                        = 0xFC0B,
    HCI_DBG_WLAN_COEXTST_SCEN_CMD_OPCODE                = 0xFC0D,
    HCI_DBG_BT_SEND_LMP_CMD_OPCODE                      = 0xFC0E,
    HCI_DBG_SET_LOCAL_CLOCK_CMD_OPCODE                  = 0xFC0F,
    HCI_DBG_RD_KE_STATS_CMD_OPCODE                      = 0xFC10,
    HCI_DBG_PLF_RESET_CMD_OPCODE                        = 0xFC11,
    HCI_DBG_RD_MEM_INFO_CMD_OPCODE                      = 0xFC12,
    HCI_VS_SET_PREF_SLAVE_LATENCY_CMD_OPCODE            = 0xFC13,
    HCI_VS_SET_PREF_SLAVE_EVT_DUR_CMD_OPCODE            = 0xFC14,
    HCI_VS_SET_MAX_RX_SIZE_AND_TIME_CMD_OPCODE          = 0xFC15,
};

enum hci_evt_code
{
    HCI_INQ_CMP_EVT_CODE                       = 0x01,
    HCI_INQ_RES_EVT_CODE                       = 0x02,
    HCI_CON_CMP_EVT_CODE                       = 0x03,
    HCI_CON_REQ_EVT_CODE                       = 0x04,
    HCI_DISC_CMP_EVT_CODE                      = 0x05,
    HCI_AUTH_CMP_EVT_CODE                      = 0x06,
    HCI_REM_NAME_REQ_CMP_EVT_CODE              = 0x07,
    HCI_ENC_CHG_EVT_CODE                       = 0x08,
    HCI_CHG_CON_LK_CMP_EVT_CODE                = 0x09,
    HCI_MASTER_LK_CMP_EVT_CODE                 = 0x0A,
    HCI_RD_REM_SUPP_FEATS_CMP_EVT_CODE         = 0x0B,
    HCI_RD_REM_VER_INFO_CMP_EVT_CODE           = 0x0C,
    HCI_QOS_SETUP_CMP_EVT_CODE                 = 0x0D,
    HCI_CMD_CMP_EVT_CODE                       = 0x0E,
    HCI_CMD_STATUS_EVT_CODE                    = 0x0F,
    HCI_HW_ERR_EVT_CODE                        = 0x10,
    HCI_FLUSH_OCCURRED_EVT_CODE                = 0x11,
    HCI_ROLE_CHG_EVT_CODE                      = 0x12,
    HCI_NB_CMP_PKTS_EVT_CODE                   = 0x13,
    HCI_MODE_CHG_EVT_CODE                      = 0x14,
    HCI_RETURN_LINK_KEYS_EVT_CODE              = 0x15,
    HCI_PIN_CODE_REQ_EVT_CODE                  = 0x16,
    HCI_LK_REQ_EVT_CODE                        = 0x17,
    HCI_LK_NOTIF_EVT_CODE                      = 0x18,
    HCI_DATA_BUF_OVFLW_EVT_CODE                = 0x1A,
    HCI_MAX_SLOT_CHG_EVT_CODE                  = 0x1B,
    HCI_RD_CLK_OFF_CMP_EVT_CODE                = 0x1C,
    HCI_CON_PKT_TYPE_CHG_EVT_CODE              = 0x1D,
    HCI_QOS_VIOL_EVT_CODE                      = 0x1E,
    HCI_PAGE_SCAN_REPET_MODE_CHG_EVT_CODE      = 0x20,
    HCI_FLOW_SPEC_CMP_EVT_CODE                 = 0x21,
    HCI_INQ_RES_WITH_RSSI_EVT_CODE             = 0x22,
    HCI_RD_REM_EXT_FEATS_CMP_EVT_CODE          = 0x23,
    HCI_SYNC_CON_CMP_EVT_CODE                  = 0x2C,
    HCI_SYNC_CON_CHG_EVT_CODE                  = 0x2D,
    HCI_SNIFF_SUB_EVT_CODE                     = 0x2E,
    HCI_EXT_INQ_RES_EVT_CODE                   = 0x2F,
    HCI_ENC_KEY_REFRESH_CMP_EVT_CODE           = 0x30,
    HCI_IO_CAP_REQ_EVT_CODE                    = 0x31,
    HCI_IO_CAP_RSP_EVT_CODE                    = 0x32,
    HCI_USER_CFM_REQ_EVT_CODE                  = 0x33,
    HCI_USER_PASSKEY_REQ_EVT_CODE              = 0x34,
    HCI_REM_OOB_DATA_REQ_EVT_CODE              = 0x35,
    HCI_SP_CMP_EVT_CODE                        = 0x36,
    HCI_LINK_SUPV_TO_CHG_EVT_CODE              = 0x38,
    HCI_ENH_FLUSH_CMP_EVT_CODE                 = 0x39,
    HCI_USER_PASSKEY_NOTIF_EVT_CODE            = 0x3B,
    HCI_KEYPRESS_NOTIF_EVT_CODE                = 0x3C,
    HCI_REM_HOST_SUPP_FEATS_NOTIF_EVT_CODE     = 0x3D,
    HCI_LE_META_EVT_CODE                       = 0x3E,
    HCI_MAX_EVT_MSK_PAGE_1_CODE                = 0x40,
    HCI_SYNC_TRAIN_CMP_EVT_CODE                = 0x4F,
    HCI_SYNC_TRAIN_REC_EVT_CODE                = 0x50,
    HCI_CON_SLV_BCST_REC_EVT_CODE              = 0x51,
    HCI_CON_SLV_BCST_TO_EVT_CODE               = 0x52,
    HCI_TRUNC_PAGE_CMP_EVT_CODE                = 0x53,
    HCI_SLV_PAGE_RSP_TO_EVT_CODE               = 0x54,
    HCI_CON_SLV_BCST_CH_MAP_CHG_EVT_CODE       = 0x55,
    HCI_AUTH_PAYL_TO_EXP_EVT_CODE              = 0x57,
    HCI_SAM_STATUS_CHANGE_EVT_CODE             = 0x58,
    HCI_MAX_EVT_MSK_PAGE_2_CODE                = 0x59,
    HCI_DBG_META_EVT_CODE                      = 0xFF,
};

enum hci_le_evt_subcode
{
    HCI_LE_CON_CMP_EVT_SUBCODE                          = 0x01,
    HCI_LE_ADV_REPORT_EVT_SUBCODE                       = 0x02,
    HCI_LE_CON_UPDATE_CMP_EVT_SUBCODE                   = 0x03,
    HCI_LE_RD_REM_FEATS_CMP_EVT_SUBCODE                 = 0x04,
    HCI_LE_LTK_REQUEST_EVT_SUBCODE                      = 0x05,
    HCI_LE_REM_CON_PARAM_REQ_EVT_SUBCODE                = 0x06,
    HCI_LE_DATA_LEN_CHG_EVT_SUBCODE                     = 0x07,
    HCI_LE_RD_LOC_P256_PUB_KEY_CMP_EVT_SUBCODE          = 0x08,
    HCI_LE_GEN_DHKEY_CMP_EVT_SUBCODE                    = 0x09,
    HCI_LE_ENH_CON_CMP_EVT_SUBCODE                      = 0x0A,
    HCI_LE_DIR_ADV_REP_EVT_SUBCODE                      = 0x0B,
    HCI_LE_PHY_UPD_CMP_EVT_SUBCODE                      = 0x0C,
    HCI_LE_EXT_ADV_REPORT_EVT_SUBCODE                   = 0x0D,
    HCI_LE_PER_ADV_SYNC_EST_EVT_SUBCODE                 = 0x0E,
    HCI_LE_PER_ADV_REPORT_EVT_SUBCODE                   = 0x0F,
    HCI_LE_PER_ADV_SYNC_LOST_EVT_SUBCODE                = 0x10,
    HCI_LE_SCAN_TIMEOUT_EVT_SUBCODE                     = 0x11,
    HCI_LE_ADV_SET_TERMINATED_EVT_SUBCODE               = 0x12,
    HCI_LE_SCAN_REQ_RCVD_EVT_SUBCODE                    = 0x13,
    HCI_LE_CH_SEL_ALGO_EVT_SUBCODE                      = 0x14,
    HCI_LE_CONLESS_IQ_REPORT_EVT_SUBCODE                = 0x15,
    HCI_LE_CON_IQ_REPORT_EVT_SUBCODE                    = 0x16,
    HCI_LE_CTE_REQ_FAILED_EVT_SUBCODE                   = 0x17,
    HCI_LE_PER_ADV_SYNC_TRANSF_REC_EVT_SUBCODE          = 0x18,
    HCI_LE_CIS_ESTABLISHED_EVT_SUBCODE                  = 0x19,
    HCI_LE_CIS_REQUEST_EVT_SUBCODE                      = 0x1A,
    HCI_LE_CREATE_BIG_CMP_EVT_SUBCODE                   = 0x1B,
    HCI_LE_TERMINATE_BIG_CMP_EVT_SUBCODE                = 0x1C,
    HCI_LE_BIG_SYNC_ESTABLISHED_EVT_SUBCODE             = 0x1D,
    HCI_LE_BIG_SYNC_LOST_EVT_SUBCODE                    = 0x1E,
    HCI_LE_REQ_PEER_SCA_CMP_EVT_SUBCODE                 = 0x1F,
    HCI_LE_PATH_LOSS_THRESHOLD_EVT_SUBCODE              = 0x20,
    HCI_LE_TX_POWER_REPORTING_EVT_SUBCODE               = 0x21,
    HCI_LE_BIG_INFO_ADV_REPORT_EVT_SUBCODE              = 0x22,

    HCI_LE_RANGING_IND_EVT_SUBCODE                      = 0x31,
    HCI_LE_RANGING_SAMPLE_REPORT_EVT_SUBCODE            = 0x32,
    HCI_LE_RANGING_CMP_EVT_SUBCODE                      = 0x33,
};


enum TASK_API_ID
{
    // -----------------------------------------------------------------------------------
    // ---------------------- Controller Task identifer ----------------------------------
    // -----------------------------------------------------------------------------------
    // Link Layer Tasks
    TASK_ID_LLM          = 0,   // BLE Link manager
    TASK_ID_LLC          = 1,   // BLE Link controller
    TASK_ID_LLD          = 2,   // BLE Link driver
    TASK_ID_LLI          = 3,   // BLE Link ISO

    TASK_ID_DBG          = 4,   // Debug task

    // BT Controller Tasks
    TASK_ID_LM           = 5,   // BT Link manager
    TASK_ID_LC           = 6,   // BT Link controller
    TASK_ID_LB           = 7,   // BT Broadcast
    TASK_ID_LD           = 8,   // BT Link driver

    // -----------------------------------------------------------------------------------
    // --------------------- BLE HL TASK API Identifiers ---------------------------------
    // ---------------------     SHALL NOT BE CHANGED    ---------------------------------
    // -----------------------------------------------------------------------------------
    TASK_ID_SDK          = 9,    //SDK Task

    TASK_ID_L2CC         = 10,   // L2CAP Controller Task
    TASK_ID_GATTM        = 11,   // Generic Attribute Profile Manager Task
    TASK_ID_GATTC        = 12,   // Generic Attribute Profile Controller Task
    TASK_ID_GAPM         = 13,   // Generic Access Profile Manager
    TASK_ID_GAPC         = 14,   // Generic Access Profile Controller
    TASK_ID_APP          = 15,   // Application API
    TASK_ID_AHI          = 16,   // Application Host Interface
    TASK_ID_HCI          = 17,   // Host to Control Interface
    TASK_ID_DISPLAY      = 19,   // LCD/Display task
    TASK_ID_MESH         = 200,  // Mesh Task
    TASK_ID_RTLS         = 250,  // RTLS Task

    TASK_ID_INVALID      = 0xFF, // Invalid Task Identifier
};

/// HCI task Message ID
/*@TRACE*/
enum hci_msg_id
{
    HCI_MSG_ID_FIRST = TASK_FIRST_MSG(TASK_ID_HCI),

    HCI_CMD_CMP_EVENT,
    HCI_CMD_STAT_EVENT,
    HCI_EVENT,
    HCI_LE_EVENT,
    HCI_COMMAND,
    HCI_ACL_DATA,
    HCI_VS_EVENT,
    HCI_MSG_ID_LAST
};

/**
 * @brief SDK_TASK Message ID.
 */
enum sdk_msg_id
{
    /// Timeout indication to sdk
    SDK_COMMON_TIMEOUT_TIMER_0 = TASK_FIRST_MSG(TASK_ID_SDK),
    SDK_COMMON_TIMEOUT_TIMER_1,
    SDK_COMMON_TIMEOUT_TIMER_2,
    SDK_COMMON_TIMEOUT_TIMER_3,
    SDK_COMMON_TIMEOUT_TIMER_4,
    SDK_COMMON_TIMEOUT_TIMER_5,
    SDK_COMMON_TIMEOUT_TIMER_6,
    SDK_COMMON_TIMEOUT_TIMER_7,
    SDK_COMMON_TIMEOUT_TIMER_8,
    SDK_COMMON_TIMEOUT_TIMER_9, // Message number for SDK_COMMON_TIMEOUT_TIMER shall be equal to SDK_TIMER_MAX_NUM.

};

/**
 * @brief GAPC_TASK Message ID.
 */
enum gapc_msg_id
{
    GAPC_CMP_EVT                                        = MSG_ID(GAPC, 0x00),
    GAPC_CONNECTION_REQ_IND                             = MSG_ID(GAPC, 0x01),
    GAPC_CONNECTION_CFM                                 = MSG_ID(GAPC, 0x02),
    GAPC_DISCONNECT_IND                                 = MSG_ID(GAPC, 0x03),
    GAPC_DISCONNECT_CMD                                 = MSG_ID(GAPC, 0x04),
    GAPC_GET_INFO_CMD                                   = MSG_ID(GAPC, 0x05),
    GAPC_PEER_ATT_INFO_IND                              = MSG_ID(GAPC, 0x06),
    GAPC_PEER_VERSION_IND                               = MSG_ID(GAPC, 0x07),
    GAPC_PEER_FEATURES_IND                              = MSG_ID(GAPC, 0x08),
    GAPC_CON_RSSI_IND                                   = MSG_ID(GAPC, 0x09),
    GAPC_GET_DEV_INFO_REQ_IND                           = MSG_ID(GAPC, 0x0A),
    GAPC_GET_DEV_INFO_CFM                               = MSG_ID(GAPC, 0x0B),
    GAPC_SET_DEV_INFO_REQ_IND                           = MSG_ID(GAPC, 0x0C),
    GAPC_SET_DEV_INFO_CFM                               = MSG_ID(GAPC, 0x0D),
    GAPC_PARAM_UPDATE_CMD                               = MSG_ID(GAPC, 0x0E),
    GAPC_PARAM_UPDATE_REQ_IND                           = MSG_ID(GAPC, 0x0F),
    GAPC_PARAM_UPDATE_CFM                               = MSG_ID(GAPC, 0x10),
    GAPC_PARAM_UPDATED_IND                              = MSG_ID(GAPC, 0x11),
    GAPC_BOND_CMD                                       = MSG_ID(GAPC, 0x12),
    GAPC_BOND_REQ_IND                                   = MSG_ID(GAPC, 0x13),
    GAPC_BOND_CFM                                       = MSG_ID(GAPC, 0x14),
    GAPC_BOND_IND                                       = MSG_ID(GAPC, 0x15),
    GAPC_ENCRYPT_CMD                                    = MSG_ID(GAPC, 0x16),
    GAPC_ENCRYPT_REQ_IND                                = MSG_ID(GAPC, 0x17),
    GAPC_ENCRYPT_CFM                                    = MSG_ID(GAPC, 0x18),
    GAPC_ENCRYPT_IND                                    = MSG_ID(GAPC, 0x19),
    GAPC_SECURITY_CMD                                   = MSG_ID(GAPC, 0x1A),
    GAPC_SECURITY_IND                                   = MSG_ID(GAPC, 0x1B),
    GAPC_SIGN_COUNTER_IND                               = MSG_ID(GAPC, 0x1C),
    GAPC_CON_CHANNEL_MAP_IND                            = MSG_ID(GAPC, 0x1D),
    GAPC_SET_LE_PING_TO_CMD                             = MSG_ID(GAPC, 0x28),
    GAPC_LE_PING_TO_VAL_IND                             = MSG_ID(GAPC, 0x29),
    GAPC_LE_PING_TO_IND                                 = MSG_ID(GAPC, 0x2A),
    GAPC_SET_LE_PKT_SIZE_CMD                            = MSG_ID(GAPC, 0x2B),
    GAPC_LE_PKT_SIZE_IND                                = MSG_ID(GAPC, 0x2C),
    GAPC_KEY_PRESS_NOTIFICATION_CMD                     = MSG_ID(GAPC, 0x2D),
    GAPC_KEY_PRESS_NOTIFICATION_IND                     = MSG_ID(GAPC, 0x2E),
    GAPC_SET_PHY_CMD                                    = MSG_ID(GAPC, 0x2F),
    GAPC_LE_PHY_IND                                     = MSG_ID(GAPC, 0x30),
    GAPC_CHAN_SEL_ALGO_IND                              = MSG_ID(GAPC, 0x31),
    GAPC_SET_PREF_SLAVE_LATENCY_CMD                     = MSG_ID(GAPC, 0x32),
    GAPC_SET_PREF_SLAVE_EVT_DUR_CMD                     = MSG_ID(GAPC, 0x33),
    GAPC_UNKNOWN_MSG_IND                                = MSG_ID(GAPC, 0x34),
    GAPC_PER_ADV_SYNC_TRANS_CMD                         = MSG_ID(GAPC, 0x35),
    GAPC_CTE_TX_CFG_CMD                                 = MSG_ID(GAPC, 0x38),
    GAPC_CTE_RX_CFG_CMD                                 = MSG_ID(GAPC, 0x39),
    GAPC_CTE_REQ_CTRL_CMD                               = MSG_ID(GAPC, 0x3A),
    GAPC_CTE_RSP_CTRL_CMD                               = MSG_ID(GAPC, 0x3B),
    GAPC_CTE_IQ_REPORT_IND                              = MSG_ID(GAPC, 0x3C),
    GAPC_READ_LOCAL_TX_PWR_CMD                          = MSG_ID(GAPC, 0x3D),
    GAPC_READ_REMOTE_TX_PWR_CMD                         = MSG_ID(GAPC, 0x3E),
    GAPC_SET_PATH_LOSS_REPORTING_PARAM_CMD              = MSG_ID(GAPC, 0x3F),
    GAPC_SET_PATH_LOSS_REPORTING_ENABLE_CMD             = MSG_ID(GAPC, 0x40),
    GAPC_SET_TX_PWR_REPORTING_ENABLE_CMD                = MSG_ID(GAPC, 0x41),
    GAPC_READ_LOCAL_TX_PWR_IND                          = MSG_ID(GAPC, 0x42),
    GAPC_READ_REMOTE_TX_PWR_IND                         = MSG_ID(GAPC, 0x43),
    GAPC_TX_PWR_CHANGE_REPORT_IND                       = MSG_ID(GAPC, 0x44),
    GAPC_PATH_LOSS_THRESHOLD_REPORT_IND                 = MSG_ID(GAPC, 0x45),
    GAPC_RANGING_START_CMD                              = MSG_ID(GAPC, 0x46),
    GAPC_RANGING_IND                                    = MSG_ID(GAPC, 0x47),
    GAPC_RANGING_SAMPLE_REPORT_IND                      = MSG_ID(GAPC, 0x48),
    GAPC_RANGING_CMP_IND                                = MSG_ID(GAPC, 0x49),
    GAPC_SIGN_CMD                                       = MSG_ID(GAPC, 0xF0),
    GAPC_SIGN_IND                                       = MSG_ID(GAPC, 0xF1),
    GAPC_PARAM_UPDATE_TO_IND                            = MSG_ID(GAPC, 0xF2),
    GAPC_SMP_TIMEOUT_TIMER_IND                          = MSG_ID(GAPC, 0xF3),
    GAPC_SMP_REP_ATTEMPTS_TIMER_IND                     = MSG_ID(GAPC, 0xF4),
};

/**
 * @brief GAPM_TASK Message ID.
 */
enum gapm_msg_id
{
    GAPM_CMP_EVT                         = MSG_ID(GAPM, 0x00),
    GAPM_RESET_CMD                       = MSG_ID(GAPM, 0x02),
    GAPM_SET_DEV_CONFIG_CMD              = MSG_ID(GAPM, 0x04),
    GAPM_SET_CHANNEL_MAP_CMD             = MSG_ID(GAPM, 0x05),
    GAPM_GET_DEV_INFO_CMD                = MSG_ID(GAPM, 0x06),
    GAPM_DEV_VERSION_IND                 = MSG_ID(GAPM, 0x07),
    GAPM_DEV_BDADDR_IND                  = MSG_ID(GAPM, 0x08),
    GAPM_DEV_ADV_TX_POWER_IND            = MSG_ID(GAPM, 0x09),
    GAPM_DBG_MEM_INFO_IND                = MSG_ID(GAPM, 0x0A),
    GAPM_ANTENNA_INF_IND                 = MSG_ID(GAPM, 0x0B),
    GAPM_RESOLV_ADDR_CMD                 = MSG_ID(GAPM, 0x14),
    GAPM_ADDR_SOLVED_IND                 = MSG_ID(GAPM, 0x15),
    GAPM_GEN_RAND_ADDR_CMD               = MSG_ID(GAPM, 0x16),
    GAPM_USE_ENC_BLOCK_CMD               = MSG_ID(GAPM, 0x17),
    GAPM_USE_ENC_BLOCK_IND               = MSG_ID(GAPM, 0x18),
    GAPM_GEN_RAND_NB_CMD                 = MSG_ID(GAPM, 0x19),
    GAPM_GEN_RAND_NB_IND                 = MSG_ID(GAPM, 0x1A),
    GAPM_PROFILE_TASK_ADD_CMD            = MSG_ID(GAPM, 0x1B),
    GAPM_PROFILE_ADDED_IND               = MSG_ID(GAPM, 0x1C),
    GAPM_UNKNOWN_TASK_IND                = MSG_ID(GAPM, 0x1D),
    GAPM_SUGG_DFLT_DATA_LEN_IND          = MSG_ID(GAPM, 0x1E),
    GAPM_MAX_DATA_LEN_IND                = MSG_ID(GAPM, 0x1F),
    GAPM_RAL_ADDR_IND                    = MSG_ID(GAPM, 0x22),
    GAPM_SET_IRK_CMD                     = MSG_ID(GAPM, 0x23),
    GAPM_LEPSM_REGISTER_CMD              = MSG_ID(GAPM, 0x24),
    GAPM_LEPSM_UNREGISTER_CMD            = MSG_ID(GAPM, 0x25),
    GAPM_LE_TEST_MODE_CTRL_CMD           = MSG_ID(GAPM, 0x26),
    GAPM_LE_TEST_END_IND                 = MSG_ID(GAPM, 0x27),
    GAPM_LE_TEST_IQ_REPORT_IND           = MSG_ID(GAPM, 0x2D),
    GAPM_ISO_STAT_IND                    = MSG_ID(GAPM, 0x28),
    GAPM_GEN_DH_KEY_CMD                  = MSG_ID(GAPM, 0x29),
    GAPM_GEN_DH_KEY_IND                  = MSG_ID(GAPM, 0x2A),
    GAPM_GET_PUB_KEY_CMD                 = MSG_ID(GAPM, 0x2B),
    GAPM_PUB_KEY_IND                     = MSG_ID(GAPM, 0x2C),
    GAPM_GET_RAL_ADDR_CMD                = MSG_ID(GAPM, 0x90),
    GAPM_LIST_SET_CMD                    = MSG_ID(GAPM, 0x91),
    GAPM_LIST_SIZE_IND                   = MSG_ID(GAPM, 0x92),
    GAPM_ACTIVITY_CREATE_CMD             = MSG_ID(GAPM, 0xA0),
    GAPM_ACTIVITY_START_CMD              = MSG_ID(GAPM, 0xA1),
    GAPM_ACTIVITY_STOP_CMD               = MSG_ID(GAPM, 0xA2),
    GAPM_ACTIVITY_DELETE_CMD             = MSG_ID(GAPM, 0xA3),
    GAPM_ACTIVITY_CREATED_IND            = MSG_ID(GAPM, 0xA4),
    GAPM_ACTIVITY_STOPPED_IND            = MSG_ID(GAPM, 0xA5),
    GAPM_SET_ADV_DATA_CMD                = MSG_ID(GAPM, 0xA6),
    GAPM_EXT_ADV_REPORT_IND              = MSG_ID(GAPM, 0xA7),
    GAPM_SCAN_REQUEST_IND                = MSG_ID(GAPM, 0xA8),
    GAPM_SYNC_ESTABLISHED_IND            = MSG_ID(GAPM, 0xA9),
    GAPM_MAX_ADV_DATA_LEN_IND            = MSG_ID(GAPM, 0xAA),
    GAPM_NB_ADV_SETS_IND                 = MSG_ID(GAPM, 0xAB),
    GAPM_DEV_TX_PWR_IND                  = MSG_ID(GAPM, 0xAC),
    GAPM_DEV_RF_PATH_COMP_IND            = MSG_ID(GAPM, 0xAD),
    GAPM_UNKNOWN_MSG_IND                 = MSG_ID(GAPM, 0xAE),
    GAPM_PER_ADV_REPORT_CTRL_CMD         = MSG_ID(GAPM, 0xAF),
    GAPM_PER_SYNC_IQ_SAMPLING_CTRL_CMD   = MSG_ID(GAPM, 0xB0),
    GAPM_PER_ADV_IQ_REPORT_IND           = MSG_ID(GAPM, 0xB1),
    GAPM_PER_ADV_CTE_TX_CTL_CMD          = MSG_ID(GAPM, 0xB2),
    GAPM_PER_ADV_CTE_TX_PARAM_SET_CMD    = MSG_ID(GAPM, 0xB3),
    GAPM_DBG_IQGEN_CFG_CMD               = MSG_ID(GAPM, 0x50),
    GAPM_UNKNOWN_TASK_MSG                = MSG_ID(GAPM, 0xF0),
    GAPM_ADDR_RENEW_TO_IND               = MSG_ID(GAPM, 0xF1),
    GAPM_AUTO_CONN_TO_IND                = MSG_ID(GAPM, 0xF2),
    GAPM_ADDR_RENEW_CMD                  = MSG_ID(GAPM, 0xF3),
    GAPM_PUBLIC_ADDR_SET_CMD             = MSG_ID(GAPM, 0xF4),
    GAPM_SET_GDX_RANGING_PARAM_CMD       = MSG_ID(GAPM, 0xF5),
    GAPM_ADV_SCAN_MONITOR_EVT_REPORT_IND = MSG_ID(GAPM, 0xF6),
};

/**
 * @brief GATTC_TASK Message ID.
 */
enum gattc_msg_id
{
    GATTC_CMP_EVT = TASK_FIRST_MSG(TASK_ID_GATTC),
    GATTC_EXC_MTU_CMD,
    GATTC_MTU_CHANGED_IND,
    GATTC_DISC_CMD,
    GATTC_DISC_SVC_IND,
    GATTC_DISC_SVC_INCL_IND,
    GATTC_DISC_CHAR_IND,
    GATTC_DISC_CHAR_DESC_IND,
    GATTC_READ_CMD,
    GATTC_READ_IND,
    GATTC_WRITE_CMD,
    GATTC_EXECUTE_WRITE_CMD,
    GATTC_EVENT_IND,
    GATTC_EVENT_REQ_IND,
    GATTC_EVENT_CFM,
    GATTC_REG_TO_PEER_EVT_CMD,
    GATTC_SEND_EVT_CMD,
    GATTC_SEND_SVC_CHANGED_CMD,
    GATTC_SVC_CHANGED_CFG_IND,
    GATTC_READ_REQ_IND,
    GATTC_READ_CFM,
    GATTC_WRITE_REQ_IND,
    GATTC_WRITE_CFM,
    GATTC_ATT_INFO_REQ_IND,
    GATTC_ATT_INFO_CFM,
    GATTC_SDP_SVC_DISC_CMD,
    GATTC_SDP_SVC_IND,
    GATTC_TRANSACTION_TO_ERROR_IND,
    GATTC_UNKNOWN_MSG_IND,
    GATTC_CLIENT_RTX_IND,
    GATTC_SERVER_RTX_IND,
};

/**
 * @brief GATTC_TASK Message ID.
 */
enum l2cc_msg_id
{
    L2CC_CMP_EVT               = TASK_FIRST_MSG(TASK_ID_L2CC) + L2CC_NB_DEPRECATED_MSG,
    L2CC_LECB_CONNECT_CMD,
    L2CC_LECB_CONNECT_REQ_IND,
    L2CC_LECB_CONNECT_CFM,
    L2CC_LECB_CONNECT_IND,
    L2CC_LECB_DISCONNECT_CMD,
    L2CC_LECB_DISCONNECT_IND,
    L2CC_LECB_ADD_CMD,
    L2CC_LECB_ADD_IND,
    L2CC_LECB_SDU_SEND_CMD,
    L2CC_LECB_SDU_RECV_IND,
    L2CC_UNKNOWN_MSG_IND,
    L2CC_DBG_PDU_SEND_CMD,
    L2CC_DBG_PDU_RECV_IND,
    L2CC_PDU_SEND_CMD,
    L2CC_PDU_RECV_IND,
    L2CC_SIGNALING_TRANS_TO_IND,
};

/** @} */
#endif // __BLE_ID_H__
/** @} */
/** @} */

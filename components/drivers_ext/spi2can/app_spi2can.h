/**
 ****************************************************************************************
 *
 * @file    app_spi2can.h
 * @author  BSP Team
 * @brief   Header file for SPI2CAN.
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2025 GOODIX
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
 ****************************************************************************************
 */
#ifndef APP_SPI2CAN_H
#define APP_SPI2CAN_H

#include "app_io.h"

/** @addtogroup APP_SPI2CAN_ENUM Enumerations
  * @{
  */

/**
  * @brief SPI2CAN mode enumerations definition.
  */
typedef enum
{
    APP_SPI2CAN_MODE_CLASSIC_CAN = 0U,
    APP_SPI2CAN_MODE_CAN_FD  = 1U
} app_spi2can_mode_t;

/**
  * @brief SPI2CAN can rate switch mode enumerations definition.
  */
typedef enum
{
    APP_SPI2CAN_MODE_RATE_SWITCH_DISABLE = 0U,
    APP_SPI2CAN_MODE_RATE_SWITCH_ENABLE  = 1U
} app_spi2can_brs_mode_t;

/**
  * @brief SPI2CAN can retransmission mode enumerations definition.
  */
typedef enum
{
    APP_SPI2CAN_MODE_AUTO_RETRANS_DISABLE = 0U,
    APP_SPI2CAN_MODE_AUTO_RETRANS_ENABLE  = 1U
} app_spi2can_retrans_mode_t;

/**
  * @brief SPI2CAN baudrate enumerations definition
  * @note: Default configurable baud rate and sampling point:
  *        can_clk = 40M,
  *        brp  seg1  seg2
  *        {10  ,28   ,4},   Baud = 125K,  SamplePoint = 87.5%
  *        {4   ,28   ,4},   Baud = 250K,  SamplePoint = 87.5%
  *        {2   ,35   ,5},   Baud = 500K,  SamplePoint = 87.5%
  *        {1   ,30   ,10},  Baud = 1000K, SamplePoint = 75%
  *        {1   ,15   ,5},   Baud = 2000K, SamplePoint = 75%
  *        {1   ,7    ,3},   Baud = 4000K, SamplePoint = 70%
  *        {1   ,6    ,2},   Baud = 5000K, SamplePoint = 75%
  *        {1   ,4    ,1},   Baud = 8000K, SamplePoint = 80%
  */
typedef enum
{
    APP_SPI2CAN_BAUD_125K  = 0,
    APP_SPI2CAN_BAUD_250K  = 1,
    APP_SPI2CAN_BAUD_500K  = 2,
    APP_SPI2CAN_BAUD_1000K = 3,
    APP_SPI2CAN_BAUD_2000K = 4,
    APP_SPI2CAN_BAUD_4000K = 5,
    APP_SPI2CAN_BAUD_5000K = 6,
    APP_SPI2CAN_BAUD_8000K = 7
} app_spi2can_baudrate_t;

/**
  * @brief SPI2CAN error_code enumerations definition
  */
typedef enum
{
    APP_SPI2CAN_ERROR_NONE              = 0x00U,
    APP_SPI2CAN_ERROR_BUSOFF            = 0x01U
} app_spi2can_error_code_t;

/**
  * @brief SPI2CAN can standard filter type.
  */
typedef enum
{
    // Range Filter. sft_id1 holds the start address, and sft_id2 holds the end address. Any address in between will match
    APP_SPI2CAN_SID_SFT_RANGE    = 0U,
    // Dual ID filter, where both sft_id1 and sft_id2 hold IDs that can match (must match exactly)
    APP_SPI2CAN_SID_SFT_DUALID   = 1U,
    // Classic filter with sft_id1 as the ID to match, and sft_id2 as the bit mask that applies to sft_id1
    APP_SPI2CAN_SID_SFT_CLASSIC  = 2U,
    // Disabled filter. This filter will match nothing
    APP_SPI2CAN_SID_SFT_DISABLED = 3U
} app_spi2can_standard_filter_type_t;

/**
  * @brief SPI2CAN can busoff state.
  */
typedef enum
{
    APP_SPI2CAN_BUSOFF_STATE_NO_OCCURRED = 0U,   /**< No bus-off occurred. */
    APP_SPI2CAN_BUSOFF_STATE_RESTORED    = 1U,   /**< Bus-off occurred but has now recovered to normal. */
    APP_SPI2CAN_BUSOFF_STATE_NO_RESTORED = 2U    /**< Bus-off occurred or has not recovered to normal. */
} app_spi2can_busoff_state_t;

/** @} */

/** @addtogroup APP_SPI2CAN_STRUCTURES Structures
  * @{
  */
/**
  * @brief SPI2CAN IO Structures
  */
typedef struct
{
    app_io_type_t        type;       /**< Specifies the type of SPI IO. */
    app_io_mux_t         mux;        /**< Specifies the Peripheral to be connected to the selected pins. */
    uint32_t             pin;        /**< Specifies the IO pins to be configured.
                                          This parameter can be any value of @ref GR5xxx_pins. */
} app_spi2can_pin_t;

/**
  * @brief SPI2CAN can standard filter init.
  */
typedef struct
{
    // filter_index is the SID filter index in FIFO to write to (starts at 0)
    uint8_t filter_index;
    app_spi2can_standard_filter_type_t sf_type;
    uint16_t sft_id1;
    uint16_t sft_id2;
} app_spi2can_standard_filter_init_t;

/**
  * @brief SPI2CAN IO configuration Structures
  */
typedef struct
{
    app_spi2can_pin_t    cs_pin;          /**< Set the configuration of SPI2CAN CS pin. */
    app_spi2can_pin_t    clk_pin;         /**< Set the configuration of SPI2CAN CLK pin. */
    app_spi2can_pin_t    mosi_pin;        /**< Set the configuration of SPI2CAN MOSI pin. */
    app_spi2can_pin_t    miso_pin;        /**< Set the configuration of SPI2CAN MISO pin. */
    app_spi2can_pin_t    rst_pin;         /**< Set the configuration of SPI2CAN RST pin. */
    app_spi2can_pin_t    int_pin;         /**< Set the configuration of SPI2CAN INT pin. */
    app_spi2can_pin_t    wakeup_pin;      /**< Set the configuration of SPI2CAN WAKEUP pin. */
} app_spi2can_pin_cfg_t;

/**
  * @brief spi2can init information.
  */
typedef struct
{
    app_spi2can_mode_t can_mode;                    /**< SPI2CAN mode. */
    app_spi2can_brs_mode_t brs_mode;                /**< SPI2CAN rate switch mode. */
    app_spi2can_retrans_mode_t retrans_mode;        /**< SPI2CAN retransmission mode. */
    app_spi2can_baudrate_t normal_bit_baudrate;     /**< Normal bit baudrate. */
    app_spi2can_baudrate_t data_bit_baudrate;       /**< Data bit Baudrate. Only for CAN-FD. */
    app_spi2can_pin_cfg_t pin_cfg;                  /**< SPI2CAN pinmux config. */
} app_spi2can_init_t;

/**
  * @brief spi2can device structure information.
  */
typedef struct
{
    app_spi2can_mode_t can_mode;                          /**< SPI2CAN mode. */
    app_spi2can_pin_t int_pin;                            /**< Set the configuration of SPI2CAN INT pin. */
    uint32_t error_code;                                  /**< Error code is used as a bitmap. */
} app_spi2can_env_t;
/** @} */

#ifdef __cplusplus
extern "C" {
#endif

/**
 ****************************************************************************************
 * @brief  Initialize the SPI2CAN DRIVER according to the specified parameters.
 *
 * @param[in]  p_spi2can_init: Pointer to app_spi2can_init_t parameter which contains the
 *                       configuration information for the specified SPI2CAN module.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
bool app_spi2can_init(app_spi2can_init_t *p_spi2can_init);

/**
 ****************************************************************************************
 * @brief  De-initialize the SPI2CAN DRIVER.
 *
 * @return Result of de-initialization.
 ****************************************************************************************
 */
bool app_spi2can_deinit(void);

/**
 ****************************************************************************************
 * @brief Configure the standard CAN filter for the SPI2CAN.
 *
 * @param[in] p_sft_init: Pointer to a structure containing CAN filter configuration parameters.
 ****************************************************************************************
 */
void app_spi2can_set_standard_filter(app_spi2can_standard_filter_init_t *p_sft_init);

/**
 ****************************************************************************************
 * @brief  Send 0-64 byte data frame in blocking mode.
 *
 * @param[in]  can_id: CAN ID.
 * @param[in]  p_data: Pointer to data buffer.
 * @param[in]  length: Amount of data to be sent.
 *              Classic CAN Support - 0, 1, 2, 3, 4, 5, 6, 7, 8
 *              CAN-FD Support - 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
 * @param[in]  timeout: Timeout duration. Unit: ms.
 *
 * @return true: call success. false: call fail, currently in the tx state or tx timeout.
 ****************************************************************************************
 */
bool app_spi2can_transmit_polling(uint32_t can_id, uint8_t *p_data, uint8_t length, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Send 0-64 byte data frame in interrupt mode.
 * @note  If the response transmission is completed, app_spi2can_tx_cplt_cb will be triggered.
 *
 * @param[in]  can_id: CAN ID.
 * @param[in]  p_data: Pointer to data buffer.
 * @param[in]  length: Amount of data to be sent.
 *              Classic CAN Support - 0, 1, 2, 3, 4, 5, 6, 7, 8
 *              CAN-FD Support - 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
 *
 * @return true: call success. false: call fail, currently in the tx state.
 ****************************************************************************************
 */
bool app_spi2can_transmit_it(uint32_t can_id, uint8_t *p_data, uint8_t length);

/**
 ****************************************************************************************
 * @brief Retrieve the current bus-off state of the SPI2CAN.
 * @return app_spi2can_busoff_state_t
 *         - Enumeration value indicating the current bus-off state of the device.
 * @note Use this function to determine whether the CAN bus is in a fault condition or normal operation.
 ****************************************************************************************
 */
app_spi2can_busoff_state_t app_spi2can_get_busoff_state(void);

/**
 ****************************************************************************************
 * @brief  SPI2CAN RX complete cb.
 * @note   This function should not be modified. When the callback is needed,
 *         the app_spi2can_rx_cplt_cb can be implemented in the user file.
 * @param[in]  id: CAN ID.
 * @param[in]  p_data: Pointer to data buffer.
 * @param[in]  size: Amount of data to be received.
 *              Classic CAN Support - 0, 1, 2, 3, 4, 5, 6, 7, 8
 *              CAN-FD Support - 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
 ****************************************************************************************
 */
void app_spi2can_rx_cplt_cb(uint32_t id, uint8_t * p_data, uint8_t size);

/**
 ****************************************************************************************
 * @brief  SPI2CAN TX response cb.
 * @note   This function should not be modified. When the callback is needed,
 *         the app_spi2can_tx_cplt_cb can be implemented in the user file.
 ****************************************************************************************
 */
void app_spi2can_tx_cplt_cb(void);

/**
 ****************************************************************************************
 * @brief  SPI2CAN restore from busoff error cb.
 * @note   This function should not be modified. When the callback is needed,
 *         the app_spi2can_busoff_restore_cb can be implemented in the user file.
 ****************************************************************************************
 */
void app_spi2can_busoff_restore_cb(void);

/**
 ****************************************************************************************
 * @brief  SPI2CAN error cb.
 * @note   This function should not be modified. When the callback is needed,
 *         the app_spi2can_error_cb can be implemented in the user file.
 * @param[in]  error_code: Error code
 ****************************************************************************************
 */
void app_spi2can_error_cb(uint32_t error_code);

#ifdef __cplusplus
}
#endif

#endif /* APP_SPI2CAN_H */

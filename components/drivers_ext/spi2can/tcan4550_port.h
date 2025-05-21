/**
 ****************************************************************************************
 *
 * @file    tcan4550_port.h
 * @author  BSP Team
 * @brief   PORT interface based on the TCAN4550 low-level driver encapsulation.
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

#ifndef TCAN4550_PORT_H
#define TCAN4550_PORT_H

#include <stdint.h>
#include <stdbool.h>

typedef void (*tcan4550_rx_cplt_callback_t)(uint32_t id, uint8_t *p_data, uint8_t size);
typedef void (*tcan4550_tx_cplt_callback_t)(void);
typedef void (*tcan4550_error_callback_t)(uint32_t error_code);
typedef void (*tcan4550_busoff_restore_callback_t)(void);

/**
  * @brief TCAN4550 can mode.
  */
typedef enum
{
    TCAN_MODE_CLASSIC_CAN = 0U,
    TCAN_MODE_CAN_FD      = 1U
} tcan4550_mode_t;

/**
  * @brief TCAN4550 can rate switch mode.
  */
typedef enum
{
    TCAN_MODE_RATE_SWITCH_DISABLE = 0U,
    TCAN_MODE_RATE_SWITCH_ENABLE  = 1U
} tcan4550_brs_mode_t;

/**
  * @brief TCAN4550 can retransmission mode.
  */
typedef enum
{
    TCAN_MODE_AUTO_RETRANS_DISABLE = 0U,
    TCAN_MODE_AUTO_RETRANS_ENABLE  = 1U
} tcan4550_retrans_mode_t;

/**
  * @brief TCAN4550 can busoff state.
  */
typedef enum
{
    TCAN_BUSOFF_STATE_NO_OCCURRED = 0U,   /**< No bus-off occurred. */
    TCAN_BUSOFF_STATE_RESTORED    = 1U,   /**< Bus-off occurred but has now recovered to normal. */
    TCAN_BUSOFF_STATE_NO_RESTORED = 2U    /**< Bus-off occurred or has not recovered to normal. */
} tcan4550_busoff_state_t;

/**
  * @brief TCAN4550 debug log level.
  */
typedef enum
{
    TCAN_DEBUG_CLOSE        = 0U,
    TCAN_DEBUG_LOG_LV_3     = 1U,
    TCAN_DEBUG_LOG_LV_2     = 2U,
    TCAN_DEBUG_LOG_LV_1     = 3U
} tcan4550_debug_log_lv_t;

/**
  * @brief TCAN4550 error_code enumerations definition
  */
typedef enum
{
    TCAN_SPI2CAN_ERROR_NONE      = 0x00U,
    TCAN_SPI2CAN_ERROR_BUSOFF    = 0x01U
} tcan4550_error_code_t;

/**
  * @brief TCAN4550 can standard filter type.
  */
typedef enum
{
    // Range Filter. sft_id1 holds the start address, and sft_id2 holds the end address. Any address in between will match
    TCAN_SID_SFT_RANGE    = 0U,
    // Dual ID filter, where both sft_id1 and sft_id2 hold IDs that can match (must match exactly)
    TCAN_SID_SFT_DUALID   = 1U,
    // Classic filter with sft_id1 as the ID to match, and sft_id2 as the bit mask that applies to sft_id1
    TCAN_SID_SFT_CLASSIC  = 2U,
    // Disabled filter. This filter will match nothing
    TCAN_SID_SFT_DISABLED = 3U
} tcan4550_standard_filter_type_t;

/**
  * @brief TCAN4550 can standard filter init.
  */
typedef struct
{
    // filter_index is the SID filter index in FIFO to write to (starts at 0)
    uint8_t filter_index;
    tcan4550_standard_filter_type_t sf_type;
    uint16_t sft_id1;
    uint16_t sft_id2;
} tcan4550_standard_filter_init_t;

/**
  * @brief TCAN4550 normal timing sample.
  */
typedef struct
{
    // The prescaler value from the MCAN system clock. Value interpreted as 1:x
    // Valid range is: 1 to 512
    uint16_t brp : 10;
    // The total number of time quanta prior to sample point
    //  Valid values are: 2 to 257
    uint16_t seg1 : 9;
    // The total number of time quanta after the sample point
    // Valid values are: 2 to 128
    uint8_t seg2 : 8;
} tcan4550_normal_timing_sample_t;

/**
  * @brief TCAN4550 normal timing sample.
  */
typedef struct
{
    // Prescaler value, interpreted as 1:x
    // Valid range is: 1 to 32
    uint8_t brp : 6;
    // Number of time quanta before sample point
    // Valid values are: 2 to 33
    uint8_t seg1 : 6;
    // Number of time quanta after sample point
    // Valid values are: 1 to 16
    uint8_t seg2 : 5;
} tcan4550_data_timing_sample_t;

typedef struct
{
    tcan4550_rx_cplt_callback_t         rx_cplt_callback;
    tcan4550_tx_cplt_callback_t         tx_cplt_callback;
    tcan4550_error_callback_t           xfer_err_callback;
    tcan4550_busoff_restore_callback_t  busoff_restore_calback;        // Check and Restore from Bus Off
} tcan4550_callback_t;

/**
  * @brief TCAN4550 init information.
  */
typedef struct
{
    tcan4550_mode_t can_mode;
    tcan4550_brs_mode_t brs_mode;
    tcan4550_retrans_mode_t retrans_mode;
    tcan4550_normal_timing_sample_t normal_timing;
    tcan4550_data_timing_sample_t data_timing;
    tcan4550_callback_t can_cb;
} tcan4550_init_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 ****************************************************************************************
 * @brief Initialize the TCAN4550 device with the specified configuration.
 * @param[in] p_init: Pointer to a structure containing initialization parameters.
 * @return bool
 *         - true: Initialization successful.
 *         - false: Initialization failed.
 * @note Ensure that the p_init structure is properly configured before calling this function.
 ****************************************************************************************
 */
bool tcan4550_init(tcan4550_init_t *p_init);

/**
 ****************************************************************************************
 * @brief Clear all interrupts flag.
 * @note This API is called after the SPI and IO initialization,
 *       to clear interference interrupts generated during the SPI2CAN initialization process.
 ****************************************************************************************
 */
void tcan4550_clear_all_interrupts(void);

/**
 ****************************************************************************************
 * @brief Configure the standard CAN filter for the TCAN4550 device.
 * @param[in] p_sft_init: Pointer to a structure containing CAN filter configuration parameters.
 * @note The CAN filter will be applied according to the configuration provided in p_sft_init.
 ****************************************************************************************
 */
void tcan4550_set_standard_filter(tcan4550_standard_filter_init_t *p_sft_init);

/**
 ****************************************************************************************
 * @brief Transmit a CAN message using interrupt-based transfer.
 * @param[in] can_id: CAN identifier of the message.
 * @param[in] p_data: Pointer to the data buffer to be transmitted.
 * @param[in] length: Length of the data buffer (up to 8 bytes).
 * @return bool
 *         - true: Message transmission request successful.
 *         - false: Message transmission request failed.
 * @note The function relies on hardware interrupts for transmission completion. Ensure that the interrupt handler is properly configured.
 ****************************************************************************************
 */
bool tcan4550_transmit_it(uint32_t can_id, uint8_t *p_data, uint8_t length);

/**
 ****************************************************************************************
 * @brief Transmit a CAN message using polling-based transfer.
 * @param[in] can_id: CAN identifier of the message.
 * @param[in] p_data: Pointer to the data buffer to be transmitted.
 * @param[in] length: Length of the data buffer (up to 8 bytes).
 * @param[in] timeout: Maximum time (in appropriate units) to wait for transmission completion. Unit: ms.
 * @return bool
 *         - true: Message transmitted successfully within the specified timeout.
 *         - false: Message transmission failed or timed out.
 * @note This function blocks until the message is transmitted or the timeout is reached.
 ****************************************************************************************
 */
bool tcan4550_transmit_polling(uint32_t can_id, uint8_t *p_data, uint8_t length, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief Process the wake-up interrupt for the TCAN4550 device.
 * @note This function handles wake-up events and should be called from the IO wake-up interrupt handler.
 ****************************************************************************************
 */
void tcan4550_run_wakeup_irq_process(void);

/**
 ****************************************************************************************
 * @brief Retrieve the current bus-off state of the TCAN4550 device.
 * @return tcan4550_busoff_state_t
 *         - Enumeration value indicating the current bus-off state of the device.
 * @note This function handles INT events and should be called from the IO int interrupt handler.
 ****************************************************************************************
 */
void tcan4550_run_int_irq_process(void);

/**
 ****************************************************************************************
 * @brief Retrieve the current bus-off state of the TCAN4550 device.
 * @return tcan4550_busoff_state_t
 *         - Enumeration value indicating the current bus-off state of the device.
 * @note Use this function to determine whether the CAN bus is in a fault condition or normal operation.
 ****************************************************************************************
 */
tcan4550_busoff_state_t tcan4550_get_busoff_state(void);

/**
 ****************************************************************************************
 * @brief Set the debug log level for the TCAN4550 device.
 * @param[in] debug_log_lv: Debug log level configuration.
 * @note Adjusting the debug log level controls the amount of debug information that is output by the device.
 ****************************************************************************************
 */
void tcan4550_set_debug_log_level(tcan4550_debug_log_lv_t debug_log_lv);

#ifdef __cplusplus
}
#endif

#endif /* TCAN4550_PORT_H */

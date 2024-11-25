#ifndef __APP_UART2LIN_H__
#define __APP_UART2LIN_H__

#include "app_uart.h"

// Min rsp timeout = (1 * 1000000 / s_uart_params.init.baud_rate * 10 * (APP_UART2LIN_DATA_LEN + APP_UART2LIN_CHECKSUM_LEN)) * 1.4
// baud_rate = 20000 bps
#define UART2LIN_MIN_RX_RSP_TIMEOUT             (7)     // Unit: ms

// UART2LIN packet len
#define APP_UART2LIN_HEAD_LEN                   (3)
#define APP_UART2LIN_DATA_LEN                   (8)
#define APP_UART2LIN_CHECKSUM_LEN               (1)
#define APP_UART2LIN_FRAME_LEN                  (APP_UART2LIN_HEAD_LEN + APP_UART2LIN_DATA_LEN + APP_UART2LIN_CHECKSUM_LEN)

#define APP_UART2LIN_BREAK_FRAME                (0x0)
#define APP_UART2LIN_SYNC_FRAME                 (0x55)
#define APP_UART2LIN_MAX_PID                    (0x3F)

// UART2LIN index
#define APP_UART2LIN_BREAK_FIELD_INDEX          (0)
#define APP_UART2LIN_SYNC_FIELD_INDEX           (1)
#define APP_UART2LIN_PID_INDEX                  (2)

typedef enum
{
    APP_UART2LIN_MODE_MASTER = 0U,
    APP_UART2LIN_MODE_SLAVE  = 1U,
} app_uart2lin_mode_t;

typedef enum
{
    APP_UART2LIN_SOFTWARE_TIMER = 0U,
    APP_UART2LIN_HARDWARE_TIMER = 1U,   // The hardware timer defaults to using dual_timer_1.
} app_uart2lin_timer_type_t;

typedef enum
{
    APP_UART2LIN_INIT_NO_READY = 0U,
    APP_UART2LIN_INIT_READY    = 1U,
} app_uart2lin_init_state_t;

typedef enum
{
    APP_UART2LIN_RX_HEAD     = 0U,
    APP_UART2LIN_RX_RESPONSE = 1U,
    APP_UART2LIN_RX_IDLE     = 0xFFU,
} app_uart2lin_rx_type_t;

typedef enum
{
    APP_UART2LIN_TX_HEAD     = 0U,
    APP_UART2LIN_TX_RESPONSE = 1U,
    APP_UART2LIN_TX_IDLE     = 0xFFU,
} app_uart2lin_tx_type_t;

typedef enum
{
    APP_UART2LIN_CHECK_ENHANCE_UNSUPPORT = 0U,    // LIN Version <= LIN 1.3.
    APP_UART2LIN_CHECK_ENHANCE_SUPPORT   = 1U,    // LIN Version >= LIN 2.0.
} app_uart2lin_check_t;

typedef enum
{
    APP_UART2LIN_BAUD_1200  = 1200U,
    APP_UART2LIN_BAUD_2400  = 2400U,
    APP_UART2LIN_BAUD_4800  = 4800U,
    APP_UART2LIN_BAUD_9600  = 9600U,
    APP_UART2LIN_BAUD_19200 = 19200U,
    APP_UART2LIN_BAUD_20000 = 20000U,
} app_uart2lin_baudrate_t;

typedef enum
{
    APP_UART2LIN_BRK_LEN_13 = 13U,
    APP_UART2LIN_BRK_LEN_14 = 14U,
    APP_UART2LIN_BRK_LEN_15 = 15U,
    APP_UART2LIN_BRK_LEN_16 = 16U,
} app_uart2lin_brk_len_t;

typedef enum
{
    APP_UART2LIN_ERROR_NONE              = 0x00U,
    APP_UART2LIN_ERROR_BUFFER_OVER       = 0x01U,
    APP_UART2LIN_ERROR_CHECKSUM          = 0x02U,
    APP_UART2LIN_ERROR_FRAME             = 0x04U,
    APP_UART2LIN_ERROR_IDENTIFIER_PARITY = 0x08U,
    APP_UART2LIN_ERROR_TIMEOUT           = 0x10U,
    APP_UART2LIN_ERROR_INCOMPLETE_DATA   = 0x20U,
    APP_UART2LIN_ERROR_BIT_ERROR         = 0x40U,
} app_uart2lin_error_code_t;

/**
  * @brief uart2lin init information.
  */
typedef struct
{
    app_uart_id_t uart_id;                 /**< Specified UART module ID. */
    app_uart2lin_mode_t lin_mode;          /**< Master or slave mode. */
    // Software timers introduce more software processing time, resulting in a longer response time for replying LIN responses.
    app_uart2lin_timer_type_t timer_type;  /**< Timer type. */
    app_uart2lin_check_t checksum_type;    /**< Checksum type. */
    app_uart2lin_baudrate_t baudrate;      /**< Baudrate. */
    app_uart2lin_brk_len_t brk_len;        /**< Broken filed length. */
    app_uart_pin_cfg_t pin_cfg;            /**< UART2LIN pinmux config. */
    dma_regs_t *rx_dma_instance;           /**< Specifies the UART RX DMA instance.*/
    dma_channel_t rx_dma_channel;          /**< Specifies the dma channel of UART RX. */
    // The rx_response_timeout needs to be related to the actual communication frequency of the LIN Master device.
    // For example, if the LIN Master sends data every 50ms, then the rx_response_timeout needs to be less than 50ms.
    // UART2LIN_MIN_RX_RSP_TIMEOUT is data (up to 8 bytes)+checksum (1 byte)+the maximum tolerable time for the head and response (40% of data + checksum).
    uint8_t rx_response_timeout;           /**< UART2LIN receive response timeout.  */
} app_uart2lin_init_t;

/**
  * @brief uart2lin device structure information.
  */
typedef struct
{
    bool is_timeout;                                                      /**< Whether the response reception timed out. */
    app_uart2lin_init_state_t init_state;                                 /**< UART2LIN init state. */
    app_uart2lin_mode_t lin_mode;                                         /**< UART2LIN mode. */
    app_uart2lin_timer_type_t timer_type;                                 /**< Timer type. */
    app_uart2lin_check_t checksum_type;                                   /**< Checksum type. */
    app_uart2lin_brk_len_t brk_len;                                       /**< UART2LIN broken filed length. */
    app_uart2lin_tx_type_t tx_type;                                       /**< Tx type. */
    app_uart2lin_rx_type_t rx_type;                                       /**< Rx type. */
    uint8_t rx_response_timeout;                                          /**< UART2LIN receive response timeout.  */
    uint8_t tx_len;                                                       /**< Tx length. */
    uint8_t tx_buffer[APP_UART2LIN_DATA_LEN + APP_UART2LIN_CHECKSUM_LEN]; /**< Tx buffer. */
    uint8_t rx_pid;                                                       /**< The PID recorded in the received head. */
    uint8_t rx_len;                                                       /**< The LEN recorded in the received response. */
    uint8_t *p_rx_buffer;                                                 /**< The data recorded in the received response. */
    uint8_t error_code;                                                   /**< Error code is used as a bitmap. */
} app_uart2lin_env_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 ****************************************************************************************
 * @brief  Initialize the UART2LIN DRIVER according to the specified parameters.
 *
 * @param[in]  p_uart2lin_init: Pointer to app_uart2lin_init_t parameter which contains the
 *                       configuration information for the specified UART2LIN module.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
bool app_uart2lin_init(app_uart2lin_init_t *p_uart2lin_init);

/**
 ****************************************************************************************
 * @brief  De-initialize the UART2LIN DRIVER.
 *
 * @return Result of de-initialization.
 ****************************************************************************************
 */
bool app_uart2lin_deinit(void);

/**
 ****************************************************************************************
 * @brief  Start receiving the LIN head.
 * @note   If the reception is completed successfully, it will trigger app_uart2lin_rx_head_cb.
 *         And the received header PID can be read from p_uart2lin->rx_pid.
 *         Before sending the head in Master, all nodes on the bus need to call app_uart2lin_rx_head().
 *
 * @return Ture: call success. Others: call fail, currently in the rx state.
 ****************************************************************************************
 */
bool app_uart2lin_rx_head(void);

/**
 ****************************************************************************************
 * @brief  Force to start receiving the LIN head.
 * @note   1. If the reception is completed successfully, it will trigger app_uart2lin_rx_head_cb.
 *            And the received header PID can be read from p_uart2lin->rx_pid.
 *            Before sending the head in Master, all nodes on the bus need to call app_uart2lin_rx_head().
 *         2. The interface will forcefully terminate all current operations of UART2LIN and restart receiving LIN head.
 *            For example, in application scenarios: if the LIN Slave receives a LIN Head but does not need to receive or send a Response,
 *            it can call app_uart2lin_force_rx_head to avoid waiting for the timeout interrupt to be triggered.
 *
 ****************************************************************************************
 */
void app_uart2lin_force_rx_head(void);

/**
 ****************************************************************************************
 * @brief  Receive the LIN response.
 * @note   If the reception is completed, app_uart2lin_rx_response_cb will be triggered.
 *         The received data can be obtained through the pointer passed in as p_data, and the length of the received data is p_uart2lin->rx_len.
 *         app_uart2lin_rx_response is usually called in app_uart2lin_rx_head_cb.
 *
 * @param[in]  p_data: Pointer to receive buffer.
 * @param[in]  size: Amount of data to be received (0 < size <= APP_UART2LIN_DATA_LEN).
 *
 * @return Ture: call success. Others: call fail, currently in the rx state.
 ****************************************************************************************
 */
bool app_uart2lin_rx_response(uint8_t *p_data, uint8_t size);

/**
 ****************************************************************************************
 * @brief  Transmit the LIN head.
 * @note   Only LIN master can make the call.
 *
 * @param[in]  pid: LIN PID (Used to indicate the function of the frame, pid <= APP_UART2LIN_MAX_PID).
 *
 * @return true: call success. false: call fail, currently in the tx state.
 ****************************************************************************************
 */
bool app_uart2lin_tx_head(uint8_t pid);

/**
 ****************************************************************************************
 * @brief  Transmit the LIN response.
 * @note  If the response transmission is completed, app_uart2lin_tx_response_cb will be triggered.
 *
 * @param[in]  p_data: Pointer to data buffer.
 * @param[in]  size: Amount of data to be sent (0 < size <= APP_UART2LIN_DATA_LEN).
 *
 * @return true: call success. false: call fail, currently in the tx state.
 ****************************************************************************************
 */
bool app_uart2lin_tx_response(uint8_t *p_data, uint8_t size);

/**
 ****************************************************************************************
 * @brief  Abort uart2lin receive.
 * @note   When a bus anomaly causes UART2LIN unable to exit the RX state, this interface can be called.
 *
 ****************************************************************************************
 */
void app_uart2lin_abort_rx(void);

/**
 ****************************************************************************************
 * @brief  Abort uart2lin receive.
 * @note   When a bus anomaly causes UART2LIN unable to exit the TX state, this interface can be called.
 *
 ****************************************************************************************
 */
void app_uart2lin_abort_tx(void);

/**
 ****************************************************************************************
 * @brief  UART2LIN rx head cb.
 * @note   This function should not be modified. When the callback is needed,
 *         the app_uart2lin_rx_head_cb can be implemented in the user file.
           In the app_uart2lin_rx_head_cb function,
           it is necessary to call app_uart2lin_rx_response to receive the response to avoid false timeouts.
 * @param[in]  p_uart2lin: Pointer to a p_uart2lin handle which contains the configuration
 *                  information for the specified uart2lin module.
 ****************************************************************************************
 */
void app_uart2lin_rx_head_cb(app_uart2lin_env_t *p_uart2lin);

/**
 ****************************************************************************************
 * @brief  UART2LIN rx response cb.
 * @note   This function should not be modified. When the callback is needed,
 *         the app_uart2lin_rx_response_cb can be implemented in the user file.
           In the app_uart2lin_rx_response_cb function,
           it is necessary to judge whether the data needs to be discarded based on p_uart2lin->pid.
           After the data parsing is completed,
           it is necessary to call app_uart2lin_rx_head again to enable reception.
 * @param[in]  p_uart2lin: Pointer to a p_uart2lin handle which contains the configuration
 *                  information for the specified uart2lin module.
 ****************************************************************************************
 */
void app_uart2lin_rx_response_cb(app_uart2lin_env_t *p_uart2lin);

/**
 ****************************************************************************************
 * @brief  UART2LIN tx response cb.
 * @note   This function should not be modified. When the callback is needed,
 *         the app_uart2lin_tx_response_cb can be implemented in the user file.
 * @param[in]  p_uart2lin: Pointer to a p_uart2lin handle which contains the configuration
 *                  information for the specified uart2lin module.
 ****************************************************************************************
 */
void app_uart2lin_tx_response_cb(app_uart2lin_env_t *p_uart2lin);

/**
 ****************************************************************************************
 * @brief  UART2LIN error cb.
 * @note   This function should not be modified. When the callback is needed,
 *         the app_uart2lin_error_cb can be implemented in the user file.
 * @param[in]  p_uart2lin: Pointer to a p_uart2lin handle which contains the configuration
 *                  information for the specified uart2lin module.
 ****************************************************************************************
 */
void app_uart2lin_error_cb(app_uart2lin_env_t *p_uart2lin);

#ifdef __cplusplus
}
#endif

#endif /* __APP_UART2LIN_H__ */

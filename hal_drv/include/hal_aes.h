/**
 ****************************************************************************************
 *
 * @file    hal_aes.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of AES HAL library.
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
 ****************************************************************************************
 */

/** @addtogroup PERIPHERAL Peripheral Driver
  * @{
  */

/** @addtogroup HAL_DRIVER HAL Driver
  * @{
  */

/** @defgroup HAL_AES AES
  * @brief AES HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR5XX_HAL_AES_H__
#define __GR5XX_HAL_AES_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "ll_aes.h"
#include "ll_misc.h"
#include "hal_def.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_AES_ENUMERATIONS Enumerations
  * @{
  */


/** @defgroup HAL_AES_state HAL AES State
  * @{
  */

/**
  * @brief HAL AES State Enumerations definition
  */
typedef enum
{
    HAL_AES_STATE_RESET = 0x00,    /**< Peripheral not initialized               */
    HAL_AES_STATE_READY = 0x01,    /**< Peripheral initialized and ready for use */
    HAL_AES_STATE_BUSY  = 0x02,    /**< Peripheral in indirect mode and busy     */
} hal_aes_state_t;

/** @} */

/** @} */

/** @addtogroup HAL_AES_STRUCTURES Structures
  * @{
  */

/** @defgroup AES_Configuration AES Configuration
  * @{
  */

/**
  * @brief AES Init Structure definition
  */
typedef struct _aes_init
{
    uint32_t  key_mode;        /**< AES key mode.
                                    This parameter can be a value of @ref AES_KEY_Mode     */
    uint32_t  key_type;        /**< AES key type.
                                    This parameter can be a value of @ref AES_KEY_TYPE     */
    uint32_t  key_size;        /**< 128, 192 or 256-bits key length.
                                    This parameter can be a value of @ref AES_KEYSIZE      */
    uint32_t *p_key;           /**< Encryption/Decryption Key                              */
    uint32_t  key_addr;        /**< Encryption/Decryption Key address                      */
    uint32_t  key_mask;        /**< Keyport Mask                                           */
    uint32_t  key_slot;        /**< Keyram Key slot                                        */
    uint32_t *p_init_vector;   /**< Initialization Vector used for CBC modes               */
    uint32_t  dpa_mode;        /**< DPA Mode.
                                    This parameter can be a value of @ref AES_DPA_MODE     */
    uint32_t *p_seed;          /**< Random seeds                                           */
    uint32_t  endian_mode;     /**< AES Endian Mode.
                                    This parameter can be a value of @ref AES_ENDIAN_MODE  */
} aes_init_t;

/** @} */

/** @defgroup AES_handle AES Handle
  * @{
  */

/**
  * @brief AES handle Structure definition
  */
typedef struct _aes_handle
{
    aes_regs_t          *p_instance;            /**< AES registers base address                                          */
    aes_init_t           init;                  /**< AES init parameters                                                 */
    uint32_t             operation_mode;        /**< AES operating mode                                                  */
    uint32_t             chaining_mode;         /**< AES chaining mode                                                   */
    uint32_t            *p_cryp_input_buffer;   /**< Pointer to CRYP processing (encryption or decryption) input buffer  */
    uint32_t            *p_cryp_output_buffer;  /**< Pointer to CRYP processing (encryption or decryption) output buffer */
    uint32_t             block_size;            /**< Data size in blocks (16 bytes per block)                            */
    uint32_t             keyram_offset;         /**< Keyram offset                                                       */
    __IO hal_aes_state_t state;                 /**< AES operation state                                                 */
    __IO uint32_t        error_code;            /**< AES Error code                                                      */
} aes_handle_t;

/** @} */

/** @} */

/** @addtogroup HAL_AES_CALLBACK_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup AES_Callback AES Callback
  * @{
  */

/**
  * @brief HAL AES Callback function definition
  */
typedef struct _hal_aes_callback
{
    void (*aes_msp_init)(aes_handle_t *p_aes);                  /**< AES init MSP callback                  */
    void (*aes_msp_deinit)(aes_handle_t *p_aes);                /**< AES de-init MSP callback               */
    void (*aes_error_callback)(aes_handle_t *p_aes);            /**< AES error callback                     */
    void (*aes_done_callback)(aes_handle_t *p_aes);             /**< AES encrypt or decrypt done callback   */
    void (*aes_abort_cplt_callback)(aes_handle_t *p_aes);       /**< AES abort complete callback            */
} hal_aes_callback_t;

/** @} */

/** @} */

/**
  * @defgroup  HAL_AES_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup AES_Exported_Constants AES Exported Constants
  * @{
  */

/** @defgroup AES_Error_Code AES Error Code
  * @{
  */
#define HAL_AES_ERROR_NONE             ((uint32_t)0x00000000)       /**< No error                 */
#define HAL_AES_ERROR_BUSY             ((uint32_t)0x00000001)       /**< Busy error               */
#define HAL_AES_ERROR_TIMEOUT          ((uint32_t)0x00000002)       /**< Timeout error            */
#define HAL_AES_ERROR_TRANSFER         ((uint32_t)0x00000004)       /**< Transfer error           */
#define HAL_AES_ERROR_INVALID_PARAM    ((uint32_t)0x00000008)       /**< Invalid parameters error */
/** @} */

/** @defgroup AES_OPERATION_MODE AES Operation Mode
  * @{
  */
#define AES_OPERATION_MODE_ENCRYPT      (1U)                         /**< Encrypt operation mode */
#define AES_OPERATION_MODE_DECRYPT      (0U)                         /**< Decrypt operation mode */
/** @} */

/** @defgroup AES_CHAININGMODE AES Chaining Mode
  * @{
  */
#define AES_CHAININGMODE_ECB           LL_AES_OPERATION_MODE_ECB    /**< ECB chaining mode */
#define AES_CHAININGMODE_CBC           LL_AES_OPERATION_MODE_CBC    /**< CBC chaining mode */
/** @} */

/** @defgroup AES_ENDIAN_MODE AES Endian Mode
  * @{
  */
#define AES_ENDIAN_MODE_BIG             (1U)                         /**< Big endian mode   */
#define AES_ENDIAN_MODE_SMALL           (0U)                         /**< Small endian mode */
/** @} */

/** @defgroup AES_KEY_TYPE AES Key Type
  * @{
  */
#define AES_KEY_TYPE_MCU               LL_AES_KEYTYPE_MCU           /**< Fetch Key from MCU    */
#define AES_KEY_TYPE_AHB               LL_AES_KEYTYPE_AHB           /**< Fetch Key from AHB    */
#define AES_KEY_TYPE_KEYRAM            LL_AES_KEYTYPE_KRAM          /**< Fetch Key from KEYRAM */
/** @} */

/** @defgroup AES_KEY_Mode AES Key Mode
  * @{
  */
#define AES_KEY_MODE_NORMAL            LL_AES_KEYMODE_NORMAL       /**< Normal Key mode   */
#define AES_KEY_MODE_KEYWRAP           LL_AES_KEYMODE_KEYWRAP      /**< Key Wrapping mode */
/** @} */

/** @defgroup AES_KEYSIZE AES Key Size
  * @{
  */
#define AES_KEYSIZE_128BITS            LL_AES_KEY_SIZE_128          /**< 128 bits */
#define AES_KEYSIZE_192BITS            LL_AES_KEY_SIZE_192          /**< 192 bits */
#define AES_KEYSIZE_256BITS            LL_AES_KEY_SIZE_256          /**< 256 bits */
/** @} */

/** @defgroup AES_DPA_MODE AES DPA Mode
  * @{
  */
#define AES_DPA_MODE_ENABLE            (1U)                         /**< Enable DPA Mode */
#define AES_DPA_MODE_DISABLE           (0U)                         /**< Disable DPA Mode */
/** @} */

/** @defgroup AES_Block_Size AES Block Size
  * @{
  */
#define AES_BLOCK_MAX                  (2048U)                      /**< Block max size       */
#define AES_BLOCKSIZE_BITS             (128U)                       /**< Block size in bits   */
#define AES_BLOCKSIZE_BYTES            (AES_BLOCKSIZE_BITS >> 3)    /**< Block size in bytes  */
#define AES_BLOCKSIZE_WORDS            (AES_BLOCKSIZE_BYTES >> 2)   /**< Block size in words  */
/** @} */

/** @defgroup AES_Flags_definition AES Flags Definition
  * @{
  */
#define AES_FLAG_DATAREADY             LL_AES_FLAG_DATAREADY        /**< Data ready flag          */
#define AES_FLAG_DMA_DONE              LL_AES_FLAG_DMA_DONE         /**< DMA transfer done flag   */
#define AES_FLAG_DMA_ERR               LL_AES_FLAG_DMA_ERR          /**< DMA transfer error flag  */
#define AES_FLAG_KEY_VALID             LL_AES_FLAG_KEY_VALID        /**< Key valid flag           */
/** @} */

/** @defgroup AES_Interrupt_definition AES Interrupt definition
  * @{
  */
#define AES_IT_DONE                    ((uint32_t)0x00000001)       /**< AES Encrypted or Decrypted Data Done Interrupt source */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup AES_Exported_Macros AES Exported Macros
  * @{
  */

/** @brief  Check whether the specified AES interrupt flag is set or not.
  * @param  __HANDLE__ Specifies the AES Handle.
  * @param  __FLAG__ Specifies the interrupt flag to check.
  *         This parameter can be the following value:
  *            @arg @ref AES_IT_DONE Encrypted or Decrypted Data Done Interrupt
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_AES_GET_FLAG_IT(__HANDLE__, __FLAG__)            (READ_BITS((__HANDLE__)->p_instance->INT, (__FLAG__)) == (__FLAG__))

/** @brief  Clear the specified AES interrupt flag.
  * @param  __HANDLE__ Specifies the AES interrupt Handle.
  * @param  __FLAG__ Specifies the flag to clear.
  *         This parameter can be the following value:
  *            @arg @ref AES_IT_DONE Encrypted or Decrypted Data Done Interrupt
  * @retval None
  */
#define __HAL_AES_CLEAR_FLAG_IT(__HANDLE__, __FLAG__)          SET_BITS((__HANDLE__)->p_instance->INT, (__FLAG__))

/** @brief  Check whether the specified AES flag is set or not.
  * @param  __HANDLE__ Specifies the AES Handle.
  * @param  __FLAG__ Specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg @ref AES_FLAG_DATAREADY Data ready flag
  *            @arg @ref AES_FLAG_DMA_DONE DMA transfer done flag
  *            @arg @ref AES_FLAG_DMA_ERR DMA transfer error flag
  *            @arg @ref AES_FLAG_KEY_VALID Key valid flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_AES_GET_FLAG(__HANDLE__, __FLAG__)               ((READ_BITS((__HANDLE__)->p_instance->STAT, (__FLAG__)) != 0U) ? SET : RESET)

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_AES_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @addtogroup AES_Exported_Functions_Group1 Initialization and de-initialization functions
  * @brief    Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          de-initialize the AESx peripheral:

      (+) User must implement hal_aes_msp_init() function in which he configures
          all related peripherals resources (GPIO, DMA, IT and NVIC ).

      (+) Call the function hal_aes_init() to configure the selected device with
          the selected configuration:
        (++) Key Type
        (++) Key Size
        (++) Key
        (++) Key Address
        (++) Key Mask
        (++) init_vector
        (++) DPA Mode
        (++) Seed
        (++) Endian Mode

      (+) Call the function hal_aes_deinit() to restore the default configuration
          of the selected AESx peripheral.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the AES according to the specified parameters
 *         in the aes_init_t and initialize the associated handle.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration
 *                    information for the specified AES module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_init(aes_handle_t *p_aes);

/**
 ****************************************************************************************
 * @brief  De-initialize the AES peripheral.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration
 *                    information for the specified AES module.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_deinit(aes_handle_t *p_aes);

/**
 ****************************************************************************************
 * @brief  Initialize the AES MSP.
 *
 * @note   This function should not be modified. When the callback is needed,
 *         the hal_aes_msp_deinit can be implemented in the user file.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration
 *                    information for the specified AES module.
 ****************************************************************************************
 */
void hal_aes_msp_init(aes_handle_t *p_aes);

/**
 ****************************************************************************************
 * @brief  De-initialize the AES MSP.
 *
 * @note   This function should not be modified. When the callback is needed,
 *         the hal_aes_msp_deinit can be implemented in the user file.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration
 *                    information for the specified AES module.
 ****************************************************************************************
 */
void hal_aes_msp_deinit(aes_handle_t *p_aes);

/** @} */

/** @addtogroup AES_Exported_Functions_Group2 AES operation functions
  * @brief AES Encrypt/Decrypt functions
  *
@verbatim
 ===============================================================================
                      ##### AES operation functions #####
 ===============================================================================
    This subsection provides a set of functions allowing to manage the AES encrypt or decrypt.

    (#) There are two mode of transfer:
       (++) Blocking mode: The communication is performed in polling mode.
           The HAL status of all data processing are returned by the same function
           after finishing transfer.
       (++) Non-Blocking mode: The communication is performed using Interrupts
           or DMA. These API return the HAL status.
           The end of the data processing will be indicated through the
           dedicated AES IRQ when using Interrupt mode or the DMA IRQ when
           using DMA mode.
           The hal_aes_done_callback() user callbacks will be executed respectively
           at the end of the encrypt or decrypt process
           The hal_aes_error_callback() user callback will be executed when a error is detected

    (#) Blocking mode API's are :
        (++) hal_aes_ecb_encrypt()
        (++) hal_aes_ecb_decrypt()
        (++) hal_aes_cbc_encrypt()
        (++) hal_aes_cbc_decrypt()

    (#) Non-Blocking mode API's with Interrupt are :
        (++) hal_aes_ecb_encrypt_it()
        (++) hal_aes_ecb_decrypt_it()
        (++) hal_aes_cbc_encrypt_it()
        (++) hal_aes_cbc_decrypt_it()

    (#) Non-Blocking mode API's with DMA are :
        (++) hal_aes_ecb_encrypt_dma()
        (++) hal_aes_ecb_decrypt_dma()
        (++) hal_aes_cbc_encrypt_dma()
        (++) hal_aes_cbc_decrypt_dma()

    (#) A set of encrypt or decrypt Callbacks are provided in Non_Blocking mode:
        (++) hal_aes_done_callback()
        (++) hal_aes_error_callback()

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Encrypted an amount of data in blocking mode in ECB mode.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 * @param[in]  p_plain_data: Pointer to plain data buffer
 * @param[in]  number: Amount of data to be decrypted in bytes
 * @param[out] p_cypher_data: Pointer to cypher data buffer
 * @param[in]  timeout: Timeout duration
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_ecb_encrypt(aes_handle_t *p_aes, uint32_t *p_plain_data, uint32_t number, uint32_t *p_cypher_data, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Decrypted an amount of data in blocking mode in ECB mode.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 * @param[in]  p_cypher_data: Pointer to cypher data buffer
 * @param[in]  number: Amount of data to be decrypted in bytes
 * @param[out] p_plain_data: Pointer to plain data buffer
 * @param[in]  timeout: Timeout duration
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_ecb_decrypt(aes_handle_t *p_aes, uint32_t *p_cypher_data, uint32_t number, uint32_t *p_plain_data, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Encrypted an amount of data in blocking mode in CBC mode.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 * @param[in]  p_plain_data: Pointer to plain data buffer
 * @param[in]  number: Amount of data to be decrypted in bytes
 * @param[out] p_cypher_data: Pointer to cypher data buffer
 * @param[in]  timeout: Timeout duration
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_cbc_encrypt(aes_handle_t *p_aes, uint32_t *p_plain_data, uint32_t number, uint32_t *p_cypher_data, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Decrypted an amount of data in blocking mode in CBC mode.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 * @param[in]  p_cypher_data: Pointer to cypher data buffer
 * @param[in]  number: Amount of data to be decrypted in bytes
 * @param[out] p_plain_data: Pointer to plain data buffer
 * @param[in]  timeout: Timeout duration
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_cbc_decrypt(aes_handle_t *p_aes, uint32_t *p_cypher_data, uint32_t number, uint32_t *p_plain_data, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Encrypted an amount of data in non-blocking mode with Interrupt in ECB mode.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 * @param[in]  p_plain_data: Pointer to plain data buffer
 * @param[in]  number: Amount of data to be decrypted in bytes
 * @param[out] p_cypher_data: Pointer to cypher data buffer
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_ecb_encrypt_it(aes_handle_t *p_aes, uint32_t *p_plain_data, uint32_t number, uint32_t *p_cypher_data);

/**
 ****************************************************************************************
 * @brief  Decrypted an amount of data in non-blocking mode with Interrupt in ECB mode.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 * @param[in]  p_cypher_data: Pointer to cypher data buffer
 * @param[in]  number: Amount of data to be decrypted in bytes
 * @param[out] p_plain_data: Pointer to plain data buffer
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_ecb_decrypt_it(aes_handle_t *p_aes, uint32_t *p_cypher_data, uint32_t number, uint32_t *p_plain_data);

/**
 ****************************************************************************************
 * @brief  Encrypted an amount of data in non-blocking mode with Interrupt in CBC mode.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 * @param[in]  p_plain_data: Pointer to plain data buffer
 * @param[in]  number: Amount of data to be decrypted in bytes
 * @param[out] p_cypher_data: Pointer to cypher data buffer
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_cbc_encrypt_it(aes_handle_t *p_aes, uint32_t *p_plain_data, uint32_t number, uint32_t *p_cypher_data);

/**
 ****************************************************************************************
 * @brief  Decrypted an amount of data in non-blocking mode with Interrupt in CBC mode.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 * @param[in]  p_cypher_data: Pointer to cypher data buffer
 * @param[in]  number: Amount of data to be decrypted in bytes
 * @param[out] p_plain_data: Pointer to plain data buffer
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_cbc_decrypt_it(aes_handle_t *p_aes, uint32_t *p_cypher_data, uint32_t number, uint32_t *p_plain_data);

/**
 ****************************************************************************************
 * @brief  Encrypted an amount of data in non-blocking mode with DMA in ECB mode.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 * @param[in]  p_plain_data: Pointer to plain data buffer
 * @param[in]  number: Amount of data to be decrypted in bytes
 * @param[out] p_cypher_data: Pointer to cypher data buffer
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_ecb_encrypt_dma(aes_handle_t *p_aes, uint32_t *p_plain_data, uint32_t number, uint32_t *p_cypher_data);

/**
 ****************************************************************************************
 * @brief  Decrypted an amount of data in non-blocking mode with DMA in ECB mode.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 * @param[in]  p_cypher_data: Pointer to cypher data buffer
 * @param[in]  number: Amount of data to be decrypted in bytes
 * @param[out] p_plain_data: Pointer to plain data buffer
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_ecb_decrypt_dma(aes_handle_t *p_aes, uint32_t *p_cypher_data, uint32_t number, uint32_t *p_plain_data);

/**
 ****************************************************************************************
 * @brief  Encrypted an amount of data in non-blocking mode with DMA in CBC mode.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 * @param[in]  p_plain_data: Pointer to plain data buffer
 * @param[in]  number: Amount of data to be decrypted in bytes
 * @param[out] p_cypher_data: Pointer to cypher data buffer
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_cbc_encrypt_dma(aes_handle_t *p_aes, uint32_t *p_plain_data, uint32_t number, uint32_t *p_cypher_data);

/**
 ****************************************************************************************
 * @brief  Decrypted an amount of data in non-blocking mode with DMA in CBC mode.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 * @param[in]  p_cypher_data: Pointer to cypher data buffer
 * @param[in]  number: Amount of data to be decrypted in bytes
 * @param[out] p_plain_data: Pointer to plain data buffer
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_aes_cbc_decrypt_dma(aes_handle_t *p_aes, uint32_t *p_cypher_data, uint32_t number, uint32_t *p_plain_data);

/** @} */

/** @addtogroup AES_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callbacks functions
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Handle AES interrupt request.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                     the specified the specified AES module.
 ****************************************************************************************
 */
void hal_aes_irq_handler(aes_handle_t *p_aes);

/**
 ****************************************************************************************
 * @brief  Encrypt or decrypt Done callback.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 ****************************************************************************************
 */
void hal_aes_done_callback(aes_handle_t *p_aes);

/**
 ****************************************************************************************
 * @brief  AES error callback.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 ****************************************************************************************
 */
void hal_aes_error_callback(aes_handle_t *p_aes);

/** @} */

/** @defgroup AES_Exported_Functions_Group3 Peripheral State and Errors functions
  * @brief   AES State and Errors functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the AES.
     (+) hal_aes_get_state() API can be helpful to check in run-time the state of the AES peripheral.
     (+) hal_aes_get_error() check in run-time Errors occurring during communication.
@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Return the AES handle state.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 *
 * @retval ::HAL_AES_STATE_RESET: Peripheral not initialized.
 * @retval ::HAL_AES_STATE_READY: Peripheral initialized and ready for use.
 * @retval ::HAL_AES_STATE_BUSY: Peripheral in indirect mode and busy.
 ****************************************************************************************
 */
hal_aes_state_t hal_aes_get_state(aes_handle_t *p_aes);

/**
 ****************************************************************************************
 * @brief  Return the AES error code.
 *
 * @param[in]  p_aes: Pointer to an AES handle which contains the configuration information for
 *                    the specified AES module.
 *
 * @return AES error code in bitmap format
 ****************************************************************************************
 */
uint32_t hal_aes_get_error(aes_handle_t *p_aes);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR5XX_HAL_AES_H__ */

/** @} */

/** @} */

/** @} */

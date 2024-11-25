/**
 ****************************************************************************************
 *
 * @file    exmon_flash.h
 * @author  BLE Driver Team
 * @brief   Header file of exception monitor flash.
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
 ****************************************************************************************
 */

#ifndef __EXMON_FLASH_H__
#define __EXMON_FLASH_H__

#include <stdint.h>

/**
  * @brief EXMON_FLASH_REGION EXMON_FLASH region.
  *        Note: The user can define its own start and end addresses of EXMON_FLASH region by defining the global macros EXMON_FLASH_START_ADDRESS and EXMON_FLASH_END_ADDRESS in its project file.
  */
#ifndef EXMON_FLASH_START_ADDRESS
#define EXMON_FLASH_START_ADDRESS            (0x0027B000)
#endif
#ifndef EXMON_FLASH_END_ADDRESS
#define EXMON_FLASH_END_ADDRESS              (0x0027C000)
#endif
/***/

/**
  * @brief EXMON_FLASH_ERROR_CODE EXMON_FLASH error code.
  */
#define EXMON_FLASH_ERROR_NONE                EXFLASH_ERROR_NONE           /**< No error                */
#define EXMON_FLASH_ERROR_TIMEOUT             EXFLASH_ERROR_TIMEOUT        /**< Timeout error           */
#define EXMON_FLASH_ERROR_ID                  EXFLASH_ERROR_ID             /**< Flash ID error          */
#define EXMON_FLASH_ERROR_INVALID_PARAM       EXFLASH_ERROR_INVALID_PARAM  /**< Invalid parameters error */
#define EXMON_FLASH_ERROR_INIT                EXFLASH_ERROR_INIT           /**< Init error */
#define EXMON_FLASH_ERROR_BUSY                EXFLASH_ERROR_BUSY           /**< Flash busy error */
#define EXMON_FLASH_ERROR_INITIALIZED         ((uint32_t)0x000000FE)       /**< Initialized error. Indicates that exmon_flash has already been initialized.
                                                                                Note: When this value is returned, it indicates that valid data has already been stored in exmon_flash.
                                                                                Subsequently, exmon_flash_read can be called to retrieve the valid data.*/
#define EXMON_FLASH_ERROR_CHECKSUM            ((uint32_t)0x000000FF)       /**< Checksum error */
/***/

/**
 ****************************************************************************************
 * @brief  Initialize the exmon flash.
 *
 * @retval error_code. @ref EXMON_FLASH_ERROR_CODE.
 ****************************************************************************************
 */
uint32_t exmon_flash_init(void);

/**
 ****************************************************************************************
 * @brief  Restore the exmon flash.
 *         Note: Before the reset, when it is necessary to record information through exmon_flash, user must call exmon_flash_restore before executing CODE in FLASH!
 *
 * @retval error_code. @ref EXMON_FLASH_ERROR_CODE.
 ****************************************************************************************
 */
uint32_t exmon_flash_restore(void);

/**
 ****************************************************************************************
 * @brief  Write an amount of data with specified instruction and address to exmon flash.
 *
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Size of buffer bytes.
 *             Note: Size max = EXMON_FLASH_END_ADDRESS - EXMON_FLASH_START_ADDRESS - 4 - 4 bytes
 *
 * @retval error_code. @ref EXMON_FLASH_ERROR_CODE.
 ****************************************************************************************
 */
uint32_t exmon_flash_write(uint8_t *p_data, uint32_t size);

/**
 ****************************************************************************************
 * @brief  Read an amount of data with specified instruction and address from exmon flash.
 *
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  p_size: Size of buffer bytes.
 *             Note: p_size is a pointer; if error_code = EXMON_FLASH_ERROR_NONE, p_size will return the actual storage length of exmon_flash.
 *
 * @retval error_code. @ref EXMON_FLASH_ERROR_CODE.
 ****************************************************************************************
 */
uint32_t exmon_flash_read(uint8_t *p_data, uint32_t *p_size);

/**
 ****************************************************************************************
 * @brief  Erase exmon flash region.
 *
 * @retval error_code. @ref EXMON_FLASH_ERROR_CODE.
 ****************************************************************************************
 */
uint32_t exmon_flash_erase(void);

#endif /* __EXMON_FLASH_H__ */

/**
 *****************************************************************************************
 *
 * @file swd_ctrl.h
 *
 * @brief SWD function control.
 *
 *****************************************************************************************
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

#ifndef __SWD_CTRL_H__
#define __SWD_CTRL_H__

#include "board_sk.h"

#ifndef SWD_CTRL_DEBUG_ENABLE
#define SWD_CTRL_DEBUG_ENABLE                  0  // 0: disable, 1: enable
#endif
#if SWD_CTRL_DEBUG_ENABLE
#define SWD_CTRL_LOG(x)                        APP_LOG_DEBUG(x)
#define SWD_CTRL_LOG_HEX_DUMP(p_data, length)  APP_LOG_HEX_DUMP(p_data, length)
#else
#define SWD_CTRL_LOG(...)
#define SWD_CTRL_LOG_HEX_DUMP(p_data, length)
#endif

#ifndef PATTERN1
#define PATTERN1 "1cfe8ea2ff3ad9d1d56fd46e5c1ead5b"
#endif

#ifndef PATTERN2
#define PATTERN2 "272ed79a5252595d67d99a68c0bbd731"
#endif

#ifndef PATTERN_LENGTH
#define PATTERN_LENGTH       32  // Unit: byte
#endif

#ifndef RX_PATTERN1_TIMEOUT
#define RX_PATTERN1_TIMEOUT  1000  // Unit:ms
#endif

#ifndef SWD_CTRL_UART_CONFIG
#define SWD_CTRL_UART_ID                APP_UART_ID
#define SWD_CTRL_UART_RX_IO_TYPE        APP_UART_RX_IO_TYPE
#define SWD_CTRL_UART_RX_PIN            APP_UART_RX_PIN
#define SWD_CTRL_UART_RX_PINMUX         APP_UART_RX_PINMUX
#endif

/**
 *****************************************************************************************
 * @brief Refresh WDT when waiting RX pattern2.
 * Note: This is an empty weak function, and users need to reimplement it.
 *
 *****************************************************************************************
 */
void swd_unlock_wdt_refresh(void);

/**
 *****************************************************************************************
 * @brief If SWD is locked, unlock SWD if pattern is received.
 *
 *****************************************************************************************
 */
void swd_unlock_process(void);

#endif /* __SWD_CTRL_H__ */

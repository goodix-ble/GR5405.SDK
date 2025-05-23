/**
 ****************************************************************************************
 *
 * @file hal_delay.h
 *
 * @brief PERIPHERAL API DELAY DRIVER
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

#ifndef ___DELAY_H__
#define ___DELAY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "gr5405.h"

/**
 ****************************************************************************************
 * @brief  Function for delaying execution for number of microseconds. x is based on
 *         Cortex-M4, and this function is based on Data Watchpoint and Trace (DWT) unit.
 * @param[in] number_of_us: number of microseconds
 ****************************************************************************************
 */
void delay_us(uint32_t number_of_us);

/**
 ****************************************************************************************
 * @brief  Function for delaying execution for number of milliseconds. x is based on
 *         Cortex-M4, and this function is based on Data Watchpoint and Trace (DWT) unit.
 * @param[in] number_of_ms: number of milliseconds
 ****************************************************************************************
 */
void delay_ms(uint32_t number_of_ms);

#ifdef __cplusplus
}
#endif

#endif /* ___DELAY_H__ */

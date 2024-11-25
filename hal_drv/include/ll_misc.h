/**
 ****************************************************************************************
 *
 * @file    ll_misc.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of MISC LL library.
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

 /** @addtogroup PERIPHERAL Peripheral Driver
  * @{
  */

/** @addtogroup LL_DRIVER LL Driver
  * @{
  */

/** @defgroup LL_MISC MISC
  * @brief MISC LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LL_MISC_H__
#define __LL_MISC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr5x.h"

/* Exported constants --------------------------------------------------------*/
/** @defgroup MISC_LL_Exported_Constants MISC Exported Constants
  * @{
  */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup MISC_LL_Exported_Functions MISC Exported Functions
  * @{
  */

/**
  * @brief  trigger global reset
  *
  *  Register   | BitsName
  *  -----------|--------
  *  AON_SW_RST | FULL
  *
  * @retval None
  */
__STATIC_INLINE void ll_misc_global_soft_reset(void)
{
    WRITE_REG(MCU_SUB->AON_SW_RST, 0xC5A10000 | MCU_SUB_AON_SW_RST_FULL);
}

/** @} */

#ifdef __cplusplus
}
#endif

#endif

/** @} */

/** @} */

/** @} */

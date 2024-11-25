/**
 ****************************************************************************************
 *
 * @file    hal_pwr_mgmt.h
 * @author  BLE Driver Team
 * @brief   This file contains all the functions prototypes for the HAL
 *          module driver.
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

/** @defgroup HAL_PWR_MGMT PWR_MGMT
  * @brief PWR_MGMT HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HAL_PWR_MGMT_H
#define HAL_PWR_MGMT_H

/* Includes ------------------------------------------------------------------*/
#include "hal_def.h"
typedef enum
{
    HAL_PM_ACTIVE = 0,
    HAL_PM_SLEEP
} hal_pm_status_t;

void hal_pm_init(void);
void hal_pm_deinit(void);
void hal_pm_resume_system(void);
void hal_pm_resume_user(void);
hal_pm_status_t hal_pm_suspend_system(void);
hal_pm_status_t hal_pm_suspend_user(void);

#define HAL_PM_SUSPEND(_PM_FUNC_)  if(HAL_PM_SLEEP != (_PM_FUNC_)){  return HAL_PM_ACTIVE; }

#define HAL_PM_ASSERT_ENABLE 0
#ifndef HAL_PM_ASSERT
#if HAL_PM_ASSERT_ENABLE
    #define HAL_PM_ASSERT(x)  if(!(x)) { while(1); }
#else
    #define HAL_PM_ASSERT(ignore)  ((void)0)
#endif
#endif

#endif /*HAL_PWR_MGMT_H*/

/** @} */

/** @} */

/** @} */

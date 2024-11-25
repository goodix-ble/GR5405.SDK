/**
  ****************************************************************************************
  * @file    hal_pwr_mgmt.c
  * @author  BLE Driver Team
  * @brief   HAL module driver.
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

/* Includes ------------------------------------------------------------------*/
#include "hal_pwr_mgmt.h"

#include "grx_hal.h"
#include "grx_sys.h"

#ifdef HAL_PWR_MGMT_REGISTER_ENABLED
#define IRQ_IS_DISABLE  (1 == __get_PRIMASK())

static void hal_pm_resume(void);
static pwr_mgmt_dev_state_t hal_pm_suspend(void);
#endif

__WEAK void hal_pm_resume_system(void)
{
}

__WEAK hal_pm_status_t hal_pm_suspend_system(void)
{
    return HAL_PM_SLEEP;
}

__WEAK hal_pm_status_t hal_pm_suspend_user(void)
{
    /*
    e.g.
    HAL_PM_SUSPEND(hal_pm_uart_suspend(&uart0_handle));
    HAL_PM_SUSPEND(hal_pm_uart_suspend(&uart1_handle));
    HAL_PM_SUSPEND(hal_pm_i2c_resume(&i2c0_handle));
    return HAL_PM_SLEEP;
    */
    return HAL_PM_SLEEP;
}

__WEAK void hal_pm_resume_user(void)
{
    /*
    e.g.
    hal_pm_uart_resume(&uart0_handle);
    hal_pm_uart_resume(&uart1_handle);
    hal_pm_i2c_resume(&i2c0_handle);
    */
}


void hal_pm_init(void)
{
#ifdef HAL_PWR_MGMT_REGISTER_ENABLED
    pwr_mgmt_dev_init(hal_pm_resume);
    pwr_mgmt_set_callback(hal_pm_suspend, NULL);
#endif
}

void hal_pm_deinit(void)
{
#ifdef HAL_PWR_MGMT_REGISTER_ENABLED
    pwr_mgmt_dev_init(NULL);
    pwr_mgmt_set_callback(NULL, NULL);
#endif
}

#ifdef HAL_PWR_MGMT_REGISTER_ENABLED
static void hal_pm_resume(void)
{
    hal_pm_resume_system();/*lint !e522 MISRA exception. hal_pm_resume_system is a weak function */
    hal_pm_resume_user();/*lint !e522 MISRA exception. hal_pm_resume_user is a weak function */
}

static pwr_mgmt_dev_state_t hal_pm_suspend(void)
{
    pwr_mgmt_dev_state_t ret;
    //assert IRQ disable
    HAL_PM_ASSERT(IRQ_IS_DISABLE);
    if(HAL_PM_ACTIVE == hal_pm_suspend_system())
    {
        ret = DEVICE_BUSY;
    }
    if(HAL_PM_ACTIVE == hal_pm_suspend_user())
    {
        ret = DEVICE_BUSY;
    }
    else
    {
        ret = DEVICE_IDLE;
    }
    return ret;
}
#endif /* HAL_PWR_MGMT_REGISTER_ENABLED */

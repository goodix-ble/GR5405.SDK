/**
 ****************************************************************************************
 *
 * @file    hal.h
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

/** @defgroup HAL_HAL HAL
  * @brief HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ___HAL_H__
#define ___HAL_H__

#include <stdint.h>
#include "gr5x.h"
/**
 ****************************************************************************************
 * @brief  This function returns the HAL revision
 *
 * @return version: 0xXYZR (8 bits for each decimal, R for RC)
 ****************************************************************************************
 */
uint32_t hal_get_hal_version(void);

#ifdef HAL_ADC_MODULE_ENABLED
#include "hal_adc.h"
#endif /* HAL_ADC_MODULE_ENABLED */

#ifdef HAL_AON_GPIO_MODULE_ENABLED
#include "hal_aon_gpio.h"
#endif /* HAL_AON_GPIO_MODULE_ENABLED */

#ifdef HAL_AON_WDT_MODULE_ENABLED
#include "hal_aon_wdt.h"
#endif /* HAL_AON_WDT_MODULE_ENABLED */

#ifdef HAL_CALENDAR_MODULE_ENABLED
#include "hal_calendar.h"
#endif /* HAL_CALENDAR_MODULE_ENABLED */

#ifdef HAL_DMA_MODULE_ENABLED
#include "hal_dma.h"
#endif /* HAL_DMA_MODULE_ENABLED */

#ifdef HAL_DUAL_TIMER_MODULE_ENABLED
#include "hal_dual_tim.h"
#endif /* HAL_DUAL_TIMER_MODULE_ENABLED */

#ifdef HAL_GPIO_MODULE_ENABLED
#include "hal_gpio.h"
#endif /* HAL_GPIO_MODULE_ENABLED */

#ifdef HAL_I2C_MODULE_ENABLED
#include "hal_i2c.h"
#endif /* HAL_I2C_MODULE_ENABLED */

#ifdef HAL_MSIO_MODULE_ENABLED
#include "hal_msio.h"
#endif /* HAL_MSIO_MODULE_ENABLED */

#ifdef HAL_PWM_MODULE_ENABLED
#include "hal_pwm.h"
#endif /* HAL_PWM_MODULE_ENABLED */

#ifdef HAL_PWR_MODULE_ENABLED
#include "hal_pwr.h"
#endif /* HAL_PWR_MODULE_ENABLED */

#if defined(HAL_SPI_MODULE_ENABLED) || defined(HAL_SPIM_MODULE_ENABLED)
#include "hal_spi.h"
#endif /* HAL_SPI_MODULE_ENABLED / HAL_SPIS_MODULE_ENABLED */

#ifdef HAL_SPIS_MODULE_ENABLED
#include "hal_spis.h"
#endif /* HAL_SPIS_MODULE_ENABLED */

#ifdef HAL_TIMER_MODULE_ENABLED
#include "hal_tim.h"
#endif /* HAL_TIMER_MODULE_ENABLED */

#ifdef HAL_UART_MODULE_ENABLED
#include "hal_uart.h"
#endif /* HAL_UART_MODULE_ENABLED */

#ifdef HAL_WDT_MODULE_ENABLED
#include "hal_wdt.h"
#endif /* HAL_WDT_MODULE_ENABLED */

#ifdef HAL_XQSPI_MODULE_ENABLED
#include "hal_xqspi.h"
#endif /* HAL_XQSPI_MODULE_ENABLED */

#ifdef HAL_EXFLASH_MODULE_ENABLED
#include "hal_exflash.h"
#endif /* HAL_EXFLASH_MODULE_ENABLED */

#ifdef HAL_EFUSE_MODULE_ENABLED
#include "hal_efuse.h"
#endif /* HAL_EFUSE_MODULE_ENABLED */

#ifdef HAL_CGC_MODULE_ENABLED
#include "hal_cgc.h"
#endif /* HAL_CGC_MODULE_ENABLED */

#ifdef HAL_AES_MODULE_ENABLED
#include "hal_aes.h"
#endif /* HAL_AES_MODULE_ENABLED */

#ifdef HAL_HMAC_MODULE_ENABLED
#include "hal_hmac.h"
#endif /* HAL_HMAC_MODULE_ENABLED */

#ifdef HAL_PKC_MODULE_ENABLED
#include "hal_pkc.h"
#endif /* HAL_PKC_MODULE_ENABLED */

#ifdef HAL_RNG_MODULE_ENABLED
#include "hal_rng.h"
#endif /* HAL_RNG_MODULE_ENABLED */

#ifdef HAL_COMP_MODULE_ENABLED
#include "hal_comp.h"
#endif /* HAL_COMP_MODULE_ENABLED */

#ifdef HAL_SLEEP_TIMER_MODULE_ENABLED
#include "hal_sleep_timer.h"
#endif /* HAL_SLEEP_TIMER_MODULE_ENABLED */

#ifdef HAL_RTC_MODULE_ENABLED
#include "hal_rtc.h"
#endif /* HAL_RTC_MODULE_ENABLED */

#ifdef HAL_PWR_MGMT_ENABLED
#include "hal_pwr_mgmt.h"
#endif

#ifdef HAL_CLOCK_MODULE_ENABLED
#include "hal_clock.h"
#endif /* HAL_CLOCK_MODULE_ENABLED */

#ifdef HAL_PMU_MODULE_ENABLED
#include "ll_aon_pmu.h"
#include "ll_aon_rf.h"
#include "ll_clk_cal.h"
#include "ll_ddvs.h"
#endif /* HAL_PMU_MODULE_ENABLED */

#ifdef HAL_PMU_MODULE_GR5410_ENABLED
#include "ll_aon_pmu.h"
#include "ll_clk.h"
#include "ll_clk_cal.h"
#endif /* HAL_PMU_MODULE_GR5410_ENABLED */

#ifdef HAL_MCU_HTABLE_MODULE_ENABLED
#include "ll_mcu_htable.h"
#endif /* HAL_MCU_HTABLE_MODULE_ENABLED */

#ifdef HAL_BR_MODULE_ENABLED
#include "hal_br.h"
#endif /* HAL_BR_MODULE_ENABLED */

#ifdef HAL_BOD_MODULE_ENABLED
#include "hal_bod.h"
#endif
#endif /* HAL_BOD_MODULE_ENABLED */

#ifdef HAL_CAN_MODULE_ENABLED
#include "hal_can.h"
#endif /* HAL_CAN_MODULE_ENABLED */

#ifdef HAL_LIN_MODULE_ENABLED
#include "hal_lin.h"
#endif /* HAL_LIN_MODULE_ENABLED */

/* ___HAL_H__ */

/** @} */

/** @} */

/** @} */

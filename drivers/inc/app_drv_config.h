/**
 ****************************************************************************************
 *
 * @file    app_drv_config.h
 * @author  BLE Driver Team
 * @brief   Header file of app driver config code.
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

/** @addtogroup APP_DRIVER APP DRIVER
 *  @{
 */

/** @defgroup APP_DRIVER_CONFIG DRIVER CONFIG
  * @brief APP DRIVER CONFIG
  * @{
  */


#ifndef _APP_DRV_CONFIG_H_
#define _APP_DRV_CONFIG_H_

#include "custom_config.h"
#include "grx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif
#define APP_DVR_LOG_LVL_NONE        (0)                   /**< None log level define */
#define APP_DVR_LOG_LVL_ERR         (1)                   /**< Error log level define */
#define APP_DVR_LOG_LVL_WARN        (2)                   /**< Warning log level define */
#define APP_DVR_LOG_LVL_INFO        (3)                   /**< Info log level define */

#ifndef APP_DRV_LOG_LEVEL
#define APP_DRV_LOG_LEVEL           APP_DVR_LOG_LVL_NONE  /**< App driver log level setting */
#endif
#ifndef APP_DRV_LOG_INTERFACE
#define APP_DRV_LOG_INTERFACE       printf                /**< App driver log interface setting */
#endif
#ifndef APP_DRV_ASSERT_ENABLE
#define APP_DRV_ASSERT_ENABLE       0                     /**< App driver assert enable */
#endif

#define APP_DRIVER_GR551X           0x0              /**< APP_DRIVER for GR551X */
#define APP_DRIVER_GR5525X          0x1              /**< APP_DRIVER for GR5525X */
#define APP_DRIVER_GR5526X          0x2              /**< APP_DRIVER for GR5526X */
#define APP_DRIVER_GR5332X          0x3              /**< APP_DRIVER for GR5332X */
#define APP_DRIVER_GR5405           0x3              /**< APP_DRIVER for GR5405 */

#ifdef SOC_GR5515
#define APP_DRIVER_CHIP_TYPE  APP_DRIVER_GR551X      /**< GR5515 chip type*/
#define SOC_GPIO_PINS_MAX     (32)                   /**< GR5515 max gpio pins */
#define SOC_AON_PINS_MAX      (8)                    /**< GR5515 max aon pins */
#elif defined(SOC_GR5X25)
#define APP_DRIVER_CHIP_TYPE  APP_DRIVER_GR5525X     /**< GR5525 chip type*/
#define SOC_GPIO_PINS_MAX     (32)                   /**< GR5525 max gpio pins */
#define SOC_AON_PINS_MAX      (8)                    /**< GR5525 max aon pins */
#elif defined(SOC_GR5526)
#define APP_DRIVER_CHIP_TYPE  APP_DRIVER_GR5526X     /**< GR5526 chip type*/
#define SOC_GPIO_PINS_MAX     (34)                   /**< GR5526 max gpio pins */
#define SOC_AON_PINS_MAX      (8)                    /**< GR5526 max aon pins */
#elif defined(SOC_GR533X)
#define APP_DRIVER_CHIP_TYPE  APP_DRIVER_GR5332X     /**< GR533X chip type*/
#define SOC_GPIO_PINS_MAX     (14)                   /**< GR533X max gpio pins */
#define SOC_AON_PINS_MAX      (8)                    /**< GR533X max aon pins */
#elif defined(SOC_GR5405)
#define APP_DRIVER_CHIP_TYPE  APP_DRIVER_GR5405      /**< GR5405 chip type*/
#define SOC_GPIO_PINS_MAX     (14)                   /**< GR5405 max gpio pins */
#define SOC_AON_PINS_MAX      (8)                    /**< GR5405 max aon pins */
#elif defined(SOC_GR5410)
#define SOC_GPIO_PINS_MAX     (14)                   /**< GR5405 max gpio pins */
#define SOC_AON_PINS_MAX      (8)                    /**< GR5405 max aon pins */
#endif
#if defined(SOC_GR515)
#define  GPIO_INSTANCE_MAX  2
#elif defined(SOC_GR5X25) || defined(SOC_GR5526)
#define  GPIO_INSTANCE_MAX  3
#elif defined(SOC_GR533X) || defined(SOC_GR5405)
#define  GPIO_INSTANCE_MAX  1
#elif defined(SOC_GR5410)
#define  GPIO_INSTANCE_MAX  2
#endif

#if defined(SOC_GR5410)
#define GPIOA_IRQn  AON_EXT_IRQn
#else
#define GPIOA_IRQn  EXT0_IRQn
#endif

#if defined(SOC_GR5515) || defined(SOC_GR5X25) || defined(SOC_GR5526) || defined(SOC_GR533X) || defined(SOC_GR5405)
#define APP_IO_AON_GPIO_ENABLE
#define APP_IO_MSIO_ENABLE
#endif
#if defined(SOC_GR5X25) || defined(SOC_GR5526) || defined(SOC_GR533X) || defined(SOC_GR5405) || defined(SOC_GR5410)
#define APP_IO_IT_BOTH_EDGE_ENABLE
#endif
#if defined(SOC_GR5X25) || defined(SOC_GR533X) || defined(SOC_GR5405) || defined(SOC_GR5410)
#define APP_IO_STRENGTH_ENABLE
#define APP_IO_SPEED_ENABLE
#define APP_IO_INPUT_TYPE_ENABLE
#endif
#if defined(SOC_GR5515)
#define APP_IO_GR551X_LEGACY_ENABLE
#endif
#if defined(SOC_GR533X) || defined(SOC_GR5405)
#define APP_IO_MUX_ARBITRARY_V0
#endif
#if defined(SOC_GR533X) || defined(SOC_GR5405) || defined(SOC_GR5410)
#define APP_IO_MUX APP_IO_MUX_0
#elif defined(SOC_GR5X25)
#define APP_IO_MUX APP_IO_MUX_8
#elif defined(SOC_GR5515) || defined(SOC_GR5526)
#define APP_IO_MUX APP_IO_MUX_7
#endif

#if defined(SOC_GR5410)
#define APP_IO_GPIO_SUPPORT_ANA_MODE
#endif

#if defined(SOC_GR5515) || defined(SOC_GR533X) || defined(SOC_GR5405) || defined(SOC_GR5410)
#ifndef  DMA0
#define  DMA_INSTANCE_MAX   1
#else
#define  DMA_INSTANCE_MAX   1
#endif
#elif defined(SOC_GR5526) || defined(SOC_GR5X25)
#define  DMA_INSTANCE_MAX   2
#define  APP_DMA_SG_LLP_ENABLE
#endif
#define  DMA_CHANNEL_MAX    DMA_Channel_NUM_MAX
#if defined(SOC_GR5515) || defined(SOC_GR5X25) || defined(SOC_GR5526)
#define APP_DMA_MODE_ENABLE
#endif
#if defined(SOC_GR5515)
#define APP_DMA_GR551X_LEGACY
#endif

#if defined(SOC_GR5410)
#ifndef DMA0_IRQn
#define DMA0_IRQn DMA_IRQn
#endif
#ifndef DMA0_REQUEST_MEM
#define DMA0_REQUEST_MEM 0
#endif
#ifndef DMA0_REQUEST_UART0_TX
#define DMA0_REQUEST_UART0_TX DMA_REQUEST_UART0_TX
#endif
#ifndef DMA0_REQUEST_UART0_RX
#define DMA0_REQUEST_UART0_RX DMA_REQUEST_UART0_RX
#endif
#ifndef DMA0_REQUEST_UART1_TX
#define DMA0_REQUEST_UART1_TX DMA_REQUEST_UART1_TX
#endif
#ifndef DMA0_REQUEST_UART1_RX
#define DMA0_REQUEST_UART1_RX DMA_REQUEST_UART1_RX
#endif
#ifndef DMA0_REQUEST_I2C0_TX
#define DMA0_REQUEST_I2C0_TX DMA_REQUEST_I2C0_TX
#endif
#ifndef DMA0_REQUEST_I2C0_RX
#define DMA0_REQUEST_I2C0_RX DMA_REQUEST_I2C0_RX
#endif
#ifndef DMA0_REQUEST_I2C1_TX
#define DMA0_REQUEST_I2C1_TX DMA_REQUEST_I2C1_TX
#endif
#ifndef DMA0_REQUEST_I2C1_RX
#define DMA0_REQUEST_I2C1_RX DMA_REQUEST_I2C1_RX
#endif
#ifndef DMA0_REQUEST_PWM0
#define DMA0_REQUEST_PWM0 DMA_REQUEST_PWM0
#endif
#ifndef DMA0_REQUEST_SPIM0_TX
#define DMA0_REQUEST_SPIM0_TX DMA_REQUEST_SPIM0_TX
#endif
#ifndef DMA0_REQUEST_SPIM0_RX
#define DMA0_REQUEST_SPIM0_RX DMA_REQUEST_SPIM0_RX
#endif
#ifndef DMA0_REQUEST_SPIM1_TX
#define DMA0_REQUEST_SPIM1_TX DMA_REQUEST_SPIM1_TX
#endif
#ifndef DMA0_REQUEST_SPIM1_RX
#define DMA0_REQUEST_SPIM1_RX DMA_REQUEST_SPIM1_RX
#endif
#ifndef DMA0_REQUEST_SPIM2_TX
#define DMA0_REQUEST_SPIM2_TX DMA_REQUEST_SPIM2_TX
#endif
#ifndef DMA0_REQUEST_SPIM2_RX
#define DMA0_REQUEST_SPIM2_RX DMA_REQUEST_SPIM2_RX
#endif
#endif

#if defined(SOC_GR515)
#define  UART_INSTANCE_MAX  2
#elif defined(SOC_GR5X25)
#define  UART_INSTANCE_MAX  4
#elif  defined(SOC_GR5526)
#define  UART_INSTANCE_MAX  6
#elif defined(SOC_GR533X) || defined(SOC_GR5405)
#define  UART_INSTANCE_MAX  2
#elif defined(SOC_GR5410)
#define  UART_INSTANCE_MAX  2
#endif
#if defined(SOC_GR515)
#define  UART1_NO_DMA
#endif
#if defined(SOC_GR5526)
#define UART0_DMA1_REQ_ENABLE
#endif

#if defined(SOC_GR515)
#define  I2C_INSTANCE_MAX  2
#elif defined(SOC_GR5X25)
#define  I2C_INSTANCE_MAX  4
#elif  defined(SOC_GR5526)
#define  I2C_INSTANCE_MAX  6
#elif defined(SOC_GR533X) || defined(SOC_GR5405)
#define  I2C_INSTANCE_MAX  2
#elif defined(SOC_GR5410)
#define  I2C_INSTANCE_MAX  2
#endif
#if defined(SOC_GR5X25) || defined(SOC_GR5526)
#define I2C0_DMA1_REQ_ENABLE
#define I2C1_DMA1_REQ_ENABLE
#endif
#if defined(SOC_GR5515) || defined(SOC_GR533X) || defined(SOC_GR5405) || defined(SOC_GR5410)
#define I2C0_DMA0_REQ_ENABLE
#define I2C1_DMA0_REQ_ENABLE
#endif

#if defined(SOC_GR5515)
#define  APP_I2C_TIMING_API_ENABLE
#define  APP_I2C_TX_RX_ARBITRARY
#define  APP_I2C_MEM_CALLBACK_ENABLE
#endif

#if defined(SOC_GR5515)
#define APP_SPI_GR551X_LEGACY
#endif
#if defined(SOC_GR5X25) || defined(SOC_GR5526)
#define APP_SPI_DMA_LLP_DISPLAY_ENABLE
#define APP_SPI_DMA1_HS_ENABLE
#endif
#if defined(SOC_GR5526)
#define APP_SPI_TX_WITH_IA_32_ENABLE
#endif
#if defined(SOC_GR5410)
#define APP_SPI_NOT_SUPPORT_SLAVE
#endif

#if defined(SOC_GR533X) || defined(SOC_GR5405) || defined(SOC_GR5410)
#define APP_TIM_IO_CAPTURE_ENABLE
#define APP_DUAL_TIM_IO_ENABLE
#endif
#if defined(SOC_GR5410)
#define APP_DUAL_TIM_IO_MUX_ENABLE
#endif
#ifndef DUAL_TIMER0_BASE
#define DUAL_TIMER0_BASE  DUAL_TIM0_BASE
#endif
#ifndef DUAL_TIMER1_BASE
#define DUAL_TIMER1_BASE  DUAL_TIM1_BASE
#endif


#if defined(SOC_GR533X) || defined(SOC_GR5405) || defined(SOC_GR5410)
#define APP_PWM_CODING_ENABLE
#endif
#if !defined(SOC_GR5515)
#define APP_PWM_STOP_SPEC_CH_ENABLE
#endif

#if defined(SOC_GR5X25) || defined(SOC_GR5526) || defined(SOC_GR533X) || defined(SOC_GR5405)
#define APP_RTC_TIME_SYNC_ENABLE
#endif
#if defined(SOC_GR5X25) || defined(SOC_GR533X) || defined(SOC_GR5405)
#define SYSTEM_RTC_SLOW_CLOCK_ENABLE
#endif
#if defined(SOC_GR533X) || defined(SOC_GR5405) || defined(SOC_GR5410)
#if defined(ENV_USE_FREERTOS)
#define OS_TICK_BASE_RTC_ENABLE
#endif
#endif
#if (CFG_LPCLK_INTERNAL_EN == 0) || defined(SOC_GR5X25) || defined(SOC_GR533X) || defined(SOC_GR5405) || defined(SOC_GR5410)
#define APP_RTC_WORK_AROUND_DISABLE
#endif
#if defined(SOC_GR5515)
#define APP_RTC_GR551X_LEGACY
#endif
#if defined(SOC_GR5410)
#ifndef CALENDAR_IRQn
#define CALENDAR_IRQn RTC_IRQn
#endif
#define APP_RTC_NO_LL_CALENDAR_DRIVER
#endif

#if defined(SOC_GR5X25) || defined(SOC_GR533X) || defined(SOC_GR5405)
#define SYSTEM_SLOW_CLOCK_ENABLE
#endif

#if !defined(SOC_GR5515)
#define APP_ADC_VBAT_TEMP_CONV_ENABLE
#endif
#if defined(SOC_GR5526)
#define APP_ADC_CLOCK_START_ENABLE
#define APP_ADC_GET_AVG_ENABLE
#endif
#if defined(SOC_GR5515)
#define  APP_ADC_INPUT_SRC_GR551X_LEGACY_ENABLE
#endif

#if defined(SOC_GR5515) || defined(SOC_GR5X25) || defined(SOC_GR5526) || defined(SOC_GR533X) || defined(SOC_GR5405)
#define APP_ADC_SNSADC_ENABLE
#define APP_ADC_IO_TYPE     APP_IO_TYPE_MSIO
#define APP_ADC_IO_MUX      APP_IO_MUX
#endif

#if defined(SOC_GR5410)
#define APP_ADC_GPADC_ENABLE
#define APP_ADC_IO_TYPE     APP_IO_TYPE_GPIOA
#define APP_ADC_IO_MUX      APP_IO_MUX_2
#endif

#if defined(SOC_GR5X25) || defined(SOC_GR5526) || defined(SOC_GR533X) || defined(SOC_GR5405) || defined(SOC_GR5410)
#define APP_BOD_DEASSERT_ENABLE
#endif
#if defined(SOC_GR533X) || defined(SOC_GR5405) || defined(SOC_GR5410)
#define APP_BOD_AUTO_POWER_BYPASS_ENABLE
#endif
#if defined(SOC_GR5526)
#define APP_BOD_LEVEL_8_15_ENABLE
#endif

#if !defined(SOC_GR5515)
#define APP_COMP_EDGE_ENABLE
#endif
#if defined(SOC_GR5515)
#define APP_COMP_WAKE_SOURCE_CFG_ENABLE
#endif
#if defined(SOC_GR5410)
#ifndef COMP_EXT_IRQn
#define COMP_EXT_IRQn COMP_IRQn
#endif
#ifndef COMP_EXT_IRQn
#define COMP_EXT_IRQn COMP_IRQn
#endif
#endif
#if defined(SOC_GR5515)
#define APP_WDT_GR515_LEGACY
#endif
#if defined(SOC_GR5410)
#define APP_AON_WDT_MULTI_INSTANCE
#define AON_WDT_INSTANCE_MAX    2
#define APP_AON_WDT_SUPPORT_FREQ_DIV
#else
#define AON_WDT_INSTANCE_MAX    1
#endif

#if defined(SOC_GR5515)
#define APP_PWR_MGMT_HAL_INIT_ENABLE
#endif
#if defined(SOC_GR5X25) || defined(SOC_GR5526) || defined(SOC_GR533X)
  #define APP_PWR_MGMT_HAL_REG_ENABLE
  #define APP_PWR_MGMT_WORD_CHECK_ENABLE
#if defined(SOC_GR5X25) || defined(SOC_GR5526)
  #define APP_PWR_MGMT_WORD_CHECK_EXTRA_ENABLE
#endif
#endif
#if defined(SOC_GR5405) || defined(SOC_GR5410)
  #define APP_PWR_MGMT_HAL_REG_ENABLE
#endif

#if defined(SOC_GR5410)
#define LIN_INSTANCE_MAX       2
#endif

/**
 * @defgroup APP_DRV_PERIPHERAL_PRIORITY_DEFINE Defines
 * @{
 */
/**@brief APP driver peripheral priority define. */
#ifndef APP_DRIVER_ADC_WAKEUP_PRIORITY
#define APP_DRIVER_ADC_WAKEUP_PRIORITY              WAKEUP_PRIORITY_HIGH  /**< ADC Wakeup priority High */
#endif

#ifndef APP_DRIVER_AES_WAKEUP_PRIORITY
#define APP_DRIVER_AES_WAKEUP_PRIORITY              WAKEUP_PRIORITY_MID   /**< AES Wakeup priority Mid */
#endif

#ifndef APP_DRIVER_COMP_WAKEUP_PRIORITY
#define APP_DRIVER_COMP_WAKEUP_PRIORITY             WAKEUP_PRIORITY_LOW   /**< COMP Wakeup priority Low */
#endif

#ifndef APP_DRIVER_DUAL_TIM_WAKEUP_PRIORITY
#define APP_DRIVER_DUAL_TIM_WAKEUP_PRIORITY         WAKEUP_PRIORITY_MID   /**< DUAL TIM Wakeup priority Mid */
#endif

#ifndef APP_DRIVER_DMA_WAKEUP_PRIORITY
#define APP_DRIVER_DMA_WAKEUP_PRIORITY              WAKEUP_PRIORITY_HIGH  /**< DMA Wakeup priority High */
#endif

#ifndef APP_DRIVER_UART_WAKEUP_PRIORITY
#define APP_DRIVER_UART_WAKEUP_PRIORITY             WAKEUP_PRIORITY_HIGH  /**< Uart Wakeup priority High */
#endif

#ifndef APP_DRIVER_HMAC_WAKEUP_PRIORITY
#define APP_DRIVER_HMAC_WAKEUP_PRIORITY             WAKEUP_PRIORITY_MID   /**< Hmac Wakeup priority Mid */
#endif

#ifndef APP_DRIVER_I2C_WAKEUP_PRIORITY
#define APP_DRIVER_I2C_WAKEUP_PRIORITY              WAKEUP_PRIORITY_HIGH  /**< I2C Wakeup priority High */
#endif

#ifndef APP_DRIVER_I2S_WAKEUP_PRIORITY
#define APP_DRIVER_I2S_WAKEUP_PRIORITY              WAKEUP_PRIORITY_HIGH  /**< I2S Wakeup priority High */
#endif

#ifndef APP_DRIVER_QSPI_WAKEUP_PRIORITY
#define APP_DRIVER_QSPI_WAKEUP_PRIORITY             WAKEUP_PRIORITY_HIGH  /**< QSPI Wakeup priority High */
#endif

#ifndef APP_DRIVER_RNG_WAKEUP_PRIORITY
#define APP_DRIVER_RNG_WAKEUP_PRIORITY              WAKEUP_PRIORITY_MID   /**< RNG Wakeup priority Mid */
#endif

#ifndef APP_DRIVER_SPI_WAKEUP_PRIORITY
#define APP_DRIVER_SPI_WAKEUP_PRIORITY              WAKEUP_PRIORITY_HIGH  /**< SPI Wakeup priority High */
#endif

#ifndef APP_DRIVER_TIM_WAKEUP_PRIORITY
#define APP_DRIVER_TIM_WAKEUP_PRIORITY              WAKEUP_PRIORITY_MID   /**< TIM Wakeup priority Mid */
#endif

#ifndef APP_DRIVER_PWM_WAKEUP_PRIORITY
#define APP_DRIVER_PWM_WAKEUP_PRIORITY              WAKEUP_PRIORITY_MID   /**< PWM Wakeup priority Mid */
#endif

#ifndef APP_DRIVER_ISO7816_WAKEUP_PRIORITY
#define APP_DRIVER_ISO7816_WAKEUP_PRIORITY          WAKEUP_PRIORITY_HIGH    /**< ISO7816 Wakeup priority High */
#endif

#ifndef APP_DRIVER_PKC_WAKEUP_PRIORITY
#define APP_DRIVER_PKC_WAKEUP_PRIORITY              WAKEUP_PRIORITY_HIGH    /**< PKC Wakeup priority High */
#endif

#ifndef APP_DRIVER_DSPI_WAKEUP_PRIORITY
#define APP_DRIVER_DSPI_WAKEUP_PRIORITY             WAKEUP_PRIORITY_HIGH    /**< DSPI Wakeup priority High */
#endif

#ifndef APP_DRIVER_PDM_WAKEUP_PRIORITY
#define APP_DRIVER_PDM_WAKEUP_PRIORITY              WAKEUP_PRIORITY_HIGH    /**< PDM Wakeup priority High */
#endif

#ifndef APP_DRIVER_CAN_WAKEUP_PRIORITY
#define APP_DRIVER_CAN_WAKEUP_PRIORITY              WAKEUP_PRIORITY_HIGH    /**< CAN Wakeup priority High */
#endif

#ifndef APP_DRIVER_LIN_WAKEUP_PRIORITY
#define APP_DRIVER_LIN_WAKEUP_PRIORITY              WAKEUP_PRIORITY_HIGH    /**< LIN Wakeup priority High */
#endif

/**@} */


/**@addtogroup APP_DRV_WAKEUP_PRIORITY_ENUM Enumerations
 * @{
 */
/**@brief APP driver peripheral wakeup priority define. */
typedef enum
{
    WAKEUP_PRIORITY_LOW = 1,          /**< Wakeup priority low */
    WAKEUP_PRIORITY_MID,              /**< Wakeup priority mid */
    WAKEUP_PRIORITY_HIGH              /**< Wakeup priority high */
} wakeup_priority_t;
/** @} */

#ifndef APP_DRIVER_WAKEUP_CALL_FUN
//#define APP_DRIVER_WAKEUP_CALL_FUN
#endif

#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */
/** @} */


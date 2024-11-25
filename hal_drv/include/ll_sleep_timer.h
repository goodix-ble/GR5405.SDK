/**
 ****************************************************************************************
 *
 * @file    ll_sleep_timer.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of sleep timer LL library.
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2023 GOODIX
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

/** @defgroup LL_SLP_TIMER SLP_TIMER
  * @brief SLP_TIMER LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ___LL_SLEEP_TIMER_H__
#define ___LL_SLEEP_TIMER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr5405.h"

#if defined(SLP_TIMER0)

/**
  * @defgroup  SLP_TIMER_LL_TIMER_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/

/** @defgroup SLP_TIMER_LL_Exported_Constants SLP_TIMER Exported Constants
  * @{
  */


/** @defgroup SLP_TIMER_RUN_MODE Sleep Timer Run Mode
 * @{
 */
#define SLP_TIMER_CFG_CNT_SLP_ONLY                     (0x1U << SLP_TIMER_CFG_COUNT_MODE_POS)
#define SLP_TIMER_CFG_CNT_ANY_CONDITION                (0x0U << SLP_TIMER_CFG_COUNT_MODE_POS)
#define SLP_TIMER_CFG_SINGLE_MODE                      (0x1U << SLP_TIMER_CFG_MODE_POS)
#define SLP_TIMER_CFG_AUTO_RELOAD                      (0x0U << SLP_TIMER_CFG_MODE_POS)
#define LL_SLEEP_TIMER_SINGLE_MODE_0                   (SLP_TIMER_CFG_SINGLE_MODE | SLP_TIMER_CFG_CNT_SLP_ONLY)
#define LL_SLEEP_TIMER_SINGLE_MODE_1                   (SLP_TIMER_CFG_SINGLE_MODE | SLP_TIMER_CFG_CNT_ANY_CONDITION)
#define LL_SLEEP_TIMER_AUTO_MODE                       (SLP_TIMER_CFG_AUTO_RELOAD | SLP_TIMER_CFG_CNT_ANY_CONDITION)

/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup SLP_TIMER_LL_Exported_Macros SLP_TIMER Exported Macros
  * @{
  */

/** @defgroup SLP_TIMER_LL_EM_WRITE_READ Common write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in SLP_TIMER register
  * @param  __instance__ SLP_TIMER instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_SLP_TIMER_WriteReg(__instance__, __REG__, __VALUE__)   WRITE_REG(__instance__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in SLP_TIMER register
  * @param  __instance__ SLP_TIMER instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_SLP_TIMER_ReadReg(__instance__, __REG__)               READ_REG(__instance__->__REG__)

/** @} */

/** @} */

/** @} */


/* Exported functions --------------------------------------------------------*/
/** @defgroup SLP_TIMER_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup SLP_TIMER_LL_EF_Configuration Configuration functions
  * @{
  */

/**
  * @brief  Configure Sleep Timer Work mode and Start Sleep Timer.
  *
  *  Register|BitsName
  *  --------|--------
  *  CFG0    | EN
  *  CFG0    | VAL_SET
  *  CFG_0   | MODE
  *  CFG_0   | COUNT_MODE
  *
  * @param  SLP_TIMERx SLEEP TIMER instance
  * @param  mode SLEEP TIMER mode
  * @retval None
  */
__STATIC_INLINE void ll_sleep_timer_config_and_start(slp_timer_regs_t *SLP_TIMERx, uint32_t mode)
{
    WRITE_REG(SLP_TIMERx->CFG, (SLP_TIMER_CFG_EN | SLP_TIMER_CFG_VAL_SET | SLP_TIMER_CFG_WEN | mode));
}

/**
  * @brief  Enable sleep timer
  *
  *  Register|BitsName
  *  --------|--------
  *  CFG0    | EN
  *
  * @param  SLP_TIMERx SLEEP TIMER instance
  * @retval None
  */
__STATIC_INLINE void ll_sleep_timer_enable(slp_timer_regs_t *SLP_TIMERx)
{
    MODIFY_REG(SLP_TIMERx->CFG, (SLP_TIMER_CFG_EN), (SLP_TIMER_CFG_EN | SLP_TIMER_CFG_WEN));
}

/**
  * @brief  disable sleep timer
  *
  *  Register|BitsName
  *  --------|--------
  *  CFG0    | EN
  *
  * @param  SLP_TIMERx SLEEP TIMER instance
  * @retval None
  */
__STATIC_INLINE void ll_sleep_timer_disable(slp_timer_regs_t *SLP_TIMERx)
{
    MODIFY_REG(SLP_TIMERx->CFG, (SLP_TIMER_CFG_EN), (SLP_TIMER_CFG_WEN));
}

/**
  * @brief  enable sleep timer
  *
  *  Register|BitsName
  *  --------|--------
  *  CFG0    | VAL_SET
  *
  * @param  SLP_TIMERx SLEEP TIMER instance
  * @retval None
  */
__STATIC_INLINE void ll_sleep_timer_load_counter_value(slp_timer_regs_t *SLP_TIMERx)
{
    MODIFY_REG(SLP_TIMERx->CFG, (SLP_TIMER_CFG_VAL_SET), (SLP_TIMER_CFG_VAL_SET | SLP_TIMER_CFG_WEN));
}

/**
  * @brief  Set the Sleep Timer Work Mode.
  *
  *  Register|BitsName
  *  --------|--------
  *  CFG_0   | MODE
  *  CFG_0   | COUNT_MODE
  *
  * @param  SLP_TIMERx SLEEP TIMER instance
  * @param  mode This parameter can be one of the following values:
  *         @arg @ref LL_SLEEP_TIMER_SINGLE_MODE_0
  *         @arg @ref LL_SLEEP_TIMER_SINGLE_MODE_1
  *         @arg @ref LL_SLEEP_TIMER_AUTO_MODE
  * @retval None
  */
__STATIC_INLINE void ll_sleep_timer_set_mode(slp_timer_regs_t *SLP_TIMERx, uint32_t mode)
{
    MODIFY_REG(SLP_TIMERx->CFG, (SLP_TIMER_CFG_MODE | SLP_TIMER_CFG_COUNT_MODE), (mode | SLP_TIMER_CFG_WEN));
}

/**
  * @brief  Get the Sleep Timer Work Mode.
  *
  *  Register|BitsName
  *  --------|--------
  *  CFG_0   | MODE
  *  CFG_0   | COUNT_MODE
  *
  * @param  SLP_TIMERx SLEEP TIMER instance
  * @retval Returned Value can be one of the following values:
  *         @arg @ref LL_SLEEP_TIMER_SINGLE_MODE_0
  *         @arg @ref LL_SLEEP_TIMER_SINGLE_MODE_1
  *         @arg @ref LL_SLEEP_TIMER_AUTO_MODE
  */
__STATIC_INLINE uint32_t ll_sleep_timer_get_mode(slp_timer_regs_t *SLP_TIMERx)
{
    return (uint32_t)(READ_BITS(SLP_TIMERx->CFG, (SLP_TIMER_CFG_MODE | SLP_TIMER_CFG_COUNT_MODE)));
}

/**
  * @brief  check the sleep timer runing state.
  *
  *  Register|BitsName
  *  --------|--------
  *  STAT    | RUNNING
  *
  * @param  SLP_TIMERx SLEEP TIMER instance
  * @retval runing state of sleep timer (1 or 0).
  */
__STATIC_INLINE uint32_t ll_sleep_timer_is_running(slp_timer_regs_t *SLP_TIMERx)
{
    return (uint32_t)(READ_BITS(SLP_TIMERx->STAT, SLP_TIMER_STAT_RUNNING) == SLP_TIMER_STAT_RUNNING);
}

/**
  * @brief  check the sleep timer busy state.
  *
  *  Register|BitsName
  *  --------|--------
  *  STAT    | BUSY
  *
  * @param  SLP_TIMERx SLEEP TIMER instance
  * @retval busy state of sleep timer (1 or 0).
  */
__STATIC_INLINE uint32_t ll_sleep_timer_is_busy(slp_timer_regs_t *SLP_TIMERx)
{
    return (uint32_t)(READ_BITS(SLP_TIMERx->STAT, SLP_TIMER_STAT_BUSY) == SLP_TIMER_STAT_BUSY);
}

/**
  * @brief  check the sleep timer busy state.
  *
  *  Register|BitsName
  *  --------|--------
  *  STAT    | COUNTING
  *
  * @param  SLP_TIMERx SLEEP TIMER instance
  * @retval busy state of sleep timer (1 or 0).
  */
__STATIC_INLINE uint32_t ll_sleep_timer_is_counting(slp_timer_regs_t *SLP_TIMERx)
{
    return (uint32_t)(READ_BITS(SLP_TIMERx->STAT, SLP_TIMER_STAT_COUNTING) == SLP_TIMER_STAT_COUNTING);
}

/**
  * @brief  Set the 32 bits reload value to register, which not real internal value.
  *
  *  Register|BitsName
  *  --------|--------
  *  TIMER_W | VAL_SET
  *
  * @param  SLP_TIMERx SLEEP TIMER instance
  * @param  value  32 bits count value loaded into the 32bit_timer
  * @retval None
  */
__STATIC_INLINE void ll_sleep_timer_set_counter_value(slp_timer_regs_t *SLP_TIMERx, uint32_t value)
{
    WRITE_REG(SLP_TIMERx->TIMER_W, value);
}

/**
  * @brief  Set the 32 bits reload value to register, which not real internal value.
  *
  *  Register|BitsName
  *  --------|--------
  *  TIMER_W | VAL_SET
  *
  * @param  SLP_TIMERx SLEEP TIMER instance
  * @retval None
  */
__STATIC_INLINE uint32_t ll_sleep_timer_get_counter_value(slp_timer_regs_t *SLP_TIMERx)
{
    return (uint32_t)READ_REG(SLP_TIMERx->TIMER_W);
}

/**
  * @brief  Get the current count value.

  *
  *  Register|BitsName
  *  --------|--------
  *  TIMER_R | VAL_READ
  *
  * @param  SLP_TIMERx SLEEP TIMER instance
  * @retval 32 bit SLEEP Timer Count Value
  */
__STATIC_INLINE uint32_t ll_sleep_timer_get_counter_int_value(slp_timer_regs_t *SLP_TIMERx)
{
    return (uint32_t)READ_REG(SLP_TIMERx->TIMER_R);
}

/**
  * @brief  Clear the sleep timer interrupt status flag.
  *
  *  Register|BitsName
  *  --------|--------
  *  INTSTAT | INTSTAT
  *
  * @retval None
  */
__STATIC_INLINE void ll_sleep_timer_clear_flag_it(void)
{
	WRITE_REG(AON_CTL->AON_SLP_EVENT, ~(AON_CTL_SLP_EVENT_SLP_TIMER));
}

/** @} */

/** @} */

#endif /* SLP_TIMER */

#ifdef __cplusplus
}
#endif

#endif /* ___LL_SLEEP_TIMER_H__ */

/** @} */

/** @} */

/** @} */

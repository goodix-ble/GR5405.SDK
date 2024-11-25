/**
  ****************************************************************************************
  * @file    hal_gpio.c
  * @author  BLE Driver Team
  * @brief   GPIO HAL module driver.
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
#include "gr5x.h"

/** @addtogroup HAL_DRIVER
  * @{
  */

#ifdef HAL_MSIO_MODULE_ENABLED
#include "hal_msio.h"
#endif

#ifdef HAL_AON_GPIO_MODULE_ENABLED
#include "hal_aon_gpio.h"
#include "ll_pwr.h"
#endif

#ifdef HAL_GPIO_MODULE_ENABLED
#include "ll_gpio.h"
#include "hal_gpio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/** @defgroup GPIO_Private_Defines GPIO Private Defines
  * @{
  */
#define GPIO_MODE                 (0x00000003U)
#define GPIO_MODE_IT_Pos          (4U)

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @defgroup GPIO_Private_Macros GPIO Private Macros
  * @{
  */
/**
  * @}
  */
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup GPIO_Exported_Functions GPIO Exported Functions
  * @{
  */

/** @defgroup GPIO_Exported_Functions_Group1 Initialization/de-initialization functions
 *  @brief    Initialization and Configuration functions
  * @{
  */
void ll_gpio_init(gpio_regs_t *GPIOx, const ll_gpio_init_t p_gpio_init)
{
    uint32_t current_pin = 0x00000000U;

    /* Check the parameters */
    gr_assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));
    gr_assert_param(IS_LL_GPIO_PIN(p_gpio_init->pin));
    gr_assert_param(IS_LL_GPIO_MODE(p_gpio_init->mode));
    gr_assert_param(IS_LL_GPIO_PULL(p_gpio_init->pull));
    gr_assert_param(IS_LL_GPIO_MUX(p_gpio_init->mux));
    gr_assert_param(IS_LL_GPIO_TRIGGER(p_gpio_init->trigger));

    /* ------------------------- Configure the port pins ---------------- */
    uint32_t pin_tmp = p_gpio_init.pin;
    while (pin_tmp != (uint32_t)0x0U)
    {
        //lint -e9034 __RBIT will not assigned to a narrower or different essential type.
        //lint -e732 No loss of sign (arg. no. 1) (int to unsigned int).
        //lint -e718 -e746 -e9027 -e526 -e628 __clz(count leading zeros) is ARM function.
        current_pin = (uint32_t)(0x1UL << POSITION_VAL(pin_tmp));
        /* Clear the lowest bit 1 */
        pin_tmp &= (pin_tmp - 1U);
        ll_gpio_set_pin_mux(GPIOx, current_pin, p_gpio_init.mux);
    }

    ll_gpio_set_pin_mode(GPIOx, p_gpio_init.pin, p_gpio_init.mode);
    ll_gpio_set_pin_speed(GPIOx, p_gpio_init.pin, p_gpio_init.speed);
    ll_gpio_set_pin_strength(GPIOx, p_gpio_init.pin, p_gpio_init.strength);
    ll_gpio_set_pin_input_type(GPIOx, p_gpio_init.pin, p_gpio_init.input_type);
    ll_gpio_set_pin_pull(GPIOx, p_gpio_init.pin, p_gpio_init.pull);

    ll_gpio_disable_it(GPIOx, p_gpio_init.pin);
    if (LL_GPIO_MODE_INPUT == p_gpio_init.mode)
    {
        ll_gpio_clear_flag_it(GPIOx, p_gpio_init.pin);
        switch (p_gpio_init.trigger)
        {
        case LL_GPIO_TRIGGER_FALLING:
            ll_gpio_enable_falling_trigger(GPIOx, p_gpio_init.pin);
            break;

        case LL_GPIO_TRIGGER_RISING:
            ll_gpio_enable_rising_trigger(GPIOx, p_gpio_init.pin);
            break;

        case LL_GPIO_TRIGGER_HIGH:
            ll_gpio_enable_high_trigger(GPIOx, p_gpio_init.pin);
            break;

        case LL_GPIO_TRIGGER_LOW:
            ll_gpio_enable_low_trigger(GPIOx, p_gpio_init.pin);
            break;

        case LL_GPIO_TRIGGER_BOTH_EDGE:
            ll_gpio_enable_both_edge_trigger(GPIOx, p_gpio_init.pin);
            break;

        default:
            /* Nothing to do */
            break;
        }

        if(LL_GPIO_TRIGGER_NONE != p_gpio_init.trigger)
        {
            ll_gpio_enable_it(GPIOx, p_gpio_init.pin);
        }
    }
}

void hal_gpio_init(gpio_regs_t *GPIOx, const gpio_init_t *p_gpio_init)
{
    /* Check the parameters */
    gr_assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));
    gr_assert_param(IS_GPIO_PIN(p_gpio_init->pin));
    gr_assert_param(IS_GPIO_MODE(p_gpio_init->mode));

    ll_gpio_init_t init;

    init.pin = p_gpio_init->pin;
    init.mode = p_gpio_init->mode & GPIO_MODE;
    init.pull = p_gpio_init->pull;
    init.mux = p_gpio_init->mux;
    init.speed = p_gpio_init->speed;
    init.strength = p_gpio_init->strength;
    init.input_type = p_gpio_init->input_type;

    init.trigger = (p_gpio_init->mode >> GPIO_MODE_IT_Pos);

    ll_gpio_init(GPIOx, init);
}

void hal_gpio_deinit(gpio_regs_t *GPIOx, uint32_t gpio_pin)
{
    uint32_t current_pin = 0x00000000U;

    /* Check the parameters */
    gr_assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));

    /* Data output register set to default reset values */
    ll_gpio_reset_output_pin(GPIOx, gpio_pin);
    /* Output enable register set to default reset values */
    ll_gpio_set_pin_mode(GPIOx, gpio_pin, GPIO_MODE_INPUT);
    /* Disable Interrupt */
    ll_gpio_disable_it(GPIOx, gpio_pin);
    /* Interrupt status clear*/
    __HAL_GPIO_IT_CLEAR_IT(GPIOx, gpio_pin);

    ll_gpio_set_pin_pull(GPIOx, gpio_pin, GPIO_PULLDOWN);
    ll_gpio_set_pin_speed(GPIOx, gpio_pin, GPIO_SPEED_MEDIUM);
    ll_gpio_set_pin_strength(GPIOx, gpio_pin, GPIO_STRENGTH_MEDIUM);
    ll_gpio_set_pin_input_type(GPIOx, gpio_pin, GPIO_INPUT_TYPE_CMOS);

    uint32_t pin_tmp = gpio_pin;
    while (pin_tmp != (uint32_t)0x0)
    {
        //lint -e9034 __RBIT will not assigned to a narrower or different essential type.
        //lint -e732 No loss of sign (arg. no. 1) (int to unsigned int).
        current_pin = (0x1UL << POSITION_VAL(pin_tmp));
        /* Clear the lowest bit 1 */
        pin_tmp &= (pin_tmp - 1U);

        ll_gpio_set_pin_mux(GPIOx, current_pin, IO_MUX_GPIO);
    }
}

/**
  * @}
  */

/** @defgroup GPIO_Exported_Functions_Group2 IO operation functions
 *  @brief GPIO Read, Write, Toggle, Lock and EXTI management functions.
  * @{
  */

gpio_pin_state_t hal_gpio_read_pin(gpio_regs_t *GPIOx, uint16_t gpio_pin)
{
    /* Check the parameters */
    gr_assert_param(IS_GPIO_PIN(gpio_pin));

    gpio_pin_state_t ret;

    if(ll_gpio_read_input_pin(GPIOx, gpio_pin) != 0U)
    {
        ret = GPIO_PIN_SET;
    }
    else
    {
        ret = GPIO_PIN_RESET;
    }
    return ret;
}

void hal_gpio_write_pin(gpio_regs_t* GPIOx, uint16_t gpio_pin, gpio_pin_state_t pin_state)
{
    /* Check the parameters */
    gr_assert_param(IS_GPIO_PIN(gpio_pin));
    gr_assert_param(IS_GPIO_PIN_ACTION(pin_state));

    if (GPIO_PIN_RESET != pin_state)
    {
        ll_gpio_set_output_pin(GPIOx, gpio_pin);
    }
    else
    {
        ll_gpio_reset_output_pin(GPIOx, gpio_pin);
    }

}

void hal_gpio_toggle_pin(gpio_regs_t *GPIOx, uint16_t gpio_pin)
{
    /* Check the parameters */
    gr_assert_param(IS_GPIO_PIN(gpio_pin));

    ll_gpio_toggle_pin(GPIOx, gpio_pin);
}

__WEAK void hal_gpio_exti_irq_handler(gpio_regs_t *GPIOx)
{
    uint16_t Triggered_Pin = (uint16_t)__HAL_GPIO_IT_GET_IT(GPIOx, GPIO_PIN_ALL);
    __HAL_GPIO_IT_CLEAR_IT(GPIOx, Triggered_Pin);
    /* GPIO pin interrupt detected */
    if (Triggered_Pin != (uint16_t)0x0)
    {
        hal_gpio_exti_callback(GPIOx, Triggered_Pin);
    }

#ifdef HAL_LOW_POWER_DEBUG_ENABLED
    if (ll_gpio_is_active_swd_flag_it(GPIOx) != 0U)
    {
        ll_gpio_clear_swd_flag_it(GPIOx);
    }
#endif
}

__WEAK void hal_gpio_exti_callback(gpio_regs_t *GPIOx, uint16_t gpio_pin)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(GPIOx);
    UNUSED(gpio_pin);
}

/**
  * @}
  */


/**
  * @}
  */

#endif /* HAL_GPIO_MODULE_ENABLED */

#ifdef HAL_AON_GPIO_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/** @defgroup AON_GPIO_Private_Defines AON_GPIO Private Defines
  * @{
  */
#define AON_GPIO_MODE             (0x00000003U)
#define AON_GPIO_MODE_IT_Pos      (4U)
#define AON_GPIO_DEFAULT_PIN_MUX    IO_MUX_GPIO
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @defgroup AON_GPIO_Private_Macros AON_GPIO Private Macros
  * @{
  */
/**
  * @}
  */
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup AON_GPIO_Exported_Functions AON_GPIO Exported Functions
  * @{
  */

/** @defgroup AON_GPIO_Exported_Functions_Group1 Initialization/de-initialization functions
  * @{
  */
void ll_aon_gpio_init(ll_aon_gpio_init_t const p_aon_gpio_init)
{
    uint32_t current_pin = 0x00000000U;

    /* Check the parameters */
    gr_assert_param(IS_LL_AON_GPIO_PIN(p_aon_gpio_init.pin));
    gr_assert_param(IS_LL_AON_GPIO_MODE(p_aon_gpio_init.mode));
    gr_assert_param(IS_LL_AON_GPIO_PULL(p_aon_gpio_init.pull));
    gr_assert_param(IS_LL_AON_GPIO_MUX(p_aon_gpio_init.mux));
    gr_assert_param(IS_LL_AON_GPIO_TRIGGER(p_aon_gpio_init.trigger));

    /* ------------------------- Configure the port pins ---------------- */
    uint32_t pin_tmp = p_aon_gpio_init.pin;
    while (pin_tmp != (uint32_t)0x0)
    {
        current_pin = (0x1UL << POSITION_VAL(pin_tmp));
        /* Clear the lowest bit 1 */
        pin_tmp &= (pin_tmp - 1U);
        ll_aon_gpio_set_pin_mux(current_pin, p_aon_gpio_init.mux);
    }

    ll_aon_gpio_set_pin_speed(p_aon_gpio_init.pin, p_aon_gpio_init.speed);
    ll_aon_gpio_set_pin_strength(p_aon_gpio_init.pin, p_aon_gpio_init.strength);
    ll_aon_gpio_set_pin_input_type(p_aon_gpio_init.pin, p_aon_gpio_init.input_type);

    ll_aon_gpio_set_pin_mode(p_aon_gpio_init.pin, p_aon_gpio_init.mode);

    ll_aon_gpio_set_pin_pull(p_aon_gpio_init.pin, p_aon_gpio_init.pull);

    ll_aon_gpio_disable_it(p_aon_gpio_init.pin);
    if (LL_AON_GPIO_MODE_INPUT == p_aon_gpio_init.mode)
    {
        switch (p_aon_gpio_init.trigger)
        {
        case LL_AON_GPIO_TRIGGER_FALLING:
            ll_aon_gpio_enable_falling_trigger(p_aon_gpio_init.pin);
            break;

        case LL_AON_GPIO_TRIGGER_RISING:
            ll_aon_gpio_enable_rising_trigger(p_aon_gpio_init.pin);
            break;

        case LL_AON_GPIO_TRIGGER_HIGH:
            ll_aon_gpio_enable_high_trigger(p_aon_gpio_init.pin);
            break;

        case LL_AON_GPIO_TRIGGER_LOW:
            ll_aon_gpio_enable_low_trigger(p_aon_gpio_init.pin);
            break;

        case LL_AON_GPIO_TRIGGER_BOTH_EDGE:
            ll_aon_gpio_enable_both_trigger(p_aon_gpio_init.pin);
            break;

        default:
            /* Nothing to do */
            break;
        }

        ll_aon_gpio_clear_flag_it(p_aon_gpio_init.pin);

        if (LL_AON_GPIO_TRIGGER_NONE != p_aon_gpio_init.trigger)
        {
            ll_aon_gpio_enable_it(p_aon_gpio_init.pin);
        }
    }
}

void hal_aon_gpio_init(const aon_gpio_init_t *p_aon_gpio_init)
{
    /* Check the parameters */
    gr_assert_param(IS_AON_GPIO_PIN(p_aon_gpio_init->pin));
    gr_assert_param(IS_AON_GPIO_MODE(p_aon_gpio_init->mode));
    gr_assert_param(IS_AON_GPIO_PULL(p_aon_gpio_init->pull));

    ll_aon_gpio_init_t aon_gpio_init;

    aon_gpio_init.pin          = p_aon_gpio_init->pin;
    aon_gpio_init.mode         = p_aon_gpio_init->mode & AON_GPIO_MODE;
    aon_gpio_init.pull         = p_aon_gpio_init->pull;
    aon_gpio_init.mux          = p_aon_gpio_init->mux;
    aon_gpio_init.speed        = p_aon_gpio_init->speed;
    aon_gpio_init.strength     = p_aon_gpio_init->strength;
    aon_gpio_init.input_type   = p_aon_gpio_init->input_type;
    aon_gpio_init.trigger = p_aon_gpio_init->mode >> AON_GPIO_MODE_IT_Pos;

    ll_aon_gpio_init(aon_gpio_init);
}

void hal_aon_gpio_deinit(uint32_t aon_gpio_pin)
{
    uint32_t current_pin = 0x00000000U;

    /* Data output register set to default reset values */
    ll_aon_gpio_reset_output_pin(aon_gpio_pin);
    /* Output enable register set to default reset values */
    ll_aon_gpio_set_pin_mode(aon_gpio_pin, AON_GPIO_MODE_INPUT);
    /* Disable Interrupt */
    ll_aon_gpio_disable_it(aon_gpio_pin);
    /* Interrupt status clear*/
    __HAL_AON_GPIO_IT_CLEAR_IT(aon_gpio_pin);

    ll_aon_gpio_set_pin_pull(aon_gpio_pin, AON_GPIO_PULLDOWN);

    ll_aon_gpio_set_pin_speed(aon_gpio_pin, AON_GPIO_SPEED_MEDIUM);
    ll_aon_gpio_set_pin_strength(aon_gpio_pin, AON_GPIO_SPEED_MEDIUM);
    ll_aon_gpio_set_pin_input_type(aon_gpio_pin, AON_GPIO_INPUT_TYPE_CMOS);

    uint32_t pin_tmp = aon_gpio_pin;
    while (pin_tmp != (uint32_t)0x0)
    {
        current_pin = (0x1UL << POSITION_VAL(pin_tmp));
        /* Clear the lowest bit 1 */
        pin_tmp &= (pin_tmp - 1U);

        ll_aon_gpio_set_pin_mux(current_pin, AON_GPIO_DEFAULT_PIN_MUX);
    }
}

/**
  * @}
  */

/** @defgroup AON_GPIO_Exported_Functions_Group2 IO operation functions
  * @{
  */
aon_gpio_pin_state_t hal_aon_gpio_read_pin(uint16_t aon_gpio_pin)
{
    /* Check the parameters */
    gr_assert_param(IS_AON_GPIO_PIN(aon_gpio_pin));

    aon_gpio_pin_state_t ret;

    if(ll_aon_gpio_read_input_pin(aon_gpio_pin) != 0U)
    {
        ret = AON_GPIO_PIN_SET;
    }
    else
    {
        ret = AON_GPIO_PIN_RESET;
    }
    return ret;
}

void hal_aon_gpio_write_pin(uint16_t aon_gpio_pin, aon_gpio_pin_state_t pin_state)
{
    /* Check the parameters */
    gr_assert_param(IS_AON_GPIO_PIN(aon_gpio_pin));
    gr_assert_param(IS_AON_GPIO_PIN_ACTION(pin_state));

    if (AON_GPIO_PIN_RESET != pin_state)
    {
        ll_aon_gpio_set_output_pin(aon_gpio_pin);
    }
    else
    {
        ll_aon_gpio_reset_output_pin(aon_gpio_pin);
    }

}

void hal_aon_gpio_toggle_pin(uint16_t aon_gpio_pin)
{
    /* Check the parameters */
    gr_assert_param(IS_AON_GPIO_PIN(aon_gpio_pin));

    ll_aon_gpio_toggle_pin(aon_gpio_pin);
}

__WEAK void hal_aon_gpio_irq_handler(void)
{
    uint16_t Triggered_Pin = __HAL_AON_GPIO_IT_GET_IT(AON_GPIO_PIN_ALL);
    uint16_t event =  ll_aon_gpio_read_event_flag_it();

    /* AON_GPIO pin interrupt detected */
    if (Triggered_Pin != (uint16_t)0x0)
    {
        __HAL_AON_GPIO_IT_CLEAR_IT(Triggered_Pin);

        /* Clear external wakeup status */
        ll_pwr_clear_ext_wakeup_status(Triggered_Pin);

        /* Clear external wakeup event */
        ll_aon_gpio_clear_event_flag_it();

        hal_aon_gpio_callback(Triggered_Pin);

    }

    if ((event != (uint16_t)0x0) && (0U == Triggered_Pin))
    {
        /* Clear external wakeup event */
        ll_aon_gpio_clear_event_flag_it();
    }
}

__WEAK void hal_aon_gpio_callback(uint16_t aon_gpio_pin)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(aon_gpio_pin);
}

/**
  * @}
  */


/**
  * @}
  */

#endif /* HAL_AON_GPIO_MODULE_ENABLED */


#ifdef HAL_MSIO_MODULE_ENABLED

#define MSIO_MODE             (0x00000003U)

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @defgroup MSIO_Private_Macros MSIO Private Macros
  * @{
  */
/**
  * @}
  */
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup MSIO_Exported_Functions MSIO Exported Functions
  * @{
  */

/** @defgroup MSIO_Exported_Functions_Group1 Initialization/de-initialization functions
  * @{
  */
void ll_msio_init(msio_pad_t MSIOx, const ll_msio_init_t *p_msio_init)
{
    uint32_t current_pin  = 0x00000000U;

    /* Check the parameters */
    gr_assert_param(IS_LL_MSIO_PIN(p_msio_init->pin));
    gr_assert_param(IS_LL_MSIO_DIRECTION(p_msio_init->direction));
    gr_assert_param(IS_LL_MSIO_MODE(p_msio_init->mode));
    gr_assert_param(IS_LL_MSIO_PULL(p_msio_init->pull));
    gr_assert_param(IS_LL_MSIO_MUX(p_msio_init->mux));

    /* ------------------------- Configure the port pins ---------------- */
    uint32_t pin_tmp = p_msio_init->pin;
    while (pin_tmp != (uint32_t)0x0)
    {
        current_pin = (0x1UL << POSITION_VAL(pin_tmp));
        /* Clear the lowest bit 1 */
        pin_tmp &= (pin_tmp - 1U);
        ll_msio_set_pin_mux(MSIOx, current_pin, p_msio_init->mux);
    }

    ll_msio_set_pin_speed(MSIOx,p_msio_init->pin, p_msio_init->speed);
    ll_msio_set_pin_strength(MSIOx,p_msio_init->pin, p_msio_init->strength);
    ll_msio_set_pin_input_type(MSIOx,p_msio_init->pin, p_msio_init->input_type);
    ll_msio_set_pin_mode(MSIOx,p_msio_init->pin, (p_msio_init->mode & MSIO_MODE));
    ll_msio_set_pin_direction(MSIOx,p_msio_init->pin, p_msio_init->direction);
    ll_msio_set_pin_pull(MSIOx, p_msio_init->pin, p_msio_init->pull);
}

void hal_msio_init(msio_pad_t MSIOx, const msio_init_t *p_msio_init)
{
    /* Check the parameters */
    gr_assert_param(IS_MSIO_PIN(p_msio_init->pin));
    gr_assert_param(IS_MSIO_DIRECTION(p_msio_init->direction));
    gr_assert_param(IS_MSIO_MODE(p_msio_init->mode));

    /* ------------------------- Configure the port pins ---------------- */
    ll_msio_init(MSIOx, p_msio_init);
}

void hal_msio_deinit(msio_pad_t MSIOx, uint32_t msio_pin)
{
    uint32_t current_pin = 0x00000000U;

    /* Output enable register set to default reset values */
    ll_msio_set_pin_direction(MSIOx, msio_pin, MSIO_DIRECTION_INPUT);
    ll_msio_set_pin_mode(MSIOx, msio_pin, MSIO_MODE_ANALOG);
    ll_msio_set_pin_pull(MSIOx, msio_pin, MSIO_PULLDOWN);
    ll_msio_set_pin_speed(MSIOx,msio_pin, MSIO_SPEED_MEDIUM);
    ll_msio_set_pin_strength(MSIOx,msio_pin, MSIO_STRENGTH_MEDIUM);
    ll_msio_set_pin_input_type(MSIOx,msio_pin, MSIO_INPUT_TYPE_CMOS);

    uint32_t pin_tmp = msio_pin;
    while (pin_tmp != (uint32_t)0x0)
    {
        current_pin = (0x1UL << POSITION_VAL(pin_tmp));
        /* Clear the lowest bit 1 */
        pin_tmp &= (pin_tmp - 1U);

        ll_msio_set_pin_mux(MSIOx, current_pin, IO_MUX_GPIO);
    }
}

/**
  * @}
  */

/** @defgroup MSIO_Exported_Functions_Group2 IO operation functions
  * @{
  */

msio_pin_state_t hal_msio_read_pin(msio_pad_t MSIOx, uint16_t msio_pin)
{
    /* Check the parameters */
    gr_assert_param(IS_MSIO_PIN(msio_pin));

    msio_pin_state_t ret;

    if(ll_msio_read_input_pin(MSIOx, msio_pin) != 0U)
    {
        ret = MSIO_PIN_SET;
    }
    else
    {
        ret = MSIO_PIN_RESET;
    }
    return ret;
}

void hal_msio_write_pin(msio_pad_t MSIOx, uint16_t msio_pin, msio_pin_state_t pin_state)
{
    /* Check the parameters */
    gr_assert_param(IS_MSIO_PIN(msio_pin));
    gr_assert_param(IS_MSIO_PIN_ACTION(pin_state));

    if (MSIO_PIN_RESET != pin_state)
    {
        ll_msio_set_output_pin(MSIOx, msio_pin);
    }
    else
    {
        ll_msio_reset_output_pin(MSIOx, msio_pin);
    }
}

void hal_msio_toggle_pin(msio_pad_t MSIOx, uint16_t msio_pin)
{
    /* Check the parameters */
    gr_assert_param(IS_MSIO_PIN(msio_pin));

    ll_msio_toggle_pin(MSIOx, msio_pin);
}

/**
  * @}
  */


/**
  * @}
  */

#endif /* HAL_MSIO_MODULE_ENABLED */
/**
  * @}
  */

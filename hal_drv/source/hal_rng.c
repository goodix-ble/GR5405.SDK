/**
  ****************************************************************************************
  * @file    hal_rng.c
  * @author  BLE Driver Team
  * @brief   RNG HAL module driver.
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

#ifdef HAL_RNG_MODULE_ENABLED

#include "hal_pwr_mgmt.h"
#include "hal_rng.h"

#ifdef HAL_CLOCK_MODULE
#include "hal_cgc.h"
#endif
#ifdef HAL_CLOCK_UNIFORM_CONTROL
#include "hal_clock.h"
#endif

/** @addtogroup HAL_DRIVER
  * @{
  */



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup RNG_Private_Constants RNG_Private_Constants
  * @{
  */
#define RNG_TIMEOUT_VALUE     200UL
/** @} */

/* Private macro -------------------------------------------------------------*/
#define LL_RNG_CONFIG_LFSR_XOR_FRO        (0x4UL << RNG_CONFIG_LFSR_XOR_SEL_POS)
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void rng_set_device_state(rng_handle_t *p_rng ,hal_rng_state_t state);


/* Exported functions --------------------------------------------------------*/

/** @defgroup RNG_Exported_Functions RNG Exported Functions
  * @{
  */

/**
  * @brief  De-initialize RNG registers (Registers restored to their default values).
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: RNG registers are de-initialized
  *          - ERROR: RNG registers are not de-initialized
  */
void ll_rng_deinit(rng_regs_t *RNGx)
{
    /* Disable the selected RNGx Peripheral */
    ll_rng_disable(RNGx);

    /* Clear status flag. */
    ll_rng_clear_flag_sts(RNGx);

    /* CONFIG register set to default reset values. */
    LL_RNG_WriteReg(RNGx, CONFIG, 0x9004);

    /* TSCON register set to default reset values. */
    LL_RNG_WriteReg(RNGx, TSCON, 0x00007864U);

    /* TSCON register set to default reset values. */
    LL_RNG_WriteReg(RNGx, FROCFG, 0x0000FFFFU);

    /* USER_SEED register set to default reset values. */
    LL_RNG_WriteReg(RNGx, USER_SEED, 0x00000000U);
}

/**
  * @brief  Initialize the RNG registers according to the specified parameters in p_rng_init.
  * @param  RNGx RNG instance.
  * @param  p_rng_init pointer to a @ref ll_rng_init_t structure.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RNG registers are initialized
  *          - ERROR: Not applicable
  */
void ll_rng_init(rng_regs_t *RNGx, const ll_rng_init_t *p_rng_init)
{
    uint32_t config_value;

//    if((LL_RNG_SEED_USER ==  p_rng_init->seed) && (LL_RNG_OUTPUT_FR0_S0 == p_rng_init->out_mode))
//    {
//        return;
//    }

    /* Note: rng registers are WRITE ONLY before enable rng module */
    /* Therefore we CAN NOT use expressions like "rng->config |= 8" to set 3rd bit */
    /* Disable the selected RNGx Peripheral */
    ll_rng_disable(RNGx);

    config_value = p_rng_init->post_mode | p_rng_init->out_mode | p_rng_init->lfsr_mode | \
                   p_rng_init->seed | p_rng_init->it | LL_RNG_CONFIG_LFSR_XOR_FRO;
    if(LL_RNG_SEED_FR0_S0 == p_rng_init->seed)
    {
        config_value |= RNG_CONFIG_FRO_EN;
    }
    else
    {
        config_value &= ~(RNG_CONFIG_FRO_EN);
    }


    /* set the value of p_rng_init to CONFIG register. */
    LL_RNG_WriteReg(RNGx, CONFIG, config_value);
}

/**
  * @brief  Set each @ref ll_rng_init_t field to default value.
  * @param  p_rng_init Pointer to a @ref ll_rng_init_t structure.
  * @retval None
  */
void ll_rng_struct_init(ll_rng_init_t *p_rng_init)
{
    /* Set RNG_InitStruct fields to default values */
    p_rng_init->seed       = LL_RNG_SEED_FR0_S0;
    p_rng_init->lfsr_mode  = LL_RNG_LFSR_MODE_59BIT;
    p_rng_init->out_mode   = LL_RNG_POST_PRO_NOT;
    p_rng_init->post_mode  = LL_RNG_OUTPUT_FR0_S0;
    p_rng_init->it         = LL_RNG_IT_DISABLE;
}

/** @defgroup RNG_Exported_Functions_Group1 Initialization and Configuration functions
 *  @brief    Initialization and Configuration functions.
  * @{
  */

__WEAK hal_status_t hal_rng_init(rng_handle_t *p_rng)
{
    /* Check the parameters */
    gr_assert_param(IS_RNG_ALL_INSTANCE(p_rng->p_instance));

    if (HAL_RNG_STATE_RESET == p_rng->state)
    {
#ifdef HAL_CLOCK_UNIFORM_CONTROL
       //lint -e923 Cast from pointer to unsigned int is necessary
       hal_clock_enable_module((uint32_t)p_rng->p_instance);
#endif
#ifdef HAL_CLOCK_SETTING_IN_DRV
        /* Enable security blocks clock and Automatic turn off security blocks clock during WFI. */
        ll_cgc_disable_force_off_secu_hclk();
        ll_cgc_disable_wfi_off_secu_hclk();

        ll_cgc_disable_force_off_rng_hclk();
        ll_cgc_disable_force_off_secu_div4_pclk();
        ll_cgc_disable_wfi_off_rng_hclk();
        ll_cgc_disable_wfi_off_secu_div4_hclk();
#endif
        /* init the low level hardware : CLOCK */
        hal_rng_msp_init(p_rng);
    }

    p_rng->state = HAL_RNG_STATE_BUSY;

    ll_rng_disable(p_rng->p_instance);

    /* Initialize the RNG state */
    rng_set_device_state(p_rng, HAL_RNG_STATE_READY);

    /* Return function status */
    return HAL_OK;
}

__WEAK hal_status_t hal_rng_deinit(rng_handle_t *p_rng)
{

#ifdef HAL_CLOCK_SETTING_IN_DRV
    /* Enable RNG Clock for operate the register. */
    ll_cgc_disable_force_off_secu_hclk();

    ll_cgc_disable_force_off_rng_hclk();
    ll_cgc_disable_force_off_secu_div4_pclk();
#endif
    /* Disable the RNG Peripheral */
    ll_rng_deinit(p_rng->p_instance);

    /* DeInit the low level hardware: CLOCK... */
    hal_rng_msp_deinit(p_rng);

#ifdef HAL_CLOCK_UNIFORM_CONTROL
    //lint -e923 Cast from pointer to unsigned int is necessary
    hal_clock_disable_module((uint32_t)p_rng->p_instance);
#endif
#ifdef HAL_CLOCK_SETTING_IN_DRV
    ll_cgc_enable_force_off_rng_hclk();
    ll_cgc_enable_wfi_off_rng_hclk();

    GLOBAL_EXCEPTION_DISABLE();
    if (ll_cgc_get_force_off_secu() == LL_CGC_MCU_SECU_FRC_OFF_HCLK)
    {
        ll_cgc_enable_force_off_secu_hclk();
        ll_cgc_enable_force_off_secu_div4_pclk();
    }
    if (ll_cgc_get_slp_off_secu() == LL_CGC_MCU_SECU_FRC_OFF_WFI_HCLK)
    {
        ll_cgc_enable_wfi_off_secu_hclk();
        ll_cgc_enable_wfi_off_secu_div4_hclk();
    }
    GLOBAL_EXCEPTION_ENABLE();
#endif
    /* Initialize the RNG state */
    rng_set_device_state(p_rng, HAL_RNG_STATE_RESET);

    return HAL_OK;
}

__WEAK void hal_rng_msp_init(rng_handle_t *p_rng)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_rng);

    /* NOTE: This function should not be modified, when the callback is needed,
        the hal_rng_msp_init could be implemented in the user file
    */
}

__WEAK void hal_rng_msp_deinit(rng_handle_t *p_rng)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_rng);

    /* NOTE: This function should not be modified, when the callback is needed,
        the hal_rng_msp_init could be implemented in the user file
    */
}

/** @} */

/** @defgroup RNG_Exported_Functions_Group2 Peripheral Control functions
 *  @brief    Peripheral Control functions
  * @{
  */

__WEAK hal_status_t hal_rng_generate_random_number(rng_handle_t *p_rng, const uint16_t *p_seed, uint32_t *p_random32bit)
{
    hal_status_t status = HAL_OK;
    ll_rng_init_t rng_init;

    /* Check RNS peripheral state */
    if(p_rng->state == HAL_RNG_STATE_READY)
    {
        /* Change RNG peripheral state */
        rng_set_device_state(p_rng, HAL_RNG_STATE_BUSY);

        rng_init.seed = p_rng->init.seed_mode;
        rng_init.lfsr_mode = p_rng->init.lfsr_mode;
        rng_init.out_mode = p_rng->init.out_mode;
        rng_init.post_mode = p_rng->init.post_mode;
        rng_init.it        = LL_RNG_IT_DISABLE;
        //lint -e934 Taking address of near auto variable is necessary
        ll_rng_init(p_rng->p_instance, &rng_init);

        if((RNG_SEED_USER == p_rng->init.seed_mode) && (NULL != p_seed))
        {
            if(RNG_LFSR_MODE_59BIT == p_rng->init.lfsr_mode)
            {
                ll_rng_set_user_seed(p_rng->p_instance, p_seed[0]);
                ll_rng_set_user_seed(p_rng->p_instance, p_seed[1]);
                ll_rng_set_user_seed(p_rng->p_instance, p_seed[2]);
                ll_rng_set_user_seed(p_rng->p_instance, p_seed[3]);
            }
            else
            {
                ll_rng_set_user_seed(p_rng->p_instance, p_seed[0]);
                ll_rng_set_user_seed(p_rng->p_instance, p_seed[1]);
                ll_rng_set_user_seed(p_rng->p_instance, p_seed[2]);
                ll_rng_set_user_seed(p_rng->p_instance, p_seed[3]);
                ll_rng_set_user_seed(p_rng->p_instance, p_seed[4]);
                ll_rng_set_user_seed(p_rng->p_instance, p_seed[5]);
                ll_rng_set_user_seed(p_rng->p_instance, p_seed[6]);
                ll_rng_set_user_seed(p_rng->p_instance, p_seed[7]);
            }
        }

        ll_rng_enable(p_rng->p_instance);

        //lint -e923 Cast from pointer to unsigned int is necessary
        HAL_TIMEOUT_INIT();
        uint32_t tickstart = HAL_TIMEOUT_GET_TICK();
        /* Update SystemCoreClock */
        SystemCoreUpdateClock();
        uint32_t timeout_counter = ((SystemCoreClock / 1000U) * RNG_TIMEOUT_VALUE);

        /* Check if data register contains valid random data */
        while( ll_rng_is_active_flag_sts(RNG) != RNG_STATUS_READY )
        {
            if ((HAL_TIMEOUT_GET_TICK() - tickstart) > timeout_counter)
            {
                rng_set_device_state(p_rng, HAL_RNG_STATE_ERROR);

                HAL_TIMEOUT_DEINIT();
                status = HAL_TIMEOUT;
                goto EXIT;
            }
        }

        HAL_TIMEOUT_DEINIT();

        /* Get a 32bit Random number */
        p_rng->random_number =  ll_rng_read_random_data32(RNG);
        *p_random32bit = p_rng->random_number;

        rng_set_device_state(p_rng, HAL_RNG_STATE_READY);
    }
    else
    {
        status = HAL_ERROR;
    }
EXIT:
    return status;
}

__WEAK hal_status_t hal_rng_generate_random_number_it(rng_handle_t *p_rng, const uint16_t *p_seed)
{
    hal_status_t status = HAL_OK;
    ll_rng_init_t rng_init;

    /* Check RNG peripheral state */
    if(p_rng->state == HAL_RNG_STATE_READY)
    {
        /* Change RNG peripheral state */
        rng_set_device_state(p_rng, HAL_RNG_STATE_BUSY);

        rng_init.seed = p_rng->init.seed_mode;
        rng_init.lfsr_mode = p_rng->init.lfsr_mode;
        rng_init.out_mode = p_rng->init.out_mode;
        rng_init.post_mode = p_rng->init.post_mode;
        rng_init.it        = LL_RNG_IT_ENABLE;

        ll_rng_init(p_rng->p_instance, &rng_init);

        if((RNG_SEED_USER == p_rng->init.seed_mode) && (NULL != p_seed))
        {
            if(RNG_LFSR_MODE_59BIT == p_rng->init.lfsr_mode)
            {
                ll_rng_set_user_seed(p_rng->p_instance, p_seed[0]);
                ll_rng_set_user_seed(p_rng->p_instance, p_seed[1]);
                ll_rng_set_user_seed(p_rng->p_instance, p_seed[2]);
                ll_rng_set_user_seed(p_rng->p_instance, p_seed[3]);
            }
            else
            {
                ll_rng_set_user_seed(p_rng->p_instance, p_seed[0]);
                ll_rng_set_user_seed(p_rng->p_instance, p_seed[1]);
                ll_rng_set_user_seed(p_rng->p_instance, p_seed[2]);
                ll_rng_set_user_seed(p_rng->p_instance, p_seed[3]);
                ll_rng_set_user_seed(p_rng->p_instance, p_seed[4]);
                ll_rng_set_user_seed(p_rng->p_instance, p_seed[5]);
                ll_rng_set_user_seed(p_rng->p_instance, p_seed[6]);
                ll_rng_set_user_seed(p_rng->p_instance, p_seed[7]);
            }
        }

        ll_rng_enable(p_rng->p_instance);
    }
    else
    {
        status = HAL_ERROR;
    }

  return status;
}

__WEAK uint32_t hal_rng_read_last_random_number(const rng_handle_t *p_rng)
{
      return(p_rng->random_number);
}

/** @} */

/** @addtogroup RNG_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callbacks functions
 * @{
 */
__WEAK void hal_rng_irq_handler(rng_handle_t *p_rng)
{
    if(ll_rng_is_active_flag_sts(p_rng->p_instance))
    {
        p_rng->random_number =  ll_rng_read_random_data32(RNG);
        ll_rng_disable(p_rng->p_instance);

        if(p_rng->state != HAL_RNG_STATE_ERROR)
        {
            /* Change RNG peripheral state */
            rng_set_device_state(p_rng, HAL_RNG_STATE_READY);

            /* Call legacy weak Data Ready callback */
            hal_rng_ready_data_callback(p_rng, p_rng->random_number);
        }
    }
}

__WEAK void hal_rng_ready_data_callback(rng_handle_t *p_rng, uint32_t random32bit)
{
    UNUSED(p_rng);
    UNUSED(random32bit);
    /* NOTE : This function should not be modified. When the callback is needed,
            function hal_rng_ready_data_callback must be implemented in the user file.
     */
}

/** @} */

/** @defgroup RNG_Exported_Functions_Group3 Peripheral State functions
  * @brief   RNG control functions
 * @{
 */
__WEAK hal_rng_state_t hal_rng_get_state(const rng_handle_t *p_rng)
{
    return p_rng->state;
}

__WEAK void hal_rng_suspend_reg(rng_handle_t *p_rng)
{
#ifndef HAL_HW_RES_SUSP_RNG
    const rng_regs_t *p_rng_regs = p_rng->p_instance;

    p_rng->retention[0] = READ_REG(p_rng_regs->CONFIG);
#endif
}

__WEAK void hal_rng_resume_reg(const rng_handle_t *p_rng)
{
#ifndef HAL_HW_RES_SUSP_RNG
    rng_regs_t *p_rng_regs = p_rng->p_instance;

//    CLEAR_BITS(p_rng_regs->CTRL, RNG_CTRL_RUN_EN);
    WRITE_REG(p_rng_regs->CONFIG, p_rng->retention[0]);
//    SET_BITS(p_rng_regs->CTRL, RNG_CTRL_RUN_EN);
#endif
}




#ifdef HAL_PM_ENABLE
__WEAK hal_pm_status_t hal_pm_rng_suspend(rng_handle_t *p_rng)
{
    hal_pm_status_t ret;
    hal_rng_state_t state;
    state = hal_rng_get_state(p_rng);
    if ((state != HAL_RNG_STATE_READY) && (state != HAL_RNG_STATE_RESET))
    {
        ret = HAL_PM_ACTIVE;
    }
    else
    {
        hal_rng_suspend_reg(p_rng);
        ret = HAL_PM_SLEEP;
    }
    return ret;
}

__WEAK void hal_pm_rng_resume(rng_handle_t *p_rng)
{
    hal_rng_resume_reg(p_rng);
}
#endif /* HAL_PM_ENABLE */

/**
 ****************************************************************************************
 * @brief  check the force off rng clk has been set or not
 ****************************************************************************************
 */
#ifdef HAL_CLOCK_MODULE
hal_status_t hal_is_rng_force_clk_enable(void)
{
    hal_status_t ret;
    if(ll_cgc_is_enabled_force_off_rng_hclk() == 0UL)
    {
        ret = HAL_OK;
    }
    else
    {
        ret = HAL_ERROR;
    }
    return ret;
}

/**
 ****************************************************************************************
 * @brief  check the rng wif clk has been set or not
 ****************************************************************************************
 */
hal_status_t hal_is_rng_wfi_clk_enable(void)
{
    hal_status_t ret;
    if(ll_cgc_is_enabled_wfi_off_rng_hclk() == 0UL)
    {
        ret = HAL_OK;
    }
    else
    {
        ret = HAL_ERROR;
    }
    return ret;
}

/**
 ****************************************************************************************
 * @brief  recover rng clk
 ****************************************************************************************
 */
hal_status_t hal_rng_clk_recover(uint8_t rng_force_clk_flag,uint8_t rng_wfi_clk_flag)
{
    if(rng_force_clk_flag == 1UL)
    {
        ll_cgc_disable_force_off_rng_hclk();
        ll_cgc_disable_force_off_secu_hclk();
        ll_cgc_disable_force_off_secu_div4_pclk();
    }
    if(rng_wfi_clk_flag == 1UL)
    {
        ll_cgc_disable_wfi_off_rng_hclk();
        ll_cgc_disable_wfi_off_secu_hclk();
        ll_cgc_disable_wfi_off_secu_div4_hclk();
    }
    return  HAL_OK ;
}
#endif

static void rng_set_device_state(rng_handle_t *p_rng ,hal_rng_state_t state)
{
    p_rng->state = state;
}
/** @} */

#endif /* HAL_RNG_MODULE_ENABLED */

/** @} */

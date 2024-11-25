/**
  ****************************************************************************************
  * @file    hal_bod.c
  * @author  BLE Driver Team
  * @brief   BOD HAL module driver.
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

#ifdef HAL_BOD_MODULE_ENABLED

#include "hal_bod.h"

/** @addtogroup BOD_LL
  * @{
  */

/** @} */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup BOD_LL_Exported_Functions
  * @{
  */

/** @addtogroup BOD_LL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize BOD registers (Registers restored to their default values).
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: BOD registers are de-initialized
  *          - ERROR:   Wrong BOD Port
  */
__WEAK void ll_bod_deinit(void)
{
    /*enable bod */
    ll_bod_bod_enable();

    /* disable bod2 */
    ll_bod_bod2_disable();

    /* set level 2 */
    ll_bod_bod2_lvl_ctrl_lv_set(LL_BOD_BOD2_LEVEL_2);

    /* bypass bod2 auto power function */
    ll_bod_bod2_auto_power_bypass_enable();

    /* enable bod and bod2 static function */
    ll_bod_static_enable();

    /* disable bod2 falling edge event */
    ll_bod_bod2_disable_fedge();

    /* disable bod2 rising edge event */
    ll_bod_bod2_disable_redge();
}

/**
  * @brief  Initialize BOD registers according to the specified parameters in bod_init_t.
  * @param  p_bod_init pointer to a @ref bod_init_t structure
  *         that contains the BODC_InitStructuration information for the BOD module.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: bod registers are initialized according to bod_init_t content
  *          - ERROR:   Not applicable
  */
__WEAK void ll_bod_init(const ll_bod_init_t *p_bod_init)
{
    /* Check the parameters */
    gr_assert_param(IS_LL_ADC_INPUT(ll_bod_init_t->bod2_lvl));
    /*enable or disable BOD*/
    if(p_bod_init->bod_en == LL_BOD_BOD_ENABLE)
    {
        ll_bod_bod_enable();
    }
    else
    {
        ll_bod_bod_disable();
    }
    /*enable or disable BOD2*/
    if(p_bod_init->bod2_en == LL_BOD_BOD2_ENABLE)
    {
        ll_bod_bod2_enable();
        ll_bod_bod2_lvl_ctrl_lv_set(p_bod_init->bod2_lvl);
        ll_bod_bod2_enable_fedge();
        ll_bod_bod2_enable_redge();
    }
    else
    {
        ll_bod_bod2_disable();
        ll_bod_bod2_lvl_ctrl_lv_set(p_bod_init->bod2_lvl);
        ll_bod_bod2_disable_fedge();
        ll_bod_bod2_disable_redge();

    }

    /*enable or disable static mode of BOD and BOD2*/
    if(p_bod_init->bod_static_en == LL_BOD_STATIC_ENABLE)
    {
        ll_bod_static_enable();
    }
    else
    {
        ll_bod_static_disable();
    }

    /*enable or disable static mode of BOD and BOD2*/
    if(p_bod_init->bod2_auto_power_bypass_en == LL_BOD_BOD2_AUTO_POWER_BYPASS_ENABLE)
    {
        ll_bod_bod2_auto_power_bypass_enable();
    }
    else
    {
        ll_bod_bod2_auto_power_bypass_disable();
    }
}

/**
  * @brief Set each @ref ll_bod_init_t field to default value.
  * @param p_bod_init pointer to a @ref ll_bod_init_t structure
  *                          whose fields will be set to default values.
  * @retval None
  */

__WEAK void ll_bod_struct_init(ll_bod_init_t *p_bod_init)
{
    /* Reset ADC init structure parameters values */
    p_bod_init->bod_en  = LL_BOD_BOD_ENABLE;
    p_bod_init->bod2_en  = LL_BOD_BOD2_ENABLE;
    p_bod_init->bod2_lvl = LL_BOD_BOD2_LEVEL_7;
    p_bod_init->bod_static_en = LL_BOD_STATIC_ENABLE;
    p_bod_init->bod2_auto_power_bypass_en = LL_BOD_BOD2_AUTO_POWER_BYPASS_ENABLE;
}

__WEAK hal_status_t hal_bod_init(const bod_handle_t *p_bod)
{
    hal_status_t   status = HAL_OK;

    /* Configure BOD peripheral */
    ll_bod_init(&p_bod->init);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_bod_deinit(const bod_handle_t *p_bod)
{
    UNUSED(p_bod);
    /* Reset ADC Peripheral */
    ll_bod_deinit();

    return HAL_OK;
}

__WEAK void hal_bod_msp_init(bod_handle_t *p_bod)
{
    UNUSED(p_bod);
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
              the hal_adc_msp_init can be implemented in the user file
     */
}

__WEAK void hal_bod_msp_deinit(bod_handle_t *p_bod)
{
    UNUSED(p_bod);
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
              the hal_adc_msp_deinit can be implemented in the user file
     */
}

__WEAK void hal_bod_fedge_callback(bod_handle_t *p_bod)
{
    UNUSED(p_bod);
    return;
}

__WEAK void hal_bod_redge_callback(bod_handle_t *p_bod)
{
    UNUSED(p_bod);
    return;
}

__WEAK void hal_bod_fedge_irq_handler(bod_handle_t *p_bod)
{
    ll_bod_bod2_clear_flag_redge();
    hal_bod_fedge_callback(p_bod);
}

__WEAK void hal_bod_redge_irq_handler(bod_handle_t *p_bod)
{
    ll_bod_bod2_clear_flag_fedge();
    hal_bod_redge_callback(p_bod);
}

__WEAK hal_status_t hal_bod_enable(bod_handle_t *p_bod, uint8_t enable)
{
    hal_status_t status = HAL_OK;
    UNUSED(p_bod);
    if(enable == HAL_BOD_ENABLE)
    {
        ll_bod_bod_enable();
    }
    else
    {
        ll_bod_bod_disable();
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_bod2_enable(bod_handle_t *p_bod, uint8_t enable)
{
    hal_status_t status = HAL_OK;
    UNUSED(p_bod);
    if(enable == HAL_BOD2_ENABLE)
    {
        ll_bod_bod2_enable();
    }
    else
    {
        ll_bod_bod2_disable();
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_bod2_set_level(bod_handle_t *p_bod, uint8_t level)
{
    UNUSED(p_bod);
    hal_status_t status = HAL_OK;

    ll_bod_bod2_lvl_ctrl_lv_set(level);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_bod_static_mode_enable(bod_handle_t *p_bod, uint8_t enable)
{
    hal_status_t status = HAL_OK;
    UNUSED(p_bod);
    if(enable == HAL_BOD_STATIC_ENABLE)
    {
        ll_bod_static_enable();
    }
    else
    {
        ll_bod_static_enable();
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_bod2_auto_power_bypass_enable(bod_handle_t *p_bod, uint8_t enable)
{
    hal_status_t status = HAL_OK;
    UNUSED(p_bod);
    if(enable == HAL_BOD2_AUTO_POWER_BYPASS_ENABLE)
    {
        ll_bod_bod2_auto_power_bypass_enable();
    }
    else
    {
        ll_bod_bod2_auto_power_bypass_disable();
    }
    /* Return function status */
    return status;
}

#endif /* HAL_BOD_MODULE_ENABLED */


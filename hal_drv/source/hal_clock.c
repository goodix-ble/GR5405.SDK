/**
  ****************************************************************************************
  * @file    hal_clock.c
  * @author  BLE Driver Team
  * @brief   CLOCK HAL module driver.
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
#include "ll_clk.h"
#include "hal_clock.h"
#include "hal_pwr.h"
#include "gr5xx_delay.h"
/** @addtogroup HAL_DRIVER
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Bit Map for xo requests from devices 1:request  0:no request */
static volatile uint32_t xo_clock_requests = 0;

/* Private function prototypes -----------------------------------------------*/
static void hal_clock_set_device_xo_request_status(xo_request_device_number_t dev_num)
{
    xo_clock_requests |= (1UL << (uint32_t)dev_num);
}

static void hal_clock_clear_device_xo_request_status(xo_request_device_number_t dev_num)
{
    xo_clock_requests &= ~(uint32_t)((1UL << (uint32_t)dev_num));
}

SECTION_RAM_CODE static void hal_clock_switch_sys_clk_to_xo_pll(void)/*lint !e618 MISRA exception. */
{
    // Ensure PLL ready
    if((ll_clk_get_hf_status() & LL_CLK_XO_PLL_PLL_STAT) != LL_CLK_XO_PLL_PLL_STAT)
    {
        // backup and minimize the xo offset for xo fast boot
        //lint -e923 Cast from pointer to unsigned int is necessary
        uint32_t tmp_xo_offset = (READ_BITS(AON_RF->RF9, AON_RF_RF9_XO_CAP_CTRL) >>  AON_RF_RF9_XO_CAP_CTRL_Pos);
        MODIFY_REG(AON_RF->RF9, AON_RF_RF9_XO_CAP_CTRL, (0x40UL << AON_RF_RF9_XO_CAP_CTRL_Pos));

        AON_PWR->XO_PLL_SET = (AON_PWR_XO_PLL_SET_PLL_SET | AON_PWR_XO_PLL_SET_XO_SET);
        while((ll_clk_get_hf_status() & LL_CLK_XO_PLL_PLL_STAT) != LL_CLK_XO_PLL_PLL_STAT)
        {
        }

        // restore the xo offset for ble tx rx and timing accuracy
        MODIFY_REG(AON_RF->RF9, AON_RF_RF9_XO_CAP_CTRL, (tmp_xo_offset << AON_RF_RF9_XO_CAP_CTRL_Pos));
        delay_us(10);
    }

    // Select XO as the system clock
    ll_clk_select_source(LL_CLK_SEL_SOURCE_CPLL_CLK);
    ll_pwr_turn_on_enable_xo_pll_after_dcdc_ready();
}

static void xo_16m_clock_switch_to_pll_16m(void)
{
    uint8_t sys_clk = (uint8_t)((AON_CTL->MCU_CLK_CTRL & AON_CTL_MCU_CLK_CTRL_SEL) >> AON_CTL_MCU_CLK_CTRL_SEL_Pos);
    if(2U == sys_clk) // sys_clk is xo_16m, shall switch to pll_16m
    {
        MODIFY_REG(AON_CTL->MCU_CLK_CTRL, AON_CTL_MCU_CLK_CTRL_SEL, (3UL << AON_CTL_MCU_CLK_CTRL_SEL_Pos));
    }
    uint8_t serial_clk = (uint8_t)((AON_CTL->MCU_CLK_CTRL & AON_CTL_MCU_CLK_CTRL_SER_CLK_SEL) >> AON_CTL_MCU_CLK_CTRL_SER_CLK_SEL_Pos);
    if(4U == serial_clk) // serial_clk is xo_16m, shall switch to pll_16m
    {
        MODIFY_REG(AON_CTL->MCU_CLK_CTRL, AON_CTL_MCU_CLK_CTRL_SER_CLK_SEL, (2UL << AON_CTL_MCU_CLK_CTRL_SER_CLK_SEL_Pos));
    }
    uint8_t flash_clk = (uint8_t)((AON_CTL->FLASH_CACHE_CTRL0 & AON_CTL_FLASH_CACHE_XF_SCK_CLK_SEL) >> AON_CTL_FLASH_CACHE_XF_SCK_CLK_SEL_Pos);
    if(4U == flash_clk) // flash_clk is xo_16m, shall switch to pll_16m
    {
        MODIFY_REG(AON_CTL->FLASH_CACHE_CTRL0, AON_CTL_FLASH_CACHE_XF_SCK_CLK_SEL, (3UL << AON_CTL_FLASH_CACHE_XF_SCK_CLK_SEL_Pos));
    }
}

SECTION_RAM_CODE static void hal_clock_switch_sys_clk_to_hf_osc(void) /*lint !e618 MISRA exception. */
{
    // Disable XO is very dangerous which shall be strictly checked
    uint32_t primask = __get_PRIMASK();
    __set_PRIMASK(1);

    AON_PWR->XO_PLL_SET = AON_PWR_XO_PLL_SET_HF_OSC_SET;

    // Waiting for HF_OSC ready
    while(((ll_clk_get_hf_status() & LL_CLK_XO_PLL_HF_STAT) != LL_CLK_XO_PLL_HF_STAT))
    {
    }

    // Auto switch xo_16m clock to pll_16m clock for safety
    xo_16m_clock_switch_to_pll_16m();

    // Select HF OSC as the system clock
    ll_clk_select_source(LL_CLK_SEL_SOURCE_HF_OSC_CLK);

    // [BALIPRO-74] Don't turn off XO By SW for Possible Clock Sync
    // Notice the HW will auto turn off the XO clock when system shutdown
    // AON_PWR->XO_PLL_CLR = AON_PWR_XO_PLL_STAT_PLL_STAT | AON_PWR_XO_PLL_STAT_XO_STAT;
    ll_pwr_turn_off_enable_xo_pll_after_dcdc_ready();

    __set_PRIMASK(primask);
}

/* Exported functions --------------------------------------------------------*/
SECTION_RAM_CODE __WEAK uint32_t hal_clock_get_xo_requests(void)
{
    return xo_clock_requests;
}

SECTION_RAM_CODE __WEAK uint32_t hal_clock_get_device_xo_request_status(xo_request_device_number_t dev_num)
{
    return (uint32_t)((xo_clock_requests & (1UL << (uint32_t)dev_num)) >> (uint32_t)dev_num);
}

SECTION_RAM_CODE void hal_clock_request_xo_osc(xo_request_device_number_t dev_num)
{
    GLOBAL_EXCEPTION_DISABLE();
    hal_clock_set_device_xo_request_status(dev_num);
    GLOBAL_EXCEPTION_ENABLE();

    hal_clock_switch_sys_clk_to_xo_pll();

    // Determine If Turning Off HF_OSC or not
    if((xo_clock_requests & (1UL << (uint32_t)XO_REQUEST_DEVICE_NUM_CALIBRATION)))
    {
        if((ll_clk_get_hf_status() & LL_CLK_XO_PLL_HF_STAT) != LL_CLK_XO_PLL_HF_STAT)
        {
            AON_PWR->XO_PLL_SET = AON_PWR_XO_PLL_SET_HF_OSC_SET;
            while(((ll_clk_get_hf_status() & LL_CLK_XO_PLL_HF_STAT) != LL_CLK_XO_PLL_HF_STAT))
            {
            }
        }
    }
    else
    {
        AON_PWR->XO_PLL_CLR = AON_PWR_XO_PLL_CLR_HF_OSC_CLR;
    }
}

SECTION_RAM_CODE void hal_clock_release_xo_osc(xo_request_device_number_t dev_num)
{
    GLOBAL_EXCEPTION_DISABLE();
    hal_clock_clear_device_xo_request_status(dev_num);
    if(xo_clock_requests == 0U)
    {
        hal_clock_switch_sys_clk_to_hf_osc();
    }
    GLOBAL_EXCEPTION_ENABLE();
}

SECTION_RAM_CODE void hal_clock_release_xo_osc_all(void)
{
    GLOBAL_EXCEPTION_DISABLE();
    xo_clock_requests = 0;
    hal_clock_switch_sys_clk_to_hf_osc();
    GLOBAL_EXCEPTION_ENABLE();
}

/**
  * @}
  */

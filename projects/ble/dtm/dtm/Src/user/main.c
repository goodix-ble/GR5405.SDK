/**
 ****************************************************************************************
 *
 * @file main.c
 *
 * @brief main function Implementation.
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
 *****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "grx_sys.h"
#include "user_app.h"
#include "scatter_common.h"
#include "flash_scatter_config.h"
#include "patch.h"
#include "hci_uart.h"
#include "ble.h"
#include "app_assert.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/**@brief Stack global variables for Bluetooth protocol stack. */
STACK_HEAP_INIT(heaps_table);

#ifdef SOC_GR5410
extern bool g_bqb_test_en;
extern uint16_t g_lld_scan_max_data_len;
extern uint16_t g_lld_sync_max_data_len;
#endif

int main (void)
{
    ble_hci_uart_init();
#if CFG_CS_SUPPORT
    //temp commit for cs test, enable aes clock for cs security process
    hal_clock_enable_module(AES_BASE);
#endif

#ifdef SOC_GR5405
    ble_rf_tx_mode_set((ble_rf_tx_mode_t)RF_TX_PA_SELECT);
    ble_common_env_init_for_controller();
    ble_stack_controller_init(&heaps_table);
#else
    //g_lld_scan_max_data_len = 1270; //should config it according to the max data sent from the peer side
    //g_lld_sync_max_data_len = 1270; //should config it according to the max data sent from the peer side
    g_bqb_test_en = true;
    ble_controller_init(&heaps_table);
#endif

    while(1)
    {
      //hal_aon_wdt_refresh(&aon_wdt_handle);
#ifdef SOC_GR5405
#ifndef DTM_ATE_ENABLE
        pwr_mgmt_schedule();
#endif
#endif
    }
}

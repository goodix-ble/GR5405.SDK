/**
 *****************************************************************************************
 *
 * @file main.c
 *
 * @brief main function Implementation.
 *
 *****************************************************************************************
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
 *****************************************************************************************
 */
#include "custom_config.h"
#include "user_app.h"
#include "user_periph_setup.h"
#include "gr_includes.h"
#include "scatter_common.h"
#include "patch.h"
#include "app_log.h"

#if DTM_TEST_ENABLE
#include "hci_uart.h"
#endif

#if DTM_TEST_ENABLE
extern bool dtm_test_enable;
#endif

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**@brief Stack global variables for Bluetooth protocol stack. */
STACK_HEAP_INIT(heaps_table);

int main(void)
{
    // Initialize user peripherals.
    app_periph_init();

#if DTM_TEST_ENABLE
    if (dtm_test_enable)
    {
        ble_hci_uart_init();
    }
#endif

    // Initialize BLE Stack.
    ble_stack_init(ble_evt_handler, &heaps_table);

    // loop
    while (1)
    {
#if DTM_TEST_ENABLE
        if (!dtm_test_enable)
        {
            app_log_flush();
        }
#else
        app_log_flush();
#endif

        pwr_mgmt_schedule();
    }
}

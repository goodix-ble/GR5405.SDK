/**
 *****************************************************************************************
 *
 * @file boot_update.h
 *
 * @brief Update APP bootloader by APP.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2024 GOODIX
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

#ifndef __BOOT_UPDATE_H__
#define __BOOT_UPDATE_H__

#ifndef BOOT_UPDATE_DEBUG_ENABLE
#define BOOT_UPDATE_DEBUG_ENABLE                  1  // 0: disable, 1: enable
#endif
#if BOOT_UPDATE_DEBUG_ENABLE
#include "app_log.h"
#define BTUP_LOG(...)                             APP_LOG_DEBUG(__VA_ARGS__)
#define BTUP_LOG_FLUSH()                          app_log_flush()
#else
#define BTUP_LOG(...)
#define BTUP_LOG_FLUSH()
#endif

/*
Demo code:
#include "boot_update.h"
#include "new_boot_fw.h"
int main(void)
{
    app_periph_init();
    // Feed watch dog if needed
    boot_update_start(0x00204000, 0x8000, (uint32_t)&new_boot_fw, sizeof(new_boot_fw));
    while (1)
    {
        delay_ms(1000);
        printf("while loop \r\n");
        app_log_flush();
    }
}
*/

/**
 *****************************************************************************************
 * @brief Update BOOT FW
 * @param[in] old_boot_addr: Old BOOT FW start address.
 * @param[in] max_size: BOOT FW Max size.
 * @param[in] new_boot_addr: New BOOT FW start address.
 * @param[in] new_size: New BOOT FW size.
 *
 *****************************************************************************************
 */
void boot_update_start(uint32_t old_boot_addr, uint32_t max_size, uint32_t new_boot_addr, uint32_t new_size);

#endif /* __BOOT_UPDATE_H__ */

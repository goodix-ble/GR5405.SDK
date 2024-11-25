/**
 ******************************************************************************
 *
 * @file gr_nvds_adapter.c
 *
 * @brief BLE stack NVDS adapter
 *
 ******************************************************************************
 * @attention
  #####Copyright (c) 2019-2024 GOODIX
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

/* *****************************************************************************
 * Includes
 */
#include "cmsis_compiler.h"
#include "gr_nvds.h"
#include <stdbool.h>

struct ble_nvds_api_t
{
    bool (*initialized)(void);
    uint8_t (*get)(uint8_t tag, uint8_t *plen, uint8_t *pbuf);
    uint8_t (*put)(uint8_t tag, uint8_t len, uint8_t *pbuf);
    uint8_t (*del)(uint8_t tag);
};

/* *****************************************************************************
 * Functions for SDK and Stack internal usage
 */
//lint -e9075  [required] external symbol defined without a prior declaration
//lint -e818  [advisory] Pointer parameter could be declared as pointing to const
__WEAK bool ble_nvds_initialized(void)
{
    return (NVDS_STATE_RESET != nvds_get_state());
}

/**@note Define the function to be compatible with the following type
 *       uint8_t (*get) (uint8_t param_id, uint8_t * lengthPtr, uint8_t *buf). */
__WEAK uint8_t ble_nvds_get(uint8_t tag, uint8_t *p_len, uint8_t *p_buf)
{
    nvds_err_t ret;
    uint16_t len = *p_len;

    ret = nvds_get((uint16_t)NV_TAGCAT_STACK | tag, &len, p_buf);
    if (NVDS_SUCCESS == ret) {
        if (len > 0xFFU) {
            *p_len = 0;
            ret = NVDS_FAIL;
        } else {
            *p_len = (uint8_t)len;
        }
    } else {
        *p_len = 0;
    }

    return (uint8_t)ret;
}

/**@note Define the function to be compatible with the following type
 *       uint8_t (*set) (uint8_t param_id, uint8_t length, uint8_t *buf). */
__WEAK uint8_t ble_nvds_put(uint8_t tag, uint8_t len, uint8_t *p_buf)
{
    return (uint8_t)nvds_put((uint16_t)NV_TAGCAT_STACK | tag, len, (const uint8_t *)p_buf);
}

/**@note Define the function to be compatible with the following type
 *       uint8_t (*del) (uint8_t param_id). */
__WEAK uint8_t ble_nvds_del(uint8_t tag)
{
    return (uint8_t)nvds_del((uint16_t)NV_TAGCAT_STACK | tag);
}

__WEAK void ble_nvds_set_entry(struct ble_nvds_api_t *p_api)
{
    p_api->initialized = ble_nvds_initialized;
    p_api->get         = ble_nvds_get;
    p_api->put         = ble_nvds_put;
    p_api->del         = ble_nvds_del;
}

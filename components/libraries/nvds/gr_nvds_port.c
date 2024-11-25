/**
 ******************************************************************************
 *
 * @file gr_nvds_port.c
 *
 * @brief NVDS port file
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
#include "gr_nvds_port.h"
#include "gr5405_sys.h"

// NVDS Flash driver APIs
__WEAK uint32_t nvds_port_read(const uint32_t addr, uint8_t *buf, const uint32_t size)
{
    return hal_flash_read(addr, buf, size);
}

__WEAK uint32_t nvds_port_write_r(const uint32_t addr, const uint8_t *buf, const uint32_t size)
{
    return hal_flash_write_r(addr, buf, size);
}

__WEAK bool nvds_port_erase(const uint32_t addr, const uint32_t size)
{
    return hal_flash_erase(addr, size);
}

__WEAK void nvds_port_get_info(uint32_t *id, uint32_t *size)
{
    hal_flash_get_info(id, size);
}

__WEAK uint32_t nvds_port_lock(void)
{
   uint32_t ret_pri = __get_PRIMASK();
   __set_PRIMASK(1);
   return ret_pri;
}

__WEAK void nvds_port_unlock(uint32_t locker)
{
   __set_PRIMASK(locker);
}

static bool bd_addr_is_valid(uint8_t* p_bd_addr, uint32_t len)
{
    /* All 0 or 0xFF means BD_ADDRESS has not been written */
    const uint8_t first = p_bd_addr[0];
    if (first != 0U && first != 0xFFU)
    {
        return true;
    }
    for (uint32_t i = 1; i < len; i++)
    {
        if (first != p_bd_addr[i])
        {
            return true;
        }
    }
    return false;
}

__WEAK nvds_err_t nvds_port_find_tag_in_other(NvdsTag_t tag, uint16_t *p_len, uint8_t *p_buf)
{
    nvds_err_t ret = NVDS_TAG_NOT_EXISTED;
    uint16_t err_code = SDK_SUCCESS;
    uint8_t bd_addr[NVDS_LEN_BD_ADDRESS] = {0};
    uint16_t xo_offset = 0;

    switch (tag)
    {
        case NVDS_TAG_BD_ADDRESS:
            err_code = sys_device_addr_get(bd_addr);
            if (err_code != (uint16_t)SDK_SUCCESS) {
                break;
            }
            if (bd_addr_is_valid(bd_addr, NVDS_LEN_BD_ADDRESS))
            {
                memcpy(p_buf, bd_addr, NVDS_LEN_BD_ADDRESS);
                *p_len = NVDS_LEN_BD_ADDRESS;
                ret = NVDS_SUCCESS;
            }
            break;

        case NVDS_TAG_XO_OFFSET:
            err_code = sys_crystal_trim_get(&xo_offset);
            if (err_code != (uint16_t)SDK_SUCCESS) {
                break;
            }
            if ((xo_offset == 0x0000U) || (xo_offset == 0xFFFFU)) {
                break;
            }
            memcpy(p_buf, (uint8_t*)&xo_offset, NVDS_LEN_XO_OFFSET);
            *p_len = NVDS_LEN_XO_OFFSET;
            ret = NVDS_SUCCESS;
            break;
        default:
            /* do nothing */
            break;
    }
    return ret;
}

NvdsTag_t nvds_find_main_tag_by_sub_tag(NvdsTag_t tag)
{
    NvdsTag_t main_tag = 0;
    switch (tag)
    {
        /* The following tag will not change in application*/
        case NVDS_TAG_BD_ADDRESS:
        case NVDS_TAG_PARAM_ID_DEVICE_NAME:
        case NVDS_TAG_LPCLK_DRIFT:
        case NVDS_TAG_ACTIVITY_MOVE_CONFIG:
        case NVDS_TAG_XO_OFFSET:
            main_tag = NVDS_TAG_BD_ADDRESS;
            break;

        case NVDS_TAG_PRD_ADV_LIST0:
        case NVDS_TAG_PRD_ADV_LIST1:
        case NVDS_TAG_PRD_ADV_LIST2:
        case NVDS_TAG_PRD_ADV_LIST3:
            main_tag = NVDS_TAG_PRD_ADV_LIST0;
            break;

        default:
            /* do nothing */
            break;
    }

    /* BOND_ADDR_INFO and BOND_PAIR_INFO are stored in one item */
    if (IS_BOND_TAG(tag))
    {
        NvdsTag_t id = tag - (uint16_t)NV_BNDMGR_BONDS_ID_MIN;
        if (IS_BOND_ADDR_INFO(id) || IS_BOND_PAIR_INFO(id))
        {
            main_tag = tag - (id % TAG_NUM_PER_BONDS);
        }
    }
    return main_tag;
}

/**
  ******************************************************************************
  * @file    hal_flash.c
  * @author  Engineering Team
  * @brief   This file contains the implementation of HAL flash.
  ******************************************************************************
  * @attention
  *
  * Copyright(C) 2016-2017, Shenzhen Huiding Technology Co., Ltd
  * All Rights Reserved
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of Goodix Technology nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for Goodix Technology.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY Goodix Technology AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL Goodix Technology OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/*******************************************************************************
 * The file is the interface of Flash HAL. We should implement the interface
 * for each specified type of flash. We must enbale only one implementation
 * with the macro *FLASH_ENABLE.
 ******************************************************************************/

#include "hal.h"
#include "hal_flash.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "gr5405_sys.h"

#ifndef EXFLASH_ENABLE
#define EXFLASH_ENABLE                        /**<Use exflash. */
#endif

#ifndef VFLASH_ENABLE
//#define VFLASH_ENABLE                       /**<Use vflash for BLE Stack in Flash. */
#endif

#ifdef EXFLASH_ENABLE

#if defined(ROM_RUN_IN_FLASH)
const uint32_t baud_rate[6] = {XQSPI_BAUD_RATE_64M, XQSPI_BAUD_RATE_48M, XQSPI_BAUD_RATE_16M,
                               XQSPI_BAUD_RATE_24M, XQSPI_BAUD_RATE_16M, XQSPI_BAUD_RATE_32M};
xqspi_handle_t g_xqspi_handle = {0};
#endif

bool hal_flash_init(void)
{
    return true;
}

uint32_t hal_flash_read(const uint32_t addr, uint8_t *buf, const uint32_t size)
{
    return (HAL_OK == hal_exflash_read(addr, buf, size)) ? size : 0;
}

uint32_t hal_flash_write(const uint32_t addr, const uint8_t *buf, const uint32_t size)
{
    return (HAL_OK == hal_exflash_write(addr, (uint8_t*)buf, size)) ? size : 0;
}

uint32_t hal_flash_write_r(const uint32_t addr, const uint8_t *buf, const uint32_t size)
{
    uint32_t status;

    /* It's possible that the data is not written to flash memory.
     * So we must read the data from flash memory, and check it. */
    uint8_t  rd_buf[EXFLASH_SIZE_PAGE_BYTES];
    uint32_t offset     = 0;
    uint32_t unrd_bytes = size;
    uint32_t rd_bytes   = size > EXFLASH_SIZE_PAGE_BYTES ?
                          EXFLASH_SIZE_PAGE_BYTES : size;

    do
    {
        memcpy(rd_buf, buf + offset, rd_bytes);
        status = hal_exflash_write(addr + offset, rd_buf, rd_bytes);
        if (HAL_OK == status)
        {
            status = hal_exflash_read(addr + offset, rd_buf, rd_bytes);
            if ((HAL_OK == status) && (memcmp(buf + offset, rd_buf, rd_bytes) == 0))
            {
                unrd_bytes -= rd_bytes;
                if (0 == unrd_bytes)
                {
                    return size;
                }
                else
                {
                    offset += rd_bytes;
                    rd_bytes = unrd_bytes > EXFLASH_SIZE_PAGE_BYTES ?
                            EXFLASH_SIZE_PAGE_BYTES : unrd_bytes;
                    if ((offset >= size) || ((offset + rd_bytes) > size))
                    {
                        break;
                    }
                }
            }
            else
            {
                break;
            }
        }
        else
        {
            break;
        }
    } while(1);

    return 0;
}

void hal_flash_set_security(bool enable)
{
    return;
}

bool hal_flash_get_security(void)
{
    return false;
}

bool hal_flash_erase(const uint32_t addr, const uint32_t size)
{
    return (HAL_OK == hal_exflash_erase(EXFLASH_ERASE_SECTOR, addr, size)) ? true : false;
}

bool hal_flash_erase_chip(void)
{
    return true;
}

void hal_flash_get_info(uint32_t *id, uint32_t *size)
{
    hal_exflash_get_info(id, size);
    return;
}

uint32_t hal_flash_sector_size(void)
{
    return EXFLASH_SIZE_SECTOR_BYTES;
}

#endif // EXFLASH_ENABLE

/******************************************************************************/
/******************************************************************************/

#ifdef VFLASH_ENABLE

#include "vflash/vflash.h"

bool hal_flash_init(void)
{
    return vflash_init();
}

uint32_t hal_flash_read(const uint32_t addr, uint8_t *buf, const uint32_t size)
{
    return vflash_read(addr, buf, size);
}

uint32_t hal_flash_write(const uint32_t addr, const uint8_t *buf,
                         const uint32_t size)
{
    return vflash_write(addr, (uint8_t*)buf, size);
}

bool hal_flash_erase(const uint32_t addr, const uint32_t size)
{
   return vflash_erase(addr, size);
}

bool hal_flash_erase_chip(void)
{
   return false;
}

void hal_flash_get_info(uint32_t *id, uint32_t *size)
{
    if (NULL == id || NULL == size)
    {
        return;
    }

    *id   = 0;
    *size = 0;
}

uint32_t hal_flash_sector_size(void)
{
    return vflash_sector_size();
}

bool hal_flash_get_security(void)
{
    return false;
}
void hal_flash_set_security(bool enable)
{
    return;
}

#endif // VFLASH_ENABLE

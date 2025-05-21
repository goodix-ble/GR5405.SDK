/**
 *****************************************************************************************
 *
 * @file boot_update.c
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
#include "hal.h"
#include "boot_update.h"
#include "hal_flash.h"
#include <string.h>

#define IMG_INFO_PATTERN               0x4744U
#define IMG_INFO_ALIGN_BYTE            16U
#define FLASH_PAGE_LEN                 256U
#define APP_INFO_PATTERN               0x47525858U
#define BUILD_IN_APP_INFO_OFFSET       0x200U
#define DFU_IMAGE_INFO_LEN             48U     /**< Image information length. */
#define DFU_FLASH_SECTOR_SIZE          4096U
#define FLOOR_ALIGN(addr, size)        ((addr) - ((addr) % (size)))

/**@brief Boot information definition. */
typedef struct
{
    uint32_t bin_size;
    uint32_t check_sum;
    uint32_t load_addr;
    uint32_t run_addr ;
    uint32_t xqspi_xip_cmd;
    uint32_t xqspi_speed:4;           /*!< bit: 0..3  clock speed */
    uint32_t code_copy_mode:1;        /*!< bit: 4 code copy mode */
    uint32_t system_clk:3;            /*!< bit: 5..7 system clock */
    uint32_t check_image:1;           /*!< bit: 8 check image */
    uint32_t boot_delay:1;            /*!< bit: 9 boot delay time */
    uint32_t signature_algorithm:2;   /*!< bit: 10..11 signature algorithm */
    uint32_t reserved:20;             /*!< bit: 20 reserved */
} boot_info_t;

/**@brief IMG information definition. */
typedef struct
{
    uint16_t        pattern;           /**< IMG info pattern. */
    uint16_t        version;           /**< IMG version. */
    boot_info_t     boot_info;         /**< IMG boot info. */
    uint8_t         comments[12];      /**< IMG comments. */
} dfu_img_info_t;

typedef struct __attribute((packed))
{
    uint32_t    app_pattern;
    uint32_t    app_info_version;
    uint32_t    chip_ver;
    uint32_t    load_addr;
    uint32_t    run_addr;
    uint32_t    app_info_sum;
    uint8_t     check_img;
    uint8_t     boot_delay;
    uint8_t     sec_cfg;
    uint8_t     reserved0;
    uint8_t     comments[12];
    uint32_t    reserved1[6];
} build_in_app_info_t;

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
static bool find_img_info(uint8_t *p_data, uint32_t len, uint32_t load_addr, dfu_img_info_t *p_img_info)
{
    dfu_img_info_t *p_temp_img_info;
    for (uint32_t i = 0; i <= (len - sizeof(dfu_img_info_t)); i += IMG_INFO_ALIGN_BYTE)
    {
        //lint -e9087 -e740 [required] Casting is safe
        p_temp_img_info = (dfu_img_info_t *)&p_data[i];
        if ((IMG_INFO_PATTERN == p_temp_img_info->pattern) && (load_addr == p_temp_img_info->boot_info.load_addr))
        {
            memcpy(p_img_info, p_temp_img_info, sizeof(dfu_img_info_t));
            return true;
        }
    }
    return false;
}

static bool get_img_info(uint32_t start_addr, uint32_t size, dfu_img_info_t *p_img_info)
{
    uint32_t fw_data[(FLASH_PAGE_LEN + sizeof(dfu_img_info_t)) / sizeof(uint32_t)];
    uint32_t load_addr;

    //lint -e9087 Casting is safe
    build_in_app_info_t *p_app_info = (build_in_app_info_t *)fw_data;

    memset(fw_data, 0x00, sizeof(fw_data));
    (void)hal_flash_read(start_addr + BUILD_IN_APP_INFO_OFFSET, (uint8_t *)fw_data, sizeof(fw_data));
    if ((APP_INFO_PATTERN == p_app_info->app_pattern)
        && (p_app_info->load_addr == p_app_info->run_addr)
        && (p_app_info->app_info_sum == (p_app_info->app_pattern + p_app_info->app_info_version + p_app_info->chip_ver + p_app_info->load_addr + p_app_info->run_addr)))
    {
        load_addr = p_app_info->load_addr;
    }
    else
    {
        BTUP_LOG("Get APP_INFO error");
        return false;
    }

    uint32_t addr = start_addr + size - sizeof(fw_data);
    addr = FLOOR_ALIGN(addr, IMG_INFO_ALIGN_BYTE);
    while (addr > start_addr)
    {
        memset(p_img_info, 0x00, sizeof(dfu_img_info_t));
        memset(fw_data, 0x00, sizeof(fw_data));
        (void)hal_flash_read(addr, (uint8_t *)fw_data, sizeof(fw_data));
        if (find_img_info((uint8_t *)fw_data, sizeof(fw_data), load_addr, p_img_info))
        {
            BTUP_LOG("Get image info OK");
            BTUP_LOG("Load Address = 0x%08x", p_img_info->boot_info.load_addr);
            BTUP_LOG("Run Address  = 0x%08x", p_img_info->boot_info.run_addr);
            BTUP_LOG("Bin Size     = 0x%08x", p_img_info->boot_info.bin_size);
            BTUP_LOG("CheckSum     = 0x%08x", p_img_info->boot_info.check_sum);
            char comments[sizeof(((dfu_img_info_t *)0)->comments) + 1U];
            memcpy(comments, p_img_info->comments, sizeof(((dfu_img_info_t *)0)->comments));
            comments[sizeof(comments) - 1U] = '\0';
            BTUP_LOG("Comments     = %s", comments);
            return true;
        }
        else
        {
            addr -= sizeof(fw_data) - sizeof(dfu_img_info_t);
        }
    }
    BTUP_LOG("Get image info error");
    return false;
}

static void fw_copy(uint32_t dst_addr, uint32_t src_addr, uint32_t size)
{
    uint8_t    flash_read_buff[DFU_FLASH_SECTOR_SIZE];
    uint32_t   copy_page = 0;
    uint32_t   remain    = 0;
    uint32_t   temp_size = 0;
    uint32_t   copy_size = 0;

    temp_size = size + DFU_IMAGE_INFO_LEN;

    BTUP_LOG("FW copy start");

    copy_page = temp_size / DFU_FLASH_SECTOR_SIZE;
    remain    = temp_size % DFU_FLASH_SECTOR_SIZE;

    if (0U != remain)
    {
        copy_page++;
    }
    GLOBAL_EXCEPTION_DISABLE();
    for (uint32_t i = 0; i < copy_page; i++)
    {
        if ((i == (copy_page - 1U)) && (0U != remain))
        {
            // The last page is not full
            copy_size = remain;
        }
        else
        {
            copy_size = DFU_FLASH_SECTOR_SIZE;
        }
        memset(flash_read_buff, 0xFF, sizeof(flash_read_buff));
        uint32_t d_addr = dst_addr + i * DFU_FLASH_SECTOR_SIZE;
        uint32_t s_addr = src_addr + i * DFU_FLASH_SECTOR_SIZE;
        BTUP_LOG("src:0x%x dst:0x%x size:0x%x", s_addr,  d_addr, copy_size);
        (void)hal_flash_read(s_addr, flash_read_buff, copy_size);
        (void)hal_flash_write(d_addr, flash_read_buff, copy_size);
    }
    GLOBAL_EXCEPTION_ENABLE();
    BTUP_LOG("FW copy end");
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void boot_update_start(uint32_t old_boot_addr, uint32_t max_size, uint32_t new_boot_addr, uint32_t new_size)
{
    dfu_img_info_t old_image_info;
    dfu_img_info_t new_image_info;
    memset(&old_image_info, 0x1, sizeof(dfu_img_info_t));
    memset(&new_image_info, 0x2, sizeof(dfu_img_info_t));
    if (get_img_info(old_boot_addr, max_size, &old_image_info))
    {
        if (get_img_info(new_boot_addr, new_size, &new_image_info))
        {
            //lint -e9007 [required] NO side effects of memcmp
            if ((old_image_info.boot_info.bin_size == new_image_info.boot_info.bin_size)
            &&(old_image_info.boot_info.check_sum == new_image_info.boot_info.check_sum)
            && (0 == memcmp(old_image_info.comments, new_image_info.comments, sizeof(new_image_info.comments))))
            {
                BTUP_LOG("BOOT is same.");
                return;
            }
            else
            {
                if (new_image_info.boot_info.bin_size > max_size)
                {
                    BTUP_LOG("New BOOT size is too large");
                    return;
                }
                BTUP_LOG("BOOT is diff, update old BOOT");
                if (hal_flash_erase(old_boot_addr, max_size))
                {
                    fw_copy(old_boot_addr, new_boot_addr, new_image_info.boot_info.bin_size);
                    BTUP_LOG_FLUSH();
                    hal_nvic_system_reset();
                }
                else
                {
                    BTUP_LOG("BOOT erase error");
                    return;
                }
            }
        }
    }
}

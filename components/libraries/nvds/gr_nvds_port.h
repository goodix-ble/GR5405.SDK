/**
 ******************************************************************************
 *
 * @file gr_nvds_port.h
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

/**
 * @addtogroup SYSTEM
 * @{
 */
 /**
 * @addtogroup NVDS Non-Volatile Data Storage
 * @{
 * @brief Definitions and prototypes for the NVDS interface.
 */

#ifndef GR_NVDS_PORT_H
#define GR_NVDS_PORT_H

#include "gr_nvds.h"
#include "cmsis_compiler.h"
#include "hal_exflash.h"
#include "hal_flash.h"

/* The follow tags are stored in one item which main tag is 0xC001. */
#define NVDS_TAG_BD_ADDRESS                   0xC001
#define NVDS_TAG_PARAM_ID_DEVICE_NAME         0xC002
#define NVDS_TAG_LPCLK_DRIFT                  0xC003
#define NVDS_TAG_ACTIVITY_MOVE_CONFIG         0xC005
#define NVDS_TAG_XO_OFFSET                    0xC016
#define NVDS_LEN_BD_ADDRESS                   6
#define NVDS_LEN_LPCLK_DRIFT                  6
#define NVDS_LEN_ACTIVITY_MOVE_CONFIG         1
#define NVDS_LEN_XO_OFFSET                    2
/* Maximum supported length of device name. */
#define NVDS_LEN_PARAM_ID_DEVICE_NAME               \
        (240 - (NVDS_LEN_BD_ADDRESS + 8 +           \
                NVDS_LEN_LPCLK_DRIFT + 8 +          \
                NVDS_LEN_ACTIVITY_MOVE_CONFIG + 8   \
                NVDS_LEN_XO_OFFSET + 8))

/* The follow tags are stored in one item which main tag is 0x8020. */
#define NVDS_TAG_PRD_ADV_LIST0                0x8020
#define NVDS_TAG_PRD_ADV_LIST1                0x8021
#define NVDS_TAG_PRD_ADV_LIST2                0x8022
#define NVDS_TAG_PRD_ADV_LIST3                0x8023
#define NVDS_LEN_PRD_ADV_LIST                 8

/* One bond device address info and pair info tags are stored in one item which main tag is address info tag. */
/* Bonding NVDS items, GAP_MAX_BONDINGS = CFG_MAX_BOND_DEVS,
 * A bonding entry consists of 3 NVDS items which are defined in ble_bondmgr.c
 * Range: 0x40 ~ (0x40 + 3 * CFG_MAX_BOND_DEVS - 1) */
#define BLE_NVIDX_BONDS_START       0x40  /* start idx for bond records */

#define MAX_BOND_DEVS                50U
#define TAG_NUM_PER_BONDS            3U
#define NV_BNDMGR_BONDS_ID_MIN       (((uint32_t)NV_TAGCAT_BNDMGR + (uint32_t)BLE_NVIDX_BONDS_START))
#define NV_BNDMGR_BONDS_ID_MAX       ((uint32_t)NV_TAGCAT_BNDMGR + (uint32_t)BLE_NVIDX_BONDS_START + (TAG_NUM_PER_BONDS*MAX_BOND_DEVS))
#define IS_BOND_TAG(tag)             (((tag) >= NV_BNDMGR_BONDS_ID_MIN) && ((tag) < NV_BNDMGR_BONDS_ID_MAX))
#define IS_BOND_ADDR_INFO(id)        (((uint32_t)(id) % TAG_NUM_PER_BONDS) == 0U)
#define IS_BOND_PAIR_INFO(id)        (((uint32_t)(id) % TAG_NUM_PER_BONDS) == 1U)

#define NVDS_SECTOR_SIZE             EXFLASH_SIZE_SECTOR_BYTES
#define NVDS_FLASH_BASE              EXFLASH_START_ADDR

uint32_t nvds_port_read(const uint32_t addr, uint8_t *buf, const uint32_t size);
/*
NOTE: The memory to which the second parameter of nvds_port_write_r() can not
locate in Flash. Because reading data from flash is disabled during calling nvds_port_write_r().
*/
uint32_t nvds_port_write_r(const uint32_t addr, const uint8_t *buf, const uint32_t size);
bool nvds_port_erase(const uint32_t addr, const uint32_t size);
void nvds_port_get_info(uint32_t *id, uint32_t *size);
nvds_err_t nvds_port_find_tag_in_other(NvdsTag_t tag, uint16_t *p_len, uint8_t *p_buf);
uint32_t nvds_port_lock(void);
void nvds_port_unlock(uint32_t locker);
NvdsTag_t nvds_find_main_tag_by_sub_tag(NvdsTag_t tag);

#endif /* GR_NVDS_PORT_H */

/** @} */
/** @} */

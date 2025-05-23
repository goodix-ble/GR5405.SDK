/**
 *****************************************************************************************
 *
 * @file sign_verify.c
 *
 * @brief Second boot function Implementation.
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
#include "grx_hal.h"
#include "grx_sys.h"
#include "bootloader_config.h"
#include "custom_config.h"
#include "flash_scatter_config.h"


#if (BOOTLOADER_SIGN_ENABLE && !BOOTLOADER_BOOT_PORT_ENABLE)

#if defined(SOC_GR533X) || defined(SOC_GR5405)
#include "app_crypto.h"
#endif

#include "drv_common.h"

#include "rsa.h"
#include "hal_flash.h"

#define APP_FLASH_START_ADDR               (FLASH_START_ADDR + 0x2000)
#define APP_FLASH_SIZE                     (0x00100000)
#define APP_FLASH_END_ADDR                 (APP_FLASH_START_ADDR + APP_FLASH_SIZE)

#define SHA256_SIZE                        32
#define SIGNATURE_SIZE                     256
#define RSA_KEY_SIZE                       256
#define RSA_PUBLIC_KEY_SIZE                (2*RSA_KEY_SIZE + 8) 
#define ECIES_PUBLIC_KEY_SIZE              64
#define ECC_AXIS_SIZE                      (ECIES_PUBLIC_KEY_SIZE/2)

typedef struct
{
    uint32_t bin_size;
    uint32_t check_sum;
    uint32_t load_addr;
    uint32_t run_addr ;
    uint32_t xqspi_xip_cmd;
    uint32_t xqspi_speed:4; 
    uint32_t code_copy_mode:1;
    uint32_t system_clk:3;
    uint32_t check_image:1;
    uint32_t boot_delay:1;
    uint32_t reserved:22;
} bl_boot_info_t;

typedef struct
{
    uint8_t *user_hash;
    uint8_t  hmac_key_type_sel;
    uint8_t  operation_mode;
    uint8_t *hmac_key;
    uint8_t  dma_enable;
    uint8_t  no_padding;
    uint8_t  interrupt_enable;
    uint8_t  continue_input;
} gm_sha_config_t;

typedef struct _gm_ecc_point
{
    uint32_t x[8];
    uint32_t y[8];
} gm_ecc_point_t;

typedef struct
{
        uint32_t reserved0;
        uint32_t fw_len;
        uint32_t checksum;
        uint32_t loadaddr;
        uint32_t runaddr;
        uint32_t reserved[7];
        uint32_t pattern;
        uint32_t paddingsize;
        uint8_t  ecies[ECIES_PUBLIC_KEY_SIZE];
        uint8_t  reserved1[8];
        uint8_t  rsa[RSA_PUBLIC_KEY_SIZE];
        uint8_t  signature[SIGNATURE_SIZE];
} bl_fw_info_t;

#if !defined(SOC_GR533X) && !defined(SOC_GR5405)
extern gm_drv_ret_e gm_drv_sha_encrypt(const gm_sha_config_t *config, const uint8_t *in, uint32_t byte_length, uint8_t *out);
#ifdef SOC_GR5515
extern void hal_xqspi_set_xip_present_status_patch(xqspi_handle_t *p_xqspi, uint32_t status);
#else
extern void hal_xqspi_set_xip_present_status(xqspi_handle_t *p_xqspi, uint32_t status);
#endif

SECTION_RAM_CODE void app_boot_xqspi_set_xip_present_status(xqspi_handle_t *p_xqspi, uint32_t status)
{
#ifdef SOC_GR5515
    hal_xqspi_set_xip_present_status_patch(p_xqspi, status);
#else
    uint32_t cache_flush = p_xqspi->init.cache_flush;
    p_xqspi->init.cache_flush = XQSPI_CACHE_FLUSH_EN;
    hal_xqspi_set_xip_present_status(p_xqspi, status);
    p_xqspi->init.cache_flush = cache_flush;
#endif
}

void app_boot_ll_cgc_disable_force_off_rng_hclk(void)
{
#if defined(SOC_GR5526) || defined(SOC_GR5X25)
    ll_cgc_disable_force_off_rng_hclk();
#endif
}

void app_boot_ll_cgc_disable_force_off_hmac_hclk(void)
{
#if defined(SOC_GR5526) || defined(SOC_GR5X25)
    ll_cgc_disable_force_off_hmac_hclk();
#endif
}

void app_boot_ll_cgc_disable_force_off_pkc_hclk(void)
{
#if defined(SOC_GR5526) || defined(SOC_GR5X25)
    ll_cgc_disable_force_off_pkc_hclk();
#endif
}

extern exflash_handle_t g_exflash_handle;
#endif

extern void security_disable(void);
extern void security_state_recovery(void);

/**
 ****************************************************************************************
 * @brief  check fw signature
 *
 * @stepone   ::read fw_info :fw_data is encrypted in flash,fw_info is not encrypted.
 * @steptwo   ::check rsa public key hash.
 * @stepthree ::check fw_data hash.
 ****************************************************************************************
 */
SECTION_RAM_CODE static int bl_check_fw_security(bl_boot_info_t *boot_info, const uint8_t *p_public_key_hash, uint32_t is_sec_enable)
{
    int ret = GM_BL_OK;
    uint8_t hash[SHA256_SIZE] = {0};
    uint8_t *fw = (uint8_t *)boot_info->load_addr;
    uint32_t fw_size = boot_info->bin_size;
    bl_fw_info_t fw_info;

    //GLOBAL_EXCEPTION_DISABLE();
    security_disable();
#ifndef SOC_GR5515
    hal_exflash_read((uint32_t )(fw + fw_size), (uint8_t *)&fw_info, sizeof(bl_fw_info_t));
#else
    extern exflash_handle_t g_exflash_handle; 
    extern hal_status_t hal_exflash_read_rom(exflash_handle_t *p_exflash, uint32_t addr, uint8_t *p_data, uint32_t size);
    hal_exflash_read_rom(&g_exflash_handle, (uint32_t )(fw + fw_size), (uint8_t *)&fw_info, sizeof(bl_fw_info_t));
#endif
    security_state_recovery();
    //GLOBAL_EXCEPTION_ENABLE();

#if defined(SOC_GR533X) || defined(SOC_GR5405)
    do
    {
        if ((ret = ac_sha256_init()) != 0)
        {
            break;
        }

        if ((ret = ac_sha256_update(fw_info.rsa, 65)) != 0)
        {
            break;
        }

        if ((ret = ac_sha256_finish(hash)) != 0)
        {
            break;
        }

        if (memcmp(hash, p_public_key_hash, SHA256_SIZE>>1) != 0)
        {
            ret = GM_BL_ERROR_SIGNATURE;
            break;
        }

        if ((ret = ac_sha256_init()) != 0)
        {
            break;
        }

        if ((ret = ac_sha256_update((uint8_t *)fw, fw_size + sizeof(bl_fw_info_t) - SIGNATURE_SIZE)) != 0)
        {
            break;
        }

        if ((ret = ac_sha256_finish(hash)) != 0)
        {
            break;
        }

        if((ret = ac_ecc_ecdsa_verify(AC_ECC_SECP256R1, fw_info.rsa + 1, hash, 32, fw_info.signature)) != 0)
        {
            ret = GM_BL_ERROR_SIGNATURE;
            break;
        }

    } while(0);
#else
    gm_sha_config_t sha_config = {0};
    #if !SECURITY_CFG_VAL
    bl_rsa_context rsa = {0};
    #else
    extern int bl_pkc_ecdsa_verify(uint8_t *messege_hash, gm_ecc_point_t *public_key, uint32_t in_signiture_r[ECC_U32_LENGTH], uint32_t in_signiture_s[ECC_U32_LENGTH]);
    gm_ecc_point_t gm_ecc_point;
    #endif

    //max fw size: 8*1024*1024 - 8*1024
    if ((boot_info->load_addr < APP_FLASH_START_ADDR) || (fw_size > (APP_FLASH_END_ADDR - APP_FLASH_START_ADDR)))
    {
        return GM_BL_ERROR_FW;
    }


    do {
        /*calculate rsa public key hash and compare with last 16bytes width efuse fw_public_key_hash*/
        memset(&sha_config, 0x0, sizeof(gm_sha_config_t));
        #if !SECURITY_CFG_VAL
        if ((ret = gm_drv_sha_encrypt(&sha_config, fw_info.rsa, RSA_PUBLIC_KEY_SIZE, hash)) < 0)
        {
            break;
        }
        #else
        if ((ret = gm_drv_sha_encrypt(&sha_config, fw_info.rsa, ECIES_PUBLIC_KEY_SIZE + 1, hash)) < 0)
        {
            break;
        }
        #endif

        if (memcmp(hash, p_public_key_hash, SHA256_SIZE>>1) != 0)
        {
            ret = GM_BL_ERROR_SIGNATURE; /*sha error, stop boot!*/
            break;
        }

        /*calculate fw hash except signature data*/
        memset(&sha_config, 0x0, sizeof(gm_sha_config_t));

        security_disable();
        do {
            if (is_sec_enable)
            {
                uint32_t __l_present_rest = ll_xqspi_get_present_bypass(g_exflash_handle.p_xqspi->p_instance);
                app_boot_xqspi_set_xip_present_status(g_exflash_handle.p_xqspi, XQSPI_DISABLE_PRESENT);
                if ((ret = gm_drv_sha_encrypt(&sha_config, fw, fw_size + sizeof(bl_fw_info_t) - SIGNATURE_SIZE, hash)) < 0)
                {
                    app_boot_xqspi_set_xip_present_status(g_exflash_handle.p_xqspi, __l_present_rest);
                    break;
                }
                app_boot_xqspi_set_xip_present_status(g_exflash_handle.p_xqspi, __l_present_rest);
            }
            else
            {
                if ((ret = gm_drv_sha_encrypt(&sha_config, fw, fw_size + sizeof(bl_fw_info_t) - SIGNATURE_SIZE, hash)) < 0)
                {
                    break;
                }
            }

            #if !SECURITY_CFG_VAL
            /*do rsa pkcs1-v2.1 format signature check*/
            rsa.len = RSA_KEY_SIZE; /*sizeof(N) in chars*/
            rsa.pk = (bl_rsa_public_key_t*)fw_info.rsa;
            if((ret = bl_rsa_rsassa_pss_verify(&rsa, hash, fw_info.signature)) < 0)
            {
                ret = GM_BL_ERROR_SIGNATURE;
                break;
            }
            #else
            uint32_t r_val[8] = {0};
            uint32_t s_val[8] = {0};
            memcpy(r_val, (fw_info.signature), 32);
            memcpy(s_val, (fw_info.signature + 32), 32);
            memcpy(&gm_ecc_point, (gm_ecc_point_t *)(fw_info.rsa + 1), ECIES_PUBLIC_KEY_SIZE);
            if ((ret = bl_pkc_ecdsa_verify(hash, &gm_ecc_point, r_val, s_val)) != 0)
            {
                ret = GM_BL_ERROR_SIGNATURE;
                break;
            }
            #endif
        }while(0);
        security_state_recovery();
    }while(0);
#endif
    return ret;
}



bool sign_verify(uint32_t fw_start_addr, uint32_t fw_size, const uint8_t *p_public_key_hash, uint32_t is_sec_enable)
{
    bool result = true;

#if !defined(SOC_GR533X) && !defined(SOC_GR5405)
    app_boot_security_clock_set();
#endif
    bl_boot_info_t  boot_info =
    {
        .bin_size  = fw_size,
        .load_addr = fw_start_addr,
        .run_addr  = fw_start_addr,
    };
#if !defined(SOC_GR533X) && !defined(SOC_GR5405)
    ll_cgc_disable_force_off_secu_hclk();
    ll_cgc_disable_force_off_secu_div4_pclk();
    app_boot_ll_cgc_disable_force_off_rng_hclk();
    app_boot_ll_cgc_disable_force_off_hmac_hclk();
    app_boot_ll_cgc_disable_force_off_pkc_hclk();
#endif
    if (bl_check_fw_security(&boot_info, p_public_key_hash, is_sec_enable) < 0)
    {
        result = false;
    }
    return result;
}


#endif



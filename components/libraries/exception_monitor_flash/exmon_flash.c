/**
 ****************************************************************************************
 *
 * @file    exmon_flash.c
 * @author  BLE Driver Team
 * @brief   Source file of exception monitor flash.
 *
 ****************************************************************************************
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
 ****************************************************************************************
 */
#include <string.h>
#include "exmon_flash.h"
#include "hal_exflash.h"

/*
*        exmon_flash Layout
*    ----------------------------------------------------------------------------------------------|
*    |   32bit    |    EXMON_FLASH_END_ADDRESS - EXMON_FLASH_START_ADDRESS - 64bit    |   32bit    |
*    |    LEN     |                                DATA                               |  CHECKSUM  |
*    ----------------------------------------------------------------------------------------------|
*/

/*
*        Use Reference code:
*        1. Initialize exmon_flash; a return value of EXMON_FLASH_ERROR_NONE indicates successful initialization.
*        2. Initialize aon_wdt and map the aon_wdt interrupt to the NMI interrupt.
*        3. Perform erase, write, and read tests on other areas (outside the exmon_flash region).
*        4. During the erase, write, and read flash tests in other areas, the aon_wdt will trigger the alarm interrupt.
*        5. Since the alarm interrupt of aon_wdt is mapped to the NMI interrupt, it can interrupt the erase, write, and read flash tests in other areas.
*           At this time, the flash is in a busy state.
*        6. In the aon_wdt alarm interrupt, first feed the watchdog, then call exmon_flash_restore to restore the flash.
*           Finally call exmon_flash_write to store the RAM information into FLASH, followed by a soft reset.
*        7. After the reset is complete, reinitialize exmon_flash. Since exmon_flash already contains valid information, it will return EXMON_FLASH_ERROR_INITIALIZED.
*        8. Read the information stored in exmon_flash and compare it with the previous RAM information.
*           If they are the same, it proves that the exmon_flash function is working correctly, and then erase the exmon_flash area.
*-------------------------------------------------------------------------------------------------------
    // Erase, read, and write tests for exmon_flash.
    #define FLASH_DATA_SIZE     (EXMON_FLASH_END_ADDRESS - EXMON_FLASH_START_ADDRESS - 4 - 4)
    static uint8_t flash_write_data_test[FLASH_DATA_SIZE];
    static uint8_t flash_read_data_test[FLASH_DATA_SIZE];

    // Erase, read, and write flash tests for other FLASH areas.
    #define FALSH_START_ADDR_OTHER_AREAS (0x00260000)
    #define FLASH_DATA_SIZE_OTHER_AREAS  (4096 * 4)
    static uint8_t flash_unwrite_data_test[FLASH_DATA_SIZE_OTHER_AREAS];
    static uint8_t flash_unread_data_test[FLASH_DATA_SIZE_OTHER_AREAS];

    static app_aon_wdt_params_t s_params;

    void app_aon_wdt_evt_handler(void)
    {
        // AON_WDT is mapped to the NMI interrupt, so app_aon_wdt_evt_handler will not be triggered.
    }

    SECTION_RAM_CODE void NMI_Handler(void)
    {
        uint32_t error_code = 0;

        // Feed the watchdog to avoid triggering a WDT reset when logging information to Flash.
        ll_aon_wdt_reload_counter();
        uint32_t timeout = 500000;
        while(ll_aon_wdt_is_busy())
        {
            if (--timeout == 0U)
            {
                break ;
            }
        }

        // Note 5: The flash has returned to a writable state.
        error_code = exmon_flash_restore();
        if (EXMON_FLASH_ERROR_NONE == error_code)
        {
            printf("4. exmon flash restore success! \r\n");
        }
        else
        {
            printf("exmon flash restore fail, error_code = 0x%x\r\n", error_code);
        }

        for (uint32_t i=0; i< FLASH_DATA_SIZE; i++)
        {
            flash_write_data_test[i] = 0x74;
        }

        // Note 6: Before the reset, write the RAM information that needs to be saved to exmon_flash.
        error_code = exmon_flash_write(flash_write_data_test, FLASH_DATA_SIZE);
        if (EXMON_FLASH_ERROR_NONE == error_code)
        {
            printf("5. exmon flash write success! \r\n");
        }
        else
        {
            printf("exmon flash write fail, error_code = 0x%x\r\n", error_code);
        }
        app_log_flush();

        hal_nvic_system_reset();
        for (;;)
        {
        }
    }

    void exmon_flash_read_erase_test(void)
    {
        uint32_t read_flash_data_size = 0;
        uint32_t error_code = 0;

        error_code = exmon_flash_read(flash_read_data_test, &read_flash_data_size);
        if (EXMON_FLASH_ERROR_NONE == error_code)
        {
            // Note 3: If exmon_flash_read returns EXMON_FLASH_ERROR_NONE, it indicates that the data has passed the CHECKSUM check, and the data read is correct.
            printf("1. exmon flash read size : %d \r\n", read_flash_data_size);
            if (0 == memcmp(flash_read_data_test, flash_write_data_test, read_flash_data_size))
            {
                printf("2. verify exmon flash success! \r\n");
            }
            else
            {
                printf("verify exmon flash fail! \r\n");
            }
        }
        else
        {
            printf("exmon flash read fail, error_code = 0x%x\r\n", error_code);
        }
        // Note 4: After the data reading is complete, exmon_flash needs to be erased.
        error_code = exmon_flash_erase();
        if (EXMON_FLASH_ERROR_NONE == error_code)
        {
            printf("3. exmon flash erase success! \r\n");
        }
        else
        {
            printf("exmon flash erase fail, error_code = 0x%x\r\n", error_code);
        }
    }

    void exmon_flash_test(void)
    {
        uint32_t error_code = 0;

        for (uint32_t i=0; i < FLASH_DATA_SIZE; i++)
        {
            flash_write_data_test[i] = 0x74;
        }

        printf("\r\n");
        error_code = exmon_flash_init();
        if (EXMON_FLASH_ERROR_NONE == error_code)
        {
            // Note 1: It will only be successfully initialized once during the entire lifecycle (without upgrading the firmware)
            //       , unless the FLASH is erased and the subsequent write to FLASH fails
            printf("0. exmon flash init success!!!\r\n");
        }
        else if (EXMON_FLASH_ERROR_INITIALIZED == error_code)
        {
            // Note 2: If EXMON_FLASH_ERROR_INITIALIZED is returned, it indicates that valid data is stored in exmon_flash.
            exmon_flash_read_erase_test();
        }
        else
        {
            printf("exmon flash init fail, error_code = 0x%x\r\n", error_code);
        }
    }

    void aon_wdt_map_to_nmi(void)
    {
        // Map the aon_wdt interrupt to the NMI interrupt.
        MCU_SUB->MCU_NMI_CFG = MCU_SUB->MCU_NMI_CFG | 0x20;
    }

    void app_aon_wdt(void)
    {
        uint16_t ret = 0;

        s_params.init.counter = 4000;          // Trigger a watchdog reset after 4 seconds
        s_params.init.alarm_counter = 2000;    // Trigger the alarm interrupt 2 seconds before the watchdog reset

        ret = app_aon_wdt_init(&s_params, app_aon_wdt_evt_handler);
        if (APP_DRV_SUCCESS != ret)
        {
            printf("\r\n AON_WDT INIT FAIL ERROR_CODE = 0x%x\r\n ", ret);
            return;
        }

        aon_wdt_map_to_nmi();

        for (uint32_t i = 0; i < 10; i++)
        {
            delay_ms(300);
            app_aon_wdt_refresh();
            printf("\r\n %dth feed dog.\r\n", i);
        }

        printf("\r\nSystem will reset.\r\n");
    }

    void flash_erase_write_read_test_for_other_areas(void)
    {
        // Perform erase, write, and read tests in FLASH areas other than exmon_flash to verify that exmon_flash can operate normally when the FLASH is in a busy state.
        for (uint32_t i=0; i < 100; i++)
        {
            hal_exflash_erase(EXFLASH_ERASE_SECTOR, FALSH_START_ADDR_OTHER_AREAS, FLASH_DATA_SIZE_OTHER_AREAS);
            hal_exflash_write(FALSH_START_ADDR_OTHER_AREAS, flash_unwrite_data_test, FLASH_DATA_SIZE_OTHER_AREAS);
            hal_exflash_read(FALSH_START_ADDR_OTHER_AREAS, flash_unread_data_test, FLASH_DATA_SIZE_OTHER_AREAS);
            if (0 == memcmp(flash_unread_data_test, flash_unwrite_data_test, FLASH_DATA_SIZE_OTHER_AREAS))
            {
                printf("flash test i = %d\r\n", i);
            }
            else
            {
                printf("flash test fail i = %d\r\n", i);
            }
        }
    }

    int main(void)
    {
        board_init();

        exmon_flash_test();
        app_aon_wdt();
        flash_erase_write_read_test_for_other_areas();

        while(1);
    }
*-------------------------------------------------------------------------------------------------------
*/

#define EXMON_FLASH_LEN_LENGTH             (4)   // Unit: byte
#define EXMON_FLASH_CHECKSUM_LENGTH        (4)   // Unit: byte
#define EXMON_FLASH_DEFAULT_CHECK_LENGTH   (128) // Unit: byte

static uint32_t exmon_flash_get_checksum(uint8_t *p_data, uint32_t size)
{
    uint32_t sum = 0;

    for (uint32_t i = 0; i < size; i++)
    {
        sum = sum + p_data[i];
    }
    return 0xFFFFFFFF - sum;
}

uint32_t exmon_flash_init(void)
{
    uint32_t error_code = EXMON_FLASH_ERROR_NONE;
    uint8_t default_data[EXMON_FLASH_DEFAULT_CHECK_LENGTH] = {0};
    uint8_t read_data[EXMON_FLASH_DEFAULT_CHECK_LENGTH] = {0};
    memset(default_data, 0xFF, EXMON_FLASH_DEFAULT_CHECK_LENGTH);

    error_code = hal_exflash_read(EXMON_FLASH_START_ADDRESS + EXMON_FLASH_LEN_LENGTH, read_data, EXMON_FLASH_DEFAULT_CHECK_LENGTH);
    if (EXFLASH_ERROR_NONE != error_code)
    {
        return error_code;
    }

    if (0 != memcmp(read_data, default_data, EXMON_FLASH_DEFAULT_CHECK_LENGTH))
    {
        error_code = EXMON_FLASH_ERROR_INITIALIZED;
    }

    return error_code;
}

SECTION_RAM_CODE uint32_t exmon_flash_restore(void)
{
    if (HAL_TIMEOUT == hal_exflash_wait_busy(HAL_EXFLASH_RETRY_DEFAULT_VALUE))
    {
        return EXMON_FLASH_ERROR_TIMEOUT;
    }

    hal_exflash_deinit();

    return hal_exflash_init(NULL);
}

uint32_t exmon_flash_write(uint8_t *p_data, uint32_t size)
{
    uint32_t error_code = EXMON_FLASH_ERROR_NONE;
    uint8_t info_size_u8[EXMON_FLASH_LEN_LENGTH] = {0};
    uint8_t checksum_u8[EXMON_FLASH_CHECKSUM_LENGTH] = {0};
    uint32_t checksum = 0;

    if (size > (EXMON_FLASH_END_ADDRESS - EXMON_FLASH_START_ADDRESS - EXMON_FLASH_LEN_LENGTH - EXMON_FLASH_CHECKSUM_LENGTH) || (NULL == p_data))
    {
        return EXFLASH_ERROR_INVALID_PARAM;
    }

    info_size_u8[0] = size & 0xFF;
    info_size_u8[1] = (size >> 8) & 0xFF;
    info_size_u8[2] = (size >> 16) & 0xFF;
    info_size_u8[3] = (size >> 24) & 0xFF;

    error_code = hal_exflash_write(EXMON_FLASH_START_ADDRESS, info_size_u8, EXMON_FLASH_LEN_LENGTH);
    if (EXFLASH_ERROR_NONE != error_code)
    {
        return error_code;
    }

    checksum = exmon_flash_get_checksum(p_data, size);

    checksum_u8[0] = checksum & 0xFF;
    checksum_u8[1] = (checksum >> 8) & 0xFF;
    checksum_u8[2] = (checksum >> 16) & 0xFF;
    checksum_u8[3] = (checksum >> 24) & 0xFF;

    error_code = hal_exflash_write(EXMON_FLASH_END_ADDRESS - EXMON_FLASH_CHECKSUM_LENGTH, checksum_u8, EXMON_FLASH_CHECKSUM_LENGTH);
    if (EXFLASH_ERROR_NONE != error_code)
    {
        return error_code;
    }

    return hal_exflash_write(EXMON_FLASH_START_ADDRESS + EXMON_FLASH_LEN_LENGTH, p_data, size);
}

uint32_t exmon_flash_read(uint8_t *p_data, uint32_t *p_size)
{
    uint32_t error_code = EXMON_FLASH_ERROR_NONE;
    uint8_t info_size_u8[EXMON_FLASH_LEN_LENGTH] = {0};
    uint8_t checksum_u8[EXMON_FLASH_CHECKSUM_LENGTH] = {0};
    uint32_t checksum = 0;

    if (NULL == p_data || (NULL == p_size))
    {
        return EXFLASH_ERROR_INVALID_PARAM;
    }

    error_code = hal_exflash_read(EXMON_FLASH_START_ADDRESS, info_size_u8, EXMON_FLASH_LEN_LENGTH);
    if (EXFLASH_ERROR_NONE == error_code)
    {
        *p_size = info_size_u8[0] + (info_size_u8[1] << 8) + (info_size_u8[2] << 16) + (info_size_u8[3] << 24);
    }
    else
    {
        return error_code;
    }

    error_code = hal_exflash_read(EXMON_FLASH_START_ADDRESS + EXMON_FLASH_LEN_LENGTH, p_data, *p_size);
    if (EXFLASH_ERROR_NONE != error_code)
    {
        return error_code;
    }

    checksum = exmon_flash_get_checksum(p_data, *p_size);
    error_code = hal_exflash_read(EXMON_FLASH_END_ADDRESS - EXMON_FLASH_CHECKSUM_LENGTH, checksum_u8, EXMON_FLASH_CHECKSUM_LENGTH);
    if (EXFLASH_ERROR_NONE != error_code)
    {
        return error_code;
    }

    if (checksum == (checksum_u8[0] + (checksum_u8[1] << 8) + (checksum_u8[2] << 16) + (checksum_u8[3] << 24)))
    {
        return EXMON_FLASH_ERROR_NONE;
    }
    else
    {
        return EXMON_FLASH_ERROR_CHECKSUM;
    }
}

uint32_t exmon_flash_erase(void)
{
    return hal_exflash_erase(EXFLASH_ERASE_SECTOR, EXMON_FLASH_START_ADDRESS, EXMON_FLASH_END_ADDRESS - EXMON_FLASH_START_ADDRESS);
}

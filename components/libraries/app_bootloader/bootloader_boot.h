/**
 *****************************************************************************************
 *
 * @file bootloader_boot.h
 *
 * @brief Header file - BootLoader Boot API
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
#ifndef _BOOTLOADER_BOOT_H_
#define _BOOTLOADER_BOOT_H_

#include "gr_includes.h"

/**
 *****************************************************************************************
 * @brief Disable flash security
 *****************************************************************************************
 */
void security_disable(void);

/**
 *****************************************************************************************
 * @brief Recovery flash security
 *****************************************************************************************
 */
void security_state_recovery(void);

/**
 *****************************************************************************************
 * @brief Bootloader dfu task
 *****************************************************************************************
 */
void bootloader_dfu_task(void);

/**
 *****************************************************************************************
 * @brief Bootloader app firmware verify task
 *****************************************************************************************
 */
void bootloader_verify_task(void);

/**
 *****************************************************************************************
 * @brief Bootloader jump task
 *****************************************************************************************
 */
void bootloader_jump_task(void);

/**
 *****************************************************************************************
 * @brief Bootloader timeout task.
 * If DFU info exists and timeout occurs, the DFU_INFO is erased and the chip is reset.
 *****************************************************************************************
 */
void bootloader_timeout_task(void);

#endif



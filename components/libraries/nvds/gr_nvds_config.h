/**
 ******************************************************************************
 *
 * @file gr_nvds_config.h
 *
 * @brief NVDS Config file
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
 * @brief NVDS config.
 */

#ifndef GR_NVDS_CONFIG_H
#define GR_NVDS_CONFIG_H

/* Enable ENABLE_TAGS_CACHE to accelerate tag search.*/
#ifndef ENABLE_TAGS_CACHE
#define ENABLE_TAGS_CACHE          1  /* 1: enable, 0: disable */
#endif

#if ENABLE_TAGS_CACHE
#ifndef TAGS_CACHE_SIZE
#define TAGS_CACHE_SIZE            30U
#endif
#endif

#endif  /* GR_NVDS_CONFIG_H */

/** @} */
/** @} */

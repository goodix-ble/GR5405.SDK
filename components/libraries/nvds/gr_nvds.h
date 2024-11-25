/**
 ******************************************************************************
 *
 * @file gr_nvds.h
 *
 * @brief NVDS API
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

//lint -e537   [warning]  Repeated include file
//lint -e934   [required] Taking address of near auto variable
//lint -e9026  [advisory] Function-like macro
//lint -e970   [advisory] Use of modifier or type '_Bool' outside of a typedef
//lint -e904   [advisory] Return statement before end of function

/**
 * @addtogroup SYSTEM
 * @{
 */
 /**
 * @addtogroup NVDS Non-Volatile Data Storage
 * @{
 * @brief Definitions and prototypes for the NVDS interface.
 */

#ifndef GR_NVDS_H
#define GR_NVDS_H

#include <stdint.h>

/**
 * @addtogroup NVDS_DEFINES Defines
 * @{
 */
/* bit15 and bit14 of NVDS TAG are mask.*/
/* NVDS tag mask for BLE Stack */
#define NV_TAGCAT_STACK         0xC000
/* NVDS tag mask for SDK Bond Manager */
#define NV_TAGCAT_BNDMGR        0x8000
/**< NVDS tag mask for user application. */
#define NV_TAGCAT_APP           0x4000

/**< Get NVDS tag for user application.
The range of idx is 0x0000~0x3FFF. */
#define NV_TAG_APP(idx)         (NV_TAGCAT_APP | ((idx) & 0x3FFF))
#define APP_NV_TAGCAT_APP       NV_TAGCAT_APP
/** @} */

/**
 * @addtogroup NVDS_ENUMERATIONS Enumerations
 * @{
 */
/**@brief NVDS Work Status. */
typedef enum
{
    NVDS_STATE_RESET   = 0x00U,  /**< NVDS is not initialized. */
    NVDS_STATE_READY   = 0x1U,   /**< NVDS initialized and ready for use. */
    NVDS_STATE_BUSY    = 0x20U,  /**< An internal process is ongoing. */
    NVDS_STATE_BUSY_GC = 0x21U   /**< GC process is ongoing. */
} nvds_state_t;

/**@brief NVDS Returned Status. */
typedef enum
{
    NVDS_SUCCESS = 0x00U,        /**< NVDS succeeds. */
    NVDS_GC_COMPLETE,            /**< NVDS garbage collection complete. */
    NVDS_FAIL,                   /**< NVDS failed. */
    NVDS_NOT_INIT,               /**< NVDS not initialize. */
    NVDS_TAG_NOT_EXISTED,        /**< NVDS tag does not exist. */
    NVDS_SPACE_NOT_ENOUGH,       /**< NVDS space is not enough. */
    NVDS_LENGTH_OUT_OF_RANGE,    /**< NVDS length out of range. */
    NVDS_INVALID_PARA,           /**< NVDS invalid params. */
    NVDS_INVALID_START_ADDR,     /**< NVDS invalid start address. */
    NVDS_INVALID_SECTORS,        /**< NVDS invalid sector. */
    NVDS_COMPACT_FAILED,         /**< NVDS failed to compact sectors. */
    NVDS_STORAGE_ACCESS_FAILED , /**< NVDS failed to access storage. */
    NVDS_POINTER_NULL,           /**< NVDS or driver function replace error. */
    NVDS_TAG_DATA_ERROR,         /**< NVDS tag data verify error. */
    NVDS_BUSY                    /**< NVDS is busy. */
} nvds_err_t;

/** @} */

/**
 * @addtogroup NVDS_TYPEDEFS Type Typedefs
 * @{
 */
/**@brief NVDS Item tag. */
typedef uint16_t NvdsTag_t;
/** @} */

/**
 * @addtogroup NVDS_STRUCTURES Enumerations
 * @{
 */

/**
 * @addtogroup NVDS_FUNCTIONS Functions
 * @{
 */
/**
 ****************************************************************************************
 * @brief Initialize the sectors for NVDS.
 *
 * @note NVDS module will use one more sector flash for garbage collection. For example,
 * sectors=1, NVDS area actually use 2 sector(one sector for garbage collection)
 *
 * @param[in] start_addr: Start address of NVDS area. If the value does not equal zero,
 * it must be sector-aligned. If the value equals zero, NVDS area will locate in the last
 * several sectors in flash memory.
 * @param[in] sectors: The number of sectors.
 *
 * @return Result of nvds init.
 ****************************************************************************************
 */
nvds_err_t nvds_init(uint32_t start_addr, uint8_t sectors);

/**
 ****************************************************************************************
 * @brief De-initialize the sectors for NVDS.
 *
 * @note nvds_deinit will erase the flash sectors.
 *
 * @param[in] start_addr: Start address of NVDS area. If the value does not equal zero,
 * it must be sector-aligned. If the value equals zero, NVDS area will locate in the last
 * several sectors in flash memory.
 * @param[in] sectors: The number of sectors.
 *
 * @return Result of nvds deinit.
 ****************************************************************************************
 */
nvds_err_t nvds_deinit(uint32_t start_addr, uint8_t sectors);

/**
 ****************************************************************************************
 * @brief Read data from NVDS.
 * NOTE: Data will be copy to p_buf to verify, if the data verification fails,
 *       there will be dirty data in p_buf.
 * @param[in]       tag: Valid NVDS item tag.
 * @param[in,out] p_len: Pointer to the length of data.
 * @param[out]    p_buf: Data is read into the buffer.
 *
 * @return Result of nvds get.
 ****************************************************************************************
 */
nvds_err_t nvds_get(NvdsTag_t tag, uint16_t *p_len, uint8_t *p_buf);

/**
 ****************************************************************************************
 * @brief Write data to NVDS. If the tag does not exist, create one.
 *
 * @param[in] tag:   Valid NVDS item tag.
 * @param[in] len:   Length of data to be written.
 * @param[in] p_buf: Data to be written.
 *
 * @return Result of nvds put.
 ****************************************************************************************
 */
nvds_err_t nvds_put(NvdsTag_t tag, uint16_t len, const uint8_t *p_buf);

/**
 ****************************************************************************************
 * @brief Delete a tag in NVDS
 *
 * @param[in] tag: The tag to be deleted.
 *
 * @return Result of nvds delete.
 ****************************************************************************************
 */
nvds_err_t nvds_del(NvdsTag_t tag);

/**
 ****************************************************************************************
 * @brief NVDS garbage collection.
 *
 * @return Result of nvds garbage collection.
 ****************************************************************************************
 */
nvds_err_t nvds_gc(void);

/**
 ****************************************************************************************
 * @brief NVDS garbage collection start callback
 *
 * @return None.
 ****************************************************************************************
 */
void nvds_gc_start_callback(void);

/**
 ****************************************************************************************
 * @brief NVDS garbage collection end callback
 *
 * @return None.
 ****************************************************************************************
 */
void nvds_gc_end_callback(void);

/**
 ****************************************************************************************
 * @brief Get the length of a tag in NVDS
 *
 * @param[in] tag: The tag to get the length.
 *
 * @return Length of the tag data. If tag does not exist, return 0.
 ****************************************************************************************
 */
uint16_t nvds_tag_length(NvdsTag_t tag);

/**
 ****************************************************************************************
 * @brief Get the work state of NVDS
 *
 * @return Work state of NVDS.
 ****************************************************************************************
 */
nvds_state_t nvds_get_state(void);

/**
 ****************************************************************************************
 * @brief Write data to NVDS as a sub tag. If the main tag or sub tag does not exist, create one.
 *
 * @param[in] main_tag:  Valid NVDS item tag.
 * @param[in] sub_tag:   Valid NVDS item tag.
 * @param[in] len:   Length of data to be written.
 * @param[in] p_buf: Data to be written.
 *
 * @return Result of nvds put.
 ****************************************************************************************
 */
nvds_err_t nvds_put_sub_tag(NvdsTag_t main_tag, NvdsTag_t sub_tag, uint16_t len, const uint8_t *p_buf);

/**
 ****************************************************************************************
 * @brief Read the sub tag data from NVDS.
 *
 * @param[in]  main_tag: Valid NVDS item tag.
 * @param[in]   sub_tag: Valid NVDS item tag.
 * @param[in,out] p_len: Pointer to the length of data.
 * In, the length of p_buf. Out, the data length of sub tag
 * @param[out]    p_buf: Data is read into the buffer.
 *
 * @return Result of nvds get.
 ****************************************************************************************
 */
nvds_err_t nvds_get_sub_tag(NvdsTag_t main_tag, NvdsTag_t sub_tag, uint16_t *p_len, uint8_t *p_buf);

/**
 ****************************************************************************************
 * @brief Delete a sub tag in NVDS
 *
 * @param[in] main_tag:  Valid NVDS item tag.
 * @param[in] sub_tag:   Valid NVDS item tag.
 *
 * @return Result of nvds delete.
 ****************************************************************************************
 */
nvds_err_t nvds_del_sub_tag(NvdsTag_t main_tag, NvdsTag_t sub_tag);

/**
 ****************************************************************************************
 * @brief Get the length of a sub tag in NVDS
 *
 * @param[in] main_tag:  Valid NVDS tag.
 * @param[in] sub_tag:   Valid NVDS tag.
 *
 * @return Length of sub tag data. If sub tag does not exist, return 0.
 ****************************************************************************************
 */
uint16_t nvds_sub_tag_length(NvdsTag_t main_tag, NvdsTag_t sub_tag);

/**
 ****************************************************************************************
 * @brief Get the length of available space
 * NOTE: Available space is the maximum space currently available in NVDS.
 * @return Length of available space.
 ****************************************************************************************
 */
uint32_t nvds_get_avail_size(void);

/**
 ****************************************************************************************
 * @brief Get the length of empty space
 * NOTE: The empty space is space that NVDS can use when it does not need GC processing.
 * @return Length of empty space.
 ****************************************************************************************
 */
uint32_t nvds_get_empty_size(void);

/** @} */
/** @} */

#endif /* GR_NVDS_H */

/** @} */
/** @} */

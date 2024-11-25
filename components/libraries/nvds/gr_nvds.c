/**
 ****************************************************************************************
 *
 * @file gr_nvds.c
 *
 * @brief NVDS API
 *
 ****************************************************************************************
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
 ****************************************************************************************
 */
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stddef.h>
#include <string.h>

#include "gr_nvds_config.h"
#include "gr_nvds_port.h"

/*
                                NVDS Map
+--------------------------------------------------------- ... ----------------+
|           |    page 0    |    page 1    |    page 2    | ... |   page15      |
+-----------+--------------+--------------+--------------+ ... +---------------+
|sector 0   |  NVDS header |    item 0    |    item 1    | ... |   item 14     |
+-----------+--------------+--------------+--------------+ ... +---------------+
|sector 1   |  NVDS header |    item 15   |    item 16   | ... |   item 29     |
+-----------+--------------+--------------+--------------+ ... +---------------+
...
+-----------+--------------+--------------+--------------+ ... +---------------+
|sector N   |  NVDS header |    item 15N  |    item 15+1 | ... |   item 15N+14 |
+-----------+--------------+--------------+--------------+ ... +---------------+
|last sector|  GC info     |    GC item 0 |    GC item 1 | ... |   GC item 14  |
+-----------+--------------+--------------+--------------+ ... +---------------+
*/

/*
 * DEFINE
 ****************************************************************************************
 */
#define NVDS_DEBUG_ENABLE            0
#if NVDS_DEBUG_ENABLE
#define NVDS_ERROR_HANDLER()         printf("NVDS_ERROR func:%s, line:%d\r\n", __FUNCTION__, __LINE__);
#else
#define NVDS_ERROR_HANDLER()
#endif

#define NVDS_VERSION_PATTERN         0x47525859U /* Just a magic number */
#define NVDS_VERSION_MAJOR           0x01U
#define NVDS_VERSION_MINOR           0x01U
#define NVDS_VERSION_BUILD           0x00U

#define NVDS_MAGIC_LEN               8U
#define NVDS_FLASH_PAGE_LEN          256U
#define NVDS_FLASH_DEFAULT_VALUE     0xFF
#define NVDS_ITEM_TAG_END            0xFFFFU
#define NVDS_ITEM_TAG_DEL            0x0U
#define NVDS_ITEM_TAG_VALID_VALID    0x2U
#define NVDS_ITEM_TAG_VALID_DEL      0x0U
#define NVDS_ITEM_TAG_ANY            0x0U
#define NVDS_ITEM_INDEX_ANY          0x0U
#define NVDS_ITEM_INDEX_FIRST        0x1U
#define NVDS_ITEM_HAVE_SUB_TAG       true
#define NVDS_ITEM_NO_SUB_TAG         false
#define NVDS_ITEM_HDR_LEN            sizeof(item_header_t)
#define NVDS_ITEM_LEN                NVDS_FLASH_PAGE_LEN
#define NVDS_MAX_ITEM_DATA_LEN       (NVDS_ITEM_LEN - NVDS_ITEM_HDR_LEN)
#define NVDS_ITEM_PER_SECTOR         (15U)
#define NVDS_MAX_TAG_DATA_LEN        (NVDS_ITEM_PER_SECTOR * NVDS_MAX_ITEM_DATA_LEN)   /* 3720 */

#define GC_BUFFER_VALID              0xA5U
#define GC_BUFFER_INVALID            0x00U

#define ALIGN_UP(addr, align)        (((addr) + (align) - 1) & ~((align) - 1))
#define UP_ALIGN_SECTOR(addr)        ALIGN_UP(addr, NVDS_SECTOR_SIZE)
#define IS_ALIGNED_SECTOR(addr)      (0U == ((addr) & (NVDS_SECTOR_SIZE - 1U)))
#define CEIL_DIV(x, y)               (((x) + (y) - 1U) / (y))
#define FLOOR_ALIGN_SECTOR(addr)     ((addr) & ~(NVDS_SECTOR_SIZE - 1U))
#define ADDR_TO_SECTOR_INDEX(addr)   ((addr) & ~(NVDS_SECTOR_SIZE - 1U))
#define TO_U8_PTR(ptr)               ((const uint8_t *)(ptr))
#define NVDS_GC_ADDR(__HANDLE__)     ((__HANDLE__).start_addr + (__HANDLE__).sectors * NVDS_SECTOR_SIZE)
#define GC_INFO_PATTERN_TYPE         uint8_t
//lint -e9007 [required] NO side effects of nvds_port_read
#define NVDS_LOCK(__HANDLE__)                                          \
do {                                                                   \
    uint32_t __l_locker = nvds_port_lock();                            \
    uint32_t __l_irq_status = NVIC_GetEnableIRQ(BLE_SDK_IRQn);         \
    uint8_t __l_arr[1];                                                \
    if ((0 != ((uint8_t)(__HANDLE__).state & (uint8_t)NVDS_STATE_BUSY))\
        || (sizeof(__l_arr) != nvds_port_read(EXFLASH_START_ADDR,      \
                                        __l_arr, sizeof(__l_arr))))    \
    {                                                                  \
        NVDS_ERROR_HANDLER();                                          \
        nvds_port_unlock(__l_locker);                                  \
        return NVDS_BUSY;                                              \
    }                                                                  \
    NVIC_DisableIRQ(BLE_SDK_IRQn);                                     \
    (__HANDLE__).state = NVDS_STATE_BUSY;                              \
    nvds_port_unlock(__l_locker)

#define NVDS_UNLOCK(__HANDLE__)                                        \
    ((__HANDLE__).state = NVDS_STATE_READY);                           \
    if (__l_irq_status)                                                \
    {                                                                  \
        NVIC_EnableIRQ(BLE_SDK_IRQn);                                  \
    }                                                                  \
} while(0)

/*
 * STRUCT DEFINE
 ****************************************************************************************
 */
/* NVDS version structure */
typedef struct __PACKED
{
    uint32_t pattern;
    uint8_t major;
    uint8_t minor;
    uint8_t build;
    uint8_t reserved;
} nvds_version_t;

/* NVDS header sector information structure */
typedef struct __PACKED
{
    uint8_t index;
    uint8_t total;
    uint8_t checksum;
    uint8_t reserved[5];
} nvds_sector_info_t;

/* NVDS header structure */
typedef struct __PACKED
{
    uint8_t magic[NVDS_MAGIC_LEN];
    nvds_version_t version;
    nvds_sector_info_t sector_info;
} nvds_header_t;

/* NVDS item header misc information structure */
typedef struct __PACKED
{
    //lint -e46 [error] uint8_t as field type is OK
    uint8_t tag_valid:2;          /* Tag validity, a tag is valid only when tag_valid is equal to NVDS_ITEM_TAG_VALID_VALID */
    uint8_t exist_sub_tag:1;      /* Indicates whether a sub tag exists */
    uint8_t reserved:5;
} item_misc_t;

/* NVDS item header part information struct */
typedef struct __PACKED
{
    uint8_t index:4;              /* Item index, if only one, index = 1 */
    uint8_t total:4;              /* Total item, if only one, total = 1 */
} part_info_t;

/* NVDS item header structure */
typedef struct __PACKED
{
    NvdsTag_t   tag;               /* Tag ID */
    uint16_t    len;               /* Length of Tag data */
    part_info_t part;              /* Item total and index information */
    uint8_t     header_checksum;   /* Checksum of item Header */
    item_misc_t misc;              /* Item miscellaneous information*/
    uint8_t     data_checksum;     /* Checksum of item data */
} item_header_t;

/* NVDS item structure */
typedef struct __PACKED
{
    item_header_t item_header;
    uint8_t data[NVDS_MAX_ITEM_DATA_LEN];
} nvds_item_t;

/* NVDS GC info structure */
typedef struct __PACKED
{
    GC_INFO_PATTERN_TYPE pattern; /* GC info pattern */
    uint8_t  r_sector;            /* Read sector index */
    uint8_t  w_sector;            /* Write sector index */
    uint8_t  r_page;              /* Read sector page index */
    uint8_t  item_num;            /* Item number in GC */
    uint8_t  checksum;            /* GC info checksum */
    uint8_t  reserved[2];         /* Reserved */
} nvds_gc_info_t;

#if NVDS_DEBUG_ENABLE
/* NVDS sector structure */
typedef struct __PACKED
{
    nvds_header_t nvds_header;
    uint8_t reserved[NVDS_FLASH_PAGE_LEN - sizeof(nvds_header_t)];
    nvds_item_t item[NVDS_ITEM_PER_SECTOR];
} nvds_sector_t;

/* GC sector structure */
typedef struct __PACKED
{
    nvds_gc_info_t gc_info;
    uint8_t reserved[NVDS_FLASH_PAGE_LEN - sizeof(nvds_gc_info_t)];
    nvds_item_t item[NVDS_ITEM_PER_SECTOR];
} gc_sector_t;
#endif

#if ENABLE_TAGS_CACHE
/* Tags cache record structure */
typedef struct
{
    uint8_t    item_index;     /* Item index */
    uint16_t   tag;            /* Tag id */
    uint32_t   item_address;   /* Item address */
} tag_cache_rec_t;
#endif

/* NVDS handle structure */
typedef struct
{
    nvds_state_t    state;                          /* NVDS Work state. */
    uint8_t         sectors;                        /* The number of sectors of NVDS area. */
#if ENABLE_TAGS_CACHE
    uint8_t         cache_rec_num;                  /* The number of available records in the cache. */
    tag_cache_rec_t cache_tags[TAGS_CACHE_SIZE];    /* The cache of tags. */
#endif
    uint32_t        start_addr;                     /* NVDS start address. */
    uint32_t        empty_addr;                     /* Point to the start of empty space in NVDS. */
    int32_t         avail_size;                     /* Available space of NVDS */
    int32_t         empty_size;                     /* Continuous empty space at the end of NVDS. */
} nvds_handle_t;

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
static const char NVDS_MAGIC[NVDS_MAGIC_LEN] = {'G', 'D', 'X', '_', 'N', 'V', 'D', 'S'};
static const nvds_version_t NVDS_VERSION = {
    .pattern  = NVDS_VERSION_PATTERN,
    .major    = NVDS_VERSION_MAJOR,
    .minor    = NVDS_VERSION_MINOR,
    .build    = NVDS_VERSION_BUILD,
    .reserved = NVDS_FLASH_DEFAULT_VALUE
};
static nvds_handle_t s_nvds_handle;

/*
 * LOCAL FUNCTION DECLARATION
 ****************************************************************************************
 */
static nvds_err_t gc_start(void);

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if ENABLE_TAGS_CACHE
/**
 ****************************************************************************************
 * @brief Clean all tags cache.
 *
 ****************************************************************************************
 */
static void tags_cache_clean(void)
{
    memset(&s_nvds_handle.cache_tags[0], 0x00, (sizeof(tag_cache_rec_t) * TAGS_CACHE_SIZE));
    s_nvds_handle.cache_rec_num = 0;
}

/**
 ****************************************************************************************
 * @brief Add a record to tags cache.
 * NOTE: If cache is full, no more records will be added.
 * @param[in] p_item_hdr: Pointer to item header.
 * @param[in]   tag_addr: Tag address in flash.
 *
 ****************************************************************************************
 */
static void tags_cache_rec_add(const item_header_t *p_item_hdr, uint32_t tag_addr)
{
    if (s_nvds_handle.state == NVDS_STATE_BUSY_GC)
    {
        return;
    }
    if (s_nvds_handle.cache_rec_num < (TAGS_CACHE_SIZE - 1U))
    {
        for (uint32_t i = 0; i < s_nvds_handle.cache_rec_num; i++)
        {
            /* If item is already in cache, update its address. */
            if ((s_nvds_handle.cache_tags[i].tag == p_item_hdr->tag)
            && (s_nvds_handle.cache_tags[i].item_index == p_item_hdr->part.index))
            {
                s_nvds_handle.cache_tags[i].item_address = tag_addr;
                return;
            }
        }
        tag_cache_rec_t *p_rec = &s_nvds_handle.cache_tags[s_nvds_handle.cache_rec_num];
        p_rec->tag         = p_item_hdr->tag;
        p_rec->item_index  = p_item_hdr->part.index;
        p_rec->item_address = tag_addr;
        s_nvds_handle.cache_rec_num++;
    }
}

/**
 ****************************************************************************************
 * @brief Delete a record in tag cache.
 *
 * @param[in] tag: Tag ID to be deleted.
 *
 ****************************************************************************************
 */
static void tags_cache_rec_del(NvdsTag_t tag)
{
    tag_cache_rec_t *p_cache = s_nvds_handle.cache_tags;
    uint8_t         rec_num = s_nvds_handle.cache_rec_num;
    /* Search tag to be deleted */
    for (uint32_t i = 0; i < rec_num; i++)
    {
        if (p_cache[i].tag == tag)
        {
            s_nvds_handle.cache_rec_num--;
            /* Move all tags after the tag to be deleted forward one position */
            for (uint32_t j = i; j < (rec_num - 1U); j++)
            {
                p_cache[j] = p_cache[j + 1U];
            }
            /* Delete the last record which move forward */
            memset(&p_cache[rec_num - 1U], 0x00, sizeof(tag_cache_rec_t));
            break;
        }
    }
}

/**
 ****************************************************************************************
 * @brief Find a tag in tag cache.
 *
 * @param[in] tag: Tag ID to find.
 * @param[in,out] p_tag_addr: Pointer of item address.
 * @param[in] item_index: Item index to find
 *
 * @return True or false.
 ****************************************************************************************
 */
static bool tags_cache_rec_find(NvdsTag_t tag, uint32_t *p_tag_addr, uint8_t item_index)
{
    tag_cache_rec_t *p_cache = s_nvds_handle.cache_tags;
    uint8_t          rec_num = s_nvds_handle.cache_rec_num;
    /* Cache not used in GC process */
    if (s_nvds_handle.state == NVDS_STATE_BUSY_GC)
    {
        return false;
    }
    if (0U == rec_num)
    {
        return false;
    }

    if ((NVDS_ITEM_TAG_ANY == tag) && (*p_tag_addr <= p_cache[0].item_address))
    {
        *p_tag_addr = p_cache[0].item_address;
        return true;
    }

    for (uint8_t i = 0; i < rec_num; i++)
    {
        if ((p_cache[i].tag == tag) &&
            ((NVDS_ITEM_INDEX_ANY == item_index) || (p_cache[i].item_index == item_index)) &&
            (*p_tag_addr <= p_cache[i].item_address))
        {
            *p_tag_addr = p_cache[i].item_address;
            return true;
        }
    }
    return false;
}
#endif /* ENABLE_TAGS_CACHE */

/**
 ****************************************************************************************
 * @brief NVDS port: Read flash.
 *
 * @param[in]  addr: Flash read address.
 * @param[in] p_buf: Pointer to the data.
 * @param[in]  size: Read size.
 *
 ****************************************************************************************
 */
static void nvds_port_read_no_check(const uint32_t addr, uint8_t *buf, const uint32_t size)
{
    if (size != nvds_port_read(addr, buf, size))
    {
        NVDS_ERROR_HANDLER();
    }
}

/**
 ****************************************************************************************
 * @brief NVDS port: Write flash.
 *
 * @param[in]  addr: Flash write address.
 * @param[in] p_buf: Pointer to the data.
 * @param[in]  size: Length of p_buf.
 *
 ****************************************************************************************
 */
static void nvds_port_write_check(const uint32_t addr, const uint8_t *p_buf, const uint32_t size)
{
    if (size != nvds_port_write_r(addr, p_buf, size))
    {
         NVDS_ERROR_HANDLER();
    }
}

/**
 ****************************************************************************************
 * @brief NVDS port: Erase flash.
 *
 * @param[in]  addr: erasing star address.
 * @param[in]  size: erasing size.
 *
 ****************************************************************************************
 */
static void nvds_port_erase_no_check(const uint32_t addr, const uint32_t size)
{
    if (!nvds_port_erase(addr, size))
    {
         NVDS_ERROR_HANDLER();
    }
}

/**
 ****************************************************************************************
 * @brief Calculate 8-bit checksum.
 *
 * @param[in] p_data: Pointer to the data to be calculated.
 * @param[in]    len: Length of p_data.
 *
 * @return 8-bit checksum of data.
 ****************************************************************************************
 */
static uint8_t calc_8bit_checksum(const uint8_t* p_data, uint32_t len)
{
    uint8_t checksum = 0;
    for (uint32_t i = 0; i < len; i++)
    {
        checksum += p_data[i];
    }
    return checksum;
}

/**
 ****************************************************************************************
 * @brief Write NVDS header.
 *
 * @param[in] p_nvds_header: Pointer of NVDS header to write.
 *
 ****************************************************************************************
 */
static void write_nvds_header(const nvds_header_t *p_nvds_header)
{
    uint32_t addr = s_nvds_handle.start_addr + ((p_nvds_header->sector_info.index - 1U) * NVDS_SECTOR_SIZE);
    nvds_port_write_check(addr, (const uint8_t *)p_nvds_header, sizeof(nvds_header_t));
}

/**
 ****************************************************************************************
 * @brief Calculate the data length of a single item.
 *
 * @param[in] p_item_header: Pointer of item header.
 *
 * @return Length of item data
 ****************************************************************************************
 */
static uint8_t calc_single_item_data_len(const item_header_t *p_item_header)
{
    uint8_t item_data_len = 0;
    uint8_t total = p_item_header->part.total;
    if (p_item_header->part.index == total)
    {
        //lint -e734  [Info] There is no precision loss
        //lint -e9034 [required] Expression assigned to a narrower or different essential type
        /* The last or only one item */
        item_data_len = p_item_header->len - (NVDS_MAX_ITEM_DATA_LEN * (total - 1U));
    }
    else
    {
        item_data_len = NVDS_MAX_ITEM_DATA_LEN;
    }

    if (item_data_len > NVDS_MAX_ITEM_DATA_LEN)
    {
        NVDS_ERROR_HANDLER();
        return 0;
    }

    return item_data_len;
}

/**
 ****************************************************************************************
 * @brief Write a single item to NVDS.
 *
 * @param[in]     item_addr: Write address of item.
 * @param[in] p_item_header: Pointer of item header to be written.
 * @param[in]        p_data: Pointer of ite data to be written.
 *
 ****************************************************************************************
 */
static void write_single_item(const uint32_t item_addr, const item_header_t *p_item_header, const uint8_t *p_data)
{
    const uint32_t nvds_end_addr = s_nvds_handle.start_addr + s_nvds_handle.sectors * NVDS_SECTOR_SIZE;
    uint8_t nvds_buf[NVDS_ITEM_LEN];
    memset(nvds_buf, NVDS_FLASH_DEFAULT_VALUE, sizeof(nvds_buf));
    memcpy(nvds_buf, p_item_header, sizeof(item_header_t));

    uint8_t item_data_len = calc_single_item_data_len(p_item_header);
    memcpy(&nvds_buf[sizeof(item_header_t)], p_data, item_data_len);
    uint32_t write_len = item_data_len + sizeof(item_header_t);
    nvds_port_write_check(item_addr, nvds_buf, write_len);

    s_nvds_handle.empty_addr += NVDS_ITEM_LEN;
    if (IS_ALIGNED_SECTOR(s_nvds_handle.empty_addr))
    {
        s_nvds_handle.empty_addr += NVDS_ITEM_LEN;
    }
    if (s_nvds_handle.empty_addr > nvds_end_addr)
    {
        s_nvds_handle.empty_addr = nvds_end_addr;
    }

    //lint -e713 -e737 [Info] there is no precision loss
    //lint -e9029 [required] Mismatched essential type categories for binary operator
    s_nvds_handle.avail_size -= NVDS_MAX_ITEM_DATA_LEN;
    s_nvds_handle.empty_size -= NVDS_MAX_ITEM_DATA_LEN;
#if ENABLE_TAGS_CACHE
    tags_cache_rec_add(p_item_header, item_addr);
#endif
}

/**
 ****************************************************************************************
 * @brief Delete a single item.
 * Note: Just write item_header->misc.tag_valid to NVDS_ITEM_TAG_VALID_DEL
 * @param[in]     item_addr: Address of item.
 * @param[in] p_item_header: Pointer of item header to be delete.
 *
 ****************************************************************************************
 */
static void del_single_item(uint32_t item_addr, item_header_t *p_item_header)
{
    p_item_header->misc.tag_valid = NVDS_ITEM_TAG_VALID_DEL;
    //lint -e718 -e746 [Info] offsetof is a STD define
    //lint -e413 -e516 [required] offsetof is a STD define
    uint32_t addr = item_addr + offsetof(item_header_t, misc);
    nvds_port_write_check(addr, (uint8_t *)&(p_item_header->misc), sizeof(p_item_header->misc));
    s_nvds_handle.avail_size += NVDS_MAX_ITEM_DATA_LEN;
#if ENABLE_TAGS_CACHE
    tags_cache_rec_del(p_item_header->tag);
#endif
}

/**
 ****************************************************************************************
 * @brief Verify item header validity and checksum
 *
 * @param[in] p_item_header: Pointer of item header to verify.
 *
 * @return True or false.
 ****************************************************************************************
 */
static bool item_header_verify(const item_header_t *p_item_header)
{
    if (NVDS_ITEM_TAG_VALID_VALID == p_item_header->misc.tag_valid)
    {
        if (p_item_header->header_checksum ==
            calc_8bit_checksum((const uint8_t *)p_item_header, offsetof(item_header_t, header_checksum)))
        {
            return true;
        }
    }
    return false;
}

/**
 ****************************************************************************************
 * @brief Verify item data checksum
 *
 * @param[in] header_addr: Item header address.
 * @param[in] p_item_header: Pointer of item header.
 * @param[in] data_buf: Pointer of item data.
 *
 * @return True or false.
 ****************************************************************************************
 */
static bool item_data_verify(uint32_t header_addr, const item_header_t *p_item_header, uint8_t *data_buf)
{
    uint8_t item_data_len = calc_single_item_data_len(p_item_header);
    nvds_port_read_no_check(header_addr + sizeof(item_header_t), data_buf, item_data_len);
    if (p_item_header->data_checksum == calc_8bit_checksum(data_buf, item_data_len))
    {
        return true;
    }
    return false;
}

/**
 ****************************************************************************************
 * @brief Find item header address
 *
 * @param[in] start_addr: Starting search address.
 * @param[in] tag: Tag id to be searched.
 * @param[in] item_index: item index to be searched.
 * @param[in, out] p_item_header: Pointer of item header.
 *
 * @return Item header address.
 ****************************************************************************************
 */
static uint32_t find_item_header(uint32_t start_addr, NvdsTag_t tag, uint8_t item_index, item_header_t *p_item_header)
{
    uint32_t item_addr = 0;
    memset(p_item_header, 0, sizeof(item_header_t));
    const uint32_t end_addr = s_nvds_handle.empty_addr;
    /* Ignore NVDS_Header */
    if (0 == (start_addr % NVDS_SECTOR_SIZE))
    {
        start_addr += NVDS_ITEM_LEN;
    }

    while (start_addr < end_addr)
    {
        nvds_port_read_no_check(start_addr, (uint8_t *)p_item_header, sizeof(item_header_t));
        /* If tag = 0, not check tag */
        if ((tag == NVDS_ITEM_TAG_ANY) || (tag == p_item_header->tag))
        {
            /* If item_index = 0, not check item_index */
            if ((item_index == NVDS_ITEM_INDEX_ANY) || (item_index == p_item_header->part.index))
            {
                if (item_header_verify(p_item_header))
                {
                    item_addr = start_addr;
                    break;
                }
            }
        }
        start_addr += NVDS_ITEM_LEN;
        /* Ignore NVDS_Header */
        if (0 == (start_addr % NVDS_SECTOR_SIZE))
        {
            start_addr += NVDS_ITEM_LEN;
        }
    }
    return item_addr;
}

/**
 ****************************************************************************************
 * @brief Find item header address in cache firstly, if not in cache, search in NVDS.
 *
 * @param[in] start_addr: Starting search address.
 * @param[in] tag: Tag id to be searched.
 * @param[in] item_index: item index to be searched.
 * @param[in, out] p_item_header: Pointer of item header.
 *
 * @return Item header address.
 ****************************************************************************************
 */
static uint32_t find_item_header_with_cache(uint32_t start_addr, NvdsTag_t tag, uint8_t item_index, item_header_t *p_item_header)
{
#if ENABLE_TAGS_CACHE
    uint32_t cache_tag_addr = start_addr;
    if (tags_cache_rec_find(tag, &cache_tag_addr, item_index))
    {
        memset(p_item_header, 0, sizeof(item_header_t));
        nvds_port_read_no_check(cache_tag_addr, (uint8_t *)p_item_header, sizeof(item_header_t));
        if (item_header_verify(p_item_header))
        {
            return cache_tag_addr;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return find_item_header(start_addr, tag, item_index, p_item_header);
    }
#else
    return find_item_header(start_addr, tag, item_index, p_item_header);
#endif
}

/**
 ****************************************************************************************
 * @brief Check if item is empty(All bytes are the content after flash erase)
 *
 * @param[in] item_addr: Item address in flash.
 *
 * @return True or false
 ****************************************************************************************
 */
static bool is_empty_item(uint32_t item_addr)
{
    uint8_t item_buf[NVDS_ITEM_LEN];
    memset(&item_buf[0], 0, sizeof(item_buf));
    nvds_port_read_no_check(item_addr, &item_buf[0], 1);
    if (NVDS_FLASH_DEFAULT_VALUE == item_buf[0])
    {
        nvds_port_read_no_check(item_addr, &item_buf[1], NVDS_ITEM_LEN - 1);
        for (uint32_t i = 1; i < NVDS_ITEM_LEN; i++)
        {
            if (NVDS_FLASH_DEFAULT_VALUE != item_buf[i])
            {
                return false;
            }
        }
    }
    else
    {
        return false;
    }
    return true;
}

/**
 ****************************************************************************************
 * @brief Delete a tag
 *
 * @param[in] tag: Tag ID.
 *
 * @return Result of tag deletion
 ****************************************************************************************
 */
static nvds_err_t del_tag(NvdsTag_t tag)
{
    nvds_err_t ret = NVDS_SUCCESS;
    uint32_t item_header_addr;
    uint32_t start_addr = s_nvds_handle.start_addr;
    item_header_t item_header;
    do
    {
        item_header_addr = find_item_header_with_cache(start_addr, tag, NVDS_ITEM_INDEX_ANY, &item_header);
        if (item_header_addr)
        {
            del_single_item(item_header_addr, &item_header);
            start_addr = item_header_addr + NVDS_ITEM_LEN;
        }
    } while (item_header_addr);

    if (start_addr == s_nvds_handle.start_addr)
    {
        ret = NVDS_TAG_NOT_EXISTED;
    }
    return ret;
}

/**
 ****************************************************************************************
 * @brief Delete the specified tag in the range of [start_addr, end_addr).
 *
 * @param[in] p_item_header: Pointer of item header
 * @param[in] start_addr: start address
 * @param[in] end_addr: end address
 *
 ****************************************************************************************
 */
static void del_tag_in_range(item_header_t *p_item_header, uint32_t start_addr, uint32_t end_addr)
{
    item_header_t item_header;
    uint8_t item_index = p_item_header->part.total;

    while (item_index)
    {
        /* Delete tags in reverse order by item_index to avoid the chip reset during the deletion process, which may
         * cause incomplete deletion. */
        uint32_t item_addr = find_item_header(start_addr, p_item_header->tag, item_index, &item_header);
        if ((0U != item_addr) && (item_addr < end_addr))
        {
            del_single_item(item_addr, &item_header);
        }
        item_index--;
    };
}

/**
 ****************************************************************************************
 * @brief Delete duplicate tag during NVDS initialization
 * NOTE: When replacing a tag, new data is written first, followed by the deletion of
 * the old data. The new data will be placed at the end of NVDS. If a chip reset occurs
 * during the tag replacement, there may be two copies of data for the tag.
 ****************************************************************************************
 */
static void del_duplicate_tag(void)
{
    item_header_t old_item_header;
    item_header_t new_item_header;
    uint32_t start_addr = s_nvds_handle.empty_addr - NVDS_ITEM_LEN;
    uint32_t new_item_addr = 0;
    uint32_t old_item_addr = 0;
    bool found_duplicate = false;

    /* Search for the last NVDS_ITEM_INDEX_FIRST item address from back to front */
    while (start_addr > s_nvds_handle.start_addr)
    {
        new_item_addr = find_item_header(start_addr, NVDS_ITEM_TAG_ANY, NVDS_ITEM_INDEX_FIRST, &new_item_header);
        if (new_item_addr)
        {
            /* Find if there is a duplicate tag */
            old_item_addr = find_item_header(s_nvds_handle.start_addr, new_item_header.tag,
                                              NVDS_ITEM_INDEX_FIRST, &old_item_header);
            if (0U != old_item_addr && (old_item_addr != new_item_addr))
            {
                found_duplicate = true;
            }
            break;
        }
        start_addr -= NVDS_ITEM_LEN;
    }

    if (found_duplicate)
    {
        uint8_t item_data[NVDS_MAX_ITEM_DATA_LEN]; /* Just a buffer for data verification */
        item_header_t temp_item_header;
        //lint -e772 [info] new_item_header has been initialized when running here.
        memcpy(&temp_item_header, &new_item_header, sizeof(item_header_t));
        uint32_t temp_item_addr = new_item_addr;
        uint8_t item_index = NVDS_ITEM_INDEX_FIRST;
        bool use_new_data = false;

        /* Verify new data */
        while (true)
        {
            if (item_data_verify(temp_item_addr, &temp_item_header, item_data))
            {
                item_index++;
                if (item_index > new_item_header.part.total)
                {
                    /* New data is valid */
                    use_new_data = true;
                    break;
                }
                temp_item_addr = find_item_header(start_addr, new_item_header.tag, item_index, &temp_item_header);
                if (temp_item_addr == 0)
                {
                    /* New data is incomplete */
                    break;
                }
            }
            else
            {
                /* New data verification failed */
                break;
            }
        }

        if (use_new_data)
        {
            /* Delete old data */
            del_tag_in_range(&old_item_header, old_item_addr, new_item_addr);
        }
        else
        {
            /* New data error, delete new data */
            del_tag_in_range(&new_item_header, new_item_addr, s_nvds_handle.empty_addr);
        }
    }
}

/**
 ****************************************************************************************
 * @brief Add a tag in NVDS empty area
 *
 * @param[in] tag: Tag ID.
 * @param[in] len: Tag length.
 * @param[in] p_buf: Pointer of tag data.
 * @param[in] have_sub_item: mark whether there is sub item in p_buf.
 *
 * @return Result of tag append
 ****************************************************************************************
 */
static nvds_err_t append_tag(NvdsTag_t tag, uint16_t len, const uint8_t *p_buf, bool have_sub_tag)
{
    item_header_t item_header;
    const uint8_t *p_buf_now = p_buf;
    uint8_t item_data_len = 0;
    if (s_nvds_handle.avail_size >= len)
    {
        if (s_nvds_handle.empty_size < len)
        {
            /* The contiguous free flash is not enough for new data.
             * We should collect the garbages. */
            (void)gc_start();
        }

        if (s_nvds_handle.empty_size >= len)
        {
            uint8_t item_total = CEIL_DIV(len, NVDS_MAX_ITEM_DATA_LEN);
            for (uint8_t i = NVDS_ITEM_INDEX_FIRST; i <= item_total; i++)
            {
                memset(&item_header, 0, sizeof(item_header_t));
                item_header.tag = tag;
                item_header.len = len;
                item_header.part.total = item_total;
                item_header.part.index = i;
                item_header.header_checksum = calc_8bit_checksum((const uint8_t *)&item_header, offsetof(item_header_t, header_checksum));
                item_header.misc.tag_valid = NVDS_ITEM_TAG_VALID_VALID;
                item_header.misc.exist_sub_tag = have_sub_tag;
                item_data_len = calc_single_item_data_len(&item_header);
                item_header.data_checksum = calc_8bit_checksum(p_buf_now, item_data_len);
                write_single_item(s_nvds_handle.empty_addr, &item_header, p_buf_now);
                p_buf_now += NVDS_MAX_ITEM_DATA_LEN;
            }
        }
        else
        {
            NVDS_ERROR_HANDLER();
            return NVDS_SPACE_NOT_ENOUGH;
        }
    }
    else
    {
        NVDS_ERROR_HANDLER();
        return NVDS_SPACE_NOT_ENOUGH;
    }
    return NVDS_SUCCESS;
}

/**
 ****************************************************************************************
 * @brief Replace a tag
 *
 * @param[in] tag: Tag ID.
 *
 * @return Result of tag replacing
 ****************************************************************************************
 */
static nvds_err_t replace_tag(NvdsTag_t tag, uint16_t len, const uint8_t *p_buf, bool have_sub_tag)
{
    nvds_err_t ret = NVDS_FAIL;
    item_header_t item_header;
    const uint32_t start_addr = s_nvds_handle.start_addr;
    /* Get the old data length by reading one of the old item headers */
    if (find_item_header_with_cache(start_addr, tag, NVDS_ITEM_INDEX_ANY, &item_header))
    {
        /* Step 1: delete the tag cache(Normally, the cache is deleted when tag is deleted) */
    #if ENABLE_TAGS_CACHE
        tags_cache_rec_del(tag);
    #endif
        /* Step 2: write new data */
        /* append_tag may trigger GC, so the position of old data may change */
        ret = append_tag(tag, len, p_buf, have_sub_tag);
        if (NVDS_SUCCESS == ret)
        {
            /* Old data is ahead of new data, and new data must be at the end of NVDS */
            uint32_t end_addr = s_nvds_handle.empty_addr - ((CEIL_DIV(len, NVDS_MAX_ITEM_DATA_LEN)) * NVDS_ITEM_LEN);
            /* A tag can span at most one sector.
             * If the tag spans sectors, the length of NVDS_Header needs to be calculated */
            if ((s_nvds_handle.empty_addr - end_addr) >= NVDS_SECTOR_SIZE)
            {
                end_addr -= NVDS_ITEM_LEN;
            }
            /* Step 3: delete old data */
            del_tag_in_range(&item_header, start_addr, end_addr);
        }
    }
    return ret;
}

/**
 ****************************************************************************************
 * @brief Init nvds header
 *
 * @param[in] p_nvds_header: Pointer of nvds header.
 *
 ****************************************************************************************
 */
static void init_nvds_header(nvds_header_t *p_nvds_header)
{
    memset(p_nvds_header, NVDS_FLASH_DEFAULT_VALUE, sizeof(nvds_header_t));
    memcpy(&p_nvds_header->magic[0], &NVDS_MAGIC[0], sizeof(NVDS_MAGIC));
    memcpy(&p_nvds_header->version, &NVDS_VERSION, sizeof(nvds_version_t));
    p_nvds_header->sector_info.total = s_nvds_handle.sectors;
}

/**
 ****************************************************************************************
 * @brief Init unused NVDS. Erase all and write NVDS header.
 *
 * @param[in] sectors: The number of sectors(Not include GC).
 *
 * @return Result of NVDS init.
 ****************************************************************************************
 */
static nvds_err_t init_unused_nvds(uint8_t sectors)
{
    /* NVDS area is not initialized, so erase it first */
    nvds_port_erase_no_check(s_nvds_handle.start_addr, (sectors + 1) * NVDS_SECTOR_SIZE);
    /* Write NVDS area header. */
    nvds_header_t nvds_header;
    init_nvds_header(&nvds_header);
    for (uint8_t i = 1; i <= sectors; i++)
    {
        nvds_header.sector_info.index = i;
        nvds_header.sector_info.checksum = nvds_header.sector_info.total + nvds_header.sector_info.index;
        write_nvds_header(&nvds_header);
    }

    s_nvds_handle.empty_addr = s_nvds_handle.start_addr + NVDS_ITEM_LEN;
    s_nvds_handle.avail_size = sectors * NVDS_ITEM_PER_SECTOR * NVDS_MAX_ITEM_DATA_LEN;
    s_nvds_handle.empty_size = s_nvds_handle.avail_size;
    return NVDS_SUCCESS;
}

/**
 ****************************************************************************************
 * @brief Find the start address of empty space in NVDS
 *
 * @param[in] start_addr: NVDS start address.
 * @param[in] sectors: The number of sectors(Not include GC).
 *
 * @return Empty space address.
 ****************************************************************************************
 */
static uint32_t find_empty_addr(uint32_t start_addr, uint32_t sectors)
{
    uint32_t item_addr;
    const uint32_t end_addr = start_addr + sectors * NVDS_SECTOR_SIZE;
    uint32_t empty_addr = end_addr;

    /* Search for empty item address from back to front */
    item_addr = end_addr - NVDS_ITEM_LEN;
    do
    {
        if (is_empty_item(item_addr))
        {
            empty_addr = item_addr;
        }
        else
        {
            /* The last empty_addr is the empty item address */
            break;
        }
        item_addr -= NVDS_ITEM_LEN;
        /* Ignore NVDS_Header */
        if (0 == (item_addr % NVDS_SECTOR_SIZE))
        {
            item_addr -= NVDS_ITEM_LEN;
        }
    }
    while (item_addr > start_addr);
    return empty_addr;
}

/**
 ****************************************************************************************
 * @brief Calculate the size of empty space in NVDS
 *
 * @param[in] empty_addr: NVDS empty space address.
 * @param[in] end_addr: NVDS end address(Not include GC).
 *
 * @return Empty space size.
 ****************************************************************************************
 */
static uint32_t calc_empty_size(uint32_t empty_addr, uint32_t end_addr)
{
    int8_t free_sector = CEIL_DIV(end_addr, NVDS_SECTOR_SIZE) - CEIL_DIV(empty_addr, NVDS_SECTOR_SIZE);
    return ((end_addr - empty_addr) / NVDS_ITEM_LEN - free_sector) * NVDS_MAX_ITEM_DATA_LEN;
}

/**
 ****************************************************************************************
 * @brief Calculate the size of available space in NVDS
 *
 * @param[in] start_addr: NVDS start address.
 * @param[in] sectors: The number of sectors(Not include GC).
 *
 * @return Available space size.
 ****************************************************************************************
 */
static uint32_t calc_avail_size(uint32_t start_addr, uint32_t sectors)
{
    /* Calculator NVDS avail_size. Just verify item header. */
    uint32_t avail_size = sectors * (NVDS_ITEM_PER_SECTOR * NVDS_MAX_ITEM_DATA_LEN);
    item_header_t item_header;
    while (1)
    {
        uint32_t item_addr = find_item_header(start_addr, NVDS_ITEM_TAG_ANY, NVDS_ITEM_INDEX_ANY, &item_header);
        if (item_addr)
        {
        #if ENABLE_TAGS_CACHE
            tags_cache_rec_add(&item_header, item_addr);
        #endif
            avail_size -= NVDS_MAX_ITEM_DATA_LEN;
            //lint -e9044 [advisory] function parameter modified
            start_addr = item_addr + NVDS_ITEM_LEN;
        }
        else
        {
            break;
        }
    }
    return avail_size;
}

/**
 ****************************************************************************************
 * @brief Init used NVDS.
 *
 * @param[in] sectors: The number of sectors(Not include GC).
 * @param[in] del_duplicate: Indicates whether to delete duplicate tags.
 *
 * @return Result of NVDS init.
 ****************************************************************************************
 */
static nvds_err_t init_used_nvds(uint8_t sectors, bool del_duplicate)
{
    const uint32_t end_addr = s_nvds_handle.start_addr + s_nvds_handle.sectors * NVDS_SECTOR_SIZE;
    /* find_empty_addr must be executed first */
    s_nvds_handle.empty_addr = find_empty_addr(s_nvds_handle.start_addr, sectors);
    s_nvds_handle.empty_size = calc_empty_size(s_nvds_handle.empty_addr, end_addr);
    s_nvds_handle.avail_size = calc_avail_size(s_nvds_handle.start_addr, sectors);

    if (del_duplicate)
    {
        del_duplicate_tag();
    }
#if NVDS_DEBUG_ENABLE
    if ((s_nvds_handle.avail_size < 0) || (s_nvds_handle.avail_size > (sectors * (NVDS_ITEM_PER_SECTOR) * NVDS_MAX_ITEM_DATA_LEN)))
    {
        NVDS_ERROR_HANDLER();
        return NVDS_LENGTH_OUT_OF_RANGE;
    }
#endif
    return NVDS_SUCCESS;
}

/**
 ****************************************************************************************
 * @brief Read GC info from GC area.
 *
 * @param[in,out] p_gc_info: Pointer of gc info.
 *
 * @return Result of read GC info.
 ****************************************************************************************
 */
static nvds_err_t nvds_get_gc_info(nvds_gc_info_t *p_gc_info)
{
    nvds_port_read_no_check(NVDS_GC_ADDR(s_nvds_handle), (uint8_t *)p_gc_info, sizeof(nvds_gc_info_t));
    if (GC_BUFFER_VALID == p_gc_info->pattern)
    {
        if (p_gc_info->checksum == calc_8bit_checksum((uint8_t *)p_gc_info, offsetof(nvds_gc_info_t, checksum)))
        {
            return NVDS_SUCCESS;
        }
    }
    return NVDS_FAIL;
}

/**
 ****************************************************************************************
 * @brief Find the first invalid item
 *
 * @return Address of the first invalid item
 ****************************************************************************************
 */
static uint32_t find_first_invalid_item(void)
{
    uint32_t item_addr = 0;
    item_header_t item_header;
    uint32_t start_addr = s_nvds_handle.start_addr + NVDS_ITEM_LEN;
    const uint32_t end_addr = s_nvds_handle.empty_addr;
    do
    {
        nvds_port_read_no_check(start_addr, (uint8_t *)&item_header, sizeof(item_header_t));
        if (item_header_verify(&item_header))
        {
            start_addr += NVDS_ITEM_LEN;
            /* Ignore NVDS_Header */
            if (0 == (start_addr % NVDS_SECTOR_SIZE))
            {
                start_addr += NVDS_ITEM_LEN;
            }
        }
        else
        {
            item_addr = start_addr;
            break;
        }
    } while (start_addr < end_addr);

    return item_addr;
}

/**
 ****************************************************************************************
 * @brief Copy the items in GC to NVDS
 *
 * @param[in] write_sector_addr: Write sector address of NVDS
 * @param[in] valid_item_num: The number of valid item in GC.
 ****************************************************************************************
 */
static void copy_gc_area_to_nvds(uint32_t write_sector_addr, uint8_t valid_item_num)
{
    uint32_t gc_item_addr = NVDS_GC_ADDR(s_nvds_handle) + NVDS_ITEM_LEN;
    uint8_t item_buf[NVDS_ITEM_LEN];
    uint32_t item_addr = write_sector_addr + NVDS_ITEM_LEN;
    for (uint32_t i = 0; i < valid_item_num; i++)
    {
        nvds_port_read_no_check(gc_item_addr, item_buf, NVDS_ITEM_LEN);
        nvds_port_write_check(item_addr, item_buf, NVDS_ITEM_LEN);
        gc_item_addr += NVDS_ITEM_LEN;
        item_addr += NVDS_ITEM_LEN;
    }
}

/**
 ****************************************************************************************
 * @brief Delete up to N valid items in the range [start_addr, end_addr)
 *
 * @param[in] start_addr: start address
 * @param[in] end_addr: end address
 * @param[in] valid_item_num: The maximum number of valid item.
 ****************************************************************************************
 */
static void del_valid_item(uint32_t start_addr, uint32_t end_addr, uint8_t valid_item_num)
{
    item_header_t item_header;
    uint32_t item_header_addr;
    //lint -e9044 [advisory] function parameter modified
    start_addr = FLOOR_ALIGN_SECTOR(start_addr);
    for (uint32_t i = 0; i < valid_item_num; i++)
    {
        item_header_addr = find_item_header(start_addr, NVDS_ITEM_TAG_ANY, NVDS_ITEM_INDEX_ANY, &item_header);
        if ((0U != item_header_addr) && (item_header_addr < end_addr))
        {
            del_single_item(item_header_addr, &item_header);
            start_addr = item_header_addr + NVDS_ITEM_LEN;
            if (start_addr >= end_addr)
            {
                break;
            }
        }
        else
        {
            break;
        }
    }
}

/**
 ****************************************************************************************
 * @brief Copy up to NVDS_ITEM_PER_SECTOR valid items from NVDS to GC area.
 *
 * @param[in]     start_addr: copy start address.
 * @param[in,out] p_end_addr: In, just a pointer. Out, copy end address(address of the next item).
 *
 * @return Valid item number
 ****************************************************************************************
 */
static uint8_t copy_valid_item_to_gc(uint32_t start_addr, uint32_t *p_end_addr)
{
    uint8_t item_buf[NVDS_ITEM_LEN];
    item_header_t item_header;
    uint32_t item_header_addr;
    uint32_t gc_empty_addr = NVDS_GC_ADDR(s_nvds_handle) + NVDS_ITEM_LEN;
    start_addr = FLOOR_ALIGN_SECTOR(start_addr);
    uint32_t valid_item_num = 0;

    /* Find and copy valid item to GC */
    for (valid_item_num = 0; valid_item_num < NVDS_ITEM_PER_SECTOR; valid_item_num++)
    {
        item_header_addr = find_item_header(start_addr, NVDS_ITEM_TAG_ANY, NVDS_ITEM_INDEX_ANY, &item_header);
        if (item_header_addr)
        {
            nvds_port_read_no_check(item_header_addr, item_buf, NVDS_ITEM_LEN);
            nvds_port_write_check(gc_empty_addr, item_buf, NVDS_ITEM_LEN);
            /* Successfully copied an item */
            start_addr = item_header_addr + NVDS_ITEM_LEN;
            gc_empty_addr += NVDS_ITEM_LEN;
        }
        else
        {
            break;
        }
    }
    *p_end_addr = start_addr;
    return (uint8_t)valid_item_num;
}

/**
 ****************************************************************************************
 * @brief Write GC info to GC area.
 *
 * @param[in] gc_start_addr: the first GC item address.
 * @param[in] gc_end_addr: the last GC item end address.
 * @param[in] valid_item_num: Valid item numbers.
 *
 ****************************************************************************************
 */
static void write_gc_info(uint32_t gc_start_addr, uint32_t gc_end_addr, uint8_t valid_item_num)
{
    nvds_gc_info_t gc_info;
    memset(&gc_info, NVDS_FLASH_DEFAULT_VALUE, sizeof(gc_info));
    gc_info.pattern = GC_BUFFER_VALID;
    gc_info.r_sector = (FLOOR_ALIGN_SECTOR(gc_end_addr) - s_nvds_handle.start_addr) / NVDS_SECTOR_SIZE;
    gc_info.w_sector = (FLOOR_ALIGN_SECTOR(gc_start_addr) - s_nvds_handle.start_addr) / NVDS_SECTOR_SIZE;
    gc_info.r_page = (gc_end_addr - FLOOR_ALIGN_SECTOR(gc_end_addr))/NVDS_ITEM_LEN;
    gc_info.item_num = valid_item_num;
    gc_info.checksum = calc_8bit_checksum((uint8_t *)&gc_info, offsetof(nvds_gc_info_t, checksum));
    nvds_port_write_check(NVDS_GC_ADDR(s_nvds_handle), (uint8_t *)&gc_info, sizeof(gc_info));
#if NVDS_DEBUG_ENABLE
    printf("----  write_gc_info  ----\r\n");
    printf("w_sector 0x%x \r\n",gc_info.w_sector);
    printf("r_sector 0x%x \r\n",gc_info.w_sector);
    printf("r_page 0x%x \r\n",gc_info.r_page);
    printf("item_num 0x%x \r\n",gc_info.item_num);
#endif
}

/**
 ****************************************************************************************
 * @brief Compare the tag content in NVDS and p_buf to see if it is the same.
 *
 * @param[in] tag: Tag ID.
 * @param[in] len: Length of p_buf.
 * @param[in] p_buf: Pointer to tag content.
 *
 * @return True or false
 ****************************************************************************************
 */
static bool is_tag_same(NvdsTag_t tag, uint16_t len, const uint8_t *p_buf)
{
    const uint8_t *p_buf_now = p_buf;
    item_header_t item_header;
    uint32_t item_header_addr = 0;
    uint32_t item_index = NVDS_ITEM_INDEX_FIRST;
    uint8_t item_data[NVDS_MAX_ITEM_DATA_LEN];
    memset(item_data, 0, sizeof(item_data));
    do
    {
        item_header_addr = find_item_header_with_cache(s_nvds_handle.start_addr, tag, item_index, &item_header);
        if (item_header_addr)
        {
            uint8_t item_data_len = calc_single_item_data_len(&item_header);
            nvds_port_read_no_check(item_header_addr + sizeof(item_header_t), item_data, item_data_len);
            if (memcmp(item_data, p_buf_now, item_data_len))
            {
                return false;
            }
            p_buf_now += item_data_len;
            item_index++;
        }
        else
        {
            return false;
        }
    } while (item_index < item_header.part.total);

    return true;
}

/**
 ****************************************************************************************
 * @brief Garbage collection process
 *
 * @param[in] start_addr: The address of the first invalid item.
 *
 * @return Result of garbage collection process.
 ****************************************************************************************
 */
static nvds_err_t gc_process(uint32_t start_addr)
{
    nvds_err_t ret = NVDS_SUCCESS;
    const uint32_t end_addr = UP_ALIGN_SECTOR(s_nvds_handle.empty_addr);
    uint32_t gc_item_end_addr;
    do
    {
        gc_item_end_addr = start_addr;
        /* 1.Erase GC area */
        nvds_port_erase_no_check(NVDS_GC_ADDR(s_nvds_handle), NVDS_SECTOR_SIZE);
        /* 2.Copy one sector valid item to gc area */
        uint8_t valid_item_num = copy_valid_item_to_gc(start_addr, &gc_item_end_addr);
        if (valid_item_num)
        {
            /* 3.Write gc info to gc sector */
            write_gc_info(start_addr, gc_item_end_addr, valid_item_num);
            /* 4.Delete copied TAG */
            del_valid_item(FLOOR_ALIGN_SECTOR(start_addr), gc_item_end_addr, valid_item_num);
        }
        /* 5.Erase copied NVDS area */
        nvds_port_erase_no_check(FLOOR_ALIGN_SECTOR(start_addr), NVDS_SECTOR_SIZE);
        /* Write NVDS header */
        nvds_header_t nvds_header;
        init_nvds_header(&nvds_header);
        nvds_header.sector_info.index = (start_addr - s_nvds_handle.start_addr)/ NVDS_SECTOR_SIZE + 1;
        nvds_header.sector_info.checksum = nvds_header.sector_info.total + nvds_header.sector_info.index;
        write_nvds_header(&nvds_header);

        if (valid_item_num)
        {
            /* 6.Copy GC area to NVDS */
            copy_gc_area_to_nvds(FLOOR_ALIGN_SECTOR(start_addr), valid_item_num);
            /* 7.Set GC info invalid */
            GC_INFO_PATTERN_TYPE pattern = GC_BUFFER_INVALID;
            nvds_port_write_check(NVDS_GC_ADDR(s_nvds_handle) + offsetof(nvds_gc_info_t, pattern), &pattern, sizeof(GC_INFO_PATTERN_TYPE));
        }
        start_addr += NVDS_SECTOR_SIZE;
    } while (start_addr < end_addr);

    ret = init_used_nvds(s_nvds_handle.sectors, false);
    return ret;
}

/**
 ****************************************************************************************
 * @brief Garbage collection start and process.
 *
 * @return Result of garbage collection.
 ****************************************************************************************
 */
static nvds_err_t gc_start(void)
{
    nvds_err_t ret = NVDS_SUCCESS;

    nvds_gc_start_callback();

    s_nvds_handle.state = NVDS_STATE_BUSY_GC;
#if ENABLE_TAGS_CACHE
    tags_cache_clean();
#endif
    uint32_t del_item_addr = find_first_invalid_item();
    if (del_item_addr)
    {
         ret = gc_process(del_item_addr);
    }
    else
    {
        ret = NVDS_COMPACT_FAILED;
    }
    s_nvds_handle.state = NVDS_STATE_BUSY;

    nvds_gc_end_callback();

    return ret;
}

/**
 ****************************************************************************************
 * @brief Restore GC processing.
 *
 * @param[in] p_gc_info: Pointer of GC info
 *
 * @return Result of restore GC processing.
 ****************************************************************************************
 */
static nvds_err_t restore_gc_process(nvds_gc_info_t* p_gc_info)
{
#if NVDS_DEBUG_ENABLE
    printf("++++ restore_gc_process ++++\r\n");
    printf("w_sector 0x%x \r\n",p_gc_info->w_sector);
    printf("r_sector 0x%x \r\n",p_gc_info->w_sector);
    printf("r_page 0x%x \r\n",p_gc_info->r_page);
    printf("item_num 0x%x \r\n",p_gc_info->item_num);
#endif
    nvds_err_t ret = NVDS_SUCCESS;
    s_nvds_handle.state = NVDS_STATE_BUSY_GC;
    uint32_t gc_start_sector_addr = s_nvds_handle.start_addr + (p_gc_info->w_sector * NVDS_SECTOR_SIZE);
    uint32_t gc_end_sector_addr = s_nvds_handle.start_addr + (p_gc_info->r_sector * NVDS_SECTOR_SIZE);
    uint32_t gc_end_addr = gc_end_sector_addr + (p_gc_info->r_page * NVDS_ITEM_LEN);
    const uint8_t valid_item_num = p_gc_info->item_num;

    /* Delete copied TAG */
    del_valid_item(gc_start_sector_addr, gc_end_addr, valid_item_num);
    /* Erase NVDS area */
    nvds_port_erase_no_check(FLOOR_ALIGN_SECTOR(gc_start_sector_addr), NVDS_SECTOR_SIZE);
    /* Copy gc area to NVDS  */
    copy_gc_area_to_nvds(gc_start_sector_addr, valid_item_num);

    /* Process other sector  */
    ret = gc_process(gc_end_sector_addr + NVDS_SECTOR_SIZE);
    return ret;
}

/**
 ****************************************************************************************
 * @brief Make a sub item.
 *
 * @param[in,out] p_item: Pointer of item.
 * @param[in] sub_tag: Sub tag ID.
 * @param[in] len: The length of data.
 * @param[in] p_buf: Pointer of the sub data.
 * @param[in] data_offset: Empty data offset in item.
 *
 * @return Result of sub item making.
 ****************************************************************************************
 */
static void make_sub_item(nvds_item_t *p_item, NvdsTag_t sub_tag, uint16_t len, const uint8_t *p_buf, uint8_t data_offset)
{
    item_header_t sub_item_header;
    memset(&sub_item_header, NVDS_FLASH_DEFAULT_VALUE, sizeof(item_header_t));
    sub_item_header.tag = sub_tag;
    sub_item_header.len = len;
    sub_item_header.part.total = 1;
    sub_item_header.part.index = 1;
    sub_item_header.header_checksum = calc_8bit_checksum((uint8_t *)(&sub_item_header), offsetof(item_header_t, header_checksum));

    sub_item_header.misc.tag_valid = NVDS_ITEM_TAG_VALID_VALID;
    sub_item_header.misc.exist_sub_tag = NVDS_ITEM_NO_SUB_TAG;
    sub_item_header.data_checksum = calc_8bit_checksum(p_buf, len);

    memcpy(&p_item->data[data_offset], &sub_item_header, NVDS_ITEM_HDR_LEN);
    memcpy(&p_item->data[data_offset + NVDS_ITEM_HDR_LEN], p_buf, len);
}

/**
 ****************************************************************************************
 * @brief Find sub tag in p_item.
 *
 * @param[in]  p_item: NVDS item.
 * @param[in] sub_tag: Sub tag ID.
 *
 * @return Address offset of p_item. If not find sub tag, return 0.
 ****************************************************************************************
 */
static uint8_t find_sub_tag(const nvds_item_t *p_item, const NvdsTag_t sub_tag)
{
    if (p_item->item_header.misc.exist_sub_tag)
    {
        if (item_header_verify(&(p_item->item_header)))
        {
            const item_header_t *p_sub_item_header;
            uint32_t item_offset = NVDS_ITEM_HDR_LEN;  /* use uint32_t to avoid item_offset overflow */
            do
            {
                //lint -e9087 [required] casting is required
                p_sub_item_header = (const item_header_t *)&(TO_U8_PTR(p_item)[item_offset]);
                if (NVDS_ITEM_TAG_END == p_sub_item_header->tag)
                {
                    break;
                }
                else if (item_header_verify(p_sub_item_header))
                {
                    if (sub_tag == p_sub_item_header->tag)
                    {
                        return (uint8_t)item_offset;
                    }
                    else /* found other sub tag */
                    {
                        item_offset += p_sub_item_header->len + NVDS_ITEM_HDR_LEN;
                    }
                }
                else
                {
                    break;
                }
            } while (item_offset < NVDS_MAX_ITEM_DATA_LEN);
        }
    }

    return 0;
}

/**
 ****************************************************************************************
 * @brief Delete sub tag in p_item.
 *
 * @param[in]  p_item: NVDS item.
 * @param[in] sub_tag: Sub tag ID.
 * @param[out] p_sub_tag_exist: Pointer to sub tag exist flag.
 *
 * @return Empty data offset of p_item.
 ****************************************************************************************
 */
static uint8_t del_sub_tag(nvds_item_t *p_item, const NvdsTag_t sub_tag, bool *p_sub_tag_exist)
{
    uint8_t item_data_buf[NVDS_MAX_ITEM_DATA_LEN];
    memset(item_data_buf, NVDS_FLASH_DEFAULT_VALUE, sizeof(item_data_buf));
    uint8_t empty_data_offset = 0;
    const item_header_t *p_sub_item_header;
    uint32_t item_offset = NVDS_ITEM_HDR_LEN;
    *p_sub_tag_exist = false;
    do
    {
        p_sub_item_header = (const item_header_t*)&((TO_U8_PTR(p_item))[item_offset]);
        if (NVDS_ITEM_TAG_END == p_sub_item_header->tag)
        {
            break;
        }
        else if (item_header_verify(p_sub_item_header))
        {
            uint8_t sub_item_len = NVDS_ITEM_HDR_LEN + p_sub_item_header->len;
            if (sub_tag == p_sub_item_header->tag)
            {
                /* Found tag to delete(not copy to buf) */
                item_offset += sub_item_len;
                *p_sub_tag_exist = true;
            }
            else
            {
                /* Copy other sub item  */
                memcpy(&item_data_buf[empty_data_offset], &((TO_U8_PTR(p_item))[item_offset]), sub_item_len);
                item_offset += sub_item_len;
                empty_data_offset += sub_item_len;
            }
        }
        else
        {
            break;
        }
    } while (item_offset < (NVDS_MAX_ITEM_DATA_LEN));

    memcpy(p_item->data, item_data_buf, NVDS_MAX_ITEM_DATA_LEN);
    return empty_data_offset;
}

/**
 ****************************************************************************************
 * @brief Read the specified item into the buffer.
 *
 * @param[in]       tag: Tag ID.
 * @param[in,out]   len: Length of p_buf.
 * @param[in,out] p_buf: Item buffer.
 *
 * @return Result of read item.
 ****************************************************************************************
 */
static nvds_err_t read_item(NvdsTag_t tag, uint16_t len, uint8_t *p_buf)
{
    item_header_t item_header;
    uint32_t item_header_addr = find_item_header_with_cache(s_nvds_handle.start_addr, tag, NVDS_ITEM_INDEX_FIRST, &item_header);
    if (item_header_addr)
    {
        if ((len + NVDS_ITEM_HDR_LEN) < item_header.len)
        {
            return NVDS_SPACE_NOT_ENOUGH;
        }
        else
        {
            nvds_port_read_no_check(item_header_addr, p_buf, NVDS_ITEM_LEN);
            return NVDS_SUCCESS;
        }
    }
    return NVDS_FAIL;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
nvds_err_t nvds_init(uint32_t start_addr, uint8_t sectors)
{
    if (0U == sectors)
    {
        return NVDS_INVALID_SECTORS;
    }

    uint32_t flash_id = 0U;
    uint32_t flash_size = 0U;
    nvds_port_get_info(&flash_id, &flash_size);

    memset(&s_nvds_handle, 0x00, sizeof(s_nvds_handle));
    if (start_addr)
    {
        /* Start address is sector aligned. */
        if (start_addr % NVDS_SECTOR_SIZE)
        {
            NVDS_ERROR_HANDLER();
            return NVDS_INVALID_START_ADDR;
        }
        /* NVDS + GC is in flash area. */
        if ((start_addr + ((sectors + 1) * NVDS_SECTOR_SIZE)) <= (NVDS_FLASH_BASE + flash_size))
        {
            s_nvds_handle.start_addr = start_addr;
        }
        else
        {
            NVDS_ERROR_HANDLER();
            return NVDS_INVALID_START_ADDR;
        }
    }
    else
    {
        /* start addr = FLASH end addr - GC - NVDS  */
        s_nvds_handle.start_addr = NVDS_FLASH_BASE + flash_size - (sectors + 1) * NVDS_SECTOR_SIZE;
    }
    s_nvds_handle.sectors = sectors;

    nvds_gc_info_t gc_info;
    if (NVDS_SUCCESS == nvds_get_gc_info(&gc_info))
    {
        /* The chip is reset during GC, so restore GC process */
        if (NVDS_SUCCESS != restore_gc_process(&gc_info))
        {
            NVDS_ERROR_HANDLER();
            return NVDS_COMPACT_FAILED;
        }
    }

    /* Check NVDS magic for all sectors, if magic is not present, it means NVDS has not been initialized. */
    nvds_header_t nvds_header;
    bool nvds_is_used = false;
    uint32_t sector_addr = s_nvds_handle.start_addr;
    for (uint32_t i = 0; i < sectors; i++)
    {
        memset(&nvds_header, 0, sizeof(nvds_header));
        nvds_port_read_no_check(sector_addr, (uint8_t *)&nvds_header, sizeof(nvds_header));
        if (0U == memcmp(nvds_header.magic, NVDS_MAGIC, NVDS_MAGIC_LEN))
        {
            nvds_is_used = true;
            break;
        }
        sector_addr += NVDS_SECTOR_SIZE;
    }
    if (!nvds_is_used)
    {
        /* NVDS is not initialized */
        nvds_err_t error = init_unused_nvds(sectors);
        if (error != NVDS_SUCCESS)
        {
            NVDS_ERROR_HANDLER();
            return error;
        }
    }
    else
    {
        /* NVDS has been initialized */
        nvds_err_t error = init_used_nvds(sectors, true);
        if (error != NVDS_SUCCESS)
        {
            NVDS_ERROR_HANDLER();
            return error;
        }
    }

    s_nvds_handle.state = NVDS_STATE_READY;
    return NVDS_SUCCESS;
}

nvds_err_t nvds_deinit(uint32_t start_addr, uint8_t sectors)
{
    /* Start address need sector aligned. */
    if (start_addr % NVDS_SECTOR_SIZE)
    {
        return NVDS_INVALID_START_ADDR;
    }
    memset(&s_nvds_handle, 0x00, sizeof(s_nvds_handle));
    uint32_t flash_id = 0U;
    uint32_t flash_size = 0U;
    nvds_port_get_info(&flash_id, &flash_size);
    if ((start_addr + (sectors + 1) * NVDS_SECTOR_SIZE) <= (NVDS_FLASH_BASE + flash_size))
    {
        nvds_port_erase_no_check(start_addr, (sectors + 1) * NVDS_SECTOR_SIZE);
        return NVDS_SUCCESS;
    }
    else
    {
        NVDS_ERROR_HANDLER();
        return NVDS_INVALID_START_ADDR;
    }
}

nvds_state_t nvds_get_state(void)
{
    return s_nvds_handle.state;
}

/* NOTE: Data will be copy to p_buf to verify, if the data verification fails, there will be dirty data in p_buf */
nvds_err_t nvds_get(NvdsTag_t tag, uint16_t *p_len, uint8_t *p_buf)
{
    /* Since ret is assigned a value only when an error occurs, the initial value of ret needs to be NVDS_SUCCESS */
    nvds_err_t ret = NVDS_SUCCESS;
    uint8_t *p_buf_now = p_buf;
    if (NVDS_STATE_RESET == s_nvds_handle.state)
    {
        return NVDS_NOT_INIT;
    }
    if ((!p_len)
        || (*p_len == 0)
        || (!p_buf)
        || (NVDS_ITEM_TAG_DEL == tag)
        ||(NVDS_ITEM_TAG_END == tag))
    {
        return NVDS_INVALID_PARA;
    }

    /* Some BLE stack tag is stored using sub tag */
    NvdsTag_t main_tag = nvds_find_main_tag_by_sub_tag(tag);
    if (main_tag)
    {
        ret = nvds_get_sub_tag(main_tag, tag, p_len, p_buf);
    }
    else
    {
        NVDS_LOCK(s_nvds_handle);
        /* Not need NVDS_LOCK in nvds_get.*/
        /* Search all items of target tag, and verify it.*/
        item_header_t item_header;
        uint8_t item_index = NVDS_ITEM_INDEX_FIRST;
        uint32_t item_header_addr;

        do
        {
            item_header_addr = find_item_header_with_cache(s_nvds_handle.start_addr, tag, item_index, &item_header);
            if (item_header_addr)
            {
                if (*p_len < item_header.len)
                {
                    ret = NVDS_SPACE_NOT_ENOUGH;
                    break;
                }
                if (item_data_verify(item_header_addr, &item_header, p_buf_now))
                {
                    *p_len = item_header.len;
                    p_buf_now += NVDS_MAX_ITEM_DATA_LEN;
                    item_index++;
                }
                else
                {
                    ret = NVDS_TAG_DATA_ERROR;
                    break;
                }
            }
            else
            {
                ret = NVDS_TAG_NOT_EXISTED;
                break;
            }
        } while (item_index <= item_header.part.total);
        NVDS_UNLOCK(s_nvds_handle);
    }

    if (NVDS_TAG_NOT_EXISTED == ret)
    {
        ret = nvds_port_find_tag_in_other(tag, p_len, p_buf);
    }

    return ret;
}

nvds_err_t nvds_put(NvdsTag_t tag, uint16_t len, const uint8_t *p_buf)
{
    nvds_err_t ret;
    if (NVDS_STATE_RESET == s_nvds_handle.state)
    {
        return NVDS_NOT_INIT;
    }

    if ((NULL == p_buf)
        || (0 == len)
        || (NVDS_MAX_TAG_DATA_LEN < len)
        || (NVDS_ITEM_TAG_DEL == tag)
        || (NVDS_ITEM_TAG_END == tag))
    {
        return NVDS_INVALID_PARA;
    }

    /* Some BLE stack tag is stored using sub tag */
    NvdsTag_t main_tag = nvds_find_main_tag_by_sub_tag(tag);
    if (main_tag)
    {
        return nvds_put_sub_tag(main_tag, tag, len, p_buf);
    }
    else
    {
        NVDS_LOCK(s_nvds_handle);

        /* Check if the same key-value already exists */
        item_header_t item_header;
        if (find_item_header_with_cache(s_nvds_handle.start_addr, tag, NVDS_ITEM_INDEX_ANY, &item_header))
        {
            if (is_tag_same(tag, len, p_buf))
            {
                /* The old data is the same as the new data and does not need to be written. */
                ret = NVDS_SUCCESS;
            }
            else
            {
                /* The old data is different from the new data, delete the old data and add the new data */
                ret = replace_tag(tag, len, p_buf, NVDS_ITEM_NO_SUB_TAG);
            }
        }
        else
        {
            /* Can not find tag, add new one */
            ret = append_tag(tag, len, p_buf, false);
        }

        NVDS_UNLOCK(s_nvds_handle);

        return ret;
    }
}

nvds_err_t nvds_del(NvdsTag_t tag)
{
    nvds_err_t ret = NVDS_SUCCESS;
    if (NVDS_STATE_RESET == s_nvds_handle.state)
    {
        return NVDS_NOT_INIT;
    }
    if (NVDS_ITEM_TAG_END == tag || NVDS_ITEM_TAG_DEL == tag)
    {
        return NVDS_INVALID_PARA;
    }

    /* Some BLE stack tag is stored using sub tag */
    NvdsTag_t main_tag = nvds_find_main_tag_by_sub_tag(tag);
    if (main_tag)
    {
        return nvds_del_sub_tag(main_tag, tag);
    }
    else
    {
        NVDS_LOCK(s_nvds_handle);
        ret = del_tag(tag);
        NVDS_UNLOCK(s_nvds_handle);
        return ret;
    }
}

nvds_err_t nvds_put_sub_tag(NvdsTag_t main_tag, NvdsTag_t sub_tag, uint16_t len, const uint8_t *p_buf)
{
    nvds_err_t  ret = NVDS_FAIL;
    if (NVDS_STATE_RESET == s_nvds_handle.state)
    {
        NVDS_ERROR_HANDLER();
        return NVDS_NOT_INIT;
    }
    if ((NULL == p_buf)
        || (0 == len)
        || (NVDS_MAX_ITEM_DATA_LEN < len)
        || (NVDS_ITEM_TAG_DEL == main_tag)
        || (NVDS_ITEM_TAG_END == main_tag)
        || (NVDS_ITEM_TAG_DEL == sub_tag)
        || (NVDS_ITEM_TAG_END == sub_tag))
    {
        return NVDS_INVALID_PARA;
    }

    NVDS_LOCK(s_nvds_handle);
    bool is_main_tag_exist = false;
    do
    {
        uint8_t item_buf[NVDS_ITEM_LEN];
        /* item_buf is used to make new item, so set it to flash default value */
        memset(item_buf, NVDS_FLASH_DEFAULT_VALUE, sizeof(item_buf));
        if (NVDS_SUCCESS == read_item(main_tag, sizeof(item_buf), item_buf))
        {
            is_main_tag_exist = true;
            uint8_t item_offset = find_sub_tag((nvds_item_t *)item_buf, sub_tag);
            if (item_offset)
            {
                item_header_t *p_sub_item_header = (item_header_t *)&item_buf[item_offset];
                if (p_sub_item_header->len == len)
                {
                    if (0U == memcmp(p_buf, &item_buf[item_offset + NVDS_ITEM_HDR_LEN], len))
                    {
                        /* The old data is as same as the new data. */
                        ret = NVDS_SUCCESS;
                        break;
                    }
                }
            }
        }

        /* Write data to NVDS */
        /*If sub tag already exists, delete it and get item data empty position. */
        bool useless;
        uint8_t empty_data_offset = del_sub_tag((nvds_item_t *)item_buf, sub_tag, &useless);
        /* Check main tag length */
        uint16_t main_tag_len = empty_data_offset + NVDS_ITEM_HDR_LEN + len;
        if (main_tag_len > NVDS_MAX_ITEM_DATA_LEN)
        {
            ret = NVDS_SPACE_NOT_ENOUGH;
            break;
        }
        /* Put the sub tag into item_buf */
        make_sub_item((nvds_item_t *)item_buf, sub_tag, len, p_buf, empty_data_offset);
        if (is_main_tag_exist)
        {
            ret = replace_tag(main_tag, main_tag_len, &item_buf[NVDS_ITEM_HDR_LEN], NVDS_ITEM_HAVE_SUB_TAG);
        }
        else
        {
            ret = append_tag(main_tag, main_tag_len, &item_buf[NVDS_ITEM_HDR_LEN], NVDS_ITEM_HAVE_SUB_TAG);
        }
    } while (0);
    NVDS_UNLOCK(s_nvds_handle);

    return ret;
}

nvds_err_t nvds_get_sub_tag(NvdsTag_t main_tag, NvdsTag_t sub_tag, uint16_t *p_len, uint8_t *p_buf)
{
    nvds_err_t ret = NVDS_SUCCESS;
    if (NVDS_STATE_RESET == s_nvds_handle.state)
    {
        return NVDS_NOT_INIT;
    }
    if ((NULL == p_len)
        || (0U == *p_len )
        || (NULL == p_buf)
        || (NVDS_ITEM_TAG_DEL == main_tag)
        || (NVDS_ITEM_TAG_END == main_tag)
        || (NVDS_ITEM_TAG_DEL == sub_tag)
        || (NVDS_ITEM_TAG_END == sub_tag))
    {
        return NVDS_INVALID_PARA;
    }
    NVDS_LOCK(s_nvds_handle);
    uint8_t item_buf[NVDS_ITEM_LEN];
    do
    {
        if (NVDS_SUCCESS == read_item(main_tag, sizeof(item_buf), item_buf))
        {
            uint8_t item_offset = find_sub_tag((nvds_item_t *)item_buf, sub_tag);
            if (item_offset)
            {
                item_header_t *p_sub_item_header = (item_header_t *)&item_buf[item_offset];
                if (p_sub_item_header->data_checksum !=
                    calc_8bit_checksum(&item_buf[item_offset + NVDS_ITEM_HDR_LEN], p_sub_item_header->len))
                {
                    ret = NVDS_TAG_DATA_ERROR;
                    break;
                }
                /* Check whether p_buf has enough space */
                if (p_sub_item_header->len <= *p_len)
                {
                    /* Copy data to  p_buf and return sub tag length */
                    *p_len = p_sub_item_header->len;
                    memcpy(p_buf, &item_buf[item_offset + NVDS_ITEM_HDR_LEN], p_sub_item_header->len);
                    ret = NVDS_SUCCESS;
                    break;
                }
                else
                {
                    ret = NVDS_SPACE_NOT_ENOUGH;
                    break;
                }
            }
            else
            {
                ret = NVDS_TAG_NOT_EXISTED;
                break;
            }
        }
        else
        {
            ret = NVDS_TAG_NOT_EXISTED;
            break;
        }
    } while (0);
    NVDS_UNLOCK(s_nvds_handle);
    return ret;
}

nvds_err_t nvds_del_sub_tag(NvdsTag_t main_tag, NvdsTag_t sub_tag)
{
    nvds_err_t ret = NVDS_SUCCESS;

    if (NVDS_STATE_RESET == s_nvds_handle.state)
    {
        return NVDS_NOT_INIT;
    }
    if ((NVDS_ITEM_TAG_END == main_tag)
        || (NVDS_ITEM_TAG_DEL == main_tag)
        || (NVDS_ITEM_TAG_END == sub_tag)
        || (NVDS_ITEM_TAG_DEL == sub_tag))
    {
        return NVDS_INVALID_PARA;
    }

    NVDS_LOCK(s_nvds_handle);
    do
    {
        uint8_t item_buf[NVDS_ITEM_LEN];
        if (NVDS_SUCCESS == read_item(main_tag, sizeof(item_buf), item_buf))
        {
            /*If sub tag already exists, delete it and get item data empty position. */
            bool sub_tag_exist;
            uint8_t empty_data_offset = del_sub_tag((nvds_item_t *)item_buf, sub_tag, &sub_tag_exist);
            if (!sub_tag_exist)
            {
                ret = NVDS_TAG_NOT_EXISTED;
                break;
            }
            if (empty_data_offset)
            {
                uint16_t main_tag_len = empty_data_offset + NVDS_ITEM_HDR_LEN;
                ret = replace_tag(main_tag, main_tag_len, &item_buf[NVDS_ITEM_HDR_LEN], NVDS_ITEM_HAVE_SUB_TAG);
            }
            else
            {
                /* No sub tag, main tag is useless. */
                ret = del_tag(main_tag);
            }
            break;
        }
        else
        {
            ret = NVDS_TAG_NOT_EXISTED;
            break;
        }
    } while (0);
    NVDS_UNLOCK(s_nvds_handle);

    return ret;
}

nvds_err_t nvds_gc(void)
{
    if (NVDS_STATE_RESET == s_nvds_handle.state)
    {
        return NVDS_NOT_INIT;
    }
    nvds_err_t ret;
    NVDS_LOCK(s_nvds_handle);
    ret = gc_start();
    NVDS_UNLOCK(s_nvds_handle);
    return ret;
}

uint16_t nvds_tag_length(NvdsTag_t tag)
{
    if (NVDS_STATE_RESET == s_nvds_handle.state)
    {
        return 0;
    }

    item_header_t item_header;
    uint32_t item_header_addr;
    item_header_addr = find_item_header_with_cache(s_nvds_handle.start_addr, tag, NVDS_ITEM_INDEX_ANY, &item_header);
    if (item_header_addr)
    {
        return item_header.len;
    }
    else
    {
        return 0;
    }
}

uint16_t nvds_sub_tag_length(NvdsTag_t main_tag, NvdsTag_t sub_tag)
{
    if (NVDS_STATE_RESET == s_nvds_handle.state)
    {
        return 0;
    }

    uint8_t item_buf[NVDS_ITEM_LEN];
    if (NVDS_SUCCESS == read_item(main_tag, sizeof(item_buf), item_buf))
    {
        uint8_t item_offset = find_sub_tag((nvds_item_t *)item_buf, sub_tag);
        if (item_offset)
        {
            item_header_t *p_sub_item_header = (item_header_t *)&item_buf[item_offset];
            if (p_sub_item_header->data_checksum ==
                calc_8bit_checksum(&item_buf[item_offset + NVDS_ITEM_HDR_LEN], p_sub_item_header->len))
            {
                return p_sub_item_header->len;
            }
        }
    }
    return 0;
}

uint32_t nvds_get_avail_size(void)
{
    return (uint32_t)s_nvds_handle.avail_size;
}

uint32_t nvds_get_empty_size(void)
{
    return (uint32_t)s_nvds_handle.empty_size;
}

__WEAK void nvds_gc_start_callback(void)
{
}

__WEAK void nvds_gc_end_callback(void)
{
}

/**************************************************************************//**
 * @file     gr5405.h
 * @brief    CMSIS Cortex-M# Core Peripheral Access Layer Header File for
 *           Device GR5405
 * @version  V1.00
 * @date     12. June 2018
 ******************************************************************************/
/*
 * Copyright (c) 2016-2018, Shenzhen Huiding Technology Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/** @addtogroup CMSIS_Device
  * @{
  */

/** @addtogroup GR5405
  * @{
  */

#ifndef __GR5405_H__
#define __GR5405_H__

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup Library_configuration_section
  * @{
  */

/**
  * @brief GR5 Family
  */
#if !defined (GR5)
#define GR5
#endif /* GR5 */

/* Uncomment the line below according to the target GR5 device used in your
   application
  */

#if !defined (GR5405)
    #define GR5405
#endif

/** @} */

/** @addtogroup Device_Included
  * @{
  */
#if defined(GR5405)

    #include "gr5405_b0.h"
    #include "gr5405_int_b0.h"

    #define GR5405_RAM_ADDRESS             0x20000000
    #define GR5405_FLASH_ADDRESS           0x00200000
    #define GR5405_FLASH_SIZE              0x01000000
    #define GR5405_ALIAS_ADDRESS           0x01000000
#else
    #error "Please select first the target GR5405 device used in your application (in gr5405.h file)"
#endif

/** @} */

/** @addtogroup Exported_types
  * @{
  */

typedef enum
{
    RESET = 0,
    SET = !RESET
} flag_status_t;

#define it_status_t flag_status_t

typedef enum
{
    DISABLE = 0,
    ENABLE = !DISABLE
} functional_state_t;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum
{
    ERROR = 0,
    SUCCESS = !ERROR
} error_status_t;

/** @} */

/** @addtogroup Exported_macros
  * @{
  */
#ifndef SET_BITS
#define SET_BITS(REG, BIT)     ((REG) |= (BIT))
#endif

#ifndef CLEAR_BITS
#define CLEAR_BITS(REG, BIT)   ((REG) &= ~(BIT))
#endif

#ifndef READ_BITS
#define READ_BITS(REG, BIT)    ((REG) & (BIT))
#endif

#ifndef CLEAR_REG
#define CLEAR_REG(REG)        ((REG) = (0x0))
#endif

#ifndef WRITE_REG
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#endif

#ifndef READ_REG
#define READ_REG(REG)         ((REG))
#endif

#ifndef MODIFY_REG
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))
#endif

#ifndef POSITION_VAL
#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))
#endif

#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif

#if defined ( __CC_ARM ) || defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6100100)
  #ifndef C_CONSTRUCTOR
    #define C_CONSTRUCTOR __attribute__((constructor))
  #endif /* C_CONSTRUCTOR */

  #ifndef SECTION_RAM_CODE
    #if (((!defined(ROM_RUN_IN_FLASH)) && (defined(CFG_SDK_ROM) || defined(CFG_LAYER_TAG_ROM))) && (!ROM_ENHANCE))
      #define SECTION_RAM_CODE
    #else
      #define SECTION_RAM_CODE __attribute__((section("RAM_CODE")))
    #endif
  #endif /* SECTION_RAM_CODE */

  #ifndef RESERVE_RAM_SECTION
    #define RESERVE_RAM_SECTION __attribute__((section("RAM_RESERVE"),zero_init))   /**< To prevent doxygen from misidentifying the function name */
  #endif

#elif defined ( __GNUC__ )
  #ifndef C_CONSTRUCTOR
    #define C_CONSTRUCTOR __attribute__((constructor))
  #endif /* C_CONSTRUCTOR */

  #ifndef SECTION_RAM_CODE
    #if (((!defined(ROM_RUN_IN_FLASH)) && (defined(CFG_SDK_ROM) || defined(CFG_LAYER_TAG_ROM))) && (!ROM_ENHANCE))
      #define SECTION_RAM_CODE
    #else
      #define SECTION_RAM_CODE __ramfunc
      #pragma GCC diagnostic ignored "-Wattributes"
      #define __ramfunc __attribute__((noinline)) \
                        __attribute__((long_call, section(".ramfunc")))
    #endif /* CFG_LAYER_TAG_ROM */
  #endif /* SECTION_RAM_CODE */

#elif defined ( __ICCARM__ )
  #ifndef C_CONSTRUCTOR
    #define C_CONSTRUCTOR
  #endif /* C_CONSTRUCTOR */

  #ifndef SECTION_RAM_CODE
    #if (((!defined(ROM_RUN_IN_FLASH)) && (defined(CFG_SDK_ROM) || defined(CFG_LAYER_TAG_ROM))) && (!ROM_ENHANCE))
      #define SECTION_RAM_CODE
    #else
      #define SECTION_RAM_CODE __ramfunc
    #endif /* CFG_LAYER_TAG_ROM */
    #pragma diag_suppress=Ta022
    #pragma diag_suppress=Ta023
  #endif /* SECTION_RAM_CODE */

#else
  #error Unknown compiler.
#endif

/** @} */

/** @addtogroup Exported_macros
  * @{
  */
/** Bit band operation is supported at the peripheral address of 4000 0000~400F FFFF
 *  and sram 2000 0000~200F FFFF.
 *  eg.BIT_ADDR((uint32_t)&GPIO1->DATA,9) = 1;
 */

#if !defined(BIT_BAND_SUPPORT)
#define BIT_BAND_SUPPORT
#endif

#if defined(BIT_BAND_SUPPORT)
#define BITBAND(addr, bitnum)           (((addr) & 0xF0000000) + 0x2000000 + (((addr) & 0xFFFFF) << 5) + ((bitnum) << 2))
#define MEM_ADDR(addr)                  *((volatile uint32_t *)(addr))
#define BIT_ADDR(addr, bitnum)          MEM_ADDR(BITBAND(addr, bitnum))
#endif

/** @} */

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR5405_H__ */

/** @} */

/** @} */

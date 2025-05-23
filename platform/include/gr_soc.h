#ifndef GR_SOC_H
#define GR_SOC_H

#include "grx_sys.h"

extern void Reset_Handler(void);
extern void NMI_Handler(void);
extern void HardFault_Handler(void);
extern void MemManage_Handler(void);
extern void BusFault_Handler(void);
extern void UsageFault_Handler(void);
extern void SVC_Handler(void);
extern void DebugMon_Handler(void);
extern void PendSV_Handler(void);
extern void SysTick_Handler(void);

extern void platform_exflash_env_init(void);
extern void vector_table_init(void);
extern void soc_init(void);
extern void warm_boot_process(void);
extern void platform_init(void);
extern void soc_register_nvic(IRQn_Type indx, uint32_t func);
extern uint32_t get_wakeup_flag(void);

extern uint32_t nvds_get_start_addr(void);
extern uint8_t  nvds_get_init_error_info(void);
extern uint16_t sys_trim_info_sync(void);

typedef void (*FuncVector_t)(void);

#define SYS_RESET_REASON_NONE          (0U)
#define SYS_RESET_REASON_AONWDT        (1U << 2U)
#define SYS_RESET_REASON_FULL          (1U << 3U)
#define SYS_RESET_REASON_POR           (1U << 4U)

/**
  * @brief  Get chip reset reason.
  *
  * @retval Returned value can be one of the following values:
  *         @arg @ref SYS_RESET_REASON_NONE
  *         @arg @ref SYS_RESET_REASON_AONWDT
  *         @arg @ref SYS_RESET_REASON_FULL
  *         @arg @ref SYS_RESET_REASON_POR
  */
uint8_t sys_device_reset_reason(void);

#endif


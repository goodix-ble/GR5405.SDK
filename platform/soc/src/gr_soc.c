#include "gr_soc.h"
#include "grx_hal.h"

#ifndef DRIVER_TEST
#include "gr_includes.h"
#endif

#include "platform_sdk.h"
#include "custom_config.h"
#include "hal_flash.h"
#include "pmu_calibration.h"
#include "app_pwr_mgmt.h"

#define PUYA_FLASH_HP_CMD               (0xA3)
#define PUYA_FLASH_HP_END_DUMMY         (2)

#define FALSH_HP_MODE                   LL_XQSPI_HP_MODE_DIS
#define FLASH_HP_CMD                    PUYA_FLASH_HP_CMD
#define FLASH_HP_END_DUMMY              PUYA_FLASH_HP_END_DUMMY

#define SDK_VER_MAJOR                   1
#define SDK_VER_MINOR                   1
#define SDK_VER_BUILD                   7
#define COMMIT_ID                       0x386f405e

static const sdk_version_t sdk_version = {SDK_VER_MAJOR,
                                          SDK_VER_MINOR,
                                          SDK_VER_BUILD,
                                          COMMIT_ID,};//sdk version

void sys_sdk_verison_get(sdk_version_t *p_version)
{
    memcpy(p_version, &sdk_version, sizeof(sdk_version_t));
}

__ALIGNED(0x100) FuncVector_t FuncVector_table[MAX_NUMS_IRQn + NVIC_USER_IRQ_OFFSET] = {
    0,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0,
    0,
    0,
    0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
};

void soc_register_nvic(IRQn_Type indx, uint32_t func)
{
    FuncVector_table[indx + NVIC_USER_IRQ_OFFSET] = (FuncVector_t)func;
}

static fun_t svc_user_func = NULL;

void svc_func_register(uint8_t svc_num, uint32_t user_func)
{
    svc_user_func = (fun_t)user_func;
}

void svc_user_handler(uint8_t svc_num)
{
    if (svc_user_func)
        svc_user_func();
}

#if (BLE_SUPPORT == 1)
static bool wait_for_clock_calibration_done(uint32_t timeout)//unit:us
{
    bool ret = true;
    uint32_t wait_time = 0;
    while(!clock_calibration_is_done())
    {
        delay_us(1);
        if(++wait_time >= timeout)
        {
            ret = false;
            break;
        }
    }
    return ret;
}

#ifndef DTM_ATE_ENABLE
__WEAK void nvds_init_error_handler(uint8_t err_code)
{
    /* nvds_deinit will erase the flash area and old data will be lost */
    nvds_deinit(NVDS_START_ADDR, NVDS_NUM_SECTOR);
    nvds_init(NVDS_START_ADDR, NVDS_NUM_SECTOR);
}

static void nvds_setup(void)
{
    uint8_t err_code = nvds_init(NVDS_START_ADDR, NVDS_NUM_SECTOR);
    switch(err_code)
    {
        case NVDS_SUCCESS:
            break;
        default:
            /* Nvds initialization errors. */
            nvds_init_error_handler(err_code);
            break;
    }
}
#else
struct rwip_nvds_api
{
    bool (*initialized)(void);
    uint8_t (*get)(uint8_t tag, uint8_t *plen, uint8_t *pbuf);
    uint8_t (*put)(uint8_t tag, uint8_t len, uint8_t *pbuf);
    uint8_t (*del)(uint8_t tag);
};

//DTM ate don't need to use nvds, define strong symble to decouple the code in gr_nvds_adapter.c
void ble_nvds_set_entry(struct rwip_nvds_api *p_api)
{

}

uint8_t nvds_put(NvdsTag_t tag, uint16_t len, const uint8_t *p_buf)
{
    return 0;
}
#endif

#if (CFG_APP_DRIVER_SUPPORT == 1)
bool system_is_allow_sleep(void)
{
    return clock_calibration_is_done();
}

static const app_sleep_callbacks_t system_sleep_cb =
{
    .app_prepare_for_sleep = system_is_allow_sleep,
    .app_wake_up_ind       = NULL,
};
#else
hal_pm_status_t hal_pm_suspend_system(void)
{
    if(clock_calibration_is_done())
    {
        return HAL_PM_SLEEP;
    }
    else
    {
     return HAL_PM_ACTIVE;
    }
}
#endif

void power_mgmt_init(void)
{
#if (CFG_APP_DRIVER_SUPPORT == 1)
    /* Init peripheral sleep management */
    app_pwr_mgmt_init();
    pwr_register_sleep_cb(&system_sleep_cb, WAKEUP_PRIORITY_LOW, SYSTEM_PWR_ID);
#else
    hal_pm_init();
#endif
}
#endif /* BLE_SUPPORT */

uint8_t sys_device_reset_reason(void)
{
   uint8_t reset_season = AON_CTL->DBG_REG_RST_SRC & 0x3FUL;
   AON_CTL->DBG_REG_RST_SRC = AON_CTL->DBG_REG_RST_SRC | reset_season;
   if(SYS_RESET_REASON_AONWDT & reset_season)
   {
       return SYS_RESET_REASON_AONWDT;
   }
   else
   {
       return SYS_RESET_REASON_NONE;
   }
}

void first_class_task(void)
{
//DTM ate worked in debug mode, no need init flash to save resource
#ifndef DTM_ATE_ENABLE
    ll_xqspi_hp_init_t hp_init;

    platform_exflash_env_init();

    ll_gpio_set_pin_pull(GPIO0, 0x1 << 14, LL_GPIO_PULL_NO);   /* cs */
    ll_gpio_set_pin_pull(GPIO0, 0x1 << 15, LL_GPIO_PULL_UP);   /* io3 */
    ll_gpio_set_pin_pull(GPIO0, 0x1 << 16, LL_GPIO_PULL_NO);   /* clk */
    ll_gpio_set_pin_pull(GPIO0, 0x1 << 17, LL_GPIO_PULL_UP);   /* io2 */
    ll_gpio_set_pin_pull(GPIO0, 0x1 << 18, LL_GPIO_PULL_UP);   /* io1 */
    ll_gpio_set_pin_pull(GPIO0, 0x1 << 19, LL_GPIO_PULL_UP);   /* io0 */

    hp_init.xqspi_hp_enable    = FALSH_HP_MODE;
    hp_init.xqspi_hp_cmd       = FLASH_HP_CMD;
    hp_init.xqspi_hp_end_dummy = FLASH_HP_END_DUMMY;
    hal_exflash_enable_quad(hp_init);

#if (BLE_SUPPORT == 1)
    /* nvds module init process. */
    nvds_setup();

    /* set sram power state. */
    mem_pwr_mgmt_mode_set(MEM_POWER_AUTO_MODE);
#endif

#endif

#if (BLE_SUPPORT == 1)
    /* platform init process. */
    platform_sdk_init();
#endif
}

void second_class_task(void)
{
#if (BLE_SUPPORT == 1)
//no need to init lp clk and trigger the pmu timer in DTM ate to save resource
#ifndef DTM_ATE_ENABLE
    /* To choose the System clock source and set the accuracy of OSC. */
#if CFG_LPCLK_INTERNAL_EN
    platform_clock_init((mcu_clock_type_t)SYSTEM_CLOCK, RC_OSC_CLK, CFG_LF_ACCURACY_PPM, 0);
#else
    platform_clock_init((mcu_clock_type_t)SYSTEM_CLOCK, RTC_OSC_CLK, CFG_LF_ACCURACY_PPM, 0);
#endif

#if PMU_CALIBRATION_ENABLE && !defined(DRIVER_TEST)
    /* Enable auto pmu calibration function. */
    if(!CHECK_IS_ON_FPGA())
    {
        system_pmu_calibration_init(30000);
    }
#endif
#endif

    system_pmu_init((mcu_clock_type_t)SYSTEM_CLOCK);
#endif

    // pmu shall be init before clock set
    system_power_mode((sys_power_t)SYSTEM_POWER_MODE);
    SetSerialClock(SERIAL_S64M_CLK);
    SystemCoreSetClock((mcu_clock_type_t)SYSTEM_CLOCK);

#if (BLE_SUPPORT == 1)
    // recover the default setting by temperature, should be called in the end
    if(!CHECK_IS_ON_FPGA())
    {
        pmu_calibration_handler(NULL);
        wait_for_clock_calibration_done(1000000);
    }

    power_mgmt_init();
#endif
}

void otp_trim_init(void)
{
#if (BLE_SUPPORT == 1)
    if(SDK_SUCCESS != sys_trim_info_sync())
    {
        if(!CHECK_IS_ON_FPGA())
        {
            /* do nothing for not calibrated chips */
            while(1);
        }
    }
#endif
}

static void patch_init(void)
{
    gr5xx_fpb_init(FPB_MODE_PATCH_AND_DEBUG);
}

void platform_init(void)
{
    patch_init();
    otp_trim_init();
    first_class_task();
    second_class_task();
}

void vector_table_init(void)
{
    __DMB(); // Data Memory Barrier
    FuncVector_table[0] = *(FuncVector_t *)(SCB->VTOR);
    SCB->VTOR = (uint32_t)FuncVector_table; // Set VTOR to the new vector table location
    __DSB(); // Data Synchronization Barrier to ensure all
}

void warm_boot_process(void)
{
#if (BLE_SUPPORT == 1)
    vector_table_init();
    pwr_mgmt_warm_boot();
#endif
}

void soc_init(void)
{
#if !defined(WDT_RUN_ENABLE) || (!WDT_RUN_ENABLE)
    extern hal_status_t hal_aon_wdt_disable(void);
    while(HAL_OK != hal_aon_wdt_disable())
    {}
#endif
    platform_init();
}

__WEAK void sdk_init(void){};

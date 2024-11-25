/**
  ****************************************************************************************
  * @file    hal_i2c.c
  * @author  BLE Driver Team
  * @brief   I2C HAL module driver.
  ****************************************************************************************
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
  ****************************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gr5x.h"

#ifdef HAL_I2C_MODULE_ENABLED
#include "hal_i2c.h"

#ifdef HAL_CLOCK_UNIFORM_CONTROL
#include "hal_clock.h"
#endif

#ifdef HAL_CLOCK_MODULE
#include "hal_cgc.h"
#else
extern uint32_t GetSerialClock(void);
#endif
/** @addtogroup HAL_DRIVER
  * @{
  */



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/** @defgroup I2C_Private_Define I2C Private Define
  * @{
  */
/*
    Note:
        When I2C detects that the RX FIFO is full, it will actively pull the SCL signal low until there is available space in the RX FIFO, thereby avoiding overflow.
        If I2C_MONITOR_SCL_STUCK_AT_LOW_ENABLE = 1, I2C will actively report an Error Event when it detects that SCL is low for an extended period.
        This Error Event may be a proactive action taken by I2C to prevent RX FIFO overflow, rather than an indication of an error on the I2C bus.
        Therefore, if users enable I2C_MONITOR_SCL_STUCK_AT_LOW_ENABLE = 1, need to handle this behavior in their application layer error processing.
*/
#define I2C_MONITOR_SCL_STUCK_AT_LOW_ENABLE    (0)     /* 1:Enable. 0:Disable. */

#define I2C_TIMEOUT_ADDR    (10000U)       /*!< 10 s  */
#define I2C_TIMEOUT_BUSY    (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_RXNE    (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_STOP    (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_TFNF    (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_FLAG    (25U)          /*!< 25 ms */

#define I2C_TXFIFO_SIZE                    (8U)
#define I2C_RXFIFO_SIZE                    (8U)

#define I2C_FS_SPKLEN                       (4U)
#define I2C_CALCULATION_CONSTANT            (3U)           /*i2c rate calculation needs*/

/* Private define for @ref PreviousState usage */
#define I2C_STATE_MSK             ((uint32_t)((uint32_t)(((uint32_t)HAL_I2C_STATE_BUSY_TX | (uint32_t)HAL_I2C_STATE_BUSY_RX)) & (~((uint32_t)HAL_I2C_STATE_READY)))) /*!< Mask State define, keep only RX and TX bits */
#define I2C_STATE_NONE            ((uint32_t)(HAL_I2C_MODE_NONE))                                                                            /*!< Default Value                                          */
#define I2C_STATE_MASTER_BUSY_TX  ((uint32_t)(((uint32_t)HAL_I2C_STATE_BUSY_TX & I2C_STATE_MSK) | (uint32_t)HAL_I2C_MODE_MASTER))            /*!< Master Busy TX, combinaison of State LSB and Mode enum */
#define I2C_STATE_MASTER_BUSY_RX  ((uint32_t)(((uint32_t)HAL_I2C_STATE_BUSY_RX & I2C_STATE_MSK) | (uint32_t)HAL_I2C_MODE_MASTER))            /*!< Master Busy RX, combinaison of State LSB and Mode enum */
#define I2C_STATE_SLAVE_BUSY_TX   ((uint32_t)(((uint32_t)HAL_I2C_STATE_BUSY_TX & I2C_STATE_MSK) | (uint32_t)HAL_I2C_MODE_SLAVE))             /*!< Slave Busy TX, combinaison of State LSB and Mode enum  */
#define I2C_STATE_SLAVE_BUSY_RX   ((uint32_t)(((uint32_t)HAL_I2C_STATE_BUSY_RX & I2C_STATE_MSK) | (uint32_t)HAL_I2C_MODE_SLAVE))             /*!< Slave Busy RX, combinaison of State LSB and Mode enum  */

/* Private define to centralize the enable/disable of Interrupts */
#if I2C_MONITOR_SCL_STUCK_AT_LOW_ENABLE
#define I2C_XFER_ERROR_IT       (LL_I2C_INTR_MASK_TX_ABRT | LL_I2C_INTR_MASK_SCL_STUCK_AT_LOW)
#else
#define I2C_XFER_ERROR_IT       (LL_I2C_INTR_MASK_TX_ABRT)
#endif

#define I2C_MST_XFER_TX_IT      (I2C_XFER_ERROR_IT  | \
                                 LL_I2C_INTR_MASK_TX_EMPTY | \
                                 LL_I2C_INTR_MASK_STOP_DET)

#define I2C_MST_XFER_RX_IT      (I2C_XFER_ERROR_IT  | \
                                 LL_I2C_INTR_MASK_TX_EMPTY | \
                                 LL_I2C_INTR_MASK_RX_FULL  | \
                                 LL_I2C_INTR_MASK_RX_OVER  | \
                                 LL_I2C_INTR_MASK_STOP_DET)

#define I2C_SLV_XFER_TX_IT      (I2C_XFER_ERROR_IT  | \
                                 LL_I2C_INTR_MASK_TX_EMPTY | \
                                 LL_I2C_INTR_MASK_STOP_DET)

#define I2C_SLV_XFER_RX_IT      (I2C_XFER_ERROR_IT  | \
                                 LL_I2C_INTR_MASK_RX_FULL  | \
                                 LL_I2C_INTR_MASK_RX_OVER  | \
                                 LL_I2C_INTR_MASK_STOP_DET)

#define I2C_XFER_LISTEN_IT      (I2C_XFER_ERROR_IT  | \
                                 LL_I2C_INTR_MASK_STOP_DET | \
                                 LL_I2C_INTR_MASK_RD_REQ)

#define I2C_XFER_CPLT_IT        (LL_I2C_INTR_MASK_STOP_DET)

/* Private define to Abort Source */
#define I2C_TX_ABRT_NOACK       (LL_I2C_ABRT_GCALL_NOACK   | \
                                 LL_I2C_ABRT_TXDATA_NOACK  | \
                                 LL_I2C_ABRT_10ADDR2_NOACK | \
                                 LL_I2C_ABRT_10ADDR1_NOACK | \
                                 LL_I2C_ABRT_7B_ADDR_NOACK)

/* Private define Sequential Transfer Options default/reset value */
#define I2C_NO_OPTION_FRAME     (0xFFFF0000U)
/** @} */

/* Private macro -------------------------------------------------------------*/
#define I2C_ABS(a, b)   (((a) > (b)) ? ((a) - (b)) : ((b) - (a)))

#define I2C_GET_TX_DMA_COUNT(__HANDLE__) \
     I2C_ABS(((uint32_t)(__HANDLE__)->p_buffer), ll_dma_get_source_address((__HANDLE__)->p_dmatx->p_instance, (__HANDLE__)->p_dmatx->channel))

#define I2C_GET_RX_DMA_COUNT(__HANDLE__) \
     I2C_ABS(((uint32_t)(__HANDLE__)->p_buffer), ll_dma_get_destination_address((__HANDLE__)->p_dmatx->p_instance, (__HANDLE__)->p_dmarx->channel))

#define I2C_GET_TX_DMA_REMAIN_DATA(__HANDLE__) ((__HANDLE__)->xfer_size - I2C_GET_TX_DMA_COUNT(__HANDLE__))
#define I2C_GET_RX_DMA_REMAIN_DATA(__HANDLE__) ((__HANDLE__)->xfer_size - I2C_GET_RX_DMA_COUNT(__HANDLE__))

#define I2C_GET_DMA_REMAIN_DATA(__HANDLE__) \
    ((((__HANDLE__)->state) == HAL_I2C_STATE_BUSY_TX) ? \
    I2C_GET_TX_DMA_REMAIN_DATA(__HANDLE__) : I2C_GET_RX_DMA_REMAIN_DATA(__HANDLE__))

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/** @defgroup I2C_Private_Functions I2C Private Functions
  * @{
  */

/* Private function to config master or slave mode  */
__STATIC_INLINE void i2c_master_transfer_config(i2c_handle_t *p_i2c, uint16_t dev_address);
__STATIC_INLINE void i2c_slave_transfer_config(i2c_handle_t *p_i2c);

/* Private function to check error flags  */
static void i2c_master_check_error(i2c_handle_t *p_i2c);
static void i2c_slave_check_error(i2c_handle_t *p_i2c);

/* Private functions to handle flags during polling transfer */
static hal_status_t i2c_wait_on_flag_until_timeout(i2c_handle_t *p_i2c, __IM uint32_t *regs, uint32_t mask, \
        uint32_t status, uint32_t timeout);
static hal_status_t i2c_wait_on_raw_flag_until_timeout(i2c_handle_t *p_i2c, uint32_t flag, uint32_t status, \
        uint32_t timeout);
static hal_status_t i2c_wait_on_sta_flag_until_timeout(i2c_handle_t *p_i2c, uint32_t flag, uint32_t status, \
        uint32_t timeout);

/* Private functions to start master transfer */
static hal_status_t i2c_master_start_transmit(i2c_handle_t *p_i2c, uint32_t timeout);
static hal_status_t i2c_master_start_receive(i2c_handle_t *p_i2c, uint32_t timeout);
static hal_status_t i2c_master_start_transmit_it(i2c_handle_t *p_i2c);
static hal_status_t i2c_master_start_receive_it(i2c_handle_t *p_i2c);
static hal_status_t i2c_master_start_transmit_dma(i2c_handle_t *p_i2c);
static hal_status_t i2c_master_start_receive_dma(i2c_handle_t *p_i2c);

/* Private functions to start slave transfer */
static hal_status_t i2c_slave_start_transmit(i2c_handle_t *p_i2c, uint32_t timeout);
static hal_status_t i2c_slave_start_receive(i2c_handle_t *p_i2c, uint32_t timeout);

/* Private functions to handle IT transfer */
static void i2c_req_mem_read_write(i2c_handle_t *p_i2c, uint16_t dev_address, uint16_t mem_address, \
        uint16_t mem_addr_size);

/* Private functions for I2C transfer IRQ handler */
static void i2c_master_isr_it(i2c_handle_t*p_i2c, uint32_t it_source, uint32_t abort_sources);
static void i2c_slave_isr_it(i2c_handle_t*p_i2c, uint32_t it_source, uint32_t abort_sources);
static void i2c_master_isr_dma(i2c_handle_t*p_i2c, uint32_t it_source, uint32_t abort_sources);
static void i2c_slave_isr_dma(i2c_handle_t*p_i2c, uint32_t it_source, uint32_t abort_sources);

/* Private functions to handle DMA transfer */
static void i2c_dma_master_transmit_cplt(dma_handle_t *p_dma);
static void i2c_dma_master_receive_cplt(dma_handle_t *p_dma);
static void i2c_dma_slave_transmit_cplt(dma_handle_t *p_dma);
static void i2c_dma_slave_receive_cplt(dma_handle_t *p_dma);
static void i2c_dma_error(dma_handle_t *p_dma);
static void i2c_dma_abort(dma_handle_t *p_dma);

/* Private functions to handle IT transfer */
static void i2c_it_master_cplt(i2c_handle_t *p_i2c);
static void i2c_it_slave_cplt(i2c_handle_t *p_i2c);
static void i2c_it_error(i2c_handle_t *p_i2c, uint32_t error_code);

static void i2c_set_device_state(i2c_handle_t *p_i2c, hal_i2c_state_t state);

/** @} */
#ifdef  USE_FULL_ASSERT
#include "gr_assert.h"
#else
#ifndef gr_assert_param
#define gr_assert_param(expr) ((void)0U)
#endif
#endif

#define IS_LL_I2C_OWN_ADDRESS(__VALUE__)       (((__VALUE__) <= 0x000003FFU) && \
                                                ((__VALUE__) > 0x07U) && \
                                                (((__VALUE__) < 0x78U) || ((__VALUE__) > 0x7FU)))

#define IS_LL_I2C_OWN_ADDRSIZE(__VALUE__)      (((__VALUE__) == LL_I2C_OWNADDRESS_7BIT) || \
                                                 ((__VALUE__) == LL_I2C_OWNADDRESS_10BIT))

#define IS_LL_I2C_SPEED(__VALUE__)             (((__VALUE__) == LL_I2C_SPEED_100K)  || \
                                                ((__VALUE__) == LL_I2C_SPEED_400K)  || \
                                                ((__VALUE__) == LL_I2C_SPEED_1000K) || \
                                                ((__VALUE__) == LL_I2C_SPEED_3500K))
/** @} */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup I2C_LL_Exported_Functions
  * @{
  */

/** @addtogroup I2C_LL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize the I2C registers to their default reset values.
  * @param  I2Cx I2C instance.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: I2C registers are de-initialized
  *          - ERROR: I2C registers are not de-initialized
  */
__WEAK void ll_i2c_deinit(i2c_regs_t *I2Cx)
{
    /* Check the I2C instance I2Cx */
    gr_assert_param(IS_I2C_ALL_INSTANCE(I2Cx));

    ll_i2c_disable(I2Cx);

    /* IC_INTR_MASK register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, INT_MASK, 0x00000000U);
    /* Clear interrupt. */
    ll_i2c_clear_flag_intr(I2Cx);

    /* IC_CON register set to default reset values. */
    LL_I2C_WriteReg(I2Cx, CTRL, 0x0000007FU);
    /* IC_TAR register set to default reset values. */
    LL_I2C_WriteReg(I2Cx, TARGET_ADDR, 0x00000055U);
    /* IC_SAR register set to default reset values. */
    LL_I2C_WriteReg(I2Cx, S_ADDR, 0x00000055U);

    /* IC_SS_SCL_HCNT register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, SS_CLK_HCOUNT, 0x00000190U);
    /* IC_SS_SCL_LCNT register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, SS_CLK_LCOUNT, 0x000001d6U);
    /* IC_FS_SCL_HCNT register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, FS_CLK_HCOUNT, 0x0000003cU);
    /* IC_FS_SCL_LCNT register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, FS_CLK_LCOUNT, 0x00000082U);
    /* IC_HS_SCL_HCNT register set to default reset values.*/

    /* IC_HS_SCL_LCNT register set to default reset values.*/


    /* IC_RX_TL register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, RX_FIFO_THD, 0x00000000U);
    /* IC_TX_TL register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, TX_FIFO_THD, 0x00000000U);

    /* IC_DMA_CR register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, DMA_CTRL, 0x00000000U);
    /* IC_DMA_TDLR register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, DMA_TX_LEVEL, 0x00000000U);
    /* IC_DMA_RDLR register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, DMA_RX_LEVEL, 0x00000000U);

    /* IC_SDA_HOLD register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, SDA_HOLD, 0x00000001U);
    /* IC_SDA_SETUP register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, SDA_SETUP, 0x00000064U);

    /* IC_ACK_GENERAL_CALL register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, ACK_GEN_CALL, 0x00000001U);

    /* IC_SCL_STUCK_AT_LOW_TIMEOUT register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, SCL_STUCK_TIMEOUT, 0xFFFFFFFFU);
    /* IC_SDA_STUCK_AT_LOW_TIMEOUT register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, SDA_STUCK_TIMEOUT, 0xFFFFFFFFU);

    /* IC_FS_SPKLEN register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, FS_SPKLEN, 0x00000005U);
    /* IC_HS_SPKLEN register set to default reset values.*/
}

/**
  * @brief  Initialize the I2C registers according to the specified parameters in p_i2c_init.
  * @param  I2Cx I2C instance.
  * @param  p_i2c_init pointer to a @ref ll_i2c_init_t structure.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: I2C registers are initialized
  *          - ERROR: Not applicable
  */
__WEAK void ll_i2c_init(i2c_regs_t *I2Cx, ll_i2c_init_t *p_i2c_init)
{
    /* Check the I2C instance I2Cx */
    gr_assert_param(IS_I2C_ALL_INSTANCE(I2Cx));
    /* Check the I2C parameters from I2C_InitStruct */
    gr_assert_param(IS_LL_I2C_OWN_ADDRESS(p_i2c_init->own_address));
    gr_assert_param(IS_LL_I2C_OWN_ADDRSIZE(p_i2c_init->own_addr_size));
    gr_assert_param(IS_LL_I2C_SPEED(p_i2c_init->speed));
}

/**
  * @brief  Set each @ref ll_i2c_init_t field to default value.
  * @param  p_i2c_init Pointer to a @ref ll_i2c_init_t structure.
  * @retval None
  */
__WEAK void ll_i2c_struct_init(ll_i2c_init_t *p_i2c_init)
{
    /* Set I2C_InitStruct fields to default values */
    p_i2c_init->speed           = LL_I2C_SPEED_400K;
    p_i2c_init->own_address     = 0x55U;
    p_i2c_init->own_addr_size   = LL_I2C_OWNADDRESS_7BIT;
}

/* Exported functions --------------------------------------------------------*/

/** @defgroup I2C_Exported_Functions I2C Exported Functions
  * @{
  */

/** @defgroup I2C_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions
  * @{
  */
__WEAK hal_status_t hal_i2c_init(i2c_handle_t *p_i2c)
{
    uint32_t hcnt, lcnt;

#if defined(HAL_PARAMS_CHECK)
    /* Check the parameters */
    gr_assert_param(IS_I2C_ALL_INSTANCE(p_i2c->p_instance));
    gr_assert_param(IS_I2C_SPEED(p_i2c->init.speed));
    gr_assert_param(IS_I2C_OWN_ADDRESS(p_i2c->init.own_address));
    gr_assert_param(IS_I2C_ADDRESSING_MODE(p_i2c->init.addressing_mode));
    gr_assert_param(IS_I2C_GENERAL_CALL(p_i2c->init.general_call_mode));
#endif

#if defined(HAL_STATUS_CHECK)
    if (HAL_I2C_STATE_RESET == p_i2c->state)
    {
#endif
        /* Enable Clock for Serial blocks and Automatic turn off Serial blocks clock during WFI. */
#ifdef HAL_CLOCK_UNIFORM_CONTROL
        //lint -e923 Cast from pointer to unsigned int is necessary
        hal_clock_enable_module((uint32_t)p_i2c->p_instance);
#endif
#ifdef HAL_CLOCK_MODULE
        ll_cgc_disable_force_off_serial_hclk();
        ll_cgc_disable_wfi_off_serial_hclk();

        /* Enable I2Cx Clock */
        //lint -e923 Cast from pointer to unsigned int is necessary
        if(p_i2c->p_instance == I2C0)
        {
            ll_cgc_disable_force_off_i2c0_hclk();
            ll_cgc_disable_i2c0_slp_wfi();
        }
        //lint -e923 Cast from pointer to unsigned int is necessary
        else if(p_i2c->p_instance == I2C1)
        {
            ll_cgc_disable_force_off_i2c1_hclk();
            ll_cgc_disable_i2c1_slp_wfi();
        }
        else
        {
            /* Do nothing*/
        }
#endif
        /* init the low level hardware : GPIO, CLOCK, CORTEX...etc */
        hal_i2c_msp_init(p_i2c);
#if defined(HAL_STATUS_CHECK)
    }
#endif

#if defined(HAL_STATUS_CHECK)
    i2c_set_device_state(p_i2c, HAL_I2C_STATE_BUSY);
#endif

    /* Disable the selected I2C peripheral */
    ll_i2c_disable(p_i2c->p_instance);

    /* Configure I2Cx: Frequency range */
    ll_i2c_set_speed_mode(p_i2c->p_instance, __LL_I2C_CONVERT_SPEED_MODE(p_i2c->init.speed));

    lcnt = (GetSerialClock() / 2U) / p_i2c->init.speed;
    hcnt = ((GetSerialClock() / 2U) / p_i2c->init.speed) - (I2C_CALCULATION_CONSTANT + I2C_FS_SPKLEN);
    ll_i2c_set_spike_len_fs(p_i2c->p_instance, I2C_FS_SPKLEN);
    if (p_i2c->init.speed < I2C_SPEED_400K)
    {
        ll_i2c_set_clock_high_period_ss(p_i2c->p_instance, hcnt);
        ll_i2c_set_clock_low_period_ss(p_i2c->p_instance, lcnt);
    }
    else
    {
        ll_i2c_set_clock_high_period_fs(p_i2c->p_instance, hcnt);
        ll_i2c_set_clock_low_period_fs(p_i2c->p_instance, lcnt);
    }

    if((p_i2c->init.rx_hold_time / (1000000000U / GetSerialClock())) <= (hcnt - (I2C_FS_SPKLEN + I2C_CALCULATION_CONSTANT)))
    {
        ll_i2c_set_data_rx_hold_time(p_i2c->p_instance, (p_i2c->init.rx_hold_time / (1000000000U / GetSerialClock())));
    } else {
        ll_i2c_set_data_rx_hold_time(p_i2c->p_instance, (hcnt - (I2C_FS_SPKLEN + I2C_CALCULATION_CONSTANT)));
    }

    ll_i2c_set_data_tx_hold_time(p_i2c->p_instance, (p_i2c->init.tx_hold_time / (1000000000U / GetSerialClock())));

    /* Configure I2Cx: Own Address, ack own address mode and Addressing Master mode */
    if (I2C_ADDRESSINGMODE_7BIT == p_i2c->init.addressing_mode)
    {
        ll_i2c_set_own_address(p_i2c->p_instance, p_i2c->init.own_address, LL_I2C_OWNADDRESS_7BIT);
        ll_i2c_set_master_addressing_mode(p_i2c->p_instance, LL_I2C_ADDRESSING_MODE_7BIT);
    }
    else /* I2C_ADDRESSINGMODE_10BIT */
    {
        ll_i2c_set_own_address(p_i2c->p_instance, p_i2c->init.own_address, LL_I2C_OWNADDRESS_10BIT);
        ll_i2c_set_master_addressing_mode(p_i2c->p_instance, LL_I2C_ADDRESSING_MODE_10BIT);
    }

    /* Configure I2Cx: Generalcall mode */
    if (I2C_GENERALCALL_ENABLE == p_i2c->init.general_call_mode)
    {
        ll_i2c_enable_general_call(p_i2c->p_instance);
    }
    else
    {
        ll_i2c_disable_general_call(p_i2c->p_instance);
    }
    /* eanble bus clear feature */
    ll_i2c_enable_bus_clear_feature(p_i2c->p_instance);

    /* set at low timerout 1ms for scl and sda*/
    ll_i2c_set_scl_stuck_at_low_timeout(p_i2c->p_instance, GetSerialClock() / 10U);
    ll_i2c_set_sda_stuck_at_low_timeout(p_i2c->p_instance, GetSerialClock() / 10U);

    /* eanble hold bus feature if rx fifo full */
    ll_i2c_hold_bus_if_rx_full(p_i2c->p_instance);
    /* Clear all interrupt */
    ll_i2c_clear_flag_intr(p_i2c->p_instance);
    /* Disable all interrupt */
    ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_ALL);


#if defined(HAL_STATUS_CHECK)
    p_i2c->error_code        = HAL_I2C_ERROR_NONE;
    i2c_set_device_state(p_i2c, HAL_I2C_STATE_READY);
    p_i2c->previous_state    = I2C_STATE_NONE;
#endif

    p_i2c->mode              = HAL_I2C_MODE_NONE;

    return HAL_OK;
}

__WEAK hal_status_t hal_i2c_deinit(i2c_handle_t *p_i2c)
{
#if defined(HAL_PARAMS_CHECK)
    /* Check the parameters */
    gr_assert_param(IS_I2C_ALL_INSTANCE(p_i2c->p_instance));
#endif

#if defined(HAL_STATUS_CHECK)
    if (p_i2c->state != HAL_I2C_STATE_RESET)
    {
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_BUSY);
#endif
        /* Disable the I2C Peripheral Clock */
        ll_i2c_deinit(p_i2c->p_instance);

        /* DeInit the low level hardware: GPIO, CLOCK, NVIC */
        hal_i2c_msp_deinit(p_i2c);
#ifdef HAL_CLOCK_UNIFORM_CONTROL
        //lint -e923 Cast from pointer to unsigned int is necessary
        hal_clock_disable_module((uint32_t)p_i2c->p_instance);
#endif
#ifdef HAL_CLOCK_MODULE
        /* Disable I2Cx Clock */
        //lint -e923 Cast from pointer to unsigned int is necessary
        if(p_i2c->p_instance == I2C0)
        {
            ll_cgc_enable_force_off_i2c0_hclk();
            ll_cgc_enable_i2c0_slp_wfi();
        }
        else //p_i2c->p_instance == I2C1
        {
            ll_cgc_enable_force_off_i2c1_hclk();
            ll_cgc_enable_i2c1_slp_wfi();
        }

        GLOBAL_EXCEPTION_DISABLE();
        if(((LL_CGC_FRC_SERIALS_HCLK2 & ll_cgc_get_force_off_hclk_2()) == LL_CGC_FRC_SERIALS_HCLK2))
        {
            /* Disable Clock for Serial blocks  */
            ll_cgc_enable_force_off_serial_hclk();
        }
        if(((LL_CGC_MCU_PERIPH_SERIALS_SLP_OFF & ll_cgc_get_wfi_off_hclk_3()) == LL_CGC_MCU_PERIPH_SERIALS_SLP_OFF))
        {
            /* Disable Clock for Serial blocks  */
            ll_cgc_enable_wfi_off_serial_hclk();
        }
        GLOBAL_EXCEPTION_ENABLE();
#endif

#if defined(HAL_STATUS_CHECK)
        p_i2c->error_code        = HAL_I2C_ERROR_NONE;
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_RESET);
        p_i2c->previous_state    = I2C_STATE_NONE;
#endif

        p_i2c->mode              = HAL_I2C_MODE_NONE;
#if defined(HAL_STATUS_CHECK)
    }
#endif

    return HAL_OK;
}

__WEAK void hal_i2c_msp_init(i2c_handle_t *p_i2c)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_i2c);
}

__WEAK void hal_i2c_msp_deinit(i2c_handle_t *p_i2c)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_i2c);
}

/** @} */

/** @defgroup I2C_Exported_Functions_Group2 Input and Output operation functions
  * @brief   Data transfers functions
  * @{
  */

__WEAK hal_status_t hal_i2c_master_transmit(i2c_handle_t *p_i2c, uint16_t dev_address, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
    hal_status_t status = HAL_OK;

#if defined(HAL_PARAMS_CHECK)
    if ((NULL == p_data) || (0U == size))
    {
        p_i2c->error_code |= HAL_I2C_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }
#endif

#if defined(HAL_STATUS_CHECK)
    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
#endif
        if (HAL_OK != i2c_wait_on_sta_flag_until_timeout(p_i2c, I2C_STAT_ACTIVITY, I2C_STAT_ACTIVITY, I2C_TIMEOUT_BUSY))
        {
            return HAL_TIMEOUT;
        }
#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_BUSY_TX);
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;
#endif
        p_i2c->mode          = HAL_I2C_MODE_MASTER;


        /* Prepare transfer parameters */
        p_i2c->p_buffer      = p_data;
        p_i2c->xfer_size     = size;
        p_i2c->xfer_count    = size;
        p_i2c->xfer_isr      = NULL;

        /* Enable Master Mode and Set Slave Address */
        i2c_master_transfer_config(p_i2c, dev_address);
        /* Disable all interrupt */
        ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_ALL);

        /* Enable the selected I2C peripheral */
        ll_i2c_enable(p_i2c->p_instance);

        status = i2c_master_start_transmit(p_i2c, timeout);

        /* Disable the selected I2C peripheral */
        ll_i2c_disable(p_i2c->p_instance);
#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_READY);
#endif
        p_i2c->mode  = HAL_I2C_MODE_NONE;

#if defined(HAL_STATUS_CHECK)
    }
    else
    {
        status = HAL_BUSY;
    }
#endif
    return status;
}

__WEAK hal_status_t hal_i2c_master_receive(i2c_handle_t *p_i2c, uint16_t dev_address, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
    hal_status_t status = HAL_OK;

#if defined(HAL_PARAMS_CHECK)
    if ((NULL == p_data) || (0U == size))
    {
        p_i2c->error_code |= HAL_I2C_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }
#endif

#if defined(HAL_STATUS_CHECK)
    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
#endif
        if (HAL_OK != i2c_wait_on_sta_flag_until_timeout(p_i2c, I2C_STAT_ACTIVITY, I2C_STAT_ACTIVITY, I2C_TIMEOUT_BUSY))
        {
            return HAL_TIMEOUT;
        }
#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_BUSY_RX);
        p_i2c->error_code        = HAL_I2C_ERROR_NONE;
#endif
        p_i2c->mode              = HAL_I2C_MODE_MASTER;


        /* Prepare transfer parameters */
        p_i2c->p_buffer          = p_data;
        p_i2c->xfer_size         = size;
        p_i2c->master_ack_count  = size;
        p_i2c->xfer_count        = size;
        p_i2c->xfer_isr          = NULL;

        /* Enable Master Mode and Set Slave Address */
        i2c_master_transfer_config(p_i2c, dev_address);
        /* Disable all interrupt */
        ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_ALL);

        /* Enable the selected I2C peripheral */
        ll_i2c_enable(p_i2c->p_instance);

        status = i2c_master_start_receive(p_i2c, timeout);

        /* Disable the selected I2C peripheral */
        ll_i2c_disable(p_i2c->p_instance);
#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_READY);
#endif
        p_i2c->mode  = HAL_I2C_MODE_NONE;
#if defined(HAL_STATUS_CHECK)
    }
    else
    {
        status = HAL_BUSY;
    }
#endif
    return status;
}

__WEAK hal_status_t hal_i2c_slave_transmit(i2c_handle_t *p_i2c, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
    hal_status_t status = HAL_OK;

#if defined(HAL_PARAMS_CHECK)
    if ((NULL == p_data) || (0U == size))
    {
        p_i2c->error_code |= HAL_I2C_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }
#endif

#if defined(HAL_STATUS_CHECK)
    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
#endif
        if (HAL_OK != i2c_wait_on_sta_flag_until_timeout(p_i2c, I2C_STAT_ACTIVITY, I2C_STAT_ACTIVITY, I2C_TIMEOUT_BUSY))
        {
            return HAL_TIMEOUT;
        }

#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_BUSY_TX);
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;
#endif
        p_i2c->mode          = HAL_I2C_MODE_SLAVE;

        /* Prepare transfer parameters */
        p_i2c->p_buffer      = p_data;
        p_i2c->xfer_count    = size;
        p_i2c->xfer_size     = size;
        p_i2c->xfer_isr      = NULL;

        /* Enable Slave Mode */
        i2c_slave_transfer_config(p_i2c);
        /* Disable all interrupt */
        ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_ALL);

        /* Enable the selected I2C peripheral */
        ll_i2c_enable(p_i2c->p_instance);

        status = i2c_slave_start_transmit(p_i2c, timeout);

        /* Disable the selected I2C peripheral */
        ll_i2c_disable(p_i2c->p_instance);
#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_READY);
#endif
        p_i2c->mode  = HAL_I2C_MODE_NONE;

#if defined(HAL_STATUS_CHECK)
    }
    else
    {
        status = HAL_BUSY;
    }
#endif
    return status;
}

__WEAK hal_status_t hal_i2c_slave_receive(i2c_handle_t *p_i2c, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
    hal_status_t status = HAL_OK;

#if defined(HAL_PARAMS_CHECK)
    if ((NULL == p_data) || (0U == size))
    {
        p_i2c->error_code |= HAL_I2C_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }
#endif

#if defined(HAL_STATUS_CHECK)
    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
#endif

#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_BUSY_RX);
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;
#endif
        p_i2c->mode          = HAL_I2C_MODE_SLAVE;

        /* Prepare transfer parameters */
        p_i2c->p_buffer      = p_data;
        p_i2c->xfer_count    = size;
        p_i2c->xfer_size     = size;
        p_i2c->xfer_isr      = NULL;

        /* Enable Slave Mode */
        i2c_slave_transfer_config(p_i2c);
        /* Disable all interrupt */
        ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_ALL);

        /* Enable the selected I2C peripheral */
        ll_i2c_enable(p_i2c->p_instance);

        status = i2c_slave_start_receive(p_i2c, timeout);

        /* Disable the selected I2C peripheral */
        ll_i2c_disable(p_i2c->p_instance);
#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_READY);
#endif
        p_i2c->mode  = HAL_I2C_MODE_NONE;

#if defined(HAL_STATUS_CHECK)
    }
    else
    {
        status = HAL_BUSY;
    }
#endif
    return status;
}

__WEAK hal_status_t hal_i2c_master_transmit_it(i2c_handle_t *p_i2c, uint16_t dev_address, uint8_t *p_data, uint16_t size)
{
#if defined(HAL_PARAMS_CHECK)
    if ((NULL == p_data) || (0U == size))
    {
        p_i2c->error_code |= HAL_I2C_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }
#endif

#if defined(HAL_STATUS_CHECK)
    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
#endif
        if (ll_i2c_is_active_flag_status_activity(p_i2c->p_instance))
        {
            return HAL_BUSY;
        }

#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_BUSY_TX);
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;
#endif
        p_i2c->mode          = HAL_I2C_MODE_MASTER;

        /* Prepare transfer parameters */
        p_i2c->p_buffer          = p_data;
        p_i2c->xfer_size         = size;
        p_i2c->xfer_count        = size;
        p_i2c->xfer_options      = I2C_NO_OPTION_FRAME;
        p_i2c->xfer_isr          = i2c_master_isr_it;

        /* Enable Master Mode and Set Slave Address */
        i2c_master_transfer_config(p_i2c, dev_address);

        /* Enable the selected I2C peripheral */
        ll_i2c_enable(p_i2c->p_instance);

        return i2c_master_start_transmit_it(p_i2c);
#if defined(HAL_STATUS_CHECK)
    }
    else
    {
        return HAL_BUSY;
    }
#endif
}

__WEAK hal_status_t hal_i2c_master_receive_it(i2c_handle_t *p_i2c, uint16_t dev_address, uint8_t *p_data, uint16_t size)
{
#if defined(HAL_PARAMS_CHECK)
    if ((NULL == p_data) || (0U == size))
    {
        p_i2c->error_code |= HAL_I2C_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }
#endif

#if defined(HAL_STATUS_CHECK)
    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
#endif
        if (ll_i2c_is_active_flag_status_activity(p_i2c->p_instance))
        {
            return HAL_BUSY;
        }

#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_BUSY_RX);
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;
#endif
        p_i2c->mode          = HAL_I2C_MODE_MASTER;

        /* Prepare transfer parameters */
        p_i2c->p_buffer          = p_data;
        p_i2c->xfer_size         = size;
        p_i2c->xfer_count        = size;
        p_i2c->master_ack_count  = size;
        p_i2c->xfer_options      = I2C_NO_OPTION_FRAME;
        p_i2c->xfer_isr          = i2c_master_isr_it;

        /* Enable Master Mode and Set Slave Address */
        i2c_master_transfer_config(p_i2c, dev_address);

        /* Enable the selected I2C peripheral */
        ll_i2c_enable(p_i2c->p_instance);

        return i2c_master_start_receive_it(p_i2c);
#if defined(HAL_STATUS_CHECK)
    }
    else
    {
        return HAL_BUSY;
    }
#endif
}

__WEAK hal_status_t hal_i2c_slave_transmit_it(i2c_handle_t *p_i2c, uint8_t *p_data, uint16_t size)
{
#if defined(HAL_PARAMS_CHECK)
    if ((NULL == p_data) || (0U == size))
    {
        p_i2c->error_code |= HAL_I2C_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }
#endif

#if defined(HAL_STATUS_CHECK)
    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
#endif
        if (ll_i2c_is_active_flag_status_activity(p_i2c->p_instance))
        {
            return HAL_BUSY;
        }

#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_BUSY_TX);
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;
#endif
        p_i2c->mode          = HAL_I2C_MODE_SLAVE;

        /* Prepare transfer parameters */
        p_i2c->p_buffer      = p_data;
        p_i2c->xfer_count    = size;
        p_i2c->xfer_size     = size;
        p_i2c->xfer_options  = I2C_NO_OPTION_FRAME;
        p_i2c->xfer_isr      = i2c_slave_isr_it;

        /* Enable Slave Mode */
        i2c_slave_transfer_config(p_i2c);
        /* Set FIFO threshold */
        ll_i2c_set_tx_fifo_threshold(p_i2c->p_instance, LL_I2C_TX_FIFO_TH_CHAR_3);

        /* Enable the selected I2C peripheral */
        ll_i2c_enable(p_i2c->p_instance);

        /* Note : The I2C interrupts must be enabled after unlocking current process
                to avoid the risk of I2C interrupt handle execution before current
                process unlock */

        /* Enable RD_REQ, STOP_DET interrupt */
        ll_i2c_enable_it(p_i2c->p_instance, I2C_XFER_LISTEN_IT);

        return HAL_OK;
#if defined(HAL_STATUS_CHECK)
    }
    else
    {
        return HAL_BUSY;
    }
#endif
}

__WEAK hal_status_t hal_i2c_slave_receive_it(i2c_handle_t *p_i2c, uint8_t *p_data, uint16_t size)
{
    uint32_t rxfifothreshold = LL_I2C_RX_FIFO_TH_CHAR_1;

#if defined(HAL_PARAMS_CHECK)
    if ((NULL == p_data) || (0U == size))
    {
        p_i2c->error_code |= HAL_I2C_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }
#endif

#if defined(HAL_STATUS_CHECK)
    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
#endif
        if (ll_i2c_is_active_flag_status_activity(p_i2c->p_instance))
        {
            return HAL_BUSY;
        }

#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_BUSY_RX);
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;
#endif
        p_i2c->mode          = HAL_I2C_MODE_SLAVE;

        /* Prepare transfer parameters */
        p_i2c->p_buffer      = p_data;
        p_i2c->xfer_count    = size;
        p_i2c->xfer_size     = size;
        p_i2c->xfer_options  = I2C_NO_OPTION_FRAME;
        p_i2c->xfer_isr      = i2c_slave_isr_it;

        /* Increase RX FIFO threshold when data size >= 5 */
        if (5U <= p_i2c->xfer_size)
        {
            rxfifothreshold = LL_I2C_RX_FIFO_TH_CHAR_5;
        }

        /* Enable Slave Mode */
        i2c_slave_transfer_config(p_i2c);
        /* Set FIFO threshold */
        ll_i2c_set_rx_fifo_threshold(p_i2c->p_instance, rxfifothreshold);

        /* Enable the selected I2C peripheral */
        ll_i2c_enable(p_i2c->p_instance);

        /* Note : The I2C interrupts must be enabled after unlocking current process
                to avoid the risk of I2C interrupt handle execution before current
                process unlock */

        /* Enable TX_ABRT, RX_FULL, RX_OVER, STOP_DET interrupt */
        ll_i2c_enable_it(p_i2c->p_instance, I2C_SLV_XFER_RX_IT);

        return HAL_OK;
#if defined(HAL_STATUS_CHECK)
    }
    else
    {
        return HAL_BUSY;
    }
#endif
}

__WEAK hal_status_t hal_i2c_master_transmit_dma(i2c_handle_t *p_i2c, uint16_t dev_address, uint8_t *p_data, uint16_t size)
{
#if defined(HAL_PARAMS_CHECK)
    if ((NULL == p_data) || (0U == size))
    {
        p_i2c->error_code |= HAL_I2C_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }
#endif

#if defined(HAL_STATUS_CHECK)
    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
#endif
        if (ll_i2c_is_active_flag_status_activity(p_i2c->p_instance))
        {
            return HAL_BUSY;
        }

        if (ll_dma_is_enabled_channel(p_i2c->p_dmatx->p_instance, p_i2c->p_dmatx->channel))
        {
            ll_dma_disable_channel(p_i2c->p_dmatx->p_instance, p_i2c->p_dmatx->channel);
        }

#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_BUSY_TX);
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;
#endif
        p_i2c->mode          = HAL_I2C_MODE_MASTER;

        /* Prepare transfer parameters */
        p_i2c->p_buffer      = p_data;
        p_i2c->xfer_size     = size;
        p_i2c->xfer_count    = size;
        p_i2c->xfer_options  = I2C_NO_OPTION_FRAME;
        p_i2c->xfer_isr      = i2c_master_isr_dma;

        /* Enable Master Mode and Set Slave Address */
        i2c_master_transfer_config(p_i2c, dev_address);

        /* Enable the selected I2C peripheral */
        ll_i2c_enable(p_i2c->p_instance);

        return i2c_master_start_transmit_dma(p_i2c);
#if defined(HAL_STATUS_CHECK)
    }
    else
    {
        return HAL_BUSY;
    }
#endif
}

__WEAK hal_status_t hal_i2c_master_receive_dma(i2c_handle_t *p_i2c, uint16_t dev_address, uint8_t *p_data, uint16_t size)
{
#if defined(HAL_PARAMS_CHECK)
    if ((NULL == p_data) || (0U == size))
    {
        p_i2c->error_code |= HAL_I2C_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }
#endif

#if defined(HAL_STATUS_CHECK)
    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
#endif
        if (ll_i2c_is_active_flag_status_activity(p_i2c->p_instance))
        {
            return HAL_BUSY;
        }

        if (ll_dma_is_enabled_channel(p_i2c->p_dmatx->p_instance, p_i2c->p_dmatx->channel))
        {
            ll_dma_disable_channel(p_i2c->p_dmatx->p_instance, p_i2c->p_dmatx->channel);
        }

        if (ll_dma_is_enabled_channel(p_i2c->p_dmarx->p_instance, p_i2c->p_dmarx->channel))
        {
            ll_dma_disable_channel(p_i2c->p_dmarx->p_instance, p_i2c->p_dmarx->channel);
        }

#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_BUSY_RX);
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;
#endif
        p_i2c->mode          = HAL_I2C_MODE_MASTER;

        /* Prepare transfer parameters */
        p_i2c->p_buffer      = p_data;
        p_i2c->xfer_size     = size;
        p_i2c->xfer_count    = size;
        p_i2c->xfer_options  = I2C_NO_OPTION_FRAME;
        p_i2c->xfer_isr      = i2c_master_isr_dma;

        /* Enable Master Mode and Set Slave Address */
        i2c_master_transfer_config(p_i2c, dev_address);

        /* Enable the selected I2C peripheral */
        ll_i2c_enable(p_i2c->p_instance);

        return i2c_master_start_receive_dma(p_i2c);
#if defined(HAL_STATUS_CHECK)
    }
    else
    {
        return HAL_BUSY;
    }
#endif
}

__WEAK hal_status_t hal_i2c_slave_transmit_dma(i2c_handle_t *p_i2c, uint8_t *p_data, uint16_t size)
{
    hal_status_t status = HAL_OK;

#if defined(HAL_PARAMS_CHECK)
    if ((NULL == p_data) || (0U == size))
    {
        p_i2c->error_code |= HAL_I2C_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }
#endif

#if defined(HAL_STATUS_CHECK)
    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
#endif
        if (ll_i2c_is_active_flag_status_activity(p_i2c->p_instance))
        {
            return HAL_BUSY;
        }

        if (ll_dma_is_enabled_channel(p_i2c->p_dmatx->p_instance, p_i2c->p_dmatx->channel))
        {
            ll_dma_disable_channel(p_i2c->p_dmatx->p_instance, p_i2c->p_dmatx->channel);
        }

#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_BUSY_TX);
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;
#endif
        p_i2c->mode          = HAL_I2C_MODE_SLAVE;

        /* Prepare transfer parameters */
        p_i2c->p_buffer      = p_data;
        p_i2c->xfer_size     = size;
        p_i2c->xfer_count    = size;
        p_i2c->xfer_options  = I2C_NO_OPTION_FRAME;
        p_i2c->xfer_isr      = i2c_slave_isr_dma;

        /* Set the I2C DMA transfer complete callback */
        p_i2c->p_dmatx->xfer_tfr_callback = i2c_dma_slave_transmit_cplt;
        /* Set the DMA error callback */
        p_i2c->p_dmatx->xfer_error_callback = i2c_dma_error;
        /* Set the unused DMA callbacks to NULL */
        p_i2c->p_dmatx->xfer_abort_callback = NULL;

        /* Enable Slave Mode */
        i2c_slave_transfer_config(p_i2c);

        /* Set DMA transfer data level */
        ll_i2c_set_dma_tx_data_level(p_i2c->p_instance, 4U);

        ll_dma_set_destination_burst_length(p_i2c->p_dmatx->p_instance, p_i2c->p_dmatx->channel, LL_DMA_DST_BURST_LENGTH_4);

        /* Enable the DMA channel */
        status = hal_dma_start_it(p_i2c->p_dmatx, (uint32_t)p_data, ll_i2c_dma_get_register_address(p_i2c->p_instance), p_i2c->xfer_size);

        if (HAL_OK == status)
        {
            /* Update XferCount value */
            p_i2c->xfer_count = 0;

            /* Enable the selected I2C peripheral */
            ll_i2c_enable(p_i2c->p_instance);

            /* Note : The I2C interrupts must be enabled after unlocking current process
                        to avoid the risk of I2C interrupt handle execution before current
                        process unlock */

            /* Enable RD_REQ interrupts */
            /* DMA Request need to be Enabled when RD_REQ interrupt occurred */
            ll_i2c_enable_it(p_i2c->p_instance, I2C_XFER_LISTEN_IT);
        }
        else
        {
#if defined(HAL_STATUS_CHECK)
            p_i2c->error_code |= HAL_I2C_ERROR_DMA;

            /* Update I2C state */
            i2c_set_device_state(p_i2c, HAL_I2C_STATE_READY);
#endif
        }
        return status;
#if defined(HAL_STATUS_CHECK)
    }
    else
    {
        return HAL_BUSY;
    }
#endif
}

__WEAK hal_status_t hal_i2c_slave_receive_dma(i2c_handle_t *p_i2c, uint8_t *p_data, uint16_t size)
{
    hal_status_t status = HAL_OK;

#if defined(HAL_PARAMS_CHECK)
    if ((NULL == p_data) || (0U == size))
    {
        p_i2c->error_code |= HAL_I2C_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }
#endif

#if defined(HAL_STATUS_CHECK)
    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
#endif
        if (ll_i2c_is_active_flag_status_activity(p_i2c->p_instance))
        {
            return HAL_BUSY;
        }

        if (ll_dma_is_enabled_channel(p_i2c->p_dmarx->p_instance, p_i2c->p_dmarx->channel))
        {
            ll_dma_disable_channel(p_i2c->p_dmarx->p_instance, p_i2c->p_dmarx->channel);
        }

#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_BUSY_RX);
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;
#endif
        p_i2c->mode          = HAL_I2C_MODE_SLAVE;

        /* Prepare transfer parameters */
        p_i2c->p_buffer      = p_data;
        p_i2c->xfer_size     = size;
        p_i2c->xfer_count    = size;
        p_i2c->xfer_options  = I2C_NO_OPTION_FRAME;
        p_i2c->xfer_isr      = i2c_slave_isr_dma;

        /* Set the I2C DMA transfer complete callback */
        p_i2c->p_dmarx->xfer_tfr_callback = i2c_dma_slave_receive_cplt;

        /* Set the DMA error callback */
        p_i2c->p_dmarx->xfer_error_callback = i2c_dma_error;

        /* Set the unused DMA callbacks to NULL */
        p_i2c->p_dmarx->xfer_abort_callback = NULL;

        /* Enable Slave Mode */
        i2c_slave_transfer_config(p_i2c);

        /* Set DMA transfer data level */
        ll_i2c_set_dma_rx_data_level(p_i2c->p_instance, 3U);

        ll_dma_set_source_burst_length(p_i2c->p_dmarx->p_instance, p_i2c->p_dmarx->channel, LL_DMA_SRC_BURST_LENGTH_4);

        /* Enable the DMA channel */
        status = hal_dma_start_it(p_i2c->p_dmarx, ll_i2c_dma_get_register_address(p_i2c->p_instance), (uint32_t)p_data, p_i2c->xfer_size);

        if (HAL_OK == status)
        {
            /* Update XferCount value */
            p_i2c->xfer_count = 0;

            /* Enable the selected I2C peripheral */
            ll_i2c_enable(p_i2c->p_instance);

            /* Note : The I2C interrupts must be enabled after unlocking current process
                        to avoid the risk of I2C interrupt handle execution before current
                        process unlock */

            /* Enable TX_ABORT interrupts */
            ll_i2c_enable_it(p_i2c->p_instance, I2C_XFER_ERROR_IT | I2C_XFER_CPLT_IT);

            /* Enable DMA Request */
            ll_i2c_enable_dma_req_rx(p_i2c->p_instance);
        }
        else
        {
#if defined(HAL_STATUS_CHECK)
            p_i2c->error_code |= HAL_I2C_ERROR_DMA;

            /* Update I2C state */
            i2c_set_device_state(p_i2c, HAL_I2C_STATE_READY);
#endif
        }
        return status;
#if defined(HAL_STATUS_CHECK)
    }
    else
    {
        return HAL_BUSY;
    }
#endif
}

__WEAK hal_status_t hal_i2c_mem_write(i2c_handle_t *p_i2c, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
    hal_status_t status = HAL_OK;

#if defined(HAL_PARAMS_CHECK)
    /* Check the parameters */
    gr_assert_param(IS_I2C_MEMADD_SIZE(mem_addr_size));

    if ((NULL == p_data) || (0U == size))
    {
        p_i2c->error_code |= HAL_I2C_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }
#endif

#if defined(HAL_STATUS_CHECK)
    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
#endif
        if (HAL_OK != i2c_wait_on_sta_flag_until_timeout(p_i2c, I2C_STAT_ACTIVITY, I2C_STAT_ACTIVITY, I2C_TIMEOUT_BUSY))
        {
            return HAL_TIMEOUT;
        }

#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_BUSY_TX);
#endif
        p_i2c->mode          = HAL_I2C_MODE_MASTER;
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        p_i2c->p_buffer      = p_data;
        p_i2c->xfer_size     = size;
        p_i2c->xfer_count    = size;
        p_i2c->xfer_isr      = NULL;

        /* Enable Master Mode and Set Slave Address */
        i2c_master_transfer_config(p_i2c, dev_address);
        /* Disable all interrupt */
        ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_ALL);

        /* After master re-configuration, TX FIFO should be empty */
        if (0U == ll_i2c_is_active_flag_status_tfe(p_i2c->p_instance))
        {
#if defined(HAL_STATUS_CHECK)
            i2c_set_device_state(p_i2c, HAL_I2C_STATE_READY);
#endif
            p_i2c->mode  = HAL_I2C_MODE_NONE;
            return HAL_ERROR;
        }

        /* Enable the selected I2C peripheral */
        ll_i2c_enable(p_i2c->p_instance);

        /* Send Slave Address and Memory Address */
        i2c_req_mem_read_write(p_i2c, dev_address, mem_address, mem_addr_size);

        status = i2c_master_start_transmit(p_i2c, timeout);

        /* Disable the selected I2C peripheral */
        ll_i2c_disable(p_i2c->p_instance);
#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_READY);
#endif
        p_i2c->mode  = HAL_I2C_MODE_NONE;

#if defined(HAL_STATUS_CHECK)
    }
    else
    {
        status = HAL_BUSY;
    }
#endif
    return status;
}

__WEAK hal_status_t hal_i2c_mem_read(i2c_handle_t *p_i2c, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
    hal_status_t status = HAL_OK;

#if defined(HAL_PARAMS_CHECK)
    /* Check the parameters */
    gr_assert_param(IS_I2C_MEMADD_SIZE(mem_addr_size));

    if ((NULL == p_data) || (0U == size))
    {
        p_i2c->error_code |= HAL_I2C_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }
#endif

#if defined(HAL_STATUS_CHECK)
    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
#endif
        if (HAL_OK != i2c_wait_on_sta_flag_until_timeout(p_i2c, I2C_STAT_ACTIVITY, I2C_STAT_ACTIVITY, I2C_TIMEOUT_BUSY))
        {
            return HAL_TIMEOUT;
        }

#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_BUSY_RX);
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;
#endif
        p_i2c->mode          = HAL_I2C_MODE_MASTER;

        /* Prepare transfer parameters */
        p_i2c->p_buffer          = p_data;
        p_i2c->xfer_size         = size;
        p_i2c->master_ack_count  = size;
        p_i2c->xfer_count        = size;
        p_i2c->xfer_isr          = NULL;

        /* Enable Master Mode and Set Slave Address */
        i2c_master_transfer_config(p_i2c, dev_address);
        /* Disable all interrupt */
        ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_ALL);

        /* After master re-configuration, TX FIFO should be empty */
        if (0U == ll_i2c_is_active_flag_status_tfe(p_i2c->p_instance))
        {
#if defined(HAL_STATUS_CHECK)
            i2c_set_device_state(p_i2c, HAL_I2C_STATE_READY);
#endif
            p_i2c->mode  = HAL_I2C_MODE_NONE;
            return HAL_ERROR;
        }

        /* Enable the selected I2C peripheral */
        ll_i2c_enable(p_i2c->p_instance);

        /* Send Slave Address and Memory Address */
        i2c_req_mem_read_write(p_i2c, dev_address, mem_address, mem_addr_size);

        status = i2c_master_start_receive(p_i2c, timeout);

        /* Disable the selected I2C peripheral */
        ll_i2c_disable(p_i2c->p_instance);
#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_READY);
#endif
        p_i2c->mode  = HAL_I2C_MODE_NONE;

#if defined(HAL_STATUS_CHECK)
    }
    else
    {
        status = HAL_BUSY;
    }
#endif
    return status;
}

__WEAK hal_status_t hal_i2c_mem_write_it(i2c_handle_t *p_i2c, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size)
{
#if defined(HAL_PARAMS_CHECK)
    /* Check the parameters */
    gr_assert_param(IS_I2C_MEMADD_SIZE(mem_addr_size));

    if ((NULL == p_data) || (0U == size))
    {
        p_i2c->error_code |= HAL_I2C_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }
#endif

#if defined(HAL_STATUS_CHECK)
    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
#endif
        if (ll_i2c_is_active_flag_status_activity(p_i2c->p_instance))
        {
            return HAL_BUSY;
        }

#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_BUSY_TX);
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;
#endif
        p_i2c->mode          = HAL_I2C_MODE_MASTER;

        /* Prepare transfer parameters */
        p_i2c->p_buffer          = p_data;
        p_i2c->xfer_size         = size;
        p_i2c->xfer_count        = size;
        p_i2c->master_ack_count  = size;
        p_i2c->xfer_options      = I2C_NO_OPTION_FRAME;
        p_i2c->xfer_isr          = i2c_master_isr_it;

        /* Enable Master Mode and Set Slave Address */
        i2c_master_transfer_config(p_i2c, dev_address);

        /* After master re-configuration, TX FIFO should be empty */
        if (0U == ll_i2c_is_active_flag_status_tfe(p_i2c->p_instance))
        {
            return HAL_ERROR;
        }

        /* Enable the selected I2C peripheral */
        ll_i2c_enable(p_i2c->p_instance);

        /* Send Slave Address and Memory Address */
        i2c_req_mem_read_write(p_i2c, dev_address, mem_address, mem_addr_size);

        return i2c_master_start_transmit_it(p_i2c);
#if defined(HAL_STATUS_CHECK)
    }
    else
    {
        return HAL_BUSY;
    }
#endif
}

__WEAK hal_status_t hal_i2c_mem_read_it(i2c_handle_t *p_i2c,
                                        uint16_t      dev_address,
                                        uint16_t      mem_address,
                                        uint16_t      mem_addr_size,
                                        uint8_t      *p_data,
                                        uint16_t      size)
{
#if defined(HAL_PARAMS_CHECK)
    /* Check the parameters */
    gr_assert_param(IS_I2C_MEMADD_SIZE(mem_addr_size));

    if ((NULL == p_data) || (0U == size))
    {
        p_i2c->error_code |= HAL_I2C_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }
#endif

#if defined(HAL_STATUS_CHECK)
    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
#endif
        if (ll_i2c_is_active_flag_status_activity(p_i2c->p_instance))
        {
            return HAL_BUSY;
        }

#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_BUSY_RX);
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;
#endif
        p_i2c->mode          = HAL_I2C_MODE_MASTER;

        /* Prepare transfer parameters */
        p_i2c->p_buffer      = p_data;
        p_i2c->xfer_size     = size;
        p_i2c->xfer_count    = size;
        p_i2c->master_ack_count  = size;
        p_i2c->xfer_options  = I2C_NO_OPTION_FRAME;
        p_i2c->xfer_isr      = i2c_master_isr_it;

        /* Enable Master Mode and Set Slave Address */
        i2c_master_transfer_config(p_i2c, dev_address);

        /* After master re-configuration, TX FIFO should be empty */
        if (0U == ll_i2c_is_active_flag_status_tfe(p_i2c->p_instance))
        {
            return HAL_ERROR;
        }

        /* Enable the selected I2C peripheral */
        ll_i2c_enable(p_i2c->p_instance);

        /* Send Slave Address and Memory Address */
        i2c_req_mem_read_write(p_i2c, dev_address, mem_address, mem_addr_size);

        return i2c_master_start_receive_it(p_i2c);
#if defined(HAL_STATUS_CHECK)
    }
    else
    {
        return HAL_BUSY;
    }
#endif
}

__WEAK hal_status_t hal_i2c_mem_write_dma(i2c_handle_t *p_i2c,
                                          uint16_t      dev_address,
                                          uint16_t      mem_address,
                                          uint16_t      mem_addr_size,
                                          uint8_t      *p_data,
                                          uint16_t      size)
{
#if defined(HAL_PARAMS_CHECK)
    /* Check the parameters */
    gr_assert_param(IS_I2C_MEMADD_SIZE(mem_addr_size));

    if ((NULL == p_data) || (0U == size))
    {
        p_i2c->error_code |= HAL_I2C_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }
#endif

#if defined(HAL_STATUS_CHECK)
    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
#endif
        if (ll_i2c_is_active_flag_status_activity(p_i2c->p_instance))
        {
            return HAL_BUSY;
        }

        if (ll_dma_is_enabled_channel(p_i2c->p_dmatx->p_instance, p_i2c->p_dmatx->channel))
        {
            ll_dma_disable_channel(p_i2c->p_dmatx->p_instance, p_i2c->p_dmatx->channel);
        }

#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_BUSY_TX);
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;
#endif
        p_i2c->mode          = HAL_I2C_MODE_MASTER;

        /* Prepare transfer parameters */
        p_i2c->p_buffer      = p_data;
        p_i2c->xfer_size     = size;
        p_i2c->xfer_count    = size;
        p_i2c->xfer_options  = I2C_NO_OPTION_FRAME;
        p_i2c->xfer_isr      = i2c_master_isr_dma;

        /* Enable Master Mode and Set Slave Address */
        i2c_master_transfer_config(p_i2c, dev_address);

        /* After master re-configuration, TX FIFO should be empty */
        if (0U == ll_i2c_is_active_flag_status_tfe(p_i2c->p_instance))
        {
            return HAL_ERROR;
        }

        /* Enable the selected I2C peripheral */
        ll_i2c_enable(p_i2c->p_instance);

        /* Send Slave Address and Memory Address */
        i2c_req_mem_read_write(p_i2c, dev_address, mem_address, mem_addr_size);

        return i2c_master_start_transmit_dma(p_i2c);
#if defined(HAL_STATUS_CHECK)
    }
    else
    {
        return HAL_BUSY;
    }
#endif
}

__WEAK hal_status_t hal_i2c_mem_read_dma(i2c_handle_t *p_i2c,
                                         uint16_t      dev_address,
                                         uint16_t      mem_address,
                                         uint16_t      mem_addr_size,
                                         uint8_t      *p_data,
                                         uint16_t      size)
{
#if defined(HAL_PARAMS_CHECK)
    /* Check the parameters */
    gr_assert_param(IS_I2C_MEMADD_SIZE(mem_addr_size));

    if ((NULL == p_data) || (0U == size))
    {
        p_i2c->error_code |= HAL_I2C_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }
#endif

#if defined(HAL_STATUS_CHECK)
    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
#endif
        if (ll_i2c_is_active_flag_status_activity(p_i2c->p_instance))
        {
            return HAL_BUSY;
        }

        if (ll_dma_is_enabled_channel(p_i2c->p_dmatx->p_instance, p_i2c->p_dmatx->channel))
        {
            ll_dma_disable_channel(p_i2c->p_dmatx->p_instance, p_i2c->p_dmatx->channel);
        }

        if (ll_dma_is_enabled_channel(p_i2c->p_dmarx->p_instance, p_i2c->p_dmarx->channel))
        {
            ll_dma_disable_channel(p_i2c->p_dmarx->p_instance, p_i2c->p_dmarx->channel);
        }

#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_BUSY_RX);
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;
#endif
        p_i2c->mode          = HAL_I2C_MODE_MASTER;

        /* Prepare transfer parameters */
        p_i2c->p_buffer      = p_data;
        p_i2c->xfer_size     = size;
        p_i2c->xfer_count    = size;
        p_i2c->xfer_options  = I2C_NO_OPTION_FRAME;
        p_i2c->xfer_isr      = i2c_master_isr_dma;

        /* Enable Master Mode and Set Slave Address */
        i2c_master_transfer_config(p_i2c, dev_address);

        /* After master re-configuration, TX FIFO should be empty */
        if (0U == ll_i2c_is_active_flag_status_tfe(p_i2c->p_instance))
        {
            return HAL_ERROR;
        }

        /* Enable the selected I2C peripheral */
        ll_i2c_enable(p_i2c->p_instance);

        /* Send Slave Address and Memory Address */
        i2c_req_mem_read_write(p_i2c, dev_address, mem_address, mem_addr_size);

        return i2c_master_start_receive_dma(p_i2c);
#if defined(HAL_STATUS_CHECK)
    }
    else
    {
        return HAL_BUSY;
    }
#endif
}

__WEAK hal_status_t hal_i2c_master_abort_it(i2c_handle_t *p_i2c)
{
#if defined(HAL_PARAMS_CHECK)

#endif

    hal_status_t ret = HAL_ERROR;

    if (HAL_I2C_MODE_MASTER == p_i2c->mode)
    {
        /* Disable Interrupts */
        ll_i2c_disable_it(p_i2c->p_instance, I2C_MST_XFER_RX_IT | I2C_MST_XFER_TX_IT);

#if defined(HAL_STATUS_CHECK)
        /* Set previous state */
        if (HAL_I2C_STATE_BUSY_TX == p_i2c->state)
        {
            p_i2c->previous_state = I2C_STATE_MASTER_BUSY_TX;
        }
        else if (HAL_I2C_STATE_BUSY_RX == p_i2c->state)
        {
            p_i2c->previous_state = I2C_STATE_MASTER_BUSY_RX;
        }
        else
        {
            /* DO nothing */
        }

        /* Set State at HAL_I2C_STATE_ABORT */
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_ABORT);
#endif
        /* Abort DMA RX transfer if any */
        if ((0U != ll_i2c_is_enabled_dma_req_rx(p_i2c->p_instance)) && (NULL != p_i2c->p_dmarx))
        {
            p_i2c->p_dmarx->xfer_abort_callback = NULL;

            /* Abort DMA TX and Rx */
            if ((HAL_I2C_MODE_MASTER == p_i2c->mode) && (NULL != p_i2c->p_dmatx))
            {
                /* Master receive need to abort Tx too. */
                p_i2c->p_dmatx->xfer_abort_callback = NULL;
                if(HAL_OK == hal_dma_abort_it(p_i2c->p_dmatx))
                {
                    ll_i2c_disable_dma_req_tx(p_i2c->p_instance);
                }
            }

            if(HAL_OK == hal_dma_abort_it(p_i2c->p_dmarx))
            {
                ll_i2c_disable_dma_req_rx(p_i2c->p_instance);
            }

            p_i2c->xfer_count = (uint16_t)I2C_GET_RX_DMA_REMAIN_DATA(p_i2c);/*lint !e666 MISRA exception. Allow repeated parameter in macro */
        }
        /* Abort DMA TX transfer if any */
        else if ((0U != ll_i2c_is_enabled_dma_req_tx(p_i2c->p_instance)) && (NULL != p_i2c->p_dmarx))
        {
            p_i2c->p_dmatx->xfer_abort_callback = NULL;

            /* Abort DMA TX */
            if(HAL_OK == hal_dma_abort_it(p_i2c->p_dmatx))
            {
                ll_i2c_disable_dma_req_tx(p_i2c->p_instance);
            }
            p_i2c->xfer_count = (uint16_t)I2C_GET_TX_DMA_REMAIN_DATA(p_i2c);/*lint !e666 MISRA exception. Allow repeated parameter in macro */
        }
        else
        {
            /* Do nothing */
        }

        /* issues a STOP and flushes the Tx FIFO after completing the current transfer */
        ll_i2c_enable_transfer_abort(p_i2c->p_instance);

        /* Note : The I2C interrupts must be enabled after unlocking current process
                    to avoid the risk of I2C interrupt handle execution before current
                    process unlock */
        ll_i2c_enable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_TX_ABRT | LL_I2C_INTR_MASK_STOP_DET);

        ret = HAL_OK;
    }
    else
    {
        /* Wrong usage of abort function */
        /* This function should be used only in case of abort monitored by master device */
        ret = HAL_ERROR;
    }
    return ret;
}

/** @} */

/** @defgroup I2C_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
 * @{
 */

__WEAK void hal_i2c_irq_handler(i2c_handle_t *p_i2c)
{
    /* Get current IT sources value */
    uint32_t itsources    = READ_REG(p_i2c->p_instance->INT_STAT);
    uint32_t abortsources = 0U;

    if (itsources & LL_I2C_INTR_STAT_TX_ABRT)
    {
        abortsources = ll_i2c_get_abort_source(p_i2c->p_instance);

        if(ll_i2c_is_enabled_dma_req_tx(p_i2c->p_instance))
        {
            p_i2c->p_dmatx->xfer_abort_callback = NULL;
            if(HAL_OK == hal_dma_abort_it(p_i2c->p_dmatx))
            {
                ll_i2c_disable_dma_req_tx(p_i2c->p_instance);
            }
        }

        /* Clear TX ABORT Flag */
        ll_i2c_clear_flag_tx_abort(p_i2c->p_instance);

        if ((0U != (abortsources & LL_I2C_ABRT_ARB_LOST)) || (0U != (abortsources & LL_I2C_ABRT_SLV_ARBLOST)))
        {
            p_i2c->error_code |= HAL_I2C_ERROR_ARB_LOST;
        }
        if (abortsources & LL_I2C_ABRT_SDA_STUCK_AT_LOW)
        {
            ll_i2c_enable_sda_stuck_recovery(p_i2c->p_instance);
            p_i2c->error_code |= HAL_I2C_ERROR_SDA_STUCK_AT_LOW;
        }
#if defined(HAL_STATUS_CHECK)
        bool temp_bool  = (I2C_STATE_MASTER_BUSY_TX == p_i2c->previous_state);
        bool temp_bool1  = (HAL_I2C_STATE_ABORT == p_i2c->state);
        temp_bool = temp_bool && temp_bool1;
        if ((HAL_I2C_STATE_BUSY_TX == p_i2c->state) || temp_bool)
        {
#endif
            /* Flushed data were send failed */
            uint16_t tx_flush_count = (uint16_t)ll_i2c_get_tx_flush_count(p_i2c->p_instance);
            p_i2c->xfer_count += tx_flush_count;
#if defined(HAL_STATUS_CHECK)
        }
#endif
    }

    if (itsources & LL_I2C_INTR_STAT_RX_OVER)
    {
        ll_i2c_clear_flag_rx_over(p_i2c->p_instance);
#if defined(HAL_STATUS_CHECK)
        p_i2c->error_code |= HAL_I2C_ERROR_OVER;
#endif
    }
#if I2C_MONITOR_SCL_STUCK_AT_LOW_ENABLE
    if (itsources & LL_I2C_INTR_STAT_SCL_STUCK_AT_LOW)
    {
        ll_i2c_clear_flag_scl_stuck_det(p_i2c->p_instance);
#if defined(HAL_STATUS_CHECK)
        p_i2c->error_code |= HAL_I2C_ERROR_SCL_STUCK_AT_LOW;
#endif
    }
#endif

    if (itsources & LL_I2C_INTR_STAT_STOP_DET)
    {
        ll_i2c_clear_flag_stop_det(p_i2c->p_instance);
    }

    if (HAL_I2C_ERROR_NONE != p_i2c->error_code)
    {
        i2c_it_error(p_i2c, p_i2c->error_code);
    }
    else if (NULL != p_i2c->xfer_isr)
    {
        p_i2c->xfer_isr(p_i2c, itsources, abortsources);
    }
    else
    {
        /* Do nothing */
    }
}

__WEAK void hal_i2c_master_tx_cplt_callback(i2c_handle_t *p_i2c)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_i2c);
}

__WEAK void hal_i2c_master_rx_cplt_callback(i2c_handle_t *p_i2c)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_i2c);
}

__WEAK void hal_i2c_slave_tx_cplt_callback(i2c_handle_t *p_i2c)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_i2c);
}

__WEAK void hal_i2c_slave_rx_cplt_callback(i2c_handle_t *p_i2c)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_i2c);
}

__WEAK void hal_i2c_error_callback(i2c_handle_t *p_i2c)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_i2c);
}

__WEAK void hal_i2c_abort_cplt_callback(i2c_handle_t *p_i2c)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_i2c);
}

/** @} */

/** @defgroup I2C_Exported_Functions_Group3 Peripheral State, Mode and Error functions
 *  @brief   Peripheral State, Mode and Error functions
  * @{
  */

__WEAK hal_i2c_state_t hal_i2c_get_state(const i2c_handle_t *p_i2c)
{
    /* Return I2C handle state */
    return p_i2c->state;
}

__WEAK hal_i2c_mode_t hal_i2c_get_mode(const i2c_handle_t *p_i2c)
{
    return p_i2c->mode;
}

#if defined(HAL_STATUS_CHECK)
__WEAK uint32_t hal_i2c_get_error(const i2c_handle_t *p_i2c)
{
    return p_i2c->error_code;
}
#endif

__WEAK uint32_t hal_i2c_sda_at_low_is_not_recovered(const i2c_handle_t *p_i2c)
{
    return ll_i2c_is_active_flag_status_sda_stuck_not_recovered(p_i2c->p_instance);
}

__WEAK void hal_i2c_suspend_reg(i2c_handle_t *p_i2c)
{
#ifndef HAL_HW_RES_SUSP_I2C
    i2c_regs_t *p_i2c_regs = p_i2c->p_instance;

    p_i2c->retention[0] = READ_REG(p_i2c_regs->CTRL);
    p_i2c->retention[1] = READ_REG(p_i2c_regs->SS_CLK_HCOUNT);
    p_i2c->retention[2] = READ_REG(p_i2c_regs->SS_CLK_LCOUNT);
    p_i2c->retention[3] = READ_REG(p_i2c_regs->FS_CLK_HCOUNT);
    p_i2c->retention[4] = READ_REG(p_i2c_regs->FS_CLK_LCOUNT);
#ifdef HAL_I2C_VERSION_OLD
    p_i2c->retention[5] = READ_REG(p_i2c_regs->HS_CLK_HCOUNT);
    p_i2c->retention[6] = READ_REG(p_i2c_regs->HS_CLK_LCOUNT);
#endif
    p_i2c->retention[7] = READ_REG(p_i2c_regs->S_ADDR);
    p_i2c->retention[8] = READ_REG(p_i2c_regs->ACK_GEN_CALL);
    p_i2c->retention[9] = READ_REG(p_i2c_regs->INT_MASK);
    p_i2c->retention[10] = READ_REG(p_i2c_regs->SDA_HOLD);
    p_i2c->retention[11] = READ_REG(p_i2c_regs->SCL_STUCK_TIMEOUT);
    p_i2c->retention[12] = READ_REG(p_i2c_regs->SDA_STUCK_TIMEOUT);
#else
    UNUSED(p_i2c);
#endif
}

__WEAK void hal_i2c_resume_reg(i2c_handle_t *p_i2c)
{
#ifndef HAL_HW_RES_SUSP_I2C
    i2c_regs_t *p_i2c_regs = p_i2c->p_instance;

    CLEAR_BITS(p_i2c_regs->EN, I2C_EN_ACTIVITY);
    WRITE_REG(p_i2c_regs->CTRL, p_i2c->retention[0]);
    WRITE_REG(p_i2c_regs->SS_CLK_HCOUNT, p_i2c->retention[1]);
    WRITE_REG(p_i2c_regs->SS_CLK_LCOUNT, p_i2c->retention[2]);
    WRITE_REG(p_i2c_regs->FS_CLK_HCOUNT, p_i2c->retention[3]);
    WRITE_REG(p_i2c_regs->FS_CLK_LCOUNT, p_i2c->retention[4]);
#ifdef HAL_I2C_VERSION_OLD
    WRITE_REG(p_i2c_regs->HS_CLK_HCOUNT, p_i2c->retention[5]);
    WRITE_REG(p_i2c_regs->HS_CLK_LCOUNT, p_i2c->retention[6]);
#endif
    WRITE_REG(p_i2c_regs->S_ADDR, p_i2c->retention[7]);
    WRITE_REG(p_i2c_regs->ACK_GEN_CALL, p_i2c->retention[8]);
    WRITE_REG(p_i2c_regs->INT_MASK, p_i2c->retention[9]);
    WRITE_REG(p_i2c_regs->SDA_HOLD, p_i2c->retention[10]);
    WRITE_REG(p_i2c_regs->SCL_STUCK_TIMEOUT, p_i2c->retention[11]);
    WRITE_REG(p_i2c_regs->SDA_STUCK_TIMEOUT, p_i2c->retention[12]);
    SET_BITS(p_i2c_regs->EN, I2C_EN_ACTIVITY);
#else
    UNUSED(p_i2c);
#endif
}

#ifdef HAL_PM_ENABLE
__WEAK hal_pm_status_t hal_pm_i2c_suspend(i2c_handle_t *p_i2c)
{
    hal_i2c_state_t state;
    state = hal_i2c_get_state(p_i2c);
    if ((state != HAL_I2C_STATE_READY) && (state != HAL_I2C_STATE_RESET))
    {
        return HAL_PM_ACTIVE;
    }
    else
    {
        hal_i2c_suspend_reg(p_i2c);
        return HAL_PM_SLEEP;
    }
}

__WEAK void hal_pm_i2c_resume(i2c_handle_t *p_i2c)
{
    hal_i2c_resume_reg(p_i2c);
}
#endif /* HAL_PM_ENABLE */

/** @} */

/** @} */

/** @addtogroup I2C_Private_Functions
  * @{
  */

/**
  * @brief  Handles I2Cx communication when starting a master transfer.
  * @param  p_i2c I2C handle.
  * @param  DevAddress Specifies the slave address to be programmed.
  *   This parameter must be a value between 0 and 0x3FF.
  * @retval None
  */
__STATIC_INLINE void i2c_master_transfer_config(i2c_handle_t *p_i2c, uint16_t dev_address)
{
#if defined(HAL_PARAMS_CHECK)
    /* Check the parameters */
    gr_assert_param(IS_I2C_ALL_INSTANCE(p_i2c->p_instance));
    gr_assert_param(IS_I2C_SLV_ADDRESS(dev_address));
#endif

    /* Enable Master Mode and Set Slave Address */
    ll_i2c_disable(p_i2c->p_instance);
    ll_i2c_enable_master_mode(p_i2c->p_instance);
    ll_i2c_set_slave_address(p_i2c->p_instance, dev_address);
    ll_i2c_enable(p_i2c->p_instance);

    /* CLear all interrupt */
    ll_i2c_clear_flag_intr(p_i2c->p_instance);
}

/**
  * @brief  Handles I2Cx communication when starting a slave transfer.
  * @param  p_i2c I2C handle.
  * @retval None
  */
__STATIC_INLINE void i2c_slave_transfer_config(i2c_handle_t *p_i2c)
{
#if defined(HAL_PARAMS_CHECK)
    /* Check the parameters */
    gr_assert_param(IS_I2C_ALL_INSTANCE(p_i2c->p_instance));
#endif

    /* Enable Slave Mode and Enable STOP_DET only if addressed */
    ll_i2c_disable(p_i2c->p_instance);
    ll_i2c_disable_master_mode(p_i2c->p_instance);
    ll_i2c_enable_stop_det_if_addressed(p_i2c->p_instance);
    ll_i2c_enable(p_i2c->p_instance);

    /* CLear all interrupt */
    ll_i2c_clear_flag_intr(p_i2c->p_instance);
}

static void i2c_master_check_error(i2c_handle_t *p_i2c)
{
    uint32_t     abortsrc;

    if (ll_i2c_is_active_flag_raw_tx_abort(p_i2c->p_instance))
    {
        abortsrc = ll_i2c_get_abort_source(p_i2c->p_instance);
        if (abortsrc & LL_I2C_ABRT_ARB_LOST)
        {
            p_i2c->error_code = HAL_I2C_ERROR_ARB_LOST;
        }
        else if ((abortsrc & I2C_TX_ABRT_NOACK))
        {
            p_i2c->error_code = HAL_I2C_ERROR_NOACK;
        }
        else if (abortsrc & LL_I2C_ABRT_SDA_STUCK_AT_LOW)
        {
            ll_i2c_enable_sda_stuck_recovery(p_i2c->p_instance);
            p_i2c->error_code |= HAL_I2C_ERROR_SDA_STUCK_AT_LOW;
        }
        else
        {
            /* Do nothing */
        }

        ll_i2c_clear_flag_tx_abort(p_i2c->p_instance);

        if (0U == (abortsrc & LL_I2C_ABRT_USER_ABRT))
        {
            /* Noting to do */
        }
    }
#if I2C_MONITOR_SCL_STUCK_AT_LOW_ENABLE
    else if(ll_i2c_is_active_flag_scl_stuck_at_low(p_i2c->p_instance))
    {
        ll_i2c_clear_flag_scl_stuck_det(p_i2c->p_instance);
        p_i2c->error_code = HAL_I2C_ERROR_SCL_STUCK_AT_LOW;
    }
#endif
    else
    {
        if (ll_i2c_is_active_flag_raw_rx_over(p_i2c->p_instance))
        {
            ll_i2c_clear_flag_rx_over(p_i2c->p_instance);
            p_i2c->error_code = HAL_I2C_ERROR_OVER;
        }
    }
}

static void i2c_slave_check_error(i2c_handle_t *p_i2c)
{
    uint32_t     abortsrc;

    if (ll_i2c_is_active_flag_raw_tx_abort(p_i2c->p_instance))
    {
        abortsrc = ll_i2c_get_abort_source(p_i2c->p_instance);
        if (abortsrc & LL_I2C_ABRT_SLV_ARBLOST)
        {
            p_i2c->error_code = HAL_I2C_ERROR_ARB_LOST;
        }
        else if ((abortsrc & I2C_TX_ABRT_NOACK))
        {
            p_i2c->error_code = HAL_I2C_ERROR_NOACK;
        }
        else
        {
            /* Do nothing */
        }

        ll_i2c_clear_flag_tx_abort(p_i2c->p_instance);
    }
    else
    {
        if (ll_i2c_is_active_flag_raw_rx_over(p_i2c->p_instance))
        {
            ll_i2c_clear_flag_rx_over(p_i2c->p_instance);
            p_i2c->error_code = HAL_I2C_ERROR_OVER;
        }
    }
}

/**
  * @brief  This function handles I2C Communication Timeout for specific usage of flag.
  * @param  p_i2c Pointer to a i2c_handle_t structure that contains
  *                the configuration information for the specified I2C.
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
static hal_status_t i2c_wait_on_flag_until_timeout(i2c_handle_t *p_i2c, __IM uint32_t *regs, uint32_t mask, \
        uint32_t status, uint32_t timeout)
{
    hal_status_t ret = HAL_OK;
    if((HAL_NEVER_TIMEOUT != timeout) && (HAL_MAX_DELAY < timeout))
    {
        return HAL_ERROR;
    }

    HAL_TIMEOUT_INIT();
    uint32_t tickstart = HAL_TIMEOUT_GET_TICK();
    /* Update SystemCoreClock */
    SystemCoreUpdateClock();
    uint32_t timeout_counter = ((SystemCoreClock / 1000U) * timeout);

    while ((*regs & mask) == status)
    {
        if(HAL_NEVER_TIMEOUT != timeout)
        {
            if((HAL_TIMEOUT_GET_TICK() - tickstart) > timeout_counter)
            {
    #if defined(HAL_STATUS_CHECK)
                p_i2c->error_code |= HAL_I2C_ERROR_TIMEOUT;
                i2c_set_device_state(p_i2c, HAL_I2C_STATE_READY);
    #endif
                p_i2c->mode = HAL_I2C_MODE_NONE;
                ret = HAL_TIMEOUT;
                goto EXIT;
            }
        }
    }
EXIT:
    HAL_TIMEOUT_DEINIT();
    return ret;
}

static hal_status_t i2c_wait_on_raw_flag_until_timeout(i2c_handle_t *p_i2c, uint32_t flag, uint32_t status, \
        uint32_t timeout)
{
    return i2c_wait_on_flag_until_timeout(p_i2c, &p_i2c->p_instance->RAW_INT_STAT, flag, status, timeout);
}

static hal_status_t i2c_wait_on_sta_flag_until_timeout(i2c_handle_t *p_i2c, uint32_t flag, uint32_t status, \
        uint32_t timeout)
{
    return i2c_wait_on_flag_until_timeout(p_i2c, &p_i2c->p_instance->STAT, flag, status, timeout);
}

static hal_status_t i2c_master_start_transmit(i2c_handle_t *p_i2c, uint32_t timeout)
{
    uint32_t cmd = LL_I2C_CMD_MST_WRITE;

    while (0U < p_i2c->xfer_count)
    {
        /* Wait until TFNF flag is set */
        if (HAL_OK != i2c_wait_on_sta_flag_until_timeout(p_i2c, I2C_STAT_TX_FIFO_NF, 0, timeout))
        {
            return HAL_TIMEOUT;
        }

        /* Generate STOP condition after transmit the last byte */
        if (1U == p_i2c->xfer_count)
        {
            cmd = LL_I2C_CMD_MST_WRITE | LL_I2C_CMD_MST_GEN_STOP;
        }

        ll_i2c_transmit_data8(p_i2c->p_instance, *p_i2c->p_buffer++, cmd);

        p_i2c->xfer_count--;
    }

    /* Wait until STOP_DET flag is set */
    if (HAL_OK != i2c_wait_on_raw_flag_until_timeout(p_i2c, I2C_RAW_INT_STAT_STOP_DET, 0, timeout))
    {
        return HAL_TIMEOUT;
    }

    /* Clear STOP_DET Flag */
    ll_i2c_clear_flag_stop_det(p_i2c->p_instance);

    /* Check what kind of error */
    i2c_master_check_error(p_i2c);
    if (p_i2c->error_code & (HAL_I2C_ERROR_ARB_LOST | HAL_I2C_ERROR_NOACK))
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

static hal_status_t i2c_master_start_receive(i2c_handle_t *p_i2c, uint32_t timeout)
{
    uint32_t cmd = LL_I2C_CMD_MST_READ;

    while (0U < p_i2c->xfer_count)
    {
        /* Write the READ command into TX FIFO to generate ACK */
        //lint -e931 No side effects
        while ((0U < p_i2c->master_ack_count))
        {
            uint32_t xfer_count = p_i2c->xfer_count;
            uint32_t master_ack_count = p_i2c->master_ack_count;
            if((xfer_count - master_ack_count) >=8U )
            {
                break;
            }
            /* Generate STOP condition after receive the last byte */
            if (1U == p_i2c->master_ack_count)
            {
                cmd = LL_I2C_CMD_MST_READ | LL_I2C_CMD_MST_GEN_STOP;
            }

            if (ll_i2c_is_active_flag_status_tfnf(p_i2c->p_instance))
            {
                ll_i2c_transmit_data8(p_i2c->p_instance, 0, cmd);
                p_i2c->master_ack_count--;
            }
            else
            {
                break;
            }
        }
        /* Wait until RFNE flag is set */
        if (HAL_OK != i2c_wait_on_sta_flag_until_timeout(p_i2c, I2C_STAT_RX_FIFO_NE, 0, timeout))
        {
            /* Check what kind of error */
            i2c_master_check_error(p_i2c);

            if (p_i2c->error_code & (HAL_I2C_ERROR_ARB_LOST | HAL_I2C_ERROR_OVER))
            {
                return HAL_ERROR;
            }
            return HAL_TIMEOUT;
        }

        *p_i2c->p_buffer++ = ll_i2c_receive_data8(p_i2c->p_instance);
        p_i2c->xfer_count--;
    }

    /* Wait until STOP_DET flag is set */
    if (HAL_OK != i2c_wait_on_raw_flag_until_timeout(p_i2c, I2C_RAW_INT_STAT_STOP_DET, 0, timeout))
    {
        return HAL_TIMEOUT;
    }

    /* Clear STOP Flag */
    ll_i2c_clear_flag_stop_det(p_i2c->p_instance);

    return HAL_OK;
}

static hal_status_t i2c_master_start_transmit_it(i2c_handle_t *p_i2c)
{
    /* Set FIFO threshold */
    ll_i2c_set_tx_fifo_threshold(p_i2c->p_instance, LL_I2C_TX_FIFO_TH_CHAR_3);

    /* Note : The I2C interrupts must be enabled after unlocking current process
            to avoid the risk of I2C interrupt handle execution before current
            process unlock */

    /* Enable TX_ABRT, TX_EMPTY, STOP_DET interrupt */
    ll_i2c_enable_it(p_i2c->p_instance, I2C_MST_XFER_TX_IT);

    return HAL_OK;
}

static hal_status_t i2c_master_start_receive_it(i2c_handle_t *p_i2c)
{
    uint32_t rxfifothreshold = LL_I2C_RX_FIFO_TH_CHAR_1;

    /* Increase RX FIFO threshold when data size >= 5 */
    if (5U <= p_i2c->xfer_size)
    {
        rxfifothreshold = LL_I2C_RX_FIFO_TH_CHAR_5;
    }

    /* Set FIFO threshold */
    ll_i2c_set_tx_fifo_threshold(p_i2c->p_instance, LL_I2C_TX_FIFO_TH_CHAR_3);
    ll_i2c_set_rx_fifo_threshold(p_i2c->p_instance, rxfifothreshold);

    /* Note : The I2C interrupts must be enabled after unlocking current process
            to avoid the risk of I2C interrupt handle execution before current
            process unlock */

    /* Enable TX_ABRT, TX_EMPTY, RX_FULL, RX_OVER, STOP_DET interrupt */
    ll_i2c_enable_it(p_i2c->p_instance, I2C_MST_XFER_RX_IT);

    return HAL_OK;
}

static hal_status_t i2c_master_start_transmit_dma(i2c_handle_t *p_i2c)
{
    hal_status_t status = HAL_OK;

    if (1U < p_i2c->xfer_size)
    {
        /* Set the I2C DMA transfer complete callback */
        p_i2c->p_dmatx->xfer_tfr_callback     = i2c_dma_master_transmit_cplt;
        /* Set the DMA error callback */
        p_i2c->p_dmatx->xfer_error_callback   = i2c_dma_error;
        /* Set the unused DMA callbacks to NULL */
        p_i2c->p_dmatx->xfer_abort_callback   = NULL;

        /* Set DMA transfer data level */
        ll_i2c_set_dma_tx_data_level(p_i2c->p_instance, 4U);

        ll_dma_set_destination_burst_length(p_i2c->p_dmatx->p_instance, p_i2c->p_dmatx->channel, LL_DMA_DST_BURST_LENGTH_4);

        /* Re-config increase mode and transfer width in case that HAL_I2C_Master_Receive_DMA has been called */
        ll_dma_set_source_increment_mode(p_i2c->p_dmatx->p_instance, p_i2c->p_dmatx->channel, p_i2c->p_dmatx->init.src_increment);
        ll_dma_set_source_width(p_i2c->p_dmatx->p_instance, p_i2c->p_dmatx->channel, p_i2c->p_dmatx->init.src_data_alignment);
        ll_dma_set_destination_width(p_i2c->p_dmatx->p_instance, p_i2c->p_dmatx->channel, p_i2c->p_dmatx->init.dst_data_alignment);

        /* Enable the DMA channel */
        status = hal_dma_start_it(p_i2c->p_dmatx, (uint32_t)(p_i2c->p_buffer), ll_i2c_dma_get_register_address(p_i2c->p_instance), p_i2c->xfer_size - 1U);

        if (HAL_OK == status)
        {
            /* Update Buffer & XferCount value */
            p_i2c->p_buffer = &p_i2c->p_buffer[p_i2c->xfer_size - 1U];
            p_i2c->xfer_count = 1;

            /* Enable DMA Request */
            ll_i2c_enable_dma_req_tx(p_i2c->p_instance);

            /* Note : The I2C interrupts must be enabled after unlocking current process
                    to avoid the risk of I2C interrupt handle execution before current
                    process unlock */
            /* Enable TX_ABORT interrupts */
            ll_i2c_enable_it(p_i2c->p_instance, I2C_XFER_ERROR_IT);
        }
        else
        {
#if defined(HAL_STATUS_CHECK)
            p_i2c->error_code |= HAL_I2C_ERROR_DMA;

            /* Update I2C state */
            i2c_set_device_state(p_i2c, HAL_I2C_STATE_READY);
#endif
        }
    }
    else
    {
        i2c_dma_master_transmit_cplt(p_i2c->p_dmatx);
    }

    return status;
}

static hal_status_t i2c_master_start_receive_dma(i2c_handle_t *p_i2c)
{
    hal_status_t status = HAL_OK;
    hal_status_t status1 = HAL_OK;
    if (1U < p_i2c->xfer_size)
    {
        // lint -e551 rxcmd is used
        // Note:The rxcmd must be declared in RAM because it is used as the starting address for DMA transfer.
        //      If it is located in flash memory and the xQSPI is in non-XIP mode, it may lead to abnormal DMA data transfer.
        static uint32_t rxcmd = LL_I2C_CMD_MST_READ;

        /* Set the I2C DMA transfer complete callback */
        p_i2c->p_dmarx->xfer_tfr_callback     = i2c_dma_master_receive_cplt;
        /* Set the DMA error callback */
        p_i2c->p_dmarx->xfer_error_callback   = i2c_dma_error;
        /* Set the unused DMA callbacks to NULL */
        p_i2c->p_dmarx->xfer_abort_callback   = NULL;
        p_i2c->p_dmatx->xfer_tfr_callback     = NULL;
        p_i2c->p_dmatx->xfer_error_callback   = NULL;
        p_i2c->p_dmatx->xfer_abort_callback   = NULL;

        /* Set DMA transfer data level */
        ll_i2c_set_dma_rx_data_level(p_i2c->p_instance, 3U);

        ll_dma_set_source_burst_length(p_i2c->p_dmarx->p_instance, p_i2c->p_dmarx->channel, LL_DMA_SRC_BURST_LENGTH_4);

        /* Configure Tx channel to generate ACK during master receiving */
        /* During master receiving, Tx channel will be used to generate ACK. */
        ll_i2c_set_dma_tx_data_level(p_i2c->p_instance, 4U);

        ll_dma_set_destination_burst_length(p_i2c->p_dmatx->p_instance, p_i2c->p_dmatx->channel, LL_DMA_DST_BURST_LENGTH_4);
        /* Config the increase mode and  to NO_CHANGE to write the MST_READ command into TX FIFO */
        ll_dma_set_source_increment_mode(p_i2c->p_dmatx->p_instance, p_i2c->p_dmatx->channel, LL_DMA_SRC_NO_CHANGE);
        ll_dma_set_source_width(p_i2c->p_dmatx->p_instance, p_i2c->p_dmatx->channel, LL_DMA_SDATAALIGN_HALFWORD);
        ll_dma_set_destination_width(p_i2c->p_dmatx->p_instance, p_i2c->p_dmatx->channel, LL_DMA_DDATAALIGN_HALFWORD);

        /* Enable the DMA channel */
        status = hal_dma_start_it(p_i2c->p_dmatx, (uint32_t)&rxcmd, ll_i2c_dma_get_register_address(p_i2c->p_instance), p_i2c->xfer_size - 1U);
        status1 = hal_dma_start_it(p_i2c->p_dmarx, ll_i2c_dma_get_register_address(p_i2c->p_instance), (uint32_t)(p_i2c->p_buffer), p_i2c->xfer_size - 1U);

        if ((HAL_OK == status) && (HAL_OK == status1))
        {
            /* Update Buffer & XferCount value */
            p_i2c->p_buffer = &p_i2c->p_buffer[p_i2c->xfer_size - 1U];
            p_i2c->xfer_count = 1;

            /* Enable DMA Request */
            ll_i2c_enable_dma_req_tx(p_i2c->p_instance);
            ll_i2c_enable_dma_req_rx(p_i2c->p_instance);

            /* Note : The I2C interrupts must be enabled after unlocking current process
                        to avoid the risk of I2C interrupt handle execution before current
                        process unlock */
            /* Enable TX_ABORT interrupts */
            ll_i2c_enable_it(p_i2c->p_instance, I2C_XFER_ERROR_IT | LL_I2C_INTR_MASK_RX_OVER);
        }
        else
        {
#if defined(HAL_STATUS_CHECK)
            p_i2c->error_code |= HAL_I2C_ERROR_DMA;

            /* Update I2C state */
            i2c_set_device_state(p_i2c, HAL_I2C_STATE_READY);
#endif

        }
    }
    else
    {
        i2c_dma_master_receive_cplt(p_i2c->p_dmarx);
    }

    return status;
}

static hal_status_t i2c_slave_start_transmit(i2c_handle_t *p_i2c, uint32_t timeout)
{
    if((HAL_NEVER_TIMEOUT != timeout) && (HAL_MAX_DELAY < timeout))
    {
        return HAL_ERROR;
    }

    HAL_TIMEOUT_INIT();
    uint32_t tickstart = HAL_TIMEOUT_GET_TICK();
    /* Update SystemCoreClock */
    SystemCoreUpdateClock();
    uint32_t timeout_counter = ((SystemCoreClock / 1000U) * timeout);

    while (0U < p_i2c->xfer_count)
    {
        if(p_i2c->p_instance->RAW_INT_STAT & I2C_RAW_INT_STAT_STOP_DET)
        {
            break;
        }

        if(p_i2c->p_instance->RAW_INT_STAT & I2C_RAW_INT_STAT_RD_REQ)
        {
            /* Clear the read req flag */
            ll_i2c_clear_flag_read_req(p_i2c->p_instance);

            ll_i2c_transmit_data8(p_i2c->p_instance, *p_i2c->p_buffer++, LL_I2C_CMD_SLV_NONE);
            p_i2c->xfer_count--;
        }

        if(HAL_NEVER_TIMEOUT != timeout)
        {
            if((HAL_TIMEOUT_GET_TICK() - tickstart) > timeout_counter)
            {
                /* Check what kind of error */
                i2c_slave_check_error(p_i2c);
                HAL_TIMEOUT_DEINIT();
                if (p_i2c->error_code & HAL_I2C_ERROR_ARB_LOST)
                {
                    return HAL_ERROR;
                }
                return HAL_TIMEOUT;
            }
        }
    }

    HAL_TIMEOUT_DEINIT();

    /* Wait until STOP_DET flag is set */
    if (HAL_OK != i2c_wait_on_raw_flag_until_timeout(p_i2c, I2C_RAW_INT_STAT_STOP_DET, 0, timeout))
    {
        return HAL_TIMEOUT;
    }

    /* Clear STOP_DET Flag */
    ll_i2c_clear_flag_stop_det(p_i2c->p_instance);

    return HAL_OK;
}

static hal_status_t i2c_slave_start_receive(i2c_handle_t *p_i2c, uint32_t timeout)
{
    if((HAL_NEVER_TIMEOUT != timeout) && (HAL_MAX_DELAY < timeout))
    {
        return HAL_ERROR;
    }

    HAL_TIMEOUT_INIT();
    uint32_t tickstart = HAL_TIMEOUT_GET_TICK();
    /* Update SystemCoreClock */
    SystemCoreUpdateClock();
    uint32_t timeout_counter = ((SystemCoreClock / 1000U) * timeout);

    while (0U < p_i2c->xfer_count)
    {
        if(p_i2c->p_instance->RAW_INT_STAT & I2C_RAW_INT_STAT_STOP_DET)
        {
            break;
        }

        if(p_i2c->p_instance->STAT & I2C_STAT_RX_FIFO_NE)
        {
            *p_i2c->p_buffer++ = ll_i2c_receive_data8(p_i2c->p_instance);
            p_i2c->xfer_count--;

        }

        if(HAL_NEVER_TIMEOUT != timeout)
        {
            if((HAL_TIMEOUT_GET_TICK() - tickstart) > timeout_counter)
            {
                /* Check what kind of error */
                i2c_slave_check_error(p_i2c);
                HAL_TIMEOUT_DEINIT();
                if (p_i2c->error_code & (HAL_I2C_ERROR_ARB_LOST | HAL_I2C_ERROR_OVER))
                {
                    return HAL_ERROR;
                }
                return HAL_TIMEOUT;
            }
        }
    }

    HAL_TIMEOUT_DEINIT();

    /* Wait until STOP_DET flag is set */
    if (HAL_OK != i2c_wait_on_raw_flag_until_timeout(p_i2c, I2C_RAW_INT_STAT_STOP_DET, 0, timeout))
    {
        return HAL_TIMEOUT;
    }

    /* Clear STOP_DET Flag */
    ll_i2c_clear_flag_stop_det(p_i2c->p_instance);

    return HAL_OK;
}

/**
  * @brief  Master sends target device address followed by internal memory address for write request.
  * @param  p_i2c Pointer to a i2c_handle_t structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @retval HAL status
  */
static void i2c_req_mem_read_write(i2c_handle_t *p_i2c, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size)
{
    UNUSED(dev_address);
    uint32_t cmd = LL_I2C_CMD_MST_WRITE;

    /* If Memory address size is 16Bit, send MSB of Memory Address first */
    if (I2C_MEMADD_SIZE_16BIT == mem_addr_size)
    {
        ll_i2c_transmit_data8(p_i2c->p_instance, I2C_MEM_ADD_MSB(mem_address), cmd);
    }

    /* Send LSB of Memory Address */
    ll_i2c_transmit_data8(p_i2c->p_instance, I2C_MEM_ADD_LSB(mem_address), cmd);
}

/**
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Master Mode with Interrupt.
  * @param  p_i2c Pointer to a i2c_handle_t structure that contains
  *                the configuration information for the specified I2C.
  * @param  ITSources Interrupt sources triggered.
  * @param  AbortSources Sources of TX_ABORT interrupt.
  * @retval HAL status
  */
static void i2c_master_isr_it(i2c_handle_t *p_i2c, uint32_t it_source, uint32_t abort_sources)
{
    uint32_t curxfercnt = 0U;
    uint32_t cmd = LL_I2C_CMD_MST_WRITE;

    if ((it_source & LL_I2C_INTR_STAT_TX_ABRT))
    {
        if ((abort_sources & I2C_TX_ABRT_NOACK))
        {
            /* Set corresponding Error Code */
            p_i2c->error_code |= HAL_I2C_ERROR_NOACK;
        }
    }
    else if ((it_source & LL_I2C_INTR_STAT_RX_FULL))
    {
        /* Get data count in RX FIFO */
        curxfercnt = ll_i2c_get_rx_fifo_level(p_i2c->p_instance);

        /* Read data from RX FIFO */
        while ((0U != (curxfercnt--)))
        {
            if(0U == p_i2c->xfer_count)
            {
                break;
            }
            (*p_i2c->p_buffer++) = ll_i2c_receive_data8(p_i2c->p_instance);
            p_i2c->xfer_count--;
        }

        if (p_i2c->xfer_count < (ll_i2c_get_rx_fifo_threshold(p_i2c->p_instance) + 1U))
        {
            ll_i2c_set_rx_fifo_threshold(p_i2c->p_instance, LL_I2C_RX_FIFO_TH_CHAR_1);
        }

        if (0U == p_i2c->xfer_count)
        {
            ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_RX_FULL);
        }
    }
    else if ((it_source & LL_I2C_INTR_STAT_TX_EMPTY))
    {
        /* Get free data count in TX FIFO */
        curxfercnt = I2C_TXFIFO_SIZE - ll_i2c_get_tx_fifo_level(p_i2c->p_instance);

        /* Master transmit process */
        if (HAL_I2C_STATE_BUSY_TX == p_i2c->state)
        {
            /* Write data into TX FIFO */
            while ((0U != (curxfercnt--)))
            {
                if(0U == p_i2c->xfer_count)
                {
                    break;
                }
                if (1U == p_i2c->xfer_count)
                {
                    cmd |= LL_I2C_CMD_MST_GEN_STOP;
                }

                ll_i2c_transmit_data8(p_i2c->p_instance, *p_i2c->p_buffer++, cmd);
                p_i2c->xfer_count--;
            }

            if (0U == p_i2c->xfer_count)
            {
                ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_TX_EMPTY);
            }
        }
        /* Master receive process */
        else if (HAL_I2C_STATE_BUSY_RX == p_i2c->state)
        {
            cmd = LL_I2C_CMD_MST_READ;

            /* Write LL_I2C_CMD_MST_READ into TX FIFO to generate ACK */
            while ((curxfercnt--))
            {
                if((0U == p_i2c->master_ack_count))
                {
                    break;
                }
                if (1U == p_i2c->master_ack_count)
                {
                    cmd |= LL_I2C_CMD_MST_GEN_STOP;
                }

                ll_i2c_transmit_data8(p_i2c->p_instance, 0U, cmd);
                p_i2c->master_ack_count--;
            }

            if (0U == p_i2c->master_ack_count)
            {
                ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_TX_EMPTY);
            }
        }
        else
        {
            /* DO nothing */
        }
    }
    else
    {
        /* DO nothing */
    }

    if ((it_source & LL_I2C_INTR_STAT_STOP_DET))
    {
        /* Call I2C Master complete process */
        i2c_it_master_cplt(p_i2c);
    }
}

/**
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Slave Mode with Interrupt.
  * @param  p_i2c    Pointer to a i2c_handle_t structure that contains
  *                 the configuration information for the specified I2C.
  * @param  it_source       Interrupt sources to handle.
  * @param  abort_sources   Abort sources.
  * @retval HAL status
  */
static void i2c_slave_isr_it(i2c_handle_t *p_i2c, uint32_t it_source, uint32_t abort_sources)
{
    uint32_t curxfercnt = 0U;

    if ((it_source & LL_I2C_INTR_STAT_TX_ABRT))
    {
        if ((abort_sources & I2C_TX_ABRT_NOACK))
        {
        }
    }
    else if ((it_source & LL_I2C_INTR_STAT_RD_REQ))
    {
        ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_RD_REQ);
        /* Start transmit */
        ll_i2c_enable_it(p_i2c->p_instance, I2C_SLV_XFER_TX_IT);
    }
    else if ((it_source & LL_I2C_INTR_STAT_RX_FULL))
    {
        /* Get data count in RX FIFO */
        curxfercnt = ll_i2c_get_rx_fifo_level(p_i2c->p_instance);

        /* Read data from RX FIFO */
        while ((curxfercnt--))
        {
            if(0U == p_i2c->xfer_count)
            {
                break;
            }
            (*p_i2c->p_buffer++) = ll_i2c_receive_data8(p_i2c->p_instance);
            p_i2c->xfer_count--;
        }

        if (p_i2c->xfer_count < (ll_i2c_get_rx_fifo_threshold(p_i2c->p_instance) + 1U))
        {
            ll_i2c_set_rx_fifo_threshold(p_i2c->p_instance, LL_I2C_RX_FIFO_TH_CHAR_1);
        }

        if (0U == p_i2c->xfer_count)
        {
            ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_RX_FULL);
        }
    }
    else if ((it_source & LL_I2C_INTR_STAT_TX_EMPTY))
    {
        /* Get free data count in TX FIFO */
        curxfercnt = I2C_TXFIFO_SIZE - ll_i2c_get_tx_fifo_level(p_i2c->p_instance);

        /* Write data into TX FIFO */
        while ((curxfercnt--))
        {
            if(0U == p_i2c->xfer_count)
            {
                break;
            }
            ll_i2c_transmit_data8(p_i2c->p_instance, *p_i2c->p_buffer++, LL_I2C_CMD_SLV_NONE);
            p_i2c->xfer_count--;
        }

        if (0U == p_i2c->xfer_count)
        {
            ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_TX_EMPTY);
        }
    }
    else
    {
        /* DO nothing */
    }

    /* Check if STOP_DET is set */
    if ((it_source & LL_I2C_INTR_STAT_STOP_DET))
    {
        /* Call I2C Slave complete process */
        i2c_it_slave_cplt(p_i2c);
    }
}

/**
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Master Mode with DMA.
  * @param  p_i2c Pointer to a i2c_handle_t structure that contains
  *                the configuration information for the specified I2C.
  * @param  it_source       Interrupt sources to handle.
  * @param  abort_sources   Abort sources.
  * @retval HAL status
  */
static void i2c_master_isr_dma(i2c_handle_t*p_i2c, uint32_t it_source, uint32_t abort_sources)
{
    if ((it_source & LL_I2C_INTR_STAT_TX_ABRT))
    {
        if ((abort_sources & I2C_TX_ABRT_NOACK))
        {
            /* Set corresponding Error Code */
            p_i2c->error_code |= HAL_I2C_ERROR_NOACK;
        }

        /* Enable STOP interrupt, to treat it */
        /* Error callback will be send during stop flag treatment */
        ll_i2c_enable_it(p_i2c->p_instance, I2C_XFER_CPLT_IT);
    }

    if ((it_source & LL_I2C_INTR_STAT_STOP_DET))
    {
        /* Call I2C Master complete process */
        i2c_it_master_cplt(p_i2c);
    }
}

/**
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Slave Mode with DMA.
  * @param  p_i2c Pointer to a i2c_handle_t structure that contains
  *                the configuration information for the specified I2C.
  * @param  it_source       Interrupt sources to handle.
  * @param  abort_sources   Abort sources.
  * @retval HAL status
  */
static void i2c_slave_isr_dma(i2c_handle_t* p_i2c, uint32_t it_source, uint32_t abort_sources)
{
    if ((it_source & LL_I2C_INTR_STAT_TX_ABRT))
    {
        if ((0U != (abort_sources & I2C_TX_ABRT_NOACK)))
        {
            if(0U != I2C_GET_DMA_REMAIN_DATA(p_i2c))/*lint !e666 MISRA exception. Allow repeated parameter in macro */
            {
                /* Set corresponding Error Code */
                p_i2c->error_code |= HAL_I2C_ERROR_NOACK;
            }
        }
    }
    else if ((it_source & LL_I2C_INTR_STAT_RD_REQ))
    {
        ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_RD_REQ);
        /* Start transmit */
        ll_i2c_enable_dma_req_tx(p_i2c->p_instance);
    }
    else
    {
        /* DO nothing */
    }

    /* Check if STOP_DET is set */
    if ((it_source & LL_I2C_INTR_STAT_STOP_DET))
    {
        /* Call I2C Slave complete process */
        i2c_it_slave_cplt(p_i2c);
    }
}

/**
  * @brief  I2C Master complete process.
  * @param  p_i2c I2C handle.
  * @retval None
  */
static void i2c_it_master_cplt(i2c_handle_t *p_i2c)
{
    /* Clear STOP_DET Flag */
    ll_i2c_clear_flag_stop_det(p_i2c->p_instance);

    /* Reset handle parameters */
    p_i2c->xfer_isr       = NULL;
    p_i2c->xfer_options   = I2C_NO_OPTION_FRAME;

    p_i2c->previous_state = I2C_STATE_NONE;

    if (ll_i2c_is_active_flag_raw_tx_abort(p_i2c->p_instance))
    {
        if(ll_i2c_get_abort_source(p_i2c->p_instance) & I2C_TX_ABRT_NOACK)
        {
            /* Clear TX_ABORT Flag */
            ll_i2c_clear_flag_tx_abort(p_i2c->p_instance);

            /* Set acknowledge error code */
            p_i2c->error_code |= HAL_I2C_ERROR_NOACK;
        }
    }

    /* Disable Interrupts */
    ll_i2c_disable_it(p_i2c->p_instance, I2C_MST_XFER_TX_IT | I2C_MST_XFER_RX_IT);

    if (HAL_I2C_STATE_BUSY_TX == p_i2c->state)
    {
        /* Data remained in TX FIFO were sent failed, FIFO will be flushed after I2C was disabled. */
        p_i2c->xfer_count += ll_i2c_get_tx_fifo_level(p_i2c->p_instance);
    }
    else
    {
        /* Store Last receive data if any */
        while (ll_i2c_is_active_flag_status_rfne(p_i2c->p_instance))
        {
            if(0U == p_i2c->xfer_count)
            {
                break;
            }
            (*p_i2c->p_buffer++) = ll_i2c_receive_data8(p_i2c->p_instance);
            p_i2c->xfer_count--;
        }
    }

    /* Call the corresponding callback to inform upper layer of End of Transfer */
    uint32_t temp_error_code = p_i2c->error_code;
    hal_i2c_state_t temp_state = p_i2c->state;
    if ( (HAL_I2C_ERROR_NONE != temp_error_code) || (HAL_I2C_STATE_ABORT == temp_state))
    {
        /* Call the corresponding callback to inform upper layer of End of Transfer */
        i2c_it_error(p_i2c, p_i2c->error_code);
    }
    /* p_i2c->State == HAL_I2C_STATE_BUSY_TX */
    else if (HAL_I2C_STATE_BUSY_TX == p_i2c->state)
    {
#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_READY);
#endif
        p_i2c->mode = HAL_I2C_MODE_NONE;

        /* Call the corresponding callback to inform upper layer of End of Transfer */
        hal_i2c_master_tx_cplt_callback(p_i2c);
    }
    /* p_i2c->State == HAL_I2C_STATE_BUSY_RX */
    else if (HAL_I2C_STATE_BUSY_RX == p_i2c->state)
    {
#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_READY);
#endif
        p_i2c->mode = HAL_I2C_MODE_NONE;

        hal_i2c_master_rx_cplt_callback(p_i2c);
    }
    else
    {
        /* DO nothing */
    }

    /* Disable the selected I2C peripheral */
    ll_i2c_disable(p_i2c->p_instance);
}

/**
  * @brief  I2C Slave complete process.
  * @param  p_i2c I2C handle.
  * @retval None
  */
static void i2c_it_slave_cplt(i2c_handle_t *p_i2c)
{
    /* Clear STOP_DET Flag */
    ll_i2c_clear_flag_stop_det(p_i2c->p_instance);

    /* Disable all interrupts */
    ll_i2c_disable_it(p_i2c->p_instance, I2C_SLV_XFER_TX_IT | I2C_SLV_XFER_RX_IT | I2C_XFER_LISTEN_IT);

    /* If a DMA is ongoing, Update handle size context */
    bool is_enabled_dma_req_tx = (bool)ll_i2c_is_enabled_dma_req_tx(p_i2c->p_instance);
    bool is_enabled_dma_req_rx = (bool)ll_i2c_is_enabled_dma_req_rx(p_i2c->p_instance);
    if (is_enabled_dma_req_tx || is_enabled_dma_req_rx)
    {
        p_i2c->xfer_count = (uint16_t)I2C_GET_DMA_REMAIN_DATA(p_i2c);/*lint !e666 MISRA exception. Allow repeated parameter in macro */
    }

    if (HAL_I2C_STATE_BUSY_TX == p_i2c->state)
    {
        /* Data remained in TX FIFO were sent failed, FIFO will be flushed after I2C was disabled. */
        p_i2c->xfer_count += ll_i2c_get_tx_fifo_level(p_i2c->p_instance);
    }
    else
    {
        /* Store Last receive data if any */
        while (0U != ll_i2c_is_active_flag_status_rfne(p_i2c->p_instance))
        {
            if((0U == p_i2c->xfer_count))
            {
                break;
            }
            if(0U != ll_i2c_is_enabled_dma_req_rx(p_i2c->p_instance))
            {
                (*(p_i2c->p_buffer + (p_i2c->xfer_size - p_i2c->xfer_count))) = ll_i2c_receive_data8(p_i2c->p_instance);
            }
            else
            {
                (*p_i2c->p_buffer++) = ll_i2c_receive_data8(p_i2c->p_instance);
            }
            p_i2c->xfer_count--;
        }
    }

    /* Abort DMA RX transfer if any */
    if ((0U != ll_i2c_is_enabled_dma_req_rx(p_i2c->p_instance)) && (NULL != p_i2c->p_dmarx))
    {
        if (HAL_OK == hal_dma_abort_it(p_i2c->p_dmarx))
        {
            ll_i2c_disable_dma_req_rx(p_i2c->p_instance);
        }
    }
    /* Abort DMA TX transfer if any */
    else if ((0U != ll_i2c_is_enabled_dma_req_tx(p_i2c->p_instance)) && (NULL != p_i2c->p_dmatx))
    {
        if (HAL_OK == hal_dma_abort_it(p_i2c->p_dmatx))
        {
            ll_i2c_disable_dma_req_tx(p_i2c->p_instance);
        }
    }
    else
    {
        /* Do nothing */
    }

    p_i2c->previous_state    = I2C_STATE_NONE;
    p_i2c->mode              = HAL_I2C_MODE_NONE;
    p_i2c->xfer_isr          = NULL;

    if (HAL_I2C_ERROR_NONE != p_i2c->error_code)
    {
        /* Call the corresponding callback to inform upper layer of End of Transfer */
        i2c_it_error(p_i2c, p_i2c->error_code);
    }
    /* Call the corresponding callback to inform upper layer of End of Transfer */
    else if (HAL_I2C_STATE_BUSY_RX == p_i2c->state)
    {
#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_READY);
#endif

        /* Call the Slave Rx Complete callback */
        hal_i2c_slave_rx_cplt_callback(p_i2c);
    }
    else
    {
#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_READY);
#endif

        /* Call the Slave Tx Complete callback */
        hal_i2c_slave_tx_cplt_callback(p_i2c);
    }

    /* Disable the selected I2C peripheral */
    ll_i2c_disable(p_i2c->p_instance);
}

/**
  * @brief  I2C interrupts error process.
  * @param  p_i2c I2C handle.
  * @param  error_code Error code to handle.
  * @retval None
  */
static void i2c_it_error(i2c_handle_t *p_i2c, uint32_t error_code)
{
    bool tx_dma_is_null = false;
    if((NULL == p_i2c->p_dmatx))
    {
        tx_dma_is_null = true;
    }
    /* Reset handle parameters */
    p_i2c->xfer_options  = I2C_NO_OPTION_FRAME;
    // p_i2c->XferCount     = 0U;

    /* Set new error code */
    p_i2c->error_code |= error_code;

    /* Disable all interrupts */
    ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_ALL);

    /* If state is an abort treatment on goind, don't change state */
    /* This change will be do later */
    if (HAL_I2C_STATE_ABORT != p_i2c->state)
    {
#if defined(HAL_STATUS_CHECK)
        /* Set HAL_I2C_STATE_READY */
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_READY);
        p_i2c->previous_state = I2C_STATE_NONE;
#endif
    }
    p_i2c->xfer_isr = NULL;

    /* Abort DMA RX transfer if any */
    if ((ll_i2c_is_enabled_dma_req_rx(p_i2c->p_instance)))
    {
        if(NULL != p_i2c->p_dmarx)
        {
            /* Set the I2C DMA Abort callback will lead to call HAL_I2C_ErrorCallback() at end of DMA abort procedure */
            p_i2c->p_dmarx->xfer_abort_callback = i2c_dma_abort;
            /* Master receive need to abort Tx too. */

            /* Abort DMA TX and Rx */
            if ((HAL_I2C_MODE_MASTER == p_i2c->mode) && (!tx_dma_is_null))
            {
                p_i2c->p_dmatx->xfer_abort_callback = NULL;
                if(HAL_OK == hal_dma_abort_it(p_i2c->p_dmatx))
                {
                    ll_i2c_disable_dma_req_tx(p_i2c->p_instance);
                }
            }

            if (HAL_OK != hal_dma_abort_it(p_i2c->p_dmarx))
            {
                ll_i2c_disable_dma_req_rx(p_i2c->p_instance);

                /* Call Directly p_i2c->p_dmarx->XferAbortCallback function in case of error */
                p_i2c->p_dmarx->xfer_abort_callback(p_i2c->p_dmarx);
            }
        }
    }
    /* Abort DMA TX transfer if any */
    else if ((0U != ll_i2c_is_enabled_dma_req_tx(p_i2c->p_instance)) && (!tx_dma_is_null))
    {
        /* Set the I2C DMA Abort callback :
        will lead to call HAL_I2C_ErrorCallback() at end of DMA abort procedure */
        p_i2c->p_dmatx->xfer_abort_callback = i2c_dma_abort;

        /* Abort DMA TX */
        if (HAL_OK != hal_dma_abort_it(p_i2c->p_dmatx))
        {
            ll_i2c_disable_dma_req_tx(p_i2c->p_instance);

            /* Call Directly XferAbortCallback function in case of error */
            p_i2c->p_dmatx->xfer_abort_callback(p_i2c->p_dmatx);
        }
    }
    else if (HAL_I2C_STATE_ABORT == p_i2c->state)
    {
#if defined(HAL_STATUS_CHECK)
        p_i2c->previous_state = I2C_STATE_NONE;
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_READY);
#endif
        /* Store Last receive data if any */
        while (ll_i2c_is_active_flag_status_rfne(p_i2c->p_instance))
        {
            if((0U == p_i2c->xfer_count))
            {
                break;
            }
            (*p_i2c->p_buffer++) = ll_i2c_receive_data8(p_i2c->p_instance);
            p_i2c->xfer_count--;
        }

        /* Call the corresponding callback to inform upper layer of End of Transfer */
        hal_i2c_abort_cplt_callback(p_i2c);
    }
    else
    {
        /* Call the corresponding callback to inform upper layer of End of Transfer */
        hal_i2c_error_callback(p_i2c);
    }

    if(p_i2c->error_code & HAL_I2C_ERROR_OVER)
    {
        ll_i2c_enable_transfer_abort(p_i2c->p_instance);
        while(READ_REG(p_i2c->p_instance->STAT)&I2C_STAT_ACTIVITY){}
    }
    /* Disable the selected I2C peripheral */
    ll_i2c_disable(p_i2c->p_instance);
}

/**
  * @brief  DMA I2C master transmit process complete callback.
  * @param  p_dma DMA handle
  * @retval None
  */
static void i2c_dma_master_transmit_cplt(dma_handle_t *p_dma)
{
    //lint -e9087 Cast a pointer to a different object type is necessary
    i2c_handle_t *p_i2c = (i2c_handle_t *)(p_dma->p_parent);

    /* Disable DMA Request */
    ll_i2c_disable_dma_req_tx(p_i2c->p_instance);

    /* Transmit the last data with STOP signal */
    if (1U == p_i2c->xfer_count)
    {
        if (HAL_OK != i2c_wait_on_sta_flag_until_timeout(p_i2c, I2C_STAT_TX_FIFO_NF, 0, I2C_TIMEOUT_BUSY))
        {
            p_i2c->error_code = HAL_I2C_ERROR_TIMEOUT;
        }
        ll_i2c_transmit_data8(p_i2c->p_instance, *p_i2c->p_buffer, LL_I2C_CMD_MST_WRITE | LL_I2C_CMD_MST_GEN_STOP);
        p_i2c->xfer_count--;
    }

    /* If last transfer, enable STOP interrupt */
    /* Enable STOP interrupt */
    ll_i2c_enable_it(p_i2c->p_instance, I2C_XFER_CPLT_IT);
}

/**
  * @brief  DMA I2C slave transmit process complete callback.
  * @param  p_dma DMA handle
  * @retval None
  */
static void i2c_dma_slave_transmit_cplt(dma_handle_t *p_dma)
{
    //lint -e9087 Cast a pointer to a different object type is necessary
    i2c_handle_t *p_i2c = (i2c_handle_t *)(p_dma->p_parent);

    /* Disable DMA Request */
    ll_i2c_disable_dma_req_tx(p_i2c->p_instance);

    /* No specific action, Master fully manage the generation of STOP condition */
    /* Mean that this generation can arrive at any time, at the end or during DMA process */
    /* So STOP condition should be manage through Interrupt treatment */
}

/**
  * @brief DMA I2C master receive process complete callback.
  * @param  p_dma DMA handle
  * @retval None
  */
static void i2c_dma_master_receive_cplt(dma_handle_t *p_dma)
{
    //lint -e9087 Cast a pointer to a different object type is necessary
    i2c_handle_t *p_i2c = (i2c_handle_t *)(p_dma->p_parent);

    /* Disable DMA Request */
    ll_i2c_disable_dma_req_tx(p_i2c->p_instance);
    ll_i2c_disable_dma_req_rx(p_i2c->p_instance);

    /* Current p_i2c->xfer_count should be equal to 1 */
    /* Receive the last data with STOP signal */
    if (1U == p_i2c->xfer_count)
    {
        if (HAL_OK != i2c_wait_on_sta_flag_until_timeout(p_i2c, I2C_STAT_TX_FIFO_NF, 0, I2C_TIMEOUT_BUSY))
        {
            p_i2c->error_code = HAL_I2C_ERROR_TIMEOUT;
        }
        ll_i2c_transmit_data8(p_i2c->p_instance, 0, LL_I2C_CMD_MST_READ | LL_I2C_CMD_MST_GEN_STOP);
        if (HAL_OK != i2c_wait_on_sta_flag_until_timeout(p_i2c, I2C_STAT_RX_FIFO_NE, 0, I2C_TIMEOUT_BUSY))
        {
            p_i2c->error_code = HAL_I2C_ERROR_TIMEOUT;
        }
        *p_i2c->p_buffer++ = ll_i2c_receive_data8(p_i2c->p_instance);
        p_i2c->xfer_count--;
    }

    /* If last transfer, enable STOP interrupt */
    /* Enable STOP interrupt */
    ll_i2c_enable_it(p_i2c->p_instance, I2C_XFER_CPLT_IT);
}

/**
  * @brief  DMA I2C slave receive process complete callback.
  * @param  p_dma DMA handle
  * @retval None
  */
static void i2c_dma_slave_receive_cplt(dma_handle_t *p_dma)
{
    //lint -e9087 Cast a pointer to a different object type is necessary
    i2c_handle_t *p_i2c = (i2c_handle_t *)(p_dma->p_parent);

    /* Disable DMA Request */
    ll_i2c_disable_dma_req_rx(p_i2c->p_instance);

    /* No specific action, Master fully manage the generation of STOP condition */
    /* Mean that this generation can arrive at any time, at the end or during DMA process */
    /* So STOP condition should be manage through Interrupt treatment */
}

/**
  * @brief  DMA I2C communication error callback.
  * @param p_dma DMA handle
  * @retval None
  */
static void i2c_dma_error(dma_handle_t *p_dma)
{
    //lint -e9087 Cast a pointer to a different object type is necessary
    i2c_handle_t *p_i2c = (i2c_handle_t *)(p_dma->p_parent);

    /* Call the corresponding callback to inform upper layer of End of Transfer */
    i2c_it_error(p_i2c, HAL_I2C_ERROR_DMA);
}

/**
  * @brief DMA I2C communication abort callback
  *        (To be called at end of DMA Abort procedure).
  * @param p_dma DMA handle.
  * @retval None
  */
static void i2c_dma_abort(dma_handle_t *p_dma)
{
    //lint -e9087 Cast a pointer to a different object type is necessary
    i2c_handle_t *p_i2c = (i2c_handle_t *)(p_dma->p_parent);

    /* Disable DMA Request */
    ll_i2c_disable_dma_req_tx(p_i2c->p_instance);
    ll_i2c_disable_dma_req_rx(p_i2c->p_instance);

    if (p_i2c->p_dmatx->channel == p_dma->channel)
    {
        p_i2c->xfer_count = (uint16_t)I2C_GET_TX_DMA_REMAIN_DATA(p_i2c);/*lint !e666 MISRA exception. Allow repeated parameter in macro */
    }
    else
    {
        p_i2c->xfer_count = (uint16_t)I2C_GET_RX_DMA_REMAIN_DATA(p_i2c);/*lint !e666 MISRA exception. Allow repeated parameter in macro */
    }

    /* Reset AbortCpltCallback */
    p_i2c->p_dmatx->xfer_abort_callback = NULL;
    p_i2c->p_dmarx->xfer_abort_callback = NULL;

    /* Check if come from abort from user */
    if (HAL_I2C_STATE_ABORT == p_i2c->state)
    {
#if defined(HAL_STATUS_CHECK)
        i2c_set_device_state(p_i2c, HAL_I2C_STATE_READY);
#endif
        /* Call the corresponding callback to inform upper layer of End of Transfer */
        hal_i2c_abort_cplt_callback(p_i2c);
    }
    else
    {
        /* Call the corresponding callback to inform upper layer of End of Transfer */
        hal_i2c_error_callback(p_i2c);
    }
}

static void i2c_set_device_state(i2c_handle_t *p_i2c, hal_i2c_state_t state)
{
    p_i2c->state = state;
}

/**
 ****************************************************************************************
 SCL_High_time = [(HCNT + IC_*_SPKLEN + 7) * ic_clk] + SCL_Fall_time = 1 / (2 * speed)
 SCL_Low_time = [(LCNT + 1) * ic_clk] - SCL_Fall_time + SCL_Rise_time = 1 / (2 * speed)
 ****************************************************************************************
 */
__WEAK hal_status_t hal_i2c_speed_config(i2c_handle_t *p_i2c, uint32_t speed, uint32_t scl_fall_time, uint32_t scl_rise_time)
{
    uint32_t hcnt, lcnt, spklen;

    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
        ll_i2c_disable(p_i2c->p_instance);
        ll_i2c_set_speed_mode(p_i2c->p_instance, __LL_I2C_CONVERT_SPEED_MODE(speed));
        lcnt = ((GetSerialClock() / 1000U / 1000U ) * (((1000U * 1000U * 1000U) / (speed * 2U)) - scl_rise_time + scl_fall_time) / 1000U) - 1U;
        spklen = ll_i2c_get_spike_len_fs(p_i2c->p_instance);
        hcnt = ((GetSerialClock() / 1000U / 1000U) * (((1000U * 1000U * 1000U) / (speed * 2U)) - scl_fall_time) / 1000U) - (7U + spklen);
        if (speed < I2C_SPEED_400K)
        {
            ll_i2c_set_clock_high_period_ss(p_i2c->p_instance, hcnt);
            ll_i2c_set_clock_low_period_ss(p_i2c->p_instance, lcnt);
        }
        else
        {
            ll_i2c_set_clock_high_period_fs(p_i2c->p_instance, hcnt);
            ll_i2c_set_clock_low_period_fs(p_i2c->p_instance, lcnt);
        }
        ll_i2c_enable(p_i2c->p_instance);
    }
    else
    {
        return HAL_BUSY;
    }
    return HAL_OK;
}

/** @} */

#endif /* HAL_I2C_MODULE_ENABLED */

/** @} */

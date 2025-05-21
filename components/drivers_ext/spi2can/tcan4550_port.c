/**
 ****************************************************************************************
 *
 * @file    tcan4550_port.c
 * @author  BSP Team
 * @brief   PORT interface based on the TCAN4550 low-level driver encapsulation.
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2025 GOODIX
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
#include <stdio.h>
#include <string.h>
#include "tcan4550_port.h"
#include "gr_common.h"
#include "gr_dwt.h"
#include "hal_def.h"
#include "hal_delay.h"
#include "app_assert.h"
#include "app_log.h"
#include "TCAN4550.h"

#ifdef TCAN4550_H_

/*
 * DEFINES
 *****************************************************************************************
 */
#define AHB_WRITE_OPCODE                    (0x61)
#define AHB_READ_OPCODE                     (0x41)

#define CANFD_READ_WORDS_MAX                (128U)

#define TCAN_RX_FIFO_ELEM_CNT               (20U)
#define TCAN_HANDLE_BUSOFF_RECOVERY_TIMEOUT (100U)     //ms

typedef struct
{
    tcan4550_mode_t can_mode;
    tcan4550_brs_mode_t brs_mode;
    tcan4550_retrans_mode_t retrans_mode;
    bool is_irq_transmit_pending;
} tcan4550_env_t;

typedef struct
{
    TCAN4x5x_MCAN_RX_Header header;
    uint8_t dat[64];
} tcan4550_rx_msg;

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static tcan4550_debug_log_lv_t s_debug_log_lv = TCAN_DEBUG_CLOSE;

static tcan4550_env_t s_tcan4550_env =
{
    .is_irq_transmit_pending = false,
};

/* Burst Write */
static uint32_t s_tx_buffer[64] = {0};
static uint32_t s_tx_size = 0;
/* Burst Read */
// aligned(16) is necessary for efficient memory access and to meet hardware requirements.
static uint32_t s_read_buff[CANFD_READ_WORDS_MAX] __attribute__((aligned(16)));
static uint32_t s_total_read_word  = 0;
static uint32_t s_cur_fetch_offset = 0;

static tcan4550_callback_t s_tcan4550_callback= {
    .rx_cplt_callback   = NULL,
    .tx_cplt_callback   = NULL,
    .xfer_err_callback  = NULL,
    .busoff_restore_calback  = NULL,
};

static tcan4550_rx_msg s_rx_msg;
static tcan4550_rx_msg s_rx_msg_buf[TCAN_RX_FIFO_ELEM_CNT + 1];

/*
 * STATIC FUNCTION DEFINITIONS
 ****************************************************************************************
 */
static void tcan4550_set_auto_retransmission(bool dis_retrans);
static bool tcan4550_poll_transmit_wait_result(uint32_t timeout);
static bool tcan4550_irq_try_wait_transmit_result(void);
static void tcan4550_irq_transmit_cplt(void);
static void tcan4550_recv_cplt(uint32_t id, uint8_t * data, uint8_t len);
static void tcan4550_busoff_restore(void);
static void tcan4550_irq_transmit_error(uint32_t error_code);

/*
 * GLOBAL PORT FUNCTION DEFINITIONS
 ****************************************************************************************
 */
__WEAK void spi_cspin_assert_port(void)
{
}

__WEAK void spi_cspin_deassert_port(void)
{
}

__WEAK bool spi_recv_word_dis_irq_port(uint8_t op_code, uint16_t address, uint32_t * p_buff, uint32_t words)
{
    return true;
}

__WEAK bool spi_recv_word_with_irq_port(uint8_t op_code, uint16_t address, uint32_t * p_buff, uint32_t words)
{
    return true;
}

__WEAK bool spi_send_word_port(uint8_t op_code, uint16_t address, uint32_t * p_buff, uint32_t words)
{
    return true;
}

__WEAK bool spi_send_word_data_only_port(uint32_t * p_buff, uint32_t words)
{
    return true;
}

__WEAK bool spi2can_get_int_irq_status(void)
{
    return true;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
// Adapt the function in TCAN4550.c.
uint32_t AHB_READ_32(uint16_t address)
{
    bool ret = true;
    uint32_t val = 0;

    spi_cspin_assert_port();
    ret = spi_recv_word_dis_irq_port(AHB_READ_OPCODE, address, &val, 1);
    spi_cspin_deassert_port();

    if (!ret && (s_debug_log_lv >= TCAN_DEBUG_LOG_LV_3))
    {
        APP_LOG_INFO("TCAN D: %s, address:0x%x Error!", __func__, address);
    }

    return ret ? val : 0;
}

// Adapt the function in TCAN4550.c.
uint32_t AHB_READ_32_Ext(uint16_t address)
{
    bool ret = true;
    uint32_t val = 0;

    spi_cspin_assert_port();
    ret = spi_recv_word_with_irq_port(AHB_READ_OPCODE, address, &val, 1);
    spi_cspin_deassert_port();

    if (!ret && (s_debug_log_lv >= TCAN_DEBUG_LOG_LV_3))
    {
        APP_LOG_INFO("TCAN D: %s, address:0x%x Error!", __func__, address);
    }

    return ret ? val : 0;
}

// Adapt the function in TCAN4550.c.
void AHB_READ_BURST_START(uint16_t address, uint8_t words)
{
    bool ret = false;

    memset(s_read_buff, 0, CANFD_READ_WORDS_MAX * 4);

    spi_cspin_assert_port();
    ret = spi_recv_word_dis_irq_port(AHB_READ_OPCODE, address, &s_read_buff[0], words);
    spi_cspin_deassert_port();

    if (!ret && (s_debug_log_lv >= TCAN_DEBUG_LOG_LV_3))
    {
        APP_LOG_INFO("TCAN D: %s, address:0x%x Error!", __func__, address);
    }

    s_total_read_word = ret ? words : 0;
    s_cur_fetch_offset = 0;
}

// Adapt the function in TCAN4550.c.
uint32_t AHB_READ_BURST_READ(void)
{
    uint32_t  data = 0;

    GLOBAL_EXCEPTION_DISABLE();
    if (s_cur_fetch_offset < s_total_read_word)
    {
        data = s_read_buff[s_cur_fetch_offset];
        s_cur_fetch_offset++;
    }
    GLOBAL_EXCEPTION_ENABLE();

    return data;
}

// Adapt the function in TCAN4550.c.
void AHB_READ_BURST_END(void)
{
    s_total_read_word  = 0;
    s_cur_fetch_offset = 0;
    memset(s_read_buff, 0, CANFD_READ_WORDS_MAX * 4);
}

// Adapt the function in TCAN4550.c.
void AHB_WRITE_32(uint16_t address, uint32_t data)
{
    bool ret = true;

    spi_cspin_assert_port();
    ret = spi_send_word_port(AHB_WRITE_OPCODE, address, &data, 1);
    spi_cspin_deassert_port();

    if (!ret && (s_debug_log_lv >= TCAN_DEBUG_LOG_LV_3))
    {
        APP_LOG_INFO("TCAN D: %s, address:0x%x Error!", __func__, address);
    }
}

// Adapt the function in TCAN4550.c.
void AHB_WRITE_BURST_START(uint16_t address, uint8_t words)
{
    s_tx_size = 0;
    s_tx_buffer[s_tx_size++] = (AHB_WRITE_OPCODE << 24UL) | (address << 8UL) | (words & 0xFF);
}

// Adapt the function in TCAN4550.c.
void AHB_WRITE_BURST_WRITE(uint32_t data)
{
    s_tx_buffer[s_tx_size++] = data;
}

// Adapt the function in TCAN4550.c.
void AHB_WRITE_BURST_END(void)
{
    spi_cspin_assert_port();
    spi_send_word_data_only_port((uint32_t *)&s_tx_buffer[0], s_tx_size );
    spi_cspin_deassert_port();
}

bool tcan4550_init(tcan4550_init_t *p_init)
{
    if (NULL == p_init)
    {
        return false;
    }

    s_tcan4550_env.can_mode = p_init->can_mode;
    s_tcan4550_env.brs_mode = p_init->brs_mode;
    s_tcan4550_env.retrans_mode = p_init->retrans_mode;

    s_tcan4550_callback.rx_cplt_callback = p_init->can_cb.rx_cplt_callback;
    s_tcan4550_callback.tx_cplt_callback = p_init->can_cb.tx_cplt_callback;
    s_tcan4550_callback.xfer_err_callback = p_init->can_cb.xfer_err_callback;
    s_tcan4550_callback.busoff_restore_calback = p_init->can_cb.busoff_restore_calback;

    TCAN4x5x_Device_ClearSPIERR(); // Clear any SPI ERR flags that might be set as a result of our pin mux changing during MCU startup

    TCAN4x5x_Device_Interrupts dev_ir; // Setup a new MCAN IR object for easy interrupt checking
    memset(&dev_ir, 0, sizeof(TCAN4x5x_Device_Interrupts));
    TCAN4x5x_Device_ReadInterrupts(&dev_ir); // Request that the struct be updated with current DEVICE (not MCAN) interrupt values
    if (dev_ir.PWRON) // If the Power On interrupt flag is set
    {
        TCAN4x5x_Device_ClearInterrupts(&dev_ir); // Clear it because if it's not cleared within ~4 minutes, it goes to sleep
    }

    /* Configure the CAN bus speeds */
    TCAN4x5x_MCAN_Nominal_Timing_Simple TCANNomTiming = {0};
    TCANNomTiming.NominalBitRatePrescaler    = p_init->normal_timing.brp;
    TCANNomTiming.NominalTqBeforeSamplePoint = p_init->normal_timing.seg1;
    TCANNomTiming.NominalTqAfterSamplePoint  = p_init->normal_timing.seg2;
    TCAN4x5x_MCAN_Data_Timing_Simple TCANDataTiming = {0};
    TCANDataTiming.DataBitRatePrescaler      = p_init->data_timing.brp;
    TCANDataTiming.DataTqBeforeSamplePoint   = p_init->data_timing.seg1;
    TCANDataTiming.DataTqAfterSamplePoint    = p_init->data_timing.seg2;

    /* Configure the MCAN core settings */
    TCAN4x5x_MCAN_CCCR_Config cccr_config;  // Remember to initialize to 0, or you'll get random garbage!
    memset(&cccr_config, 0, sizeof(TCAN4x5x_MCAN_CCCR_Config));
    cccr_config.FDOE = p_init->can_mode;                         // CAN FD mode enable
    cccr_config.BRSE = p_init->brs_mode;                         // CAN FD Bit rate switch enable
    cccr_config.DAR  = p_init->retrans_mode ^ 1;                 // Disable automatic retransmission

    /* Configure the default CAN packet filtering settings */
    TCAN4x5x_MCAN_Global_Filter_Configuration gfc;
    memset(&gfc, 0, sizeof(TCAN4x5x_MCAN_Global_Filter_Configuration));
    gfc.RRFE = 1;                                               // Reject remote frames (TCAN4x5x doesn't support this)
    gfc.RRFS = 1;                                               // Reject remote frames (TCAN4x5x doesn't support this)
    gfc.ANFE = TCAN4x5x_GFC_REJECT;                             // Default behavior if incoming message doesn't match a filter is to accept into RXFIO0 for extended ID messages (29 bit IDs)
    gfc.ANFS = TCAN4x5x_GFC_REJECT;                             // Default behavior if incoming message doesn't match a filter is to accept into RXFIO0 for standard ID messages (11 bit IDs)

    /* ************************************************************************
     * In the next configuration block, we will set the MCAN core up to have:
     *   - 1 SID filter element
     *   - 1 XID Filter element
     *   - 5 RX FIFO 0 elements
     *   - RX FIFO 0 supports data payloads up to 64 bytes
     *   - RX FIFO 1 and RX Buffer will not have any elements, but we still set their data payload sizes, even though it's not required
     *   - No TX Event FIFOs
     *   - 2 Transmit buffers supporting up to 64 bytes of data payload
     */
    TCAN4x5x_MRAM_Config MRAMConfiguration = {0};
    MRAMConfiguration.SIDNumElements = 8;                        // Standard ID number of elements, you MUST have a filter written to MRAM for each element defined
    MRAMConfiguration.XIDNumElements = 1;                        // Extended ID number of elements, you MUST have a filter written to MRAM for each element defined
    MRAMConfiguration.Rx0NumElements = TCAN_RX_FIFO_ELEM_CNT;    // RX0 Number of elements
    MRAMConfiguration.Rx0ElementSize = MRAM_64_Byte_Data;        // RX0 data payload size
    MRAMConfiguration.Rx1NumElements = 0;                        // RX1 number of elements
    MRAMConfiguration.Rx1ElementSize = MRAM_64_Byte_Data;        // RX1 data payload size
    MRAMConfiguration.RxBufNumElements = 0;                      // RX buffer number of elements
    MRAMConfiguration.RxBufElementSize = MRAM_64_Byte_Data;      // RX buffer data payload size
    MRAMConfiguration.TxEventFIFONumElements = 5;                // TX Event FIFO number of elements
    MRAMConfiguration.TxBufferNumElements = 5;                   // TX buffer number of elements
    MRAMConfiguration.TxBufferElementSize = MRAM_64_Byte_Data;   // TX buffer data payload size

    /* Configure the MCAN core with the settings above, the changes in this block are write protected registers,      *
     * so it makes the most sense to do them all at once, so we only unlock and lock once                             */

    TCAN4x5x_MCAN_EnableProtectedRegisters();                        // Start by making protected registers accessible
    TCAN4x5x_MCAN_ConfigureCCCRRegister(&cccr_config);               // Enable FD mode and Bit rate switching
    TCAN4x5x_MCAN_ConfigureGlobalFilter(&gfc);                       // Configure the global filter configuration (Default CAN message behavior)
    TCAN4x5x_MCAN_ConfigureNominalTiming_Simple(&TCANNomTiming);     // Setup nominal/arbitration bit timing
    TCAN4x5x_MCAN_ConfigureDataTiming_Simple(&TCANDataTiming);       // Setup CAN FD timing
    TCAN4x5x_MRAM_Clear();                                           // Clear all of MRAM (Writes 0's to all of it)
    TCAN4x5x_MRAM_Configure(&MRAMConfiguration);                     // Set up the applicable registers related to MRAM configuration
    TCAN4x5x_MCAN_DisableProtectedRegisters();                       // Disable protected write and take device out of INIT mode

    /* Set the interrupts we want to enable for MCAN */
    TCAN4x5x_MCAN_Interrupt_Enable mcan_ie;                      // Remember to initialize to 0, or you'll get random garbage!
    memset(&mcan_ie, 0, sizeof(TCAN4x5x_MCAN_Interrupt_Enable));
    mcan_ie.RF0NE = 1;                                           // RX FIFO 0 new message interrupt enable

    TCAN4x5x_MCAN_ConfigureInterruptEnable(&mcan_ie);            // Enable the appropriate registers

    /* Setup filters, this filter will mark any message with ID 0x055 as a priority message */
    TCAN4x5x_MCAN_SID_Filter SID_ID;
    memset(&SID_ID, 0, sizeof(TCAN4x5x_MCAN_SID_Filter));
    SID_ID.SFT   = TCAN4x5x_SID_SFT_RANGE;                       // SFT: Standard filter type. Configured as a classic filter
    SID_ID.SFEC  = TCAN4x5x_SID_SFEC_PRIORITYSTORERX0;           // Standard filter element configuration, store it in RX fifo 0 as a priority message
    SID_ID.SFID1 = 0x00;                                         // SFID1 (Classic mode Filter)
    SID_ID.SFID2 = 0x7FF;                                        // SFID2 (Classic mode Mask)
    TCAN4x5x_MCAN_WriteSIDFilter(0, &SID_ID);                    // Write to the MRAM

    /* Configure the TCAN4550 Non-CAN-related functions */
    TCAN4x5x_DEV_CONFIG devConfig;
    memset(&devConfig, 0, sizeof(TCAN4x5x_DEV_CONFIG));
    devConfig.SWE_DIS = 0;                                      // Keep Sleep Wake Error Enabled (it's a disable bit, not an enable)
    devConfig.DEVICE_RESET = 0;                                 // Not requesting a software reset
    devConfig.WD_EN = 0;                                        // Watchdog disabled
    devConfig.nWKRQ_CONFIG = 0;                                 // Mirror INH function (default)
    devConfig.INH_DIS = 0;                                      // INH enabled (default)
    devConfig.GPIO1_GPO_CONFIG = TCAN4x5x_DEV_CONFIG_GPO1_MCAN_INT1;    // MCAN nINT 1 (default)
    devConfig.FAIL_SAFE_EN = 0;                                 // Failsafe disabled (default)
    devConfig.GPIO1_CONFIG = TCAN4x5x_DEV_CONFIG_GPIO1_CONFIG_GPO;      // GPIO set as GPO (Default)
    devConfig.WD_ACTION = TCAN4x5x_DEV_CONFIG_WDT_ACTION_nINT;  // Watchdog set an interrupt (default)
    devConfig.WD_BIT_RESET = 0;                                 // Don't reset the watchdog
    devConfig.nWKRQ_VOLTAGE = 0;                                // Set nWKRQ to internal voltage rail (default)
    devConfig.GPO2_CONFIG = TCAN4x5x_DEV_CONFIG_GPO2_NO_ACTION; // GPO2 has no behavior (default)
    devConfig.CLK_REF = 1;                                      // Input crystal is a 40 MHz crystal (default)
    devConfig.WAKE_CONFIG = TCAN4x5x_DEV_CONFIG_WAKE_DISABLED;  // Wake pin can be triggered by either edge (default)
    TCAN4x5x_Device_Configure(&devConfig);                      // Configure the device with the above configuration

    TCAN4x5x_Device_SetMode(TCAN4x5x_DEVICE_MODE_NORMAL);       // Set to normal mode, since configuration is done. This line turns on the transceiver

    /* enable all tx buffer interrupt */
    AHB_WRITE_32(REG_MCAN_TXBTIE, 0xFFFFFFFFU);

    AHB_WRITE_32(0x800, 0x8004CA0);
    AHB_WRITE_32(0x1020, 0x70001);

    /* Enable irq */
    AHB_WRITE_32(0x10E0, 0x1);          // Transmission interrupt enable for Tx Buffer 0
    AHB_WRITE_32(0x1054, 0x3AF00E01);   // Enable important Interrupt Fields
    AHB_WRITE_32(0x1058, 0x0);          // All Interrupts assigned to m_can_int0 line
    AHB_WRITE_32(0x105C, 0x1);          // Interrupt line m_can_int0 enabled
    AHB_WRITE_32(0x0820, 0xFFFFFFFFU);   // Clear all error status

//    TCAN4x5x_MRAM_Conf_Trace();

    TCAN4x5x_Device_ClearSPIERR();

    return true;
}

void tcan4550_clear_all_interrupts(void)
{
    TCAN4x5x_MCAN_ClearInterruptsAll();
}

void tcan4550_set_standard_filter(tcan4550_standard_filter_init_t *p_sft_init)
{
    TCAN4x5x_MCAN_SID_Filter SID_ID;
    memset(&SID_ID, 0, sizeof(TCAN4x5x_MCAN_SID_Filter));
    SID_ID.SFT   = (TCAN4x5x_SID_SFT_Values)p_sft_init->sf_type;
    SID_ID.SFEC  = TCAN4x5x_SID_SFEC_STORERX0;
    SID_ID.SFID1 = p_sft_init->sft_id1;
    SID_ID.SFID2 = p_sft_init->sft_id2;
    TCAN4x5x_MCAN_WriteSIDFilter(p_sft_init->filter_index, &SID_ID);
}

bool tcan4550_transmit_it(uint32_t can_id, uint8_t *p_data, uint8_t length)
{
    uint8_t buf_idx = 0;
    bool b_ret = true;
    TCAN4x5x_MCAN_TX_Header header = {0};

    // Enable can retrans
    tcan4550_set_auto_retransmission(false);

    header.DLC = TCAN4x5x_MCAN_DataLen_To_DLC(length);
    header.ID  = can_id;
    header.XTD = 0;
    header.BRS = s_tcan4550_env.brs_mode;
    header.FDF = s_tcan4550_env.can_mode;
    header.RTR = 0;

    if (TCAN_BUSOFF_STATE_NO_RESTORED == tcan4550_get_busoff_state())
    {
        if (s_debug_log_lv >= TCAN_DEBUG_LOG_LV_3)
        {
            APP_LOG_INFO("TCAN D: BUS OFF NO RESTORED IN TX IT!");
        }
        return false;
    }

    s_tcan4550_env.is_irq_transmit_pending = false;

    GLOBAL_EXCEPTION_DISABLE();
    TCAN4x5x_MCAN_WriteTXBuffer(buf_idx, &header, p_data);
    TCAN4x5x_MCAN_TransmitBufferContents(buf_idx);
    s_tcan4550_env.is_irq_transmit_pending = true;
    GLOBAL_EXCEPTION_ENABLE();

//    APP_LOG_INFO("IRQ Tran id: %d, len: %d, b_ret: %d\r\n", can_id, length, b_ret);

    return b_ret;
}

bool tcan4550_transmit_polling(uint32_t can_id, uint8_t *p_data, uint8_t length, uint32_t timeout)
{
    uint8_t buf_idx = 0;
    bool b_ret = true;
    TCAN4x5x_MCAN_TX_Header header = {0};

    // Disable can retrans
    tcan4550_set_auto_retransmission(true);

    header.DLC = TCAN4x5x_MCAN_DataLen_To_DLC(length);
    header.ID  = can_id;
    header.XTD = 0;
    header.BRS = s_tcan4550_env.brs_mode;
    header.FDF = s_tcan4550_env.can_mode;
    header.RTR = 0;

    if (TCAN_BUSOFF_STATE_NO_RESTORED == tcan4550_get_busoff_state())
    {
        if (s_debug_log_lv >= TCAN_DEBUG_LOG_LV_3)
        {
            APP_LOG_INFO("TCAN D: BUSOFF NO RESTORED IN TX POLL!");
        }
        return false;
    }

    s_tcan4550_env.is_irq_transmit_pending = false;

    AHB_WRITE_32(REG_MCAN_TXBTIE, 0x0);          // Transmission interrupt disable for Tx Buffer 0

    GLOBAL_EXCEPTION_DISABLE();
    TCAN4x5x_MCAN_WriteTXBuffer(buf_idx, &header, p_data);
    TCAN4x5x_MCAN_TransmitBufferContents(buf_idx);
    GLOBAL_EXCEPTION_ENABLE();

    //polling
    b_ret = tcan4550_poll_transmit_wait_result(timeout);
    if (!b_ret && (s_debug_log_lv >= TCAN_DEBUG_LOG_LV_2))
    {
        APP_LOG_INFO("TCAN D: POLL SEND TIMEOUT!");
    }

//    APP_LOG_INFO("Poll Tran id: %d, len: %d, b_ret: %d\r\n", can_id, length, b_ret);

    AHB_WRITE_32(REG_MCAN_TXBTIE, 0x1);          // Transmission interrupt enable for Tx Buffer 0

    return b_ret;
}

void tcan4550_run_wakeup_irq_process(void)
{
    //TODO : ADD WAKEUP FUNC.
}

void tcan4550_run_int_irq_process(void)
{
    static uint32_t fail_counter = 0;
    uint8_t counter = 0;
    TCAN4x5x_Device_Interrupts dev_ir; // Define a new Device IR object for device (non-CAN) interrupt checking
    TCAN4x5x_MCAN_Interrupts mcan_ir;  // Setup a new MCAN IR object for easy interrupt checking
    TCAN4x5x_Protocol_Status dev_psr;
    TCAN4x5x_MCAN_CCCR_Config dev_cccr;
    memset(&dev_ir, 0, sizeof(TCAN4x5x_Device_Interrupts));
    memset(&mcan_ir, 0, sizeof(TCAN4x5x_MCAN_Interrupts));
    memset(&dev_psr, 0, sizeof(TCAN4x5x_Protocol_Status));
    memset(&dev_cccr, 0, sizeof(TCAN4x5x_MCAN_CCCR_Config));

    TCAN4x5x_Device_ReadInterrupts(&dev_ir);  // Read the device interrupt register
    TCAN4x5x_MCAN_ReadInterrupts(&mcan_ir);   // Read the interrupt register
    TCAN4x5x_MCAN_ReadCPSRRegister(&dev_psr);
    TCAN4x5x_MCAN_ReadCCCRRegister(&dev_cccr);

    if (s_debug_log_lv >= TCAN_DEBUG_LOG_LV_1)
    {
        APP_LOG_INFO("TCAN D: MCAN.IR=0x%08x,PSR=0x%08x,FAIL=%d", mcan_ir.word, dev_psr.word, fail_counter);
    }

    if (mcan_ir.TC)
    {
        // Transmission Finished
        s_tcan4550_env.is_irq_transmit_pending = false;

        tcan4550_irq_transmit_cplt();
    }

    if (mcan_ir.TFE && (s_debug_log_lv >= TCAN_DEBUG_LOG_LV_1))
    {
        APP_LOG_INFO("TCAN D: TFE");
    }

    if (mcan_ir.TCF && (s_debug_log_lv >= TCAN_DEBUG_LOG_LV_1))
    {
        APP_LOG_INFO("TCAN D: TCF");
    }

    if (mcan_ir.RF0N || !TCAN4x5x_MCAN_Is_RxFifo_Empty()) // If a new message in RX FIFO 0
    {
        uint8_t cnt = 0;
        do {
            counter = TCAN4x5x_MCAN_ReadNextFIFO(RXFIFO0, &s_rx_msg.header, s_rx_msg.dat); // This will read the next element in the RX FIFO 0
            if (counter != 0)
            {
                if (counter != TCAN4x5x_MCAN_DLCtoBytes(s_rx_msg.header.DLC))
                {
                    /* NO Need to handle rx error */
                    if (s_debug_log_lv >= TCAN_DEBUG_LOG_LV_3)
                    {
                        APP_LOG_INFO("TCAN D: Read rx fifo0 error %d - %d", counter, TCAN4x5x_MCAN_DLCtoBytes(s_rx_msg.header.DLC));
                    }
                    break;
                }

                memcpy(&s_rx_msg_buf[cnt], &s_rx_msg, sizeof(tcan4550_rx_msg));
                cnt++;
            }
        } while (counter != 0 && (cnt < 10)); // Limit the number of consecutive reads from the RX FIFO in SPI2CAN to avoid the CAN RX interrupt occupying too much time.

        for (uint8_t i = 0; i < cnt; i++)
        {
            uint8_t dlen = TCAN4x5x_MCAN_DLC_To_DataLen(s_rx_msg_buf[i].header.DLC);
            tcan4550_recv_cplt(s_rx_msg_buf[i].header.ID, s_rx_msg_buf[i].dat, dlen);
        }

        if (s_tcan4550_env.is_irq_transmit_pending)
        {
            TCAN4x5x_MCAN_Interrupts tmp_ir;
            memset(&tmp_ir, 0, sizeof(TCAN4x5x_MCAN_Interrupts));
            TCAN4x5x_MCAN_ReadInterrupts(&tmp_ir);

            if (tmp_ir.TC)
            {
                tcan4550_irq_transmit_cplt();
                s_tcan4550_env.is_irq_transmit_pending = false;
                fail_counter = 0;
            }
            else
            {
                fail_counter ++;
            }

            if (fail_counter >= 2)
            {
                tcan4550_busoff_restore();
                fail_counter = 0;
                s_tcan4550_env.is_irq_transmit_pending = false;
            }
        }
    }

    /*
        CAN ERROR TYPE:
          1. CANERR, CAN Error
          2. CANDOM, Can bus stuck dominant
          3. EP, Error Passive
          4. EW, Error Warning
          5. BO, BUS OFF
          6. PEA, Protocol Error in Arbitration Phase
          7. PED, Protocol Error in Data Phase
    */
    if (dev_ir.CANERR || dev_ir.CANDOM ||
       dev_psr.EP || dev_psr.EW || dev_psr.BO  ||
       mcan_ir.EP || mcan_ir.EW || mcan_ir.PEA || mcan_ir.PED || mcan_ir.BO ||
       (dev_cccr.word & REG_BITS_MCAN_CCCR_INIT) )
    {
        // BUS OFF
        if (dev_psr.BO || mcan_ir.BO /*|| (dev_cccr.word & REG_BITS_MCAN_CCCR_INIT)*/)
        {
            uint8_t bus_recory = 0;

            if (s_debug_log_lv >= TCAN_DEBUG_LOG_LV_3)
            {
                APP_LOG_INFO("TCAN D: BUS-OFF in it.");
            }

            bus_recory = tcan4550_get_busoff_state();
            if (TCAN_BUSOFF_STATE_RESTORED == bus_recory)
            {
                tcan4550_busoff_restore();
            }
            else if (TCAN_BUSOFF_STATE_NO_RESTORED == bus_recory)
            {
                tcan4550_irq_transmit_error(TCAN_SPI2CAN_ERROR_BUSOFF);
                if (s_debug_log_lv >= TCAN_DEBUG_LOG_LV_3)
                {
                    APP_LOG_INFO("TCAN D: Bus Off Recovery fail in it.");
                }
            }
            else
            {
                // Nothing to do.
            }
        }
        else
        {
            // Other Errors
            if (s_tcan4550_env.is_irq_transmit_pending)
            {
                if (tcan4550_irq_try_wait_transmit_result())
                {
                    s_tcan4550_env.is_irq_transmit_pending = false;
                }
            }
        }
    }

    if (mcan_ir.word != 0)
    {
        TCAN4x5x_MCAN_ClearInterruptsAll();
    }

    if (dev_ir.word != 0)
    {
        TCAN4x5x_Device_ClearInterruptsAll();
    }

    while (!spi2can_get_int_irq_status())
    {
        delay_us(5);
        TCAN4x5x_MCAN_ClearInterruptsAll();
        TCAN4x5x_Device_ClearInterruptsAll();
    }

    if (dev_ir.SPIERR) // If the SPIERR flag is set
    {
        TCAN4x5x_Device_ClearSPIERR(); // Clear the SPIERR flag
    }

    return;
}

tcan4550_busoff_state_t tcan4550_get_busoff_state(void)
{
    TCAN4x5x_Protocol_Status dev_psr;
    memset(&dev_psr, 0, sizeof(TCAN4x5x_Protocol_Status));
    TCAN4x5x_MCAN_ReadCPSRRegister(&dev_psr);

    if (dev_psr.BO)
    {
        uint8_t r1, r2;
        uint32_t counter = TCAN_HANDLE_BUSOFF_RECOVERY_TIMEOUT;
        bool is_bus_recory = false;

        TCAN4x5x_MCAN_Cancel_Transmission(0);

        r1 = TCAN4x5x_MCAN_EnableProtectedRegisters();
        r2 = TCAN4x5x_MCAN_DisableProtectedRegisters();

        if (s_debug_log_lv >= TCAN_DEBUG_LOG_LV_3)
        {
            APP_LOG_INFO("TCAN D: Bus Off, try reset ,%d:%d.", r1, r2);
        }

        while (counter--)
        {
            TCAN4x5x_MCAN_ReadCPSRRegister(&dev_psr);

            if (dev_psr.BO == 0)
            {
                is_bus_recory = true;
                break;
            }

            delay_ms(1);
        }

        if (is_bus_recory)
        {
            return TCAN_BUSOFF_STATE_RESTORED;
        }
        else
        {
            return TCAN_BUSOFF_STATE_NO_RESTORED;
        }
    }

    return TCAN_BUSOFF_STATE_NO_OCCURRED;
}

void tcan4550_set_debug_log_level(tcan4550_debug_log_lv_t debug_log_lv)
{
    s_debug_log_lv = debug_log_lv;
}

/*
 * STATIC FUNCTION DEFINITIONS
 ****************************************************************************************
 */
static void tcan4550_set_auto_retransmission(bool dis_retrans)
{
    if ((s_tcan4550_env.retrans_mode ^ 1) == (uint8_t)dis_retrans)
    {
        return;
    }

    TCAN4x5x_MCAN_CCCR_Config cccrConfig; // Remember to initialize to 0, or you'll get random garbage!
    memset(&cccrConfig, 0, sizeof(TCAN4x5x_MCAN_CCCR_Config));
    cccrConfig.FDOE = s_tcan4550_env.can_mode;                  // CAN FD mode enable
    cccrConfig.BRSE = s_tcan4550_env.brs_mode;                  // CAN FD Bit rate switch enable
    cccrConfig.DAR  = dis_retrans ? 1 : 0;                      // Disable automatic retransmission

    if (s_debug_log_lv >= TCAN_DEBUG_LOG_LV_1)
    {
        APP_LOG_INFO("TCAN D: Set Retrans : %d", (uint8_t)dis_retrans);
    }

    s_tcan4550_env.retrans_mode = (tcan4550_retrans_mode_t)(!dis_retrans);

    TCAN4x5x_MCAN_EnableProtectedRegisters();                   // Start by making protected registers accessible
    TCAN4x5x_MCAN_ConfigureCCCRRegister(&cccrConfig);           // Enable FD mode and Bit rate switching
    TCAN4x5x_MCAN_DisableProtectedRegisters();                  // Disable protected write and take device out of INIT mode
}

static bool tcan4550_poll_transmit_wait_result(uint32_t timeout)
{
    TCAN4x5x_MCAN_Interrupts mcan_ir; // Setup a new MCAN IR object for easy interrupt checking
    memset(&mcan_ir, 0, sizeof(TCAN4x5x_MCAN_Interrupts));
    bool is_timeout = false;

    HAL_TIMEOUT_INIT();
    uint32_t tickstart = HAL_TIMEOUT_GET_TICK();
    /* Update SystemCoreClock */
    SystemCoreUpdateClock();
    uint32_t timeout_counter = (SystemCoreClock / 1000 * timeout);

    /*
     * Wait until flag is in expected state
     * Disable IRQ may affect BLE performance
     */
    while (1)
    {
        if ((((HAL_TIMEOUT_GET_TICK() - tickstart) > timeout_counter)))
        {
            is_timeout = true;
            break;
        }

        TCAN4x5x_MCAN_ReadInterrupts_Ext(&mcan_ir);

        if (!AHB_READ_32_Ext(REG_MCAN_TXBRP)) // Transmission Request Pending
        {
            is_timeout = false;
            break;
        }
    }
    HAL_TIMEOUT_DEINIT();

    if (is_timeout)
    {
        uint32_t ret2 = 0;

        TCAN4x5x_MCAN_Cancel_Transmission(0);

        HAL_TIMEOUT_INIT();
        tickstart = HAL_TIMEOUT_GET_TICK();
        /* Update SystemCoreClock */
        SystemCoreUpdateClock();
        timeout_counter = (SystemCoreClock / 1000 * 3);

        while (1)
        {
            if ((((HAL_TIMEOUT_GET_TICK() - tickstart) > timeout_counter)))
            {
                ret2 = 0;
                break;
            }

            TCAN4x5x_MCAN_ReadInterrupts_Ext(&mcan_ir);

            if (mcan_ir.TC == 1)
            {
                ret2 = 1;
                break;
            }
            else if (mcan_ir.TCF == 1)
            {
                ret2 = 2;
                break;
            }
            else
            {
                // Nothing to do.
            }
        }

        HAL_TIMEOUT_DEINIT();

        if (ret2 == 1)
        {
            mcan_ir.word = 0;
            mcan_ir.TC   = 1;
            TCAN4x5x_MCAN_ClearInterrupts(&mcan_ir);
            tcan4550_irq_transmit_cplt();

            return true;
        }
        else if (ret2 == 2)
        {
            mcan_ir.word = 0;
            mcan_ir.TCF  = 1;
            TCAN4x5x_MCAN_ClearInterrupts(&mcan_ir);

            return false;
        }
        else
        {
            // Nothing to do.
        }

        return false;
    }
    else
    {
        mcan_ir.word = 0;
        mcan_ir.TC   = 1;
        TCAN4x5x_MCAN_ClearInterrupts(&mcan_ir);

        tcan4550_irq_transmit_cplt();
        return true;
    }
}

/* return:
    *        false : timeout
    *        true : transmit ok
 ************************/
static bool tcan4550_irq_try_wait_transmit_result(void)
{
    bool ret               = false;
    uint32_t r_mcan        = 0;
    const uint32_t timeout = 5;
    uint32_t status        = 0;

    HAL_TIMEOUT_INIT();
    uint32_t tickstart = HAL_TIMEOUT_GET_TICK();
    /* Update SystemCoreClock */
    SystemCoreUpdateClock();
    uint32_t timeout_counter = (SystemCoreClock / 1000 * timeout);

    while (1)
    {
        if ((((HAL_TIMEOUT_GET_TICK() - tickstart) > timeout_counter)))
        {
            status = 0;
            break;
        }

        r_mcan = TCAN4x5x_Device_ReadRegister(0x0824);

        if (r_mcan & 0x200) //TC
        {
            status = 1;
            break;
        }

        if (r_mcan & 0x2000000) //BO
        {
            status = 2;
            break;
        }
    }
    HAL_TIMEOUT_DEINIT();

    switch (status)
    {
        case 1:
        {
            tcan4550_irq_transmit_cplt();
            ret = true;
        }
        break;

        case 2:
        {
            uint8_t status = 0;
            status = tcan4550_get_busoff_state();

            if (TCAN_BUSOFF_STATE_RESTORED == status)
            {
                tcan4550_busoff_restore();
                ret = true;
            }
            else
            {
                if (s_debug_log_lv >= TCAN_DEBUG_LOG_LV_3)
                {
                    APP_LOG_INFO("TCAN D: Bus Off Recovery fail in try wait.");
                }
            }
        }
        break;

        case 0:
        default:
        {
            ret = false;
        }
        break;
    }

    return ret;
}

static void tcan4550_irq_transmit_cplt(void)
{
    if (s_tcan4550_callback.tx_cplt_callback != NULL)
    {
        s_tcan4550_callback.tx_cplt_callback();
    }
}

static void tcan4550_recv_cplt(uint32_t id, uint8_t * data, uint8_t len)
{
    if (s_tcan4550_callback.rx_cplt_callback != NULL)
    {
        s_tcan4550_callback.rx_cplt_callback(id, data, len);
    }
}

static void tcan4550_busoff_restore(void)
{
    if (s_tcan4550_callback.busoff_restore_calback != NULL)
    {
        s_tcan4550_callback.busoff_restore_calback();
    }
}

static void tcan4550_irq_transmit_error(uint32_t error_code)
{
    if (s_tcan4550_env.is_irq_transmit_pending)
    {
        s_tcan4550_env.is_irq_transmit_pending = false;

        if (s_tcan4550_callback.xfer_err_callback != NULL)
        {
            s_tcan4550_callback.xfer_err_callback(error_code);
        }
    }
}

#else
bool tcan4550_init(tcan4550_init_t *p_init)
{
    APP_LOG_INFO("Error: TCAN4550.h is not found.");
    APP_LOG_INFO("Error: Please download the TCAN4550 driver from the corresponding chip manufacturer.");
    APP_LOG_INFO("Error: Which includes: TCAN4x5x_Data_Structs.h, TCAN4x5x_Reg.h, TCAN4550.c, TCAN4550.h.");
    APP_ASSERT_CHECK(0);
    return false;
}

void tcan4550_clear_all_interrupts(void)
{
}

void tcan4550_set_standard_filter(tcan4550_standard_filter_init_t *p_sft_init)
{
}

bool tcan4550_transmit_it(uint32_t can_id, uint8_t *p_data, uint8_t length)
{
    return false;
}

bool tcan4550_transmit_polling(uint32_t can_id, uint8_t *p_data, uint8_t length, uint32_t timeout)
{
    return false;
}

void tcan4550_run_wakeup_irq_process(void)
{
}

void tcan4550_run_int_irq_process(void)
{
}

tcan4550_busoff_state_t tcan4550_get_busoff_state(void)
{
    return TCAN_BUSOFF_STATE_NO_OCCURRED;
}

void tcan4550_set_debug_log_level(tcan4550_debug_log_lv_t debug_log_lv)
{
}

#endif

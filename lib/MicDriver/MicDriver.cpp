#include "MicDriver.h"

#include <FspTimer.h>
#include <r_adc.h>
#include <r_dmac.h>
#include <r_elc.h>

#include <algorithm>

MicDriver::MicDriver(float freq_hz, pin_size_t adc_pin_number,
                     uint32_t dmac_channel)
    : freq_hz(freq_hz),
      adc_pin_number(adc_pin_number),
      dmac_channel(dmac_channel) {}

MicDriver::~MicDriver() {}

MicDriver::BeginStatus MicDriver::begin() {
    auto rv = init_timer();
    if (rv != MicDriver::BeginStatus::SUCCESS) {
        return rv;
    }

    adc_extended_cfg_t adc_extend = {};
    adc_extend.add_average_count = ADC_ADD_OFF;
    adc_extend.clearing = ADC_CLEAR_AFTER_READ_OFF;
    adc_extend.trigger_group_b = ADC_TRIGGER_SYNC_ELC;
    adc_extend.double_trigger_mode = ADC_DOUBLE_TRIGGER_DISABLED;
    adc_extend.adc_vref_control = ADC_VREF_CONTROL_AVCC0_AVSS0;
    adc_extend.enable_adbuf = 0;
    adc_extend.window_a_irq = FSP_INVALID_VECTOR;
    adc_extend.window_b_irq = FSP_INVALID_VECTOR;
    adc_extend.window_a_ipl = 0;
    adc_extend.window_b_ipl = 0;

    adc_cfg_t adc_cfg = {};
    adc_cfg.unit = 0;
    adc_cfg.mode = ADC_MODE_SINGLE_SCAN;
    adc_cfg.resolution = ADC_RESOLUTION_14_BIT;
    adc_cfg.alignment = ADC_ALIGNMENT_RIGHT;
    adc_cfg.trigger = ADC_TRIGGER_SYNC_ELC;
    adc_cfg.scan_end_irq = FSP_INVALID_VECTOR;
    adc_cfg.scan_end_b_irq = FSP_INVALID_VECTOR;
    adc_cfg.scan_end_ipl = 0;
    adc_cfg.scan_end_b_ipl = 0;
    adc_cfg.p_callback = nullptr;
    adc_cfg.p_context = nullptr;
    adc_cfg.p_extend = &adc_extend;

    fsp_err_t fsp_err = R_ADC_Open(&adc_ctrl, &adc_cfg);
    if (fsp_err != FSP_SUCCESS) {
        Serial.print("R_ADC_Open: ");
        Serial.println(fsp_err);
        return MicDriver::BeginStatus::FAIL;
    }

    int32_t adc_pin_idx = digitalPinToAnalogPin(adc_pin_number);
    pin_size_t bsp_pin = digitalPinToBspPin(adc_pin_idx);

    adc_window_cfg_t adc_window_cfg = {};
    adc_channel_cfg_t adc_ch_cfg = {};
    adc_ch_cfg.scan_mask = (1U << bsp_pin);
    adc_ch_cfg.scan_mask_group_b = 0;
    adc_ch_cfg.add_mask = 0;
    adc_ch_cfg.p_window_cfg = &adc_window_cfg;
    adc_ch_cfg.priority_group_a = ADC_GROUP_A_PRIORITY_OFF;
    adc_ch_cfg.sample_hold_mask = 0;
    adc_ch_cfg.sample_hold_states = 0;
    R_ADC_ScanCfg(&adc_ctrl, &adc_ch_cfg);

    elc_cfg_t elc_cfg = {ELC_EVENT_NONE};
    fsp_err = R_ELC_Open(&elc_ctrl, &elc_cfg);
    fsp_err = R_ELC_Enable(&elc_ctrl);
    fsp_err = R_ELC_LinkSet(&g_elc_ctrl, ELC_PERIPHERAL_ADC0, timer_event);

    transfer_info_t dmac_info = {};
    dmac_extended_cfg_t dmac_extend_cfg = {};
    transfer_cfg_t dmac_cfg = {&dmac_info, &dmac_extend_cfg};

    dmac_extend_cfg.activation_source = ELC_EVENT_ADC0_SCAN_END;
    dmac_extend_cfg.p_callback = nullptr;
    dmac_extend_cfg.p_context = nullptr;
    dmac_extend_cfg.channel = dmac_channel;
    dmac_extend_cfg.offset = 1;
    dmac_extend_cfg.src_buffer_size = 1;
    dmac_extend_cfg.irq = FSP_INVALID_VECTOR;
    dmac_extend_cfg.ipl = 12;

    dmac_info.transfer_settings_word_b.dest_addr_mode =
        TRANSFER_ADDR_MODE_INCREMENTED;
    dmac_info.transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_FIXED;
    dmac_info.transfer_settings_word_b.mode = TRANSFER_MODE_REPEAT;
    dmac_info.transfer_settings_word_b.size = TRANSFER_SIZE_2_BYTE;
    dmac_info.transfer_settings_word_b.repeat_area =
        TRANSFER_REPEAT_AREA_DESTINATION;
    dmac_info.transfer_settings_word_b.irq = TRANSFER_IRQ_END;
    dmac_info.transfer_settings_word_b.chain_mode =
        TRANSFER_CHAIN_MODE_DISABLED;
    dmac_info.p_src = (void*)&R_ADC0->ADDR[bsp_pin];
    dmac_info.p_dest = (void*)&dma_buffer[0];
    dmac_info.length = DMA_BUFFER_LEN;
    dmac_info.num_blocks = 0;

    R_DMAC_Open(&dmac_ctrl, &dmac_cfg);
    R_DMAC_Enable(&dmac_ctrl);

    timer.open();
    timer.start();

    R_ADC_ScanStart(&adc_ctrl);

    return MicDriver::BeginStatus::SUCCESS;
}

uint8_t* MicDriver::read_packet() {
    // Snapshot write_pos once — it's volatile and advances in the ISR
    int w = write_pos;

    // Calculate how many samples are available
    int available = (w - read_pos) & RING_BUFFER_LEN_MASK;

    if (available < BUFFER_LEN) {
        return nullptr;  // not enough data yet
    }

    // If we've fallen more than one ring buffer behind, skip ahead to the
    // most recent BUFFER_LEN samples so we don't send stale audio
    if (available > RING_BUFFER_LEN / 2) {
        read_pos = (w - BUFFER_LEN) & RING_BUFFER_LEN_MASK;
    }

    // Copy BUFFER_LEN bytes from ring buffer into read_buffer,
    // handling wrap-around
    int first_chunk = std::min(BUFFER_LEN, RING_BUFFER_LEN - read_pos);
    memcpy(read_buffer, &ring_buffer[read_pos], first_chunk);
    if (first_chunk < BUFFER_LEN) {
        memcpy(read_buffer + first_chunk, &ring_buffer[0],
               BUFFER_LEN - first_chunk);
    }

    read_pos = (read_pos + BUFFER_LEN) & RING_BUFFER_LEN_MASK;
    return read_buffer;
}

void MicDriver::on_timer(timer_callback_args_t* args) {
    constexpr int32_t IN_MIN = 819;
    constexpr int32_t IN_MAX = 7373;
    constexpr int32_t OUT_MIN = 0;
    constexpr int32_t OUT_MAX = 255;
    constexpr int32_t SCALE_FIXED =
        (int32_t)((OUT_MAX - OUT_MIN) * 16384.0f / (IN_MAX - IN_MIN));

    // Convert 14-bit ADC reading to 8-bit sample
    int32_t dma_value = dma_buffer[dma_buffer_read_pos++];
    dma_buffer_read_pos &= DMA_BUFFER_LEN_MASK;

    int32_t result = (dma_value - IN_MIN) * SCALE_FIXED >> 14;
    uint8_t clamped = (uint8_t)std::clamp(result, OUT_MIN, OUT_MAX);

    // Always write — overwrite old data if consumer is too slow
    ring_buffer[write_pos] = clamped;
    write_pos = (write_pos + 1) & RING_BUFFER_LEN_MASK;
}

MicDriver::BeginStatus MicDriver::init_timer() {
    uint8_t timer_type;
    int8_t timer_index = FspTimer::get_available_timer(timer_type);

    if (timer_index < 0) {
        timer_index = FspTimer::get_available_timer(timer_type, true);
    }
    if (timer_index < 0) {
        return MicDriver::BeginStatus::FAIL_CANNOT_GET_TIMER;
    }

    if (timer_type == GPT_TIMER) {
        switch (timer_index) {
            case 0: timer_event = ELC_EVENT_GPT0_COUNTER_OVERFLOW; break;
            case 1: timer_event = ELC_EVENT_GPT1_COUNTER_OVERFLOW; break;
            case 2: timer_event = ELC_EVENT_GPT2_COUNTER_OVERFLOW; break;
            case 3: timer_event = ELC_EVENT_GPT3_COUNTER_OVERFLOW; break;
            case 4: timer_event = ELC_EVENT_GPT4_COUNTER_OVERFLOW; break;
            case 5: timer_event = ELC_EVENT_GPT5_COUNTER_OVERFLOW; break;
            case 6: timer_event = ELC_EVENT_GPT6_COUNTER_OVERFLOW; break;
            case 7: timer_event = ELC_EVENT_GPT7_COUNTER_OVERFLOW; break;
            default: return MicDriver::BeginStatus::FAIL_TIMER_INDEX_OUT_OF_RANGE;
        }
    } else if (timer_type == AGT_TIMER) {
        switch (timer_index) {
            case 0: timer_event = ELC_EVENT_AGT0_INT; break;
            case 1: timer_event = ELC_EVENT_AGT1_INT; break;
            default: return MicDriver::BeginStatus::FAIL_TIMER_INDEX_OUT_OF_RANGE;
        }
    }

    timer.begin(TIMER_MODE_PERIODIC, timer_type, timer_index, freq_hz, 50.0f,
                MicDriver::timer_callback, this);
    timer.setup_overflow_irq(1);

    return MicDriver::BeginStatus::SUCCESS;
}
#include "MicDriver.h"

#include <FspTimer.h>
#include <r_adc.h>
#include <r_dmac.h>
#include <r_elc.h>

#include <algorithm>

MicDriver::MicDriver(float freq_hz, pin_size_t adc_pin_number,
                     uint32_t dmac_channel,
                     uint8_t* buf0, uint8_t* buf1, uint8_t* buf2)
    : freq_hz(freq_hz),
      adc_pin_number(adc_pin_number),
      dmac_channel(dmac_channel),
      buf0(buf0), buf1(buf1), buf2(buf2) {}

MicDriver::BeginStatus MicDriver::begin() {
    auto rv = init_timer();
    if (rv != BeginStatus::SUCCESS) return rv;

    rv = init_adc();
    if (rv != BeginStatus::SUCCESS) return rv;

    return init_dma();
}

void MicDriver::start() {
    // Reset buffer state
    dma_writing_buf      = buf0;
    staged_ready_buf     = nullptr;
    net_reading_buf      = nullptr;
    dma_write_packet_count = 0;
    packet_sample_pos    = 0;

    R_DMAC_Enable(&dmac_ctrl);
    R_ADC_ScanStart(&adc_ctrl);
    timer.start();
}

void MicDriver::stop() {
    timer.stop();
    R_ADC_ScanStop(&adc_ctrl);
    R_DMAC_Disable(&dmac_ctrl);
}

uint8_t* MicDriver::read_packet() {
    if (staged_ready_buf == nullptr) return nullptr;

    // Point net_reading_buf at the staged buffer
    net_reading_buf  = (uint8_t*)staged_ready_buf;
    staged_ready_buf = nullptr;

    // The buffer we just consumed is now free — give it back as dma_writing_buf
    // if the ISR has filled its current one and promoted it already
    // (The ISR will pick it up on the next buffer promotion)

    // Return pointer to first packet in the buffer
    // Caller advances through packets via repeated calls
    static int read_packet_index = 0;

    if (net_reading_buf == nullptr) return nullptr;

    // Return all packets from net_reading_buf one at a time
    // We use a static index that resets when we get a new buffer
    uint8_t* packet = net_reading_buf + (read_packet_index * PACKET_LEN);
    read_packet_index++;

    if (read_packet_index >= PACKETS_PER_BUFFER) {
        read_packet_index = 0;
        // Free this buffer — find the one that's not dma_writing_buf or staged
        uint8_t* freed = net_reading_buf;
        net_reading_buf = nullptr;
        // Hand freed buffer back to ISR if it needs one
        // (ISR checks for nullptr dma_writing_buf after promotion)
        if (dma_writing_buf == nullptr) {
            dma_writing_buf = freed;
            dma_write_packet_count = 0;
            packet_sample_pos = 0;
        }
    }

    return packet;
}

void MicDriver::on_timer(timer_callback_args_t* args) {
    constexpr int32_t IN_MIN = 819;
    constexpr int32_t IN_MAX = 7373;
    constexpr int32_t OUT_MIN = 0;
    constexpr int32_t OUT_MAX = 255;
    constexpr int32_t SCALE_FIXED =
        (int32_t)((OUT_MAX - OUT_MIN) * 16384.0f / (IN_MAX - IN_MIN));

    if (dma_writing_buf == nullptr) {
        // No buffer available — drop sample
        adc_dma_read_pos = (adc_dma_read_pos + 1) & ADC_DMA_LEN_MASK;
        return;
    }

    // Convert 14-bit ADC reading to 8-bit unsigned PCM
    int32_t raw    = adc_dma_buffer[adc_dma_read_pos];
    adc_dma_read_pos = (adc_dma_read_pos + 1) & ADC_DMA_LEN_MASK;

    int32_t result  = (raw - IN_MIN) * SCALE_FIXED >> 14;
    uint8_t sample  = (uint8_t)std::clamp(result, OUT_MIN, OUT_MAX);

    // Write into current packet position within dma_writing_buf
    int buf_offset = dma_write_packet_count * PACKET_LEN + packet_sample_pos;
    dma_writing_buf[buf_offset] = sample;
    packet_sample_pos++;

    if (packet_sample_pos >= PACKET_LEN) {
        packet_sample_pos = 0;
        dma_write_packet_count++;

        if (dma_write_packet_count >= PACKETS_PER_BUFFER) {
            // Buffer is full — promote to staged if slot is free
            if (staged_ready_buf == nullptr) {
                staged_ready_buf = dma_writing_buf;
            }
            // Grab the third buffer for next write cycle
            uint8_t* next = buf2;
            if (next == staged_ready_buf || next == net_reading_buf) next = buf0;
            if (next == staged_ready_buf || next == net_reading_buf) next = buf1;
            if (next == staged_ready_buf || next == net_reading_buf) {
                // No free buffer — stall
                dma_writing_buf = nullptr;
            } else {
                dma_writing_buf = next;
            }
            dma_write_packet_count = 0;
        }
    }
}

MicDriver::BeginStatus MicDriver::init_adc() {
    adc_extended_cfg_t adc_extend = {};
    adc_extend.add_average_count   = ADC_ADD_OFF;
    adc_extend.clearing            = ADC_CLEAR_AFTER_READ_OFF;
    adc_extend.trigger_group_b     = ADC_TRIGGER_SYNC_ELC;
    adc_extend.double_trigger_mode = ADC_DOUBLE_TRIGGER_DISABLED;
    adc_extend.adc_vref_control    = ADC_VREF_CONTROL_AVCC0_AVSS0;
    adc_extend.enable_adbuf        = 0;
    adc_extend.window_a_irq        = FSP_INVALID_VECTOR;
    adc_extend.window_b_irq        = FSP_INVALID_VECTOR;
    adc_extend.window_a_ipl        = 0;
    adc_extend.window_b_ipl        = 0;

    adc_cfg_t adc_cfg = {};
    adc_cfg.unit          = 0;
    adc_cfg.mode          = ADC_MODE_SINGLE_SCAN;
    adc_cfg.resolution    = ADC_RESOLUTION_14_BIT;
    adc_cfg.alignment     = ADC_ALIGNMENT_RIGHT;
    adc_cfg.trigger       = ADC_TRIGGER_SYNC_ELC;
    adc_cfg.scan_end_irq  = FSP_INVALID_VECTOR;
    adc_cfg.scan_end_b_irq = FSP_INVALID_VECTOR;
    adc_cfg.scan_end_ipl  = 0;
    adc_cfg.scan_end_b_ipl = 0;
    adc_cfg.p_callback    = nullptr;
    adc_cfg.p_context     = nullptr;
    adc_cfg.p_extend      = &adc_extend;

    fsp_err_t fsp_err = R_ADC_Open(&adc_ctrl, &adc_cfg);
    if (fsp_err != FSP_SUCCESS) {
        Serial.print("R_ADC_Open: ");
        Serial.println(fsp_err);
        return BeginStatus::FAIL;
    }

    int32_t adc_pin_idx = digitalPinToAnalogPin(adc_pin_number);
    bsp_pin = digitalPinToBspPin(adc_pin_idx);

    adc_window_cfg_t  adc_window_cfg = {};
    adc_channel_cfg_t adc_ch_cfg     = {};
    adc_ch_cfg.scan_mask         = (1U << bsp_pin);
    adc_ch_cfg.scan_mask_group_b = 0;
    adc_ch_cfg.add_mask          = 0;
    adc_ch_cfg.p_window_cfg      = &adc_window_cfg;
    adc_ch_cfg.priority_group_a  = ADC_GROUP_A_PRIORITY_OFF;
    adc_ch_cfg.sample_hold_mask  = 0;
    adc_ch_cfg.sample_hold_states = 0;
    R_ADC_ScanCfg(&adc_ctrl, &adc_ch_cfg);

    elc_cfg_t elc_cfg = {ELC_EVENT_NONE};
    R_ELC_Open(&elc_ctrl, &elc_cfg);
    R_ELC_Enable(&elc_ctrl);
    R_ELC_LinkSet(&g_elc_ctrl, ELC_PERIPHERAL_ADC0, timer_event);

    return BeginStatus::SUCCESS;
}

MicDriver::BeginStatus MicDriver::init_dma() {
    transfer_info_t      dmac_info       = {};
    dmac_extended_cfg_t  dmac_extend_cfg = {};
    transfer_cfg_t       dmac_cfg        = {&dmac_info, &dmac_extend_cfg};

    dmac_extend_cfg.activation_source = ELC_EVENT_ADC0_SCAN_END;
    dmac_extend_cfg.p_callback        = nullptr;
    dmac_extend_cfg.p_context         = nullptr;
    dmac_extend_cfg.channel           = dmac_channel;
    dmac_extend_cfg.offset            = 1;
    dmac_extend_cfg.src_buffer_size   = 1;
    dmac_extend_cfg.irq               = FSP_INVALID_VECTOR;
    dmac_extend_cfg.ipl               = 12;

    dmac_info.transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED;
    dmac_info.transfer_settings_word_b.src_addr_mode  = TRANSFER_ADDR_MODE_FIXED;
    dmac_info.transfer_settings_word_b.mode           = TRANSFER_MODE_REPEAT;
    dmac_info.transfer_settings_word_b.size           = TRANSFER_SIZE_2_BYTE;
    dmac_info.transfer_settings_word_b.repeat_area    = TRANSFER_REPEAT_AREA_DESTINATION;
    dmac_info.transfer_settings_word_b.irq            = TRANSFER_IRQ_END;
    dmac_info.transfer_settings_word_b.chain_mode     = TRANSFER_CHAIN_MODE_DISABLED;
    dmac_info.p_src    = (void*)&R_ADC0->ADDR[bsp_pin];
    dmac_info.p_dest   = (void*)&adc_dma_buffer[0];
    dmac_info.length   = ADC_DMA_LEN;
    dmac_info.num_blocks = 0;

    R_DMAC_Open(&dmac_ctrl, &dmac_cfg);
    // Do NOT enable here — start() handles that
    timer.open();

    return BeginStatus::SUCCESS;
}

MicDriver::BeginStatus MicDriver::init_timer() {
    uint8_t timer_type;
    int8_t timer_index = FspTimer::get_available_timer(timer_type);
    if (timer_index < 0) timer_index = FspTimer::get_available_timer(timer_type, true);
    if (timer_index < 0) return BeginStatus::FAIL_CANNOT_GET_TIMER;

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
            default: return BeginStatus::FAIL_TIMER_INDEX_OUT_OF_RANGE;
        }
    } else if (timer_type == AGT_TIMER) {
        switch (timer_index) {
            case 0: timer_event = ELC_EVENT_AGT0_INT; break;
            case 1: timer_event = ELC_EVENT_AGT1_INT; break;
            default: return BeginStatus::FAIL_TIMER_INDEX_OUT_OF_RANGE;
        }
    }

    timer.begin(TIMER_MODE_PERIODIC, timer_type, timer_index, freq_hz, 50.0f,
                MicDriver::timer_callback, this);
    timer.setup_overflow_irq(1);

    return BeginStatus::SUCCESS;
}

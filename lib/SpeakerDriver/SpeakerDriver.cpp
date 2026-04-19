#include "SpeakerDriver.h"

#include <FspTimer.h>
#include <dac.h>

SpeakerDriver::SpeakerDriver(float freq_hz, pin_size_t dac_pin_number,
                             uint32_t dmac_channel)
    : freq_hz(freq_hz),
      dac_pin_number(dac_pin_number),
      dmac_channel(dmac_channel) {}

SpeakerDriver::~SpeakerDriver() {}

SpeakerDriver::BeginStatus SpeakerDriver::begin() {
    SpeakerDriver::BeginStatus rv = init_dac();
    if (rv != SpeakerDriver::BeginStatus::SUCCESS) {
        return rv;
    }

    rv = init_timer();
    if (rv != SpeakerDriver::BeginStatus::SUCCESS) {
        return rv;
    }

    rv = init_dma();
    return rv;
}

SpeakerDriver::BufferPtr SpeakerDriver::get_buffers() { return buffers; }

int SpeakerDriver::reserve_buffer() {
    int buffer_num = read_buffer_num;
    for (int i = 0; i < NUM_BUFFERS; i++) {
        if (!buffer_populated[buffer_num]) {
            return buffer_num;
        }
        buffer_num++;
        buffer_num = buffer_num & NUM_BUFFERS_MASK;
    }
    return -1;
}

void SpeakerDriver::release_buffer(int buffer_num, bool populated) {
    buffer_populated[buffer_num] = populated;
}

void SpeakerDriver::on_timer(timer_callback_args_t* args) {
    if (buffer_populated[read_buffer_num]) {
        dma_buffer[dma_buffer_write_pos++] =
            (buffers[read_buffer_num][read_buffer_pos++]
             << 4);  // convert 8-bit sample -> 12-bit

        if (read_buffer_pos == BUFFER_LEN) {
            buffer_populated[read_buffer_num] = false;

            read_buffer_pos = 0;
            read_buffer_num++;
            read_buffer_num = read_buffer_num & NUM_BUFFERS_MASK;
        }
    } else {
        dma_buffer[dma_buffer_write_pos++] = 0;
    }

    dma_buffer_write_pos = dma_buffer_write_pos & DMA_BUFFER_LEN_MASK;
}

SpeakerDriver::BeginStatus SpeakerDriver::init_dac() {
    if (!IS_DAC(dac_pin_number)) {
        return SpeakerDriver::BeginStatus::FAIL_NOT_A_DAC_PIN;
    }

    auto cfg_dac = getPinCfgs(dac_pin_number, PIN_CFG_REQ_DAC);
    uint32_t dac_pin = cfg_dac[0];
    if (IS_DAC_8BIT(dac_pin)) {
        return SpeakerDriver::BeginStatus::FAIL_IS_8_BIT_DAC;
    }

    if (GET_CHANNEL(dac_pin) >= DAC12_HOWMANY) {
        return SpeakerDriver::BeginStatus::FAIL_DAC_CHANNEL_OUT_OF_RANGE;
    }

    if (GET_CHANNEL(dac_pin) == 0) {
        dac_address = (void*)DAC_ADDRESS_12_CH0;
    } else {
        return SpeakerDriver::BeginStatus::FAIL_DAC_CHANNEL_NOT_SUPPORTED;
    }

    _dac12[GET_CHANNEL(dac_pin)].init();
    analogWriteResolution(12);

    return SpeakerDriver::BeginStatus::SUCCESS;
}

SpeakerDriver::BeginStatus SpeakerDriver::init_timer() {
    uint8_t timer_type;
    int8_t timer_index = FspTimer::get_available_timer(timer_type);

    if (timer_index < 0) {
        timer_index = FspTimer::get_available_timer(timer_type, true);
    }
    if (timer_index < 0) {
        return SpeakerDriver::BeginStatus::FAIL_CANNOT_GET_TIMER;
    }

    if (timer_type == GPT_TIMER) {
        switch (timer_index) {
            case 0:
                timer_event = ELC_EVENT_GPT0_COUNTER_OVERFLOW;
                break;
            case 1:
                timer_event = ELC_EVENT_GPT1_COUNTER_OVERFLOW;
                break;
            case 2:
                timer_event = ELC_EVENT_GPT2_COUNTER_OVERFLOW;
                break;
            case 3:
                timer_event = ELC_EVENT_GPT3_COUNTER_OVERFLOW;
                break;
            case 4:
                timer_event = ELC_EVENT_GPT4_COUNTER_OVERFLOW;
                break;
            case 5:
                timer_event = ELC_EVENT_GPT5_COUNTER_OVERFLOW;
                break;
            case 6:
                timer_event = ELC_EVENT_GPT6_COUNTER_OVERFLOW;
                break;
            case 7:
                timer_event = ELC_EVENT_GPT7_COUNTER_OVERFLOW;
                break;
            default:
                return SpeakerDriver::BeginStatus::
                    FAIL_TIMER_INDEX_OUT_OF_RANGE;
        }
    } else if (timer_type == AGT_TIMER) {
        switch (timer_index) {
            case 0:
                timer_event = ELC_EVENT_AGT0_INT;
                break;
            case 1:
                timer_event = ELC_EVENT_AGT1_INT;
                break;
            default:
                return SpeakerDriver::BeginStatus::
                    FAIL_TIMER_INDEX_OUT_OF_RANGE;
        }
    }

    timer.begin(TIMER_MODE_PERIODIC, timer_type, timer_index, freq_hz, 50.0f,
                SpeakerDriver::timer_callback, this);
    timer.setup_overflow_irq(1);

    return SpeakerDriver::BeginStatus::SUCCESS;
}

SpeakerDriver::BeginStatus SpeakerDriver::init_dma() {
    transfer_info_t dmac_info = {};
    dmac_extended_cfg_t dmac_extend_cfg = {};
    transfer_cfg_t dmac_cfg = {&dmac_info, &dmac_extend_cfg};

    dmac_extend_cfg.activation_source = timer_event;
    dmac_extend_cfg.p_callback = nullptr;
    dmac_extend_cfg.p_context = nullptr;
    dmac_extend_cfg.channel = dmac_channel;
    dmac_extend_cfg.offset = 1;
    dmac_extend_cfg.src_buffer_size = 1;
    dmac_extend_cfg.irq = FSP_INVALID_VECTOR;
    dmac_extend_cfg.ipl = 12;

    dmac_info.transfer_settings_word_b.dest_addr_mode =
        TRANSFER_ADDR_MODE_FIXED;
    dmac_info.transfer_settings_word_b.src_addr_mode =
        TRANSFER_ADDR_MODE_INCREMENTED;
    dmac_info.transfer_settings_word_b.mode = TRANSFER_MODE_REPEAT;
    dmac_info.transfer_settings_word_b.size = TRANSFER_SIZE_2_BYTE;
    dmac_info.transfer_settings_word_b.repeat_area =
        TRANSFER_REPEAT_AREA_SOURCE;
    dmac_info.transfer_settings_word_b.irq = TRANSFER_IRQ_END;
    dmac_info.transfer_settings_word_b.chain_mode =
        TRANSFER_CHAIN_MODE_DISABLED;
    dmac_info.p_src = (void*)&dma_buffer[0];
    dmac_info.p_dest = dac_address;
    dmac_info.length = DMA_BUFFER_LEN;
    dmac_info.num_blocks = 0;

    R_DMAC_Open(&dmac_ctrl, &dmac_cfg);
    R_DMAC_Enable(&dmac_ctrl);

    timer.open();
    timer.start();

    return SpeakerDriver::BeginStatus::SUCCESS;
}

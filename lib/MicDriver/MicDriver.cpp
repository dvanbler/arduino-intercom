#include "MicDriver.h"

#include <FspTimer.h>
#include <WiFiS3.h>
#include <dac.h>

MicDriver::MicDriver(float freq_hz, pin_size_t dac_pin_number)
    : freq_hz(freq_hz), dac_pin_number(dac_pin_number) {}

MicDriver::~MicDriver() {}

MicDriver::BeginStatus MicDriver::begin() {
    MicDriver::BeginStatus rv = init_dac();
    if (rv != MicDriver::BeginStatus::SUCCESS) {
        return rv;
    }

    rv = init_timer();
    if (rv != MicDriver::BeginStatus::SUCCESS) {
        return rv;
    }

    rv = init_dma();
    return rv;
}

void MicDriver::end() {
    timer.stop();
    timer.close();
    R_DMAC_Disable(&dma_ctrl);
    R_DMAC_Close(&dma_ctrl);
    timer.end();
}

MicDriver::BufferPtr MicDriver::get_buffers() {
    return buffers;
}

int MicDriver::reserve_buffer() {
    int buffer_num = read_buffer_num;
    for (int i = 0; i < NUM_BUFFERS; i++) {
        if (!buffer_populated[buffer_num]) {
            return buffer_num;
        }
        buffer_num++;
        if (buffer_num == NUM_BUFFERS) {
            buffer_num = 0;
        }
    }
    return -1;
}

void MicDriver::release_buffer(int buffer_num, bool populated) {
    buffer_populated[buffer_num] = populated;
}

void MicDriver::on_timer(timer_callback_args_t* args) {
    if (buffer_populated[read_buffer_num]) {
        dma_buffer[dma_buffer_write_pos++] = (buffers[read_buffer_num][read_buffer_pos++] << 4); // convert 8-bit sample -> 12-bit
        
        if (read_buffer_pos == BUFFER_LEN) {
            buffer_populated[read_buffer_num] = false;

            read_buffer_pos = 0;
            read_buffer_num++;
            if (read_buffer_num == NUM_BUFFERS) {
                read_buffer_num = 0;
            }
        }
    } else {
        dma_buffer[dma_buffer_write_pos++] = 0;
        no_data_events++;
    }

    if (dma_buffer_write_pos == DMA_BUFFER_LEN) {
        dma_buffer_write_pos = 0;
    }
}

MicDriver::BeginStatus MicDriver::init_dac() {
    if (!IS_DAC(dac_pin_number)) {
        return MicDriver::BeginStatus::FAIL_NOT_A_DAC_PIN;
    }

    auto cfg_dac = getPinCfgs(dac_pin_number, PIN_CFG_REQ_DAC);
    uint32_t dac_pin = cfg_dac[0];
    if (IS_DAC_8BIT(dac_pin)) {
        return MicDriver::BeginStatus::FAIL_IS_8_BIT_DAC;
    }

    if (GET_CHANNEL(dac_pin) >= DAC12_HOWMANY) {
        return MicDriver::BeginStatus::FAIL_DAC_CHANNEL_OUT_OF_RANGE;
    }

    if (GET_CHANNEL(dac_pin) == 0) {
        dac_address = (void*)DAC_ADDRESS_12_CH0;
    } else {
        return MicDriver::BeginStatus::FAIL_DAC_CHANNEL_NOT_SUPPORTED;
    }

    _dac12[GET_CHANNEL(dac_pin)].init();
    analogWriteResolution(12);

    return MicDriver::BeginStatus::SUCCESS;
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
                return MicDriver::BeginStatus::
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
                return MicDriver::BeginStatus::
                    FAIL_TIMER_INDEX_OUT_OF_RANGE;
        }
    }

    timer.begin(TIMER_MODE_PERIODIC, timer_type, timer_index, freq_hz, 50.0f,
                MicDriver::timer_callback, this);
    timer.setup_overflow_irq(1);

    return MicDriver::BeginStatus::SUCCESS;
}

MicDriver::BeginStatus MicDriver::init_dma() {
    dma_info.transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED;
    dma_info.transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_SOURCE;
    dma_info.transfer_settings_word_b.irq = TRANSFER_IRQ_END;
    dma_info.transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_DISABLED;
    dma_info.transfer_settings_word_b.src_addr_mode =
        TRANSFER_ADDR_MODE_INCREMENTED;
    dma_info.transfer_settings_word_b.size = TRANSFER_SIZE_2_BYTE;
    dma_info.transfer_settings_word_b.mode = TRANSFER_MODE_REPEAT;
    dma_info.p_dest = dac_address;
    dma_info.p_src = dma_buffer;
    dma_info.num_blocks = 0;
    dma_info.length = DMA_BUFFER_LEN;

    dma_extend_cfg.offset =
        1;  // offset size if using TRANSFER_ADDR_MODE_OFFSET
    dma_extend_cfg.src_buffer_size = 1;       //  used for repeat - block mode
    dma_extend_cfg.irq = FSP_INVALID_VECTOR;  //  IRQManager will set this
    dma_extend_cfg.ipl = BSP_IRQ_DISABLED;    //  IRQManager will set this
    dma_extend_cfg.channel = 0;               //  IRQManager will set this
    dma_extend_cfg.p_context =
        NULL;  // void* pointer to anything will be available in callback
    dma_extend_cfg.activation_source = ELC_EVENT_GPT4_COUNTER_OVERFLOW;

    R_DMAC_Open(&dma_ctrl, &dma_cfg);
    R_DMAC_Enable(&dma_ctrl);

    timer.open();
    timer.start();

    return MicDriver::BeginStatus::SUCCESS;
}

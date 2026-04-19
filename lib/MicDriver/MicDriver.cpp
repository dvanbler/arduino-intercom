#include "MicDriver.h"

#include <FspTimer.h>
#include <r_adc.h>
#include <r_dmac.h>
#include <r_elc.h>

MicDriver::MicDriver(float freq_hz, pin_size_t adc_pin_number,
                     uint32_t dmac_channel)
    : freq_hz(freq_hz),
      adc_pin_number(adc_pin_number),
      dmac_channel(dmac_channel) {}

MicDriver::~MicDriver() {}

MicDriver::BeginStatus MicDriver::begin() {
    /*
     Configures:
       timer at freqHz ->
       overflow event ->
       elc ->
       adc0 ->
       triggers read on A1 ->
       scan end event ->
       DMA trigger

     Result: DMA buffer gets filled at specified freqHz by analog reads on A1
    */

    // Timer config
    auto rv = init_timer();
    if (rv != MicDriver::BeginStatus::SUCCESS) {
        return rv;
    }

    // ADC config
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

    // Must convert the arduino pin to the channel, which is not necessarily the
    // same (i.e. A0 -> channel 14, A1 -> channel 0)
    int32_t adc_pin_idx = digitalPinToAnalogPin(adc_pin_number);
    pin_size_t bsp_pin = digitalPinToBspPin(adc_pin_idx);

    // Enable scanning on only the specified input pin
    adc_window_cfg_t adc_window_cfg = {};
    adc_channel_cfg_t adc_ch_cfg = {};
    adc_ch_cfg.scan_mask = (1U << bsp_pin);  // See ADC_MASK_CHANNEL_0;
    adc_ch_cfg.scan_mask_group_b = 0;
    adc_ch_cfg.add_mask = 0;
    adc_ch_cfg.p_window_cfg = &adc_window_cfg;
    adc_ch_cfg.priority_group_a = ADC_GROUP_A_PRIORITY_OFF;
    adc_ch_cfg.sample_hold_mask = 0;
    adc_ch_cfg.sample_hold_states = 0;
    R_ADC_ScanCfg(&adc_ctrl, &adc_ch_cfg);

    // ELC config. Link timer overflow to ADC scan
    elc_cfg_t elc_cfg = {ELC_EVENT_NONE};
    fsp_err = R_ELC_Open(&elc_ctrl, &elc_cfg);
    fsp_err = R_ELC_Enable(&elc_ctrl);
    fsp_err = R_ELC_LinkSet(&g_elc_ctrl, ELC_PERIPHERAL_ADC0, timer_event);

    // DMAC config. Read from ADC -> ring buffer, triggered by ADC scan end.
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
    dmac_info.p_src =
        (void*)&R_ADC0->ADDR[bsp_pin];  // (void*)&R_ADC0->ADDR[0];
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

void MicDriver::on_timer(timer_callback_args_t* args) { callback_count++; }

void MicDriver::print_debug() {
    // value range (volts): 1.25 +- 1.0 = 0.25 - 2.25
    // pct range: (.25 / 5) - (2.25 / 5) = .05 - .45
    // 14-bit range = 819 - 7373

    transfer_properties_t props = {};
    R_DMAC_InfoGet(&dmac_ctrl, &props);
    uint16_t writePos = DMA_BUFFER_LEN - props.transfer_length_remaining;

    Serial.print("dma_buffer[");
    Serial.print(writePos);
    Serial.print("]=");
    Serial.println(dma_buffer[writePos]);
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
                return MicDriver::BeginStatus::FAIL_TIMER_INDEX_OUT_OF_RANGE;
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
                return MicDriver::BeginStatus::FAIL_TIMER_INDEX_OUT_OF_RANGE;
        }
    }

    timer.begin(TIMER_MODE_PERIODIC, timer_type, timer_index, freq_hz, 50.0f,
                MicDriver::timer_callback, this);
    timer.setup_overflow_irq(1);

    return MicDriver::BeginStatus::SUCCESS;
}

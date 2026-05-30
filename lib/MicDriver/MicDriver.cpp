#include "MicDriver.h"

#include <FspTimer.h>
#include <analog.h>
#include <r_adc.h>

#include <algorithm>

#include "CircularBuffer.h"

MicDriver::MicDriver(float freq_hz, pin_size_t adc_pin_number,
                     CircularBuffer& buffer)
    : freq_hz(freq_hz), adc_pin_number(adc_pin_number), buffer(buffer) {}

MicDriver::InitStatus MicDriver::init() {
    auto rv = init_timer();
    if (rv != InitStatus::SUCCESS) return rv;

    return init_adc();
}

void MicDriver::start() {
    noInterrupts();
    buffer.reset();
    R_ADC_ScanStart(&adc->ctrl);
    timer.start();
    interrupts();
}

void MicDriver::stop() {
    noInterrupts();
    timer.stop();
    R_ADC_ScanStop(&adc->ctrl);
    interrupts();
}

int MicDriver::read(uint8_t* samples, int len) {
    if (buffer.available() < len) return 0;
    return buffer.consume(samples, len);
}

MicDriver::InitStatus MicDriver::init_adc() {
    // ADC_Container pre-fills cfg, cfg_extend, and channel_cfg with sensible
    // defaults and wires our callback + context. Resolution is forced to the
    // hardware max (14-bit on the RA4M1), matching our scaling constants.
    adc = new ADC_Container(0, MicDriver::adc_callback, this);

    // The container defaults trigger to ADC_TRIGGER_SOFTWARE — we must switch
    // to ELC sync so the timer overflow event drives each conversion.
    adc->cfg.trigger = ADC_TRIGGER_SOFTWARE;

    // Must run before R_ADC_Open: fills cfg.scan_end_irq so Open can enable
    // ADIE and configure the NVIC. Calling it after Open leaves ADIE=0 and the
    // NVIC disabled, so the scan-end ISR never fires even when scans complete.
    if (!IRQManager::getInstance().addADCScanEnd(adc)) {
        Serial.println("addADCScanEnd failed");
        return InitStatus::FAIL;
    }

    fsp_err_t fsp_err = R_ADC_Open(&adc->ctrl, &adc->cfg);
    if (fsp_err != FSP_SUCCESS) {
        Serial.print("R_ADC_Open: ");
        Serial.println(fsp_err);
        return InitStatus::FAIL;
    }

    // Resolve the Arduino analog pin to its ADC channel (bsp_pin)
    int32_t adc_pin_idx = digitalPinToAnalogPin(adc_pin_number);
    bsp_pin = digitalPinToBspPin(adc_pin_idx);

    // Scan only our single input channel
    adc->channel_cfg.scan_mask = (1U << bsp_pin);
    adc->channel_cfg.scan_mask_group_b = 0;
    adc->channel_cfg.add_mask = 0;
    adc->channel_cfg.p_window_cfg = nullptr;
    adc->channel_cfg.priority_group_a = ADC_GROUP_A_PRIORITY_OFF;
    adc->channel_cfg.sample_hold_mask = 0;
    adc->channel_cfg.sample_hold_states = 0;

    fsp_err = R_ADC_ScanCfg(&adc->ctrl, &adc->channel_cfg);
    if (fsp_err != FSP_SUCCESS) {
        Serial.print("R_ADC_ScanCfg: ");
        Serial.println(fsp_err);
        return InitStatus::FAIL;
    }

    return InitStatus::SUCCESS;
}

MicDriver::InitStatus MicDriver::init_timer() {
    uint8_t timer_type;
    int8_t timer_index = FspTimer::get_available_timer(timer_type);
    if (timer_index < 0)
        timer_index = FspTimer::get_available_timer(timer_type, true);
    if (timer_index < 0) return InitStatus::FAIL_CANNOT_GET_TIMER;

    if (timer_type == GPT_TIMER) {
        if (timer_index > 7) return InitStatus::FAIL_TIMER_INDEX_OUT_OF_RANGE;
    } else if (timer_type == AGT_TIMER) {
        if (timer_index > 1) return InitStatus::FAIL_TIMER_INDEX_OUT_OF_RANGE;
    }

    timer.begin(TIMER_MODE_PERIODIC, timer_type, timer_index, freq_hz, 50.0f,
                timer_callback, this);
    timer.setup_overflow_irq(1);
    timer.open();

    return InitStatus::SUCCESS;
}

void MicDriver::on_timer(timer_callback_args_t* args) {
    R_ADC_ScanStart(&adc->ctrl);
};

void MicDriver::on_adc_complete(adc_callback_args_t* args) {
    constexpr int32_t IN_MIN = 819;
    constexpr int32_t IN_MAX = 7373;
    constexpr int32_t OUT_MIN = 0;
    constexpr int32_t OUT_MAX = 255;
    constexpr int32_t SCALE_FIXED =
        (int32_t)((OUT_MAX - OUT_MIN) * 16384.0f / (IN_MAX - IN_MIN));

    uint16_t raw = R_ADC0->ADDR[bsp_pin];
    int32_t result = (raw - IN_MIN) * SCALE_FIXED >> 14;
    uint8_t sample = (uint8_t)std::clamp(result, OUT_MIN, OUT_MAX);
    buffer.produce(sample);
}
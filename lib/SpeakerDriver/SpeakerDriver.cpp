#include "SpeakerDriver.h"

#include <FspTimer.h>
#include <dac.h>

#include "CircularBuffer.h"

SpeakerDriver::SpeakerDriver(float freq_hz, pin_size_t dac_pin_number,
                             CircularBuffer& buffer)
    : freq_hz(freq_hz), dac_pin_number(dac_pin_number), buffer(buffer) {}

SpeakerDriver::InitStatus SpeakerDriver::init() {
    auto rv = init_dac();
    if (rv != InitStatus::SUCCESS) return rv;

    return init_timer();
}

SpeakerDriver::InitStatus SpeakerDriver::init_dac() {
    if (!IS_DAC(dac_pin_number)) {
        return InitStatus::FAIL_NOT_A_DAC_PIN;
    }

    auto cfg_dac = getPinCfgs(dac_pin_number, PIN_CFG_REQ_DAC);
    uint32_t dac_pin = cfg_dac[0];
    if (IS_DAC_8BIT(dac_pin)) {
        return InitStatus::FAIL_IS_8_BIT_DAC;
    }
    if (GET_CHANNEL(dac_pin) >= DAC12_HOWMANY) {
        return InitStatus::FAIL_DAC_CHANNEL_OUT_OF_RANGE;
    }
    if (GET_CHANNEL(dac_pin) != 0) {
        return InitStatus::FAIL_DAC_CHANNEL_NOT_SUPPORTED;
    }

    dac_address = &R_DAC->DADR[0];

    _dac12[GET_CHANNEL(dac_pin)].init();
    analogWriteResolution(12);

    return InitStatus::SUCCESS;
}

SpeakerDriver::InitStatus SpeakerDriver::init_timer() {
    uint8_t timer_type;
    int8_t timer_index = FspTimer::get_available_timer(timer_type);
    if (timer_index < 0)
        timer_index = FspTimer::get_available_timer(timer_type, true);
    if (timer_index < 0) return InitStatus::FAIL_CANNOT_GET_TIMER;

    timer.begin(TIMER_MODE_PERIODIC, timer_type, timer_index, freq_hz, 50.0f,
                timer_callback, this);
    timer.setup_overflow_irq(1);
    timer.open();

    return InitStatus::SUCCESS;
}

void SpeakerDriver::start() {
    noInterrupts();
    last_value = 0;
    buffer.reset();
    status = Status::STARTING;
    interrupts();
    timer.start();
}

void SpeakerDriver::stop() {
    noInterrupts();
    if (status == Status::STARTING || status == Status::PLAYING) {
        fade_out_diff = max(1, last_value / FADE_OUT_STEPS);
        status = Status::STOPPING;
    }
    bool need_wait = (status == Status::STOPPING);
    interrupts();

    if (need_wait) {
        // Wait for the ISR to fade out and reach STOPPED.
        while (status != Status::STOPPED) {
        }
    }

    timer.stop();
}

int SpeakerDriver::play(const uint8_t* samples, int len) {
    return buffer.produce(samples, len);
}

void SpeakerDriver::on_timer_overflow() {
    if (status == Status::PLAYING) {
        int b = buffer.consume();
        if (b != -1) {
            // Buffer has more data - output it
            last_value = (uint16_t)(b << 4);
            *dac_address = last_value;
        } else {
            // Buffer is empty - transition to starting
            status = Status::STARTING;
            fade_out_diff = last_value / FADE_OUT_STEPS;
        }
    } else if (status == Status::STARTING) {
        // Starting - wait until we have enough buffered data before we start
        // playing
        if (buffer.available() >= REQUIRED_BUFFERED_TO_START) {
            status = Status::PLAYING;
        } else {
            // Fade out the last sample we played -> 0 to avoid popping
            last_value -= fade_out_diff;
            if (last_value < 0) {
                last_value = 0;
            }
            *dac_address = last_value;
        }
    } else if (status == Status::STOPPING) {
        // Stopping - fade out
        last_value -= fade_out_diff;
        if (last_value < 0) {
            last_value = 0;
            status = Status::STOPPED;
        }
        *dac_address = last_value;
    } else if (status == Status::STOPPED) {
        *dac_address = 0;
    }
}
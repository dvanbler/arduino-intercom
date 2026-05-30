#pragma once

#include <FspTimer.h>

#include "CircularBuffer.h"

class SpeakerDriver {
   public:
    enum class InitStatus {
        SUCCESS = 0,
        FAIL_NOT_A_DAC_PIN = -1,
        FAIL_IS_8_BIT_DAC = -2,
        FAIL_DAC_CHANNEL_OUT_OF_RANGE = -3,
        FAIL_DAC_CHANNEL_NOT_SUPPORTED = -4,
        FAIL_CANNOT_GET_TIMER = -5,
        FAIL_TIMER_INDEX_OUT_OF_RANGE = -6
    };

    enum class Status { STOPPED = 0, STARTING = 1, PLAYING = 2, STOPPING = 3 };

   private:
    static constexpr int REQUIRED_BUFFERED_TO_START =
        CircularBuffer::BUFFER_LEN / 2;
    static constexpr int FADE_OUT_STEPS = 160;

    const float freq_hz;
    const pin_size_t dac_pin_number;
    CircularBuffer& buffer;

    volatile uint16_t* dac_address;

    FspTimer timer;

    volatile Status status = Status::STOPPED;
    volatile int last_value = 0;
    volatile int fade_out_diff;

   public:
    explicit SpeakerDriver(float freq_hz, pin_size_t dac_pin_number,
                           CircularBuffer& buffer);
    ~SpeakerDriver() {}

    // One-time hardware init
    InitStatus init();

    // Starts processing buffer for speaker output
    void start();

    // Stops processing buffer for speaker output - call when switching to mic
    // mode
    void stop();

    // Inserts samples into the buffer
    int play(const uint8_t* samples, int len);

   private:
    InitStatus init_dac();
    InitStatus init_timer();

    void on_timer_overflow();

    static void timer_callback(timer_callback_args_t* args) {
        SpeakerDriver* self = (SpeakerDriver*)(args->p_context);
        self->on_timer_overflow();
    }
};

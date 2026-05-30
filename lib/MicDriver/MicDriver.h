#pragma once

#include <FspTimer.h>
#include <analog.h>
#include <r_adc.h>
#include <r_dmac.h>
#include <r_elc.h>

#include "CircularBuffer.h"

class MicDriver {
   public:
    enum class InitStatus {
        SUCCESS = 0,
        FAIL = -1,
        FAIL_CANNOT_GET_TIMER = -5,
        FAIL_TIMER_INDEX_OUT_OF_RANGE = -6
    };

    volatile uint32_t timer_count = 0;
    volatile uint16_t last_raw = 0;
    volatile uint32_t adc_count = 0;

    elc_event_t timer_event;

   private:
    const float freq_hz;
    const pin_size_t adc_pin_number;
    CircularBuffer& buffer;

    ADC_Container* adc = nullptr;
    FspTimer timer;

    pin_size_t bsp_pin = 0;  // stored during begin() for ADC address

   public:
    explicit MicDriver(float freq_hz, pin_size_t adc_pin_number,
                       CircularBuffer& buffer);
    ~MicDriver() {}

    InitStatus init();

    void start();

    void stop();

    int read(uint8_t* samples, int len);

   private:
    InitStatus init_timer();
    InitStatus init_adc();

    static void timer_callback(timer_callback_args_t* args) {
        MicDriver* self = (MicDriver*)(args->p_context);
        self->on_timer(args);
    }

    static void adc_callback(adc_callback_args_t* args) {
        MicDriver* self = (MicDriver*)(args->p_context);
        self->on_adc_complete(args);
    }

    void on_adc_complete(adc_callback_args_t* args);
    void on_timer(timer_callback_args_t* args);
};

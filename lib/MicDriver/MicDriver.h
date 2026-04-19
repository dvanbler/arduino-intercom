#pragma once

#include <r_adc.h>
#include <r_dmac.h>
#include <r_elc.h>

#include <FspTimer.h>

class MicDriver {
   public:
    explicit MicDriver(float freq_hz, pin_size_t adc_pin_number);
    ~MicDriver();

    static constexpr int DMA_BUFFER_LEN_BITS = 9;
    static constexpr int DMA_BUFFER_LEN = 1 << DMA_BUFFER_LEN_BITS;
    static constexpr int DMA_BUFFER_LEN_MASK = DMA_BUFFER_LEN - 1;

    enum class BeginStatus {
        SUCCESS = 0,
        FAIL = -1,
        FAIL_CANNOT_GET_TIMER = -5,
        FAIL_TIMER_INDEX_OUT_OF_RANGE = -6
    };

    BeginStatus begin();

    void end();

    void print_debug();

   private:
    float freq_hz;
    const pin_size_t adc_pin_number;

    volatile unsigned long callback_count = 0;

    adc_instance_ctrl_t adc_ctrl = {};
    dmac_instance_ctrl_t dmac_ctrl = {};
    elc_instance_ctrl_t elc_ctrl = {};

    FspTimer timer;
    elc_event_t timer_event;

    // memory that will be read by DMA, written to by timer_callback
    uint16_t dma_buffer[DMA_BUFFER_LEN] __attribute__((aligned(4))) = {};

    MicDriver::BeginStatus init_timer();

    // callback to handle moving data from the dma_buffer to sample buffers
    void on_timer(timer_callback_args_t* args);

    static void timer_callback(timer_callback_args_t* args) {
        if (args->event != TIMER_EVENT_CYCLE_END) {
            return;
        }
        MicDriver* self = (MicDriver*)(args->p_context);
        self->on_timer(args);
    }
};

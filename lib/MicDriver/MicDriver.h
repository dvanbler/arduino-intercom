#pragma once

#include <FspTimer.h>
#include <r_adc.h>
#include <r_dmac.h>
#include <r_elc.h>

class MicDriver {
   public:
    static constexpr int DMA_BUFFER_LEN_BITS = 9;
    static constexpr int DMA_BUFFER_LEN = 1 << DMA_BUFFER_LEN_BITS;
    static constexpr int DMA_BUFFER_LEN_MASK = DMA_BUFFER_LEN - 1;

    static constexpr int BUFFER_LEN = 548;

    static constexpr int RING_BUFFER_LEN_BITS = 13;
    static constexpr int RING_BUFFER_LEN = 1 << RING_BUFFER_LEN_BITS;
    static constexpr int RING_BUFFER_LEN_MASK = RING_BUFFER_LEN - 1;

    enum class BeginStatus {
        SUCCESS = 0,
        FAIL = -1,
        FAIL_CANNOT_GET_TIMER = -5,
        FAIL_TIMER_INDEX_OUT_OF_RANGE = -6
    };

   private:
    const float freq_hz;
    const pin_size_t adc_pin_number;
    const uint32_t dmac_channel;

    adc_instance_ctrl_t adc_ctrl = {};
    dmac_instance_ctrl_t dmac_ctrl = {};
    elc_instance_ctrl_t elc_ctrl = {};

    FspTimer timer;
    elc_event_t timer_event;

    // Memory written by DMA, read by timer callback
    uint16_t dma_buffer[DMA_BUFFER_LEN] __attribute__((aligned(4))) = {};

    // Current read position in the DMA buffer
    volatile int dma_buffer_read_pos = DMA_BUFFER_LEN / 2;

    // Ring buffer — writer always advances, reader catches up
    uint8_t ring_buffer[RING_BUFFER_LEN] = {};

    // Write pos advanced by timer ISR, read pos advanced by consumer
    volatile int write_pos = 0;
    int read_pos = 0;

    // Scratch buffer handed to caller for a single packet read
    uint8_t read_buffer[BUFFER_LEN] = {};

   public:
    explicit MicDriver(float freq_hz, pin_size_t adc_pin_number,
                       uint32_t dmac_channel);
    ~MicDriver();

    BeginStatus begin();

    // Returns a pointer to a BUFFER_LEN packet of mic data, or nullptr
    // if not enough data is available yet.
    // The pointer is valid until the next call to read_packet().
    uint8_t* read_packet();

   private:
    BeginStatus init_timer();

    void on_timer(timer_callback_args_t* args);

    static void timer_callback(timer_callback_args_t* args) {
        if (args->event != TIMER_EVENT_CYCLE_END) {
            return;
        }
        MicDriver* self = (MicDriver*)(args->p_context);
        self->on_timer(args);
    }
};
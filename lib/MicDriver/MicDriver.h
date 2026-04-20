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

    static constexpr int NUM_BUFFERS_BITS = 3;
    static constexpr int NUM_BUFFERS = 1 << NUM_BUFFERS_BITS;
    static constexpr int NUM_BUFFERS_MASK = NUM_BUFFERS - 1;

    // pointer to array of buffers
    typedef uint8_t (*BufferPtr)[BUFFER_LEN];

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

    volatile unsigned long callback_count = 0;

    adc_instance_ctrl_t adc_ctrl = {};
    dmac_instance_ctrl_t dmac_ctrl = {};
    elc_instance_ctrl_t elc_ctrl = {};

    FspTimer timer;
    elc_event_t timer_event;

    // memory that will be written by DMA, read by timer_callback
    uint16_t dma_buffer[DMA_BUFFER_LEN] __attribute__((aligned(4))) = {};

    // current read position in the dma buffer
    volatile int dma_buffer_read_pos = DMA_BUFFER_LEN / 2;

    // buffers to hold pending samples
    uint8_t buffers[NUM_BUFFERS][BUFFER_LEN];

    // is the buffer ready to read from?
    volatile bool buffer_populated[NUM_BUFFERS] = {0};

    // current buffer we're writing to
    volatile int write_buffer_num = 0;

    // current buffer position in the current buffer
    volatile int write_buffer_pos = 0;

   public:
    explicit MicDriver(float freq_hz, pin_size_t adc_pin_number,
                       uint32_t dmac_channel);
    ~MicDriver();

    BeginStatus begin();

    BufferPtr get_buffers();

    int reserve_buffer_for_read();

    void release_buffer(int buffer_num, bool was_read);

    void print_debug();

   private:
    BeginStatus init_timer();

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

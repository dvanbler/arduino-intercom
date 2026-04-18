#pragma once

#include <FspTimer.h>
#include <dac.h>

class SpeakerDriver {
   public:
    explicit SpeakerDriver(float freq_hz, pin_size_t dac_pin_number);
    ~SpeakerDriver();

    static constexpr int DMA_BUFFER_LEN_BITS = 9; // buffer length must be a power of 2
    static constexpr int DMA_BUFFER_LEN = 1 << DMA_BUFFER_LEN_BITS;
    static constexpr int DMA_BUFFER_LEN_MASK = DMA_BUFFER_LEN - 1;

    static constexpr int BUFFER_LEN = 548;

    static constexpr int NUM_BUFFERS_BITS = 3; // buffer length must be a power of 2
    static constexpr int NUM_BUFFERS = 1 << NUM_BUFFERS_BITS;
    static constexpr int NUM_BUFFERS_MASK = NUM_BUFFERS - 1;

    // pointer to array of buffers
    typedef uint8_t (*BufferPtr)[SpeakerDriver::BUFFER_LEN];

    enum class BeginStatus {
        SUCCESS = 0,
        FAIL_NOT_A_DAC_PIN = -1,
        FAIL_IS_8_BIT_DAC = -2,
        FAIL_DAC_CHANNEL_OUT_OF_RANGE = -3,
        FAIL_DAC_CHANNEL_NOT_SUPPORTED = -4,
        FAIL_CANNOT_GET_TIMER = -5,
        FAIL_TIMER_INDEX_OUT_OF_RANGE = -6
    };

    BeginStatus begin();

    void end();

    BufferPtr get_buffers();

    // Gets the index of next buffer you can write to. This must only be called
    // once prior to calling release_buffer. Returns -1 if no buffer is
    // available yet.
    int reserve_buffer();

    // Indicates that the buffer is ready to be sent to the speaker.
    // Modifications must not be made to the buffer after this call.
    void release_buffer(int buffer_num, bool populated);

    volatile unsigned long no_data_events = 0;

   private:
    float freq_hz;
    const pin_size_t dac_pin_number;

    // memory that will be read by DMA, written to by timer_callback
    uint16_t dma_buffer[DMA_BUFFER_LEN] __attribute__((aligned(4))) = {};

    // current write position in the dma buffer
    volatile int dma_buffer_write_pos = DMA_BUFFER_LEN / 2;

    // buffers to hold pending samples
    uint8_t buffers[SpeakerDriver::NUM_BUFFERS][SpeakerDriver::BUFFER_LEN];

    // is the buffer ready to write to the dma buffer?
    volatile bool buffer_populated[NUM_BUFFERS] = {0};

    // current buffer we're reading from
    volatile int read_buffer_num = 0;

    // current read position in the current buffer
    volatile int read_buffer_pos = 0;

    // timer that will trigger a dma transfer, and write more data to the
    // dma buffer in lock-step
    FspTimer timer;

    // address of dac output register
    void* dac_address;

    // event that will trigger the dma transfer
    elc_event_t timer_event;

    dmac_instance_ctrl_t dma_ctrl;
    transfer_info_t dma_info;
    dmac_extended_cfg_t dma_extend_cfg;
    transfer_cfg_t dma_cfg = {&dma_info, &dma_extend_cfg};

    SpeakerDriver::BeginStatus init_dac();

    SpeakerDriver::BeginStatus init_timer();

    SpeakerDriver::BeginStatus init_dma();

    // callback to handle moving data from the sample buffers to the dma_buffer
    void on_timer(timer_callback_args_t* args);

    static void timer_callback(timer_callback_args_t* args) {
        if (args->event != TIMER_EVENT_CYCLE_END) {
            return;
        }
        SpeakerDriver* self = (SpeakerDriver*)(args->p_context);
        self->on_timer(args);
    }
};

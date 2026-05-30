#pragma once

#include <FspTimer.h>
#include <r_adc.h>
#include <r_dmac.h>
#include <r_elc.h>

class MicDriver {
   public:
    static constexpr int BUFFER_LEN   = 4384;   // 8 UDP packets * 548 bytes
    static constexpr int PACKET_LEN   = 548;
    static constexpr int PACKETS_PER_BUFFER = BUFFER_LEN / PACKET_LEN;

    // ADC DMA scratch buffer — 14-bit samples from hardware
    static constexpr int ADC_DMA_LEN_BITS = 9;
    static constexpr int ADC_DMA_LEN      = 1 << ADC_DMA_LEN_BITS;
    static constexpr int ADC_DMA_LEN_MASK = ADC_DMA_LEN - 1;

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

    adc_instance_ctrl_t  adc_ctrl  = {};
    dmac_instance_ctrl_t dmac_ctrl = {};
    elc_instance_ctrl_t  elc_ctrl  = {};
    FspTimer  timer;
    elc_event_t timer_event;

    // ADC DMA scratch buffer — raw 14-bit hardware values
    uint16_t adc_dma_buffer[ADC_DMA_LEN] __attribute__((aligned(4))) = {};
    volatile int adc_dma_read_pos = ADC_DMA_LEN / 2;

    // Triple buffers — shared with SpeakerDriver, passed in via constructor
    uint8_t* buf0;
    uint8_t* buf1;
    uint8_t* buf2;

    // Triple buffer roles
    uint8_t* dma_writing_buf = nullptr;    // DMA/timer ISR is filling this
    volatile uint8_t* staged_ready_buf = nullptr;  // full, waiting for loop() to send
    uint8_t* net_reading_buf = nullptr;    // loop() is currently sending this

    volatile int dma_write_packet_count = 0;  // packets filled in dma_writing_buf
    volatile int dma_write_sample_count = 0;  // samples written in current packet

    // Current packet being assembled within dma_writing_buf
    volatile int packet_sample_pos = 0;

    pin_size_t bsp_pin = 0;  // stored during begin() for ADC address

   public:
    explicit MicDriver(float freq_hz, pin_size_t adc_pin_number,
                       uint32_t dmac_channel,
                       uint8_t* buf0, uint8_t* buf1, uint8_t* buf2);
    ~MicDriver() {}

    // One-time hardware init — call in setup()
    BeginStatus begin();

    // Enable ADC + DMA + timer, reset buffer state
    void start();

    // Disable ADC + DMA + timer
    void stop();

    // Returns pointer to a ready PACKET_LEN packet, or nullptr if not ready.
    // Pointer valid until next call to read_packet().
    uint8_t* read_packet();

    int get_write_pos() const { return dma_write_packet_count; }

   private:
    BeginStatus init_timer();
    BeginStatus init_adc();
    BeginStatus init_dma();

    void on_timer(timer_callback_args_t* args);

    static void timer_callback(timer_callback_args_t* args) {
        if (args->event != TIMER_EVENT_CYCLE_END) return;
        MicDriver* self = (MicDriver*)(args->p_context);
        self->on_timer(args);
    }
};

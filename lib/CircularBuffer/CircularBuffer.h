#pragma once

#include <cstdint>
#include <cstring>

class CircularBuffer {
   public:
    static constexpr int BUFFER_LEN_BITS = 13;
    static constexpr int BUFFER_LEN = 1 << BUFFER_LEN_BITS;
    static constexpr int BUFFER_LEN_MASK = BUFFER_LEN - 1;

   private:
    alignas(2) uint8_t buffer[BUFFER_LEN] = {};

    // points to the next byte to be read
    volatile int consumer_pos = 0;

    // points to the next byte to be written
    volatile int producer_pos = 0;

   public:
    explicit CircularBuffer() {};
    ~CircularBuffer() {}

    void reset() {
        consumer_pos = 0;
        producer_pos = 0;
    }

    bool is_full() const {
        return producer_pos == ((consumer_pos - 1) & BUFFER_LEN_MASK);
    }

    bool is_empty() const { return producer_pos == consumer_pos; }

    int available() const {
        return (producer_pos - consumer_pos) & BUFFER_LEN_MASK;
    }

    int produce(const uint8_t* data, int len) {
        if (is_full()) return 0;

        int bytes_written = 0;
        while (!is_full() && bytes_written < len) {
            buffer[producer_pos] = data[bytes_written];
            producer_pos = (producer_pos + 1) & BUFFER_LEN_MASK;
            bytes_written++;
        }
        return bytes_written;
    }

    int consume_8() {
        if (is_empty()) return -1;

        int data = buffer[consumer_pos];
        consumer_pos = (consumer_pos + 1) & BUFFER_LEN_MASK;
        return data;
    }

    int consume_16() {
        if (available() < 2) return -1;

        uint16_t raw_value;
        std::memcpy(&raw_value, &buffer[consumer_pos], sizeof(raw_value));

        consumer_pos = (consumer_pos + 2) & BUFFER_LEN_MASK;
        return static_cast<int>(raw_value);
    }
};

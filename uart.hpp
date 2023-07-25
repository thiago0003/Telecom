#ifndef UART_HPP
#define UART_HPP

#include <functional>
#include <deque>
#include <mutex>
#include <stdint.h>
#include <stdexcept>
#include "config.hpp"

class UART_RX
{
public:
    UART_RX(std::function<void(uint8_t)> get_byte) :get_byte(get_byte) {}
    void put_samples(const unsigned int *buffer, unsigned int n);
private:
    std::function<void(uint8_t)> get_byte;
    float ring_buffer[SAMPLES_PER_SYMBOL + 1] = {};
    unsigned int bit_counter = 0;
    unsigned int bit_counter_noisy_tolerance = 0;
    uint8_t received_byte = 0;
    int count = 0;
};

class UART_TX
{
public:
    void put_byte(uint8_t byte);
    void get_samples(unsigned int *buffer, unsigned int n);
private:
    std::deque<unsigned int> samples;
    std::mutex samples_mutex;
    void put_bit(unsigned int bit);
};

#endif

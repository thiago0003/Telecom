#pragma once
#ifndef RING_BUFFER_HPP
#define RING_BUFFER_HPP

#include <deque>
#include <stdexcept>

class CircularBuffer
{
public:
    CircularBuffer(std::size_t buffer_size) : buffer(buffer_size, 0), head(0), tail(0), count(0) {}

    void push_back(unsigned int value)
    {
        buffer[tail] = value;
        tail = (tail + 1) % buffer.size();
        if (count < buffer.size())
            count++;
        else
            head = (head + 1) % buffer.size();
    }

    void pop_back()
    {
        if (count > 0)
        {
            tail = (tail + buffer.size() - 1) % buffer.size();
            count--;
        }
    }

    unsigned int operator[](std::size_t idx) const
    {
        if (idx >= count)
            throw std::out_of_range("Circular buffer index out of range");
        return buffer[(head + idx) % buffer.size()];
    }

    std::size_t size() const { return count; }
    bool empty() const { return count == 0; }
    void clear()
    {
        head = tail = count = 0;
    }

private:
    std::deque<unsigned int> buffer;
    std::size_t head;
    std::size_t tail;
    std::size_t count;
};

#endif
#ifndef DMARXRING_H
#define DMARXRING_H

#include <cstddef>
#include <cstdint>
#include <cstring>

// Single-producer/single-consumer byte ring. Callers must prevent the DMA IRQ
// from interrupting pop(); push() runs from that IRQ or with it disabled.
template <std::size_t Capacity>
class DmaRxRing {
    static_assert(Capacity > 0, "DMA RX ring must not be empty");

public:
    void reset()
    {
        write_ = 0;
        read_ = 0;
        count_ = 0;
        overflow_ = false;
    }

    std::size_t size() const { return count_; }
    std::size_t capacity() const { return Capacity; }

    std::size_t push(const uint8_t* source, std::size_t length)
    {
        const std::size_t free = Capacity - count_;
        if (length > free) {
            length = free;
            overflow_ = true;
        }
        if (length == 0) return 0;

        const std::size_t first = length < Capacity - write_
            ? length : Capacity - write_;
        std::memcpy(data_ + write_, source, first);
        std::memcpy(data_, source + first, length - first);
        write_ = (write_ + length) % Capacity;
        count_ += length;
        return length;
    }

    std::size_t pop(uint8_t* destination, std::size_t length)
    {
        if (length > count_) length = count_;
        if (length == 0) return 0;

        const std::size_t first = length < Capacity - read_
            ? length : Capacity - read_;
        std::memcpy(destination, data_ + read_, first);
        std::memcpy(destination + first, data_, length - first);
        read_ = (read_ + length) % Capacity;
        count_ -= length;
        return length;
    }

    bool take_overflow()
    {
        const bool overflow = overflow_;
        overflow_ = false;
        return overflow;
    }

private:
    uint8_t data_[Capacity] = {};
    std::size_t write_ = 0;
    std::size_t read_ = 0;
    std::size_t count_ = 0;
    bool overflow_ = false;
};

#endif

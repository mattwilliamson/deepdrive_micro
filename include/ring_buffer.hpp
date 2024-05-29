#ifndef RING_BUFFER_HPP
#define RING_BUFFER_HPP

#include <cstddef>

template <typename T, size_t N>
/**
 * @brief A class representing a ring buffer.
 * 
 * The RingBuffer class provides functionality to store and retrieve elements in a circular buffer.
 * It supports pushing elements into the buffer and reading the average of all elements in the buffer.
 * 
 * @tparam T The type of elements stored in the buffer.
 * @tparam N The maximum size of the buffer.
 */
class RingBuffer {
public:
    /**
     * @brief Default constructor for the RingBuffer class.
     * 
     * Initializes the buffer size and front index to zero.
     */
    RingBuffer() : size_(0), front_(0) {}

    /**
     * @brief Pushes an element into the buffer.
     * 
     * The element is added to the front of the buffer and the front index is updated accordingly.
     * If the buffer is full, the oldest element is overwritten.
     * 
     * @param value The element to be added to the buffer.
     */
    void push(const T& value) {
        buffer_[front_] = value;
        front_ = (front_ + 1) % N;
        if (size_ < N) {
            size_++;
        }
    }

    /**
     * @brief Reads the average of all elements in the buffer.
     * 
     * Calculates and returns the average of all elements in the buffer.
     * 
     * @return The average of all elements in the buffer.
     */
    T read() const {
        T sum = T();
        for (size_t i = 0; i < size_; i++) {
            sum += buffer_[(front_ + i) % N];
        }
        return sum / size_;
    }

    /**
     * @brief Calculates and returns the average of all elements in the buffer.
     * 
     * If the buffer is empty, returns 0.0.
     * 
     * @return The average of all elements in the buffer.
     */
    T average() const {
        if (size_ == 0) {
            return 0.0;
        }

        T sum = 0.0;
        for (size_t i = 0; i < size_; i++) {
            sum += buffer_[i];
        }

        return sum / size_;
    }

private:
    T buffer_[N];       // The buffer to store elements
    size_t size_;       // The current size of the buffer
    size_t front_;      // The index of the front element in the buffer
};

#endif // RING_BUFFER_HPP
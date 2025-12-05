#pragma once

/**
 * @file lock_free_queue.hpp
 * @brief Lock-free Single-Producer Single-Consumer (SPSC) queue
 *
 * Designed for real-time audio applications:
 * - No locks or mutexes
 * - No memory allocation after construction
 * - Cache-line padded to prevent false sharing
 * - Wait-free for both producer and consumer
 * - Fixed capacity (power of 2 for fast modulo)
 */

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <array>
#include <type_traits>
#include "dsp_core.hpp"

namespace oscilloplot {

//==============================================================================
// Cache-line padding to prevent false sharing
//==============================================================================

struct OSCILLOPLOT_CACHE_ALIGN CacheLinePad {
    char pad[dsp::CACHE_LINE_SIZE];
};

//==============================================================================
// Lock-free SPSC Queue
//==============================================================================

/**
 * @brief Lock-free single-producer single-consumer queue
 *
 * @tparam T Element type (should be trivially copyable for best performance)
 * @tparam Capacity Queue capacity (must be power of 2)
 *
 * Usage:
 * - Producer thread: push()
 * - Consumer thread: pop()
 * - Both threads can call size(), empty()
 *
 * Memory model: Uses acquire-release semantics for synchronization
 */
template<typename T, size_t Capacity>
class SPSCQueue {
    static_assert((Capacity & (Capacity - 1)) == 0, "Capacity must be power of 2");
    static_assert(Capacity >= 2, "Capacity must be at least 2");

public:
    SPSCQueue() : m_head(0), m_tail(0) {
        static_assert(std::is_trivially_copyable_v<T>,
            "SPSCQueue works best with trivially copyable types");
    }

    // Non-copyable, non-movable (contains atomics)
    SPSCQueue(const SPSCQueue&) = delete;
    SPSCQueue& operator=(const SPSCQueue&) = delete;
    SPSCQueue(SPSCQueue&&) = delete;
    SPSCQueue& operator=(SPSCQueue&&) = delete;

    /**
     * @brief Push an element (producer only)
     * @return true if successful, false if queue is full
     */
    bool push(const T& item) {
        const size_t head = m_head.load(std::memory_order_relaxed);
        const size_t nextHead = (head + 1) & MASK;

        if (nextHead == m_tail.load(std::memory_order_acquire)) {
            return false;  // Queue is full
        }

        m_buffer[head] = item;
        m_head.store(nextHead, std::memory_order_release);
        return true;
    }

    /**
     * @brief Push multiple elements (producer only)
     * @return Number of elements actually pushed
     */
    size_t pushBatch(const T* items, size_t count) {
        size_t pushed = 0;
        for (size_t i = 0; i < count; ++i) {
            if (!push(items[i])) break;
            ++pushed;
        }
        return pushed;
    }

    /**
     * @brief Pop an element (consumer only)
     * @return true if successful, false if queue is empty
     */
    bool pop(T& item) {
        const size_t tail = m_tail.load(std::memory_order_relaxed);

        if (tail == m_head.load(std::memory_order_acquire)) {
            return false;  // Queue is empty
        }

        item = m_buffer[tail];
        m_tail.store((tail + 1) & MASK, std::memory_order_release);
        return true;
    }

    /**
     * @brief Pop multiple elements (consumer only)
     * @return Number of elements actually popped
     */
    size_t popBatch(T* items, size_t maxCount) {
        size_t popped = 0;
        for (size_t i = 0; i < maxCount; ++i) {
            if (!pop(items[i])) break;
            ++popped;
        }
        return popped;
    }

    /**
     * @brief Peek at front element without removing (consumer only)
     */
    bool peek(T& item) const {
        const size_t tail = m_tail.load(std::memory_order_relaxed);

        if (tail == m_head.load(std::memory_order_acquire)) {
            return false;
        }

        item = m_buffer[tail];
        return true;
    }

    /**
     * @brief Check if queue is empty (approximate, may change)
     */
    bool empty() const {
        return m_head.load(std::memory_order_acquire) ==
               m_tail.load(std::memory_order_acquire);
    }

    /**
     * @brief Check if queue is full (approximate, may change)
     */
    bool full() const {
        const size_t head = m_head.load(std::memory_order_acquire);
        const size_t tail = m_tail.load(std::memory_order_acquire);
        return ((head + 1) & MASK) == tail;
    }

    /**
     * @brief Get approximate size (may change)
     */
    size_t size() const {
        const size_t head = m_head.load(std::memory_order_acquire);
        const size_t tail = m_tail.load(std::memory_order_acquire);
        return (head - tail) & MASK;
    }

    /**
     * @brief Get capacity
     */
    static constexpr size_t capacity() { return Capacity; }

    /**
     * @brief Clear the queue (must be called when no concurrent access)
     */
    void clear() {
        m_head.store(0, std::memory_order_relaxed);
        m_tail.store(0, std::memory_order_relaxed);
    }

private:
    static constexpr size_t MASK = Capacity - 1;

    // Cache-line padded to prevent false sharing between producer and consumer
    OSCILLOPLOT_CACHE_ALIGN std::atomic<size_t> m_head;
    CacheLinePad m_pad1;
    OSCILLOPLOT_CACHE_ALIGN std::atomic<size_t> m_tail;
    CacheLinePad m_pad2;
    OSCILLOPLOT_CACHE_ALIGN std::array<T, Capacity> m_buffer;
};

//==============================================================================
// Lock-free Ring Buffer for Audio Samples
//==============================================================================

/**
 * @brief Optimized ring buffer for audio sample streaming
 *
 * Features:
 * - Contiguous read/write regions for efficient memcpy
 * - Power-of-2 size for fast wraparound
 * - Cache-friendly layout
 */
template<size_t Capacity>
class AudioRingBuffer {
    static_assert((Capacity & (Capacity - 1)) == 0, "Capacity must be power of 2");

public:
    AudioRingBuffer() : m_writePos(0), m_readPos(0) {
        // Zero-initialize buffer
        std::fill(m_buffer.begin(), m_buffer.end(), 0.0f);
    }

    /**
     * @brief Write samples to buffer (producer thread)
     * @return Number of samples actually written
     */
    size_t write(const float* data, size_t count) {
        const size_t writePos = m_writePos.load(std::memory_order_relaxed);
        const size_t readPos = m_readPos.load(std::memory_order_acquire);

        const size_t available = Capacity - ((writePos - readPos) & MASK) - 1;
        const size_t toWrite = (count < available) ? count : available;

        if (toWrite == 0) return 0;

        const size_t firstPart = Capacity - (writePos & MASK);
        if (firstPart >= toWrite) {
            std::copy(data, data + toWrite, m_buffer.data() + (writePos & MASK));
        } else {
            std::copy(data, data + firstPart, m_buffer.data() + (writePos & MASK));
            std::copy(data + firstPart, data + toWrite, m_buffer.data());
        }

        m_writePos.store(writePos + toWrite, std::memory_order_release);
        return toWrite;
    }

    /**
     * @brief Read samples from buffer (consumer thread)
     * @return Number of samples actually read
     */
    size_t read(float* data, size_t count) {
        const size_t writePos = m_writePos.load(std::memory_order_acquire);
        const size_t readPos = m_readPos.load(std::memory_order_relaxed);

        const size_t available = (writePos - readPos) & MASK;
        const size_t toRead = (count < available) ? count : available;

        if (toRead == 0) return 0;

        const size_t firstPart = Capacity - (readPos & MASK);
        if (firstPart >= toRead) {
            std::copy(m_buffer.data() + (readPos & MASK),
                     m_buffer.data() + (readPos & MASK) + toRead, data);
        } else {
            std::copy(m_buffer.data() + (readPos & MASK),
                     m_buffer.data() + Capacity, data);
            std::copy(m_buffer.data(), m_buffer.data() + (toRead - firstPart),
                     data + firstPart);
        }

        m_readPos.store(readPos + toRead, std::memory_order_release);
        return toRead;
    }

    /**
     * @brief Get number of samples available for reading
     */
    size_t availableRead() const {
        const size_t writePos = m_writePos.load(std::memory_order_acquire);
        const size_t readPos = m_readPos.load(std::memory_order_relaxed);
        return (writePos - readPos) & MASK;
    }

    /**
     * @brief Get number of samples that can be written
     */
    size_t availableWrite() const {
        const size_t writePos = m_writePos.load(std::memory_order_relaxed);
        const size_t readPos = m_readPos.load(std::memory_order_acquire);
        return Capacity - ((writePos - readPos) & MASK) - 1;
    }

    void clear() {
        m_writePos.store(0, std::memory_order_relaxed);
        m_readPos.store(0, std::memory_order_relaxed);
    }

private:
    static constexpr size_t MASK = Capacity - 1;

    OSCILLOPLOT_CACHE_ALIGN std::atomic<size_t> m_writePos;
    CacheLinePad m_pad1;
    OSCILLOPLOT_CACHE_ALIGN std::atomic<size_t> m_readPos;
    CacheLinePad m_pad2;
    OSCILLOPLOT_SIMD_ALIGN std::array<float, Capacity> m_buffer;
};

} // namespace oscilloplot

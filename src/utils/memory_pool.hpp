#pragma once

/**
 * @file memory_pool.hpp
 * @brief Fixed-size memory pool with zero runtime allocation
 *
 * Embedded-style memory management:
 * - Pre-allocated at startup
 * - O(1) allocation and deallocation
 * - No fragmentation
 * - Cache-friendly layout
 * - No heap operations after initialization
 */

#include <cstddef>
#include <cstdint>
#include <array>
#include <bitset>
#include <new>
#include "dsp_core.hpp"

namespace oscilloplot {

//==============================================================================
// Fixed-size Object Pool
//==============================================================================

/**
 * @brief Object pool with compile-time fixed capacity
 *
 * @tparam T Object type
 * @tparam Capacity Maximum number of objects
 *
 * All memory is pre-allocated. Allocation/deallocation is O(1).
 */
template<typename T, size_t Capacity>
class ObjectPool {
public:
    ObjectPool() : m_freeCount(Capacity) {
        // Initialize free list
        for (size_t i = 0; i < Capacity - 1; ++i) {
            *reinterpret_cast<size_t*>(&m_storage[i]) = i + 1;
        }
        *reinterpret_cast<size_t*>(&m_storage[Capacity - 1]) = INVALID_INDEX;
        m_freeHead = 0;
    }

    ~ObjectPool() {
        // Note: Does not call destructors - caller must return objects
    }

    /**
     * @brief Allocate an object from the pool
     * @return Pointer to uninitialized memory, or nullptr if pool exhausted
     */
    T* allocate() {
        if (m_freeHead == INVALID_INDEX) {
            return nullptr;  // Pool exhausted
        }

        size_t index = m_freeHead;
        m_freeHead = *reinterpret_cast<size_t*>(&m_storage[index]);
        --m_freeCount;

        return reinterpret_cast<T*>(&m_storage[index]);
    }

    /**
     * @brief Allocate and construct an object
     */
    template<typename... Args>
    T* create(Args&&... args) {
        T* ptr = allocate();
        if (ptr) {
            new (ptr) T(std::forward<Args>(args)...);
        }
        return ptr;
    }

    /**
     * @brief Return an object to the pool
     */
    void deallocate(T* ptr) {
        if (!ptr) return;

        // Calculate index
        auto* storage = reinterpret_cast<Storage*>(ptr);
        size_t index = storage - m_storage.data();

        if (index >= Capacity) return;  // Invalid pointer

        // Add to free list
        *reinterpret_cast<size_t*>(storage) = m_freeHead;
        m_freeHead = index;
        ++m_freeCount;
    }

    /**
     * @brief Destroy and deallocate an object
     */
    void destroy(T* ptr) {
        if (ptr) {
            ptr->~T();
            deallocate(ptr);
        }
    }

    size_t available() const { return m_freeCount; }
    size_t used() const { return Capacity - m_freeCount; }
    static constexpr size_t capacity() { return Capacity; }
    bool empty() const { return m_freeCount == Capacity; }
    bool full() const { return m_freeCount == 0; }

private:
    static constexpr size_t INVALID_INDEX = ~size_t(0);

    // Aligned storage for objects
    struct alignas(alignof(T)) Storage {
        std::byte data[sizeof(T)];
    };

    std::array<Storage, Capacity> m_storage;
    size_t m_freeHead;
    size_t m_freeCount;
};

//==============================================================================
// Stack Allocator for Temporary Buffers
//==============================================================================

/**
 * @brief Linear allocator for frame-temporary allocations
 *
 * Usage pattern:
 * - Reset at start of each frame
 * - Allocate temporary buffers during frame
 * - All memory freed at once on reset
 *
 * Perfect for per-frame audio processing buffers.
 */
template<size_t Capacity>
class StackAllocator {
public:
    StackAllocator() : m_offset(0) {}

    /**
     * @brief Allocate aligned memory
     * @param size Size in bytes
     * @param alignment Alignment requirement (default: SIMD alignment)
     * @return Pointer to allocated memory, or nullptr if insufficient space
     */
    void* allocate(size_t size, size_t alignment = dsp::SIMD_ALIGNMENT) {
        // Align offset
        size_t alignedOffset = (m_offset + alignment - 1) & ~(alignment - 1);

        if (alignedOffset + size > Capacity) {
            return nullptr;  // Out of memory
        }

        void* ptr = m_buffer.data() + alignedOffset;
        m_offset = alignedOffset + size;
        return ptr;
    }

    /**
     * @brief Allocate array of type T
     */
    template<typename T>
    T* allocateArray(size_t count) {
        return static_cast<T*>(allocate(sizeof(T) * count, alignof(T)));
    }

    /**
     * @brief Reset allocator (free all allocations)
     */
    void reset() {
        m_offset = 0;
    }

    size_t used() const { return m_offset; }
    size_t available() const { return Capacity - m_offset; }
    static constexpr size_t capacity() { return Capacity; }

private:
    OSCILLOPLOT_SIMD_ALIGN std::array<std::byte, Capacity> m_buffer;
    size_t m_offset;
};

//==============================================================================
// Pre-allocated Audio Buffer Pool
//==============================================================================

/**
 * @brief Pool of fixed-size audio buffers for DSP processing
 *
 * Provides scratch buffers for effects without runtime allocation.
 */
template<size_t BufferSize, size_t NumBuffers>
class AudioBufferPool {
public:
    static constexpr size_t BUFFER_SIZE = BufferSize;

    AudioBufferPool() {
        // Initialize all buffers as available
        m_available.set();
    }

    /**
     * @brief Acquire a buffer
     * @return Pointer to buffer, or nullptr if none available
     */
    float* acquire() {
        for (size_t i = 0; i < NumBuffers; ++i) {
            if (m_available.test(i)) {
                m_available.reset(i);
                return m_buffers[i].data();
            }
        }
        return nullptr;
    }

    /**
     * @brief Release a buffer back to the pool
     */
    void release(float* buffer) {
        for (size_t i = 0; i < NumBuffers; ++i) {
            if (m_buffers[i].data() == buffer) {
                m_available.set(i);
                return;
            }
        }
    }

    size_t availableCount() const { return m_available.count(); }

    /**
     * @brief RAII wrapper for automatic buffer release
     */
    class ScopedBuffer {
    public:
        ScopedBuffer(AudioBufferPool& pool) : m_pool(pool), m_buffer(pool.acquire()) {}
        ~ScopedBuffer() { if (m_buffer) m_pool.release(m_buffer); }

        float* get() { return m_buffer; }
        operator float*() { return m_buffer; }
        explicit operator bool() const { return m_buffer != nullptr; }

        // Non-copyable
        ScopedBuffer(const ScopedBuffer&) = delete;
        ScopedBuffer& operator=(const ScopedBuffer&) = delete;

    private:
        AudioBufferPool& m_pool;
        float* m_buffer;
    };

private:
    std::array<OSCILLOPLOT_SIMD_ALIGN std::array<float, BufferSize>, NumBuffers> m_buffers;
    std::bitset<NumBuffers> m_available;
};

//==============================================================================
// Static Pattern Storage (Data-Oriented Design)
//==============================================================================

/**
 * @brief Fixed-capacity pattern storage with Structure-of-Arrays layout
 *
 * DoD benefits:
 * - Better cache utilization for batch processing
 * - SIMD-friendly memory layout
 * - No heap allocation
 */
template<size_t MaxPoints>
class StaticPattern {
public:
    StaticPattern() : m_size(0) {}

    // Accessors
    float* xData() { return m_x.data(); }
    float* yData() { return m_y.data(); }
    const float* xData() const { return m_x.data(); }
    const float* yData() const { return m_y.data(); }

    size_t size() const { return m_size; }
    bool empty() const { return m_size == 0; }
    static constexpr size_t capacity() { return MaxPoints; }

    void setSize(size_t size) {
        m_size = (size <= MaxPoints) ? size : MaxPoints;
    }

    void clear() { m_size = 0; }

    // Element access
    void setPoint(size_t index, float x, float y) {
        if (index < MaxPoints) {
            m_x[index] = x;
            m_y[index] = y;
            if (index >= m_size) m_size = index + 1;
        }
    }

    void getPoint(size_t index, float& x, float& y) const {
        if (index < m_size) {
            x = m_x[index];
            y = m_y[index];
        }
    }

    // Batch operations (cache-friendly)
    void copyFrom(const float* srcX, const float* srcY, size_t count) {
        size_t toCopy = (count <= MaxPoints) ? count : MaxPoints;
        std::copy(srcX, srcX + toCopy, m_x.data());
        std::copy(srcY, srcY + toCopy, m_y.data());
        m_size = toCopy;
    }

    void copyTo(float* dstX, float* dstY) const {
        std::copy(m_x.data(), m_x.data() + m_size, dstX);
        std::copy(m_y.data(), m_y.data() + m_size, dstY);
    }

private:
    OSCILLOPLOT_SIMD_ALIGN std::array<float, MaxPoints> m_x;
    OSCILLOPLOT_SIMD_ALIGN std::array<float, MaxPoints> m_y;
    size_t m_size;
};

} // namespace oscilloplot

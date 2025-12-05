#pragma once

/**
 * @file dsp_core.hpp
 * @brief High-performance DSP core utilities with embedded-style optimizations
 *
 * Design principles:
 * - Zero heap allocation in hot paths
 * - Cache-friendly memory layouts
 * - SIMD-ready data alignment
 * - Lookup tables for transcendental functions
 * - Compile-time constants where possible
 * - No virtual functions in critical paths
 */

#include <cstdint>
#include <cstddef>
#include <cmath>
#include <array>
#include <type_traits>

// Platform-specific SIMD includes
#if defined(__SSE2__) || defined(_M_X64) || defined(_M_AMD64)
    #define OSCILLOPLOT_HAS_SSE2 1
    #include <emmintrin.h>
#endif

#if defined(__SSE4_1__)
    #define OSCILLOPLOT_HAS_SSE41 1
    #include <smmintrin.h>
#endif

#if defined(__AVX__)
    #define OSCILLOPLOT_HAS_AVX 1
    #include <immintrin.h>
#endif

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
    #define OSCILLOPLOT_HAS_NEON 1
    #include <arm_neon.h>
#endif

namespace oscilloplot {
namespace dsp {

//==============================================================================
// Compile-time constants
//==============================================================================

inline constexpr float PI = 3.14159265358979323846f;
inline constexpr float TWO_PI = 6.28318530717958647692f;
inline constexpr float HALF_PI = 1.57079632679489661923f;
inline constexpr float INV_PI = 0.31830988618379067154f;
inline constexpr float INV_TWO_PI = 0.15915494309189533577f;

inline constexpr size_t CACHE_LINE_SIZE = 64;
inline constexpr size_t SIMD_ALIGNMENT = 32;  // AVX alignment

// Lookup table sizes (power of 2 for fast modulo)
inline constexpr size_t SINE_TABLE_SIZE = 4096;
inline constexpr size_t SINE_TABLE_MASK = SINE_TABLE_SIZE - 1;

//==============================================================================
// Memory alignment utilities
//==============================================================================

#if defined(_MSC_VER)
    #define OSCILLOPLOT_ALIGN(x) __declspec(align(x))
    #define OSCILLOPLOT_RESTRICT __restrict
#else
    #define OSCILLOPLOT_ALIGN(x) __attribute__((aligned(x)))
    #define OSCILLOPLOT_RESTRICT __restrict__
#endif

#define OSCILLOPLOT_CACHE_ALIGN OSCILLOPLOT_ALIGN(CACHE_LINE_SIZE)
#define OSCILLOPLOT_SIMD_ALIGN OSCILLOPLOT_ALIGN(SIMD_ALIGNMENT)

// Force inline for critical functions
#if defined(_MSC_VER)
    #define OSCILLOPLOT_FORCE_INLINE __forceinline
#else
    #define OSCILLOPLOT_FORCE_INLINE inline __attribute__((always_inline))
#endif

// Likely/unlikely branch hints
#if defined(__GNUC__) || defined(__clang__)
    #define OSCILLOPLOT_LIKELY(x) __builtin_expect(!!(x), 1)
    #define OSCILLOPLOT_UNLIKELY(x) __builtin_expect(!!(x), 0)
#else
    #define OSCILLOPLOT_LIKELY(x) (x)
    #define OSCILLOPLOT_UNLIKELY(x) (x)
#endif

//==============================================================================
// Sine/Cosine Lookup Table
//==============================================================================

/**
 * @brief Compile-time generated sine lookup table
 *
 * Uses constexpr to generate table at compile time.
 * Linear interpolation for sub-sample accuracy.
 */
class SineLUT {
public:
    // Generate table at compile time (C++17)
    static constexpr std::array<float, SINE_TABLE_SIZE + 1> generateTable() {
        std::array<float, SINE_TABLE_SIZE + 1> table{};
        for (size_t i = 0; i <= SINE_TABLE_SIZE; ++i) {
            table[i] = std::sin(TWO_PI * static_cast<float>(i) / SINE_TABLE_SIZE);
        }
        return table;
    }

    static inline const std::array<float, SINE_TABLE_SIZE + 1> table = generateTable();

    /**
     * @brief Fast sine using lookup table with linear interpolation
     * @param phase Normalized phase [0, 1)
     * @return Sine value in [-1, 1]
     */
    OSCILLOPLOT_FORCE_INLINE static float sin(float phase) {
        // Wrap phase to [0, 1)
        phase = phase - static_cast<int>(phase);
        if (phase < 0.0f) phase += 1.0f;

        const float indexF = phase * SINE_TABLE_SIZE;
        const size_t index0 = static_cast<size_t>(indexF);
        const size_t index1 = (index0 + 1) & SINE_TABLE_MASK;
        const float frac = indexF - static_cast<float>(index0);

        // Linear interpolation
        return table[index0] + frac * (table[index1] - table[index0]);
    }

    /**
     * @brief Fast cosine using lookup table
     * @param phase Normalized phase [0, 1)
     */
    OSCILLOPLOT_FORCE_INLINE static float cos(float phase) {
        return sin(phase + 0.25f);  // cos(x) = sin(x + π/2)
    }

    /**
     * @brief Compute both sin and cos efficiently
     */
    OSCILLOPLOT_FORCE_INLINE static void sincos(float phase, float& sinOut, float& cosOut) {
        sinOut = sin(phase);
        cosOut = sin(phase + 0.25f);
    }
};

//==============================================================================
// Fast math utilities
//==============================================================================

/**
 * @brief Fast inverse square root (Quake III style, modernized)
 */
OSCILLOPLOT_FORCE_INLINE float fastInvSqrt(float x) {
    union { float f; uint32_t i; } conv = { x };
    conv.i = 0x5f3759df - (conv.i >> 1);
    conv.f *= 1.5f - (x * 0.5f * conv.f * conv.f);
    return conv.f;
}

/**
 * @brief Fast approximation of tanh for soft clipping
 * Using rational approximation: tanh(x) ≈ x / (1 + |x| + x²/3)
 */
OSCILLOPLOT_FORCE_INLINE float fastTanh(float x) {
    const float x2 = x * x;
    return x / (1.0f + std::abs(x) + x2 * 0.333333f);
}

/**
 * @brief Soft clipping with adjustable drive
 */
OSCILLOPLOT_FORCE_INLINE float softClip(float x, float drive = 1.0f) {
    return fastTanh(x * drive);
}

/**
 * @brief Linear interpolation
 */
OSCILLOPLOT_FORCE_INLINE float lerp(float a, float b, float t) {
    return a + t * (b - a);
}

/**
 * @brief Clamp value to range
 */
OSCILLOPLOT_FORCE_INLINE float clamp(float x, float minVal, float maxVal) {
    return (x < minVal) ? minVal : ((x > maxVal) ? maxVal : x);
}

/**
 * @brief Convert decibels to linear amplitude
 */
OSCILLOPLOT_FORCE_INLINE float dbToLinear(float db) {
    return std::pow(10.0f, db * 0.05f);
}

/**
 * @brief Convert linear amplitude to decibels
 */
OSCILLOPLOT_FORCE_INLINE float linearToDb(float linear) {
    return 20.0f * std::log10(linear + 1e-10f);
}

//==============================================================================
// SIMD Vector Operations
//==============================================================================

#if OSCILLOPLOT_HAS_SSE2

/**
 * @brief Process 4 floats in parallel using SSE2
 */
struct Vec4f {
    __m128 v;

    Vec4f() : v(_mm_setzero_ps()) {}
    Vec4f(__m128 val) : v(val) {}
    Vec4f(float val) : v(_mm_set1_ps(val)) {}
    Vec4f(float a, float b, float c, float d) : v(_mm_setr_ps(a, b, c, d)) {}

    static Vec4f load(const float* ptr) { return Vec4f(_mm_loadu_ps(ptr)); }
    static Vec4f loadAligned(const float* ptr) { return Vec4f(_mm_load_ps(ptr)); }

    void store(float* ptr) const { _mm_storeu_ps(ptr, v); }
    void storeAligned(float* ptr) const { _mm_store_ps(ptr, v); }

    Vec4f operator+(const Vec4f& other) const { return Vec4f(_mm_add_ps(v, other.v)); }
    Vec4f operator-(const Vec4f& other) const { return Vec4f(_mm_sub_ps(v, other.v)); }
    Vec4f operator*(const Vec4f& other) const { return Vec4f(_mm_mul_ps(v, other.v)); }
    Vec4f operator/(const Vec4f& other) const { return Vec4f(_mm_div_ps(v, other.v)); }

    Vec4f& operator+=(const Vec4f& other) { v = _mm_add_ps(v, other.v); return *this; }
    Vec4f& operator*=(const Vec4f& other) { v = _mm_mul_ps(v, other.v); return *this; }
};

/**
 * @brief SIMD sine approximation (Taylor series, good for small angles)
 */
inline Vec4f simdSinApprox(Vec4f x) {
    // sin(x) ≈ x - x³/6 + x⁵/120 for small x
    const Vec4f x2 = x * x;
    const Vec4f x3 = x2 * x;
    const Vec4f x5 = x3 * x2;
    const Vec4f c3(-0.166666667f);
    const Vec4f c5(0.00833333333f);
    return x + x3 * c3 + x5 * c5;
}

#endif // OSCILLOPLOT_HAS_SSE2

//==============================================================================
// Batch processing utilities
//==============================================================================

/**
 * @brief Apply gain to buffer (SIMD optimized)
 */
inline void applyGain(float* OSCILLOPLOT_RESTRICT buffer, size_t count, float gain) {
#if OSCILLOPLOT_HAS_SSE2
    const Vec4f gainVec(gain);
    size_t i = 0;

    // Process 4 samples at a time
    for (; i + 4 <= count; i += 4) {
        Vec4f samples = Vec4f::load(buffer + i);
        samples *= gainVec;
        samples.store(buffer + i);
    }

    // Handle remaining samples
    for (; i < count; ++i) {
        buffer[i] *= gain;
    }
#else
    for (size_t i = 0; i < count; ++i) {
        buffer[i] *= gain;
    }
#endif
}

/**
 * @brief Mix two buffers: dest = dest + src * gain
 */
inline void mixBuffers(float* OSCILLOPLOT_RESTRICT dest,
                       const float* OSCILLOPLOT_RESTRICT src,
                       size_t count, float gain) {
#if OSCILLOPLOT_HAS_SSE2
    const Vec4f gainVec(gain);
    size_t i = 0;

    for (; i + 4 <= count; i += 4) {
        Vec4f d = Vec4f::load(dest + i);
        Vec4f s = Vec4f::load(src + i);
        d += s * gainVec;
        d.store(dest + i);
    }

    for (; i < count; ++i) {
        dest[i] += src[i] * gain;
    }
#else
    for (size_t i = 0; i < count; ++i) {
        dest[i] += src[i] * gain;
    }
#endif
}

/**
 * @brief Apply rotation to X,Y coordinate pairs
 * Optimized for batch processing oscilloscope data
 */
inline void rotateXY(float* OSCILLOPLOT_RESTRICT x,
                     float* OSCILLOPLOT_RESTRICT y,
                     size_t count,
                     float cosAngle,
                     float sinAngle) {
#if OSCILLOPLOT_HAS_SSE2
    const Vec4f cosVec(cosAngle);
    const Vec4f sinVec(sinAngle);
    size_t i = 0;

    for (; i + 4 <= count; i += 4) {
        Vec4f xv = Vec4f::load(x + i);
        Vec4f yv = Vec4f::load(y + i);

        // x' = x*cos - y*sin
        // y' = x*sin + y*cos
        Vec4f xNew = xv * cosVec - yv * sinVec;
        Vec4f yNew = xv * sinVec + yv * cosVec;

        xNew.store(x + i);
        yNew.store(y + i);
    }

    for (; i < count; ++i) {
        float xOld = x[i];
        float yOld = y[i];
        x[i] = xOld * cosAngle - yOld * sinAngle;
        y[i] = xOld * sinAngle + yOld * cosAngle;
    }
#else
    for (size_t i = 0; i < count; ++i) {
        float xOld = x[i];
        float yOld = y[i];
        x[i] = xOld * cosAngle - yOld * sinAngle;
        y[i] = xOld * sinAngle + yOld * cosAngle;
    }
#endif
}

/**
 * @brief Scale X,Y coordinates uniformly
 */
inline void scaleXY(float* OSCILLOPLOT_RESTRICT x,
                    float* OSCILLOPLOT_RESTRICT y,
                    size_t count,
                    float scale) {
#if OSCILLOPLOT_HAS_SSE2
    const Vec4f scaleVec(scale);
    size_t i = 0;

    for (; i + 4 <= count; i += 4) {
        Vec4f xv = Vec4f::load(x + i);
        Vec4f yv = Vec4f::load(y + i);
        xv *= scaleVec;
        yv *= scaleVec;
        xv.store(x + i);
        yv.store(y + i);
    }

    for (; i < count; ++i) {
        x[i] *= scale;
        y[i] *= scale;
    }
#else
    for (size_t i = 0; i < count; ++i) {
        x[i] *= scale;
        y[i] *= scale;
    }
#endif
}

} // namespace dsp
} // namespace oscilloplot

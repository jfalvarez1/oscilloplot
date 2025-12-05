#pragma once

/**
 * @file effect_processor.hpp
 * @brief High-performance effect processor using Data-Oriented Design
 *
 * Key optimizations:
 * - No virtual function calls in processing loop
 * - Compile-time effect dispatch using std::variant
 * - SIMD-accelerated batch processing
 * - Pre-allocated effect state
 * - Cache-friendly memory layout
 */

#include <variant>
#include <array>
#include <cstdint>
#include "utils/dsp_core.hpp"
#include "utils/memory_pool.hpp"

namespace oscilloplot {
namespace effects {

//==============================================================================
// Effect Parameters (Plain-Old-Data for cache efficiency)
//==============================================================================

struct RotationParams {
    enum class Mode : uint8_t { Off, Static, CW, CCW };
    Mode mode = Mode::Off;
    float staticAngle = 0.0f;      // degrees
    float speed = 5.0f;            // degrees per cycle
    float currentAngle = 0.0f;     // runtime state
};

struct FadeParams {
    bool xEnabled = false;
    bool yEnabled = false;
    bool shrinkEnabled = false;
    bool alternateXY = false;
    uint16_t xSteps = 10;
    uint16_t ySteps = 10;
    uint16_t shrinkSteps = 10;
    uint16_t xSpeed = 1;
    uint16_t ySpeed = 1;
    uint16_t shrinkSpeed = 1;
    uint32_t currentStep = 0;      // runtime state
    uint32_t repeatCounter = 0;
};

struct NoiseParams {
    bool xEnabled = false;
    bool yEnabled = false;
    float xAmplitude = 0.05f;
    float yAmplitude = 0.05f;
    uint32_t seed = 12345;         // PRNG state
};

struct WavyParams {
    bool xEnabled = false;
    bool yEnabled = false;
    float xAmplitude = 0.2f;
    float yAmplitude = 0.2f;
    float xFrequency = 10.0f;
    float yFrequency = 10.0f;
};

struct TremoloParams {
    enum class Waveform : uint8_t { Sine, Triangle, Square };
    bool enabled = false;
    float depth = 0.5f;            // 0-1
    float rate = 2.0f;             // Hz
    Waveform waveform = Waveform::Sine;
};

struct RingModParams {
    bool enabled = false;
    float carrierFreq = 200.0f;
    float mix = 0.5f;              // 0-1
};

struct EchoParams {
    bool enabled = false;
    uint8_t echoCount = 3;
    float decay = 0.7f;
    float delayPercent = 0.1f;     // fraction of pattern length
};

struct KaleidoscopeParams {
    bool enabled = false;
    uint8_t segments = 6;
    float rotationSpeed = 0.0f;
    float currentRotation = 0.0f;  // runtime state
};

//==============================================================================
// Fast PRNG for noise generation (xorshift32)
//==============================================================================

class FastRNG {
public:
    explicit FastRNG(uint32_t seed = 12345) : m_state(seed) {}

    OSCILLOPLOT_FORCE_INLINE uint32_t next() {
        m_state ^= m_state << 13;
        m_state ^= m_state >> 17;
        m_state ^= m_state << 5;
        return m_state;
    }

    // Returns float in [-1, 1]
    OSCILLOPLOT_FORCE_INLINE float nextFloat() {
        return static_cast<float>(next()) / 2147483648.0f - 1.0f;
    }

    // Returns float in [0, 1]
    OSCILLOPLOT_FORCE_INLINE float nextFloatUniform() {
        return static_cast<float>(next()) / 4294967296.0f;
    }

private:
    uint32_t m_state;
};

//==============================================================================
// Effect Processing Functions (Non-virtual, SIMD-optimized)
//==============================================================================

namespace processors {

/**
 * @brief Apply rotation effect (SIMD optimized)
 */
inline void processRotation(float* OSCILLOPLOT_RESTRICT x,
                           float* OSCILLOPLOT_RESTRICT y,
                           size_t count,
                           RotationParams& params,
                           float deltaTime) {
    if (params.mode == RotationParams::Mode::Off) return;

    float angleDeg;
    switch (params.mode) {
        case RotationParams::Mode::Static:
            angleDeg = params.staticAngle;
            break;
        case RotationParams::Mode::CW:
            params.currentAngle += params.speed * deltaTime;
            angleDeg = params.currentAngle;
            break;
        case RotationParams::Mode::CCW:
            params.currentAngle -= params.speed * deltaTime;
            angleDeg = params.currentAngle;
            break;
        default:
            return;
    }

    // Use lookup table for sin/cos
    const float phase = angleDeg / 360.0f;
    float sinA, cosA;
    dsp::SineLUT::sincos(phase, sinA, cosA);

    // SIMD rotation
    dsp::rotateXY(x, y, count, cosA, sinA);
}

/**
 * @brief Apply fade/shrink effect
 */
inline void processFade(float* OSCILLOPLOT_RESTRICT x,
                       float* OSCILLOPLOT_RESTRICT y,
                       size_t count,
                       FadeParams& params) {
    float xScale = 1.0f;
    float yScale = 1.0f;

    // Calculate shrink factor
    if (params.shrinkEnabled && params.shrinkSteps > 1) {
        uint32_t period = params.shrinkSteps * 2;
        float t = static_cast<float>(params.currentStep % period);
        if (t >= params.shrinkSteps) t = period - t;
        float scale = t / params.shrinkSteps;
        xScale *= scale;
        yScale *= scale;
    }

    // Calculate X fade
    if (params.xEnabled && params.xSteps > 1) {
        uint32_t period = params.xSteps * 2;
        float t = static_cast<float>(params.currentStep % period);
        if (t >= params.xSteps) t = period - t;
        xScale *= t / params.xSteps;
    }

    // Calculate Y fade
    if (params.yEnabled && params.ySteps > 1) {
        uint32_t period = params.ySteps * 2;
        float t = static_cast<float>(params.currentStep % period);
        if (t >= params.ySteps) t = period - t;
        yScale *= t / params.ySteps;
    }

    // Apply scaling (SIMD optimized)
    dsp::applyGain(x, count, xScale);
    dsp::applyGain(y, count, yScale);

    // Advance counter
    ++params.repeatCounter;
    uint16_t maxSpeed = params.xSpeed;
    if (params.ySpeed > maxSpeed) maxSpeed = params.ySpeed;
    if (params.shrinkSpeed > maxSpeed) maxSpeed = params.shrinkSpeed;

    if (params.repeatCounter >= maxSpeed) {
        params.repeatCounter = 0;
        ++params.currentStep;
    }
}

/**
 * @brief Apply noise effect (fast PRNG)
 */
inline void processNoise(float* OSCILLOPLOT_RESTRICT x,
                        float* OSCILLOPLOT_RESTRICT y,
                        size_t count,
                        NoiseParams& params) {
    FastRNG rng(params.seed);

    if (params.xEnabled) {
        for (size_t i = 0; i < count; ++i) {
            x[i] += rng.nextFloat() * params.xAmplitude;
        }
    }

    if (params.yEnabled) {
        for (size_t i = 0; i < count; ++i) {
            y[i] += rng.nextFloat() * params.yAmplitude;
        }
    }

    // Update seed for next call
    params.seed = rng.nextFloat() > 0 ? params.seed + 1 : params.seed + 2;
}

/**
 * @brief Apply wavy modulation effect
 */
inline void processWavy(float* OSCILLOPLOT_RESTRICT x,
                       float* OSCILLOPLOT_RESTRICT y,
                       size_t count,
                       const WavyParams& params,
                       float time) {
    if (!params.xEnabled && !params.yEnabled) return;

    const float invCount = 1.0f / static_cast<float>(count);

    for (size_t i = 0; i < count; ++i) {
        const float t = time + static_cast<float>(i) * invCount;

        if (params.xEnabled) {
            const float phase = t * params.xFrequency * dsp::INV_TWO_PI;
            x[i] += params.xAmplitude * dsp::SineLUT::sin(phase);
        }

        if (params.yEnabled) {
            const float phase = t * params.yFrequency * dsp::INV_TWO_PI;
            y[i] += params.yAmplitude * dsp::SineLUT::sin(phase);
        }
    }
}

/**
 * @brief Apply tremolo (amplitude modulation)
 */
inline void processTremolo(float* OSCILLOPLOT_RESTRICT x,
                          float* OSCILLOPLOT_RESTRICT y,
                          size_t count,
                          const TremoloParams& params,
                          float time) {
    if (!params.enabled) return;

    const float phase = time * params.rate;
    float modValue;

    switch (params.waveform) {
        case TremoloParams::Waveform::Sine:
            modValue = (dsp::SineLUT::sin(phase) + 1.0f) * 0.5f;
            break;
        case TremoloParams::Waveform::Triangle: {
            float t = phase - static_cast<int>(phase);
            modValue = (t < 0.5f) ? (t * 2.0f) : (2.0f - t * 2.0f);
            break;
        }
        case TremoloParams::Waveform::Square:
            modValue = (dsp::SineLUT::sin(phase) >= 0.0f) ? 1.0f : 0.0f;
            break;
    }

    const float amplitude = 1.0f - params.depth * (1.0f - modValue);

    // Apply amplitude modulation (SIMD)
    dsp::applyGain(x, count, amplitude);
    dsp::applyGain(y, count, amplitude);
}

/**
 * @brief Apply ring modulation
 */
inline void processRingMod(float* OSCILLOPLOT_RESTRICT x,
                          float* OSCILLOPLOT_RESTRICT y,
                          size_t count,
                          const RingModParams& params,
                          float time) {
    if (!params.enabled) return;

    const float invCount = 1.0f / static_cast<float>(count);
    const float wetMix = params.mix;
    const float dryMix = 1.0f - params.mix;

    for (size_t i = 0; i < count; ++i) {
        const float t = time + static_cast<float>(i) * invCount;
        const float phase = t * params.carrierFreq * dsp::INV_TWO_PI;
        const float carrier = dsp::SineLUT::sin(phase);

        x[i] = x[i] * dryMix + x[i] * carrier * wetMix;
        y[i] = y[i] * dryMix + y[i] * carrier * wetMix;
    }
}

} // namespace processors

//==============================================================================
// Main Effect Processor (Data-Oriented)
//==============================================================================

/**
 * @brief High-performance effect processor
 *
 * Features:
 * - All effect parameters in contiguous memory
 * - No virtual function calls
 * - Batch processing with SIMD
 * - Zero allocation during processing
 */
class EffectProcessor {
public:
    // Effect parameters (all POD, contiguous)
    RotationParams rotation;
    FadeParams fade;
    NoiseParams noise;
    WavyParams wavy;
    TremoloParams tremolo;
    RingModParams ringMod;
    EchoParams echo;
    KaleidoscopeParams kaleidoscope;

    /**
     * @brief Process all enabled effects on pattern data
     *
     * @param x X coordinate array (modified in-place)
     * @param y Y coordinate array (modified in-place)
     * @param count Number of points
     * @param time Current time in seconds
     * @param deltaTime Time since last frame
     */
    void process(float* OSCILLOPLOT_RESTRICT x,
                float* OSCILLOPLOT_RESTRICT y,
                size_t count,
                float time,
                float deltaTime) {
        // Process effects in optimal order
        // (rotation first, then scaling effects, then modulation)

        processors::processRotation(x, y, count, rotation, deltaTime);
        processors::processFade(x, y, count, fade);
        processors::processNoise(x, y, count, noise);
        processors::processWavy(x, y, count, wavy, time);
        processors::processTremolo(x, y, count, tremolo, time);
        processors::processRingMod(x, y, count, ringMod, time);

        // Note: Echo and Kaleidoscope modify array size, handle separately
    }

    /**
     * @brief Reset all runtime state
     */
    void reset() {
        rotation.currentAngle = 0.0f;
        fade.currentStep = 0;
        fade.repeatCounter = 0;
        kaleidoscope.currentRotation = 0.0f;
    }

    /**
     * @brief Disable all effects
     */
    void disableAll() {
        rotation.mode = RotationParams::Mode::Off;
        fade.xEnabled = false;
        fade.yEnabled = false;
        fade.shrinkEnabled = false;
        noise.xEnabled = false;
        noise.yEnabled = false;
        wavy.xEnabled = false;
        wavy.yEnabled = false;
        tremolo.enabled = false;
        ringMod.enabled = false;
        echo.enabled = false;
        kaleidoscope.enabled = false;
    }
};

//==============================================================================
// Processing Context (All state for one frame)
//==============================================================================

/**
 * @brief Frame processing context with pre-allocated scratch buffers
 */
template<size_t MaxPoints = 65536>
class ProcessingContext {
public:
    // Scratch buffers for intermediate processing
    StaticPattern<MaxPoints> scratchPattern;

    // Buffer pool for effects that need temporary storage
    AudioBufferPool<MaxPoints, 4> bufferPool;

    // Stack allocator for frame-temporary allocations
    StackAllocator<MaxPoints * sizeof(float) * 4> frameAllocator;

    // Effect processor
    EffectProcessor effects;

    // Timing
    float currentTime = 0.0f;
    float deltaTime = 0.0f;

    /**
     * @brief Begin new processing frame
     */
    void beginFrame(float dt) {
        deltaTime = dt;
        currentTime += dt;
        frameAllocator.reset();
    }

    /**
     * @brief Process pattern through effect chain
     */
    void processPattern(float* x, float* y, size_t count) {
        effects.process(x, y, count, currentTime, deltaTime);
    }
};

} // namespace effects
} // namespace oscilloplot

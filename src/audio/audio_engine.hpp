#pragma once

/**
 * @file audio_engine.hpp
 * @brief High-performance audio engine with real-time effect processing
 *
 * Optimizations:
 * - SIMD batch processing for effects
 * - Lock-free parameter updates from UI thread
 * - Zero allocation in audio callback
 * - Cache-aligned scratch buffers
 * - Sine LUT for fast transcendentals
 */

#include "data/pattern.hpp"
#include "utils/dsp_core.hpp"
#include <portaudio.h>
#include <vector>
#include <atomic>
#include <functional>
#include <cstdint>

namespace oscilloplot {

//==============================================================================
// Effect Parameters - Cache-line aligned, atomic-friendly
//==============================================================================

struct alignas(64) EffectParams {
    // Rotation
    enum class RotationMode : uint8_t { Off = 0, Static, CW, CCW };
    std::atomic<RotationMode> rotationMode{RotationMode::Off};
    std::atomic<float> rotationAngle{0.0f};     // Static angle (degrees)
    std::atomic<float> rotationSpeed{5.0f};     // Degrees per pattern cycle

    // Fade/Shrink
    std::atomic<bool> fadeXEnabled{false};
    std::atomic<bool> fadeYEnabled{false};
    std::atomic<bool> shrinkEnabled{false};
    std::atomic<uint16_t> fadeXSteps{10};
    std::atomic<uint16_t> fadeYSteps{10};
    std::atomic<uint16_t> shrinkSteps{10};
    std::atomic<uint16_t> fadeXSpeed{1};       // Repeats per fade step
    std::atomic<uint16_t> fadeYSpeed{1};
    std::atomic<uint16_t> shrinkSpeed{1};
    std::atomic<bool> alternateXYFade{false};  // Alternate X then Y fade (instead of simultaneous)

    // Noise
    std::atomic<bool> noiseXEnabled{false};
    std::atomic<bool> noiseYEnabled{false};
    std::atomic<float> noiseXAmount{0.05f};
    std::atomic<float> noiseYAmount{0.05f};

    // Wavy
    std::atomic<bool> wavyXEnabled{false};
    std::atomic<bool> wavyYEnabled{false};
    std::atomic<float> wavyXAmplitude{0.2f};
    std::atomic<float> wavyYAmplitude{0.2f};
    std::atomic<float> wavyXFrequency{3.0f};
    std::atomic<float> wavyYFrequency{3.0f};

    // Tremolo
    std::atomic<bool> tremoloEnabled{false};
    std::atomic<float> tremoloDepth{0.5f};      // 0-1
    std::atomic<float> tremoloRate{2.0f};       // Hz
    enum class TremoloWave : uint8_t { Sine = 0, Triangle, Square };
    std::atomic<TremoloWave> tremoloWaveform{TremoloWave::Sine};

    // Ring Modulation
    std::atomic<bool> ringModEnabled{false};
    std::atomic<float> ringModFreq{200.0f};     // Hz
    std::atomic<float> ringModMix{0.5f};        // 0-1

    // Mirror
    std::atomic<bool> mirrorX{false};
    std::atomic<bool> mirrorY{false};
    std::atomic<bool> mirrorXY{false};          // Diagonal mirror (creates 4 copies)

    //--------------------------------------------------------------------------
    // NEW EFFECTS
    //--------------------------------------------------------------------------

    // Echo/Delay
    std::atomic<bool> echoEnabled{false};
    std::atomic<uint8_t> echoCount{3};          // 1-10 echoes
    std::atomic<float> echoDecay{0.5f};         // 0.1-0.95 decay factor
    std::atomic<float> echoDelay{0.2f};         // 0-0.5 (fraction of pattern)

    // Kaleidoscope
    std::atomic<bool> kaleidoscopeEnabled{false};
    std::atomic<uint8_t> kaleidoscopeSections{6};    // 2-12 sections
    std::atomic<bool> kaleidoscopeMirror{true};      // Mirror alternating sections
    std::atomic<float> kaleidoscopeRotation{0.0f};   // Rotation speed deg/cycle

    // Distortion
    std::atomic<bool> distortionEnabled{false};
    enum class DistortionType : uint8_t { SoftClip = 0, HardClip, Fold };
    std::atomic<DistortionType> distortionType{DistortionType::SoftClip};
    std::atomic<float> distortionThreshold{0.8f};    // 0.1-2.0
    std::atomic<float> distortionDrive{1.5f};        // 1.0-5.0 pre-gain
};

//==============================================================================
// Audio Engine with Integrated Effects
//==============================================================================

class AudioEngine {
public:
    static constexpr size_t BUFFER_FRAMES = 256;
    static constexpr size_t MAX_PATTERN_SIZE = 65536;
    static constexpr size_t MAX_ECHO_SAMPLES = 32768;  // For echo delay buffer

    AudioEngine();
    ~AudioEngine();

    // Non-copyable
    AudioEngine(const AudioEngine&) = delete;
    AudioEngine& operator=(const AudioEngine&) = delete;

    bool init();
    void shutdown();

    // Playback control
    void play();
    void stop();
    void pause();
    void resume();
    bool isPlaying() const { return m_isPlaying.load(std::memory_order_relaxed); }

    // Update (call each frame)
    void update();

    // Set the pattern to play
    void setPattern(const Pattern& pattern);

    // Audio parameters
    void setSampleRate(int baseRate, int multiplier);
    int getActualSampleRate() const { return m_actualSampleRate; }

    void setDuration(float seconds) { m_duration = seconds; }
    void setPatternRepeats(int repeats) { m_patternRepeats = repeats; }

    // Get current playback position (0.0 - 1.0)
    float getPlaybackPosition() const;

    // Get current XY values for visualization
    void getCurrentXY(float& x, float& y) const;

    // Effect parameters - direct access for UI binding
    EffectParams& effects() { return m_effects; }
    const EffectParams& effects() const { return m_effects; }

    // Visualization callback
    using VisualizationCallback = std::function<void(const float* x, const float* y, int count)>;
    void setVisualizationCallback(VisualizationCallback callback) {
        m_visualizationCallback = callback;
    }

private:
    static int audioCallback(const void* input, void* output,
                            unsigned long frameCount,
                            const PaStreamCallbackTimeInfo* timeInfo,
                            PaStreamCallbackFlags statusFlags,
                            void* userData);

    int processAudio(float* output, unsigned long frameCount);

    // Effect processing (SIMD optimized, called from audio thread)
    void applyEffects(float* OSCILLOPLOT_RESTRICT x,
                      float* OSCILLOPLOT_RESTRICT y,
                      size_t count);

    // Individual effect processors
    void applyEcho(float* OSCILLOPLOT_RESTRICT x,
                   float* OSCILLOPLOT_RESTRICT y,
                   size_t count);

    void applyKaleidoscope(float* OSCILLOPLOT_RESTRICT x,
                           float* OSCILLOPLOT_RESTRICT y,
                           size_t count);

    void applyDistortion(float* OSCILLOPLOT_RESTRICT x,
                         float* OSCILLOPLOT_RESTRICT y,
                         size_t count);

    // PortAudio
    PaStream* m_stream = nullptr;
    bool m_paInitialized = false;

    // Pattern data (Structure-of-Arrays for SIMD)
    alignas(32) float m_patternX[MAX_PATTERN_SIZE];
    alignas(32) float m_patternY[MAX_PATTERN_SIZE];
    std::atomic<size_t> m_patternSize{0};

    // Scratch buffers for effect processing (pre-allocated)
    alignas(32) float m_scratchX[BUFFER_FRAMES];
    alignas(32) float m_scratchY[BUFFER_FRAMES];

    // Echo delay buffers (circular)
    alignas(32) float m_echoBufferX[MAX_ECHO_SAMPLES];
    alignas(32) float m_echoBufferY[MAX_ECHO_SAMPLES];
    size_t m_echoWritePos = 0;

    // Playback state
    std::atomic<size_t> m_playbackPosition{0};
    std::atomic<bool> m_isPlaying{false};
    std::atomic<bool> m_isPaused{false};

    // Parameters
    int m_baseSampleRate = 1000;
    int m_multiplier = 100;
    int m_actualSampleRate = 100000;
    float m_duration = 15.0f;
    int m_patternRepeats = 200;

    // Effect parameters (cache-line aligned)
    EffectParams m_effects;

    // Effect runtime state (audio thread only)
    float m_currentRotationAngle = 0.0f;
    float m_kaleidoscopeAngle = 0.0f;
    uint32_t m_fadeStep = 0;
    uint32_t m_noiseState = 12345;  // xorshift PRNG state
    float m_effectTime = 0.0f;      // Time accumulator for time-based effects
    uint32_t m_patternCycleCount = 0;

    // Visualization
    VisualizationCallback m_visualizationCallback;
    std::atomic<float> m_currentX{0.0f};
    std::atomic<float> m_currentY{0.0f};

    // Oscilloscope visualization ring buffer (lock-free)
    // Multiple persistence layers for phosphor effect
    static constexpr size_t VIZ_BUFFER_SIZE = 16384;
    static constexpr size_t VIZ_HISTORY_LAYERS = 8;  // For phosphor persistence
    alignas(32) float m_vizBufferX[VIZ_BUFFER_SIZE];
    alignas(32) float m_vizBufferY[VIZ_BUFFER_SIZE];
    std::atomic<size_t> m_vizWritePos{0};

public:
    // Get visualization data for oscilloscope display
    size_t getVizBufferSize() const { return VIZ_BUFFER_SIZE; }

    // Copy recent samples for display (called from UI thread)
    size_t getVizSamples(float* outX, float* outY, size_t maxSamples) const {
        size_t writePos = m_vizWritePos.load(std::memory_order_acquire);
        size_t count = (maxSamples < VIZ_BUFFER_SIZE) ? maxSamples : VIZ_BUFFER_SIZE;

        // Copy samples starting from oldest
        size_t startPos = (writePos >= count) ? (writePos - count) : (VIZ_BUFFER_SIZE - count + writePos);

        for (size_t i = 0; i < count; ++i) {
            size_t idx = (startPos + i) % VIZ_BUFFER_SIZE;
            outX[i] = m_vizBufferX[idx];
            outY[i] = m_vizBufferY[idx];
        }
        return count;
    }

    // Get samples with layer offset for persistence effect
    size_t getVizSamplesWithOffset(float* outX, float* outY, size_t maxSamples, size_t offset) const {
        size_t writePos = m_vizWritePos.load(std::memory_order_acquire);
        if (writePos < offset) return 0;

        size_t adjustedPos = writePos - offset;
        size_t count = (maxSamples < VIZ_BUFFER_SIZE) ? maxSamples : VIZ_BUFFER_SIZE;
        if (count > adjustedPos) count = adjustedPos;

        size_t startPos = (adjustedPos >= count) ? (adjustedPos - count) : 0;

        for (size_t i = 0; i < count; ++i) {
            size_t idx = (startPos + i) % VIZ_BUFFER_SIZE;
            outX[i] = m_vizBufferX[idx];
            outY[i] = m_vizBufferY[idx];
        }
        return count;
    }
};

} // namespace oscilloplot

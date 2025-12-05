#pragma once

/**
 * @file realtime_audio.hpp
 * @brief Real-time audio engine with embedded-style optimizations
 *
 * Design for real-time audio:
 * - No memory allocation in audio callback
 * - Lock-free communication with main thread
 * - Pre-computed audio buffers
 * - SIMD-optimized sample generation
 * - Cache-friendly data layout
 */

#include <portaudio.h>
#include <atomic>
#include <cstdint>
#include "utils/dsp_core.hpp"
#include "utils/lock_free_queue.hpp"
#include "utils/memory_pool.hpp"
#include "effects/effect_processor.hpp"

namespace oscilloplot {
namespace audio {

//==============================================================================
// Audio Configuration (Compile-time constants)
//==============================================================================

struct AudioConfig {
    static constexpr int DEFAULT_SAMPLE_RATE = 96000;
    static constexpr int MIN_SAMPLE_RATE = 44100;
    static constexpr int MAX_SAMPLE_RATE = 192000;

    static constexpr size_t BUFFER_FRAMES = 256;      // Low latency
    static constexpr size_t RING_BUFFER_SIZE = 65536; // Power of 2
    static constexpr size_t MAX_PATTERN_POINTS = 16384;
    static constexpr size_t VIZ_BUFFER_SIZE = 8192;   // Visualization samples

    static constexpr int NUM_CHANNELS = 2;            // Stereo: X=L, Y=R
};

//==============================================================================
// Audio Commands (Lock-free main->audio thread communication)
//==============================================================================

enum class AudioCommand : uint8_t {
    None,
    Play,
    Stop,
    Pause,
    Resume,
    SetPattern,
    SetSampleRate,
    SetEffects,
};

struct CommandData {
    AudioCommand command = AudioCommand::None;
    union {
        struct {
            int sampleRate;
            int multiplier;
        } rateData;
        struct {
            float* patternX;
            float* patternY;
            size_t patternSize;
            int repeats;
        } patternData;
    };
};

//==============================================================================
// Visualization Data (Lock-free audio->main thread)
//==============================================================================

struct VizSample {
    float x;
    float y;
};

//==============================================================================
// Audio State Machine
//==============================================================================

enum class PlaybackState : uint8_t {
    Stopped,
    Playing,
    Paused,
};

//==============================================================================
// Real-Time Audio Engine
//==============================================================================

class RealtimeAudioEngine {
public:
    RealtimeAudioEngine();
    ~RealtimeAudioEngine();

    // Non-copyable
    RealtimeAudioEngine(const RealtimeAudioEngine&) = delete;
    RealtimeAudioEngine& operator=(const RealtimeAudioEngine&) = delete;

    //--------------------------------------------------------------------------
    // Lifecycle
    //--------------------------------------------------------------------------

    bool init(int sampleRate = AudioConfig::DEFAULT_SAMPLE_RATE);
    void shutdown();
    bool isInitialized() const { return m_initialized; }

    //--------------------------------------------------------------------------
    // Playback Control (called from main thread)
    //--------------------------------------------------------------------------

    void play();
    void stop();
    void pause();
    void resume();

    PlaybackState getState() const {
        return m_state.load(std::memory_order_acquire);
    }

    bool isPlaying() const { return getState() == PlaybackState::Playing; }

    //--------------------------------------------------------------------------
    // Pattern Management (called from main thread)
    //--------------------------------------------------------------------------

    /**
     * @brief Set pattern data for playback
     * Thread-safe: copies data to internal buffer
     */
    void setPattern(const float* x, const float* y, size_t count, int repeats);

    /**
     * @brief Get current playback position (0.0 - 1.0)
     */
    float getPlaybackPosition() const;

    //--------------------------------------------------------------------------
    // Audio Parameters (called from main thread)
    //--------------------------------------------------------------------------

    void setSampleRate(int baseRate, int multiplier);
    int getActualSampleRate() const { return m_actualSampleRate.load(); }

    //--------------------------------------------------------------------------
    // Visualization (called from main thread)
    //--------------------------------------------------------------------------

    /**
     * @brief Get visualization samples (non-blocking)
     * @return Number of samples retrieved
     */
    size_t getVizSamples(VizSample* output, size_t maxCount);

    /**
     * @brief Get current X,Y position for display
     */
    void getCurrentXY(float& x, float& y) const {
        x = m_currentX.load(std::memory_order_relaxed);
        y = m_currentY.load(std::memory_order_relaxed);
    }

    //--------------------------------------------------------------------------
    // Effect Control
    //--------------------------------------------------------------------------

    effects::EffectProcessor& getEffects() { return m_effects; }

private:
    //--------------------------------------------------------------------------
    // Audio Callback (real-time thread)
    //--------------------------------------------------------------------------

    static int audioCallback(const void* input, void* output,
                            unsigned long frameCount,
                            const PaStreamCallbackTimeInfo* timeInfo,
                            PaStreamCallbackFlags statusFlags,
                            void* userData);

    int processAudio(float* output, unsigned long frameCount);

    //--------------------------------------------------------------------------
    // Command Processing
    //--------------------------------------------------------------------------

    void processCommands();

    //--------------------------------------------------------------------------
    // Internal State
    //--------------------------------------------------------------------------

    // PortAudio
    PaStream* m_stream = nullptr;
    bool m_initialized = false;

    // Playback state (atomic for thread safety)
    std::atomic<PlaybackState> m_state{PlaybackState::Stopped};
    std::atomic<int> m_actualSampleRate{AudioConfig::DEFAULT_SAMPLE_RATE};

    // Pattern data (double-buffered for lock-free updates)
    struct PatternBuffer {
        OSCILLOPLOT_SIMD_ALIGN float x[AudioConfig::MAX_PATTERN_POINTS];
        OSCILLOPLOT_SIMD_ALIGN float y[AudioConfig::MAX_PATTERN_POINTS];
        size_t size = 0;
        int repeats = 1;
    };

    PatternBuffer m_patternBuffers[2];
    std::atomic<int> m_activeBuffer{0};
    std::atomic<int> m_pendingBuffer{-1};

    // Playback position
    std::atomic<size_t> m_playbackPos{0};
    std::atomic<size_t> m_totalSamples{0};

    // Current output for visualization
    std::atomic<float> m_currentX{0.0f};
    std::atomic<float> m_currentY{0.0f};

    // Lock-free queues
    SPSCQueue<CommandData, 64> m_commandQueue;
    SPSCQueue<VizSample, AudioConfig::VIZ_BUFFER_SIZE> m_vizQueue;

    // Effects processor
    effects::EffectProcessor m_effects;

    // Processing scratch buffer
    OSCILLOPLOT_SIMD_ALIGN float m_scratchX[AudioConfig::BUFFER_FRAMES * 2];
    OSCILLOPLOT_SIMD_ALIGN float m_scratchY[AudioConfig::BUFFER_FRAMES * 2];
};

//==============================================================================
// Implementation
//==============================================================================

inline RealtimeAudioEngine::RealtimeAudioEngine() {
    // Initialize pattern buffers to zero
    for (auto& buf : m_patternBuffers) {
        std::fill(std::begin(buf.x), std::end(buf.x), 0.0f);
        std::fill(std::begin(buf.y), std::end(buf.y), 0.0f);
    }
}

inline RealtimeAudioEngine::~RealtimeAudioEngine() {
    shutdown();
}

inline bool RealtimeAudioEngine::init(int sampleRate) {
    PaError err = Pa_Initialize();
    if (err != paNoError) return false;

    m_actualSampleRate.store(sampleRate);

    err = Pa_OpenDefaultStream(
        &m_stream,
        0,                          // no input
        AudioConfig::NUM_CHANNELS,  // stereo output
        paFloat32,
        sampleRate,
        AudioConfig::BUFFER_FRAMES,
        audioCallback,
        this
    );

    if (err != paNoError) {
        Pa_Terminate();
        return false;
    }

    m_initialized = true;
    return true;
}

inline void RealtimeAudioEngine::shutdown() {
    stop();

    if (m_stream) {
        Pa_CloseStream(m_stream);
        m_stream = nullptr;
    }

    if (m_initialized) {
        Pa_Terminate();
        m_initialized = false;
    }
}

inline void RealtimeAudioEngine::play() {
    if (!m_initialized || !m_stream) return;

    m_playbackPos.store(0);
    m_state.store(PlaybackState::Playing, std::memory_order_release);
    Pa_StartStream(m_stream);
}

inline void RealtimeAudioEngine::stop() {
    m_state.store(PlaybackState::Stopped, std::memory_order_release);
    m_playbackPos.store(0);

    if (m_stream && Pa_IsStreamActive(m_stream)) {
        Pa_StopStream(m_stream);
    }
}

inline void RealtimeAudioEngine::pause() {
    if (getState() == PlaybackState::Playing) {
        m_state.store(PlaybackState::Paused, std::memory_order_release);
        if (m_stream) Pa_StopStream(m_stream);
    }
}

inline void RealtimeAudioEngine::resume() {
    if (getState() == PlaybackState::Paused) {
        m_state.store(PlaybackState::Playing, std::memory_order_release);
        if (m_stream) Pa_StartStream(m_stream);
    }
}

inline void RealtimeAudioEngine::setPattern(const float* x, const float* y,
                                            size_t count, int repeats) {
    // Copy to inactive buffer
    int activeIdx = m_activeBuffer.load(std::memory_order_acquire);
    int newIdx = 1 - activeIdx;

    auto& buf = m_patternBuffers[newIdx];
    size_t toCopy = (count <= AudioConfig::MAX_PATTERN_POINTS)
                    ? count : AudioConfig::MAX_PATTERN_POINTS;

    std::copy(x, x + toCopy, buf.x);
    std::copy(y, y + toCopy, buf.y);
    buf.size = toCopy;
    buf.repeats = repeats;

    // Calculate total samples
    m_totalSamples.store(toCopy * repeats);

    // Signal buffer swap (audio thread will pick it up)
    m_pendingBuffer.store(newIdx, std::memory_order_release);
}

inline float RealtimeAudioEngine::getPlaybackPosition() const {
    size_t total = m_totalSamples.load(std::memory_order_relaxed);
    if (total == 0) return 0.0f;
    size_t pos = m_playbackPos.load(std::memory_order_relaxed);
    return static_cast<float>(pos) / static_cast<float>(total);
}

inline void RealtimeAudioEngine::setSampleRate(int baseRate, int multiplier) {
    int newRate = baseRate * multiplier;

    // Clamp to valid range
    if (newRate < AudioConfig::MIN_SAMPLE_RATE)
        newRate = AudioConfig::MIN_SAMPLE_RATE;
    if (newRate > AudioConfig::MAX_SAMPLE_RATE)
        newRate = AudioConfig::MAX_SAMPLE_RATE;

    // Reinitialize stream with new rate
    bool wasPlaying = isPlaying();
    stop();

    if (m_stream) {
        Pa_CloseStream(m_stream);
    }

    m_actualSampleRate.store(newRate);

    Pa_OpenDefaultStream(
        &m_stream, 0, AudioConfig::NUM_CHANNELS,
        paFloat32, newRate, AudioConfig::BUFFER_FRAMES,
        audioCallback, this
    );

    if (wasPlaying) play();
}

inline size_t RealtimeAudioEngine::getVizSamples(VizSample* output, size_t maxCount) {
    return m_vizQueue.popBatch(output, maxCount);
}

inline int RealtimeAudioEngine::audioCallback(
    const void* input, void* output,
    unsigned long frameCount,
    const PaStreamCallbackTimeInfo* timeInfo,
    PaStreamCallbackFlags statusFlags,
    void* userData)
{
    (void)input;
    (void)timeInfo;
    (void)statusFlags;

    auto* engine = static_cast<RealtimeAudioEngine*>(userData);
    return engine->processAudio(static_cast<float*>(output), frameCount);
}

inline int RealtimeAudioEngine::processAudio(float* output, unsigned long frameCount) {
    // Check for buffer swap
    int pending = m_pendingBuffer.load(std::memory_order_acquire);
    if (pending >= 0) {
        m_activeBuffer.store(pending, std::memory_order_release);
        m_pendingBuffer.store(-1, std::memory_order_release);
    }

    PlaybackState state = m_state.load(std::memory_order_acquire);

    if (state != PlaybackState::Playing) {
        // Output silence
        std::fill(output, output + frameCount * AudioConfig::NUM_CHANNELS, 0.0f);
        return paContinue;
    }

    int bufIdx = m_activeBuffer.load(std::memory_order_acquire);
    const auto& pattern = m_patternBuffers[bufIdx];

    if (pattern.size == 0) {
        std::fill(output, output + frameCount * AudioConfig::NUM_CHANNELS, 0.0f);
        return paContinue;
    }

    size_t totalSamples = pattern.size * pattern.repeats;
    size_t pos = m_playbackPos.load(std::memory_order_relaxed);

    for (unsigned long i = 0; i < frameCount; ++i) {
        if (pos >= totalSamples) {
            // End of playback
            m_state.store(PlaybackState::Stopped, std::memory_order_release);
            std::fill(output + i * 2, output + frameCount * 2, 0.0f);
            return paComplete;
        }

        size_t patternIdx = pos % pattern.size;
        float x = pattern.x[patternIdx];
        float y = pattern.y[patternIdx];

        // Output: Left = X, Right = Y
        output[i * 2] = x;
        output[i * 2 + 1] = y;

        // Update visualization (best effort, non-blocking)
        VizSample viz{x, y};
        m_vizQueue.push(viz);

        // Update current position
        m_currentX.store(x, std::memory_order_relaxed);
        m_currentY.store(y, std::memory_order_relaxed);

        ++pos;
    }

    m_playbackPos.store(pos, std::memory_order_relaxed);
    return paContinue;
}

} // namespace audio
} // namespace oscilloplot

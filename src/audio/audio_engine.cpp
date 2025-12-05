#include "audio_engine.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <cstring>

namespace oscilloplot {

//==============================================================================
// Fast PRNG (xorshift32) - inline for audio thread
//==============================================================================

OSCILLOPLOT_FORCE_INLINE uint32_t xorshift32(uint32_t& state) {
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
}

OSCILLOPLOT_FORCE_INLINE float randomFloat(uint32_t& state) {
    return static_cast<float>(xorshift32(state)) / 4294967296.0f * 2.0f - 1.0f;
}

//==============================================================================
// Fast math approximations for audio thread
//==============================================================================

// Fast tanh approximation (Pade approximant) - for soft clipping
OSCILLOPLOT_FORCE_INLINE float fastTanh(float x) {
    if (x < -3.0f) return -1.0f;
    if (x > 3.0f) return 1.0f;
    float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

// Fast atan2 approximation for kaleidoscope
OSCILLOPLOT_FORCE_INLINE float fastAtan2(float y, float x) {
    constexpr float PI = 3.14159265358979323846f;
    constexpr float PI_2 = PI / 2.0f;

    if (x == 0.0f) {
        if (y > 0.0f) return PI_2;
        if (y < 0.0f) return -PI_2;
        return 0.0f;
    }

    float abs_y = (y < 0.0f) ? -y : y;
    float angle;

    if (x >= 0.0f) {
        float r = (x - abs_y) / (x + abs_y + 1e-10f);
        angle = 0.1963f * r * r * r - 0.9817f * r + PI / 4.0f;
    } else {
        float r = (x + abs_y) / (abs_y - x + 1e-10f);
        angle = 0.1963f * r * r * r - 0.9817f * r + 3.0f * PI / 4.0f;
    }

    return (y < 0.0f) ? -angle : angle;
}

//==============================================================================
// AudioEngine Implementation
//==============================================================================

AudioEngine::AudioEngine() {
    // Zero-initialize pattern buffers
    std::memset(m_patternX, 0, sizeof(m_patternX));
    std::memset(m_patternY, 0, sizeof(m_patternY));
    std::memset(m_scratchX, 0, sizeof(m_scratchX));
    std::memset(m_scratchY, 0, sizeof(m_scratchY));
    std::memset(m_vizBufferX, 0, sizeof(m_vizBufferX));
    std::memset(m_vizBufferY, 0, sizeof(m_vizBufferY));
    std::memset(m_echoBufferX, 0, sizeof(m_echoBufferX));
    std::memset(m_echoBufferY, 0, sizeof(m_echoBufferY));
}

AudioEngine::~AudioEngine() {
    shutdown();
}

bool AudioEngine::init() {
    PaError err = Pa_Initialize();
    if (err != paNoError) {
        std::cerr << "PortAudio init error: " << Pa_GetErrorText(err) << std::endl;
        return false;
    }
    m_paInitialized = true;

    err = Pa_OpenDefaultStream(
        &m_stream,
        0,                      // no input
        2,                      // stereo output
        paFloat32,
        m_actualSampleRate,
        BUFFER_FRAMES,
        audioCallback,
        this
    );

    if (err != paNoError) {
        std::cerr << "PortAudio stream error: " << Pa_GetErrorText(err) << std::endl;
        return false;
    }

    return true;
}

void AudioEngine::shutdown() {
    stop();

    if (m_stream) {
        Pa_CloseStream(m_stream);
        m_stream = nullptr;
    }

    if (m_paInitialized) {
        Pa_Terminate();
        m_paInitialized = false;
    }
}

void AudioEngine::setPattern(const Pattern& pattern) {
    // Copy to internal SoA buffers
    size_t count = std::min(pattern.size(), MAX_PATTERN_SIZE);

    // Use memcpy for efficiency
    std::memcpy(m_patternX, pattern.x.data(), count * sizeof(float));
    std::memcpy(m_patternY, pattern.y.data(), count * sizeof(float));

    m_patternSize.store(count, std::memory_order_release);
    m_playbackPosition.store(0, std::memory_order_relaxed);
}

void AudioEngine::setSampleRate(int baseRate, int multiplier) {
    m_baseSampleRate = baseRate;
    m_multiplier = multiplier;
    m_actualSampleRate = baseRate * multiplier;

    if (m_stream) {
        bool wasPlaying = m_isPlaying.load();
        stop();
        Pa_CloseStream(m_stream);

        PaError err = Pa_OpenDefaultStream(
            &m_stream, 0, 2, paFloat32,
            m_actualSampleRate, BUFFER_FRAMES,
            audioCallback, this
        );

        if (err != paNoError) {
            std::cerr << "Failed to reopen stream: " << Pa_GetErrorText(err) << std::endl;
        } else if (wasPlaying) {
            play();
        }
    }
}

void AudioEngine::play() {
    if (!m_stream || m_patternSize.load() == 0) return;

    m_playbackPosition.store(0, std::memory_order_relaxed);
    m_effectTime = 0.0f;
    m_currentRotationAngle = 0.0f;
    m_kaleidoscopeAngle = 0.0f;
    m_fadeStep = 0;
    m_patternCycleCount = 0;
    m_noiseState = 12345;
    m_echoWritePos = 0;

    // Clear echo buffer
    std::memset(m_echoBufferX, 0, sizeof(m_echoBufferX));
    std::memset(m_echoBufferY, 0, sizeof(m_echoBufferY));

    m_isPlaying.store(true, std::memory_order_release);
    m_isPaused.store(false, std::memory_order_release);

    PaError err = Pa_StartStream(m_stream);
    if (err != paNoError) {
        std::cerr << "PortAudio start error: " << Pa_GetErrorText(err) << std::endl;
        m_isPlaying.store(false, std::memory_order_release);
    }
}

void AudioEngine::stop() {
    m_isPlaying.store(false, std::memory_order_release);
    m_isPaused.store(false, std::memory_order_release);
    m_playbackPosition.store(0, std::memory_order_relaxed);

    if (m_stream && Pa_IsStreamActive(m_stream)) {
        Pa_StopStream(m_stream);
    }
}

void AudioEngine::pause() {
    if (m_isPlaying.load() && !m_isPaused.load()) {
        m_isPaused.store(true, std::memory_order_release);
        if (m_stream) Pa_StopStream(m_stream);
    }
}

void AudioEngine::resume() {
    if (m_isPaused.load()) {
        m_isPaused.store(false, std::memory_order_release);
        if (m_stream) Pa_StartStream(m_stream);
    }
}

void AudioEngine::update() {
    // Called from main thread - nothing needed currently
}

float AudioEngine::getPlaybackPosition() const {
    size_t patternSize = m_patternSize.load(std::memory_order_relaxed);
    if (patternSize == 0) return 0.0f;

    size_t pos = m_playbackPosition.load(std::memory_order_relaxed);
    // Return position within current pattern cycle (0-1)
    return static_cast<float>(pos % patternSize) / static_cast<float>(patternSize);
}

void AudioEngine::getCurrentXY(float& x, float& y) const {
    x = m_currentX.load(std::memory_order_relaxed);
    y = m_currentY.load(std::memory_order_relaxed);
}

int AudioEngine::audioCallback(const void* input, void* output,
                               unsigned long frameCount,
                               const PaStreamCallbackTimeInfo* timeInfo,
                               PaStreamCallbackFlags statusFlags,
                               void* userData) {
    (void)input;
    (void)timeInfo;
    (void)statusFlags;

    AudioEngine* engine = static_cast<AudioEngine*>(userData);
    return engine->processAudio(static_cast<float*>(output), frameCount);
}

int AudioEngine::processAudio(float* output, unsigned long frameCount) {
    const bool playing = m_isPlaying.load(std::memory_order_acquire);
    const bool paused = m_isPaused.load(std::memory_order_acquire);

    if (!playing || paused) {
        std::memset(output, 0, frameCount * 2 * sizeof(float));
        return paContinue;
    }

    const size_t patternSize = m_patternSize.load(std::memory_order_acquire);
    if (patternSize == 0) {
        std::memset(output, 0, frameCount * 2 * sizeof(float));
        return paContinue;
    }

    size_t pos = m_playbackPosition.load(std::memory_order_relaxed);

    // Process in chunks up to BUFFER_FRAMES
    size_t processed = 0;
    while (processed < frameCount) {
        // Calculate chunk size
        size_t remaining = frameCount - processed;
        size_t chunkSize = std::min(remaining, static_cast<size_t>(BUFFER_FRAMES));

        // Copy pattern data to scratch buffers (with wraparound)
        for (size_t i = 0; i < chunkSize; ++i) {
            size_t patternIdx = (pos + i) % patternSize;
            m_scratchX[i] = m_patternX[patternIdx];
            m_scratchY[i] = m_patternY[patternIdx];
        }

        // Track pattern cycle transitions for fade effect
        size_t startCycle = pos / patternSize;
        size_t endCycle = (pos + chunkSize - 1) / patternSize;
        if (endCycle > startCycle) {
            m_fadeStep++;
            m_patternCycleCount++;
        }

        // Apply effects (SIMD optimized)
        applyEffects(m_scratchX, m_scratchY, chunkSize);

        // Interleave to output (Left=X, Right=Y)
        // AND write to visualization ring buffer
        float* outPtr = output + processed * 2;
        size_t vizPos = m_vizWritePos.load(std::memory_order_relaxed);

        for (size_t i = 0; i < chunkSize; ++i) {
            outPtr[i * 2] = m_scratchX[i];
            outPtr[i * 2 + 1] = m_scratchY[i];

            // Write to viz buffer (ring buffer)
            size_t vizIdx = vizPos % VIZ_BUFFER_SIZE;
            m_vizBufferX[vizIdx] = m_scratchX[i];
            m_vizBufferY[vizIdx] = m_scratchY[i];
            ++vizPos;
        }

        m_vizWritePos.store(vizPos, std::memory_order_release);

        // Update current position for single-point display
        m_currentX.store(m_scratchX[chunkSize - 1], std::memory_order_relaxed);
        m_currentY.store(m_scratchY[chunkSize - 1], std::memory_order_relaxed);

        pos += chunkSize;
        processed += chunkSize;

        // Update effect time
        m_effectTime += static_cast<float>(chunkSize) / static_cast<float>(m_actualSampleRate);
    }

    m_playbackPosition.store(pos, std::memory_order_relaxed);
    return paContinue;
}

//==============================================================================
// SIMD-Optimized Effect Processing
//==============================================================================

void AudioEngine::applyEffects(float* OSCILLOPLOT_RESTRICT x,
                               float* OSCILLOPLOT_RESTRICT y,
                               size_t count) {
    // Load effect parameters (atomic reads)
    const auto rotMode = m_effects.rotationMode.load(std::memory_order_relaxed);
    const float rotAngle = m_effects.rotationAngle.load(std::memory_order_relaxed);
    const float rotSpeed = m_effects.rotationSpeed.load(std::memory_order_relaxed);

    const bool fadeX = m_effects.fadeXEnabled.load(std::memory_order_relaxed);
    const bool fadeY = m_effects.fadeYEnabled.load(std::memory_order_relaxed);
    const bool shrink = m_effects.shrinkEnabled.load(std::memory_order_relaxed);
    const uint16_t fadeXSteps = m_effects.fadeXSteps.load(std::memory_order_relaxed);
    const uint16_t fadeYSteps = m_effects.fadeYSteps.load(std::memory_order_relaxed);
    const uint16_t shrinkSteps = m_effects.shrinkSteps.load(std::memory_order_relaxed);

    const bool noiseX = m_effects.noiseXEnabled.load(std::memory_order_relaxed);
    const bool noiseY = m_effects.noiseYEnabled.load(std::memory_order_relaxed);
    const float noiseXAmt = m_effects.noiseXAmount.load(std::memory_order_relaxed);
    const float noiseYAmt = m_effects.noiseYAmount.load(std::memory_order_relaxed);

    const bool wavyX = m_effects.wavyXEnabled.load(std::memory_order_relaxed);
    const bool wavyY = m_effects.wavyYEnabled.load(std::memory_order_relaxed);
    const float wavyXAmp = m_effects.wavyXAmplitude.load(std::memory_order_relaxed);
    const float wavyYAmp = m_effects.wavyYAmplitude.load(std::memory_order_relaxed);
    const float wavyXFreq = m_effects.wavyXFrequency.load(std::memory_order_relaxed);
    const float wavyYFreq = m_effects.wavyYFrequency.load(std::memory_order_relaxed);

    const bool tremolo = m_effects.tremoloEnabled.load(std::memory_order_relaxed);
    const float tremDepth = m_effects.tremoloDepth.load(std::memory_order_relaxed);
    const float tremRate = m_effects.tremoloRate.load(std::memory_order_relaxed);
    const auto tremWave = m_effects.tremoloWaveform.load(std::memory_order_relaxed);

    const bool ringMod = m_effects.ringModEnabled.load(std::memory_order_relaxed);
    const float ringFreq = m_effects.ringModFreq.load(std::memory_order_relaxed);
    const float ringMix = m_effects.ringModMix.load(std::memory_order_relaxed);

    const bool mirrorX = m_effects.mirrorX.load(std::memory_order_relaxed);
    const bool mirrorY = m_effects.mirrorY.load(std::memory_order_relaxed);

    const bool echoEnabled = m_effects.echoEnabled.load(std::memory_order_relaxed);
    const bool kaleidoscopeEnabled = m_effects.kaleidoscopeEnabled.load(std::memory_order_relaxed);
    const bool distortionEnabled = m_effects.distortionEnabled.load(std::memory_order_relaxed);

    //--------------------------------------------------------------------------
    // 1. ROTATION (SIMD batch)
    //--------------------------------------------------------------------------
    if (rotMode != EffectParams::RotationMode::Off) {
        float angleDeg;
        switch (rotMode) {
            case EffectParams::RotationMode::Static:
                angleDeg = rotAngle;
                break;
            case EffectParams::RotationMode::CW:
                m_currentRotationAngle += rotSpeed * (static_cast<float>(count) /
                    static_cast<float>(m_patternSize.load(std::memory_order_relaxed)));
                angleDeg = m_currentRotationAngle;
                break;
            case EffectParams::RotationMode::CCW:
                m_currentRotationAngle -= rotSpeed * (static_cast<float>(count) /
                    static_cast<float>(m_patternSize.load(std::memory_order_relaxed)));
                angleDeg = m_currentRotationAngle;
                break;
            default:
                angleDeg = 0.0f;
        }

        // Use LUT for sin/cos
        const float phase = angleDeg / 360.0f;
        float sinA, cosA;
        dsp::SineLUT::sincos(phase, sinA, cosA);

        // SIMD rotation
        dsp::rotateXY(x, y, count, cosA, sinA);
    }

    //--------------------------------------------------------------------------
    // 2. KALEIDOSCOPE (before other effects for best visual)
    //--------------------------------------------------------------------------
    if (kaleidoscopeEnabled) {
        applyKaleidoscope(x, y, count);
    }

    //--------------------------------------------------------------------------
    // 3. FADE/SHRINK (SIMD scale)
    //--------------------------------------------------------------------------
    if (fadeX || fadeY || shrink) {
        float xScale = 1.0f;
        float yScale = 1.0f;

        if (shrink && shrinkSteps > 1) {
            uint32_t period = shrinkSteps * 2;
            float t = static_cast<float>(m_fadeStep % period);
            if (t >= shrinkSteps) t = period - t;
            float scale = t / static_cast<float>(shrinkSteps);
            xScale *= scale;
            yScale *= scale;
        }

        if (fadeX && fadeXSteps > 1) {
            uint32_t period = fadeXSteps * 2;
            float t = static_cast<float>(m_fadeStep % period);
            if (t >= fadeXSteps) t = period - t;
            xScale *= t / static_cast<float>(fadeXSteps);
        }

        if (fadeY && fadeYSteps > 1) {
            uint32_t period = fadeYSteps * 2;
            float t = static_cast<float>(m_fadeStep % period);
            if (t >= fadeYSteps) t = period - t;
            yScale *= t / static_cast<float>(fadeYSteps);
        }

        // Apply scale (SIMD)
        dsp::applyGain(x, count, xScale);
        dsp::applyGain(y, count, yScale);
    }

    //--------------------------------------------------------------------------
    // 4. NOISE (fast PRNG, per-sample)
    //--------------------------------------------------------------------------
    if (noiseX || noiseY) {
        for (size_t i = 0; i < count; ++i) {
            if (noiseX) {
                x[i] += randomFloat(m_noiseState) * noiseXAmt;
            }
            if (noiseY) {
                y[i] += randomFloat(m_noiseState) * noiseYAmt;
            }
        }
    }

    //--------------------------------------------------------------------------
    // 5. WAVY (LUT sine, per-sample with time)
    //--------------------------------------------------------------------------
    if (wavyX || wavyY) {
        const float invSampleRate = 1.0f / static_cast<float>(m_actualSampleRate);
        for (size_t i = 0; i < count; ++i) {
            float t = m_effectTime + static_cast<float>(i) * invSampleRate;
            if (wavyX) {
                float phase = t * wavyXFreq;
                x[i] += wavyXAmp * dsp::SineLUT::sin(phase);
            }
            if (wavyY) {
                float phase = t * wavyYFreq;
                y[i] += wavyYAmp * dsp::SineLUT::sin(phase);
            }
        }
    }

    //--------------------------------------------------------------------------
    // 6. TREMOLO (amplitude modulation)
    //--------------------------------------------------------------------------
    if (tremolo) {
        const float invSampleRate = 1.0f / static_cast<float>(m_actualSampleRate);
        for (size_t i = 0; i < count; ++i) {
            float t = m_effectTime + static_cast<float>(i) * invSampleRate;
            float phase = t * tremRate;
            float modValue;

            switch (tremWave) {
                case EffectParams::TremoloWave::Sine:
                    modValue = (dsp::SineLUT::sin(phase) + 1.0f) * 0.5f;
                    break;
                case EffectParams::TremoloWave::Triangle: {
                    float tp = phase - static_cast<int>(phase);
                    modValue = (tp < 0.5f) ? (tp * 2.0f) : (2.0f - tp * 2.0f);
                    break;
                }
                case EffectParams::TremoloWave::Square:
                    modValue = (dsp::SineLUT::sin(phase) >= 0.0f) ? 1.0f : 0.0f;
                    break;
                default:
                    modValue = 1.0f;
            }

            float amp = 1.0f - tremDepth * (1.0f - modValue);
            x[i] *= amp;
            y[i] *= amp;
        }
    }

    //--------------------------------------------------------------------------
    // 7. RING MODULATION
    //--------------------------------------------------------------------------
    if (ringMod) {
        const float invSampleRate = 1.0f / static_cast<float>(m_actualSampleRate);
        const float dryMix = 1.0f - ringMix;

        for (size_t i = 0; i < count; ++i) {
            float t = m_effectTime + static_cast<float>(i) * invSampleRate;
            float phase = t * ringFreq;
            float carrier = dsp::SineLUT::sin(phase);

            x[i] = x[i] * dryMix + x[i] * carrier * ringMix;
            y[i] = y[i] * dryMix + y[i] * carrier * ringMix;
        }
    }

    //--------------------------------------------------------------------------
    // 8. DISTORTION
    //--------------------------------------------------------------------------
    if (distortionEnabled) {
        applyDistortion(x, y, count);
    }

    //--------------------------------------------------------------------------
    // 9. ECHO/DELAY
    //--------------------------------------------------------------------------
    if (echoEnabled) {
        applyEcho(x, y, count);
    }

    //--------------------------------------------------------------------------
    // 10. MIRROR (axis flip)
    //--------------------------------------------------------------------------
    if (mirrorX) {
        for (size_t i = 0; i < count; ++i) {
            x[i] = -x[i];
        }
    }
    if (mirrorY) {
        for (size_t i = 0; i < count; ++i) {
            y[i] = -y[i];
        }
    }
}

//==============================================================================
// Echo/Delay Effect
//==============================================================================

void AudioEngine::applyEcho(float* OSCILLOPLOT_RESTRICT x,
                            float* OSCILLOPLOT_RESTRICT y,
                            size_t count) {
    const uint8_t numEchoes = m_effects.echoCount.load(std::memory_order_relaxed);
    const float decay = m_effects.echoDecay.load(std::memory_order_relaxed);
    const float delayFrac = m_effects.echoDelay.load(std::memory_order_relaxed);

    // Calculate delay in samples (fraction of pattern size)
    const size_t patternSize = m_patternSize.load(std::memory_order_relaxed);
    const size_t delaySamples = static_cast<size_t>(delayFrac * patternSize);

    if (delaySamples == 0 || numEchoes == 0) return;

    // Process each sample
    for (size_t i = 0; i < count; ++i) {
        float dryX = x[i];
        float dryY = y[i];

        // Add echoes from delay buffer
        float wetX = 0.0f;
        float wetY = 0.0f;
        float amplitude = decay;

        for (uint8_t echo = 0; echo < numEchoes; ++echo) {
            size_t readOffset = (echo + 1) * delaySamples;
            if (readOffset >= MAX_ECHO_SAMPLES) break;

            size_t readPos = (m_echoWritePos + MAX_ECHO_SAMPLES - readOffset) % MAX_ECHO_SAMPLES;
            wetX += m_echoBufferX[readPos] * amplitude;
            wetY += m_echoBufferY[readPos] * amplitude;
            amplitude *= decay;
        }

        // Mix dry + wet
        x[i] = dryX + wetX;
        y[i] = dryY + wetY;

        // Write to delay buffer
        m_echoBufferX[m_echoWritePos] = dryX;
        m_echoBufferY[m_echoWritePos] = dryY;
        m_echoWritePos = (m_echoWritePos + 1) % MAX_ECHO_SAMPLES;
    }
}

//==============================================================================
// Kaleidoscope Effect
//==============================================================================

void AudioEngine::applyKaleidoscope(float* OSCILLOPLOT_RESTRICT x,
                                    float* OSCILLOPLOT_RESTRICT y,
                                    size_t count) {
    const uint8_t sections = m_effects.kaleidoscopeSections.load(std::memory_order_relaxed);
    const bool mirror = m_effects.kaleidoscopeMirror.load(std::memory_order_relaxed);
    const float rotSpeed = m_effects.kaleidoscopeRotation.load(std::memory_order_relaxed);

    if (sections < 2) return;

    constexpr float PI = 3.14159265358979323846f;
    constexpr float TWO_PI = 2.0f * PI;

    // Update rotation angle
    const size_t patternSize = m_patternSize.load(std::memory_order_relaxed);
    m_kaleidoscopeAngle += rotSpeed * (static_cast<float>(count) / static_cast<float>(patternSize));

    const float sectionAngle = TWO_PI / static_cast<float>(sections);

    for (size_t i = 0; i < count; ++i) {
        float px = x[i];
        float py = y[i];

        // Convert to polar coordinates
        float r = std::sqrt(px * px + py * py);
        float theta = fastAtan2(py, px);

        // Add rotation
        theta += m_kaleidoscopeAngle * PI / 180.0f;

        // Fold angle into first section
        // Normalize to 0..2PI
        while (theta < 0) theta += TWO_PI;
        while (theta >= TWO_PI) theta -= TWO_PI;

        // Which section are we in?
        int sectionIdx = static_cast<int>(theta / sectionAngle);
        float localAngle = theta - sectionIdx * sectionAngle;

        // Mirror odd sections if enabled
        if (mirror && (sectionIdx & 1)) {
            localAngle = sectionAngle - localAngle;
        }

        // Convert back to cartesian
        float sinA, cosA;
        dsp::SineLUT::sincos(localAngle / TWO_PI, sinA, cosA);
        x[i] = r * cosA;
        y[i] = r * sinA;
    }
}

//==============================================================================
// Distortion Effect
//==============================================================================

void AudioEngine::applyDistortion(float* OSCILLOPLOT_RESTRICT x,
                                  float* OSCILLOPLOT_RESTRICT y,
                                  size_t count) {
    const auto type = m_effects.distortionType.load(std::memory_order_relaxed);
    const float threshold = m_effects.distortionThreshold.load(std::memory_order_relaxed);
    const float drive = m_effects.distortionDrive.load(std::memory_order_relaxed);

    switch (type) {
        case EffectParams::DistortionType::SoftClip:
            // Soft clipping using tanh
            for (size_t i = 0; i < count; ++i) {
                x[i] = fastTanh(x[i] * drive) * threshold;
                y[i] = fastTanh(y[i] * drive) * threshold;
            }
            break;

        case EffectParams::DistortionType::HardClip:
            // Hard clipping
            for (size_t i = 0; i < count; ++i) {
                float vx = x[i] * drive;
                float vy = y[i] * drive;
                x[i] = (vx > threshold) ? threshold : ((vx < -threshold) ? -threshold : vx);
                y[i] = (vy > threshold) ? threshold : ((vy < -threshold) ? -threshold : vy);
            }
            break;

        case EffectParams::DistortionType::Fold:
            // Wave folding - fold back when exceeding threshold
            for (size_t i = 0; i < count; ++i) {
                float vx = x[i] * drive;
                float vy = y[i] * drive;

                // Fold X
                while (vx > threshold || vx < -threshold) {
                    if (vx > threshold) vx = 2.0f * threshold - vx;
                    else if (vx < -threshold) vx = -2.0f * threshold - vx;
                }

                // Fold Y
                while (vy > threshold || vy < -threshold) {
                    if (vy > threshold) vy = 2.0f * threshold - vy;
                    else if (vy < -threshold) vy = -2.0f * threshold - vy;
                }

                x[i] = vx;
                y[i] = vy;
            }
            break;
    }
}

} // namespace oscilloplot

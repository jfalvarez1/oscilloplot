#pragma once

#include <SDL.h>
#include <memory>
#include <string>
#include <vector>

namespace oscilloplot {

// Forward declarations
class AudioEngine;
class UIManager;
struct Pattern;
struct EffectParams;

class App {
public:
    App();
    ~App();

    // Non-copyable
    App(const App&) = delete;
    App& operator=(const App&) = delete;

    bool init();
    void run();
    void shutdown();

    // Accessors
    SDL_Window* getWindow() const { return m_window; }
    SDL_GLContext getGLContext() const { return m_glContext; }

    // Application state
    bool isPlaying() const { return m_isPlaying; }
    void setPlaying(bool playing);

    // Request a graceful shutdown of the main loop (File > Exit, etc.)
    void requestExit() { m_running = false; }

    Pattern& getPattern() { return *m_pattern; }
    const Pattern& getPattern() const { return *m_pattern; }

    AudioEngine& getAudioEngine() { return *m_audioEngine; }

    // False when no audio output device could be opened; the app still runs,
    // with the visualizer and file export intact.
    bool isAudioAvailable() const { return m_audioAvailable; }

    // Effect parameters (for UI binding)
    EffectParams& getEffects();
    const EffectParams& getEffects() const;

    // Audio parameters
    int getSampleRate() const { return m_sampleRate; }
    void setSampleRate(int rate) { m_sampleRate = rate; }

    int getPlaybackMultiplier() const { return m_playbackMultiplier; }
    void setPlaybackMultiplier(int mult) { m_playbackMultiplier = mult; }

    int getDuration() const { return m_duration; }
    void setDuration(int dur) { m_duration = dur; }

    int getPatternRepeats() const { return m_patternRepeats; }
    void setPatternRepeats(int repeats) { m_patternRepeats = repeats; }

private:
    bool initSDL();
    bool initOpenGL();

    // Try one OpenGL configuration, creating the window and context together.
    // On Windows a window's pixel format is fixed at creation, so a failed
    // attempt must destroy the window before the next one is tried.
    bool tryCreateContext(int major, int minor, int profile, const char* glslVersion);
    bool initImGui();
    bool initAudio();

    void processEvents();
    void update();
    void render();
    void renderBasicUI();

    // Window
    SDL_Window* m_window = nullptr;
    SDL_GLContext m_glContext = nullptr;
    const char* m_glslVersion = "#version 330";  // Set by the context fallback
    bool m_audioAvailable = false;               // False if no output device
    int m_windowWidth = 1280;
    int m_windowHeight = 800;
    bool m_running = false;
    bool m_initialized = false;   // Guards shutdown() against running twice

    // Subsystems
    std::unique_ptr<AudioEngine> m_audioEngine;
    std::unique_ptr<UIManager> m_uiManager;
    std::unique_ptr<Pattern> m_pattern;

    // Audio state - Optimal defaults for XY audio visualization
    // Actual rate = sampleRate × multiplier = 192000 Hz (CD quality × 4.3)
    // This gives smooth beam movement at typical pattern sizes (500-2000 points)
    bool m_isPlaying = false;
    int m_sampleRate = 1200;           // Base rate for pattern timing
    int m_playbackMultiplier = 160;    // × multiplier = 192kHz actual audio
    int m_duration = 30;               // 30 second loops
    int m_patternRepeats = 100;        // Pattern repeats (can adjust for speed)
};

} // namespace oscilloplot

#include "app.hpp"
#include "audio/audio_engine.hpp"
#include "ui/ui_manager.hpp"
#include "data/pattern.hpp"
#include "generators/test_pattern.hpp"

#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

#if defined(_WIN32)
#include <windows.h>
#endif
#include <GL/gl.h>
#include <iostream>
#include <cmath>

namespace oscilloplot {

App::App() = default;

App::~App() {
    shutdown();
}

bool App::init() {
    if (!initSDL()) return false;
    if (!initOpenGL()) return false;
    if (!initImGui()) return false;
    if (!initAudio()) return false;

    // Initialize UI manager
    m_uiManager = std::make_unique<UIManager>();
    m_uiManager->init();

    // Create default pattern (sine wave)
    m_pattern = std::make_unique<Pattern>();
    generators::generateSineWave(*m_pattern, 500, 3.0f);

    // Send pattern to audio engine
    m_audioEngine->setPattern(*m_pattern);
    m_audioEngine->setSampleRate(m_sampleRate, m_playbackMultiplier);
    m_audioEngine->setPatternRepeats(m_patternRepeats);

    m_running = true;
    return true;
}

bool App::initSDL() {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_TIMER) != 0) {
        std::cerr << "SDL_Init error: " << SDL_GetError() << std::endl;
        return false;
    }

    // OpenGL attributes
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);

    // Create window
    Uint32 windowFlags = SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI;
    m_window = SDL_CreateWindow(
        "Oscilloplot - XY Audio Generator",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        m_windowWidth,
        m_windowHeight,
        windowFlags
    );

    if (!m_window) {
        std::cerr << "SDL_CreateWindow error: " << SDL_GetError() << std::endl;
        return false;
    }

    return true;
}

bool App::initOpenGL() {
    m_glContext = SDL_GL_CreateContext(m_window);
    if (!m_glContext) {
        std::cerr << "SDL_GL_CreateContext error: " << SDL_GetError() << std::endl;
        return false;
    }

    SDL_GL_MakeCurrent(m_window, m_glContext);
    SDL_GL_SetSwapInterval(1); // VSync

    // Load OpenGL functions using GLAD or similar
    // For simplicity, we'll use SDL's built-in GL loading
    // In production, use glad or glew

    return true;
}

bool App::initImGui() {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();

    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    // Style
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 5.0f;
    style.FrameRounding = 3.0f;
    style.GrabRounding = 3.0f;

    // Initialize backends
    ImGui_ImplSDL2_InitForOpenGL(m_window, m_glContext);
    ImGui_ImplOpenGL3_Init("#version 330");

    return true;
}

bool App::initAudio() {
    m_audioEngine = std::make_unique<AudioEngine>();
    if (!m_audioEngine->init()) {
        std::cerr << "Failed to initialize audio engine" << std::endl;
        return false;
    }
    return true;
}

void App::run() {
    while (m_running) {
        processEvents();
        update();
        render();
    }
}

void App::processEvents() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        ImGui_ImplSDL2_ProcessEvent(&event);

        switch (event.type) {
            case SDL_QUIT:
                m_running = false;
                break;
            case SDL_WINDOWEVENT:
                if (event.window.event == SDL_WINDOWEVENT_CLOSE &&
                    event.window.windowID == SDL_GetWindowID(m_window)) {
                    m_running = false;
                }
                if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
                    m_windowWidth = event.window.data1;
                    m_windowHeight = event.window.data2;
                }
                break;
            case SDL_KEYDOWN:
                if (event.key.keysym.sym == SDLK_ESCAPE) {
                    m_running = false;
                }
                if (event.key.keysym.sym == SDLK_SPACE) {
                    setPlaying(!m_isPlaying);
                }
                break;
        }
    }
}

void App::update() {
    // Update audio engine
    if (m_isPlaying && m_audioEngine) {
        m_audioEngine->update();
    }
}

void App::render() {
    // Start ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    // Render UI
    if (m_uiManager) {
        m_uiManager->render(*this);
    } else {
        // Temporary: render basic UI directly
        renderBasicUI();
    }

    // Render ImGui
    ImGui::Render();

    glViewport(0, 0, m_windowWidth, m_windowHeight);
    glClearColor(0.1f, 0.1f, 0.12f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    SDL_GL_SwapWindow(m_window);
}

void App::renderBasicUI() {
    // Control panel
    ImGui::Begin("Controls");

    ImGui::Text("Audio Parameters");
    ImGui::Separator();

    ImGui::SliderInt("Sample Rate", &m_sampleRate, 100, 10000);
    ImGui::SliderInt("Playback Multiplier", &m_playbackMultiplier, 10, 500);
    int actualRate = m_sampleRate * m_playbackMultiplier;
    ImGui::Text("Actual Rate: %d Hz", actualRate);

    ImGui::SliderInt("Duration (s)", &m_duration, 1, 120);
    ImGui::SliderInt("Pattern Repeats", &m_patternRepeats, 1, 2000);

    ImGui::Separator();

    if (ImGui::Button(m_isPlaying ? "Stop (Space)" : "Play (Space)", ImVec2(-1, 40))) {
        setPlaying(!m_isPlaying);
    }

    ImGui::Separator();
    ImGui::Text("Pattern: %zu points", m_pattern ? m_pattern->size() : 0);

    if (ImGui::Button("Generate Sine Wave", ImVec2(-1, 0))) {
        generators::generateSineWave(*m_pattern, 500, 3.0f);
        m_audioEngine->setPattern(*m_pattern);
    }
    if (ImGui::Button("Generate Circle", ImVec2(-1, 0))) {
        generators::generateCircle(*m_pattern, 360);
        m_audioEngine->setPattern(*m_pattern);
    }
    if (ImGui::Button("Generate Lissajous", ImVec2(-1, 0))) {
        generators::generateLissajous(*m_pattern, 1000, 3, 2, 0.0f);
        m_audioEngine->setPattern(*m_pattern);
    }

    ImGui::End();

    // Oscilloscope display
    ImGui::Begin("Oscilloscope");

    if (m_pattern && !m_pattern->empty()) {
        ImVec2 plotSize = ImGui::GetContentRegionAvail();

        if (ImPlot::BeginPlot("##Scope", plotSize, ImPlotFlags_Equal)) {
            ImPlot::SetupAxes("X", "Y");
            ImPlot::SetupAxesLimits(-1.5, 1.5, -1.5, 1.5);

            ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.0f, 1.0f, 0.4f, 1.0f));
            ImPlot::PlotLine("Pattern",
                m_pattern->xData(),
                m_pattern->yData(),
                static_cast<int>(m_pattern->size()));
            ImPlot::PopStyleColor();

            ImPlot::EndPlot();
        }
    }

    ImGui::End();
}

void App::setPlaying(bool playing) {
    m_isPlaying = playing;
    if (playing) {
        // Update audio engine parameters before playing
        m_audioEngine->setSampleRate(m_sampleRate, m_playbackMultiplier);
        m_audioEngine->setPatternRepeats(m_patternRepeats);
        m_audioEngine->setPattern(*m_pattern);
        m_audioEngine->play();
    } else {
        m_audioEngine->stop();
    }
}

EffectParams& App::getEffects() {
    return m_audioEngine->effects();
}

const EffectParams& App::getEffects() const {
    return m_audioEngine->effects();
}

void App::shutdown() {
    if (m_audioEngine) {
        m_audioEngine->shutdown();
        m_audioEngine.reset();
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    if (m_glContext) {
        SDL_GL_DeleteContext(m_glContext);
        m_glContext = nullptr;
    }
    if (m_window) {
        SDL_DestroyWindow(m_window);
        m_window = nullptr;
    }

    SDL_Quit();
}

} // namespace oscilloplot

#include "ui_manager.hpp"
#include "app.hpp"
#include "data/pattern.hpp"
#include "audio/audio_engine.hpp"
#include "generators/test_pattern.hpp"
#include "generators/stroke_font.hpp"

#include <imgui.h>
#include <implot.h>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <set>
#include <cfloat>

namespace oscilloplot {

//==============================================================================
// Constants
//==============================================================================
constexpr float PI = 3.14159265358979323846f;
constexpr float TWO_PI = 2.0f * PI;

//==============================================================================
// 3D Math Helpers
//==============================================================================
struct Vec3 {
    float x, y, z;
    Vec3(float x_ = 0, float y_ = 0, float z_ = 0) : x(x_), y(y_), z(z_) {}
};

// Rotate around X axis
inline Vec3 rotateX(const Vec3& v, float angle) {
    float c = std::cos(angle), s = std::sin(angle);
    return Vec3(v.x, v.y * c - v.z * s, v.y * s + v.z * c);
}

// Rotate around Y axis
inline Vec3 rotateY(const Vec3& v, float angle) {
    float c = std::cos(angle), s = std::sin(angle);
    return Vec3(v.x * c + v.z * s, v.y, -v.x * s + v.z * c);
}

// Rotate around Z axis
inline Vec3 rotateZ(const Vec3& v, float angle) {
    float c = std::cos(angle), s = std::sin(angle);
    return Vec3(v.x * c - v.y * s, v.x * s + v.y * c, v.z);
}

// Project 3D to 2D with perspective
inline void project(const Vec3& v, float perspective, float scale, float& outX, float& outY) {
    if (perspective > 0.01f) {
        float z = v.z + perspective;
        float factor = perspective / z;
        outX = v.x * factor * scale;
        outY = v.y * factor * scale;
    } else {
        // Orthographic
        outX = v.x * scale;
        outY = v.y * scale;
    }
}

//==============================================================================
// UIManager Implementation
//==============================================================================

UIManager::UIManager() {
    std::memset(m_vizX, 0, sizeof(m_vizX));
    std::memset(m_vizY, 0, sizeof(m_vizY));

    // Initialize harmonics with sensible defaults
    m_harmonics.xTerms[0] = {1.0f, 3.0f, 0.0f, true};
    m_harmonics.yTerms[0] = {1.0f, 2.0f, PI / 2.0f, true};

    // Initialize sound pad with a default pattern
    for (int i = 0; i < SoundPadState::NUM_STEPS; ++i) {
        float angle = TWO_PI * static_cast<float>(i) / SoundPadState::NUM_STEPS;
        m_soundPad.x[i] = std::cos(angle) * 0.8f;
        m_soundPad.y[i] = std::sin(angle) * 0.8f;
        m_soundPad.active[i] = true;
    }
}

UIManager::~UIManager() = default;

bool UIManager::init() {
    return true;
}

void UIManager::shutdown() {
}

void UIManager::render(App& app) {
    renderMenuBar(app);
    renderControlPanel(app);
    renderOscilloscopeDisplay(app);

    if (m_showEffectsPanel) {
        renderEffectsPanel(app);
    }
    if (m_showGeneratorsPanel) {
        renderGeneratorsPanel(app);
    }
    if (m_showSoundPad) {
        renderSoundPad(app);
    }
    if (m_showHarmonicsEditor) {
        renderHarmonicsEditor(app);
    }
    if (m_showDrawingCanvas) {
        renderDrawingCanvas(app);
    }
    if (m_show3DShapeGenerator) {
        render3DShapeGenerator(app);
    }
    if (m_showDisplaySettings) {
        renderDisplaySettings(app);
    }

    if (m_showDemoWindow) {
        ImGui::ShowDemoWindow(&m_showDemoWindow);
        ImPlot::ShowDemoWindow();
    }

    // Animate 3D shape if enabled
    if (m_shape3D.animate) {
        m_shape3D.rotationX += m_shape3D.rotationSpeedX;
        m_shape3D.rotationY += m_shape3D.rotationSpeedY;
        m_shape3D.rotationZ += m_shape3D.rotationSpeedZ;
        generate3DShapePattern(app);
    }
}

void UIManager::renderMenuBar(App& app) {
    (void)app;

    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Load Pattern...")) {
                // TODO: File dialog
            }
            if (ImGui::MenuItem("Save Pattern...")) {
                // TODO: File dialog
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Export WAV...")) {
                // TODO: WAV export
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Exit")) {
                // TODO: Signal exit
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("View")) {
            ImGui::MenuItem("Effects Panel", nullptr, &m_showEffectsPanel);
            ImGui::MenuItem("Generators Panel", nullptr, &m_showGeneratorsPanel);
            ImGui::Separator();
            ImGui::MenuItem("Sound Pad", nullptr, &m_showSoundPad);
            ImGui::MenuItem("Harmonics Editor", nullptr, &m_showHarmonicsEditor);
            ImGui::MenuItem("Drawing Canvas", nullptr, &m_showDrawingCanvas);
            ImGui::MenuItem("3D Shape Generator", nullptr, &m_show3DShapeGenerator);
            ImGui::Separator();
            ImGui::MenuItem("Display Settings", nullptr, &m_showDisplaySettings);
            ImGui::Separator();
            ImGui::MenuItem("ImGui Demo", nullptr, &m_showDemoWindow);
            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();
    }
}

void UIManager::renderControlPanel(App& app) {
    // Position: Left side
    ImGui::SetNextWindowPos(ImVec2(10, 30), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(280, 450), ImGuiCond_FirstUseEver);
    ImGui::Begin("Controls");

    ImGui::Text("Audio Parameters");
    ImGui::Separator();

    int sampleRate = app.getSampleRate();
    if (ImGui::SliderInt("Base Sample Rate", &sampleRate, 100, 10000)) {
        app.setSampleRate(sampleRate);
    }

    int multiplier = app.getPlaybackMultiplier();
    if (ImGui::SliderInt("Playback Multiplier", &multiplier, 10, 500)) {
        app.setPlaybackMultiplier(multiplier);
    }

    ImGui::Text("Actual Rate: %d Hz", sampleRate * multiplier);

    int duration = app.getDuration();
    if (ImGui::SliderInt("Duration (s)", &duration, 1, 120)) {
        app.setDuration(duration);
    }

    int repeats = app.getPatternRepeats();
    if (ImGui::SliderInt("Pattern Repeats", &repeats, 1, 2000)) {
        app.setPatternRepeats(repeats);
    }

    ImGui::Separator();

    // Play/Stop button with color
    if (app.isPlaying()) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.2f, 0.2f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.9f, 0.3f, 0.3f, 1.0f));
    } else {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.7f, 0.3f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.8f, 0.4f, 1.0f));
    }

    if (ImGui::Button(app.isPlaying() ? "STOP [Space]" : "PLAY [Space]", ImVec2(-1, 50))) {
        app.setPlaying(!app.isPlaying());
    }
    ImGui::PopStyleColor(2);

    ImGui::Separator();

    // Pattern info
    ImGui::Text("Pattern: %zu points", app.getPattern().size());

    // Playback position indicator
    if (app.isPlaying()) {
        float pos = app.getAudioEngine().getPlaybackPosition();
        ImGui::ProgressBar(pos, ImVec2(-1, 0), "");
    }

    ImGui::End();
}

//==============================================================================
// Phosphor Display - Realistic CRT Oscilloscope
//==============================================================================

void UIManager::renderOscilloscopeDisplay(App& app) {
    // Position: Center
    ImGui::SetNextWindowPos(ImVec2(300, 30), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(600, 600), ImGuiCond_FirstUseEver);
    // Set minimum window size to ensure display is always visible
    ImGui::SetNextWindowSizeConstraints(ImVec2(400, 400), ImVec2(FLT_MAX, FLT_MAX));

    ImGui::Begin("Oscilloscope");

    // Mini toolbar for display settings (Tektronix 465B style controls)
    if (ImGui::Button("Settings")) {
        m_showDisplaySettings = !m_showDisplaySettings;
    }
    ImGui::SameLine();
    ImGui::Text("INTEN:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(80);
    ImGui::SliderFloat("##Intens", &m_phosphor.brightness, 0.5f, 2.5f, "%.1f");
    ImGui::SameLine();
    ImGui::Text("FOCUS:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(60);
    ImGui::SliderFloat("##Focus", &m_phosphor.beamFocus, 0.5f, 1.0f, "%.2f");
    ImGui::SameLine();
    ImGui::Text("PERSIST:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(70);
    ImGui::SliderFloat("##Decay", &m_phosphor.decayTime, 20.0f, 150.0f, "%.0f");

    renderPhosphorScope(app);

    ImGui::End();
}

void UIManager::renderPhosphorScope(App& app) {
    ImVec2 plotSize = ImGui::GetContentRegionAvail();

    // Ensure minimum valid plot size
    if (plotSize.x < 100.0f) plotSize.x = 100.0f;
    if (plotSize.y < 100.0f) plotSize.y = 100.0f;

    // Calculate frame timing for decay
    ++m_frameCount;
    float currentTime = static_cast<float>(m_frameCount) / 60.0f * 1000.0f; // Approximate ms
    (void)currentTime; // Suppress unused variable warning

    //==========================================================================
    // CRT-style dark background with slight green tint (like real phosphor glow)
    //==========================================================================
    ImPlot::PushStyleColor(ImPlotCol_PlotBg, ImVec4(0.01f, 0.015f, 0.01f, 1.0f));
    ImPlot::PushStyleColor(ImPlotCol_PlotBorder, ImVec4(0.08f, 0.2f, 0.08f, 0.8f));
    ImPlot::PushStyleColor(ImPlotCol_FrameBg, ImVec4(0.0f, 0.0f, 0.0f, 1.0f));

    ImPlotFlags plotFlags = ImPlotFlags_Equal | ImPlotFlags_NoLegend | ImPlotFlags_NoMenus;

    if (ImPlot::BeginPlot("##PhosphorScope", plotSize, plotFlags)) {
        ImPlot::SetupAxes("", "", ImPlotAxisFlags_NoTickLabels | ImPlotAxisFlags_NoGridLines,
                                  ImPlotAxisFlags_NoTickLabels | ImPlotAxisFlags_NoGridLines);
        ImPlot::SetupAxesLimits(-1.2, 1.2, -1.2, 1.2);

        //======================================================================
        // GRATICULE (Tektronix 465B style: 8x10 divisions)
        //======================================================================
        if (m_phosphor.showGrid) {
            // Major grid lines (every 0.2 units = 1 division on 8x10 cm display)
            ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.06f, 0.18f, 0.06f, m_phosphor.gridAlpha));
            for (int i = -5; i <= 5; ++i) {
                float pos = i * 0.2f;
                float lineH[2] = {-1.0f, 1.0f};
                float lineV[2] = {pos, pos};
                ImPlot::PlotLine("##GH", lineH, lineV, 2);
                float linePosH[2] = {pos, pos};
                float linePosV[2] = {-1.0f, 1.0f};
                ImPlot::PlotLine("##GV", linePosH, linePosV, 2);
            }
            ImPlot::PopStyleColor();

            // Center crosshairs (brighter)
            ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.1f, 0.25f, 0.1f, m_phosphor.gridAlpha * 1.5f));
            float ch[2] = {-1.0f, 1.0f}; float cv[2] = {0.0f, 0.0f};
            ImPlot::PlotLine("##CH", ch, cv, 2);
            float cvh[2] = {0.0f, 0.0f}; float cvv[2] = {-1.0f, 1.0f};
            ImPlot::PlotLine("##CV", cvh, cvv, 2);
            ImPlot::PopStyleColor();

            // Minor tick marks on center lines (0.04 unit ticks = 0.2 div subdivision)
            if (m_phosphor.showGraticuleMarks) {
                ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.08f, 0.2f, 0.08f, m_phosphor.gridAlpha));
                for (int i = -25; i <= 25; ++i) {
                    if (i % 5 == 0) continue; // Skip major divisions
                    float pos = i * 0.04f;
                    float tickH[2] = {pos, pos}; float tickV[2] = {-0.02f, 0.02f};
                    ImPlot::PlotLine("##TH", tickH, tickV, 2);
                    float tickVH[2] = {-0.02f, 0.02f}; float tickVV[2] = {pos, pos};
                    ImPlot::PlotLine("##TV", tickVH, tickVV, 2);
                }
                ImPlot::PopStyleColor();
            }
        }

        //======================================================================
        // TRACE RENDERING - Physically-based phosphor simulation
        //======================================================================
        if (app.isPlaying()) {
            size_t count = app.getAudioEngine().getVizSamples(m_vizX, m_vizY,
                           static_cast<size_t>(m_phosphor.trailSamples));

            if (count > 1) {
                //--------------------------------------------------------------
                // STEP 1: Calculate velocity-based intensity for each segment
                // Physics: Brightness ∝ 1/velocity (phosphor dwell time)
                //--------------------------------------------------------------
                float maxVelocity = 0.0f;
                for (size_t i = 1; i < count; ++i) {
                    float dx = m_vizX[i] - m_vizX[i-1];
                    float dy = m_vizY[i] - m_vizY[i-1];
                    float velocity = std::sqrt(dx*dx + dy*dy);
                    m_vizIntensity[i] = velocity;
                    if (velocity > maxVelocity) maxVelocity = velocity;
                }
                m_vizIntensity[0] = m_vizIntensity[1]; // First sample

                // Normalize and invert (slow = bright, fast = dim)
                if (maxVelocity > 0.0001f) {
                    for (size_t i = 0; i < count; ++i) {
                        float normalizedVel = m_vizIntensity[i] / maxVelocity;
                        // Inverse relationship with velocity effect control
                        float velocityFactor = 1.0f - m_phosphor.velocityEffect * normalizedVel;
                        m_vizIntensity[i] = m_phosphor.minBrightness +
                            (m_phosphor.maxBrightness - m_phosphor.minBrightness) * velocityFactor;
                    }
                }

                //--------------------------------------------------------------
                // STEP 2: Render with exponential decay (phosphor persistence)
                // Physics: I(t) = I₀ × e^(-t/τ)
                //--------------------------------------------------------------
                const int numDecayLayers = 8;
                const float decayTimeConstant = m_phosphor.decayTime; // ms

                for (int layer = numDecayLayers - 1; layer >= 0; --layer) {
                    size_t layerStart = (count * layer) / numDecayLayers;
                    size_t layerEnd = (count * (layer + 1)) / numDecayLayers;
                    size_t layerCount = layerEnd - layerStart;
                    if (layerCount < 2) continue;

                    // Calculate age in ms (assuming ~60fps, 16.67ms per frame)
                    float ageMs = (numDecayLayers - 1 - layer) * (16.67f * numDecayLayers / 8.0f);

                    // Exponential decay: e^(-t/τ)
                    float decayFactor = std::exp(-ageMs / decayTimeConstant);
                    decayFactor = std::pow(decayFactor, m_phosphor.decayExponent);

                    // Average intensity for this layer segment
                    float avgIntensity = 0.0f;
                    for (size_t i = layerStart; i < layerEnd; ++i) {
                        avgIntensity += m_vizIntensity[i];
                    }
                    avgIntensity /= static_cast<float>(layerCount);

                    float finalIntensity = avgIntensity * decayFactor * m_phosphor.brightness;
                    finalIntensity = std::min(1.0f, std::max(0.01f, finalIntensity));

                    //----------------------------------------------------------
                    // LAYER 1: Outer glow (exponential falloff bloom)
                    // Physics: Light scattering in phosphor particles
                    //----------------------------------------------------------
                    if (m_phosphor.glowIntensity > 0.0f && layer < 4) {
                        float glowAlpha = finalIntensity * m_phosphor.glowIntensity * 0.25f;
                        float glowWidth = m_phosphor.beamWidth * (m_phosphor.glowRadius +
                                         (1.0f - decayFactor) * 2.0f); // Glow expands as it decays

                        ImPlot::PushStyleColor(ImPlotCol_Line,
                            ImVec4(m_phosphor.colorR * 0.4f,
                                   m_phosphor.colorG * 0.6f,
                                   m_phosphor.colorB * 0.3f, glowAlpha));
                        ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, glowWidth);
                        ImPlot::PlotLine("##Glow", &m_vizX[layerStart], &m_vizY[layerStart],
                                        static_cast<int>(layerCount));
                        ImPlot::PopStyleVar();
                        ImPlot::PopStyleColor();
                    }

                    //----------------------------------------------------------
                    // LAYER 2: Core trace (main phosphor emission)
                    //----------------------------------------------------------
                    float coreWidth = m_phosphor.beamWidth * m_phosphor.beamFocus;
                    ImPlot::PushStyleColor(ImPlotCol_Line,
                        ImVec4(m_phosphor.colorR * finalIntensity,
                               m_phosphor.colorG * finalIntensity,
                               m_phosphor.colorB * finalIntensity * 0.9f,
                               finalIntensity));
                    ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, coreWidth);
                    ImPlot::PlotLine("##Core", &m_vizX[layerStart], &m_vizY[layerStart],
                                    static_cast<int>(layerCount));
                    ImPlot::PopStyleVar();
                    ImPlot::PopStyleColor();
                }

                //--------------------------------------------------------------
                // STEP 3: Fresh beam head (most recent samples - hottest)
                // Physics: Freshly excited phosphor at maximum brightness
                //--------------------------------------------------------------
                size_t headSamples = std::min(count, static_cast<size_t>(256));
                size_t headStart = count - headSamples;

                // Intense glow around beam head
                ImPlot::PushStyleColor(ImPlotCol_Line,
                    ImVec4(m_phosphor.colorR * 0.5f,
                           m_phosphor.colorG * 0.8f,
                           m_phosphor.colorB * 0.4f,
                           m_phosphor.glowIntensity * m_phosphor.brightness * 0.4f));
                ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight,
                                     m_phosphor.beamWidth * m_phosphor.beamSpotSize * 2.5f);
                ImPlot::PlotLine("##HeadGlow", &m_vizX[headStart], &m_vizY[headStart],
                                static_cast<int>(headSamples));
                ImPlot::PopStyleVar();
                ImPlot::PopStyleColor();

                // Bright core of beam head
                float headIntensity = m_phosphor.brightness * m_phosphor.maxBrightness;
                ImPlot::PushStyleColor(ImPlotCol_Line,
                    ImVec4(std::min(1.0f, m_phosphor.colorR + 0.2f) * headIntensity,
                           std::min(1.0f, m_phosphor.colorG + 0.05f) * headIntensity,
                           std::min(1.0f, m_phosphor.colorB + 0.1f) * headIntensity,
                           headIntensity));
                ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, m_phosphor.beamWidth * 1.3f);
                ImPlot::PlotLine("##Head", &m_vizX[headStart], &m_vizY[headStart],
                                static_cast<int>(headSamples));
                ImPlot::PopStyleVar();
                ImPlot::PopStyleColor();

                // Hot center (white-hot core for very fresh samples)
                size_t hotSamples = std::min(count, static_cast<size_t>(64));
                size_t hotStart = count - hotSamples;
                ImPlot::PushStyleColor(ImPlotCol_Line,
                    ImVec4(std::min(1.0f, m_phosphor.colorR + 0.5f),
                           std::min(1.0f, m_phosphor.colorG + 0.3f),
                           std::min(1.0f, m_phosphor.colorB + 0.4f),
                           m_phosphor.brightness * 0.9f));
                ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, m_phosphor.beamWidth);
                ImPlot::PlotLine("##Hot", &m_vizX[hotStart], &m_vizY[hotStart],
                                static_cast<int>(hotSamples));
                ImPlot::PopStyleVar();
                ImPlot::PopStyleColor();
            }
        } else {
            //==================================================================
            // STATIC PREVIEW (when not playing)
            //==================================================================
            const auto& pattern = app.getPattern();
            if (!pattern.empty()) {
                // Calculate velocity-based intensity for preview
                size_t pCount = pattern.size();

                // Dim glow layer
                ImPlot::PushStyleColor(ImPlotCol_Line,
                    ImVec4(m_phosphor.colorR * 0.25f, m_phosphor.colorG * 0.35f,
                           m_phosphor.colorB * 0.2f, 0.35f));
                ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, m_phosphor.beamWidth * 3.0f);
                ImPlot::PlotLine("##PrevGlow", pattern.xData(), pattern.yData(),
                                static_cast<int>(pCount));
                ImPlot::PopStyleVar();
                ImPlot::PopStyleColor();

                // Core preview trace
                ImPlot::PushStyleColor(ImPlotCol_Line,
                    ImVec4(m_phosphor.colorR * 0.5f, m_phosphor.colorG * 0.65f,
                           m_phosphor.colorB * 0.45f, 0.7f));
                ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, m_phosphor.beamWidth);
                ImPlot::PlotLine("##Preview", pattern.xData(), pattern.yData(),
                                static_cast<int>(pCount));
                ImPlot::PopStyleVar();
                ImPlot::PopStyleColor();
            }
        }

        ImPlot::EndPlot();
    }

    ImPlot::PopStyleColor(3);
}

//==============================================================================
// Effects Panel
//==============================================================================

void UIManager::renderEffectsPanel(App& app) {
    auto& fx = app.getEffects();

    // Position: Right side, top
    ImGui::SetNextWindowPos(ImVec2(910, 30), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(360, 350), ImGuiCond_FirstUseEver);
    ImGui::Begin("Effects", &m_showEffectsPanel);

    // ROTATION
    if (ImGui::CollapsingHeader("Rotation", ImGuiTreeNodeFlags_DefaultOpen)) {
        int rotationMode = static_cast<int>(fx.rotationMode.load(std::memory_order_relaxed));
        bool changed = false;
        changed |= ImGui::RadioButton("Off##rot", &rotationMode, 0); ImGui::SameLine();
        changed |= ImGui::RadioButton("Static##rot", &rotationMode, 1); ImGui::SameLine();
        changed |= ImGui::RadioButton("CW##rot", &rotationMode, 2); ImGui::SameLine();
        changed |= ImGui::RadioButton("CCW##rot", &rotationMode, 3);
        if (changed) fx.rotationMode.store(static_cast<EffectParams::RotationMode>(rotationMode), std::memory_order_relaxed);

        float staticAngle = fx.rotationAngle.load(std::memory_order_relaxed);
        if (ImGui::SliderFloat("Static Angle", &staticAngle, -180.0f, 180.0f, "%.1f deg"))
            fx.rotationAngle.store(staticAngle, std::memory_order_relaxed);

        float speed = fx.rotationSpeed.load(std::memory_order_relaxed);
        if (ImGui::SliderFloat("Speed", &speed, 0.5f, 45.0f, "%.1f deg/cycle"))
            fx.rotationSpeed.store(speed, std::memory_order_relaxed);
    }

    // FADE / SHRINK
    if (ImGui::CollapsingHeader("Fade / Shrink")) {
        // X-Axis Fade
        bool fadeX = fx.fadeXEnabled.load(std::memory_order_relaxed);
        if (ImGui::Checkbox("X-Axis Fade", &fadeX)) fx.fadeXEnabled.store(fadeX, std::memory_order_relaxed);
        int fadeXSteps = fx.fadeXSteps.load(std::memory_order_relaxed);
        if (ImGui::SliderInt("X Fade Steps", &fadeXSteps, 2, 100))
            fx.fadeXSteps.store(static_cast<uint16_t>(fadeXSteps), std::memory_order_relaxed);
        int fadeXSpeed = fx.fadeXSpeed.load(std::memory_order_relaxed);
        if (ImGui::SliderInt("X Fade Speed##xspeed", &fadeXSpeed, 1, 20, "%d rep/step"))
            fx.fadeXSpeed.store(static_cast<uint16_t>(fadeXSpeed), std::memory_order_relaxed);

        ImGui::Separator();

        // Y-Axis Fade
        bool fadeY = fx.fadeYEnabled.load(std::memory_order_relaxed);
        if (ImGui::Checkbox("Y-Axis Fade", &fadeY)) fx.fadeYEnabled.store(fadeY, std::memory_order_relaxed);
        int fadeYSteps = fx.fadeYSteps.load(std::memory_order_relaxed);
        if (ImGui::SliderInt("Y Fade Steps", &fadeYSteps, 2, 100))
            fx.fadeYSteps.store(static_cast<uint16_t>(fadeYSteps), std::memory_order_relaxed);
        int fadeYSpeed = fx.fadeYSpeed.load(std::memory_order_relaxed);
        if (ImGui::SliderInt("Y Fade Speed##yspeed", &fadeYSpeed, 1, 20, "%d rep/step"))
            fx.fadeYSpeed.store(static_cast<uint16_t>(fadeYSpeed), std::memory_order_relaxed);

        // Alternate X/Y Fade option
        bool alternateXY = fx.alternateXYFade.load(std::memory_order_relaxed);
        if (ImGui::Checkbox("Alternate X/Y Fade (X first, then Y)", &alternateXY))
            fx.alternateXYFade.store(alternateXY, std::memory_order_relaxed);
        ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "Instead of simultaneous X+Y fading");

        ImGui::Separator();

        // Shrink/Unshrink
        bool shrink = fx.shrinkEnabled.load(std::memory_order_relaxed);
        if (ImGui::Checkbox("Shrink/Unshrink", &shrink)) fx.shrinkEnabled.store(shrink, std::memory_order_relaxed);
        int shrinkSteps = fx.shrinkSteps.load(std::memory_order_relaxed);
        if (ImGui::SliderInt("Shrink Steps", &shrinkSteps, 2, 100))
            fx.shrinkSteps.store(static_cast<uint16_t>(shrinkSteps), std::memory_order_relaxed);
        int shrinkSpeed = fx.shrinkSpeed.load(std::memory_order_relaxed);
        if (ImGui::SliderInt("Shrink Speed##sspeed", &shrinkSpeed, 1, 20, "%d rep/step"))
            fx.shrinkSpeed.store(static_cast<uint16_t>(shrinkSpeed), std::memory_order_relaxed);
    }

    // NOISE
    if (ImGui::CollapsingHeader("Noise")) {
        bool noiseX = fx.noiseXEnabled.load(std::memory_order_relaxed);
        if (ImGui::Checkbox("X-Channel Noise", &noiseX)) fx.noiseXEnabled.store(noiseX, std::memory_order_relaxed);
        float noiseXAmt = fx.noiseXAmount.load(std::memory_order_relaxed);
        if (ImGui::SliderFloat("X Noise Amount", &noiseXAmt, 0.001f, 0.5f, "%.3f"))
            fx.noiseXAmount.store(noiseXAmt, std::memory_order_relaxed);

        bool noiseY = fx.noiseYEnabled.load(std::memory_order_relaxed);
        if (ImGui::Checkbox("Y-Channel Noise", &noiseY)) fx.noiseYEnabled.store(noiseY, std::memory_order_relaxed);
        float noiseYAmt = fx.noiseYAmount.load(std::memory_order_relaxed);
        if (ImGui::SliderFloat("Y Noise Amount", &noiseYAmt, 0.001f, 0.5f, "%.3f"))
            fx.noiseYAmount.store(noiseYAmt, std::memory_order_relaxed);
    }

    // WAVY
    if (ImGui::CollapsingHeader("Wavy")) {
        bool wavyX = fx.wavyXEnabled.load(std::memory_order_relaxed);
        if (ImGui::Checkbox("X-Axis Wavy", &wavyX)) fx.wavyXEnabled.store(wavyX, std::memory_order_relaxed);
        float wavyXAmp = fx.wavyXAmplitude.load(std::memory_order_relaxed);
        if (ImGui::SliderFloat("X Wavy Amplitude", &wavyXAmp, 0.01f, 1.0f))
            fx.wavyXAmplitude.store(wavyXAmp, std::memory_order_relaxed);
        float wavyXFreq = fx.wavyXFrequency.load(std::memory_order_relaxed);
        if (ImGui::SliderFloat("X Wavy Frequency", &wavyXFreq, 0.5f, 20.0f, "%.1f Hz"))
            fx.wavyXFrequency.store(wavyXFreq, std::memory_order_relaxed);

        bool wavyY = fx.wavyYEnabled.load(std::memory_order_relaxed);
        if (ImGui::Checkbox("Y-Axis Wavy", &wavyY)) fx.wavyYEnabled.store(wavyY, std::memory_order_relaxed);
        float wavyYAmp = fx.wavyYAmplitude.load(std::memory_order_relaxed);
        if (ImGui::SliderFloat("Y Wavy Amplitude", &wavyYAmp, 0.01f, 1.0f))
            fx.wavyYAmplitude.store(wavyYAmp, std::memory_order_relaxed);
        float wavyYFreq = fx.wavyYFrequency.load(std::memory_order_relaxed);
        if (ImGui::SliderFloat("Y Wavy Frequency", &wavyYFreq, 0.5f, 20.0f, "%.1f Hz"))
            fx.wavyYFrequency.store(wavyYFreq, std::memory_order_relaxed);
    }

    // TREMOLO
    if (ImGui::CollapsingHeader("Tremolo")) {
        bool tremolo = fx.tremoloEnabled.load(std::memory_order_relaxed);
        if (ImGui::Checkbox("Enable Tremolo", &tremolo)) fx.tremoloEnabled.store(tremolo, std::memory_order_relaxed);
        float depth = fx.tremoloDepth.load(std::memory_order_relaxed);
        if (ImGui::SliderFloat("Depth", &depth, 0.0f, 1.0f)) fx.tremoloDepth.store(depth, std::memory_order_relaxed);
        float rate = fx.tremoloRate.load(std::memory_order_relaxed);
        if (ImGui::SliderFloat("Rate", &rate, 0.1f, 20.0f, "%.1f Hz")) fx.tremoloRate.store(rate, std::memory_order_relaxed);

        int waveform = static_cast<int>(fx.tremoloWaveform.load(std::memory_order_relaxed));
        bool waveChanged = false;
        waveChanged |= ImGui::RadioButton("Sine##trem", &waveform, 0); ImGui::SameLine();
        waveChanged |= ImGui::RadioButton("Triangle##trem", &waveform, 1); ImGui::SameLine();
        waveChanged |= ImGui::RadioButton("Square##trem", &waveform, 2);
        if (waveChanged) fx.tremoloWaveform.store(static_cast<EffectParams::TremoloWave>(waveform), std::memory_order_relaxed);
    }

    // RING MODULATION
    if (ImGui::CollapsingHeader("Ring Modulation")) {
        bool ringMod = fx.ringModEnabled.load(std::memory_order_relaxed);
        if (ImGui::Checkbox("Enable Ring Mod", &ringMod)) fx.ringModEnabled.store(ringMod, std::memory_order_relaxed);
        float freq = fx.ringModFreq.load(std::memory_order_relaxed);
        if (ImGui::SliderFloat("Carrier Freq", &freq, 10.0f, 1000.0f, "%.0f Hz", ImGuiSliderFlags_Logarithmic))
            fx.ringModFreq.store(freq, std::memory_order_relaxed);
        float mix = fx.ringModMix.load(std::memory_order_relaxed);
        if (ImGui::SliderFloat("Wet/Dry Mix", &mix, 0.0f, 1.0f)) fx.ringModMix.store(mix, std::memory_order_relaxed);
    }

    // ECHO / DELAY
    if (ImGui::CollapsingHeader("Echo / Delay")) {
        bool echoEnabled = fx.echoEnabled.load(std::memory_order_relaxed);
        if (ImGui::Checkbox("Enable Echo", &echoEnabled)) fx.echoEnabled.store(echoEnabled, std::memory_order_relaxed);
        int echoCount = fx.echoCount.load(std::memory_order_relaxed);
        if (ImGui::SliderInt("Number of Echoes", &echoCount, 1, 10))
            fx.echoCount.store(static_cast<uint8_t>(echoCount), std::memory_order_relaxed);
        float decay = fx.echoDecay.load(std::memory_order_relaxed);
        if (ImGui::SliderFloat("Decay Factor", &decay, 0.1f, 0.95f, "%.2f")) fx.echoDecay.store(decay, std::memory_order_relaxed);
        float delay = fx.echoDelay.load(std::memory_order_relaxed);
        if (ImGui::SliderFloat("Delay Time", &delay, 0.01f, 0.5f, "%.2f")) fx.echoDelay.store(delay, std::memory_order_relaxed);
    }

    // KALEIDOSCOPE
    if (ImGui::CollapsingHeader("Kaleidoscope")) {
        bool kaleidoscopeEnabled = fx.kaleidoscopeEnabled.load(std::memory_order_relaxed);
        if (ImGui::Checkbox("Enable Kaleidoscope", &kaleidoscopeEnabled))
            fx.kaleidoscopeEnabled.store(kaleidoscopeEnabled, std::memory_order_relaxed);
        int sections = fx.kaleidoscopeSections.load(std::memory_order_relaxed);
        if (ImGui::SliderInt("Sections", &sections, 2, 12))
            fx.kaleidoscopeSections.store(static_cast<uint8_t>(sections), std::memory_order_relaxed);
        bool mirror = fx.kaleidoscopeMirror.load(std::memory_order_relaxed);
        if (ImGui::Checkbox("Mirror Sections", &mirror)) fx.kaleidoscopeMirror.store(mirror, std::memory_order_relaxed);
        float rotation = fx.kaleidoscopeRotation.load(std::memory_order_relaxed);
        if (ImGui::SliderFloat("Rotation Speed", &rotation, 0.0f, 10.0f, "%.1f deg/cycle"))
            fx.kaleidoscopeRotation.store(rotation, std::memory_order_relaxed);
    }

    // DISTORTION
    if (ImGui::CollapsingHeader("Distortion")) {
        bool distortionEnabled = fx.distortionEnabled.load(std::memory_order_relaxed);
        if (ImGui::Checkbox("Enable Distortion", &distortionEnabled))
            fx.distortionEnabled.store(distortionEnabled, std::memory_order_relaxed);
        int type = static_cast<int>(fx.distortionType.load(std::memory_order_relaxed));
        bool typeChanged = false;
        typeChanged |= ImGui::RadioButton("Soft Clip", &type, 0); ImGui::SameLine();
        typeChanged |= ImGui::RadioButton("Hard Clip", &type, 1); ImGui::SameLine();
        typeChanged |= ImGui::RadioButton("Fold", &type, 2);
        if (typeChanged) fx.distortionType.store(static_cast<EffectParams::DistortionType>(type), std::memory_order_relaxed);
        float threshold = fx.distortionThreshold.load(std::memory_order_relaxed);
        if (ImGui::SliderFloat("Threshold", &threshold, 0.1f, 2.0f)) fx.distortionThreshold.store(threshold, std::memory_order_relaxed);
        float drive = fx.distortionDrive.load(std::memory_order_relaxed);
        if (ImGui::SliderFloat("Drive", &drive, 1.0f, 5.0f)) fx.distortionDrive.store(drive, std::memory_order_relaxed);
    }

    // MIRROR
    if (ImGui::CollapsingHeader("Mirror")) {
        bool mirrorX = fx.mirrorX.load(std::memory_order_relaxed);
        if (ImGui::Checkbox("Mirror X (Flip Horizontal)", &mirrorX)) fx.mirrorX.store(mirrorX, std::memory_order_relaxed);
        bool mirrorY = fx.mirrorY.load(std::memory_order_relaxed);
        if (ImGui::Checkbox("Mirror Y (Flip Vertical)", &mirrorY)) fx.mirrorY.store(mirrorY, std::memory_order_relaxed);
    }

    ImGui::End();
}

//==============================================================================
// Generators Panel
//==============================================================================

void UIManager::renderGeneratorsPanel(App& app) {
    // Position: Right side, below Effects
    ImGui::SetNextWindowPos(ImVec2(910, 390), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(360, 380), ImGuiCond_FirstUseEver);
    ImGui::Begin("Generators", &m_showGeneratorsPanel);

    // Shared parameters
    static int numPoints = 500;
    ImGui::SliderInt("Points", &numPoints, 100, 5000);

    // Helper lambda to update pattern and audio engine
    auto setPattern = [&]() {
        app.getAudioEngine().setPattern(app.getPattern());
    };

    //==========================================================================
    // BASIC SHAPES
    //==========================================================================
    if (ImGui::CollapsingHeader("Basic Shapes", ImGuiTreeNodeFlags_DefaultOpen)) {
        static float sineFreq = 3.0f;
        static float ellipseX = 1.0f, ellipseY = 0.6f;

        if (ImGui::Button("Circle", ImVec2(100, 0))) {
            generators::generateCircle(app.getPattern(), numPoints);
            setPattern();
        }
        ImGui::SameLine();
        if (ImGui::Button("Ellipse", ImVec2(100, 0))) {
            generators::generateEllipse(app.getPattern(), numPoints, ellipseX, ellipseY);
            setPattern();
        }
        ImGui::SetNextItemWidth(80);
        ImGui::InputFloat("X##ell", &ellipseX, 0.0f, 0.0f, "%.2f"); ImGui::SameLine();
        ImGui::SetNextItemWidth(80);
        ImGui::InputFloat("Y##ell", &ellipseY, 0.0f, 0.0f, "%.2f");

        ImGui::SetNextItemWidth(120);
        ImGui::SliderFloat("Sine Cycles", &sineFreq, 1.0f, 10.0f);
        ImGui::SameLine();
        if (ImGui::Button("Sine Wave", ImVec2(-1, 0))) {
            generators::generateSineWave(app.getPattern(), numPoints, sineFreq);
            setPattern();
        }
    }

    //==========================================================================
    // LISSAJOUS CURVES
    //==========================================================================
    if (ImGui::CollapsingHeader("Lissajous Curves")) {
        static int lissA = 3, lissB = 2;
        static float lissPhase = 0.0f;

        ImGui::SetNextItemWidth(60);
        ImGui::InputInt("A##liss", &lissA); ImGui::SameLine();
        ImGui::SetNextItemWidth(60);
        ImGui::InputInt("B##liss", &lissB); ImGui::SameLine();
        ImGui::SetNextItemWidth(80);
        ImGui::SliderFloat("Phase##liss", &lissPhase, 0.0f, PI, "%.2f");

        if (ImGui::Button("3:2", ImVec2(50, 0))) { lissA = 3; lissB = 2; lissPhase = 0.0f; }
        ImGui::SameLine();
        if (ImGui::Button("3:2 (+90)", ImVec2(70, 0))) { lissA = 3; lissB = 2; lissPhase = PI / 2.0f; }
        ImGui::SameLine();
        if (ImGui::Button("5:4", ImVec2(50, 0))) { lissA = 5; lissB = 4; lissPhase = 0.0f; }
        ImGui::SameLine();
        if (ImGui::Button("7:5", ImVec2(50, 0))) { lissA = 7; lissB = 5; lissPhase = PI / 3.0f; }

        if (ImGui::Button("Generate Lissajous", ImVec2(-1, 0))) {
            generators::generateLissajous(app.getPattern(), numPoints, lissA, lissB, lissPhase);
            setPattern();
        }
    }

    //==========================================================================
    // STARS & FLOWERS
    //==========================================================================
    if (ImGui::CollapsingHeader("Stars & Flowers")) {
        static int starSpikes = 5;
        static float starInner = 0.5f;
        static int flowerPetals = 6;
        static float petalDepth = 0.3f;
        static int roseK = 4;

        if (ImGui::Button("Star (5-pt)", ImVec2(80, 0))) {
            generators::generateStar(app.getPattern(), numPoints, 5, 0.5f);
            setPattern();
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(50);
        ImGui::InputInt("Spikes", &starSpikes);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(60);
        ImGui::SliderFloat("Inner##star", &starInner, 0.1f, 0.9f, "%.2f");
        ImGui::SameLine();
        if (ImGui::Button("Custom Star", ImVec2(-1, 0))) {
            generators::generateStar(app.getPattern(), numPoints, starSpikes, starInner);
            setPattern();
        }

        if (ImGui::Button("Flower (6-petal)", ImVec2(100, 0))) {
            generators::generateFlower(app.getPattern(), numPoints, 6, 0.3f);
            setPattern();
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(50);
        ImGui::InputInt("Petals", &flowerPetals);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(60);
        ImGui::SliderFloat("Depth##fl", &petalDepth, 0.1f, 0.5f, "%.2f");
        ImGui::SameLine();
        if (ImGui::Button("Custom Flower", ImVec2(-1, 0))) {
            generators::generateFlower(app.getPattern(), numPoints, flowerPetals, petalDepth);
            setPattern();
        }

        ImGui::SetNextItemWidth(50);
        ImGui::InputInt("k##rose", &roseK);
        ImGui::SameLine();
        if (ImGui::Button("Rose Curve", ImVec2(-1, 0))) {
            generators::generateRoseCurve(app.getPattern(), numPoints, roseK);
            setPattern();
        }
    }

    //==========================================================================
    // SPIRALS
    //==========================================================================
    if (ImGui::CollapsingHeader("Spirals")) {
        static float spiralTurns = 5.0f;
        static float spiralStart = 0.1f, spiralEnd = 1.0f;
        static float logA = 0.1f, logB = 0.1f, logMaxAngle = 18.85f;

        ImGui::Text("Archimedean Spiral");
        ImGui::SetNextItemWidth(80);
        ImGui::SliderFloat("Turns##arch", &spiralTurns, 1.0f, 20.0f);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(60);
        ImGui::SliderFloat("Start##arch", &spiralStart, 0.0f, 0.5f, "%.2f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(60);
        ImGui::SliderFloat("End##arch", &spiralEnd, 0.5f, 1.5f, "%.2f");
        if (ImGui::Button("Generate Archimedean", ImVec2(-1, 0))) {
            generators::generateSpiral(app.getPattern(), numPoints, spiralTurns, spiralStart, spiralEnd);
            setPattern();
        }

        ImGui::Text("Logarithmic Spiral");
        ImGui::SetNextItemWidth(60);
        ImGui::SliderFloat("a##log", &logA, 0.01f, 0.5f, "%.2f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(60);
        ImGui::SliderFloat("b##log", &logB, 0.01f, 0.3f, "%.2f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80);
        ImGui::SliderFloat("Max Angle##log", &logMaxAngle, 6.0f, 30.0f, "%.1f");
        if (ImGui::Button("Generate Logarithmic", ImVec2(-1, 0))) {
            generators::generateLogSpiral(app.getPattern(), numPoints, logA, logB, logMaxAngle);
            setPattern();
        }
    }

    //==========================================================================
    // KNOTS & 3D-STYLE
    //==========================================================================
    if (ImGui::CollapsingHeader("Knots & 3D-Style")) {
        static int torusP = 2, torusQ = 3;

        if (ImGui::Button("Helix", ImVec2(80, 0))) {
            generators::generateHelix(app.getPattern(), numPoints);
            setPattern();
        }
        ImGui::SameLine();
        if (ImGui::Button("Trefoil Knot", ImVec2(100, 0))) {
            generators::generateTrefoilKnot(app.getPattern(), numPoints);
            setPattern();
        }

        ImGui::SetNextItemWidth(50);
        ImGui::InputInt("p##torus", &torusP);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(50);
        ImGui::InputInt("q##torus", &torusQ);
        ImGui::SameLine();
        if (ImGui::Button("Torus Knot", ImVec2(-1, 0))) {
            generators::generateTorusKnot(app.getPattern(), numPoints, torusP, torusQ);
            setPattern();
        }
    }

    //==========================================================================
    // COMPLEX CURVES
    //==========================================================================
    if (ImGui::CollapsingHeader("Complex Curves")) {
        static float hypoR = 5.0f, hypoR2 = 3.0f, hypoD = 5.0f;
        static float epiR = 5.0f, epiR2 = 2.0f, epiD = 2.0f;

        if (ImGui::Button("Butterfly", ImVec2(80, 0))) {
            generators::generateButterfly(app.getPattern(), 1000);  // Needs more points
            setPattern();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cardioid", ImVec2(80, 0))) {
            generators::generateCardioid(app.getPattern(), numPoints);
            setPattern();
        }
        ImGui::SameLine();
        if (ImGui::Button("Deltoid", ImVec2(80, 0))) {
            generators::generateDeltoid(app.getPattern(), numPoints);
            setPattern();
        }

        ImGui::Text("Hypotrochoid (R, r, d):");
        ImGui::SetNextItemWidth(50);
        ImGui::InputFloat("R##hypo", &hypoR, 0.0f, 0.0f, "%.1f"); ImGui::SameLine();
        ImGui::SetNextItemWidth(50);
        ImGui::InputFloat("r##hypo", &hypoR2, 0.0f, 0.0f, "%.1f"); ImGui::SameLine();
        ImGui::SetNextItemWidth(50);
        ImGui::InputFloat("d##hypo", &hypoD, 0.0f, 0.0f, "%.1f"); ImGui::SameLine();
        if (ImGui::Button("Generate##hypo", ImVec2(-1, 0))) {
            generators::generateHypotrochoid(app.getPattern(), numPoints, hypoR, hypoR2, hypoD);
            setPattern();
        }

        ImGui::Text("Epitrochoid (R, r, d):");
        ImGui::SetNextItemWidth(50);
        ImGui::InputFloat("R##epi", &epiR, 0.0f, 0.0f, "%.1f"); ImGui::SameLine();
        ImGui::SetNextItemWidth(50);
        ImGui::InputFloat("r##epi", &epiR2, 0.0f, 0.0f, "%.1f"); ImGui::SameLine();
        ImGui::SetNextItemWidth(50);
        ImGui::InputFloat("d##epi", &epiD, 0.0f, 0.0f, "%.1f"); ImGui::SameLine();
        if (ImGui::Button("Generate##epi", ImVec2(-1, 0))) {
            generators::generateEpitrochoid(app.getPattern(), numPoints, epiR, epiR2, epiD);
            setPattern();
        }
    }

    //==========================================================================
    // SPECIAL SHAPES
    //==========================================================================
    if (ImGui::CollapsingHeader("Special Shapes")) {
        if (ImGui::Button("Figure-8", ImVec2(80, 0))) {
            generators::generateFigure8(app.getPattern(), numPoints);
            setPattern();
        }
        ImGui::SameLine();
        if (ImGui::Button("Infinity", ImVec2(80, 0))) {
            generators::generateInfinity(app.getPattern(), numPoints);
            setPattern();
        }
        ImGui::SameLine();
        if (ImGui::Button("Heart", ImVec2(80, 0))) {
            generators::generateHeart(app.getPattern(), numPoints);
            setPattern();
        }
    }

    //==========================================================================
    // WAVEFORMS
    //==========================================================================
    if (ImGui::CollapsingHeader("Waveforms")) {
        static float waveFreq = 3.0f;
        ImGui::SetNextItemWidth(120);
        ImGui::SliderFloat("Frequency##wave", &waveFreq, 1.0f, 10.0f);

        if (ImGui::Button("Square Wave", ImVec2(100, 0))) {
            generators::generateSquareWave(app.getPattern(), numPoints, waveFreq);
            setPattern();
        }
        ImGui::SameLine();
        if (ImGui::Button("Sawtooth", ImVec2(100, 0))) {
            generators::generateSawtoothWave(app.getPattern(), numPoints, waveFreq);
            setPattern();
        }
        ImGui::SameLine();
        if (ImGui::Button("Triangle", ImVec2(-1, 0))) {
            generators::generateTriangleWave(app.getPattern(), numPoints, waveFreq);
            setPattern();
        }
    }

    ImGui::Separator();

    //==========================================================================
    // ADVANCED GENERATORS
    //==========================================================================
    ImGui::Text("Advanced Generators");
    if (ImGui::Button("Harmonics Editor...", ImVec2(-1, 0))) m_showHarmonicsEditor = true;
    if (ImGui::Button("Sound Pad...", ImVec2(-1, 0))) m_showSoundPad = true;
    if (ImGui::Button("Drawing Canvas...", ImVec2(-1, 0))) m_showDrawingCanvas = true;
    if (ImGui::Button("3D Shape Generator...", ImVec2(-1, 0))) m_show3DShapeGenerator = true;

    ImGui::Separator();

    if (ImGui::Button("Random Harmonics", ImVec2(-1, 0))) {
        Pattern& pattern = app.getPattern();
        pattern.clear();
        pattern.x.reserve(numPoints);
        pattern.y.reserve(numPoints);
        int freqX = (rand() % 7) + 1;
        int freqY = (rand() % 7) + 1;
        float phaseX = static_cast<float>(rand()) / RAND_MAX * TWO_PI;
        float phaseY = static_cast<float>(rand()) / RAND_MAX * TWO_PI;
        for (int i = 0; i < numPoints; ++i) {
            float t = TWO_PI * static_cast<float>(i) / static_cast<float>(numPoints);
            float x = std::sin(freqX * t + phaseX);
            float y = std::sin(freqY * t + phaseY);
            if (rand() % 2) { x += 0.3f * std::sin((freqX * 2 + 1) * t); y += 0.3f * std::sin((freqY * 2 + 1) * t); }
            pattern.x.push_back(x * 0.9f);
            pattern.y.push_back(y * 0.9f);
        }
        setPattern();
    }

    ImGui::End();
}

//==============================================================================
// Harmonics Editor - Text inputs with live preview
//==============================================================================

void UIManager::generateHarmonicsPattern(App& app) {
    Pattern& pattern = app.getPattern();
    pattern.clear();

    // Check if any term has phase or frequency sweep enabled
    bool hasPhaseSweep = false;
    bool hasFreqSweep = false;
    int maxSteps = 1;

    for (int h = 0; h < m_harmonics.numXTerms; ++h) {
        if (m_harmonics.xTerms[h].enabled) {
            if (m_harmonics.xTerms[h].phaseSweep) {
                hasPhaseSweep = true;
                maxSteps = std::max(maxSteps, m_harmonics.xTerms[h].phaseSweepSteps);
            }
            if (m_harmonics.xTerms[h].freqSweep) {
                hasFreqSweep = true;
                maxSteps = std::max(maxSteps, m_harmonics.xTerms[h].freqSweepSteps);
            }
        }
    }
    for (int h = 0; h < m_harmonics.numYTerms; ++h) {
        if (m_harmonics.yTerms[h].enabled) {
            if (m_harmonics.yTerms[h].phaseSweep) {
                hasPhaseSweep = true;
                maxSteps = std::max(maxSteps, m_harmonics.yTerms[h].phaseSweepSteps);
            }
            if (m_harmonics.yTerms[h].freqSweep) {
                hasFreqSweep = true;
                maxSteps = std::max(maxSteps, m_harmonics.yTerms[h].freqSweepSteps);
            }
        }
    }

    // Reserve space for all frames
    int totalPoints = m_harmonics.numPoints * maxSteps;
    pattern.x.reserve(totalPoints);
    pattern.y.reserve(totalPoints);

    // Generate frames
    for (int step = 0; step < maxSteps; ++step) {
        float stepProgress = (maxSteps > 1) ? static_cast<float>(step) / static_cast<float>(maxSteps - 1) : 0.0f;

        for (int i = 0; i < m_harmonics.numPoints; ++i) {
            float t = TWO_PI * static_cast<float>(i) / static_cast<float>(m_harmonics.numPoints);
            float x = 0.0f, y = 0.0f;

            // Calculate X channel
            for (int h = 0; h < m_harmonics.numXTerms; ++h) {
                if (m_harmonics.xTerms[h].enabled) {
                    const auto& term = m_harmonics.xTerms[h];

                    // Calculate current phase (with sweep if enabled)
                    float phase = term.phase;
                    if (term.phaseSweep) {
                        float sweepProgress = (term.phaseSweepSteps > 1) ?
                            static_cast<float>(step % term.phaseSweepSteps) / static_cast<float>(term.phaseSweepSteps - 1) : 0.0f;
                        phase = term.phaseStart + (term.phaseEnd - term.phaseStart) * sweepProgress;
                    }

                    // Calculate current frequency (with sweep if enabled)
                    float freq = term.frequency;
                    if (term.freqSweep) {
                        float sweepProgress = (term.freqSweepSteps > 1) ?
                            static_cast<float>(step % term.freqSweepSteps) / static_cast<float>(term.freqSweepSteps - 1) : 0.0f;
                        freq = term.freqStart + (term.freqEnd - term.freqStart) * sweepProgress;
                    }

                    if (term.useSin) {
                        x += term.amplitude * std::sin(freq * t + phase);
                    } else {
                        x += term.amplitude * std::cos(freq * t + phase);
                    }
                }
            }

            // Calculate Y channel
            for (int h = 0; h < m_harmonics.numYTerms; ++h) {
                if (m_harmonics.yTerms[h].enabled) {
                    const auto& term = m_harmonics.yTerms[h];

                    // Calculate current phase (with sweep if enabled)
                    float phase = term.phase;
                    if (term.phaseSweep) {
                        float sweepProgress = (term.phaseSweepSteps > 1) ?
                            static_cast<float>(step % term.phaseSweepSteps) / static_cast<float>(term.phaseSweepSteps - 1) : 0.0f;
                        phase = term.phaseStart + (term.phaseEnd - term.phaseStart) * sweepProgress;
                    }

                    // Calculate current frequency (with sweep if enabled)
                    float freq = term.frequency;
                    if (term.freqSweep) {
                        float sweepProgress = (term.freqSweepSteps > 1) ?
                            static_cast<float>(step % term.freqSweepSteps) / static_cast<float>(term.freqSweepSteps - 1) : 0.0f;
                        freq = term.freqStart + (term.freqEnd - term.freqStart) * sweepProgress;
                    }

                    if (term.useSin) {
                        y += term.amplitude * std::sin(freq * t + phase);
                    } else {
                        y += term.amplitude * std::cos(freq * t + phase);
                    }
                }
            }

            pattern.x.push_back(x * 0.8f);
            pattern.y.push_back(y * 0.8f);
        }
    }

    app.getAudioEngine().setPattern(pattern);
}

void UIManager::renderHarmonicsEditor(App& app) {
    // Position: Floating, center-left area
    ImGui::SetNextWindowPos(ImVec2(50, 200), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(500, 500), ImGuiCond_FirstUseEver);
    ImGui::Begin("Harmonics Editor", &m_showHarmonicsEditor);

    ImGui::Text("Create custom Lissajous curves with harmonics and sweeps");
    ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "X(t) = sum[A*sin/cos(w*t + phi)]");
    ImGui::Separator();

    bool changed = false;

    changed |= ImGui::SliderInt("Points per Frame", &m_harmonics.numPoints, 100, 5000);
    ImGui::Checkbox("Live Update", &m_harmonics.liveUpdate);

    // Helper lambda to render a harmonic term with sweep controls
    auto renderTermControls = [&](HarmonicTerm& term, int index, const char* prefix) -> bool {
        bool termChanged = false;
        ImGui::PushID(index);

        // Row 1: Basic controls
        ImGui::Text("%s%d:", prefix, index + 1);
        ImGui::SameLine();
        termChanged |= ImGui::Checkbox("##En", &term.enabled);
        ImGui::SameLine();
        termChanged |= ImGui::Checkbox(term.useSin ? "sin" : "cos", &term.useSin);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Toggle sin/cos");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(60);
        termChanged |= ImGui::InputFloat("Amp", &term.amplitude, 0.0f, 0.0f, "%.2f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(60);
        termChanged |= ImGui::InputFloat("Freq", &term.frequency, 0.0f, 0.0f, "%.2f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(60);
        termChanged |= ImGui::InputFloat("Phase", &term.phase, 0.0f, 0.0f, "%.2f");

        // Row 2: Phase Sweep
        ImGui::Indent(20.0f);
        termChanged |= ImGui::Checkbox("Phase Sweep##ps", &term.phaseSweep);
        if (term.phaseSweep) {
            ImGui::SameLine();
            ImGui::SetNextItemWidth(50);
            termChanged |= ImGui::InputFloat("Start##phs", &term.phaseStart, 0.0f, 0.0f, "%.2f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(50);
            termChanged |= ImGui::InputFloat("End##phe", &term.phaseEnd, 0.0f, 0.0f, "%.2f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(50);
            termChanged |= ImGui::InputInt("Steps##pst", &term.phaseSweepSteps);
            term.phaseSweepSteps = std::max(2, std::min(100, term.phaseSweepSteps));
        }

        // Row 3: Frequency Sweep
        termChanged |= ImGui::Checkbox("Freq Sweep##fs", &term.freqSweep);
        if (term.freqSweep) {
            ImGui::SameLine();
            ImGui::SetNextItemWidth(50);
            termChanged |= ImGui::InputFloat("Start##frs", &term.freqStart, 0.0f, 0.0f, "%.2f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(50);
            termChanged |= ImGui::InputFloat("End##fre", &term.freqEnd, 0.0f, 0.0f, "%.2f");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(50);
            termChanged |= ImGui::InputInt("Steps##fst", &term.freqSweepSteps);
            term.freqSweepSteps = std::max(2, std::min(100, term.freqSweepSteps));
        }
        ImGui::Unindent(20.0f);

        ImGui::PopID();
        return termChanged;
    };

    // X Channel Harmonics
    ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.5f, 0.2f, 0.2f, 1.0f));
    if (ImGui::CollapsingHeader("X Channel Harmonics", ImGuiTreeNodeFlags_DefaultOpen)) {
        changed |= ImGui::SliderInt("X Terms##xn", &m_harmonics.numXTerms, 1, HarmonicsState::MAX_HARMONICS);

        for (int i = 0; i < m_harmonics.numXTerms; ++i) {
            changed |= renderTermControls(m_harmonics.xTerms[i], i, "X");
            if (i < m_harmonics.numXTerms - 1) ImGui::Separator();
        }
    }
    ImGui::PopStyleColor();

    // Y Channel Harmonics
    ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.2f, 0.5f, 0.2f, 1.0f));
    if (ImGui::CollapsingHeader("Y Channel Harmonics", ImGuiTreeNodeFlags_DefaultOpen)) {
        changed |= ImGui::SliderInt("Y Terms##yn", &m_harmonics.numYTerms, 1, HarmonicsState::MAX_HARMONICS);

        for (int i = 0; i < m_harmonics.numYTerms; ++i) {
            changed |= renderTermControls(m_harmonics.yTerms[i], i + 100, "Y");
            if (i < m_harmonics.numYTerms - 1) ImGui::Separator();
        }
    }
    ImGui::PopStyleColor();

    ImGui::Separator();

    // Presets
    ImGui::Text("Presets:");
    if (ImGui::Button("Classic 3:2", ImVec2(85, 0))) {
        m_harmonics.numXTerms = 1; m_harmonics.numYTerms = 1;
        m_harmonics.xTerms[0] = {1.0f, 3.0f, 0.0f, true, true, false, 0.0f, TWO_PI, 20, false, 1.0f, 5.0f, 20};
        m_harmonics.yTerms[0] = {1.0f, 2.0f, PI / 2.0f, true, true, false, 0.0f, TWO_PI, 20, false, 1.0f, 5.0f, 20};
        changed = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("5:4", ImVec2(50, 0))) {
        m_harmonics.numXTerms = 1; m_harmonics.numYTerms = 1;
        m_harmonics.xTerms[0] = {1.0f, 5.0f, 0.0f, true, true, false, 0.0f, TWO_PI, 20, false, 1.0f, 5.0f, 20};
        m_harmonics.yTerms[0] = {1.0f, 4.0f, PI / 4.0f, true, true, false, 0.0f, TWO_PI, 20, false, 1.0f, 5.0f, 20};
        changed = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Phase Sweep", ImVec2(90, 0))) {
        m_harmonics.numXTerms = 1; m_harmonics.numYTerms = 1;
        m_harmonics.xTerms[0] = {1.0f, 3.0f, 0.0f, true, true, true, 0.0f, TWO_PI, 30, false, 1.0f, 5.0f, 20};
        m_harmonics.yTerms[0] = {1.0f, 2.0f, 0.0f, true, true, false, 0.0f, TWO_PI, 20, false, 1.0f, 5.0f, 20};
        changed = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Freq Sweep", ImVec2(85, 0))) {
        m_harmonics.numXTerms = 1; m_harmonics.numYTerms = 1;
        m_harmonics.xTerms[0] = {1.0f, 1.0f, 0.0f, true, true, false, 0.0f, TWO_PI, 20, true, 1.0f, 8.0f, 40};
        m_harmonics.yTerms[0] = {1.0f, 1.0f, PI / 2.0f, true, true, false, 0.0f, TWO_PI, 20, false, 1.0f, 5.0f, 20};
        changed = true;
    }

    if (ImGui::Button("Complex", ImVec2(70, 0))) {
        m_harmonics.numXTerms = 3; m_harmonics.numYTerms = 3;
        m_harmonics.xTerms[0] = {1.0f, 1.0f, 0.0f, true, true, false, 0.0f, TWO_PI, 20, false, 1.0f, 5.0f, 20};
        m_harmonics.xTerms[1] = {0.3f, 3.0f, 0.0f, true, true, false, 0.0f, TWO_PI, 20, false, 1.0f, 5.0f, 20};
        m_harmonics.xTerms[2] = {0.1f, 5.0f, 0.0f, true, true, false, 0.0f, TWO_PI, 20, false, 1.0f, 5.0f, 20};
        m_harmonics.yTerms[0] = {1.0f, 2.0f, PI / 2.0f, true, true, false, 0.0f, TWO_PI, 20, false, 1.0f, 5.0f, 20};
        m_harmonics.yTerms[1] = {0.3f, 4.0f, 0.0f, true, true, false, 0.0f, TWO_PI, 20, false, 1.0f, 5.0f, 20};
        m_harmonics.yTerms[2] = {0.1f, 6.0f, 0.0f, true, true, false, 0.0f, TWO_PI, 20, false, 1.0f, 5.0f, 20};
        changed = true;
    }

    ImGui::Separator();

    // Live update or manual generate
    if (m_harmonics.liveUpdate && changed) {
        generateHarmonicsPattern(app);
    }

    if (ImGui::Button("Generate Pattern", ImVec2(-1, 40))) {
        generateHarmonicsPattern(app);
    }

    ImGui::End();
}

//==============================================================================
// Drawing Canvas - Fixed using ImGui native drawing
//==============================================================================

void UIManager::renderDrawingCanvas(App& app) {
    // Position: Floating, center area
    ImGui::SetNextWindowPos(ImVec2(350, 150), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(450, 550), ImGuiCond_FirstUseEver);
    ImGui::Begin("Drawing Canvas", &m_showDrawingCanvas);

    ImGui::Text("Click and drag to draw. Release to finish a stroke.");
    ImGui::SliderFloat("Smoothing", &m_drawing.smoothing, 0.0f, 0.9f);

    ImGui::Separator();

    // Canvas dimensions
    ImVec2 canvasPos = ImGui::GetCursorScreenPos();
    ImVec2 canvasSize = ImVec2(400, 400);

    // Create invisible button for interaction
    ImGui::InvisibleButton("##canvas", canvasSize,
        ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);

    bool isHovered = ImGui::IsItemHovered();
    bool isActive = ImGui::IsItemActive();

    // Get draw list for custom rendering
    ImDrawList* drawList = ImGui::GetWindowDrawList();

    // Draw canvas background
    drawList->AddRectFilled(canvasPos, ImVec2(canvasPos.x + canvasSize.x, canvasPos.y + canvasSize.y),
                           IM_COL32(20, 20, 25, 255));
    drawList->AddRect(canvasPos, ImVec2(canvasPos.x + canvasSize.x, canvasPos.y + canvasSize.y),
                     IM_COL32(50, 100, 50, 255));

    // Draw grid
    for (int i = 0; i <= 10; ++i) {
        float t = i / 10.0f;
        float x = canvasPos.x + t * canvasSize.x;
        float y = canvasPos.y + t * canvasSize.y;
        drawList->AddLine(ImVec2(x, canvasPos.y), ImVec2(x, canvasPos.y + canvasSize.y),
                         IM_COL32(40, 60, 40, 100));
        drawList->AddLine(ImVec2(canvasPos.x, y), ImVec2(canvasPos.x + canvasSize.x, y),
                         IM_COL32(40, 60, 40, 100));
    }

    // Center crosshairs
    drawList->AddLine(ImVec2(canvasPos.x + canvasSize.x * 0.5f, canvasPos.y),
                     ImVec2(canvasPos.x + canvasSize.x * 0.5f, canvasPos.y + canvasSize.y),
                     IM_COL32(50, 100, 50, 150));
    drawList->AddLine(ImVec2(canvasPos.x, canvasPos.y + canvasSize.y * 0.5f),
                     ImVec2(canvasPos.x + canvasSize.x, canvasPos.y + canvasSize.y * 0.5f),
                     IM_COL32(50, 100, 50, 150));

    // Handle mouse input
    ImGuiIO& io = ImGui::GetIO();

    if (isHovered && ImGui::IsMouseClicked(0)) {
        // Start a new stroke (but DON'T clear existing drawing!)
        m_drawing.isDrawing = true;
        m_drawing.newStrokeStarted = true;
        // Record where this stroke begins
        m_drawing.strokeStarts.push_back(m_drawing.pointsX.size());
    }

    if (m_drawing.isDrawing && isActive) {
        // Convert mouse position to normalized coordinates (-1 to 1)
        float mouseX = (io.MousePos.x - canvasPos.x) / canvasSize.x * 2.0f - 1.0f;
        float mouseY = -((io.MousePos.y - canvasPos.y) / canvasSize.y * 2.0f - 1.0f); // Flip Y

        mouseX = std::clamp(mouseX, -1.0f, 1.0f);
        mouseY = std::clamp(mouseY, -1.0f, 1.0f);

        if (m_drawing.newStrokeStarted) {
            // First point of a new stroke - add without smoothing or distance check
            m_drawing.pointsX.push_back(mouseX);
            m_drawing.pointsY.push_back(mouseY);
            m_drawing.lastX = mouseX;
            m_drawing.lastY = mouseY;
            m_drawing.newStrokeStarted = false;
        } else if (!m_drawing.pointsX.empty()) {
            // Apply smoothing for subsequent points in the stroke
            mouseX = m_drawing.lastX * m_drawing.smoothing + mouseX * (1.0f - m_drawing.smoothing);
            mouseY = m_drawing.lastY * m_drawing.smoothing + mouseY * (1.0f - m_drawing.smoothing);

            // Only add if moved enough
            float dx = mouseX - m_drawing.pointsX.back();
            float dy = mouseY - m_drawing.pointsY.back();
            if (dx * dx + dy * dy > 0.0001f) {
                m_drawing.pointsX.push_back(mouseX);
                m_drawing.pointsY.push_back(mouseY);
            }
            m_drawing.lastX = mouseX;
            m_drawing.lastY = mouseY;
        }
    }

    if (m_drawing.isDrawing && ImGui::IsMouseReleased(0)) {
        m_drawing.isDrawing = false;
    }

    // Draw the path - skip lines between different strokes
    if (m_drawing.pointsX.size() >= 2) {
        // Build a set of stroke start indices for fast lookup
        std::set<size_t> strokeStartSet(m_drawing.strokeStarts.begin(), m_drawing.strokeStarts.end());

        for (size_t i = 0; i < m_drawing.pointsX.size() - 1; ++i) {
            // Don't draw a line if the next point is the start of a new stroke
            if (strokeStartSet.count(i + 1) > 0) {
                continue;
            }

            // Convert normalized to screen coordinates
            float x1 = canvasPos.x + (m_drawing.pointsX[i] + 1.0f) * 0.5f * canvasSize.x;
            float y1 = canvasPos.y + (-m_drawing.pointsY[i] + 1.0f) * 0.5f * canvasSize.y;
            float x2 = canvasPos.x + (m_drawing.pointsX[i + 1] + 1.0f) * 0.5f * canvasSize.x;
            float y2 = canvasPos.y + (-m_drawing.pointsY[i + 1] + 1.0f) * 0.5f * canvasSize.y;

            // Glow
            drawList->AddLine(ImVec2(x1, y1), ImVec2(x2, y2), IM_COL32(50, 200, 80, 100), 4.0f);
            // Main line
            drawList->AddLine(ImVec2(x1, y1), ImVec2(x2, y2), IM_COL32(100, 255, 150, 255), 2.0f);
        }
    }

    // Show cursor position
    if (isHovered) {
        float mouseX = (io.MousePos.x - canvasPos.x) / canvasSize.x * 2.0f - 1.0f;
        float mouseY = -((io.MousePos.y - canvasPos.y) / canvasSize.y * 2.0f - 1.0f);
        ImGui::Text("Cursor: (%.2f, %.2f)", mouseX, mouseY);
    }

    ImGui::Separator();
    ImGui::Text("Points: %zu  Strokes: %zu", m_drawing.pointsX.size(), m_drawing.strokeStarts.size());

    if (ImGui::Button("Clear Canvas", ImVec2(-1, 0))) {
        m_drawing.pointsX.clear();
        m_drawing.pointsY.clear();
        m_drawing.strokeStarts.clear();
    }

    if (ImGui::Button("Use as Pattern", ImVec2(-1, 30))) {
        if (m_drawing.pointsX.size() >= 2) {
            Pattern& pattern = app.getPattern();
            pattern.clear();
            pattern.x = m_drawing.pointsX;
            pattern.y = m_drawing.pointsY;
            app.getAudioEngine().setPattern(pattern);
        }
    }

    ImGui::End();
}

//==============================================================================
// 3D Shape Generator
//==============================================================================

void UIManager::generate3DShapePattern(App& app) {
    Pattern& pattern = app.getPattern();
    pattern.clear();

    std::vector<Vec3> vertices;
    std::vector<std::pair<int, int>> edges;

    float rx = m_shape3D.rotationX * PI / 180.0f;
    float ry = m_shape3D.rotationY * PI / 180.0f;
    float rz = m_shape3D.rotationZ * PI / 180.0f;

    switch (m_shape3D.shapeType) {
        case Shape3DState::ShapeType::Cube: {
            // 8 vertices of a cube
            float s = 0.5f;
            vertices = {
                {-s, -s, -s}, {s, -s, -s}, {s, s, -s}, {-s, s, -s},
                {-s, -s, s}, {s, -s, s}, {s, s, s}, {-s, s, s}
            };
            edges = {{0,1},{1,2},{2,3},{3,0}, {4,5},{5,6},{6,7},{7,4}, {0,4},{1,5},{2,6},{3,7}};
            break;
        }
        case Shape3DState::ShapeType::Tetrahedron: {
            float s = 0.6f;
            vertices = {{s, s, s}, {s, -s, -s}, {-s, s, -s}, {-s, -s, s}};
            edges = {{0,1},{0,2},{0,3},{1,2},{1,3},{2,3}};
            break;
        }
        case Shape3DState::ShapeType::Octahedron: {
            float s = 0.7f;
            vertices = {{s,0,0}, {-s,0,0}, {0,s,0}, {0,-s,0}, {0,0,s}, {0,0,-s}};
            edges = {{0,2},{0,3},{0,4},{0,5}, {1,2},{1,3},{1,4},{1,5}, {2,4},{2,5},{3,4},{3,5}};
            break;
        }
        case Shape3DState::ShapeType::Icosahedron: {
            float phi = (1.0f + std::sqrt(5.0f)) / 2.0f;
            float s = 0.4f;
            vertices = {
                {0, s, s*phi}, {0, s, -s*phi}, {0, -s, s*phi}, {0, -s, -s*phi},
                {s, s*phi, 0}, {s, -s*phi, 0}, {-s, s*phi, 0}, {-s, -s*phi, 0},
                {s*phi, 0, s}, {s*phi, 0, -s}, {-s*phi, 0, s}, {-s*phi, 0, -s}
            };
            edges = {{0,2},{0,4},{0,6},{0,8},{0,10},{2,5},{2,7},{2,8},{2,10},{4,6},{4,8},{4,9},
                     {5,7},{5,8},{5,9},{6,10},{6,11},{7,10},{7,11},{1,3},{1,4},{1,6},{1,9},{1,11},
                     {3,5},{3,7},{3,9},{3,11},{8,9},{10,11}};
            break;
        }
        case Shape3DState::ShapeType::Dodecahedron: {
            // Dodecahedron: 20 vertices, 30 edges
            float phi = (1.0f + std::sqrt(5.0f)) / 2.0f;
            float s = 0.4f;
            float invPhi = 1.0f / phi;
            vertices = {
                // Cube vertices
                {s, s, s}, {s, s, -s}, {s, -s, s}, {s, -s, -s},
                {-s, s, s}, {-s, s, -s}, {-s, -s, s}, {-s, -s, -s},
                // Rectangle vertices on XY plane
                {0, s*phi, s*invPhi}, {0, s*phi, -s*invPhi}, {0, -s*phi, s*invPhi}, {0, -s*phi, -s*invPhi},
                // Rectangle vertices on YZ plane
                {s*invPhi, 0, s*phi}, {-s*invPhi, 0, s*phi}, {s*invPhi, 0, -s*phi}, {-s*invPhi, 0, -s*phi},
                // Rectangle vertices on XZ plane
                {s*phi, s*invPhi, 0}, {s*phi, -s*invPhi, 0}, {-s*phi, s*invPhi, 0}, {-s*phi, -s*invPhi, 0}
            };
            edges = {{0,8},{0,12},{0,16}, {1,9},{1,14},{1,16}, {2,10},{2,12},{2,17}, {3,11},{3,14},{3,17},
                     {4,8},{4,13},{4,18}, {5,9},{5,15},{5,18}, {6,10},{6,13},{6,19}, {7,11},{7,15},{7,19},
                     {8,9},{10,11},{12,13},{14,15},{16,17},{18,19}};
            break;
        }
        case Shape3DState::ShapeType::Sphere: {
            int res = m_shape3D.resolution;
            int latDiv = static_cast<int>(std::sqrt(res / 2.0f));
            int lonDiv = latDiv * 2;
            for (int lat = 0; lat <= latDiv; ++lat) {
                float theta = PI * lat / latDiv;
                for (int lon = 0; lon <= lonDiv; ++lon) {
                    float phi = TWO_PI * lon / lonDiv;
                    float x = 0.7f * std::sin(theta) * std::cos(phi);
                    float y = 0.7f * std::sin(theta) * std::sin(phi);
                    float z = 0.7f * std::cos(theta);
                    vertices.push_back({x, y, z});
                    int idx = lat * (lonDiv + 1) + lon;
                    if (lon < lonDiv) edges.push_back({idx, idx + 1});
                    if (lat < latDiv) edges.push_back({idx, idx + lonDiv + 1});
                }
            }
            break;
        }
        case Shape3DState::ShapeType::Torus: {
            int res = m_shape3D.resolution;
            int majorDiv = static_cast<int>(std::sqrt(res));
            int minorDiv = majorDiv;
            float R = m_shape3D.torusMajorRadius;
            float r = m_shape3D.torusMinorRadius;
            for (int i = 0; i < majorDiv; ++i) {
                float u = TWO_PI * i / majorDiv;
                for (int j = 0; j < minorDiv; ++j) {
                    float v = TWO_PI * j / minorDiv;
                    float x = (R + r * std::cos(v)) * std::cos(u);
                    float y = (R + r * std::cos(v)) * std::sin(u);
                    float z = r * std::sin(v);
                    vertices.push_back({x, y, z});
                    int idx = i * minorDiv + j;
                    int nextI = ((i + 1) % majorDiv) * minorDiv + j;
                    int nextJ = i * minorDiv + ((j + 1) % minorDiv);
                    edges.push_back({idx, nextI});
                    edges.push_back({idx, nextJ});
                }
            }
            break;
        }
        case Shape3DState::ShapeType::Cylinder: {
            int res = 32;
            float h = 0.8f, r = 0.5f;
            for (int i = 0; i < res; ++i) {
                float theta = TWO_PI * i / res;
                float x = r * std::cos(theta), z = r * std::sin(theta);
                vertices.push_back({x, -h, z});
                vertices.push_back({x, h, z});
                int base = i * 2;
                edges.push_back({base, base + 1});
                edges.push_back({base, ((i + 1) % res) * 2});
                edges.push_back({base + 1, ((i + 1) % res) * 2 + 1});
            }
            break;
        }
        case Shape3DState::ShapeType::Cone: {
            int res = 32;
            float h = 0.8f, r = 0.6f;
            int tipIdx = res;
            for (int i = 0; i < res; ++i) {
                float theta = TWO_PI * i / res;
                vertices.push_back({r * std::cos(theta), -h, r * std::sin(theta)});
                edges.push_back({i, (i + 1) % res});
            }
            vertices.push_back({0, h, 0});
            for (int i = 0; i < res; ++i) edges.push_back({i, tipIdx});
            break;
        }
        case Shape3DState::ShapeType::Pyramid: {
            // Square-based pyramid
            float s = 0.5f, h = 0.8f;
            vertices = {
                {-s, -h, -s}, {s, -h, -s}, {s, -h, s}, {-s, -h, s},  // Base
                {0, h, 0}  // Apex
            };
            edges = {{0,1},{1,2},{2,3},{3,0}, {0,4},{1,4},{2,4},{3,4}};
            break;
        }
        case Shape3DState::ShapeType::Prism: {
            // N-sided prism
            int sides = m_shape3D.prismSides;
            float h = 0.7f, r = 0.5f;
            for (int i = 0; i < sides; ++i) {
                float theta = TWO_PI * i / sides;
                float x = r * std::cos(theta), z = r * std::sin(theta);
                vertices.push_back({x, -h, z});  // Bottom
                vertices.push_back({x, h, z});   // Top
                int base = i * 2;
                edges.push_back({base, base + 1});  // Vertical edge
                edges.push_back({base, ((i + 1) % sides) * 2});  // Bottom edge
                edges.push_back({base + 1, ((i + 1) % sides) * 2 + 1});  // Top edge
            }
            break;
        }
        case Shape3DState::ShapeType::Spiral3D: {
            int res = m_shape3D.resolution;
            float turns = 5.0f;
            for (int i = 0; i < res; ++i) {
                float t = static_cast<float>(i) / res;
                float theta = turns * TWO_PI * t;
                float r = 0.1f + 0.5f * t;
                float x = r * std::cos(theta);
                float y = -0.8f + 1.6f * t;
                float z = r * std::sin(theta);
                vertices.push_back({x, y, z});
                if (i > 0) edges.push_back({i - 1, i});
            }
            break;
        }
        case Shape3DState::ShapeType::Helix: {
            // Double helix (DNA-like)
            int res = m_shape3D.resolution;
            float turns = m_shape3D.helixTurns;
            float r = m_shape3D.helixRadius;
            float h = 0.8f;
            for (int i = 0; i < res; ++i) {
                float t = static_cast<float>(i) / res;
                float theta = turns * TWO_PI * t;
                // First strand
                float x1 = r * std::cos(theta);
                float z1 = r * std::sin(theta);
                float y = -h + 2.0f * h * t;
                vertices.push_back({x1, y, z1});
                // Second strand (offset by PI)
                float x2 = r * std::cos(theta + PI);
                float z2 = r * std::sin(theta + PI);
                vertices.push_back({x2, y, z2});

                int base = i * 2;
                if (i > 0) {
                    edges.push_back({base - 2, base});      // Connect strand 1
                    edges.push_back({base - 1, base + 1});  // Connect strand 2
                }
                // Connect the two strands at intervals
                if (i % (res / 10) == 0) {
                    edges.push_back({base, base + 1});
                }
            }
            break;
        }
        case Shape3DState::ShapeType::Knot: {
            int res = m_shape3D.resolution;
            float p = m_shape3D.knotP, q = m_shape3D.knotQ;
            for (int i = 0; i < res; ++i) {
                float t = TWO_PI * i / res;
                float r = 0.3f * std::cos(q * t) + 0.6f;
                float x = r * std::cos(p * t);
                float y = r * std::sin(p * t);
                float z = 0.3f * std::sin(q * t);
                vertices.push_back({x, y, z});
                edges.push_back({i, (i + 1) % res});
            }
            break;
        }
        case Shape3DState::ShapeType::MobiusStrip: {
            int res = m_shape3D.resolution;
            int uDiv = static_cast<int>(std::sqrt(res));
            int vDiv = std::max(3, uDiv / 4);
            float R = 0.6f;  // Major radius
            float w = 0.3f;  // Half-width of strip

            for (int i = 0; i <= uDiv; ++i) {
                float u = TWO_PI * i / uDiv;
                for (int j = 0; j <= vDiv; ++j) {
                    float v = -w + 2.0f * w * j / vDiv;
                    // Mobius strip parametric equations
                    float x = (R + v * std::cos(u / 2.0f)) * std::cos(u);
                    float y = (R + v * std::cos(u / 2.0f)) * std::sin(u);
                    float z = v * std::sin(u / 2.0f);
                    vertices.push_back({x, y, z});

                    int idx = i * (vDiv + 1) + j;
                    if (j < vDiv) edges.push_back({idx, idx + 1});
                    if (i < uDiv) edges.push_back({idx, idx + vDiv + 1});
                }
            }
            break;
        }
        case Shape3DState::ShapeType::KleinBottle: {
            int res = m_shape3D.resolution;
            int uDiv = static_cast<int>(std::sqrt(res));
            int vDiv = uDiv;

            for (int i = 0; i < uDiv; ++i) {
                float u = TWO_PI * i / uDiv;
                for (int j = 0; j < vDiv; ++j) {
                    float v = TWO_PI * j / vDiv;
                    // Klein bottle parametric equations (figure-8 immersion)
                    float r = 0.3f;
                    float x = (r + std::cos(u / 2.0f) * std::sin(v) - std::sin(u / 2.0f) * std::sin(2.0f * v)) * std::cos(u);
                    float y = (r + std::cos(u / 2.0f) * std::sin(v) - std::sin(u / 2.0f) * std::sin(2.0f * v)) * std::sin(u);
                    float z = std::sin(u / 2.0f) * std::sin(v) + std::cos(u / 2.0f) * std::sin(2.0f * v);
                    vertices.push_back({x * 0.7f, y * 0.7f, z * 0.7f});

                    int idx = i * vDiv + j;
                    int nextI = ((i + 1) % uDiv) * vDiv + j;
                    int nextJ = i * vDiv + ((j + 1) % vDiv);
                    edges.push_back({idx, nextI});
                    edges.push_back({idx, nextJ});
                }
            }
            break;
        }
        case Shape3DState::ShapeType::Spring: {
            // Coil spring (3D helix with circular cross-section)
            int res = m_shape3D.resolution;
            int coils = 6;
            int crossSectionPts = 8;
            float springRadius = 0.5f;
            float wireRadius = 0.1f;
            float length = 1.4f;

            for (int i = 0; i < res; ++i) {
                float t = static_cast<float>(i) / res;
                float theta = coils * TWO_PI * t;
                // Center of coil at this point
                float cx = springRadius * std::cos(theta);
                float cy = -length / 2.0f + length * t;
                float cz = springRadius * std::sin(theta);

                // Normal vectors for cross-section
                float nx = -std::sin(theta);
                float nz = std::cos(theta);

                for (int j = 0; j < crossSectionPts; ++j) {
                    float phi = TWO_PI * j / crossSectionPts;
                    float dx = wireRadius * std::cos(phi);
                    float dy = wireRadius * std::sin(phi);

                    float x = cx + dx * nx;
                    float y = cy + dy;
                    float z = cz + dx * nz;
                    vertices.push_back({x, y, z});

                    int idx = i * crossSectionPts + j;
                    int nextJ = i * crossSectionPts + ((j + 1) % crossSectionPts);
                    edges.push_back({idx, nextJ});  // Around cross-section
                    if (i > 0) {
                        int prevIdx = (i - 1) * crossSectionPts + j;
                        edges.push_back({prevIdx, idx});  // Along coil
                    }
                }
            }
            break;
        }
        case Shape3DState::ShapeType::Star3D: {
            // 3D star (stellated shape)
            int points = m_shape3D.starPoints;
            float outerR = 0.7f;
            float innerR = m_shape3D.starInnerRadius;
            float depth = 0.3f;

            // Create front and back star shapes
            int vertPerFace = points * 2;
            for (int face = 0; face < 2; ++face) {
                float z = (face == 0) ? depth : -depth;
                for (int i = 0; i < points * 2; ++i) {
                    float theta = PI * i / points - PI / 2.0f;
                    float r = (i % 2 == 0) ? outerR : innerR;
                    float x = r * std::cos(theta);
                    float y = r * std::sin(theta);
                    vertices.push_back({x, y, z});

                    // Connect around star
                    int idx = face * vertPerFace + i;
                    int nextIdx = face * vertPerFace + ((i + 1) % (points * 2));
                    edges.push_back({idx, nextIdx});
                }
            }
            // Connect front to back
            for (int i = 0; i < points * 2; ++i) {
                edges.push_back({i, vertPerFace + i});
            }
            break;
        }
        case Shape3DState::ShapeType::Text3D: {
            // Generate 3D text using stroke font
            std::vector<float> textX, textY;
            std::vector<size_t> textStrokeStarts;
            Text3DGenerator::generateText3D(
                m_shape3D.textBuffer,
                0.5f,  // Character height
                m_shape3D.textDepth,
                m_shape3D.textConnectFaces,
                textX, textY, textStrokeStarts
            );

            // Apply 3D rotation to the text
            for (size_t i = 0; i < textX.size(); ++i) {
                Vec3 v(textX[i], textY[i], 0.0f);
                v = rotateX(v, rx);
                v = rotateY(v, ry);
                v = rotateZ(v, rz);
                float projX_val, projY_val;
                project(v, m_shape3D.perspective, m_shape3D.scale, projX_val, projY_val);
                textX[i] = projX_val;
                textY[i] = projY_val;
            }

            // Generate pattern with proper stroke handling
            pattern.x.clear();
            pattern.y.clear();
            int pointsPerSegment = std::max(2, m_shape3D.resolution / static_cast<int>(std::max(size_t(1), textX.size())));

            for (size_t strokeIdx = 0; strokeIdx < textStrokeStarts.size(); ++strokeIdx) {
                size_t start = textStrokeStarts[strokeIdx];
                size_t end = (strokeIdx + 1 < textStrokeStarts.size()) ? textStrokeStarts[strokeIdx + 1] : textX.size();

                for (size_t i = start; i + 1 < end; ++i) {
                    float x1 = textX[i], y1 = textY[i];
                    float x2 = textX[i + 1], y2 = textY[i + 1];
                    for (int p = 0; p < pointsPerSegment; ++p) {
                        float t = static_cast<float>(p) / (pointsPerSegment - 1);
                        pattern.x.push_back(x1 * (1 - t) + x2 * t);
                        pattern.y.push_back(y1 * (1 - t) + y2 * t);
                    }
                }
            }

            app.getAudioEngine().setPattern(pattern);
            return;  // Early return since we handle pattern directly
        }
    }

    // Transform and project vertices
    std::vector<float> projX(vertices.size()), projY(vertices.size());
    for (size_t i = 0; i < vertices.size(); ++i) {
        Vec3 v = vertices[i];
        v = rotateX(v, rx);
        v = rotateY(v, ry);
        v = rotateZ(v, rz);
        project(v, m_shape3D.perspective, m_shape3D.scale, projX[i], projY[i]);
    }

    // Generate pattern by tracing edges
    int pointsPerEdge = std::max(2, m_shape3D.resolution / static_cast<int>(edges.size()));
    pattern.x.reserve(edges.size() * pointsPerEdge);
    pattern.y.reserve(edges.size() * pointsPerEdge);

    for (const auto& edge : edges) {
        float x1 = projX[edge.first], y1 = projY[edge.first];
        float x2 = projX[edge.second], y2 = projY[edge.second];
        for (int p = 0; p < pointsPerEdge; ++p) {
            float t = static_cast<float>(p) / (pointsPerEdge - 1);
            pattern.x.push_back(x1 * (1 - t) + x2 * t);
            pattern.y.push_back(y1 * (1 - t) + y2 * t);
        }
    }

    app.getAudioEngine().setPattern(pattern);
}

void UIManager::render3DShapeGenerator(App& app) {
    // Position: Floating, upper center
    ImGui::SetNextWindowPos(ImVec2(400, 100), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(350, 550), ImGuiCond_FirstUseEver);
    ImGui::Begin("3D Shape Generator", &m_show3DShapeGenerator);

    ImGui::Text("Project 3D wireframes to 2D XY");
    ImGui::Separator();

    // Shape selection - expanded list
    const char* shapes[] = {
        "Cube", "Tetrahedron", "Octahedron", "Icosahedron", "Dodecahedron",
        "Sphere", "Torus", "Cylinder", "Cone", "Pyramid", "Prism",
        "3D Spiral", "Double Helix", "Trefoil Knot",
        "Mobius Strip", "Klein Bottle", "Spring", "3D Star",
        "3D Text"
    };
    int shapeIdx = static_cast<int>(m_shape3D.shapeType);
    if (ImGui::Combo("Shape", &shapeIdx, shapes, IM_ARRAYSIZE(shapes))) {
        m_shape3D.shapeType = static_cast<Shape3DState::ShapeType>(shapeIdx);
        generate3DShapePattern(app);
    }

    ImGui::Separator();
    ImGui::Text("Rotation (degrees)");

    bool rotChanged = false;
    rotChanged |= ImGui::SliderFloat("X Rotation", &m_shape3D.rotationX, 0.0f, 360.0f);
    rotChanged |= ImGui::SliderFloat("Y Rotation", &m_shape3D.rotationY, 0.0f, 360.0f);
    rotChanged |= ImGui::SliderFloat("Z Rotation", &m_shape3D.rotationZ, 0.0f, 360.0f);

    ImGui::Separator();
    ImGui::Text("Animation");
    ImGui::Checkbox("Animate", &m_shape3D.animate);
    if (m_shape3D.animate) {
        ImGui::SliderFloat("X Speed", &m_shape3D.rotationSpeedX, -5.0f, 5.0f);
        ImGui::SliderFloat("Y Speed", &m_shape3D.rotationSpeedY, -5.0f, 5.0f);
        ImGui::SliderFloat("Z Speed", &m_shape3D.rotationSpeedZ, -5.0f, 5.0f);
    }

    ImGui::Separator();
    ImGui::Text("Projection");
    rotChanged |= ImGui::SliderFloat("Scale", &m_shape3D.scale, 0.2f, 1.5f);
    rotChanged |= ImGui::SliderFloat("Perspective", &m_shape3D.perspective, 0.0f, 5.0f, "%.1f (0=ortho)");
    rotChanged |= ImGui::SliderInt("Resolution", &m_shape3D.resolution, 100, 2000);

    // Shape-specific parameters
    if (m_shape3D.shapeType == Shape3DState::ShapeType::Torus) {
        ImGui::Separator();
        ImGui::Text("Torus Parameters");
        rotChanged |= ImGui::SliderFloat("Major Radius", &m_shape3D.torusMajorRadius, 0.3f, 1.0f);
        rotChanged |= ImGui::SliderFloat("Minor Radius", &m_shape3D.torusMinorRadius, 0.1f, 0.5f);
    }

    if (m_shape3D.shapeType == Shape3DState::ShapeType::Knot) {
        ImGui::Separator();
        ImGui::Text("Knot Parameters");
        rotChanged |= ImGui::SliderFloat("P (windings)", &m_shape3D.knotP, 1.0f, 7.0f);
        rotChanged |= ImGui::SliderFloat("Q (windings)", &m_shape3D.knotQ, 1.0f, 7.0f);
    }

    if (m_shape3D.shapeType == Shape3DState::ShapeType::Helix) {
        ImGui::Separator();
        ImGui::Text("Helix Parameters");
        rotChanged |= ImGui::SliderFloat("Turns", &m_shape3D.helixTurns, 1.0f, 10.0f);
        rotChanged |= ImGui::SliderFloat("Radius", &m_shape3D.helixRadius, 0.2f, 0.8f);
    }

    if (m_shape3D.shapeType == Shape3DState::ShapeType::Prism) {
        ImGui::Separator();
        ImGui::Text("Prism Parameters");
        rotChanged |= ImGui::SliderInt("Sides", &m_shape3D.prismSides, 3, 12);
    }

    if (m_shape3D.shapeType == Shape3DState::ShapeType::Star3D) {
        ImGui::Separator();
        ImGui::Text("Star Parameters");
        rotChanged |= ImGui::SliderInt("Points", &m_shape3D.starPoints, 3, 12);
        rotChanged |= ImGui::SliderFloat("Inner Radius", &m_shape3D.starInnerRadius, 0.1f, 0.6f);
    }

    if (m_shape3D.shapeType == Shape3DState::ShapeType::Text3D) {
        ImGui::Separator();
        ImGui::Text("3D Text");
        ImGui::PushItemWidth(-1);
        if (ImGui::InputText("##TextInput", m_shape3D.textBuffer, sizeof(m_shape3D.textBuffer),
                             ImGuiInputTextFlags_EnterReturnsTrue)) {
            generate3DShapePattern(app);
        }
        ImGui::PopItemWidth();
        ImGui::TextDisabled("(Press Enter to apply)");

        rotChanged |= ImGui::SliderFloat("Depth", &m_shape3D.textDepth, 0.0f, 1.0f);
        rotChanged |= ImGui::Checkbox("Connect Faces", &m_shape3D.textConnectFaces);

        ImGui::TextDisabled("A-Z, 0-9, punctuation supported");
    }

    ImGui::Separator();

    if (rotChanged && !m_shape3D.animate) {
        generate3DShapePattern(app);
    }

    if (ImGui::Button("Generate Shape", ImVec2(-1, 40))) {
        generate3DShapePattern(app);
    }

    ImGui::End();
}

//==============================================================================
// Sound Pad and Display Settings (unchanged)
//==============================================================================

void UIManager::renderSoundPad(App& app) {
    // Position: Floating, left side below Controls
    ImGui::SetNextWindowPos(ImVec2(10, 490), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(280, 300), ImGuiCond_FirstUseEver);
    ImGui::Begin("Sound Pad", &m_showSoundPad);
    ImGui::Text("16-Step XY Sequencer");
    ImGui::Separator();

    float buttonSize = 60.0f;
    for (int row = 0; row < SoundPadState::GRID_SIZE; ++row) {
        for (int col = 0; col < SoundPadState::GRID_SIZE; ++col) {
            int idx = row * SoundPadState::GRID_SIZE + col;
            if (col > 0) ImGui::SameLine();

            if (m_soundPad.active[idx]) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.6f, 0.3f, 1.0f));
            else ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.3f, 0.3f, 0.3f, 1.0f));

            char label[32];
            snprintf(label, sizeof(label), "%d\n%.1f,%.1f", idx + 1, m_soundPad.x[idx], m_soundPad.y[idx]);
            if (ImGui::Button(label, ImVec2(buttonSize, buttonSize))) m_soundPad.active[idx] = !m_soundPad.active[idx];
            ImGui::PopStyleColor();

            if (ImGui::IsItemActive() && ImGui::IsMouseDragging(0)) {
                ImVec2 delta = ImGui::GetMouseDragDelta(0);
                m_soundPad.x[idx] = std::clamp(m_soundPad.x[idx] + delta.x * 0.01f, -1.0f, 1.0f);
                m_soundPad.y[idx] = std::clamp(m_soundPad.y[idx] - delta.y * 0.01f, -1.0f, 1.0f);
                ImGui::ResetMouseDragDelta(0);
            }
        }
    }

    ImGui::Separator();
    if (ImGui::Button("Circle Preset", ImVec2(-1, 0))) {
        for (int i = 0; i < SoundPadState::NUM_STEPS; ++i) {
            float angle = TWO_PI * static_cast<float>(i) / SoundPadState::NUM_STEPS;
            m_soundPad.x[i] = std::cos(angle) * 0.8f; m_soundPad.y[i] = std::sin(angle) * 0.8f; m_soundPad.active[i] = true;
        }
    }
    if (ImGui::Button("Clear All", ImVec2(-1, 0))) {
        for (int i = 0; i < SoundPadState::NUM_STEPS; ++i) { m_soundPad.x[i] = 0; m_soundPad.y[i] = 0; m_soundPad.active[i] = false; }
    }

    ImGui::Separator();
    if (ImGui::Button("Generate Pattern", ImVec2(-1, 30))) {
        Pattern& pattern = app.getPattern();
        pattern.clear();
        std::vector<int> activeIndices;
        for (int i = 0; i < SoundPadState::NUM_STEPS; ++i) if (m_soundPad.active[i]) activeIndices.push_back(i);
        if (!activeIndices.empty()) {
            int pointsPerStep = 100;
            for (size_t s = 0; s < activeIndices.size(); ++s) {
                int curr = activeIndices[s], next = activeIndices[(s + 1) % activeIndices.size()];
                for (int p = 0; p < pointsPerStep; ++p) {
                    float t = static_cast<float>(p) / pointsPerStep;
                    pattern.x.push_back(m_soundPad.x[curr] * (1 - t) + m_soundPad.x[next] * t);
                    pattern.y.push_back(m_soundPad.y[curr] * (1 - t) + m_soundPad.y[next] * t);
                }
            }
            app.getAudioEngine().setPattern(pattern);
        }
    }
    ImGui::End();
}

void UIManager::renderDisplaySettings(App& app) {
    (void)app;
    // Position: Floating, right of oscilloscope
    ImGui::SetNextWindowPos(ImVec2(600, 150), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(300, 500), ImGuiCond_FirstUseEver);
    ImGui::Begin("Display Settings", &m_showDisplaySettings);

    //==========================================================================
    // PHOSPHOR TYPE PRESETS
    //==========================================================================
    ImGui::Text("Phosphor Type Presets");
    if (ImGui::Button("P31 (Tek 465B)", ImVec2(100, 0))) {
        // Tektronix 465B default - yellowish-green P31
        m_phosphor.colorR = 0.35f; m_phosphor.colorG = 1.0f; m_phosphor.colorB = 0.25f;
        m_phosphor.decayTime = 50.0f; m_phosphor.decayExponent = 2.0f;
    }
    ImGui::SameLine();
    if (ImGui::Button("P1 Green", ImVec2(80, 0))) {
        m_phosphor.colorR = 0.2f; m_phosphor.colorG = 1.0f; m_phosphor.colorB = 0.35f;
        m_phosphor.decayTime = 80.0f; m_phosphor.decayExponent = 1.8f;
    }
    ImGui::SameLine();
    if (ImGui::Button("P7 Blue", ImVec2(70, 0))) {
        m_phosphor.colorR = 0.3f; m_phosphor.colorG = 0.6f; m_phosphor.colorB = 1.0f;
        m_phosphor.decayTime = 120.0f; m_phosphor.decayExponent = 1.5f;
    }
    ImGui::SameLine();
    if (ImGui::Button("Amber", ImVec2(60, 0))) {
        m_phosphor.colorR = 1.0f; m_phosphor.colorG = 0.7f; m_phosphor.colorB = 0.1f;
        m_phosphor.decayTime = 40.0f; m_phosphor.decayExponent = 2.2f;
    }

    float color[3] = {m_phosphor.colorR, m_phosphor.colorG, m_phosphor.colorB};
    if (ImGui::ColorEdit3("Custom Color", color)) {
        m_phosphor.colorR = color[0]; m_phosphor.colorG = color[1]; m_phosphor.colorB = color[2];
    }

    ImGui::Separator();

    //==========================================================================
    // PHOSPHOR DECAY (PERSISTENCE)
    //==========================================================================
    if (ImGui::CollapsingHeader("Phosphor Persistence", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "Controls how long traces persist");
        ImGui::SliderFloat("Decay Time (ms)", &m_phosphor.decayTime, 10.0f, 200.0f, "%.0f ms");
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("P31: ~50ms, P1: ~80ms, P7: ~120ms");
        ImGui::SliderFloat("Decay Curve", &m_phosphor.decayExponent, 1.0f, 3.0f, "%.1f");
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("1=linear, 2=exponential (realistic)");
    }

    //==========================================================================
    // BEAM INTENSITY (VELOCITY-BASED)
    //==========================================================================
    if (ImGui::CollapsingHeader("Beam Intensity", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "Real CRT: slower beam = brighter trace");
        ImGui::SliderFloat("Brightness", &m_phosphor.brightness, 0.5f, 3.0f);
        ImGui::SliderFloat("Velocity Effect", &m_phosphor.velocityEffect, 0.0f, 1.0f);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("How much beam speed affects brightness");
        ImGui::SliderFloat("Min Brightness", &m_phosphor.minBrightness, 0.0f, 0.5f);
        ImGui::SliderFloat("Max Brightness", &m_phosphor.maxBrightness, 0.5f, 1.0f);
    }

    //==========================================================================
    // GLOW / BLOOM
    //==========================================================================
    if (ImGui::CollapsingHeader("Glow / Bloom")) {
        ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "Light scatter in phosphor particles");
        ImGui::SliderFloat("Glow Intensity", &m_phosphor.glowIntensity, 0.0f, 1.0f);
        ImGui::SliderFloat("Glow Radius", &m_phosphor.glowRadius, 1.0f, 8.0f);
        ImGui::SliderFloat("Glow Falloff", &m_phosphor.glowFalloff, 1.0f, 3.0f);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Exponential falloff rate");
    }

    //==========================================================================
    // BEAM CHARACTERISTICS
    //==========================================================================
    if (ImGui::CollapsingHeader("Beam Characteristics")) {
        ImGui::SliderFloat("Beam Width", &m_phosphor.beamWidth, 0.5f, 3.0f);
        ImGui::SliderFloat("Beam Focus", &m_phosphor.beamFocus, 0.5f, 1.0f);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Higher = sharper trace");
        ImGui::SliderFloat("Spot Size", &m_phosphor.beamSpotSize, 1.0f, 4.0f);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Bloom at slow-moving beam positions");
    }

    //==========================================================================
    // GRATICULE
    //==========================================================================
    if (ImGui::CollapsingHeader("Graticule")) {
        ImGui::Checkbox("Show Graticule", &m_phosphor.showGrid);
        ImGui::Checkbox("Show Tick Marks", &m_phosphor.showGraticuleMarks);
        ImGui::SliderFloat("Grid Alpha", &m_phosphor.gridAlpha, 0.0f, 0.4f);
    }

    //==========================================================================
    // CRT EFFECTS (ADVANCED)
    //==========================================================================
    if (ImGui::CollapsingHeader("CRT Effects (Advanced)")) {
        ImGui::SliderFloat("Screen Curvature", &m_phosphor.screenCurvature, 0.0f, 0.1f);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Not yet implemented");
        ImGui::SliderFloat("Vignette", &m_phosphor.vignetteStrength, 0.0f, 0.3f);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Edge darkening (not yet implemented)");
        ImGui::SliderFloat("Analog Noise", &m_phosphor.noiseAmount, 0.0f, 0.1f);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Not yet implemented");
        ImGui::SliderFloat("Scanlines", &m_phosphor.scanlineEffect, 0.0f, 0.5f);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Not applicable to XY mode");
    }

    //==========================================================================
    // DISPLAY BUFFER
    //==========================================================================
    ImGui::Separator();
    ImGui::SliderInt("Trail Samples", &m_phosphor.trailSamples, 1024, 16384);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("More samples = longer visible trail");

    ImGui::End();
}

} // namespace oscilloplot

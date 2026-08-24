#include "ui_manager.hpp"
#include "app.hpp"
#include "data/pattern.hpp"
#include "audio/audio_engine.hpp"
#include "generators/test_pattern.hpp"
#include "generators/stroke_font.hpp"
#include "data/file_loader.hpp"
#include "data/file_saver.hpp"
#include "data/obj_loader.hpp"
#include "audio/wav_export.hpp"

#include <imgui.h>
#include <implot.h>
#include <SDL_opengl.h>
#include <portable-file-dialogs.h>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <set>
#include <cfloat>
#include <vector>

// Single-header PNG writer, matching the existing stb usage for image loading.
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STBI_WRITE_NO_STDIO_STDARG
#include <stb_image_write.h>

#ifndef OSCILLOPLOT_VERSION
#define OSCILLOPLOT_VERSION "dev"
#endif

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

    // Sound pad seeds itself with a ready-to-play circle (SoundPadState ctor)
}

UIManager::~UIManager() = default;

bool UIManager::init() {
    applyStyle();
    loadRecentFiles();
    return true;
}

void UIManager::shutdown() {
}

void UIManager::render(App& app) {
    // Generate default rotating torus on first render
    if (m_firstRender) {
        generate3DShapePattern(app);
        m_firstRender = false;
    }

    updateSequencer(app);

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
    if (m_showSequencer) {
        renderSequencer(app);
    }
    if (m_showImageVectorizer) {
        renderImageVectorizer(app);
    }
    if (m_showDisplaySettings) {
        renderDisplaySettings(app);
    }

    if (m_showDemoWindow) {
        ImGui::ShowDemoWindow(&m_showDemoWindow);
        ImPlot::ShowDemoWindow();
    }

    if (m_layoutResetFrames > 0) m_layoutResetFrames--;

    // Animate 3D shape if enabled AND 3D shape is the active pattern source
    if (m_shape3D.animate && m_3dShapeActive) {
        m_shape3D.rotationX += m_shape3D.rotationSpeedX;
        m_shape3D.rotationY += m_shape3D.rotationSpeedY;
        m_shape3D.rotationZ += m_shape3D.rotationSpeedZ;
        generate3DShapePattern(app);
    }
}

//==============================================================================
// File menu actions
//==============================================================================

namespace {

// Write an RGBA buffer out as a PNG. Isolated so the export path has one place
// to change if the encoder ever does.
bool writePng(const std::string& path, const unsigned char* rgba, int w, int h) {
    return stbi_write_png(path.c_str(), w, h, 4, rgba, w * 4) != 0;
}

// Lowercase extension of a path, including the dot (".txt"), or "" if none.
std::string fileExtension(const std::string& path) {
    size_t slash = path.find_last_of("/\\");
    size_t dot = path.find_last_of('.');
    if (dot == std::string::npos) return "";
    if (slash != std::string::npos && dot < slash) return "";

    std::string ext = path.substr(dot);
    for (char& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return ext;
}

// Append `ext` if the path has no extension of its own. Some GTK/Qt save
// dialogs return a bare name even when a filter is selected.
std::string ensureExtension(const std::string& path, const std::string& ext) {
    return fileExtension(path).empty() ? path + ext : path;
}

} // namespace

void UIManager::setStatus(const std::string& message, bool isError) {
    m_statusMessage = message;
    m_statusIsError = isError;
    m_statusTime = ImGui::GetTime();
}

bool UIManager::loadPatternFile(App& app, const std::string& path) {
    std::string ext = fileExtension(path);

    Pattern loaded;
    bool ok = false;
    if (ext == ".osc") {
        ok = FileLoader::loadBinaryFile(path, loaded);
    } else if (ext == ".m") {
        ok = FileLoader::loadMatlabFile(path, loaded);
    } else {
        ok = FileLoader::loadTextFile(path, loaded);
    }

    if (!ok || loaded.empty()) {
        setStatus("Failed to load " + path, true);
        // A file that will not open should not linger in the recent list.
        m_recentFiles.remove(path);
        saveRecentFiles();
        return false;
    }

    // A loaded pattern becomes the active source; stop the 3D animation and
    // the live generator sliders from overwriting it on the next frame.
    claimPatternSource();

    Pattern& pattern = app.getPattern();
    pattern = loaded;
    app.getAudioEngine().setPattern(pattern);

    m_recentFiles.add(path);
    saveRecentFiles();

    // setPattern() silently clamps to MAX_PATTERN_SIZE, so a long capture would
    // otherwise appear to load fine and then play back only its opening slice.
    if (pattern.size() > AudioEngine::MAX_PATTERN_SIZE) {
        char msg[160];
        snprintf(msg, sizeof(msg),
                 "Loaded %zu points, but only the first %zu will play",
                 pattern.size(), AudioEngine::MAX_PATTERN_SIZE);
        setStatus(msg, true);
    } else {
        setStatus("Loaded " + std::to_string(pattern.size()) + " points");
    }
    return true;
}

void UIManager::doLoadPattern(App& app) {
    auto selection = pfd::open_file(
        "Load Pattern",
        "",
        { "Pattern Files", "*.txt *.csv *.dat *.osc *.m",
          "Text (X,Y per line)", "*.txt *.csv *.dat",
          "Oscilloplot Binary", "*.osc",
          "MATLAB Script", "*.m",
          "All Files", "*" }
    ).result();

    if (selection.empty()) return;
    loadPatternFile(app, selection[0]);
}

void UIManager::doSavePattern(App& app) {
    const Pattern& pattern = app.getPattern();
    if (pattern.empty()) {
        setStatus("Nothing to save - pattern is empty", true);
        return;
    }

    std::string path = pfd::save_file(
        "Save Pattern",
        "pattern.txt",
        { "Text (X,Y per line)", "*.txt *.csv *.dat",
          "Oscilloplot Binary", "*.osc",
          "All Files", "*" }
    ).result();

    if (path.empty()) return;

    // Decide the format from the extension the user actually typed BEFORE
    // defaulting: some pickers (GTK/Zenity) return a bare name even when the
    // .osc filter is selected, and appending .txt first would silently
    // override that choice.
    std::string ext = fileExtension(path);
    bool binary = (ext == ".osc");
    if (ext.empty()) path += ".txt";

    bool ok = binary
        ? FileSaver::saveBinaryFile(path, pattern)
        : FileSaver::saveTextFile(path, pattern);

    if (ok) {
        setStatus("Saved " + std::to_string(pattern.size()) + " points");
    } else {
        setStatus("Failed to save " + path, true);
    }
}

void UIManager::doExportWav(App& app) {
    const Pattern& pattern = app.getPattern();
    if (pattern.empty()) {
        setStatus("Nothing to export - pattern is empty", true);
        return;
    }

    std::string path = pfd::save_file(
        "Export WAV",
        "oscilloplot.wav",
        { "WAV Audio", "*.wav", "All Files", "*" }
    ).result();

    if (path.empty()) return;
    path = ensureExtension(path, ".wav");

    // Match playback: the pattern is clocked at base rate x multiplier.
    int sampleRate = app.getSampleRate() * app.getPlaybackMultiplier();
    if (sampleRate < 1) sampleRate = 44100;

    // Length comes from the Duration control. Playback loops the pattern
    // indefinitely, so the number of repeats needed is derived from how many
    // seconds of audio the user asked for.
    double wanted = static_cast<double>(app.getDuration()) * sampleRate;
    double rawRepeats = wanted / static_cast<double>(pattern.size()) + 0.5;

    // exportPattern() builds the whole interleaved buffer in memory before
    // writing. A long duration against a short pattern at 192 kHz can ask for
    // hundreds of MB, so cap the frame count and say so rather than attempting
    // an allocation that may fail outright.
    constexpr size_t MAX_EXPORT_FRAMES = 64u * 1024u * 1024u;  // 512 MB stereo float
    size_t maxRepeats = MAX_EXPORT_FRAMES / pattern.size();
    if (maxRepeats < 1) maxRepeats = 1;

    bool clamped = false;
    if (rawRepeats > static_cast<double>(maxRepeats)) {
        rawRepeats = static_cast<double>(maxRepeats);
        clamped = true;
    }

    int repeats = static_cast<int>(rawRepeats);
    if (repeats < 1) repeats = 1;

    if (WavExport::exportPattern(path, pattern, sampleRate, repeats)) {
        double seconds = static_cast<double>(pattern.size()) * repeats / sampleRate;
        char msg[192];
        snprintf(msg, sizeof(msg), "Exported %.2fs at %d Hz (%d loops)%s",
                 seconds, sampleRate, repeats,
                 clamped ? " - truncated to fit memory" : "");
        setStatus(msg, clamped);
    } else {
        setStatus("Failed to export " + path, true);
    }
}

//==============================================================================
// Visual style
//
// One coherent dark theme instead of stock ImGui: near-black surfaces so the
// phosphor scope is the brightest thing on screen, a single phosphor-green
// accent for anything active or primary, consistent 4px rounding, and enough
// frame padding to give controls comfortable hit targets.
//==============================================================================

void UIManager::applyStyle() {
    ImGuiStyle& style = ImGui::GetStyle();

    // Geometry
    style.WindowPadding    = ImVec2(12, 10);
    style.FramePadding     = ImVec2(8, 5);
    style.CellPadding      = ImVec2(6, 4);
    style.ItemSpacing      = ImVec2(8, 6);
    style.ItemInnerSpacing = ImVec2(6, 4);
    style.IndentSpacing    = 16.0f;
    style.ScrollbarSize    = 12.0f;
    style.GrabMinSize      = 12.0f;

    style.WindowRounding    = 6.0f;
    style.ChildRounding     = 4.0f;
    style.FrameRounding     = 4.0f;
    style.PopupRounding     = 4.0f;
    style.ScrollbarRounding = 6.0f;
    style.GrabRounding      = 4.0f;
    style.TabRounding       = 4.0f;

    style.WindowBorderSize = 1.0f;
    style.FrameBorderSize  = 0.0f;
    style.PopupBorderSize  = 1.0f;

    style.WindowTitleAlign = ImVec2(0.0f, 0.5f);
    style.SeparatorTextBorderSize = 2.0f;

    // Palette: charcoal surfaces + phosphor green accent (P31: 89,255,64)
    const ImVec4 bg0     (0.075f, 0.080f, 0.090f, 1.00f);  // window
    const ImVec4 bg1     (0.105f, 0.110f, 0.125f, 1.00f);  // child/frame
    const ImVec4 bg2     (0.150f, 0.160f, 0.180f, 1.00f);  // hovered frame
    const ImVec4 bg3     (0.195f, 0.210f, 0.235f, 1.00f);  // active frame
    const ImVec4 accent  (0.286f, 0.780f, 0.240f, 1.00f);  // phosphor green
    const ImVec4 accentHi(0.360f, 0.900f, 0.310f, 1.00f);
    const ImVec4 accentLo(0.220f, 0.560f, 0.190f, 1.00f);
    const ImVec4 text    (0.920f, 0.930f, 0.920f, 1.00f);
    const ImVec4 textDim (0.520f, 0.545f, 0.530f, 1.00f);
    const ImVec4 border  (0.220f, 0.235f, 0.250f, 0.60f);

    ImVec4* c = style.Colors;
    c[ImGuiCol_Text]                 = text;
    c[ImGuiCol_TextDisabled]         = textDim;
    c[ImGuiCol_WindowBg]             = bg0;
    c[ImGuiCol_ChildBg]              = ImVec4(0, 0, 0, 0.12f);
    c[ImGuiCol_PopupBg]              = ImVec4(0.09f, 0.095f, 0.105f, 0.98f);
    c[ImGuiCol_Border]               = border;
    c[ImGuiCol_BorderShadow]         = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_FrameBg]              = bg1;
    c[ImGuiCol_FrameBgHovered]       = bg2;
    c[ImGuiCol_FrameBgActive]        = bg3;
    c[ImGuiCol_TitleBg]              = ImVec4(0.055f, 0.058f, 0.065f, 1.0f);
    c[ImGuiCol_TitleBgActive]        = ImVec4(0.10f, 0.115f, 0.11f, 1.0f);
    c[ImGuiCol_TitleBgCollapsed]     = ImVec4(0.055f, 0.058f, 0.065f, 0.8f);
    c[ImGuiCol_MenuBarBg]            = ImVec4(0.090f, 0.095f, 0.105f, 1.0f);
    c[ImGuiCol_ScrollbarBg]          = ImVec4(0, 0, 0, 0.15f);
    c[ImGuiCol_ScrollbarGrab]        = bg2;
    c[ImGuiCol_ScrollbarGrabHovered] = bg3;
    c[ImGuiCol_ScrollbarGrabActive]  = accentLo;
    c[ImGuiCol_CheckMark]            = accent;
    c[ImGuiCol_SliderGrab]           = accent;
    c[ImGuiCol_SliderGrabActive]     = accentHi;
    c[ImGuiCol_Button]               = bg2;
    c[ImGuiCol_ButtonHovered]        = bg3;
    c[ImGuiCol_ButtonActive]         = accentLo;
    c[ImGuiCol_Header]               = ImVec4(0.286f, 0.780f, 0.240f, 0.18f);
    c[ImGuiCol_HeaderHovered]        = ImVec4(0.286f, 0.780f, 0.240f, 0.28f);
    c[ImGuiCol_HeaderActive]         = ImVec4(0.286f, 0.780f, 0.240f, 0.38f);
    c[ImGuiCol_Separator]            = border;
    c[ImGuiCol_SeparatorHovered]     = accentLo;
    c[ImGuiCol_SeparatorActive]      = accent;
    c[ImGuiCol_ResizeGrip]           = ImVec4(0.286f, 0.780f, 0.240f, 0.15f);
    c[ImGuiCol_ResizeGripHovered]    = ImVec4(0.286f, 0.780f, 0.240f, 0.45f);
    c[ImGuiCol_ResizeGripActive]     = accent;
    c[ImGuiCol_Tab]                  = bg1;
    c[ImGuiCol_TabHovered]           = ImVec4(0.286f, 0.780f, 0.240f, 0.30f);
    c[ImGuiCol_TabActive]            = ImVec4(0.286f, 0.780f, 0.240f, 0.22f);
    c[ImGuiCol_TabUnfocused]         = bg1;
    c[ImGuiCol_TabUnfocusedActive]   = bg2;
    c[ImGuiCol_PlotLines]            = accent;
    c[ImGuiCol_PlotLinesHovered]     = accentHi;
    c[ImGuiCol_PlotHistogram]        = accent;
    c[ImGuiCol_PlotHistogramHovered] = accentHi;
    c[ImGuiCol_TextSelectedBg]       = ImVec4(0.286f, 0.780f, 0.240f, 0.30f);
    c[ImGuiCol_DragDropTarget]       = accentHi;
    c[ImGuiCol_NavHighlight]         = accent;
    c[ImGuiCol_ModalWindowDimBg]     = ImVec4(0, 0, 0, 0.55f);
}

//==============================================================================
// Default layout
//
// Windows are placed as fractions of the viewport work area instead of pixel
// positions tuned for one monitor: controls rail on the left, the scope takes
// the center, effects and generators stack on the right. View > Reset Layout
// re-applies this to every window that is currently open.
//==============================================================================

void UIManager::placeWindow(float x, float y, float w, float h) {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGuiCond cond = (m_layoutResetFrames > 0) ? ImGuiCond_Always : ImGuiCond_FirstUseEver;

    // WorkPos only accounts for the main menu bar from the second frame on
    // (side bars adjust the work area for the NEXT frame). On the first frame
    // WorkPos.y is still 0, and a FirstUseEver position taken then would sit
    // on top of - and permanently hide - the menu bar. Clamp the top edge to
    // at least one frame height (exactly the menu bar's height).
    float top = std::max(vp->WorkPos.y, vp->Pos.y + ImGui::GetFrameHeight());
    float availW = vp->WorkSize.x;
    float availH = vp->Pos.y + vp->Size.y - top;

    ImGui::SetNextWindowPos(
        ImVec2(vp->WorkPos.x + x * availW, top + y * availH), cond);
    ImGui::SetNextWindowSize(
        ImVec2(w * availW, h * availH), cond);
}

//==============================================================================
// Sequencer - chains patterns over time
//
// The audio engine loops a single pattern forever; the sequencer drives it
// from the UI thread by watching the engine's cycle counter and swapping in
// the next step's pattern when the current step has run its cycles. This is
// the same UI-thread setPattern() path the 3D animation already uses, so no
// new realtime machinery is needed.
//==============================================================================

void UIManager::sequencerApplyStep(App& app, int index) {
    if (index < 0 || index >= static_cast<int>(m_sequencer.seq.steps.size())) return;
    m_sequencer.currentStep = index;
    app.getPattern() = m_sequencer.seq.steps[index].pattern;
    app.getAudioEngine().setPattern(app.getPattern());
    m_sequencer.cyclesAtStepStart = app.getAudioEngine().getPatternCycleCount();
}

void UIManager::sequencerStart(App& app) {
    if (m_sequencer.seq.steps.empty()) {
        setStatus("Sequence is empty - capture a pattern first", true);
        return;
    }
    // The sequence owns the pattern now; stop other sources overwriting it.
    claimPatternSource();

    // Step timing is driven by the engine's pattern-cycle counter, which only
    // advances while audio is streaming. With no output device the sequence
    // would flash step 0 for one frame and silently stop, so say why.
    if (!app.isAudioAvailable()) {
        setStatus("Sequencer needs an audio output device", true);
        return;
    }

    m_sequencer.playing = true;
    if (!app.isPlaying()) {
        app.setPlaying(true);   // Starts the stream and resets the cycle counter
    }
    sequencerApplyStep(app, 0);
}

void UIManager::sequencerStop(App& app) {
    m_sequencer.playing = false;
    m_sequencer.currentStep = -1;
    if (app.isPlaying()) {
        app.setPlaying(false);
    }
}

void UIManager::updateSequencer(App& app) {
    if (!m_sequencer.playing) return;

    // The user stopped playback out from under the sequence (Space/Stop):
    // treat that as stopping the sequence too.
    if (!app.isPlaying()) {
        m_sequencer.playing = false;
        m_sequencer.currentStep = -1;
        return;
    }
    if (m_sequencer.currentStep < 0 ||
        m_sequencer.currentStep >= static_cast<int>(m_sequencer.seq.steps.size())) {
        sequencerStop(app);
        return;
    }

    const SequenceStep& step = m_sequencer.seq.steps[m_sequencer.currentStep];
    const uint32_t total = static_cast<uint32_t>(std::max(1, step.cycles));
    const uint32_t done = app.getAudioEngine().getPatternCycleCount() -
                          m_sequencer.cyclesAtStepStart;

    // Index of whatever plays after this step (may wrap, may not exist).
    int next = m_sequencer.currentStep + 1;
    bool haveNext = true;
    if (next >= static_cast<int>(m_sequencer.seq.steps.size())) {
        if (m_sequencer.seq.loop) next = 0;
        else haveNext = false;
    }

    if (done >= total) {
        if (!haveNext) {
            sequencerStop(app);
            setStatus("Sequence finished");
            return;
        }
        sequencerApplyStep(app, next);
        return;
    }

    // Crossfade: over the final `crossfade` fraction of the step, blend this
    // pattern into the next one so steps flow instead of hard-switching.
    const float cf = m_sequencer.seq.crossfade;
    if (cf > 0.001f && haveNext && next != m_sequencer.currentStep) {
        const float fadeCycles = cf * static_cast<float>(total);
        const float remaining  = static_cast<float>(total - done);
        if (fadeCycles >= 1.0f && remaining <= fadeCycles) {
            float t = 1.0f - (remaining / fadeCycles);   // 0 -> 1 across the fade
            t = std::clamp(t, 0.0f, 1.0f);

            blendPatterns(step.pattern, m_sequencer.seq.steps[next].pattern,
                          t, m_sequencer.blendScratch);
            app.getPattern() = m_sequencer.blendScratch;
            // Continuous swap: resetting the position here would stall the
            // cycle counter this very loop depends on.
            app.getAudioEngine().setPatternContinuous(app.getPattern());
        }
    }
}

void UIManager::sequencerSave() {
    if (m_sequencer.seq.empty()) {
        setStatus("Sequence is empty - nothing to save", true);
        return;
    }
    std::string path = pfd::save_file(
        "Save Sequence", "sequence.oseq",
        { "Oscilloplot Sequence", "*.oseq", "All Files", "*" }
    ).result();
    if (path.empty()) return;
    path = ensureExtension(path, ".oseq");

    std::string error;
    if (saveSequence(path, m_sequencer.seq, error)) {
        m_recentFiles.add(path);
        saveRecentFiles();
        setStatus("Saved sequence (" +
                  std::to_string(m_sequencer.seq.size()) + " steps)");
    } else {
        setStatus("Failed to save sequence: " + error, true);
    }
}

void UIManager::sequencerLoad() {
    auto selection = pfd::open_file(
        "Load Sequence", "",
        { "Oscilloplot Sequence", "*.oseq", "All Files", "*" }
    ).result();
    if (selection.empty()) return;
    loadSequenceFile(selection[0]);
}

void UIManager::loadSequenceFile(const std::string& path) {
    Sequence loaded;
    std::string error;
    if (!loadSequence(path, loaded, error)) {
        setStatus("Failed to load sequence: " + error, true);
        m_recentFiles.remove(path);
        saveRecentFiles();
        return;
    }

    m_sequencer.seq = std::move(loaded);
    m_sequencer.playing = false;
    m_sequencer.currentStep = -1;
    m_sequencerUndo.clear();

    m_recentFiles.add(path);
    saveRecentFiles();
    setStatus("Loaded sequence (" +
              std::to_string(m_sequencer.seq.size()) + " steps)");
}

void UIManager::sequencerExportWav(App& app) {
    if (m_sequencer.seq.steps.empty()) {
        setStatus("Sequence is empty - nothing to export", true);
        return;
    }
    std::string path = pfd::save_file(
        "Export Sequence WAV", "sequence.wav",
        { "WAV Audio", "*.wav", "All Files", "*" }
    ).result();
    if (path.empty()) return;
    path = ensureExtension(path, ".wav");

    int sampleRate = app.getSampleRate() * app.getPlaybackMultiplier();
    if (sampleRate < 1) sampleRate = 44100;

    constexpr size_t MAX_EXPORT_FRAMES = 64u * 1024u * 1024u;  // 512 MB stereo float
    std::vector<float> buffer;
    bool truncated = false;

    for (const auto& step : m_sequencer.seq.steps) {
        if (step.pattern.empty()) continue;
        int cycles = std::max(1, step.cycles);
        for (int c = 0; c < cycles && !truncated; ++c) {
            if (buffer.size() / 2 + step.pattern.size() > MAX_EXPORT_FRAMES) {
                truncated = true;
                break;
            }
            for (size_t i = 0; i < step.pattern.size(); ++i) {
                buffer.push_back(step.pattern.x[i]);
                buffer.push_back(step.pattern.y[i]);
            }
        }
        if (truncated) break;
    }

    if (buffer.empty()) {
        setStatus("Sequence has no points to export", true);
        return;
    }

    if (WavExport::exportToWav(path, buffer, sampleRate)) {
        double seconds = static_cast<double>(buffer.size() / 2) / sampleRate;
        char msg[192];
        snprintf(msg, sizeof(msg), "Exported sequence: %.2fs at %d Hz%s",
                 seconds, sampleRate, truncated ? " - truncated to fit memory" : "");
        setStatus(msg, truncated);
    } else {
        setStatus("Failed to export " + path, true);
    }
}

void UIManager::renderSequencer(App& app) {
    placeWindow(0.33f, 0.08f, 0.32f, 0.65f);
    ImGui::Begin("Sequencer", &m_showSequencer);

    // Transport
    const bool seqPlaying = m_sequencer.playing;
    if (seqPlaying) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.2f, 0.2f, 1.0f));
        if (ImGui::Button("Stop Sequence", ImVec2(-1, 34))) sequencerStop(app);
        ImGui::PopStyleColor();
    } else {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.7f, 0.3f, 1.0f));
        if (ImGui::Button("Play Sequence", ImVec2(-1, 34))) sequencerStart(app);
        ImGui::PopStyleColor();
    }
    ImGui::Checkbox("Loop", &m_sequencer.seq.loop);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(140);
    ImGui::SliderFloat("Crossfade", &m_sequencer.seq.crossfade, 0.0f, 0.5f, "%.2f");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Fraction of each step spent blending into the next.\n0 = hard switch.");
    }

    ImGui::Separator();

    // Undo / Redo for structural edits (capture, reorder, delete)
    {
        float w = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
        if (!m_sequencerUndo.canUndo()) ImGui::BeginDisabled();
        if (ImGui::Button("Undo##seq", ImVec2(w, 0))) {
            if (const Sequence* prev = m_sequencerUndo.undo(m_sequencer.seq)) {
                m_sequencer.seq = *prev;
                if (m_sequencer.currentStep >=
                    static_cast<int>(m_sequencer.seq.steps.size())) {
                    sequencerStop(app);
                }
            }
        }
        if (!m_sequencerUndo.canUndo()) ImGui::EndDisabled();
        ImGui::SameLine();
        if (!m_sequencerUndo.canRedo()) ImGui::BeginDisabled();
        if (ImGui::Button("Redo##seq", ImVec2(w, 0))) {
            if (const Sequence* next = m_sequencerUndo.redo(m_sequencer.seq)) {
                m_sequencer.seq = *next;
                if (m_sequencer.currentStep >=
                    static_cast<int>(m_sequencer.seq.steps.size())) {
                    sequencerStop(app);
                }
            }
        }
        if (!m_sequencerUndo.canRedo()) ImGui::EndDisabled();
    }

    ImGui::Separator();

    // Capture the pattern currently on screen as a new step
    if (ImGui::Button("+ Capture Current Pattern", ImVec2(-1, 0))) {
        const Pattern& current = app.getPattern();
        if (current.empty()) {
            setStatus("Current pattern is empty", true);
        } else {
            m_sequencerUndo.push(m_sequencer.seq);
            SequenceStep step;
            step.pattern = current;
            snprintf(step.name, sizeof(step.name), "Step %d",
                     static_cast<int>(m_sequencer.seq.steps.size()) + 1);
            m_sequencer.seq.steps.push_back(std::move(step));
        }
    }

    // Step list
    int actualRate = app.getSampleRate() * app.getPlaybackMultiplier();
    int removeIdx = -1, moveUpIdx = -1, moveDownIdx = -1;

    ImGui::BeginChild("StepList", ImVec2(0, -70), true);
    for (int i = 0; i < static_cast<int>(m_sequencer.seq.steps.size()); ++i) {
        SequenceStep& step = m_sequencer.seq.steps[i];
        ImGui::PushID(i);

        bool isCurrent = seqPlaying && (i == m_sequencer.currentStep);
        if (isCurrent) {
            ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.4f, 1.0f), ">");
        } else {
            ImGui::TextDisabled("%d", i + 1);
        }
        ImGui::SameLine();

        ImGui::SetNextItemWidth(110);
        ImGui::InputText("##name", step.name, sizeof(step.name));
        ImGui::SameLine();

        ImGui::SetNextItemWidth(80);
        ImGui::DragInt("##cycles", &step.cycles, 1.0f, 1, 100000, "%d cyc");
        if (ImGui::IsItemHovered() && actualRate > 0 && !step.pattern.empty()) {
            float secs = static_cast<float>(step.cycles) *
                         static_cast<float>(step.pattern.size()) / static_cast<float>(actualRate);
            ImGui::SetTooltip("%zu points, ~%.2fs", step.pattern.size(), secs);
        }
        ImGui::SameLine();

        // Preview loads the step's pattern without starting the sequence
        if (ImGui::SmallButton("View")) {
            if (!seqPlaying) {
                claimPatternSource();
                app.getPattern() = step.pattern;
                app.getAudioEngine().setPattern(app.getPattern());
            }
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("^") && i > 0) moveUpIdx = i;
        ImGui::SameLine();
        if (ImGui::SmallButton("v") && i + 1 < static_cast<int>(m_sequencer.seq.steps.size())) moveDownIdx = i;
        ImGui::SameLine();
        if (ImGui::SmallButton("X")) removeIdx = i;

        ImGui::PopID();
    }
    if (m_sequencer.seq.steps.empty()) {
        ImGui::TextDisabled("No steps yet.");
        ImGui::TextWrapped("Make a pattern with any generator, then press "
                           "'+ Capture Current Pattern' to add it here.");
    }
    ImGui::EndChild();

    // Structural edits happen outside the loop; adjust indices so a playing
    // sequence keeps pointing at the same step.
    if (moveUpIdx > 0 || moveDownIdx >= 0 || removeIdx >= 0) {
        m_sequencerUndo.push(m_sequencer.seq);
    }

    if (moveUpIdx > 0) {
        std::swap(m_sequencer.seq.steps[moveUpIdx], m_sequencer.seq.steps[moveUpIdx - 1]);
        if (m_sequencer.currentStep == moveUpIdx) m_sequencer.currentStep--;
        else if (m_sequencer.currentStep == moveUpIdx - 1) m_sequencer.currentStep++;
    }
    if (moveDownIdx >= 0) {
        std::swap(m_sequencer.seq.steps[moveDownIdx], m_sequencer.seq.steps[moveDownIdx + 1]);
        if (m_sequencer.currentStep == moveDownIdx) m_sequencer.currentStep++;
        else if (m_sequencer.currentStep == moveDownIdx + 1) m_sequencer.currentStep--;
    }
    if (removeIdx >= 0) {
        m_sequencer.seq.steps.erase(m_sequencer.seq.steps.begin() + removeIdx);
        if (m_sequencer.seq.steps.empty()) {
            sequencerStop(app);
        } else if (m_sequencer.currentStep > removeIdx) {
            m_sequencer.currentStep--;
        } else if (m_sequencer.currentStep == removeIdx) {
            // The playing step vanished; jump to what now sits at its index
            int next = std::min(removeIdx,
                                static_cast<int>(m_sequencer.seq.steps.size()) - 1);
            if (m_sequencer.playing) sequencerApplyStep(app, next);
        }
    }

    // File row
    float w = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x * 2) / 3.0f;
    if (ImGui::Button("Save...", ImVec2(w, 0))) sequencerSave();
    ImGui::SameLine();
    if (ImGui::Button("Load...", ImVec2(w, 0))) sequencerLoad();
    ImGui::SameLine();
    if (ImGui::Button("Export WAV...", ImVec2(w, 0))) sequencerExportWav(app);

    ImGui::End();
}

//==============================================================================
// Presets, recent files and image export
//==============================================================================

std::string UIManager::recentFilesPath() {
    // Sits beside imgui.ini, in whatever directory the app runs from.
    return "oscilloplot_recent.txt";
}

void UIManager::loadRecentFiles() {
    m_recentFiles.load(recentFilesPath());
    // Silently drop anything that has since been deleted or moved.
    m_recentFiles.pruneMissing();
}

void UIManager::saveRecentFiles() const {
    m_recentFiles.save(recentFilesPath());
}

void UIManager::doSavePreset(App& app) {
    const Pattern& pattern = app.getPattern();
    if (pattern.empty()) {
        setStatus("Nothing to save - pattern is empty", true);
        return;
    }

    std::string path = pfd::save_file(
        "Save Preset", "preset.opreset",
        { "Oscilloplot Preset", "*.opreset", "All Files", "*" }
    ).result();
    if (path.empty()) return;
    path = ensureExtension(path, ".opreset");

    Preset preset;
    preset.pattern = pattern;
    preset.name = RecentFiles::displayName(path);
    // Strip the extension so the stored name reads as a title.
    size_t dot = preset.name.find_last_of('.');
    if (dot != std::string::npos) preset.name = preset.name.substr(0, dot);

    captureEffects(app.getEffects(), preset);

    std::string error;
    if (savePreset(path, preset, error)) {
        m_recentFiles.add(path);
        saveRecentFiles();
        setStatus("Saved preset with " +
                  std::to_string(preset.effectValues.size()) + " effect settings");
    } else {
        setStatus("Failed to save preset: " + error, true);
    }
}

void UIManager::loadPresetFile(App& app, const std::string& path) {
    Preset preset;
    std::string error;
    if (!loadPreset(path, preset, error)) {
        setStatus("Failed to load preset: " + error, true);
        m_recentFiles.remove(path);
        saveRecentFiles();
        return;
    }

    claimPatternSource();

    if (!preset.pattern.empty()) {
        app.getPattern() = preset.pattern;
        app.getAudioEngine().setPattern(app.getPattern());
    }
    applyEffects(preset, app.getEffects());

    m_recentFiles.add(path);
    saveRecentFiles();
    setStatus("Loaded preset '" + preset.name + "'");
}

void UIManager::doLoadPreset(App& app) {
    auto selection = pfd::open_file(
        "Load Preset", "",
        { "Oscilloplot Preset", "*.opreset", "All Files", "*" }
    ).result();
    if (selection.empty()) return;
    loadPresetFile(app, selection[0]);
}

//------------------------------------------------------------------------------
// Export the scope view as a PNG.
//
// The phosphor display is drawn with ImGui draw-list calls rather than into a
// texture, so there is no framebuffer to grab. Instead the pattern is rendered
// again here into an RGBA buffer using the same beam model - velocity-based
// brightness plus a soft glow - which also means the export is not limited to
// the on-screen resolution.
//------------------------------------------------------------------------------
void UIManager::doExportImage(App& app) {
    const Pattern& pattern = app.getPattern();
    if (pattern.size() < 2) {
        setStatus("Nothing to export - pattern is empty", true);
        return;
    }

    std::string path = pfd::save_file(
        "Export Image", "oscilloplot.png",
        { "PNG Image", "*.png", "All Files", "*" }
    ).result();
    if (path.empty()) return;
    path = ensureExtension(path, ".png");

    const int size = m_exportImageSize;
    std::vector<float> energy(static_cast<size_t>(size) * size, 0.0f);

    // Accumulate beam energy along the trace, brighter where the beam moves
    // slowly - the same relationship the live display uses.
    const float half = size * 0.5f;
    const float scale = half * 0.92f;      // leave a small margin
    const float core = std::max(0.7f, size / 900.0f * m_phosphor.beamWidth);
    const float halo = core * 3.0f;
    const int reach = static_cast<int>(halo * 2.5f) + 1;

    for (size_t i = 1; i < pattern.size(); ++i) {
        const float x0 = half + pattern.x[i - 1] * scale;
        const float y0 = half - pattern.y[i - 1] * scale;
        const float x1 = half + pattern.x[i] * scale;
        const float y1 = half - pattern.y[i] * scale;

        const float dx = x1 - x0, dy = y1 - y0;
        const float dist = std::sqrt(dx * dx + dy * dy);

        // Slow segments deposit more energy per unit length.
        const float speedTerm = 1.0f / (1.0f + dist * 0.25f);
        const float bright = m_phosphor.minBrightness +
            (m_phosphor.maxBrightness - m_phosphor.minBrightness) *
            (m_phosphor.velocityEffect * speedTerm + (1.0f - m_phosphor.velocityEffect));

        // Walk the segment so fast moves still leave a continuous trace.
        const int steps = std::max(1, static_cast<int>(dist));
        for (int s = 0; s <= steps; ++s) {
            const float t = static_cast<float>(s) / static_cast<float>(steps);
            const float px = x0 + dx * t;
            const float py = y0 + dy * t;

            const int cx = static_cast<int>(px);
            const int cy = static_cast<int>(py);
            for (int oy = -reach; oy <= reach; ++oy) {
                const int Y = cy + oy;
                if (Y < 0 || Y >= size) continue;
                for (int ox = -reach; ox <= reach; ++ox) {
                    const int X = cx + ox;
                    if (X < 0 || X >= size) continue;
                    const float ddx = X - px, ddy = Y - py;
                    const float d2 = ddx * ddx + ddy * ddy;
                    energy[static_cast<size_t>(Y) * size + X] +=
                        bright * (std::exp(-d2 / (2.0f * core * core)) +
                                  m_phosphor.glowIntensity *
                                  std::exp(-d2 / (2.0f * halo * halo)));
                }
            }
        }
    }

    // Normalise against a high percentile so a few very bright pixels do not
    // wash the whole trace out.
    std::vector<float> sorted;
    sorted.reserve(energy.size() / 8 + 1);
    for (size_t i = 0; i < energy.size(); i += 8) {
        if (energy[i] > 0.0f) sorted.push_back(energy[i]);
    }
    float ref = 1.0f;
    if (!sorted.empty()) {
        size_t idx = static_cast<size_t>(sorted.size() * 0.98);
        if (idx >= sorted.size()) idx = sorted.size() - 1;
        std::nth_element(sorted.begin(), sorted.begin() + idx, sorted.end());
        ref = std::max(1e-6f, sorted[idx]);
    }

    std::vector<unsigned char> rgba(static_cast<size_t>(size) * size * 4);
    for (size_t i = 0; i < energy.size(); ++i) {
        float e = std::min(1.0f, energy[i] / ref);
        e = std::pow(e, 1.0f / std::max(0.1f, m_phosphor.decayExponent * 0.7f));

        // Dark CRT ground plus the phosphor colour scaled by beam energy.
        const float r = 0.02f + e * m_phosphor.colorR;
        const float g = 0.03f + e * m_phosphor.colorG;
        const float b = 0.02f + e * m_phosphor.colorB;

        rgba[i * 4 + 0] = static_cast<unsigned char>(std::min(1.0f, r) * 255.0f);
        rgba[i * 4 + 1] = static_cast<unsigned char>(std::min(1.0f, g) * 255.0f);
        rgba[i * 4 + 2] = static_cast<unsigned char>(std::min(1.0f, b) * 255.0f);
        rgba[i * 4 + 3] = 255;
    }

    if (writePng(path, rgba.data(), size, size)) {
        setStatus("Exported " + std::to_string(size) + "x" +
                  std::to_string(size) + " image");
    } else {
        setStatus("Failed to write " + path, true);
    }
}

void UIManager::renderMenuBar(App& app) {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Load Pattern...", "Ctrl+O")) {
                doLoadPattern(app);
            }
            if (ImGui::MenuItem("Save Pattern...", "Ctrl+S")) {
                doSavePattern(app);
            }

            // Recent files: pattern, preset and sequence files alike, opened
            // by whichever loader matches the extension.
            if (ImGui::BeginMenu("Open Recent", !m_recentFiles.empty())) {
                std::string chosen;
                for (const auto& entry : m_recentFiles.entries()) {
                    std::string label = RecentFiles::displayName(entry);
                    if (ImGui::MenuItem(label.c_str())) chosen = entry;
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", entry.c_str());
                }
                ImGui::Separator();
                if (ImGui::MenuItem("Clear List")) {
                    m_recentFiles.clear();
                    saveRecentFiles();
                }
                ImGui::EndMenu();

                // Acted on outside the loop so the menu is not mutated mid-iteration.
                if (!chosen.empty()) {
                    std::string ext = fileExtension(chosen);
                    if (ext == ".opreset")   loadPresetFile(app, chosen);
                    else if (ext == ".oseq") loadSequenceFile(chosen);
                    else                     loadPatternFile(app, chosen);
                }
            }

            ImGui::Separator();
            if (ImGui::MenuItem("Save Preset...")) {
                doSavePreset(app);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Saves the pattern together with every effect setting");
            }
            if (ImGui::MenuItem("Load Preset...")) {
                doLoadPreset(app);
            }

            ImGui::Separator();
            if (ImGui::MenuItem("Export WAV...", "Ctrl+E")) {
                doExportWav(app);
            }
            if (ImGui::MenuItem("Export Image...", "Ctrl+I")) {
                doExportImage(app);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Render the scope view to a PNG");
            }

            ImGui::Separator();
            if (ImGui::MenuItem("Exit", "Alt+F4")) {
                app.requestExit();
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
            ImGui::MenuItem("Image Vectorizer", nullptr, &m_showImageVectorizer);
            ImGui::MenuItem("Sequencer", nullptr, &m_showSequencer);
            ImGui::Separator();
            ImGui::MenuItem("Display Settings", nullptr, &m_showDisplaySettings);
            ImGui::Separator();
            if (ImGui::MenuItem("Reset Layout")) {
                m_layoutResetFrames = 2;   // Re-place every open window
            }
            ImGui::Separator();
            ImGui::MenuItem("ImGui Demo", nullptr, &m_showDemoWindow);
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Help")) {
            if (ImGui::MenuItem("About Oscilloplot")) {
                m_showAbout = true;
            }
            ImGui::EndMenu();
        }

        // Persistent warning when the machine has no usable audio output
        if (!app.isAudioAvailable()) {
            ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "  [No audio device - preview only]");
        }

        // Transient status readout, right-aligned; fades out after a few seconds
        if (!m_statusMessage.empty()) {
            constexpr double STATUS_HOLD = 4.0;   // Fully opaque for this long
            constexpr double STATUS_FADE = 2.0;   // Then fades over this long

            double age = ImGui::GetTime() - m_statusTime;
            if (age > STATUS_HOLD + STATUS_FADE) {
                m_statusMessage.clear();
            } else {
                float alpha = (age <= STATUS_HOLD)
                    ? 1.0f
                    : 1.0f - static_cast<float>((age - STATUS_HOLD) / STATUS_FADE);

                float textWidth = ImGui::CalcTextSize(m_statusMessage.c_str()).x;
                ImGui::SameLine(ImGui::GetWindowWidth() - textWidth -
                                ImGui::GetStyle().ItemSpacing.x * 2.0f);

                ImVec4 color = m_statusIsError ? ImVec4(1.0f, 0.45f, 0.4f, alpha)
                                               : ImVec4(0.5f, 1.0f, 0.6f, alpha);
                ImGui::TextColored(color, "%s", m_statusMessage.c_str());
            }
        }

        ImGui::EndMainMenuBar();
    }

    // Keyboard shortcuts (ignored while a text field has focus)
    if (!ImGui::GetIO().WantTextInput && ImGui::GetIO().KeyCtrl) {
        if (ImGui::IsKeyPressed(ImGuiKey_O, false)) doLoadPattern(app);
        if (ImGui::IsKeyPressed(ImGuiKey_S, false)) doSavePattern(app);
        if (ImGui::IsKeyPressed(ImGuiKey_E, false)) doExportWav(app);
        if (ImGui::IsKeyPressed(ImGuiKey_I, false)) doExportImage(app);
    }

    if (m_showAbout) {
        ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(),
                                ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
        ImGui::Begin("About Oscilloplot", &m_showAbout,
                     ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse);
        ImGui::Text("Oscilloplot %s", OSCILLOPLOT_VERSION);
        ImGui::TextDisabled("XY oscilloscope audio generator");
        ImGui::Separator();
        ImGui::Text("Renderer: %s", (const char*)glGetString(GL_VERSION));
        ImGui::Text("Built %s", __DATE__);
        ImGui::Separator();
        ImGui::TextDisabled("MIT License");
        ImGui::TextDisabled("github.com/jfalvarez1/oscilloplot");
        if (ImGui::Button("Close", ImVec2(-1, 0))) m_showAbout = false;
        ImGui::End();
    }
}

void UIManager::renderControlPanel(App& app) {
    // Position: Left side
    placeWindow(0.006f, 0.008f, 0.20f, 0.60f);
    ImGui::Begin("Controls");

    // Primary action first
    if (app.isPlaying()) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.75f, 0.22f, 0.20f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.85f, 0.30f, 0.28f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.65f, 0.18f, 0.16f, 1.0f));
    } else {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.22f, 0.56f, 0.19f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.29f, 0.68f, 0.25f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.18f, 0.46f, 0.16f, 1.0f));
    }
    if (ImGui::Button(app.isPlaying() ? "Stop" : "Play", ImVec2(-1, 46))) {
        app.setPlaying(!app.isPlaying());
    }
    ImGui::PopStyleColor(3);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Space");

    // Pattern info + playback position
    ImGui::Text("Pattern: %zu points", app.getPattern().size());
    if (app.isPlaying()) {
        float pos = app.getAudioEngine().getPlaybackPosition();
        ImGui::ProgressBar(pos, ImVec2(-1, 4), "");
    }
    if (!app.isAudioAvailable()) {
        ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "No audio device");
    }

    ImGui::Spacing();
    ImGui::SeparatorText("Audio");

    // Narrow panel: labels above full-width sliders so nothing truncates
    ImGui::PushItemWidth(-FLT_MIN);

    ImGui::TextUnformatted("Base Rate");
    int sampleRate = app.getSampleRate();
    if (ImGui::SliderInt("##BaseRate", &sampleRate, 100, 10000, "%d Hz")) {
        app.setSampleRate(sampleRate);
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("How fast the beam steps through the pattern.\nLower = slower, brighter trace.");
    }

    ImGui::TextUnformatted("Multiplier");
    int multiplier = app.getPlaybackMultiplier();
    if (ImGui::SliderInt("##Multiplier", &multiplier, 10, 500, "x%d")) {
        app.setPlaybackMultiplier(multiplier);
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Output sample rate = base rate x multiplier.\nHigher = smoother beam movement.");
    }

    ImGui::TextDisabled("Output rate: %d Hz", sampleRate * multiplier);

    ImGui::TextUnformatted("Export Length");
    int duration = app.getDuration();
    if (ImGui::SliderInt("##ExportLength", &duration, 1, 120, "%d s")) {
        app.setDuration(duration);
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Length of exported WAV files.\nLive playback loops until stopped.");
    }

    ImGui::PopItemWidth();

    ImGui::End();
}

//==============================================================================
// Phosphor Display - Realistic CRT Oscilloscope
//==============================================================================

void UIManager::renderOscilloscopeDisplay(App& app) {
    // Position: Center
    placeWindow(0.212f, 0.008f, 0.50f, 0.965f);
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
    ImGui::SameLine();
    ImGui::Checkbox("FX Preview", &m_previewEffects);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Show effects on the scope while stopped");
    }

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
            //
            // Effects used to be invisible until playback started, so every
            // effect had to be tuned blind. The chain is run here on the UI
            // thread - safe because the audio callback is not running - and the
            // result is what gets drawn.
            //==================================================================
            const Pattern& source = app.getPattern();
            size_t pCount = 0;
            const float* px = nullptr;
            const float* py = nullptr;

            if (!source.empty()) {
                if (m_previewEffects && !m_effectsBypassed) {
                    pCount = app.getAudioEngine().renderPreview(
                        source, m_vizX, m_vizY, VIZ_SAMPLES,
                        static_cast<float>(ImGui::GetTime()));
                    px = m_vizX;
                    py = m_vizY;
                }
                if (pCount == 0) {          // preview off, or nothing rendered
                    pCount = source.size();
                    px = source.xData();
                    py = source.yData();
                }
            }

            if (pCount > 0) {

                // Dim glow layer
                ImPlot::PushStyleColor(ImPlotCol_Line,
                    ImVec4(m_phosphor.colorR * 0.25f, m_phosphor.colorG * 0.35f,
                           m_phosphor.colorB * 0.2f, 0.35f));
                ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, m_phosphor.beamWidth * 3.0f);
                ImPlot::PlotLine("##PrevGlow", px, py, static_cast<int>(pCount));
                ImPlot::PopStyleVar();
                ImPlot::PopStyleColor();

                // Core preview trace
                ImPlot::PushStyleColor(ImPlotCol_Line,
                    ImVec4(m_phosphor.colorR * 0.5f, m_phosphor.colorG * 0.65f,
                           m_phosphor.colorB * 0.45f, 0.7f));
                ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, m_phosphor.beamWidth);
                ImPlot::PlotLine("##Preview", px, py, static_cast<int>(pCount));
                ImPlot::PopStyleVar();
                ImPlot::PopStyleColor();
            }
        }

        //======================================================================
        // CRT post-effects, drawn over the trace inside the plot rect.
        //
        // These were previously exposed as sliders that did nothing. They are
        // draw-list overlays rather than a shader: the scope is rendered with
        // ImGui/ImPlot primitives, so there is no offscreen texture to sample,
        // and overlays cost nothing when their strength is zero.
        //======================================================================
        // Drawn inside the plot, so GetWindowDrawList() is the plot's own child
        // draw list and the overlays land on top of the trace. Drawing them
        // after EndPlot puts them on the parent list, which renders first and
        // leaves them hidden underneath.
        renderCrtOverlays(ImPlot::GetPlotPos(), ImPlot::GetPlotSize());

        ImPlot::EndPlot();
    }

    ImPlot::PopStyleColor(3);
}

void UIManager::renderCrtOverlays(const ImVec2& pos, const ImVec2& size) {
    const bool anyEnabled = m_phosphor.vignetteStrength > 0.001f ||
                            m_phosphor.scanlineEffect  > 0.001f ||
                            m_phosphor.noiseAmount     > 0.001f;
    if (!anyEnabled) return;

    if (size.x < 1.0f || size.y < 1.0f) return;

    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->PushClipRect(pos, ImVec2(pos.x + size.x, pos.y + size.y), true);

    //--------------------------------------------------------------------------
    // Scanlines: horizontal dark bands, as on a raster CRT.
    //--------------------------------------------------------------------------
    if (m_phosphor.scanlineEffect > 0.001f) {
        const float spacing = 3.0f;
        const int alpha = static_cast<int>(m_phosphor.scanlineEffect * 110.0f);
        const ImU32 col = IM_COL32(0, 0, 0, alpha);
        for (float y = pos.y; y < pos.y + size.y; y += spacing) {
            dl->AddLine(ImVec2(pos.x, y), ImVec2(pos.x + size.x, y), col, 1.0f);
        }
    }

    //--------------------------------------------------------------------------
    // Vignette: darkening towards the edges, built from nested rectangle
    // outlines so it stays cheap and needs no gradient texture.
    //--------------------------------------------------------------------------
    if (m_phosphor.vignetteStrength > 0.001f) {
        const int steps = 24;
        const float maxInset = std::min(size.x, size.y) * 0.45f;
        for (int i = 0; i < steps; ++i) {
            const float t = static_cast<float>(i) / static_cast<float>(steps);
            const float inset = maxInset * t;
            // Strongest at the very edge, fading inwards.
            const float strength = (1.0f - t) * (1.0f - t) *
                                   m_phosphor.vignetteStrength;
            const int alpha = static_cast<int>(strength * 90.0f);
            if (alpha <= 0) continue;
            dl->AddRect(ImVec2(pos.x + inset, pos.y + inset),
                        ImVec2(pos.x + size.x - inset, pos.y + size.y - inset),
                        IM_COL32(0, 0, 0, alpha),
                        0.0f, 0, maxInset / steps + 1.0f);
        }
    }

    //--------------------------------------------------------------------------
    // Analog noise: sparse bright speckle. Deterministic per frame via a small
    // xorshift so it shimmers without needing a RNG object.
    //--------------------------------------------------------------------------
    if (m_phosphor.noiseAmount > 0.001f) {
        uint32_t rng = 0x9E3779B9u ^ (m_frameCount * 2654435761u);
        auto next = [&rng]() {
            rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
            return rng;
        };

        const int count = static_cast<int>(m_phosphor.noiseAmount * 4000.0f);
        for (int i = 0; i < count; ++i) {
            const float fx = pos.x + (next() % 10000) / 10000.0f * size.x;
            const float fy = pos.y + (next() % 10000) / 10000.0f * size.y;
            const int a = 20 + static_cast<int>((next() % 60));
            dl->AddRectFilled(ImVec2(fx, fy), ImVec2(fx + 1.0f, fy + 1.0f),
                              IM_COL32(static_cast<int>(m_phosphor.colorR * 255),
                                       static_cast<int>(m_phosphor.colorG * 255),
                                       static_cast<int>(m_phosphor.colorB * 255), a));
        }
    }

    dl->PopClipRect();
}

//==============================================================================
// Effects Panel
//==============================================================================

void UIManager::renderEffectsPanel(App& app) {
    auto& fx = app.getEffects();

    // Position: Right side, top
    placeWindow(0.718f, 0.008f, 0.276f, 0.475f);
    ImGui::Begin("Effects", &m_showEffectsPanel);

    //--------------------------------------------------------------------------
    // Master controls: bypass everything at once, or return to defaults. There
    // was previously no way to clear the chain without visiting every section.
    //--------------------------------------------------------------------------
    if (ImGui::Checkbox("Bypass All", &m_effectsBypassed)) {
        if (m_effectsBypassed) {
            // Remember the live settings so the switch is non-destructive.
            captureEffects(fx, m_bypassSnapshot);
            Preset silent;                 // no keys -> every effect defaults off
            applyEffects(silent, fx);
        } else {
            applyEffects(m_bypassSnapshot, fx);
        }
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Silence every effect without losing the settings");
    }
    ImGui::SameLine();

    if (ImGui::Button("Reset All")) {
        ImGui::OpenPopup("Reset all effects?");
    }
    if (ImGui::BeginPopupModal("Reset all effects?", nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted("Return every effect to its default? This cannot be undone.");
        ImGui::Spacing();
        if (ImGui::Button("Reset", ImVec2(120, 0))) {
            Preset defaults;               // empty preset == all defaults
            applyEffects(defaults, fx);
            m_effectsBypassed = false;
            m_bypassSnapshot = Preset{};
            setStatus("Effects reset to defaults");
            ImGui::CloseCurrentPopup();
        }
        ImGui::SetItemDefaultFocus();
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(120, 0))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    if (m_effectsBypassed) {
        ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "All effects bypassed");
    }
    ImGui::Separator();

    // While bypassed the individual controls would write into settings that are
    // not in use, which reads as broken; show them disabled instead.
    if (m_effectsBypassed) ImGui::BeginDisabled();

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

    if (m_effectsBypassed) ImGui::EndDisabled();

    ImGui::End();
}

//==============================================================================
// Generators Panel
//==============================================================================

void UIManager::renderGeneratorsPanel(App& app) {
    // Position: Right side, below Effects
    placeWindow(0.718f, 0.495f, 0.276f, 0.478f);
    ImGui::Begin("Generators", &m_showGeneratorsPanel);

    //--------------------------------------------------------------------------
    // Live parameter feedback
    //
    // Clicking a Generate button makes that generator "active"; from then on
    // any change to its parameters (or the shared Points slider) regenerates
    // the pattern immediately, so sliders give live feedback on the scope.
    //--------------------------------------------------------------------------
    using Gen = ActiveGen;

    // All generator parameters (declared up front so the dispatcher sees them)
    static int numPoints = 500;
    static float sineFreq = 3.0f;
    static float ellipseX = 1.0f, ellipseY = 0.6f;
    static int lissA = 3, lissB = 2;
    static float lissPhase = 0.0f;
    static int starSpikes = 5;
    static float starInner = 0.5f;
    static int flowerPetals = 6;
    static float petalDepth = 0.5f;
    static int roseK = 4;
    static float spiralTurns = 5.0f;
    static float spiralStart = 0.1f, spiralEnd = 1.0f;
    static float logA = 0.1f, logB = 0.1f, logMaxAngle = 18.85f;
    static int torusP = 2, torusQ = 3;
    static float hypoR = 5.0f, hypoR2 = 3.0f, hypoD = 5.0f;
    static float epiR = 5.0f, epiR2 = 2.0f, epiD = 2.0f;
    static float waveFreq = 3.0f;

    // Generate `id` into the app pattern and make it the active generator.
    auto runGenerator = [&](Gen id) {
        Pattern& p = app.getPattern();
        switch (id) {
            case Gen::None:      return;
            case Gen::Circle:    generators::generateCircle(p, numPoints); break;
            case Gen::Ellipse:   generators::generateEllipse(p, numPoints, ellipseX, ellipseY); break;
            case Gen::Sine:      generators::generateSineWave(p, numPoints, sineFreq); break;
            case Gen::Lissajous: generators::generateLissajous(p, numPoints, lissA, lissB, lissPhase); break;
            case Gen::Star:      generators::generateStar(p, numPoints, starSpikes, starInner); break;
            case Gen::Flower:    generators::generateFlower(p, numPoints, flowerPetals, petalDepth); break;
            case Gen::Rose:      generators::generateRoseCurve(p, numPoints, roseK); break;
            case Gen::ArchSpiral: generators::generateSpiral(p, numPoints, spiralTurns, spiralStart, spiralEnd); break;
            case Gen::LogSpiral: generators::generateLogSpiral(p, numPoints, logA, logB, logMaxAngle); break;
            case Gen::Helix:     generators::generateHelix(p, numPoints); break;
            case Gen::Trefoil:   generators::generateTrefoilKnot(p, numPoints); break;
            case Gen::TorusKnot: generators::generateTorusKnot(p, numPoints, torusP, torusQ); break;
            case Gen::Butterfly: generators::generateButterfly(p, 2000); break;  // Needs more points
            case Gen::Cardioid:  generators::generateCardioid(p, numPoints); break;
            case Gen::Deltoid:   generators::generateDeltoid(p, numPoints); break;
            case Gen::Hypotrochoid: generators::generateHypotrochoid(p, numPoints, hypoR, hypoR2, hypoD); break;
            case Gen::Epitrochoid:  generators::generateEpitrochoid(p, numPoints, epiR, epiR2, epiD); break;
            case Gen::Figure8:   generators::generateFigure8(p, numPoints); break;
            case Gen::Infinity:  generators::generateInfinity(p, numPoints); break;
            case Gen::Heart:     generators::generateHeart(p, numPoints); break;
            case Gen::Square:    generators::generateSquareWave(p, numPoints, waveFreq); break;
            case Gen::Sawtooth:  generators::generateSawtoothWave(p, numPoints, waveFreq); break;
            case Gen::Triangle:  generators::generateTriangleWave(p, numPoints, waveFreq); break;
        }
        m_activeGen = id;
        m_3dShapeActive = false;  // Stop 3D animation from overwriting
        app.getAudioEngine().setPattern(p);
    };

    // Re-run the active generator when one of its parameters changed.
    auto liveUpdate = [&](bool changed, Gen id) {
        if (changed && m_activeGen == id) runGenerator(id);
    };

    // Shared parameter: affects whichever generator is active
    if (ImGui::SliderInt("Points", &numPoints, 100, 5000) && m_activeGen != Gen::None) {
        runGenerator(m_activeGen);
    }

    //==========================================================================
    // BASIC SHAPES
    //==========================================================================
    if (ImGui::CollapsingHeader("Basic Shapes", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button("Circle", ImVec2(100, 0))) runGenerator(Gen::Circle);
        ImGui::SameLine();
        if (ImGui::Button("Ellipse", ImVec2(100, 0))) runGenerator(Gen::Ellipse);

        bool ch = false;
        ImGui::SetNextItemWidth(80);
        ch |= ImGui::InputFloat("X##ell", &ellipseX, 0.0f, 0.0f, "%.2f"); ImGui::SameLine();
        ImGui::SetNextItemWidth(80);
        ch |= ImGui::InputFloat("Y##ell", &ellipseY, 0.0f, 0.0f, "%.2f");
        liveUpdate(ch, Gen::Ellipse);

        ImGui::SetNextItemWidth(120);
        liveUpdate(ImGui::SliderFloat("Sine Cycles", &sineFreq, 1.0f, 10.0f), Gen::Sine);
        ImGui::SameLine();
        if (ImGui::Button("Sine Wave", ImVec2(-1, 0))) runGenerator(Gen::Sine);
    }

    //==========================================================================
    // LISSAJOUS CURVES
    //==========================================================================
    if (ImGui::CollapsingHeader("Lissajous Curves")) {
        bool ch = false;
        ImGui::SetNextItemWidth(60);
        ch |= ImGui::InputInt("A##liss", &lissA); ImGui::SameLine();
        ImGui::SetNextItemWidth(60);
        ch |= ImGui::InputInt("B##liss", &lissB); ImGui::SameLine();
        ImGui::SetNextItemWidth(80);
        ch |= ImGui::SliderFloat("Phase##liss", &lissPhase, 0.0f, PI, "%.2f");

        if (ImGui::Button("3:2", ImVec2(50, 0)))       { lissA = 3; lissB = 2; lissPhase = 0.0f; ch = true; }
        ImGui::SameLine();
        if (ImGui::Button("3:2 (+90)", ImVec2(70, 0))) { lissA = 3; lissB = 2; lissPhase = PI / 2.0f; ch = true; }
        ImGui::SameLine();
        if (ImGui::Button("5:4", ImVec2(50, 0)))       { lissA = 5; lissB = 4; lissPhase = 0.0f; ch = true; }
        ImGui::SameLine();
        if (ImGui::Button("7:5", ImVec2(50, 0)))       { lissA = 7; lissB = 5; lissPhase = PI / 3.0f; ch = true; }
        liveUpdate(ch, Gen::Lissajous);

        if (ImGui::Button("Generate Lissajous", ImVec2(-1, 0))) runGenerator(Gen::Lissajous);
    }

    //==========================================================================
    // STARS & FLOWERS
    //==========================================================================
    if (ImGui::CollapsingHeader("Stars & Flowers")) {
        bool ch = false;
        ImGui::Text("Star:");
        ImGui::SetNextItemWidth(100);
        ch |= ImGui::SliderInt("Spikes##star", &starSpikes, 3, 20);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(100);
        ch |= ImGui::SliderFloat("Inner Radius##star", &starInner, 0.1f, 0.9f, "%.2f");
        liveUpdate(ch, Gen::Star);
        ImGui::SameLine();
        if (ImGui::Button("Generate Star", ImVec2(-1, 0))) runGenerator(Gen::Star);

        ch = false;
        ImGui::Text("Flower:");
        ImGui::SetNextItemWidth(100);
        ch |= ImGui::SliderInt("Petals##flower", &flowerPetals, 2, 20);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(100);
        ch |= ImGui::SliderFloat("Petal Depth##flower", &petalDepth, 0.1f, 0.9f, "%.2f");
        liveUpdate(ch, Gen::Flower);
        ImGui::SameLine();
        if (ImGui::Button("Generate Flower", ImVec2(-1, 0))) runGenerator(Gen::Flower);

        ImGui::Text("Rose Curve (k petals):");
        ImGui::SetNextItemWidth(100);
        liveUpdate(ImGui::SliderInt("k (petals)##rose", &roseK, 1, 12), Gen::Rose);
        ImGui::SameLine();
        if (ImGui::Button("Generate Rose", ImVec2(-1, 0))) runGenerator(Gen::Rose);
    }

    //==========================================================================
    // SPIRALS
    //==========================================================================
    if (ImGui::CollapsingHeader("Spirals")) {
        bool ch = false;
        ImGui::Text("Archimedean Spiral");
        ImGui::SetNextItemWidth(80);
        ch |= ImGui::SliderFloat("Turns##arch", &spiralTurns, 1.0f, 20.0f);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(60);
        ch |= ImGui::SliderFloat("Start##arch", &spiralStart, 0.0f, 0.5f, "%.2f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(60);
        ch |= ImGui::SliderFloat("End##arch", &spiralEnd, 0.5f, 1.5f, "%.2f");
        liveUpdate(ch, Gen::ArchSpiral);
        if (ImGui::Button("Generate Archimedean", ImVec2(-1, 0))) runGenerator(Gen::ArchSpiral);

        ch = false;
        ImGui::Text("Logarithmic Spiral");
        ImGui::SetNextItemWidth(60);
        ch |= ImGui::SliderFloat("a##log", &logA, 0.01f, 0.5f, "%.2f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(60);
        ch |= ImGui::SliderFloat("b##log", &logB, 0.01f, 0.3f, "%.2f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80);
        ch |= ImGui::SliderFloat("Max Angle##log", &logMaxAngle, 6.0f, 30.0f, "%.1f");
        liveUpdate(ch, Gen::LogSpiral);
        if (ImGui::Button("Generate Logarithmic", ImVec2(-1, 0))) runGenerator(Gen::LogSpiral);
    }

    //==========================================================================
    // KNOTS & 3D-STYLE
    //==========================================================================
    if (ImGui::CollapsingHeader("Knots & 3D-Style")) {
        if (ImGui::Button("Helix", ImVec2(80, 0))) runGenerator(Gen::Helix);
        ImGui::SameLine();
        if (ImGui::Button("Trefoil Knot", ImVec2(100, 0))) runGenerator(Gen::Trefoil);

        bool ch = false;
        ImGui::Text("Torus Knot (p:q ratio):");
        ImGui::SetNextItemWidth(80);
        ch |= ImGui::SliderInt("p##torus", &torusP, 1, 10);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80);
        ch |= ImGui::SliderInt("q##torus", &torusQ, 1, 10);
        liveUpdate(ch, Gen::TorusKnot);
        ImGui::SameLine();
        if (ImGui::Button("Generate Torus Knot", ImVec2(-1, 0))) runGenerator(Gen::TorusKnot);
    }

    //==========================================================================
    // COMPLEX CURVES
    //==========================================================================
    if (ImGui::CollapsingHeader("Complex Curves")) {
        if (ImGui::Button("Butterfly", ImVec2(80, 0))) runGenerator(Gen::Butterfly);
        ImGui::SameLine();
        if (ImGui::Button("Cardioid", ImVec2(80, 0))) runGenerator(Gen::Cardioid);
        ImGui::SameLine();
        if (ImGui::Button("Deltoid", ImVec2(80, 0))) runGenerator(Gen::Deltoid);

        bool ch = false;
        ImGui::Text("Hypotrochoid (R, r, d):");
        ImGui::SetNextItemWidth(60);
        ch |= ImGui::SliderFloat("R##hypo", &hypoR, 1.0f, 10.0f, "%.1f"); ImGui::SameLine();
        ImGui::SetNextItemWidth(60);
        ch |= ImGui::SliderFloat("r##hypo", &hypoR2, 0.5f, 8.0f, "%.1f"); ImGui::SameLine();
        ImGui::SetNextItemWidth(60);
        ch |= ImGui::SliderFloat("d##hypo", &hypoD, 0.5f, 10.0f, "%.1f"); ImGui::SameLine();
        liveUpdate(ch, Gen::Hypotrochoid);
        if (ImGui::Button("Generate##hypo", ImVec2(-1, 0))) runGenerator(Gen::Hypotrochoid);

        ch = false;
        ImGui::Text("Epitrochoid (R, r, d):");
        ImGui::SetNextItemWidth(60);
        ch |= ImGui::SliderFloat("R##epi", &epiR, 1.0f, 10.0f, "%.1f"); ImGui::SameLine();
        ImGui::SetNextItemWidth(60);
        ch |= ImGui::SliderFloat("r##epi", &epiR2, 0.5f, 8.0f, "%.1f"); ImGui::SameLine();
        ImGui::SetNextItemWidth(60);
        ch |= ImGui::SliderFloat("d##epi", &epiD, 0.5f, 10.0f, "%.1f"); ImGui::SameLine();
        liveUpdate(ch, Gen::Epitrochoid);
        if (ImGui::Button("Generate##epi", ImVec2(-1, 0))) runGenerator(Gen::Epitrochoid);
    }

    //==========================================================================
    // SPECIAL SHAPES
    //==========================================================================
    if (ImGui::CollapsingHeader("Special Shapes")) {
        if (ImGui::Button("Figure-8", ImVec2(80, 0))) runGenerator(Gen::Figure8);
        ImGui::SameLine();
        if (ImGui::Button("Infinity", ImVec2(80, 0))) runGenerator(Gen::Infinity);
        ImGui::SameLine();
        if (ImGui::Button("Heart", ImVec2(80, 0))) runGenerator(Gen::Heart);
    }

    //==========================================================================
    // WAVEFORMS
    //==========================================================================
    if (ImGui::CollapsingHeader("Waveforms")) {
        ImGui::SetNextItemWidth(120);
        // Frequency drives whichever of the three waveforms was last generated
        if (ImGui::SliderFloat("Frequency##wave", &waveFreq, 1.0f, 10.0f)) {
            if (m_activeGen == Gen::Square || m_activeGen == Gen::Sawtooth ||
                m_activeGen == Gen::Triangle) {
                runGenerator(m_activeGen);
            }
        }

        if (ImGui::Button("Square Wave", ImVec2(100, 0))) runGenerator(Gen::Square);
        ImGui::SameLine();
        if (ImGui::Button("Sawtooth", ImVec2(100, 0))) runGenerator(Gen::Sawtooth);
        ImGui::SameLine();
        if (ImGui::Button("Triangle", ImVec2(-1, 0))) runGenerator(Gen::Triangle);
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
        claimPatternSource();       // Random result has no live parameters
        app.getAudioEngine().setPattern(pattern);
    }

    ImGui::End();
}

//==============================================================================
// Harmonics Editor - Text inputs with live preview
//==============================================================================

void UIManager::generateHarmonicsPattern(App& app) {
    claimPatternSource();  // Stop 3D/generator sources overwriting it
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
        // Each term derives its own progress from its own sweep length below,
        // so there is no shared per-step progress value to compute here.
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
    placeWindow(0.24f, 0.10f, 0.42f, 0.68f);
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
    ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f), "Presets (replaces current settings!):");
    if (ImGui::Button("3:2 Lissajous", ImVec2(90, 0))) {
        m_harmonics.numXTerms = 1; m_harmonics.numYTerms = 1;
        m_harmonics.xTerms[0] = {1.0f, 3.0f, 0.0f, true, true, false, 0.0f, TWO_PI, 20, false, 1.0f, 5.0f, 20};
        m_harmonics.yTerms[0] = {1.0f, 2.0f, PI / 2.0f, true, true, false, 0.0f, TWO_PI, 20, false, 1.0f, 5.0f, 20};
        changed = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("5:4 Lissajous", ImVec2(90, 0))) {
        m_harmonics.numXTerms = 1; m_harmonics.numYTerms = 1;
        m_harmonics.xTerms[0] = {1.0f, 5.0f, 0.0f, true, true, false, 0.0f, TWO_PI, 20, false, 1.0f, 5.0f, 20};
        m_harmonics.yTerms[0] = {1.0f, 4.0f, PI / 4.0f, true, true, false, 0.0f, TWO_PI, 20, false, 1.0f, 5.0f, 20};
        changed = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Complex 3+3", ImVec2(85, 0))) {
        m_harmonics.numXTerms = 3; m_harmonics.numYTerms = 3;
        m_harmonics.xTerms[0] = {1.0f, 1.0f, 0.0f, true, true, false, 0.0f, TWO_PI, 20, false, 1.0f, 5.0f, 20};
        m_harmonics.xTerms[1] = {0.3f, 3.0f, 0.0f, true, true, false, 0.0f, TWO_PI, 20, false, 1.0f, 5.0f, 20};
        m_harmonics.xTerms[2] = {0.1f, 5.0f, 0.0f, true, true, false, 0.0f, TWO_PI, 20, false, 1.0f, 5.0f, 20};
        m_harmonics.yTerms[0] = {1.0f, 2.0f, PI / 2.0f, true, true, false, 0.0f, TWO_PI, 20, false, 1.0f, 5.0f, 20};
        m_harmonics.yTerms[1] = {0.3f, 4.0f, 0.0f, true, true, false, 0.0f, TWO_PI, 20, false, 1.0f, 5.0f, 20};
        m_harmonics.yTerms[2] = {0.1f, 6.0f, 0.0f, true, true, false, 0.0f, TWO_PI, 20, false, 1.0f, 5.0f, 20};
        changed = true;
    }

    ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "Sweep Demos:");
    if (ImGui::Button("Phase Sweep Demo", ImVec2(120, 0))) {
        m_harmonics.numXTerms = 1; m_harmonics.numYTerms = 1;
        m_harmonics.xTerms[0] = {1.0f, 3.0f, 0.0f, true, true, true, 0.0f, TWO_PI, 30, false, 1.0f, 5.0f, 20};
        m_harmonics.yTerms[0] = {1.0f, 2.0f, 0.0f, true, true, false, 0.0f, TWO_PI, 20, false, 1.0f, 5.0f, 20};
        changed = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Freq Sweep Demo", ImVec2(120, 0))) {
        m_harmonics.numXTerms = 1; m_harmonics.numYTerms = 1;
        m_harmonics.xTerms[0] = {1.0f, 1.0f, 0.0f, true, true, false, 0.0f, TWO_PI, 20, true, 1.0f, 8.0f, 40};
        m_harmonics.yTerms[0] = {1.0f, 1.0f, PI / 2.0f, true, true, false, 0.0f, TWO_PI, 20, false, 1.0f, 5.0f, 20};
        changed = true;
    }

    ImGui::TextDisabled("Tip: Use checkboxes above to add sweeps to your own terms");

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
    placeWindow(0.28f, 0.08f, 0.36f, 0.75f);
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
        // Auto-update pattern when stroke is finished
        if (m_drawing.pointsX.size() >= 2) {
            claimPatternSource();  // Stop 3D/generator sources overwriting it
            Pattern& pattern = app.getPattern();
            pattern.clear();
            pattern.x = m_drawing.pointsX;
            pattern.y = m_drawing.pointsY;
            app.getAudioEngine().setPattern(pattern);
        }
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

    // Undo removes the most recent stroke; the stroke boundaries are already
    // tracked for rendering, so undo is just truncating to the last one.
    bool undoRequested = false;
    bool canUndo = !m_drawing.strokeStarts.empty();
    if (!canUndo) ImGui::BeginDisabled();
    if (ImGui::Button("Undo Stroke (Ctrl+Z)", ImVec2(-1, 0))) {
        undoRequested = true;
    }
    if (!canUndo) ImGui::EndDisabled();

    if (canUndo && ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) &&
        ImGui::GetIO().KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Z, false)) {
        undoRequested = true;
    }

    if (undoRequested && canUndo) {
        size_t cut = m_drawing.strokeStarts.back();
        m_drawing.strokeStarts.pop_back();
        m_drawing.pointsX.resize(cut);
        m_drawing.pointsY.resize(cut);
    }

    if (ImGui::Button("Clear Canvas", ImVec2(-1, 0)) && !m_drawing.pointsX.empty()) {
        ImGui::OpenPopup("Clear canvas?");
    }
    if (ImGui::BeginPopupModal("Clear canvas?", nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("Delete all %zu strokes? This cannot be undone.",
                    m_drawing.strokeStarts.size());
        ImGui::Spacing();
        if (ImGui::Button("Clear", ImVec2(120, 0))) {
            m_drawing.pointsX.clear();
            m_drawing.pointsY.clear();
            m_drawing.strokeStarts.clear();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SetItemDefaultFocus();
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(120, 0))) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    if (ImGui::Button("Use as Pattern", ImVec2(-1, 30))) {
        if (m_drawing.pointsX.size() >= 2) {
            claimPatternSource();  // Stop 3D/generator sources overwriting it
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
    buildShapeFrame(app.getPattern());
    app.getAudioEngine().setPattern(app.getPattern());
}

// Build one projected frame of the current 3D shape at the current rotation
// into `pattern`. Pure geometry - no audio engine interaction - so the bake
// path can call it repeatedly at stepped rotations.
void UIManager::buildShapeFrame(Pattern& pattern) {
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

            return;  // Early return since we handle pattern directly
        }
        case Shape3DState::ShapeType::ObjModel: {
            // Nothing loaded yet - leave the pattern empty rather than
            // silently falling back to another shape.
            if (m_shape3D.objMesh.empty()) {
                return;
            }

            vertices.reserve(m_shape3D.objMesh.vertices.size());
            for (const auto& v : m_shape3D.objMesh.vertices) {
                vertices.emplace_back(v.x, v.y, v.z);
            }
            edges = m_shape3D.objMesh.edges;
            break;
        }
    }

    if (vertices.empty() || edges.empty()) {
        return;
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
}

void UIManager::loadObjModel(App& app, const std::string& path) {
    ObjMesh mesh;
    std::string error;

    if (!ObjLoader::load(path, mesh, error)) {
        m_shape3D.objStatus = "Load failed: " + error;
        setStatus("OBJ load failed: " + error, true);
        return;
    }

    m_shape3D.objSourceEdges = mesh.edges.size();
    ObjLoader::decimate(mesh, static_cast<size_t>(m_shape3D.objMaxEdges));

    m_shape3D.objMesh = std::move(mesh);
    m_shape3D.objPath = path;

    char summary[256];
    if (m_shape3D.objMesh.edges.size() < m_shape3D.objSourceEdges) {
        snprintf(summary, sizeof(summary),
                 "%zu verts, %zu faces, %zu of %zu edges",
                 m_shape3D.objMesh.vertices.size(), m_shape3D.objMesh.faceCount,
                 m_shape3D.objMesh.edges.size(), m_shape3D.objSourceEdges);
    } else {
        snprintf(summary, sizeof(summary),
                 "%zu verts, %zu faces, %zu edges",
                 m_shape3D.objMesh.vertices.size(), m_shape3D.objMesh.faceCount,
                 m_shape3D.objMesh.edges.size());
    }
    m_shape3D.objStatus = summary;

    // Loading a model makes it the active shape.
    m_shape3D.shapeType = Shape3DState::ShapeType::ObjModel;
    m_3dShapeActive = true;
    generate3DShapePattern(app);

    setStatus("Loaded " + m_shape3D.objMesh.name + " (" + m_shape3D.objStatus + ")");
}

void UIManager::render3DShapeGenerator(App& app) {
    // Position: Floating, upper center
    placeWindow(0.30f, 0.05f, 0.28f, 0.82f);
    ImGui::Begin("3D Shape Generator", &m_show3DShapeGenerator);

    ImGui::Text("Project 3D wireframes to 2D XY");
    ImGui::Separator();

    // Shape selection - expanded list
    const char* shapes[] = {
        "Cube", "Tetrahedron", "Octahedron", "Icosahedron", "Dodecahedron",
        "Sphere", "Torus", "Cylinder", "Cone", "Pyramid", "Prism",
        "3D Spiral", "Double Helix", "Trefoil Knot",
        "Mobius Strip", "Klein Bottle", "Spring", "3D Star",
        "3D Text", "OBJ Model"
    };
    int shapeIdx = static_cast<int>(m_shape3D.shapeType);
    if (ImGui::Combo("Shape", &shapeIdx, shapes, IM_ARRAYSIZE(shapes))) {
        m_shape3D.shapeType = static_cast<Shape3DState::ShapeType>(shapeIdx);
        m_3dShapeActive = true;   // Re-claim the pattern from other sources
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

    // Live animation regenerates the pattern per UI frame, so a WAV export
    // only ever captures one orientation. Baking freezes N rotation steps into
    // a single long pattern that plays (and exports) as a rotating shape.
    if (ImGui::TreeNode("Bake Rotation")) {
        ImGui::SliderInt("Frames", &m_shape3D.bakeFrames, 2, 400);
        ImGui::TextDisabled("Each frame advances by the animation speeds");

        // Two destinations: one long pattern (simple, but bounded by the
        // engine's buffer) or one sequencer step per frame (no frame limit,
        // and the result stays editable afterwards).
        ImGui::RadioButton("To Pattern", &m_bakeToSequencer, 0);
        ImGui::SameLine();
        ImGui::RadioButton("To Sequencer", &m_bakeToSequencer, 1);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("One step per frame - no frame cap, and you can edit or crossfade the result");
        }

        if (m_bakeToSequencer) {
            ImGui::SliderInt("Cycles/frame", &m_bakeCyclesPerFrame, 1, 50);
        }

        if (m_bakeToSequencer && ImGui::Button("Bake to Sequencer", ImVec2(-1, 30))) {
            float rx0 = m_shape3D.rotationX;
            float ry0 = m_shape3D.rotationY;
            float rz0 = m_shape3D.rotationZ;

            m_sequencerUndo.push(m_sequencer.seq);
            Sequence built;
            built.loop = true;
            built.crossfade = 0.0f;   // frames already form a continuous motion

            Pattern frame;
            for (int f = 0; f < m_shape3D.bakeFrames; ++f) {
                buildShapeFrame(frame);
                if (!frame.empty()) {
                    SequenceStep step;
                    step.pattern = frame;
                    step.cycles = m_bakeCyclesPerFrame;
                    snprintf(step.name, sizeof(step.name), "Frame %d", f + 1);
                    built.steps.push_back(std::move(step));
                }
                m_shape3D.rotationX += m_shape3D.rotationSpeedX;
                m_shape3D.rotationY += m_shape3D.rotationSpeedY;
                m_shape3D.rotationZ += m_shape3D.rotationSpeedZ;
            }

            m_shape3D.rotationX = rx0;
            m_shape3D.rotationY = ry0;
            m_shape3D.rotationZ = rz0;

            if (built.steps.empty()) {
                setStatus("Nothing to bake - shape produced no points", true);
            } else {
                m_sequencer.seq = std::move(built);
                m_sequencer.playing = false;
                m_sequencer.currentStep = -1;
                m_showSequencer = true;
                claimPatternSource();
                setStatus("Baked " + std::to_string(m_sequencer.seq.size()) +
                          " frames into the sequencer");
            }
        }

        if (!m_bakeToSequencer && ImGui::Button("Bake to Pattern", ImVec2(-1, 30))) {
            float rx0 = m_shape3D.rotationX;
            float ry0 = m_shape3D.rotationY;
            float rz0 = m_shape3D.rotationZ;

            Pattern baked;
            Pattern frame;
            bool truncated = false;
            int framesDone = 0;

            for (int f = 0; f < m_shape3D.bakeFrames; ++f) {
                buildShapeFrame(frame);
                if (baked.size() + frame.size() > AudioEngine::MAX_PATTERN_SIZE) {
                    truncated = true;
                    break;
                }
                baked.x.insert(baked.x.end(), frame.x.begin(), frame.x.end());
                baked.y.insert(baked.y.end(), frame.y.begin(), frame.y.end());
                framesDone++;

                m_shape3D.rotationX += m_shape3D.rotationSpeedX;
                m_shape3D.rotationY += m_shape3D.rotationSpeedY;
                m_shape3D.rotationZ += m_shape3D.rotationSpeedZ;
            }

            // Restore so baking is repeatable from the same starting pose
            m_shape3D.rotationX = rx0;
            m_shape3D.rotationY = ry0;
            m_shape3D.rotationZ = rz0;

            if (baked.empty()) {
                setStatus("Nothing to bake - shape produced no points", true);
            } else {
                // The baked result replaces the live pattern; stop the
                // animation from overwriting it on the next frame.
                claimPatternSource();
                app.getPattern() = baked;
                app.getAudioEngine().setPattern(app.getPattern());

                char msg[160];
                if (truncated) {
                    snprintf(msg, sizeof(msg),
                             "Baked %d of %d frames (%zu points - buffer full)",
                             framesDone, m_shape3D.bakeFrames, baked.size());
                } else {
                    snprintf(msg, sizeof(msg), "Baked %d frames (%zu points)",
                             framesDone, baked.size());
                }
                setStatus(msg, truncated);
            }
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Freezes the rotation into one long pattern so \nWAV export and playback actually rotate.");
        }
        ImGui::TreePop();
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

    if (m_shape3D.shapeType == Shape3DState::ShapeType::ObjModel) {
        ImGui::Separator();
        ImGui::Text("OBJ Model");

        if (ImGui::Button("Browse...", ImVec2(100, 0))) {
            auto selection = pfd::open_file(
                "Load OBJ Model",
                "",
                { "Wavefront OBJ", "*.obj", "All Files", "*" }
            ).result();

            if (!selection.empty()) {
                loadObjModel(app, selection[0]);
            }
        }

        if (!m_shape3D.objPath.empty()) {
            ImGui::SameLine();
            if (ImGui::Button("Reload", ImVec2(-1, 0))) {
                loadObjModel(app, m_shape3D.objPath);
            }
        }

        if (m_shape3D.objMesh.empty()) {
            ImGui::TextDisabled("No model loaded");
        } else {
            ImGui::TextWrapped("%s", m_shape3D.objMesh.name.c_str());
        }

        if (!m_shape3D.objStatus.empty()) {
            ImGui::TextDisabled("%s", m_shape3D.objStatus.c_str());
        }

        // Dense meshes trace slowly; the cap keeps playback responsive.
        ImGui::SliderInt("Max Edges", &m_shape3D.objMaxEdges, 100, 20000);
        // Apply on release only - re-reading the file every drag frame would
        // stall on large models. Re-loading from disk (rather than thinning the
        // current mesh again) keeps decimation based on the full edge list.
        if (ImGui::IsItemDeactivatedAfterEdit() && !m_shape3D.objPath.empty()) {
            loadObjModel(app, m_shape3D.objPath);
        }
        ImGui::TextDisabled("Model is auto-centered and scaled to fit");
    }

    ImGui::Separator();

    if (rotChanged) {
        m_3dShapeActive = true;  // Re-enable 3D shape as active source
        generate3DShapePattern(app);
    }

    if (ImGui::Button("Generate Shape", ImVec2(-1, 40))) {
        m_3dShapeActive = true;  // Re-enable 3D shape as active source
        generate3DShapePattern(app);
    }

    ImGui::End();
}

//==============================================================================
// Sound Pad and Display Settings (unchanged)
//==============================================================================

// Build the traced path from the active pad steps: the beam walks from each
// active step to the next and wraps back to the first.
void UIManager::generateSoundPadPattern(App& app) {
    std::vector<int> activeIndices;
    for (int i = 0; i < SoundPadState::NUM_STEPS; ++i) {
        if (m_soundPad.active[i]) activeIndices.push_back(i);
    }
    if (activeIndices.empty()) return;

    // Claim the pattern before writing it, or the rotating 3D shape (on by
    // default) overwrites it on the very next frame and the pad looks dead.
    claimPatternSource();

    Pattern& pattern = app.getPattern();
    pattern.clear();

    constexpr int pointsPerStep = 100;
    if (activeIndices.size() == 1) {
        // A single step is a point; hold it so there is something to see.
        int only = activeIndices[0];
        for (int p = 0; p < pointsPerStep; ++p) {
            pattern.push_back(m_soundPad.x[only], m_soundPad.y[only]);
        }
    } else {
        for (size_t s = 0; s < activeIndices.size(); ++s) {
            int curr = activeIndices[s];
            int next = activeIndices[(s + 1) % activeIndices.size()];
            for (int p = 0; p < pointsPerStep; ++p) {
                float t = static_cast<float>(p) / pointsPerStep;
                pattern.x.push_back(m_soundPad.x[curr] * (1 - t) + m_soundPad.x[next] * t);
                pattern.y.push_back(m_soundPad.y[curr] * (1 - t) + m_soundPad.y[next] * t);
            }
        }
    }

    app.getAudioEngine().setPattern(pattern);
}

void UIManager::renderSoundPad(App& app) {
    // Position: Floating, left side below Controls
    placeWindow(0.006f, 0.62f, 0.20f, 0.36f);
    ImGui::Begin("Sound Pad", &m_showSoundPad);
    ImGui::Text("16-Step XY Sequencer");
    ImGui::TextDisabled("Click a cell to switch its step on/off.");
    ImGui::TextDisabled("Drag a cell to move its point.");
    ImGui::Separator();

    bool padChanged = false;
    // Snapshot taken lazily: only the first edit of a drag records history, so
    // dragging a cell produces one undo entry rather than one per frame.
    bool padSnapshotTaken = false;
    auto snapshotPad = [&]() {
        if (!padSnapshotTaken) {
            m_padUndo.push(m_soundPad);
            padSnapshotTaken = true;
        }
    };

    float buttonSize = 60.0f;
    for (int row = 0; row < SoundPadState::GRID_SIZE; ++row) {
        for (int col = 0; col < SoundPadState::GRID_SIZE; ++col) {
            int idx = row * SoundPadState::GRID_SIZE + col;
            if (col > 0) ImGui::SameLine();

            if (m_soundPad.active[idx]) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.6f, 0.3f, 1.0f));
            else ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.3f, 0.3f, 0.3f, 1.0f));

            char label[32];
            snprintf(label, sizeof(label), "%d\n%.1f,%.1f", idx + 1, m_soundPad.x[idx], m_soundPad.y[idx]);
            if (ImGui::Button(label, ImVec2(buttonSize, buttonSize))) {
                snapshotPad();
                m_soundPad.active[idx] = !m_soundPad.active[idx];
                padChanged = true;
            }
            ImGui::PopStyleColor();

            if (ImGui::IsItemActive() && ImGui::IsMouseDragging(0)) {
                if (ImGui::IsMouseClicked(0) || !m_padDragging) {
                    snapshotPad();
                    m_padDragging = true;
                }
                ImVec2 delta = ImGui::GetMouseDragDelta(0);
                m_soundPad.x[idx] = std::clamp(m_soundPad.x[idx] + delta.x * 0.01f, -1.0f, 1.0f);
                m_soundPad.y[idx] = std::clamp(m_soundPad.y[idx] - delta.y * 0.01f, -1.0f, 1.0f);
                ImGui::ResetMouseDragDelta(0);
                padChanged = true;
            }
        }
    }

    if (!ImGui::IsMouseDown(0)) m_padDragging = false;

    ImGui::Separator();

    // Undo / Redo
    {
        float w = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
        if (!m_padUndo.canUndo()) ImGui::BeginDisabled();
        if (ImGui::Button("Undo", ImVec2(w, 0))) {
            if (const SoundPadState* prev = m_padUndo.undo(m_soundPad)) {
                m_soundPad = *prev;
                padChanged = true;
            }
        }
        if (!m_padUndo.canUndo()) ImGui::EndDisabled();
        ImGui::SameLine();
        if (!m_padUndo.canRedo()) ImGui::BeginDisabled();
        if (ImGui::Button("Redo", ImVec2(w, 0))) {
            if (const SoundPadState* next = m_padUndo.redo(m_soundPad)) {
                m_soundPad = *next;
                padChanged = true;
            }
        }
        if (!m_padUndo.canRedo()) ImGui::EndDisabled();

        if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) &&
            ImGui::GetIO().KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Z, false)) {
            if (const SoundPadState* prev = m_padUndo.undo(m_soundPad)) {
                m_soundPad = *prev;
                padChanged = true;
            }
        }
    }

    ImGui::Separator();
    if (ImGui::Button("Circle Preset", ImVec2(-1, 0))) {
        snapshotPad();
        m_soundPad.resetToCircle();
        padChanged = true;
    }
    if (ImGui::Button("Activate All", ImVec2(-1, 0))) {
        snapshotPad();
        for (int i = 0; i < SoundPadState::NUM_STEPS; ++i) m_soundPad.active[i] = true;
        padChanged = true;
    }
    if (ImGui::Button("Grid Layout", ImVec2(-1, 0))) {
        snapshotPad();
        m_soundPad.gridLayout();
        padChanged = true;
    }
    if (ImGui::Button("Clear All", ImVec2(-1, 0))) {
        // Deactivate only - the point positions stay put, so switching cells
        // back on still draws something.
        snapshotPad();
        for (int i = 0; i < SoundPadState::NUM_STEPS; ++i) m_soundPad.active[i] = false;
        padChanged = true;
    }

    int activeCount = 0;
    for (int i = 0; i < SoundPadState::NUM_STEPS; ++i) if (m_soundPad.active[i]) activeCount++;
    if (activeCount == 0) {
        ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "No steps active");
    } else {
        ImGui::TextDisabled("%d step%s active", activeCount, activeCount == 1 ? "" : "s");
    }

    // Live feedback: edits apply straight to the scope, no button press needed.
    if (padChanged) generateSoundPadPattern(app);

    ImGui::Separator();
    if (ImGui::Button("Generate Pattern", ImVec2(-1, 30))) {
        generateSoundPadPattern(app);
    }
    ImGui::End();
}

void UIManager::renderDisplaySettings(App& app) {
    (void)app;
    // Position: Floating, right of oscilloscope
    placeWindow(0.44f, 0.10f, 0.25f, 0.70f);
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
        ImGui::SliderFloat("Vignette", &m_phosphor.vignetteStrength, 0.0f, 1.0f);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Darken the edges of the tube");

        ImGui::SliderFloat("Scanlines", &m_phosphor.scanlineEffect, 0.0f, 1.0f);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Horizontal raster banding");

        ImGui::SliderFloat("Analog Noise", &m_phosphor.noiseAmount, 0.0f, 0.1f);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Sparse phosphor speckle");

        ImGui::TextDisabled("Screen curvature needs a shader pass - not implemented");
    }

    //==========================================================================
    // DISPLAY BUFFER
    //==========================================================================
    ImGui::Separator();
    ImGui::SliderInt("Trail Samples", &m_phosphor.trailSamples, 1024, 16384);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("More samples = longer visible trail");

    ImGui::End();
}

//==============================================================================
// Image Vectorizer Panel
//==============================================================================
void UIManager::renderImageVectorizer(App& app) {
    placeWindow(0.10f, 0.05f, 0.68f, 0.86f);
    ImGui::Begin("Image Vectorizer", &m_showImageVectorizer);

    // Local state for file path input
    static char pathBuffer[512] = "";
    static std::string statusMessage = "Ready";
    static std::vector<float> previewX, previewY;

    // Left panel - Controls
    ImGui::BeginChild("VectorizerControls", ImVec2(380, 0), true);

    ImGui::Text("IMAGE VECTORIZER");
    ImGui::Separator();
    ImGui::Spacing();

    //==========================================================================
    // FILE LOADING
    //==========================================================================
    ImGui::Text("Load Image");
    ImGui::Separator();

    if (ImGui::Button("Browse...", ImVec2(100, 0))) {
        auto selection = pfd::open_file(
            "Select Image",
            "",
            { "Image Files", "*.png *.jpg *.jpeg *.bmp *.gif *.tga",
              "All Files", "*" }
        ).result();

        if (!selection.empty()) {
            strncpy(pathBuffer, selection[0].c_str(), sizeof(pathBuffer) - 1);
            pathBuffer[sizeof(pathBuffer) - 1] = '\0';
        }
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(-1);
    ImGui::InputText("##VecPath", pathBuffer, sizeof(pathBuffer));

    bool canLoad = strlen(pathBuffer) > 0;
    if (!canLoad) ImGui::BeginDisabled();
    if (ImGui::Button("Load Image", ImVec2(-1, 30))) {
        if (m_vectorizer.loadImage(pathBuffer)) {
            m_vectorizerImagePath = pathBuffer;
            m_vectorizerImageLoaded = true;
            statusMessage = "Loaded: " + std::to_string(m_vectorizer.getWidth()) +
                           "x" + std::to_string(m_vectorizer.getHeight());
        } else {
            statusMessage = "Failed to load image!";
            m_vectorizerImageLoaded = false;
        }
    }
    if (!canLoad) ImGui::EndDisabled();

    if (m_vectorizerImageLoaded) {
        ImGui::Text("Size: %dx%d", m_vectorizer.getWidth(), m_vectorizer.getHeight());
    }

    ImGui::Spacing();
    ImGui::Spacing();

    //==========================================================================
    // IMAGE TYPE GUIDE
    //==========================================================================
    if (ImGui::CollapsingHeader("What type of image?", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.8f, 0.8f, 0.5f, 1.0f));
        ImGui::TextWrapped("Choose the mode that best matches your image:");
        ImGui::PopStyleColor();
        ImGui::Spacing();

        // Photo modes
        ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "PHOTOS:");
        if (ImGui::Button("General Photo", ImVec2(-1, 0))) {
            m_vectorizerParams.mode = VectorizerParams::Mode::PhotoGeneral;
            m_vectorizerParams.resetToDefaults();
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Landscapes, objects, general photography\nUses Canny edge detection");

        if (ImGui::Button("Portrait / Face", ImVec2(-1, 0))) {
            m_vectorizerParams.mode = VectorizerParams::Mode::PhotoPortrait;
            m_vectorizerParams.resetToDefaults();
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("People, faces, portraits\nBilateral filter smooths skin while preserving features");

        if (ImGui::Button("High Detail", ImVec2(-1, 0))) {
            m_vectorizerParams.mode = VectorizerParams::Mode::PhotoHighDetail;
            m_vectorizerParams.resetToDefaults();
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Architecture, textures, detailed scenes\nCaptures maximum edge detail");

        ImGui::Spacing();

        // People detection modes
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.4f, 1.0f), "PEOPLE (Smart Detection):");
        if (ImGui::Button("Face Focus", ImVec2(-1, 0))) {
            m_vectorizerParams.mode = VectorizerParams::Mode::PeopleFace;
            m_vectorizerParams.resetToDefaults();
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Detect face and emphasize facial features\nSkin detection + face region estimation");

        if (ImGui::Button("Headshot / Bust", ImVec2(-1, 0))) {
            m_vectorizerParams.mode = VectorizerParams::Mode::PeopleHeadshot;
            m_vectorizerParams.resetToDefaults();
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Head and shoulders portrait\nFace + upper body with smooth transitions");

        if (ImGui::Button("Full Body", ImVec2(-1, 0))) {
            m_vectorizerParams.mode = VectorizerParams::Mode::PeopleFullBody;
            m_vectorizerParams.resetToDefaults();
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Complete figure with face detail\nBody silhouette + facial feature emphasis");

        if (ImGui::Button("Artistic Portrait", ImVec2(-1, 0))) {
            m_vectorizerParams.mode = VectorizerParams::Mode::PeopleArtistic;
            m_vectorizerParams.resetToDefaults();
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Stylized line portrait\nDoG edge detection + artistic simplification");

        ImGui::Spacing();

        // Artwork modes
        ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.8f, 1.0f), "ARTWORK:");
        if (ImGui::Button("Cartoon / Anime", ImVec2(-1, 0))) {
            m_vectorizerParams.mode = VectorizerParams::Mode::Cartoon;
            m_vectorizerParams.resetToDefaults();
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Cartoons, anime, cel-shaded art\nColor quantization + edge tracing");

        if (ImGui::Button("Line Art / Sketch", ImVec2(-1, 0))) {
            m_vectorizerParams.mode = VectorizerParams::Mode::LineArt;
            m_vectorizerParams.resetToDefaults();
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Pencil drawings, ink sketches, line work\nDifference of Gaussians for line detection");

        if (ImGui::Button("Pixel Art", ImVec2(-1, 0))) {
            m_vectorizerParams.mode = VectorizerParams::Mode::PixelArt;
            m_vectorizerParams.resetToDefaults();
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Pixel art, sprites, retro graphics\nPreserves sharp pixel edges");

        ImGui::Spacing();

        // Graphics modes
        ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.5f, 1.0f), "GRAPHICS:");
        if (ImGui::Button("Logo / Icon", ImVec2(-1, 0))) {
            m_vectorizerParams.mode = VectorizerParams::Mode::LogoIcon;
            m_vectorizerParams.resetToDefaults();
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Logos, icons, high-contrast graphics\nSimple binary threshold");

        if (ImGui::Button("Document / Text", ImVec2(-1, 0))) {
            m_vectorizerParams.mode = VectorizerParams::Mode::Document;
            m_vectorizerParams.resetToDefaults();
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Scanned documents, printed text\nAdaptive threshold for uneven lighting");

        if (ImGui::Button("Silhouette", ImVec2(-1, 0))) {
            m_vectorizerParams.mode = VectorizerParams::Mode::Silhouette;
            m_vectorizerParams.resetToDefaults();
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Solid shapes, shadows, outlines\nHigh contrast + morphological cleanup");

        ImGui::Spacing();
        ImGui::Separator();
    }

    //==========================================================================
    // CURRENT MODE INFO
    //==========================================================================
    {
        const char* modeNames[] = {
            "Photo - General", "Photo - Portrait", "Photo - High Detail",
            "People - Face Focus", "People - Headshot", "People - Full Body", "People - Artistic",
            "Cartoon / Anime", "Line Art / Sketch", "Pixel Art",
            "Logo / Icon", "Document / Text", "Silhouette"
        };
        int mode = static_cast<int>(m_vectorizerParams.mode);
        ImGui::Text("Current: %s", modeNames[mode]);

        ImGui::Checkbox("Invert Image", &m_vectorizerParams.invert);

        ImGui::Spacing();
        if (ImGui::Button("Reset to Defaults", ImVec2(-1, 0))) {
            m_vectorizerParams.resetToDefaults();
        }
    }

    //==========================================================================
    // PREPROCESSING
    //==========================================================================
    if (ImGui::CollapsingHeader("Preprocessing")) {
        ImGui::SliderFloat("Blur", &m_vectorizerParams.blurRadius, 0.0f, 5.0f, "%.1f");
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Reduce noise before edge detection.");

        ImGui::SliderFloat("Brightness", &m_vectorizerParams.brightness, -1.0f, 1.0f, "%.2f");
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Adjust overall image brightness.");

        ImGui::SliderFloat("Contrast", &m_vectorizerParams.contrast, 0.5f, 2.0f, "%.2f");
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Increase contrast for clearer edges.");
    }

    //==========================================================================
    // MODE-SPECIFIC PARAMETERS
    //==========================================================================
    if (ImGui::CollapsingHeader("Detection Parameters")) {
        switch (m_vectorizerParams.mode) {
            case VectorizerParams::Mode::PhotoGeneral:
            case VectorizerParams::Mode::PhotoHighDetail:
                ImGui::SliderFloat("Low Threshold", &m_vectorizerParams.cannyLow, 10.0f, 150.0f, "%.0f");
                ImGui::SliderFloat("High Threshold", &m_vectorizerParams.cannyHigh, 30.0f, 300.0f, "%.0f");
                if (m_vectorizerParams.cannyLow >= m_vectorizerParams.cannyHigh) {
                    m_vectorizerParams.cannyLow = m_vectorizerParams.cannyHigh * 0.33f;
                }
                break;

            case VectorizerParams::Mode::PhotoPortrait:
                ImGui::Text("Bilateral Filter:");
                ImGui::SliderInt("Filter Size", &m_vectorizerParams.bilateralD, 5, 15);
                ImGui::SliderFloat("Color Sigma", &m_vectorizerParams.bilateralSigmaColor, 20.0f, 150.0f, "%.0f");
                ImGui::SliderFloat("Space Sigma", &m_vectorizerParams.bilateralSigmaSpace, 20.0f, 150.0f, "%.0f");
                ImGui::Separator();
                ImGui::Text("Edge Detection:");
                ImGui::SliderFloat("Low Threshold", &m_vectorizerParams.cannyLow, 10.0f, 150.0f, "%.0f");
                ImGui::SliderFloat("High Threshold", &m_vectorizerParams.cannyHigh, 30.0f, 300.0f, "%.0f");
                break;

            case VectorizerParams::Mode::PeopleFace:
            case VectorizerParams::Mode::PeopleHeadshot:
            case VectorizerParams::Mode::PeopleFullBody:
                ImGui::Text("Face Detection:");
                ImGui::SliderFloat("Skin Sensitivity", &m_vectorizerParams.skinSensitivity, 0.5f, 2.0f, "%.1f");
                ImGui::SliderFloat("Min Face Size", &m_vectorizerParams.minFaceRatio, 0.02f, 0.2f, "%.2f");
                ImGui::Checkbox("Detect Multiple Faces", &m_vectorizerParams.detectMultipleFaces);
                ImGui::Separator();
                ImGui::Text("Regional Processing:");
                ImGui::SliderFloat("Face Emphasis", &m_vectorizerParams.faceEmphasis, 1.0f, 3.0f, "%.1f");
                ImGui::SliderFloat("Background Simplify", &m_vectorizerParams.backgroundSimplify, 1.0f, 5.0f, "%.1f");
                ImGui::SliderFloat("Detail Emphasis", &m_vectorizerParams.detailEmphasis, 0.0f, 1.0f, "%.2f");
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Keeps more edges in regions the importance mask \nmarks as interesting (skin, detected detail). 0 = off.");
                }
                ImGui::Separator();
                ImGui::SliderFloat("Low Threshold", &m_vectorizerParams.cannyLow, 10.0f, 150.0f, "%.0f");
                ImGui::SliderFloat("High Threshold", &m_vectorizerParams.cannyHigh, 30.0f, 300.0f, "%.0f");
                break;

            case VectorizerParams::Mode::PeopleArtistic:
                ImGui::Text("Face Detection:");
                ImGui::SliderFloat("Skin Sensitivity", &m_vectorizerParams.skinSensitivity, 0.5f, 2.0f, "%.1f");
                ImGui::SliderFloat("Face Emphasis", &m_vectorizerParams.faceEmphasis, 1.5f, 4.0f, "%.1f");
                ImGui::Separator();
                ImGui::Text("Line Art (DoG):");
                ImGui::SliderFloat("Fine Detail", &m_vectorizerParams.dogSigma1, 0.5f, 2.0f, "%.1f");
                ImGui::SliderFloat("Coarse Detail", &m_vectorizerParams.dogSigma2, 1.0f, 4.0f, "%.1f");
                if (m_vectorizerParams.dogSigma2 <= m_vectorizerParams.dogSigma1) {
                    m_vectorizerParams.dogSigma2 = m_vectorizerParams.dogSigma1 * 2.0f;
                }
                ImGui::SliderFloat("Line Threshold", &m_vectorizerParams.lineThreshold, 5.0f, 40.0f, "%.0f");
                break;

            case VectorizerParams::Mode::Cartoon:
                ImGui::SliderInt("Color Levels", &m_vectorizerParams.colorLevels, 2, 16);
                ImGui::SliderFloat("Edge Threshold", &m_vectorizerParams.threshold, 10.0f, 100.0f, "%.0f");
                break;

            case VectorizerParams::Mode::LineArt:
                ImGui::SliderFloat("Fine Detail (Sigma 1)", &m_vectorizerParams.dogSigma1, 0.5f, 3.0f, "%.1f");
                ImGui::SliderFloat("Coarse Detail (Sigma 2)", &m_vectorizerParams.dogSigma2, 1.0f, 6.0f, "%.1f");
                if (m_vectorizerParams.dogSigma2 <= m_vectorizerParams.dogSigma1) {
                    m_vectorizerParams.dogSigma2 = m_vectorizerParams.dogSigma1 * 2.0f;
                }
                ImGui::SliderFloat("Line Threshold", &m_vectorizerParams.lineThreshold, 5.0f, 50.0f, "%.0f");
                break;

            case VectorizerParams::Mode::PixelArt:
            case VectorizerParams::Mode::LogoIcon:
            case VectorizerParams::Mode::Silhouette:
                ImGui::SliderFloat("Threshold", &m_vectorizerParams.threshold, 0.0f, 255.0f, "%.0f");
                break;

            case VectorizerParams::Mode::Document:
                ImGui::SliderInt("Block Size", &m_vectorizerParams.adaptiveBlockSize, 3, 51);
                if (m_vectorizerParams.adaptiveBlockSize % 2 == 0) m_vectorizerParams.adaptiveBlockSize++;
                ImGui::SliderFloat("Constant C", &m_vectorizerParams.adaptiveC, -10.0f, 15.0f, "%.1f");
                break;
        }
    }

    //==========================================================================
    // CONTOUR PROCESSING
    //==========================================================================
    if (ImGui::CollapsingHeader("Contour Processing")) {
        ImGui::SliderInt("Min Length", &m_vectorizerParams.minContourLength, 3, 100);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Minimum points per contour.");

        ImGui::SliderFloat("Simplify", &m_vectorizerParams.simplifyEpsilon, 0.5f, 10.0f, "%.1f");
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Douglas-Peucker simplification.");

        ImGui::SliderFloat("Connect Dist", &m_vectorizerParams.connectDistance, 0.0f, 20.0f, "%.1f");
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Max gap to bridge between contours.");

        ImGui::Checkbox("Close Contours", &m_vectorizerParams.closedContours);
    }

    //==========================================================================
    // OUTPUT
    //==========================================================================
    if (ImGui::CollapsingHeader("Output")) {
        ImGui::SliderInt("Max Points", &m_vectorizerParams.maxOutputPoints, 1000, 20000);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Maximum points in output pattern.");
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    //==========================================================================
    // PROCESS BUTTON
    //==========================================================================
    bool canProcess = m_vectorizerImageLoaded;

    // Re-runs detection and refreshes the preview. Shared by the button and
    // the live path below.
    auto runVectorizer = [&]() {
        m_vectorizer.process(m_vectorizerParams);
        previewX.clear();
        previewY.clear();
        m_vectorizer.generatePattern(previewX, previewY, m_vectorizerParams);
        statusMessage = "Done! " + std::to_string(previewX.size()) + " points generated";
    };

    ImGui::Checkbox("Live preview", &m_vectorizerLive);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Re-process automatically when a setting changes.\nRuns on release, not while dragging - detection is expensive.");
    }

    if (!canProcess) ImGui::BeginDisabled();
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.6f, 0.9f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.7f, 1.0f, 1.0f));
    if (ImGui::Button("Process Image", ImVec2(-1, 40))) {
        statusMessage = "Processing...";
        runVectorizer();
        // Keep the live watcher in step so it does not immediately re-run.
        std::memcpy(&m_vectorizerLastParams, &m_vectorizerParams,
                    sizeof(VectorizerParams));
        m_vectorizerHasLast = true;
    }
    ImGui::PopStyleColor(2);
    if (!canProcess) ImGui::EndDisabled();

    // Live path: compare the whole parameter block byte-for-byte rather than
    // instrumenting several dozen individual widgets. The snapshot is taken
    // with memcpy so padding matches and the comparison stays reliable.
    // Deferred until no widget is active, so a slider drag processes once on
    // release instead of on every frame.
    if (m_vectorizerLive && m_vectorizerImageLoaded && !ImGui::IsAnyItemActive()) {
        if (!m_vectorizerHasLast ||
            std::memcmp(&m_vectorizerLastParams, &m_vectorizerParams,
                        sizeof(VectorizerParams)) != 0) {
            runVectorizer();
            std::memcpy(&m_vectorizerLastParams, &m_vectorizerParams,
                        sizeof(VectorizerParams));
            m_vectorizerHasLast = true;
        }
    }

    ImGui::Spacing();

    //==========================================================================
    // APPLY TO OSCILLOSCOPE BUTTON
    //==========================================================================
    bool canApply = !previewX.empty();
    if (!canApply) ImGui::BeginDisabled();
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.7f, 0.3f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.8f, 0.4f, 1.0f));
    if (ImGui::Button("Apply to Oscilloscope", ImVec2(-1, 35))) {
        // Copy pattern to app
        Pattern& pattern = app.getPattern();
        pattern.clear();
        pattern.reserve(previewX.size());
        for (size_t i = 0; i < previewX.size(); ++i) {
            pattern.push_back(previewX[i], previewY[i]);
        }
        app.getAudioEngine().setPattern(pattern);
        claimPatternSource();  // Stop 3D/generator sources overwriting it
        statusMessage = "Pattern applied to oscilloscope!";
    }
    ImGui::PopStyleColor(2);
    if (!canApply) ImGui::EndDisabled();

    ImGui::Spacing();
    ImGui::Separator();

    // Status
    ImGui::TextWrapped("Status: %s", statusMessage.c_str());

    // Stats
    if (!previewX.empty()) {
        ImGui::Text("Preview: %zu points", previewX.size());
        ImGui::Text("Contours: %zu", m_vectorizer.getContours().size());
    }

    ImGui::EndChild();

    ImGui::SameLine();

    // Right panel - Preview
    ImGui::BeginChild("VectorizerPreview", ImVec2(0, 0), true);

    ImGui::Text("Preview");
    ImGui::Separator();

    // Draw the pattern preview using ImPlot
    float previewSize = ImGui::GetContentRegionAvail().x - 20;
    if (previewSize < 200) previewSize = 200;

    if (ImPlot::BeginPlot("##VecPatternPreview", ImVec2(previewSize, previewSize),
                          ImPlotFlags_Equal | ImPlotFlags_NoLegend)) {
        ImPlot::SetupAxes("X", "Y", ImPlotAxisFlags_NoTickLabels, ImPlotAxisFlags_NoTickLabels);
        ImPlot::SetupAxisLimits(ImAxis_X1, -1.1, 1.1, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -1.1, 1.1, ImGuiCond_Always);

        // Draw pattern
        if (!previewX.empty()) {
            ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.4f, 1.0f, 0.3f, 1.0f));
            ImPlot::PlotLine("Pattern", previewX.data(), previewY.data(),
                            static_cast<int>(previewX.size()));
            ImPlot::PopStyleColor();
        }

        ImPlot::EndPlot();
    }

    ImGui::Spacing();
    ImGui::TextWrapped("Tip: After processing, click 'Apply to Oscilloscope' to use the pattern.");

    ImGui::EndChild();

    ImGui::End();
}

} // namespace oscilloplot

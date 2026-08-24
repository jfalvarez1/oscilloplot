#pragma once

#include <vector>
#include <array>
#include <cmath>
#include <string>
#include "vectorizer/image_vectorizer.hpp"
#include "data/obj_loader.hpp"
#include "data/pattern.hpp"
#include "data/preset.hpp"
#include "data/recent_files.hpp"
#include "data/sequence.hpp"
#include "utils/undo_stack.hpp"

// ImGui's vector type, forward-declared so this header does not have to pull
// in all of imgui.h for one function signature.
struct ImVec2;

namespace oscilloplot {

class App;
struct Pattern;

//==============================================================================
// Phosphor Display Settings - Physically-Based CRT Simulation
//==============================================================================
struct PhosphorSettings {
    //--------------------------------------------------------------------------
    // Phosphor Decay (Temporal)
    // P31 phosphor: technical decay 0.01-1ms, visible persistence ~30-100ms
    //--------------------------------------------------------------------------
    float decayTime = 50.0f;            // Persistence time constant in ms (10-200)
    float decayExponent = 2.0f;         // Decay curve shape (1=linear, 2=exponential)

    //--------------------------------------------------------------------------
    // Beam Intensity
    // On real CRT: Brightness ∝ 1/velocity (slower = brighter)
    //--------------------------------------------------------------------------
    float brightness = 1.2f;            // Overall intensity multiplier (0.5-3.0)
    float velocityEffect = 0.7f;        // How much velocity affects brightness (0-1)
    float minBrightness = 0.15f;        // Minimum brightness for fast traces
    float maxBrightness = 1.0f;         // Maximum brightness (prevents burn-in look)

    //--------------------------------------------------------------------------
    // Glow/Bloom (Spatial)
    // Real phosphor has exponential light scatter, not Gaussian
    //--------------------------------------------------------------------------
    float glowIntensity = 0.5f;         // Glow/bloom amount (0-1)
    float glowRadius = 3.0f;            // Glow spread in pixels (1-8)
    float glowFalloff = 1.5f;           // Exponential falloff rate (1-3)

    //--------------------------------------------------------------------------
    // Beam Characteristics
    //--------------------------------------------------------------------------
    float beamWidth = 1.2f;             // Core beam thickness (0.5-3.0)
    float beamFocus = 0.8f;             // Beam focus/sharpness (0.5-1.0, 1=sharp)
    float beamSpotSize = 2.0f;          // Additional spot bloom at slow points

    //--------------------------------------------------------------------------
    // Display Settings
    //--------------------------------------------------------------------------
    int trailSamples = 8192;            // Number of samples to display (1024-16384)
    bool showGrid = true;               // Show graticule
    float gridAlpha = 0.12f;            // Graticule line opacity
    bool showGraticuleMarks = true;     // Show 0.2 div tick marks

    //--------------------------------------------------------------------------
    // Phosphor Color - P31 default (Tektronix 465B)
    // P31: ZnS:Cu - yellowish-green, peak ~525nm
    // RGB: 89, 255, 64
    //--------------------------------------------------------------------------
    float colorR = 89.0f / 255.0f;      // 0.349 - P31 has slight yellow tint
    float colorG = 255.0f / 255.0f;     // 1.0
    float colorB = 64.0f / 255.0f;      // 0.251

    //--------------------------------------------------------------------------
    // CRT Simulation
    //--------------------------------------------------------------------------
    float screenCurvature = 0.0f;       // CRT screen curvature simulation (0-0.1)
    float vignetteStrength = 0.15f;     // Edge darkening (0-1)
    float noiseAmount = 0.0f;           // Analog noise speckle (0-0.1)
    float scanlineEffect = 0.0f;        // Raster scanline banding (0-1)
};

//==============================================================================
// Sound Pad State (16-step sequencer)
//==============================================================================
struct SoundPadState {
    static constexpr int GRID_SIZE = 4;
    static constexpr int NUM_STEPS = GRID_SIZE * GRID_SIZE;

    // Grid layout: each cell's point sits at the cell's own place in the
    // grid, so the pad reads as a spatial XY surface. (The startup default is
    // the circle below; an all-zero layout would collapse every step onto the
    // origin and draw an invisible single dot.)
    static constexpr float cellX(int idx) {
        return -0.8f + 1.6f * static_cast<float>(idx % GRID_SIZE) / (GRID_SIZE - 1);
    }
    static constexpr float cellY(int idx) {
        return 0.8f - 1.6f * static_cast<float>(idx / GRID_SIZE) / (GRID_SIZE - 1);
    }

    std::array<float, NUM_STEPS> x{};   // X position for each step (-1 to 1)
    std::array<float, NUM_STEPS> y{};   // Y position for each step (-1 to 1)
    std::array<bool, NUM_STEPS> active{}; // Which steps are active
    int currentStep = 0;

    SoundPadState() { resetToCircle(); }

    // Points laid out on the grid, matching the button positions.
    void gridLayout() {
        for (int i = 0; i < NUM_STEPS; ++i) { x[i] = cellX(i); y[i] = cellY(i); }
    }

    // Startup default: a ready-to-play circle with every step enabled.
    void resetToCircle() {
        constexpr float kTwoPi = 6.28318530718f;
        for (int i = 0; i < NUM_STEPS; ++i) {
            float a = kTwoPi * static_cast<float>(i) / NUM_STEPS;
            x[i] = std::cos(a) * 0.8f;
            y[i] = std::sin(a) * 0.8f;
            active[i] = true;
        }
    }
};

//==============================================================================
// Sum of Harmonics State - Enhanced with Phase & Frequency Sweep
//==============================================================================
struct HarmonicTerm {
    // Basic parameters
    float amplitude = 1.0f;
    float frequency = 1.0f;
    float phase = 0.0f;
    bool enabled = true;
    bool useSin = true;         // true = sin, false = cos

    // Phase sweep (animate phase over time)
    bool phaseSweep = false;
    float phaseStart = 0.0f;
    float phaseEnd = 6.28318f;  // 2*PI
    int phaseSweepSteps = 20;

    // Frequency sweep (animate frequency over time)
    bool freqSweep = false;
    float freqStart = 1.0f;
    float freqEnd = 5.0f;
    int freqSweepSteps = 20;
};

struct HarmonicsState {
    static constexpr int MAX_HARMONICS = 8;
    std::array<HarmonicTerm, MAX_HARMONICS> xTerms;
    std::array<HarmonicTerm, MAX_HARMONICS> yTerms;
    int numXTerms = 1;
    int numYTerms = 1;
    int numPoints = 1000;
    bool liveUpdate = true;  // Auto-update pattern when values change
};

//==============================================================================
// Drawing Canvas State
//==============================================================================
struct DrawingState {
    std::vector<float> pointsX;
    std::vector<float> pointsY;
    std::vector<size_t> strokeStarts;  // Indices where each stroke begins (for proper rendering)
    bool isDrawing = false;
    bool newStrokeStarted = false;     // Flag to mark beginning of a new stroke
    float smoothing = 0.3f;
    float lastX = 0.0f;
    float lastY = 0.0f;
};

//==============================================================================
// 3D Shape Projection State
//==============================================================================
struct Shape3DState {
    enum class ShapeType {
        Cube = 0,
        Tetrahedron,
        Octahedron,
        Icosahedron,
        Dodecahedron,
        Sphere,
        Torus,
        Cylinder,
        Cone,
        Pyramid,
        Prism,
        Spiral3D,
        Helix,
        Knot,
        MobiusStrip,
        KleinBottle,
        Spring,
        Star3D,
        Text3D,
        ObjModel
    };

    ShapeType shapeType = ShapeType::Torus;  // Default to rotating donut
    float rotationX = 0.0f;         // Current X rotation (degrees)
    float rotationY = 0.0f;         // Current Y rotation (degrees)
    float rotationZ = 0.0f;         // Current Z rotation (degrees)
    float rotationSpeedX = 0.5f;    // Animation speed (deg/frame)
    float rotationSpeedY = 1.0f;
    float rotationSpeedZ = 0.0f;
    float scale = 0.8f;
    float perspective = 2.0f;       // Distance for perspective (0 = orthographic)
    int resolution = 500;           // Points for curved shapes
    bool animate = true;            // Start with animation enabled

    // Torus specific
    float torusMajorRadius = 0.7f;
    float torusMinorRadius = 0.3f;

    // Spiral/Knot specific
    float knotP = 2.0f;  // Trefoil knot parameters
    float knotQ = 3.0f;

    // Helix specific
    float helixTurns = 5.0f;
    float helixRadius = 0.5f;

    // Prism specific
    int prismSides = 6;

    // Star 3D specific
    int starPoints = 5;
    float starInnerRadius = 0.3f;

    // Text 3D specific
    char textBuffer[64] = "HELLO";
    float textDepth = 0.3f;
    bool textConnectFaces = true;

    // Bake animation
    int bakeFrames = 120;           // Rotation frames baked into one pattern

    // OBJ model specific
    ObjMesh objMesh;                // Loaded wireframe; empty until a file loads
    std::string objPath;            // Path of the loaded model
    std::string objStatus;          // Last load result, shown in the panel
    int objMaxEdges = 2000;         // Cap on traced edges (dense meshes are slow)
    size_t objSourceEdges = 0;      // Edge count before decimation
};

//==============================================================================
// Sequencer State - chains patterns over time.
// The step list and its file format live in data/sequence.hpp; this holds only
// the transient playback position.
//==============================================================================
struct SequencerState {
    Sequence seq;                   // Steps, loop flag and crossfade amount
    bool playing = false;           // Sequence playback active
    int currentStep = -1;           // Index while playing, -1 otherwise
    uint32_t cyclesAtStepStart = 0; // Engine cycle count when this step began
    Pattern blendScratch;           // Reused so crossfading allocates nothing
};

//==============================================================================
// UI Manager
//==============================================================================
class UIManager {
public:
    UIManager();
    ~UIManager();

    bool init();
    void shutdown();

    void render(App& app);

private:
    void renderMenuBar(App& app);
    void renderControlPanel(App& app);
    void renderOscilloscopeDisplay(App& app);
    void renderEffectsPanel(App& app);
    void renderGeneratorsPanel(App& app);
    void renderSoundPad(App& app);
    void renderHarmonicsEditor(App& app);
    void renderDrawingCanvas(App& app);
    void renderDisplaySettings(App& app);
    void render3DShapeGenerator(App& app);
    void renderImageVectorizer(App& app);

    // Phosphor display rendering
    void renderPhosphorScope(App& app);

    // CRT post-effects drawn over the trace (vignette, scanlines, noise)
    void renderCrtOverlays(const ImVec2& plotPos, const ImVec2& plotSize);

    // Helper: Generate pattern from harmonics
    void generateHarmonicsPattern(App& app);

    // Helper: Generate 3D shape pattern
    void generate3DShapePattern(App& app);

    // Helper: Build one frame of the current 3D shape (no engine interaction)
    void buildShapeFrame(Pattern& pattern);

    // Helper: Load a Wavefront .obj model into the 3D shape generator
    void loadObjModel(App& app, const std::string& path);

    // File menu actions
    void doLoadPattern(App& app);
    void doSavePattern(App& app);
    void doExportWav(App& app);
    void doExportImage(App& app);

    // Load a pattern file by path (shared by the dialog and the recent list)
    bool loadPatternFile(App& app, const std::string& path);

    // Presets: pattern + effect settings together
    void doSavePreset(App& app);
    void doLoadPreset(App& app);
    void loadPresetFile(App& app, const std::string& path);

    // Recent files, persisted beside imgui.ini
    RecentFiles m_recentFiles;
    void loadRecentFiles();
    void saveRecentFiles() const;
    static std::string recentFilesPath();

    // Undo history for the editors that mutate a list in place
    UndoStack<DrawingState> m_canvasUndo;
    UndoStack<SoundPadState> m_padUndo;
    UndoStack<Sequence> m_sequencerUndo;
    void loadSequenceFile(const std::string& path);

    // Visual style and default layout
    void applyStyle();
    void placeWindow(float x, float y, float w, float h);  // viewport fractions
    int m_layoutResetFrames = 0;   // >0: re-apply default layout this frame

    // Which generator's parameters are live-editable. Lives on the object (not
    // as a function static) so every other pattern source can clear it - the
    // Points slider would otherwise regenerate over a loaded/drawn/captured
    // pattern the next time it moved.
    enum class ActiveGen {
        None, Circle, Ellipse, Sine, Lissajous, Star, Flower, Rose,
        ArchSpiral, LogSpiral, Helix, Trefoil, TorusKnot,
        Butterfly, Cardioid, Deltoid, Hypotrochoid, Epitrochoid,
        Figure8, Infinity, Heart, Square, Sawtooth, Triangle
    };
    ActiveGen m_activeGen = ActiveGen::None;

    // Call from any non-generator pattern source before it installs a pattern.
    void claimPatternSource() {
        m_activeGen = ActiveGen::None;
        m_3dShapeActive = false;
        m_shape3D.animate = false;
    }

    // Helper: build the pattern from the active sound pad steps
    void generateSoundPadPattern(App& app);

    // Sequencer
    void renderSequencer(App& app);
    void updateSequencer(App& app);     // Per-frame step advancement
    void sequencerStart(App& app);
    void sequencerStop(App& app);
    void sequencerApplyStep(App& app, int index);
    void sequencerSave();
    void sequencerLoad();
    void sequencerExportWav(App& app);

    // Transient status line shown at the right of the menu bar
    void setStatus(const std::string& message, bool isError = false);
    std::string m_statusMessage;
    bool m_statusIsError = false;
    double m_statusTime = 0.0;   // ImGui time when the message was set

    // Panel visibility
    bool m_showAbout = false;
    bool m_showDemoWindow = false;
    bool m_showEffectsPanel = true;
    bool m_showGeneratorsPanel = true;
    bool m_showSoundPad = false;
    bool m_showHarmonicsEditor = false;
    bool m_showDrawingCanvas = false;
    bool m_showDisplaySettings = false;
    bool m_show3DShapeGenerator = false;

    // Phosphor display
    PhosphorSettings m_phosphor;
    int m_exportImageSize = 1024;   // Square edge length for Export Image

    // Effects master bypass. The snapshot holds the settings that were live
    // when bypass was switched on, so turning it off restores them.
    bool m_effectsBypassed = false;
    Preset m_bypassSnapshot;

    // True while a sound pad cell is being dragged, so the whole drag makes
    // a single undo entry instead of one per frame.
    bool m_padDragging = false;

    // Run the effect chain on the preview shown while playback is stopped,
    // so effects can be tuned without pressing Play.
    bool m_previewEffects = true;

    // Bake destination: 0 = single long pattern, 1 = one sequencer step per
    // frame (no frame cap, and the result stays editable).
    int m_bakeToSequencer = 0;
    int m_bakeCyclesPerFrame = 4;

    // Visualization buffers
    static constexpr size_t VIZ_SAMPLES = 16384;
    float m_vizX[VIZ_SAMPLES];
    float m_vizY[VIZ_SAMPLES];
    float m_vizIntensity[VIZ_SAMPLES];  // Per-sample intensity (velocity-based)

    // Frame timing for decay calculation
    float m_lastFrameTime = 0.0f;
    uint32_t m_frameCount = 0;

    // Sound Pad state
    SoundPadState m_soundPad;

    // Harmonics editor state
    HarmonicsState m_harmonics;

    // Drawing canvas state
    DrawingState m_drawing;

    // Sequencer state
    SequencerState m_sequencer;
    bool m_showSequencer = false;

    // 3D shape generator state
    Shape3DState m_shape3D;
    bool m_3dShapeActive = true;  // Track if 3D shape is the active pattern source

    // First render flag for initial pattern generation
    bool m_firstRender = true;

    // Image vectorizer state
    bool m_showImageVectorizer = false;
    ImageVectorizer m_vectorizer;
    VectorizerParams m_vectorizerParams;
    bool m_vectorizerImageLoaded = false;
    std::string m_vectorizerImagePath;

    // Live preview: re-process when a setting changes and no widget is being
    // dragged. The byte snapshot avoids instrumenting every control.
    bool m_vectorizerLive = false;
    bool m_vectorizerHasLast = false;
    VectorizerParams m_vectorizerLastParams;
};

} // namespace oscilloplot

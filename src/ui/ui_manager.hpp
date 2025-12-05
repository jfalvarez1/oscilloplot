#pragma once

#include <vector>
#include <array>

namespace oscilloplot {

class App;

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
    //--------------------------------------------------------------------------
    float colorR = 0.35f;               // P31 has slight yellow tint
    float colorG = 1.0f;
    float colorB = 0.25f;

    //--------------------------------------------------------------------------
    // CRT Simulation
    //--------------------------------------------------------------------------
    float screenCurvature = 0.0f;       // CRT screen curvature simulation (0-0.1)
    float vignetteStrength = 0.1f;      // Edge darkening (0-0.3)
    float noiseAmount = 0.02f;          // Analog noise simulation (0-0.1)
    float scanlineEffect = 0.0f;        // Raster scanline simulation (0-0.5)
};

//==============================================================================
// Sound Pad State (16-step sequencer)
//==============================================================================
struct SoundPadState {
    static constexpr int GRID_SIZE = 4;
    static constexpr int NUM_STEPS = GRID_SIZE * GRID_SIZE;

    std::array<float, NUM_STEPS> x{};   // X position for each step (-1 to 1)
    std::array<float, NUM_STEPS> y{};   // Y position for each step (-1 to 1)
    std::array<bool, NUM_STEPS> active{}; // Which steps are active
    int currentStep = 0;
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
    bool isDrawing = false;
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
        Sphere,
        Torus,
        Cylinder,
        Cone,
        Spiral3D,
        Knot
    };

    ShapeType shapeType = ShapeType::Cube;
    float rotationX = 0.0f;         // Current X rotation (degrees)
    float rotationY = 0.0f;         // Current Y rotation (degrees)
    float rotationZ = 0.0f;         // Current Z rotation (degrees)
    float rotationSpeedX = 0.0f;    // Animation speed (deg/frame)
    float rotationSpeedY = 1.0f;
    float rotationSpeedZ = 0.0f;
    float scale = 0.8f;
    float perspective = 2.0f;       // Distance for perspective (0 = orthographic)
    int resolution = 500;           // Points for curved shapes
    bool animate = false;

    // Torus specific
    float torusMajorRadius = 0.7f;
    float torusMinorRadius = 0.3f;

    // Spiral/Knot specific
    float knotP = 2.0f;  // Trefoil knot parameters
    float knotQ = 3.0f;
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

    // Phosphor display rendering
    void renderPhosphorScope(App& app);

    // Helper: Generate pattern from harmonics
    void generateHarmonicsPattern(App& app);

    // Helper: Generate 3D shape pattern
    void generate3DShapePattern(App& app);

    // Panel visibility
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

    // 3D shape generator state
    Shape3DState m_shape3D;
};

} // namespace oscilloplot

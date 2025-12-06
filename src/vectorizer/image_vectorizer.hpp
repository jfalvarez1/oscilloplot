#pragma once

#include <vector>
#include <string>
#include <cstdint>

namespace oscilloplot {

//==============================================================================
// Vectorizer Parameters
//==============================================================================
struct VectorizerParams {
    // Preprocessing
    float blurRadius = 1.0f;          // Gaussian blur sigma (0-5)
    float brightness = 0.0f;          // Brightness adjustment (-1 to 1)
    float contrast = 1.0f;            // Contrast multiplier (0.5-2.0)
    bool invert = false;              // Invert image before processing

    //==========================================================================
    // Detection Modes - Optimized for different image types
    //==========================================================================
    enum class Mode {
        // === PHOTOS ===
        PhotoGeneral,       // General photos - Canny edge detection
        PhotoPortrait,      // Portraits/faces - Bilateral filter + Canny
        PhotoHighDetail,    // Architecture, landscapes - Multi-scale edges

        // === PEOPLE (Smart Detection) ===
        PeopleFace,         // Face detection - emphasize facial features
        PeopleHeadshot,     // Head + shoulders - portrait bust style
        PeopleFullBody,     // Full body silhouette with face detail
        PeopleArtistic,     // Artistic line portrait - stylized features

        // === ARTWORK ===
        Cartoon,            // Cartoons/anime - Color quantization + contour
        LineArt,            // Sketches/drawings - Difference of Gaussians
        PixelArt,           // Pixel art/sprites - Sharp edge preservation

        // === GRAPHICS ===
        LogoIcon,           // Logos/icons - Binary threshold
        Document,           // Scanned docs - Adaptive threshold
        Silhouette          // Solid shapes - High contrast extraction
    };
    Mode mode = Mode::PhotoGeneral;

    // Canny parameters (for photo modes)
    float cannyLow = 30.0f;           // Low threshold (10-200)
    float cannyHigh = 100.0f;         // High threshold (50-400)

    // Threshold parameters
    float threshold = 128.0f;         // Binary threshold (0-255)
    int adaptiveBlockSize = 15;       // Adaptive threshold block size (odd, 3-51)
    float adaptiveC = 5.0f;           // Adaptive threshold constant

    // Cartoon mode parameters
    int colorLevels = 6;              // Number of color levels for quantization (2-16)
    float edgeStrength = 1.0f;        // Edge line darkness (0.5-2.0)

    // Line art parameters (Difference of Gaussians)
    float dogSigma1 = 1.0f;           // First Gaussian sigma
    float dogSigma2 = 2.0f;           // Second Gaussian sigma (should be > sigma1)
    float lineThreshold = 15.0f;      // Threshold for line detection

    // Bilateral filter parameters (for portraits)
    int bilateralD = 9;               // Filter diameter
    float bilateralSigmaColor = 75.0f;  // Color sigma
    float bilateralSigmaSpace = 75.0f;  // Spatial sigma

    // People detection parameters
    float skinSensitivity = 1.0f;     // Skin detection sensitivity (0.5-2.0)
    float faceEmphasis = 1.5f;        // How much to emphasize face region (1.0-3.0)
    float backgroundSimplify = 2.0f;  // Background simplification factor (1.0-5.0)
    bool detectMultipleFaces = true;  // Detect all faces or just the largest
    float minFaceRatio = 0.05f;       // Minimum face size as ratio of image (0.02-0.2)

    // Contour processing
    int minContourLength = 8;         // Minimum points in contour
    float simplifyEpsilon = 1.0f;     // Douglas-Peucker tolerance (0.5-10)
    float connectDistance = 3.0f;     // Max distance to connect paths

    // Output
    int maxOutputPoints = 10000;      // Maximum points in final pattern
    bool closedContours = true;       // Close contours that are nearly closed
    float closeThreshold = 5.0f;      // Distance threshold for closing

    //==========================================================================
    // Preset Functions - Research-based optimal defaults
    //==========================================================================

    void resetToDefaults() {
        switch (mode) {
            case Mode::PhotoGeneral:    applyPhotoGeneralPreset(); break;
            case Mode::PhotoPortrait:   applyPhotoPortraitPreset(); break;
            case Mode::PhotoHighDetail: applyPhotoHighDetailPreset(); break;
            case Mode::PeopleFace:      applyPeopleFacePreset(); break;
            case Mode::PeopleHeadshot:  applyPeopleHeadshotPreset(); break;
            case Mode::PeopleFullBody:  applyPeopleFullBodyPreset(); break;
            case Mode::PeopleArtistic:  applyPeopleArtisticPreset(); break;
            case Mode::Cartoon:         applyCartoonPreset(); break;
            case Mode::LineArt:         applyLineArtPreset(); break;
            case Mode::PixelArt:        applyPixelArtPreset(); break;
            case Mode::LogoIcon:        applyLogoIconPreset(); break;
            case Mode::Document:        applyDocumentPreset(); break;
            case Mode::Silhouette:      applySilhouettePreset(); break;
        }
    }

    //--------------------------------------------------------------------------
    // PHOTO PRESETS
    //--------------------------------------------------------------------------

    // General photos - Standard Canny with balanced parameters
    // Algorithm: Gaussian blur → Sobel gradients → Non-max suppression → Hysteresis
    void applyPhotoGeneralPreset() {
        blurRadius = 1.0f;
        brightness = 0.0f;
        contrast = 1.0f;
        invert = false;
        cannyLow = 30.0f;             // 1:3 ratio (Canny 1986 recommendation)
        cannyHigh = 100.0f;
        minContourLength = 8;
        simplifyEpsilon = 1.0f;
        connectDistance = 3.0f;
        maxOutputPoints = 10000;
        closedContours = true;
        closeThreshold = 5.0f;
    }

    // Portraits/faces - Bilateral filter preserves skin while detecting features
    // Algorithm: Bilateral filter → Canny (higher thresholds for major features only)
    // Research: Tomasi & Manduchi 1998 - bilateral filtering for edge-preserving smoothing
    void applyPhotoPortraitPreset() {
        blurRadius = 0.5f;            // Light Gaussian after bilateral
        brightness = 0.0f;
        contrast = 1.1f;
        invert = false;
        bilateralD = 9;               // Filter size
        bilateralSigmaColor = 75.0f;  // Preserve edges with similar colors
        bilateralSigmaSpace = 75.0f;  // Spatial smoothing radius
        cannyLow = 40.0f;             // Higher thresholds - major features only
        cannyHigh = 120.0f;
        minContourLength = 15;        // Filter small noise
        simplifyEpsilon = 1.5f;       // Smoother curves for faces
        connectDistance = 5.0f;
        maxOutputPoints = 8000;
        closedContours = true;
        closeThreshold = 8.0f;
    }

    // High detail photos - Multi-scale edge detection
    // Algorithm: Lower thresholds + finer simplification for maximum detail
    void applyPhotoHighDetailPreset() {
        blurRadius = 0.5f;            // Minimal blur
        brightness = 0.0f;
        contrast = 1.2f;
        invert = false;
        cannyLow = 20.0f;             // Lower thresholds = more edges
        cannyHigh = 60.0f;
        minContourLength = 5;
        simplifyEpsilon = 0.5f;       // Less simplification = more detail
        connectDistance = 2.0f;
        maxOutputPoints = 15000;      // More points for detail
        closedContours = true;
        closeThreshold = 3.0f;
    }

    //--------------------------------------------------------------------------
    // PEOPLE DETECTION PRESETS
    // Research: Skin detection using YCbCr color space (Chai & Ngan 1999)
    // Face proportions based on anthropometric studies
    //--------------------------------------------------------------------------

    // Face Focus - Detect face and emphasize facial features
    // Algorithm: Skin detection → Face region estimation → Enhanced edge detection on face
    void applyPeopleFacePreset() {
        blurRadius = 0.5f;
        brightness = 0.0f;
        contrast = 1.1f;
        invert = false;
        bilateralD = 7;               // Moderate smoothing
        bilateralSigmaColor = 50.0f;
        bilateralSigmaSpace = 50.0f;
        cannyLow = 25.0f;             // Sensitive for facial details
        cannyHigh = 80.0f;
        skinSensitivity = 1.0f;
        faceEmphasis = 2.0f;          // Strong face emphasis
        backgroundSimplify = 3.0f;    // Simplify background
        detectMultipleFaces = false;  // Focus on main face
        minFaceRatio = 0.08f;         // Expect reasonably sized face
        minContourLength = 5;
        simplifyEpsilon = 0.6f;       // Preserve face detail
        connectDistance = 2.0f;
        maxOutputPoints = 10000;
        closedContours = true;
        closeThreshold = 4.0f;
    }

    // Headshot/Bust - Head and shoulders portrait
    // Algorithm: Face detection + body proportion estimation + portrait framing
    void applyPeopleHeadshotPreset() {
        blurRadius = 0.8f;
        brightness = 0.0f;
        contrast = 1.0f;
        invert = false;
        bilateralD = 9;
        bilateralSigmaColor = 75.0f;
        bilateralSigmaSpace = 75.0f;
        cannyLow = 30.0f;
        cannyHigh = 100.0f;
        skinSensitivity = 1.0f;
        faceEmphasis = 1.5f;
        backgroundSimplify = 2.5f;
        detectMultipleFaces = false;
        minFaceRatio = 0.1f;          // Larger face expected in headshot
        minContourLength = 8;
        simplifyEpsilon = 1.0f;
        connectDistance = 3.0f;
        maxOutputPoints = 8000;
        closedContours = true;
        closeThreshold = 5.0f;
    }

    // Full Body - Complete figure with face detail
    // Algorithm: Skin detection for body regions + face emphasis + silhouette
    void applyPeopleFullBodyPreset() {
        blurRadius = 1.0f;
        brightness = 0.0f;
        contrast = 1.2f;
        invert = false;
        bilateralD = 5;               // Less smoothing for body detail
        bilateralSigmaColor = 40.0f;
        bilateralSigmaSpace = 40.0f;
        cannyLow = 35.0f;
        cannyHigh = 110.0f;
        skinSensitivity = 1.2f;       // More sensitive for distant skin
        faceEmphasis = 1.3f;          // Moderate face emphasis
        backgroundSimplify = 2.0f;
        detectMultipleFaces = true;   // May have multiple people
        minFaceRatio = 0.03f;         // Smaller faces in full body shots
        minContourLength = 10;
        simplifyEpsilon = 1.5f;       // More simplification for body
        connectDistance = 4.0f;
        maxOutputPoints = 12000;
        closedContours = true;
        closeThreshold = 6.0f;
    }

    // Artistic Line Portrait - Stylized feature extraction
    // Algorithm: Face detection + feature enhancement + artistic line simplification
    // Inspired by: Winnemöller et al. "XDoG: An eXtended Difference-of-Gaussians"
    void applyPeopleArtisticPreset() {
        blurRadius = 0.0f;            // DoG handles blurring
        brightness = 0.1f;
        contrast = 1.4f;
        invert = false;
        dogSigma1 = 0.8f;             // Fine lines for features
        dogSigma2 = 1.6f;
        lineThreshold = 12.0f;
        skinSensitivity = 0.8f;       // Less sensitive - artistic interpretation
        faceEmphasis = 2.5f;          // Strong artistic emphasis on face
        backgroundSimplify = 4.0f;    // Heavy background simplification
        detectMultipleFaces = false;
        minFaceRatio = 0.06f;
        minContourLength = 6;
        simplifyEpsilon = 0.8f;
        connectDistance = 2.0f;
        maxOutputPoints = 8000;
        closedContours = false;       // Artistic lines don't need closing
        closeThreshold = 5.0f;
    }

    //--------------------------------------------------------------------------
    // ARTWORK PRESETS
    //--------------------------------------------------------------------------

    // Cartoon/Anime - Color quantization + edge detection
    // Algorithm: Median filter → Color quantization → Edge detection on color boundaries
    // Research: Winnemoller 2006 - "Real-Time Video Abstraction"
    void applyCartoonPreset() {
        blurRadius = 0.0f;            // No Gaussian - use median filter instead
        brightness = 0.0f;
        contrast = 1.0f;
        invert = false;
        colorLevels = 6;              // Reduce to 6 colors
        edgeStrength = 1.0f;
        threshold = 20.0f;            // Low threshold for color boundaries
        minContourLength = 10;
        simplifyEpsilon = 0.8f;       // Preserve cartoon shapes
        connectDistance = 2.0f;
        maxOutputPoints = 10000;
        closedContours = true;
        closeThreshold = 4.0f;
    }

    // Line art/Sketches - Difference of Gaussians (DoG)
    // Algorithm: DoG approximates Laplacian of Gaussian, excellent for line detection
    // Research: Marr & Hildreth 1980 - "Theory of Edge Detection"
    void applyLineArtPreset() {
        blurRadius = 0.0f;            // DoG handles its own blurring
        brightness = 0.1f;            // Slight brightness boost
        contrast = 1.3f;              // Increase contrast for faint lines
        invert = false;
        dogSigma1 = 1.0f;             // Inner Gaussian
        dogSigma2 = 2.0f;             // Outer Gaussian (2x is standard)
        lineThreshold = 15.0f;        // Threshold for line detection
        minContourLength = 5;
        simplifyEpsilon = 0.6f;       // Preserve line detail
        connectDistance = 3.0f;
        maxOutputPoints = 12000;
        closedContours = false;       // Lines often don't close
        closeThreshold = 5.0f;
    }

    // Pixel art - No blur, preserve sharp pixel edges
    // Algorithm: Direct edge detection without anti-aliasing
    void applyPixelArtPreset() {
        blurRadius = 0.0f;            // NO blur - preserve pixels
        brightness = 0.0f;
        contrast = 1.5f;              // High contrast
        invert = false;
        threshold = 128.0f;
        minContourLength = 4;         // Small contours are valid in pixel art
        simplifyEpsilon = 0.3f;       // Minimal simplification
        connectDistance = 1.5f;       // Tight connections
        maxOutputPoints = 8000;
        closedContours = true;
        closeThreshold = 2.0f;
    }

    //--------------------------------------------------------------------------
    // GRAPHICS PRESETS
    //--------------------------------------------------------------------------

    // Logos/Icons - Binary threshold for high-contrast graphics
    void applyLogoIconPreset() {
        blurRadius = 0.5f;
        brightness = 0.0f;
        contrast = 1.2f;
        invert = false;
        threshold = 128.0f;
        minContourLength = 5;
        simplifyEpsilon = 0.8f;
        connectDistance = 2.0f;
        maxOutputPoints = 8000;
        closedContours = true;
        closeThreshold = 3.0f;
    }

    // Documents - Adaptive threshold for uneven lighting
    // Algorithm: Local mean thresholding with offset
    // Research: Sauvola & Pietaksinen 2000 - "Adaptive document binarization"
    void applyDocumentPreset() {
        blurRadius = 0.8f;
        brightness = 0.0f;
        contrast = 1.1f;
        invert = false;
        adaptiveBlockSize = 15;
        adaptiveC = 5.0f;
        minContourLength = 10;
        simplifyEpsilon = 1.2f;
        connectDistance = 4.0f;
        maxOutputPoints = 10000;
        closedContours = true;
        closeThreshold = 5.0f;
    }

    // Silhouette - Extract solid shapes with high contrast
    // Algorithm: Aggressive threshold + morphological cleanup
    void applySilhouettePreset() {
        blurRadius = 1.5f;            // Smooth out noise
        brightness = 0.0f;
        contrast = 1.5f;              // High contrast
        invert = false;
        threshold = 100.0f;           // Lower threshold - capture more of the shape
        minContourLength = 20;        // Only major contours
        simplifyEpsilon = 2.0f;       // Smooth silhouette
        connectDistance = 5.0f;
        maxOutputPoints = 5000;       // Fewer points for smooth shapes
        closedContours = true;
        closeThreshold = 10.0f;
    }
};

//==============================================================================
// Contour Structure
//==============================================================================
struct Contour {
    std::vector<float> x;
    std::vector<float> y;
    bool closed = false;

    size_t size() const { return x.size(); }
    bool empty() const { return x.empty(); }

    void addPoint(float px, float py) {
        x.push_back(px);
        y.push_back(py);
    }

    void clear() {
        x.clear();
        y.clear();
        closed = false;
    }
};

//==============================================================================
// Image Vectorizer
//==============================================================================
class ImageVectorizer {
public:
    ImageVectorizer();
    ~ImageVectorizer();

    // Load image from file
    bool loadImage(const std::string& path);

    // Load image from memory
    bool loadImageFromMemory(const uint8_t* data, int width, int height, int channels);

    // Check if image is loaded
    bool hasImage() const { return m_imageData != nullptr; }

    // Get image dimensions
    int getWidth() const { return m_width; }
    int getHeight() const { return m_height; }

    // Get raw image data (for texture upload)
    const uint8_t* getImageData() const { return m_imageData; }

    // Get edge detection result (grayscale, for preview)
    const uint8_t* getEdgeData() const { return m_edgeData.data(); }

    // Process image with current parameters
    void process(const VectorizerParams& params);

    // Get extracted contours
    const std::vector<Contour>& getContours() const { return m_contours; }

    // Generate final XY pattern (normalized to -1..1)
    void generatePattern(std::vector<float>& outX, std::vector<float>& outY,
                        const VectorizerParams& params);

    // Free loaded image
    void unloadImage();

private:
    // Image data
    uint8_t* m_imageData = nullptr;
    int m_width = 0;
    int m_height = 0;
    int m_channels = 0;

    // Processing buffers
    std::vector<uint8_t> m_grayscale;
    std::vector<uint8_t> m_blurred;
    std::vector<float> m_gradientMag;
    std::vector<float> m_gradientDir;
    std::vector<uint8_t> m_edgeData;

    // Extracted contours
    std::vector<Contour> m_contours;

    // Processing steps - Basic
    void convertToGrayscale();
    void applyBrightnessContrast(float brightness, float contrast);
    void gaussianBlur(float sigma);

    // Edge detection methods
    void sobelGradient();
    void nonMaxSuppression();
    void hysteresisThreshold(float low, float high);
    void simpleThreshold(float thresh, bool invert);
    void adaptiveThreshold(int blockSize, float C, bool invert);

    // Advanced filters
    void bilateralFilter(int d, float sigmaColor, float sigmaSpace);
    void medianFilter(int radius);
    void differenceOfGaussians(float sigma1, float sigma2, float threshold);
    void colorQuantize(int levels);
    void morphologicalClean(int iterations);  // Erosion + dilation for cleanup

    // People/Face detection (no external ML required)
    // Research: YCbCr skin detection (Chai & Ngan 1999, Phung et al. 2005)
    struct FaceRegion {
        int x, y, width, height;      // Bounding box
        float confidence;             // Detection confidence (0-1)
        int centerX, centerY;         // Center point
    };
    std::vector<FaceRegion> m_detectedFaces;

    void detectSkinRegions(float sensitivity);           // Find skin-colored pixels
    void findFaceRegions(float minRatio, bool multiple); // Estimate face bounding boxes
    void createFaceMask();                               // Create mask for face emphasis
    void applyRegionalProcessing(const VectorizerParams& params);  // Different processing per region

    std::vector<uint8_t> m_skinMask;      // Skin detection result
    std::vector<uint8_t> m_faceMask;      // Face region mask (for emphasis)
    std::vector<uint8_t> m_importanceMask; // Per-pixel importance (face=high, bg=low)

    // Contour extraction
    void traceContours(int minLength);
    void traceContour(int startX, int startY, std::vector<bool>& visited);

    // Path optimization
    void simplifyContour(Contour& contour, float epsilon);
    void douglasPeucker(const Contour& input, Contour& output,
                        size_t start, size_t end, float epsilon,
                        std::vector<bool>& keep);
    void orderContours();
    void connectNearbyContours(float maxDistance);
};

} // namespace oscilloplot

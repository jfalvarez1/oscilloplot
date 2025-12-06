#define STB_IMAGE_IMPLEMENTATION
#include "image_vectorizer.hpp"
#include <stb_image.h>
#include <cmath>
#include <algorithm>
#include <queue>
#include <cstring>
#include <thread>
#include <vector>

// Get number of hardware threads
static int getNumThreads() {
    int n = static_cast<int>(std::thread::hardware_concurrency());
    return (n > 0) ? n : 4;
}

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace oscilloplot {

//==============================================================================
// Embedded-Style Optimizations
//==============================================================================

// Fixed-point math for grayscale (16.16 format)
// 0.299 * 65536 = 19595, 0.587 * 65536 = 38470, 0.114 * 65536 = 7471
static constexpr int32_t FP_R = 19595;
static constexpr int32_t FP_G = 38470;
static constexpr int32_t FP_B = 7471;

// Pre-computed atan2 lookup table for gradient direction (256x256 = 64KB)
// Direction quantized to 0-3 (horizontal, diagonal up, vertical, diagonal down)
static uint8_t s_atan2LUT[256][256];
static bool s_lutInitialized = false;

static void initLUT() {
    if (s_lutInitialized) return;
    for (int gy = 0; gy < 256; ++gy) {
        for (int gx = 0; gx < 256; ++gx) {
            // Map signed gradient to unsigned (128 = 0)
            float dx = static_cast<float>(gx) - 128.0f;
            float dy = static_cast<float>(gy) - 128.0f;
            float angle = std::atan2(dy, dx) * 180.0f / static_cast<float>(M_PI);
            if (angle < 0) angle += 180.0f;

            // Quantize to 4 directions
            if ((angle >= 0 && angle < 22.5f) || (angle >= 157.5f && angle <= 180.0f)) {
                s_atan2LUT[gy][gx] = 0; // Horizontal
            } else if (angle >= 22.5f && angle < 67.5f) {
                s_atan2LUT[gy][gx] = 1; // Diagonal /
            } else if (angle >= 67.5f && angle < 112.5f) {
                s_atan2LUT[gy][gx] = 2; // Vertical
            } else {
                s_atan2LUT[gy][gx] = 3; // Diagonal (backslash)
            }
        }
    }
    s_lutInitialized = true;
}

//==============================================================================
// Constructor / Destructor
//==============================================================================

ImageVectorizer::ImageVectorizer() {
    initLUT();
}

ImageVectorizer::~ImageVectorizer() {
    unloadImage();
}

//==============================================================================
// Image Loading
//==============================================================================

bool ImageVectorizer::loadImage(const std::string& path) {
    unloadImage();

    // Load with stb_image
    m_imageData = stbi_load(path.c_str(), &m_width, &m_height, &m_channels, 0);
    if (!m_imageData) {
        return false;
    }

    // Pre-allocate processing buffers (avoid reallocation)
    size_t pixelCount = static_cast<size_t>(m_width) * m_height;
    m_grayscale.resize(pixelCount);
    m_blurred.resize(pixelCount);
    m_gradientMag.resize(pixelCount);
    m_gradientDir.resize(pixelCount);
    m_edgeData.resize(pixelCount);

    return true;
}

bool ImageVectorizer::loadImageFromMemory(const uint8_t* data, int width, int height, int channels) {
    unloadImage();

    m_width = width;
    m_height = height;
    m_channels = channels;

    size_t dataSize = static_cast<size_t>(width) * height * channels;
    m_imageData = static_cast<uint8_t*>(malloc(dataSize));
    if (!m_imageData) return false;

    memcpy(m_imageData, data, dataSize);

    size_t pixelCount = static_cast<size_t>(m_width) * m_height;
    m_grayscale.resize(pixelCount);
    m_blurred.resize(pixelCount);
    m_gradientMag.resize(pixelCount);
    m_gradientDir.resize(pixelCount);
    m_edgeData.resize(pixelCount);

    return true;
}

void ImageVectorizer::unloadImage() {
    if (m_imageData) {
        stbi_image_free(m_imageData);
        m_imageData = nullptr;
    }
    m_width = 0;
    m_height = 0;
    m_channels = 0;
    m_grayscale.clear();
    m_blurred.clear();
    m_gradientMag.clear();
    m_gradientDir.clear();
    m_edgeData.clear();
    m_contours.clear();
}

//==============================================================================
// Image Processing Pipeline - Mode-specific processing
//==============================================================================

void ImageVectorizer::process(const VectorizerParams& params) {
    if (!m_imageData) return;

    // Step 1: Convert to grayscale
    convertToGrayscale();

    // Step 2: Apply brightness/contrast if needed
    if (params.brightness != 0.0f || params.contrast != 1.0f) {
        applyBrightnessContrast(params.brightness, params.contrast);
    }

    // Step 3: Mode-specific edge detection
    switch (params.mode) {
        //======================================================================
        // PHOTO MODES
        //======================================================================
        case VectorizerParams::Mode::PhotoGeneral:
            // Standard Canny edge detection
            if (params.blurRadius > 0.1f) {
                gaussianBlur(params.blurRadius);
            } else {
                m_blurred = m_grayscale;
            }
            sobelGradient();
            nonMaxSuppression();
            hysteresisThreshold(params.cannyLow, params.cannyHigh);
            break;

        case VectorizerParams::Mode::PhotoPortrait:
            // Bilateral filter preserves edges while smoothing skin
            bilateralFilter(params.bilateralD, params.bilateralSigmaColor, params.bilateralSigmaSpace);
            if (params.blurRadius > 0.1f) {
                gaussianBlur(params.blurRadius);
            }
            sobelGradient();
            nonMaxSuppression();
            hysteresisThreshold(params.cannyLow, params.cannyHigh);
            break;

        case VectorizerParams::Mode::PhotoHighDetail:
            // Minimal blur, lower thresholds for maximum detail
            if (params.blurRadius > 0.1f) {
                gaussianBlur(params.blurRadius);
            } else {
                m_blurred = m_grayscale;
            }
            sobelGradient();
            nonMaxSuppression();
            hysteresisThreshold(params.cannyLow, params.cannyHigh);
            break;

        //======================================================================
        // PEOPLE DETECTION MODES
        //======================================================================
        case VectorizerParams::Mode::PeopleFace:
            // Face-focused processing with smart skin/face detection
            detectSkinRegions(params.skinSensitivity);
            findFaceRegions(params.minFaceRatio, params.detectMultipleFaces);
            createFaceMask();
            bilateralFilter(params.bilateralD, params.bilateralSigmaColor, params.bilateralSigmaSpace);
            if (params.blurRadius > 0.1f) {
                gaussianBlur(params.blurRadius);
            }
            applyRegionalProcessing(params);
            break;

        case VectorizerParams::Mode::PeopleHeadshot:
            // Head + shoulders portrait processing
            detectSkinRegions(params.skinSensitivity);
            findFaceRegions(params.minFaceRatio, params.detectMultipleFaces);
            createFaceMask();
            bilateralFilter(params.bilateralD, params.bilateralSigmaColor, params.bilateralSigmaSpace);
            if (params.blurRadius > 0.1f) {
                gaussianBlur(params.blurRadius);
            }
            applyRegionalProcessing(params);
            break;

        case VectorizerParams::Mode::PeopleFullBody:
            // Full body with face emphasis
            detectSkinRegions(params.skinSensitivity);
            findFaceRegions(params.minFaceRatio, params.detectMultipleFaces);
            createFaceMask();
            if (params.bilateralD > 0) {
                bilateralFilter(params.bilateralD, params.bilateralSigmaColor, params.bilateralSigmaSpace);
            }
            if (params.blurRadius > 0.1f) {
                gaussianBlur(params.blurRadius);
            }
            applyRegionalProcessing(params);
            break;

        case VectorizerParams::Mode::PeopleArtistic:
            // Artistic line portrait using DoG
            detectSkinRegions(params.skinSensitivity);
            findFaceRegions(params.minFaceRatio, params.detectMultipleFaces);
            createFaceMask();
            differenceOfGaussians(params.dogSigma1, params.dogSigma2, params.lineThreshold);
            // Apply face emphasis by adjusting edge strength in face regions
            if (!m_detectedFaces.empty()) {
                for (size_t i = 0; i < m_edgeData.size(); ++i) {
                    if (m_faceMask[i] > 128) {
                        // Boost edges in face region
                        if (m_edgeData[i] > 0) {
                            m_edgeData[i] = 255;
                        }
                    } else if (m_importanceMask[i] < 64) {
                        // Reduce edges in background
                        m_edgeData[i] = (m_edgeData[i] > 128) ? 255 : 0;
                    }
                }
            }
            break;

        //======================================================================
        // ARTWORK MODES
        //======================================================================
        case VectorizerParams::Mode::Cartoon:
            // Color quantization + edge detection on color boundaries
            colorQuantize(params.colorLevels);
            medianFilter(2);  // Clean up quantization artifacts
            m_blurred = m_grayscale;
            sobelGradient();
            nonMaxSuppression();
            // Use threshold parameter for edge detection
            hysteresisThreshold(params.threshold * 0.5f, params.threshold);
            break;

        case VectorizerParams::Mode::LineArt:
            // Difference of Gaussians - excellent for line detection
            differenceOfGaussians(params.dogSigma1, params.dogSigma2, params.lineThreshold);
            break;

        case VectorizerParams::Mode::PixelArt:
            // No blur - preserve sharp pixel edges
            m_blurred = m_grayscale;
            simpleThreshold(params.threshold, params.invert);
            break;

        //======================================================================
        // GRAPHICS MODES
        //======================================================================
        case VectorizerParams::Mode::LogoIcon:
            // Simple binary threshold for high-contrast graphics
            if (params.blurRadius > 0.1f) {
                gaussianBlur(params.blurRadius);
            } else {
                m_blurred = m_grayscale;
            }
            simpleThreshold(params.threshold, params.invert);
            break;

        case VectorizerParams::Mode::Document:
            // Adaptive threshold for uneven lighting
            if (params.blurRadius > 0.1f) {
                gaussianBlur(params.blurRadius);
            } else {
                m_blurred = m_grayscale;
            }
            adaptiveThreshold(params.adaptiveBlockSize, params.adaptiveC, params.invert);
            break;

        case VectorizerParams::Mode::Silhouette:
            // High contrast threshold + morphological cleanup
            if (params.blurRadius > 0.1f) {
                gaussianBlur(params.blurRadius);
            } else {
                m_blurred = m_grayscale;
            }
            simpleThreshold(params.threshold, params.invert);
            morphologicalClean(2);  // Clean up noise
            break;
    }

    // Step 4: Trace contours
    traceContours(params.minContourLength);

    // Step 5: Simplify contours
    for (auto& contour : m_contours) {
        simplifyContour(contour, params.simplifyEpsilon);
    }

    // Step 6: Connect nearby contours
    if (params.connectDistance > 0) {
        connectNearbyContours(params.connectDistance);
    }

    // Step 7: Order contours for optimal beam path
    orderContours();

    // Step 8: Close contours if requested
    if (params.closedContours) {
        float threshold2 = params.closeThreshold * params.closeThreshold;
        for (auto& contour : m_contours) {
            if (contour.size() >= 3) {
                float dx = contour.x.back() - contour.x.front();
                float dy = contour.y.back() - contour.y.front();
                if (dx*dx + dy*dy < threshold2) {
                    contour.x.push_back(contour.x.front());
                    contour.y.push_back(contour.y.front());
                    contour.closed = true;
                }
            }
        }
    }
}

//==============================================================================
// Grayscale Conversion (Multithreaded)
//==============================================================================

void ImageVectorizer::convertToGrayscale() {
    const size_t pixelCount = static_cast<size_t>(m_width) * m_height;
    uint8_t* dst = m_grayscale.data();
    const uint8_t* src = m_imageData;

    if (m_channels == 1) {
        memcpy(dst, src, pixelCount);
        return;
    }

    if (m_channels >= 3) {
        const int numThreads = getNumThreads();
        const int channels = m_channels;
        std::vector<std::thread> threads;

        auto worker = [&](size_t start, size_t end) {
            for (size_t i = start; i < end; ++i) {
                const uint8_t* pixel = src + i * channels;
                int32_t gray = (pixel[0] * FP_R + pixel[1] * FP_G + pixel[2] * FP_B) >> 16;
                dst[i] = static_cast<uint8_t>(gray);
            }
        };

        size_t chunkSize = pixelCount / numThreads;
        for (int t = 0; t < numThreads; ++t) {
            size_t start = t * chunkSize;
            size_t end = (t == numThreads - 1) ? pixelCount : start + chunkSize;
            threads.emplace_back(worker, start, end);
        }

        for (auto& th : threads) th.join();
    }
}

//==============================================================================
// Brightness / Contrast (LUT-based)
//==============================================================================

void ImageVectorizer::applyBrightnessContrast(float brightness, float contrast) {
    // Build 256-entry lookup table for fast transform
    uint8_t lut[256];
    const float brightnessOffset = brightness * 255.0f;

    for (int i = 0; i < 256; ++i) {
        float val = (static_cast<float>(i) - 127.5f) * contrast + 127.5f + brightnessOffset;
        if (val < 0.0f) val = 0.0f;
        if (val > 255.0f) val = 255.0f;
        lut[i] = static_cast<uint8_t>(val);
    }

    // Apply LUT (very fast - single lookup per pixel)
    uint8_t* ptr = m_grayscale.data();
    const size_t count = m_grayscale.size();
    for (size_t i = 0; i < count; ++i) {
        ptr[i] = lut[ptr[i]];
    }
}

//==============================================================================
// Gaussian Blur (Multithreaded Box Blur Approximation)
//==============================================================================

void ImageVectorizer::gaussianBlur(float sigma) {
    if (sigma < 0.1f) {
        m_blurred = m_grayscale;
        return;
    }

    int radius = static_cast<int>(sigma + 0.5f);
    if (radius < 1) radius = 1;
    if (radius > 10) radius = 10;

    const int w = m_width;
    const int h = m_height;
    const int kernelSize = 2 * radius + 1;
    const int numThreads = getNumThreads();

    // --- Horizontal pass (parallel by rows) ---
    {
        std::vector<std::thread> threads;
        auto hWorker = [&](int yStart, int yEnd) {
            for (int y = yStart; y < yEnd; ++y) {
                const uint8_t* srcRow = m_grayscale.data() + y * w;
                uint8_t* dstRow = m_blurred.data() + y * w;

                int sum = 0;
                for (int x = 0; x < radius; ++x) {
                    sum += srcRow[x];
                }

                for (int x = 0; x < w; ++x) {
                    int right = x + radius;
                    if (right < w) sum += srcRow[right];

                    int left = x - radius - 1;
                    int count = radius + 1 + std::min(radius, w - 1 - x);
                    if (left >= 0) count = kernelSize;

                    dstRow[x] = static_cast<uint8_t>(sum / count);

                    if (left >= 0) sum -= srcRow[left];
                }
            }
        };

        int rowsPerThread = h / numThreads;
        for (int t = 0; t < numThreads; ++t) {
            int yStart = t * rowsPerThread;
            int yEnd = (t == numThreads - 1) ? h : yStart + rowsPerThread;
            threads.emplace_back(hWorker, yStart, yEnd);
        }
        for (auto& th : threads) th.join();
    }

    // --- Vertical pass (parallel by columns) ---
    {
        std::vector<std::thread> threads;
        auto vWorker = [&](int xStart, int xEnd) {
            for (int x = xStart; x < xEnd; ++x) {
                int sum = 0;
                for (int y = 0; y < radius; ++y) {
                    sum += m_blurred[y * w + x];
                }

                for (int y = 0; y < h; ++y) {
                    int bottom = y + radius;
                    if (bottom < h) sum += m_blurred[bottom * w + x];

                    int top = y - radius - 1;
                    int count = radius + 1 + std::min(radius, h - 1 - y);
                    if (top >= 0) count = kernelSize;

                    m_grayscale[y * w + x] = static_cast<uint8_t>(sum / count);

                    if (top >= 0) sum -= m_blurred[top * w + x];
                }
            }
        };

        int colsPerThread = w / numThreads;
        for (int t = 0; t < numThreads; ++t) {
            int xStart = t * colsPerThread;
            int xEnd = (t == numThreads - 1) ? w : xStart + colsPerThread;
            threads.emplace_back(vWorker, xStart, xEnd);
        }
        for (auto& th : threads) th.join();
    }

    m_blurred = m_grayscale;
}

//==============================================================================
// Sobel Gradient (Multithreaded)
//==============================================================================

void ImageVectorizer::sobelGradient() {
    const int w = m_width;
    const int h = m_height;
    const uint8_t* src = m_blurred.data();
    float* mag = m_gradientMag.data();
    float* dir = m_gradientDir.data();

    // Clear borders
    memset(mag, 0, w * sizeof(float));
    memset(mag + (h - 1) * w, 0, w * sizeof(float));
    for (int y = 0; y < h; ++y) {
        mag[y * w] = 0;
        mag[y * w + w - 1] = 0;
    }

    // Multithreaded interior pixel processing
    const int numThreads = getNumThreads();
    std::vector<std::thread> threads;

    auto worker = [&](int yStart, int yEnd) {
        for (int y = yStart; y < yEnd; ++y) {
            const uint8_t* row0 = src + (y - 1) * w;
            const uint8_t* row1 = src + y * w;
            const uint8_t* row2 = src + (y + 1) * w;
            float* magRow = mag + y * w;
            float* dirRow = dir + y * w;

            for (int x = 1; x < w - 1; ++x) {
                int gx = -row0[x-1] + row0[x+1]
                       - 2*row1[x-1] + 2*row1[x+1]
                       - row2[x-1] + row2[x+1];

                int gy = -row0[x-1] - 2*row0[x] - row0[x+1]
                       + row2[x-1] + 2*row2[x] + row2[x+1];

                magRow[x] = std::sqrt(static_cast<float>(gx*gx + gy*gy));
                dirRow[x] = std::atan2(static_cast<float>(gy), static_cast<float>(gx));
            }
        }
    };

    int rowsPerThread = (h - 2) / numThreads;
    for (int t = 0; t < numThreads; ++t) {
        int yStart = 1 + t * rowsPerThread;
        int yEnd = (t == numThreads - 1) ? (h - 1) : yStart + rowsPerThread;
        threads.emplace_back(worker, yStart, yEnd);
    }

    for (auto& th : threads) th.join();
}

//==============================================================================
// Non-Maximum Suppression (Multithreaded)
//==============================================================================

void ImageVectorizer::nonMaxSuppression() {
    const int w = m_width;
    const int h = m_height;
    const float* mag = m_gradientMag.data();
    const float* dir = m_gradientDir.data();

    std::vector<uint8_t> temp(m_edgeData.size(), 0);
    uint8_t* tempData = temp.data();

    const int numThreads = getNumThreads();
    std::vector<std::thread> threads;

    auto worker = [&](int yStart, int yEnd) {
        for (int y = yStart; y < yEnd; ++y) {
            for (int x = 1; x < w - 1; ++x) {
                const int idx = y * w + x;
                const float m = mag[idx];
                const float d = dir[idx];

                float angle = d * 180.0f / static_cast<float>(M_PI);
                if (angle < 0) angle += 180.0f;

                float m1, m2;
                if ((angle >= 0 && angle < 22.5f) || (angle >= 157.5f && angle <= 180.0f)) {
                    m1 = mag[idx - 1];
                    m2 = mag[idx + 1];
                } else if (angle >= 22.5f && angle < 67.5f) {
                    m1 = mag[idx - w + 1];
                    m2 = mag[idx + w - 1];
                } else if (angle >= 67.5f && angle < 112.5f) {
                    m1 = mag[idx - w];
                    m2 = mag[idx + w];
                } else {
                    m1 = mag[idx - w - 1];
                    m2 = mag[idx + w + 1];
                }

                if (m >= m1 && m >= m2) {
                    tempData[idx] = static_cast<uint8_t>(std::min(255.0f, m));
                }
            }
        }
    };

    int rowsPerThread = (h - 2) / numThreads;
    for (int t = 0; t < numThreads; ++t) {
        int yStart = 1 + t * rowsPerThread;
        int yEnd = (t == numThreads - 1) ? (h - 1) : yStart + rowsPerThread;
        threads.emplace_back(worker, yStart, yEnd);
    }

    for (auto& th : threads) th.join();
    m_edgeData = std::move(temp);
}

//==============================================================================
// Hysteresis Thresholding
//==============================================================================

void ImageVectorizer::hysteresisThreshold(float low, float high) {
    const int w = m_width;
    const int h = m_height;
    std::vector<uint8_t> result(m_edgeData.size(), 0);
    std::vector<bool> visited(m_edgeData.size(), false);

    std::queue<int> queue;

    // Find strong edges
    const uint8_t highThresh = static_cast<uint8_t>(high);
    const uint8_t lowThresh = static_cast<uint8_t>(low);

    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            const int idx = y * w + x;
            if (m_edgeData[idx] >= highThresh) {
                result[idx] = 255;
                visited[idx] = true;
                queue.push(idx);
            }
        }
    }

    // 8-connected neighbors
    const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};

    while (!queue.empty()) {
        const int idx = queue.front();
        queue.pop();

        const int x = idx % w;
        const int y = idx / w;

        for (int i = 0; i < 8; ++i) {
            const int nx = x + dx[i];
            const int ny = y + dy[i];

            if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
                const int nidx = ny * w + nx;
                if (!visited[nidx] && m_edgeData[nidx] >= lowThresh) {
                    result[nidx] = 255;
                    visited[nidx] = true;
                    queue.push(nidx);
                }
            }
        }
    }

    m_edgeData = std::move(result);
}

//==============================================================================
// Simple Threshold
//==============================================================================

void ImageVectorizer::simpleThreshold(float thresh, bool invert) {
    const uint8_t t = static_cast<uint8_t>(thresh);
    const size_t count = m_blurred.size();

    for (size_t i = 0; i < count; ++i) {
        bool isWhite = m_blurred[i] > t;
        if (invert) isWhite = !isWhite;
        m_edgeData[i] = isWhite ? 255 : 0;
    }

    // Extract boundary pixels only
    const int w = m_width;
    const int h = m_height;
    std::vector<uint8_t> edges(count, 0);

    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            const int idx = y * w + x;
            if (m_edgeData[idx] == 255) {
                // Check 4-neighbors for boundary
                if (m_edgeData[idx - 1] == 0 || m_edgeData[idx + 1] == 0 ||
                    m_edgeData[idx - w] == 0 || m_edgeData[idx + w] == 0) {
                    edges[idx] = 255;
                }
            }
        }
    }
    m_edgeData = std::move(edges);
}

//==============================================================================
// Adaptive Threshold (Integral Image - O(1) per pixel)
//==============================================================================

void ImageVectorizer::adaptiveThreshold(int blockSize, float C, bool invert) {
    if (blockSize % 2 == 0) blockSize++;
    const int halfBlock = blockSize / 2;
    const int w = m_width;
    const int h = m_height;

    // Build integral image (64-bit to avoid overflow for large images)
    std::vector<int64_t> integral((w + 1) * (h + 1), 0);

    for (int y = 0; y < h; ++y) {
        int64_t rowSum = 0;
        for (int x = 0; x < w; ++x) {
            rowSum += m_blurred[y * w + x];
            integral[(y + 1) * (w + 1) + (x + 1)] =
                integral[y * (w + 1) + (x + 1)] + rowSum;
        }
    }

    // Apply threshold
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const int x1 = std::max(0, x - halfBlock);
            const int y1 = std::max(0, y - halfBlock);
            const int x2 = std::min(w - 1, x + halfBlock);
            const int y2 = std::min(h - 1, y + halfBlock);

            const int count = (x2 - x1 + 1) * (y2 - y1 + 1);

            const int64_t sum = integral[(y2 + 1) * (w + 1) + (x2 + 1)]
                              - integral[y1 * (w + 1) + (x2 + 1)]
                              - integral[(y2 + 1) * (w + 1) + x1]
                              + integral[y1 * (w + 1) + x1];

            const float mean = static_cast<float>(sum) / count;
            const float threshold = mean - C;

            bool isWhite = m_blurred[y * w + x] > threshold;
            if (invert) isWhite = !isWhite;
            m_edgeData[y * w + x] = isWhite ? 255 : 0;
        }
    }

    // Extract boundary pixels
    std::vector<uint8_t> edges(m_edgeData.size(), 0);
    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            const int idx = y * w + x;
            if (m_edgeData[idx] == 255) {
                if (m_edgeData[idx - 1] == 0 || m_edgeData[idx + 1] == 0 ||
                    m_edgeData[idx - w] == 0 || m_edgeData[idx + w] == 0) {
                    edges[idx] = 255;
                }
            }
        }
    }
    m_edgeData = std::move(edges);
}

//==============================================================================
// Contour Tracing (Moore-Neighbor) - Fixed infinite loop bug
//==============================================================================

void ImageVectorizer::traceContours(int minLength) {
    m_contours.clear();

    const int w = m_width;
    const int h = m_height;
    std::vector<bool> visited(m_edgeData.size(), false);

    // 8-connected neighbor offsets (clockwise from right)
    const int dx[] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dy[] = {0, 1, 1, 1, 0, -1, -1, -1};

    // Maximum points per contour to prevent infinite loops
    const size_t maxContourPoints = static_cast<size_t>(w) * h;

    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            const int idx = y * w + x;

            if (m_edgeData[idx] == 255 && !visited[idx]) {
                Contour contour;

                int cx = x, cy = y;
                const int startX = x, startY = y;
                int dir = 0;

                do {
                    contour.addPoint(static_cast<float>(cx), static_cast<float>(cy));
                    visited[cy * w + cx] = true;

                    // Safety: prevent infinite loops
                    if (contour.size() > maxContourPoints) break;

                    bool found = false;
                    const int startDir = (dir + 6) % 8;

                    for (int i = 0; i < 8; ++i) {
                        const int d = (startDir + i) % 8;
                        const int nx = cx + dx[d];
                        const int ny = cy + dy[d];

                        if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
                            // Check BOTH edge data AND not visited (except for start point)
                            bool isStart = (nx == startX && ny == startY);
                            if (m_edgeData[ny * w + nx] == 255 &&
                                (!visited[ny * w + nx] || isStart)) {
                                cx = nx;
                                cy = ny;
                                dir = d;
                                found = true;
                                break;
                            }
                        }
                    }

                    if (!found) break;

                } while (cx != startX || cy != startY);

                if (static_cast<int>(contour.size()) >= minLength) {
                    m_contours.push_back(std::move(contour));
                }
            }
        }
    }
}

//==============================================================================
// Douglas-Peucker Simplification
//==============================================================================

void ImageVectorizer::simplifyContour(Contour& contour, float epsilon) {
    if (contour.size() < 3) return;

    std::vector<bool> keep(contour.size(), false);
    keep[0] = true;
    keep[contour.size() - 1] = true;

    Contour dummy;
    douglasPeucker(contour, dummy, 0, contour.size() - 1, epsilon, keep);

    Contour simplified;
    for (size_t i = 0; i < contour.size(); ++i) {
        if (keep[i]) {
            simplified.addPoint(contour.x[i], contour.y[i]);
        }
    }

    contour = std::move(simplified);
}

void ImageVectorizer::douglasPeucker(const Contour& input, Contour& output,
                                     size_t start, size_t end, float epsilon,
                                     std::vector<bool>& keep) {
    if (end <= start + 1) return;

    const float x1 = input.x[start], y1 = input.y[start];
    const float x2 = input.x[end], y2 = input.y[end];
    const float dx = x2 - x1;
    const float dy = y2 - y1;
    const float lenSq = dx*dx + dy*dy;

    if (lenSq < 0.001f) return;

    float maxDist = 0;
    size_t maxIdx = start;

    for (size_t i = start + 1; i < end; ++i) {
        const float px = input.x[i] - x1;
        const float py = input.y[i] - y1;
        const float dist = std::abs(px * dy - py * dx) / std::sqrt(lenSq);

        if (dist > maxDist) {
            maxDist = dist;
            maxIdx = i;
        }
    }

    if (maxDist > epsilon) {
        keep[maxIdx] = true;
        douglasPeucker(input, output, start, maxIdx, epsilon, keep);
        douglasPeucker(input, output, maxIdx, end, epsilon, keep);
    }
}

//==============================================================================
// Contour Ordering (Greedy TSP)
//==============================================================================

void ImageVectorizer::orderContours() {
    if (m_contours.size() < 2) return;

    std::vector<Contour> ordered;
    std::vector<bool> used(m_contours.size(), false);

    ordered.push_back(std::move(m_contours[0]));
    used[0] = true;

    float lastX = ordered[0].x.back();
    float lastY = ordered[0].y.back();

    for (size_t n = 1; n < m_contours.size(); ++n) {
        float minDist = 1e9f;
        size_t minIdx = 0;
        bool reverse = false;

        for (size_t j = 0; j < m_contours.size(); ++j) {
            if (used[j] || m_contours[j].empty()) continue;

            float dx = m_contours[j].x.front() - lastX;
            float dy = m_contours[j].y.front() - lastY;
            float distStart = dx*dx + dy*dy;

            dx = m_contours[j].x.back() - lastX;
            dy = m_contours[j].y.back() - lastY;
            float distEnd = dx*dx + dy*dy;

            if (distStart < minDist) {
                minDist = distStart;
                minIdx = j;
                reverse = false;
            }
            if (distEnd < minDist) {
                minDist = distEnd;
                minIdx = j;
                reverse = true;
            }
        }

        Contour& next = m_contours[minIdx];
        if (reverse) {
            Contour reversed;
            for (int k = static_cast<int>(next.size()) - 1; k >= 0; --k) {
                reversed.addPoint(next.x[k], next.y[k]);
            }
            ordered.push_back(std::move(reversed));
        } else {
            ordered.push_back(std::move(next));
        }

        used[minIdx] = true;
        lastX = ordered.back().x.back();
        lastY = ordered.back().y.back();
    }

    m_contours = std::move(ordered);
}

//==============================================================================
// Connect Nearby Contours
//==============================================================================

void ImageVectorizer::connectNearbyContours(float maxDistance) {
    if (m_contours.size() < 2) return;

    const float maxDistSq = maxDistance * maxDistance;
    std::vector<Contour> connected;

    Contour current = std::move(m_contours[0]);

    for (size_t i = 1; i < m_contours.size(); ++i) {
        const float dx = m_contours[i].x.front() - current.x.back();
        const float dy = m_contours[i].y.front() - current.y.back();
        const float distSq = dx*dx + dy*dy;

        if (distSq < maxDistSq) {
            for (size_t j = 0; j < m_contours[i].size(); ++j) {
                current.addPoint(m_contours[i].x[j], m_contours[i].y[j]);
            }
        } else {
            connected.push_back(std::move(current));
            current = std::move(m_contours[i]);
        }
    }

    connected.push_back(std::move(current));
    m_contours = std::move(connected);
}

//==============================================================================
// Generate Final Pattern
//==============================================================================

void ImageVectorizer::generatePattern(std::vector<float>& outX, std::vector<float>& outY,
                                      const VectorizerParams& params) {
    outX.clear();
    outY.clear();

    if (m_contours.empty() || m_width == 0 || m_height == 0) return;

    size_t totalPoints = 0;
    for (const auto& c : m_contours) {
        totalPoints += c.size();
    }

    if (totalPoints == 0) return;

    const size_t maxPoints = static_cast<size_t>(params.maxOutputPoints);
    const float decimation = (totalPoints > maxPoints) ?
                             static_cast<float>(totalPoints) / maxPoints : 1.0f;

    const float scaleX = 2.0f / m_width;
    const float scaleY = 2.0f / m_height;
    const float scale = std::min(scaleX, scaleY) * 0.95f;

    const float offsetX = -m_width / 2.0f;
    const float offsetY = -m_height / 2.0f;

    outX.reserve(maxPoints);
    outY.reserve(maxPoints);

    float accumulator = 0.0f;
    for (const auto& contour : m_contours) {
        for (size_t i = 0; i < contour.size(); ++i) {
            accumulator += 1.0f;
            if (accumulator >= decimation) {
                accumulator -= decimation;
                outX.push_back((contour.x[i] + offsetX) * scale);
                outY.push_back(-(contour.y[i] + offsetY) * scale);
            }
        }
    }
}

//==============================================================================
// Bilateral Filter (Edge-Preserving Smoothing)
// Research: Tomasi & Manduchi 1998 - "Bilateral Filtering for Gray and Color Images"
// Smooths while preserving edges by considering both spatial and intensity distance
//==============================================================================

void ImageVectorizer::bilateralFilter(int d, float sigmaColor, float sigmaSpace) {
    const int w = m_width;
    const int h = m_height;
    const int radius = d / 2;

    std::vector<uint8_t> output(m_grayscale.size());
    const uint8_t* src = m_grayscale.data();
    uint8_t* dst = output.data();

    // Precompute spatial Gaussian weights
    std::vector<float> spatialWeights((2 * radius + 1) * (2 * radius + 1));
    const float spaceCoeff = -0.5f / (sigmaSpace * sigmaSpace);
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            float dist2 = static_cast<float>(dx * dx + dy * dy);
            spatialWeights[(dy + radius) * (2 * radius + 1) + (dx + radius)] =
                std::exp(dist2 * spaceCoeff);
        }
    }

    // Precompute color/intensity Gaussian weights (256 possible differences)
    std::vector<float> colorWeights(256);
    const float colorCoeff = -0.5f / (sigmaColor * sigmaColor);
    for (int i = 0; i < 256; ++i) {
        colorWeights[i] = std::exp(static_cast<float>(i * i) * colorCoeff);
    }

    // Apply bilateral filter (multithreaded)
    const int numThreads = getNumThreads();
    std::vector<std::thread> threads;

    auto worker = [&](int yStart, int yEnd) {
        for (int y = yStart; y < yEnd; ++y) {
            for (int x = 0; x < w; ++x) {
                const int centerVal = src[y * w + x];
                float sum = 0.0f;
                float wsum = 0.0f;

                for (int dy = -radius; dy <= radius; ++dy) {
                    const int ny = y + dy;
                    if (ny < 0 || ny >= h) continue;

                    for (int dx = -radius; dx <= radius; ++dx) {
                        const int nx = x + dx;
                        if (nx < 0 || nx >= w) continue;

                        const int neighborVal = src[ny * w + nx];
                        const int colorDiff = std::abs(neighborVal - centerVal);

                        const float spatialW = spatialWeights[(dy + radius) * (2 * radius + 1) + (dx + radius)];
                        const float colorW = colorWeights[colorDiff];
                        const float weight = spatialW * colorW;

                        sum += weight * neighborVal;
                        wsum += weight;
                    }
                }

                dst[y * w + x] = static_cast<uint8_t>(sum / wsum);
            }
        }
    };

    int rowsPerThread = h / numThreads;
    for (int t = 0; t < numThreads; ++t) {
        int yStart = t * rowsPerThread;
        int yEnd = (t == numThreads - 1) ? h : yStart + rowsPerThread;
        threads.emplace_back(worker, yStart, yEnd);
    }
    for (auto& th : threads) th.join();

    m_grayscale = std::move(output);
    m_blurred = m_grayscale;
}

//==============================================================================
// Median Filter (Salt-and-Pepper Noise Removal)
// Good for cleaning up quantization artifacts while preserving edges
//==============================================================================

void ImageVectorizer::medianFilter(int radius) {
    const int w = m_width;
    const int h = m_height;
    const int kernelSize = 2 * radius + 1;
    const int kernelArea = kernelSize * kernelSize;
    const int medianIdx = kernelArea / 2;

    std::vector<uint8_t> output(m_grayscale.size());
    const uint8_t* src = m_grayscale.data();
    uint8_t* dst = output.data();

    // Use histogram-based median for efficiency
    const int numThreads = getNumThreads();
    std::vector<std::thread> threads;

    auto worker = [&](int yStart, int yEnd) {
        std::vector<uint8_t> values(kernelArea);

        for (int y = yStart; y < yEnd; ++y) {
            for (int x = 0; x < w; ++x) {
                int count = 0;

                for (int dy = -radius; dy <= radius; ++dy) {
                    const int ny = std::max(0, std::min(h - 1, y + dy));
                    for (int dx = -radius; dx <= radius; ++dx) {
                        const int nx = std::max(0, std::min(w - 1, x + dx));
                        values[count++] = src[ny * w + nx];
                    }
                }

                // Partial sort to find median
                std::nth_element(values.begin(), values.begin() + medianIdx, values.begin() + count);
                dst[y * w + x] = values[medianIdx];
            }
        }
    };

    int rowsPerThread = h / numThreads;
    for (int t = 0; t < numThreads; ++t) {
        int yStart = t * rowsPerThread;
        int yEnd = (t == numThreads - 1) ? h : yStart + rowsPerThread;
        threads.emplace_back(worker, yStart, yEnd);
    }
    for (auto& th : threads) th.join();

    m_grayscale = std::move(output);
}

//==============================================================================
// Difference of Gaussians (DoG) - Line Detection
// Research: Marr & Hildreth 1980 - approximates Laplacian of Gaussian
// Excellent for detecting lines of varying thickness in sketches/drawings
//==============================================================================

void ImageVectorizer::differenceOfGaussians(float sigma1, float sigma2, float threshold) {
    const int w = m_width;
    const int h = m_height;
    const size_t pixelCount = static_cast<size_t>(w) * h;

    // Create two blurred versions
    std::vector<uint8_t> blur1(pixelCount);
    std::vector<uint8_t> blur2(pixelCount);

    // Save original grayscale
    std::vector<uint8_t> original = m_grayscale;

    // First blur (smaller sigma - finer details)
    gaussianBlur(sigma1);
    blur1 = m_blurred;

    // Second blur (larger sigma - coarser)
    m_grayscale = original;  // Restore original
    gaussianBlur(sigma2);
    blur2 = m_blurred;

    // Compute difference and threshold
    m_edgeData.resize(pixelCount);
    const uint8_t thresh = static_cast<uint8_t>(threshold);

    for (size_t i = 0; i < pixelCount; ++i) {
        int diff = static_cast<int>(blur1[i]) - static_cast<int>(blur2[i]);
        // Take absolute value and apply threshold
        int absDiff = std::abs(diff);
        m_edgeData[i] = (absDiff > thresh) ? 255 : 0;
    }

    // Restore grayscale
    m_grayscale = original;
    m_blurred = original;
}

//==============================================================================
// Color Quantization (Reduce colors for cartoon-style vectorization)
// Uses uniform quantization for speed - reduces grayscale to N levels
//==============================================================================

void ImageVectorizer::colorQuantize(int levels) {
    if (levels < 2) levels = 2;
    if (levels > 256) levels = 256;

    const size_t count = m_grayscale.size();
    uint8_t* data = m_grayscale.data();

    // Calculate level size
    const float levelSize = 256.0f / levels;

    // Build quantization LUT
    uint8_t lut[256];
    for (int i = 0; i < 256; ++i) {
        int level = static_cast<int>(i / levelSize);
        if (level >= levels) level = levels - 1;
        // Map to center of the level
        lut[i] = static_cast<uint8_t>((level + 0.5f) * levelSize);
    }

    // Apply quantization
    for (size_t i = 0; i < count; ++i) {
        data[i] = lut[data[i]];
    }
}

//==============================================================================
// Morphological Operations (Clean up edges)
// Erosion followed by dilation removes small noise while preserving shape
//==============================================================================

void ImageVectorizer::morphologicalClean(int iterations) {
    const int w = m_width;
    const int h = m_height;
    std::vector<uint8_t> temp(m_edgeData.size());

    for (int iter = 0; iter < iterations; ++iter) {
        // Erosion - shrink white regions (remove small noise)
        for (int y = 1; y < h - 1; ++y) {
            for (int x = 1; x < w - 1; ++x) {
                const int idx = y * w + x;
                // 3x3 erosion: pixel is white only if all neighbors are white
                bool allWhite = true;
                for (int dy = -1; dy <= 1 && allWhite; ++dy) {
                    for (int dx = -1; dx <= 1 && allWhite; ++dx) {
                        if (m_edgeData[(y + dy) * w + (x + dx)] == 0) {
                            allWhite = false;
                        }
                    }
                }
                temp[idx] = allWhite ? 255 : 0;
            }
        }
        m_edgeData = temp;

        // Dilation - expand white regions (restore shape)
        for (int y = 1; y < h - 1; ++y) {
            for (int x = 1; x < w - 1; ++x) {
                const int idx = y * w + x;
                // 3x3 dilation: pixel is white if any neighbor is white
                bool anyWhite = false;
                for (int dy = -1; dy <= 1 && !anyWhite; ++dy) {
                    for (int dx = -1; dx <= 1 && !anyWhite; ++dx) {
                        if (m_edgeData[(y + dy) * w + (x + dx)] == 255) {
                            anyWhite = true;
                        }
                    }
                }
                temp[idx] = anyWhite ? 255 : 0;
            }
        }
        m_edgeData = temp;
    }
}

//==============================================================================
// Skin Detection using YCbCr Color Space
// Research: Chai & Ngan 1999, Phung et al. 2005
// YCbCr is more robust to lighting variations than RGB
// Skin typically falls in: 77 < Cb < 127, 133 < Cr < 173
//==============================================================================

void ImageVectorizer::detectSkinRegions(float sensitivity) {
    if (!m_imageData || m_channels < 3) {
        // Can't detect skin without color image
        m_skinMask.assign(m_grayscale.size(), 0);
        return;
    }

    const int w = m_width;
    const int h = m_height;
    const size_t pixelCount = static_cast<size_t>(w) * h;
    m_skinMask.resize(pixelCount);

    // Sensitivity adjusts the range of accepted skin colors
    // Higher sensitivity = wider range = more false positives
    const float cbMin = 77.0f - (sensitivity - 1.0f) * 15.0f;
    const float cbMax = 127.0f + (sensitivity - 1.0f) * 15.0f;
    const float crMin = 133.0f - (sensitivity - 1.0f) * 15.0f;
    const float crMax = 173.0f + (sensitivity - 1.0f) * 15.0f;

    const int numThreads = getNumThreads();
    std::vector<std::thread> threads;

    auto worker = [&](size_t start, size_t end) {
        for (size_t i = start; i < end; ++i) {
            const uint8_t* pixel = m_imageData + i * m_channels;
            const float r = pixel[0];
            const float g = pixel[1];
            const float b = pixel[2];

            // Convert RGB to YCbCr
            // Y  =  0.299*R + 0.587*G + 0.114*B
            // Cb = -0.169*R - 0.331*G + 0.500*B + 128
            // Cr =  0.500*R - 0.419*G - 0.081*B + 128
            const float cb = -0.169f * r - 0.331f * g + 0.500f * b + 128.0f;
            const float cr =  0.500f * r - 0.419f * g - 0.081f * b + 128.0f;

            // Check if pixel is in skin color range
            bool isSkin = (cb >= cbMin && cb <= cbMax && cr >= crMin && cr <= crMax);

            // Additional heuristic: skin should not be too dark or too bright
            const float y = 0.299f * r + 0.587f * g + 0.114f * b;
            if (y < 40.0f || y > 240.0f) isSkin = false;

            // Exclude very saturated colors (not skin-like)
            const float maxC = std::max({r, g, b});
            const float minC = std::min({r, g, b});
            if (maxC > 0 && (maxC - minC) / maxC > 0.8f) isSkin = false;

            m_skinMask[i] = isSkin ? 255 : 0;
        }
    };

    size_t chunkSize = pixelCount / numThreads;
    for (int t = 0; t < numThreads; ++t) {
        size_t start = t * chunkSize;
        size_t end = (t == numThreads - 1) ? pixelCount : start + chunkSize;
        threads.emplace_back(worker, start, end);
    }
    for (auto& th : threads) th.join();

    // Morphological cleanup - remove noise
    std::vector<uint8_t> temp(pixelCount);

    // Erosion followed by dilation (opening) to remove small noise
    for (int iter = 0; iter < 2; ++iter) {
        // Erosion
        for (int y = 1; y < h - 1; ++y) {
            for (int x = 1; x < w - 1; ++x) {
                const int idx = y * w + x;
                bool allSkin = true;
                for (int dy = -1; dy <= 1 && allSkin; ++dy) {
                    for (int dx = -1; dx <= 1 && allSkin; ++dx) {
                        if (m_skinMask[(y + dy) * w + (x + dx)] == 0) {
                            allSkin = false;
                        }
                    }
                }
                temp[idx] = allSkin ? 255 : 0;
            }
        }
        m_skinMask = temp;

        // Dilation
        for (int y = 1; y < h - 1; ++y) {
            for (int x = 1; x < w - 1; ++x) {
                const int idx = y * w + x;
                bool anySkin = false;
                for (int dy = -1; dy <= 1 && !anySkin; ++dy) {
                    for (int dx = -1; dx <= 1 && !anySkin; ++dx) {
                        if (m_skinMask[(y + dy) * w + (x + dx)] == 255) {
                            anySkin = true;
                        }
                    }
                }
                temp[idx] = anySkin ? 255 : 0;
            }
        }
        m_skinMask = temp;
    }
}

//==============================================================================
// Find Face Regions from Skin Mask
// Uses connected component analysis and shape heuristics
// Faces are roughly elliptical with aspect ratio 0.7-1.0 (width/height)
//==============================================================================

void ImageVectorizer::findFaceRegions(float minRatio, bool multiple) {
    m_detectedFaces.clear();

    const int w = m_width;
    const int h = m_height;
    const size_t pixelCount = static_cast<size_t>(w) * h;
    const int minArea = static_cast<int>(pixelCount * minRatio * minRatio);

    // Find connected components in skin mask
    std::vector<int> labels(pixelCount, 0);
    int nextLabel = 1;

    // Simple flood-fill labeling
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const int idx = y * w + x;
            if (m_skinMask[idx] == 255 && labels[idx] == 0) {
                // BFS flood fill
                std::queue<std::pair<int, int>> queue;
                queue.push({x, y});
                labels[idx] = nextLabel;

                int minX = x, maxX = x, minY = y, maxY = y;
                int area = 0;

                while (!queue.empty()) {
                    auto [cx, cy] = queue.front();
                    queue.pop();

                    minX = std::min(minX, cx);
                    maxX = std::max(maxX, cx);
                    minY = std::min(minY, cy);
                    maxY = std::max(maxY, cy);
                    area++;

                    // 4-connected neighbors
                    const int dx[] = {-1, 1, 0, 0};
                    const int dy[] = {0, 0, -1, 1};
                    for (int i = 0; i < 4; ++i) {
                        int nx = cx + dx[i];
                        int ny = cy + dy[i];
                        if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
                            int nidx = ny * w + nx;
                            if (m_skinMask[nidx] == 255 && labels[nidx] == 0) {
                                labels[nidx] = nextLabel;
                                queue.push({nx, ny});
                            }
                        }
                    }
                }

                // Analyze this region
                int regionW = maxX - minX + 1;
                int regionH = maxY - minY + 1;
                float aspectRatio = static_cast<float>(regionW) / regionH;
                float fillRatio = static_cast<float>(area) / (regionW * regionH);

                // Face heuristics:
                // - Aspect ratio roughly 0.6-1.2 (faces are roughly as wide as tall)
                // - Fill ratio > 0.4 (faces are fairly solid, not scattered pixels)
                // - Minimum size
                bool isFaceLike = (aspectRatio >= 0.5f && aspectRatio <= 1.3f) &&
                                  (fillRatio >= 0.35f) &&
                                  (area >= minArea);

                if (isFaceLike) {
                    FaceRegion face;
                    face.x = minX;
                    face.y = minY;
                    face.width = regionW;
                    face.height = regionH;
                    face.centerX = (minX + maxX) / 2;
                    face.centerY = (minY + maxY) / 2;
                    // Confidence based on how "face-like" the shape is
                    float aspectScore = 1.0f - std::abs(aspectRatio - 0.85f) / 0.35f;
                    float fillScore = std::min(1.0f, fillRatio / 0.6f);
                    face.confidence = aspectScore * 0.5f + fillScore * 0.5f;

                    m_detectedFaces.push_back(face);
                }

                nextLabel++;
            }
        }
    }

    // Sort by area (largest first) and optionally keep only the largest
    std::sort(m_detectedFaces.begin(), m_detectedFaces.end(),
              [](const FaceRegion& a, const FaceRegion& b) {
                  return (a.width * a.height) > (b.width * b.height);
              });

    if (!multiple && m_detectedFaces.size() > 1) {
        m_detectedFaces.resize(1);
    }
}

//==============================================================================
// Create Face Mask for Regional Processing
// Extends face regions to include hair and creates importance gradient
//==============================================================================

void ImageVectorizer::createFaceMask() {
    const int w = m_width;
    const int h = m_height;
    const size_t pixelCount = static_cast<size_t>(w) * h;

    m_faceMask.assign(pixelCount, 0);
    m_importanceMask.assign(pixelCount, 64);  // Default medium-low importance

    if (m_detectedFaces.empty()) {
        // No faces detected - use skin regions as fallback
        m_faceMask = m_skinMask;
        for (size_t i = 0; i < pixelCount; ++i) {
            m_importanceMask[i] = (m_skinMask[i] > 0) ? 192 : 64;
        }
        return;
    }

    for (const auto& face : m_detectedFaces) {
        // Extend upward for hair (typically 30-40% of face height)
        int hairExtension = static_cast<int>(face.height * 0.4f);
        int headTop = std::max(0, face.y - hairExtension);

        // Extend downward for neck/shoulders
        int shoulderExtension = static_cast<int>(face.height * 0.8f);
        int shoulderBottom = std::min(h - 1, face.y + face.height + shoulderExtension);

        // Extend sideways slightly for ears
        int earExtension = static_cast<int>(face.width * 0.2f);
        int headLeft = std::max(0, face.x - earExtension);
        int headRight = std::min(w - 1, face.x + face.width + earExtension);

        // Create elliptical mask for face region
        float faceCenterX = face.centerX;
        float faceCenterY = face.centerY;
        float faceRadiusX = face.width * 0.6f;
        float faceRadiusY = face.height * 0.6f;

        for (int y = headTop; y <= shoulderBottom; ++y) {
            for (int x = headLeft; x <= headRight; ++x) {
                const int idx = y * w + x;

                // Distance from face center (normalized)
                float dx = (x - faceCenterX) / faceRadiusX;
                float dy = (y - faceCenterY) / faceRadiusY;
                float dist = std::sqrt(dx * dx + dy * dy);

                // Importance decreases with distance from face center
                float importance;
                if (dist < 0.8f) {
                    // Core face region - highest importance
                    m_faceMask[idx] = 255;
                    importance = 255.0f;
                } else if (dist < 1.5f) {
                    // Near-face region (hair, ears) - high importance
                    m_faceMask[idx] = 192;
                    importance = 255.0f - (dist - 0.8f) * 100.0f;
                } else if (dist < 3.0f) {
                    // Extended region (shoulders) - medium importance
                    m_faceMask[idx] = std::max(m_faceMask[idx], static_cast<uint8_t>(128));
                    importance = 180.0f - (dist - 1.5f) * 40.0f;
                } else {
                    continue;
                }

                m_importanceMask[idx] = static_cast<uint8_t>(std::max(64.0f, std::min(255.0f, importance)));
            }
        }
    }

    // Smooth the importance mask
    std::vector<uint8_t> temp(pixelCount);
    for (int y = 2; y < h - 2; ++y) {
        for (int x = 2; x < w - 2; ++x) {
            int sum = 0;
            for (int dy = -2; dy <= 2; ++dy) {
                for (int dx = -2; dx <= 2; ++dx) {
                    sum += m_importanceMask[(y + dy) * w + (x + dx)];
                }
            }
            temp[y * w + x] = static_cast<uint8_t>(sum / 25);
        }
    }
    m_importanceMask = temp;
}

//==============================================================================
// Apply Regional Processing - Different edge detection per region
// Face: Fine detail with bilateral smoothing
// Background: Simplified with higher thresholds
//==============================================================================

void ImageVectorizer::applyRegionalProcessing(const VectorizerParams& params) {
    const int w = m_width;
    const int h = m_height;
    const size_t pixelCount = static_cast<size_t>(w) * h;

    // First pass: full edge detection
    sobelGradient();
    nonMaxSuppression();

    // Apply hysteresis with region-dependent thresholds
    std::vector<uint8_t> result(pixelCount, 0);
    std::vector<bool> visited(pixelCount, false);
    std::queue<int> queue;

    // Adjusted thresholds based on region importance
    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            const int idx = y * w + x;

            // Adjust thresholds based on importance
            float importance = m_importanceMask[idx] / 255.0f;
            float faceBoost = (m_faceMask[idx] > 128) ? params.faceEmphasis : 1.0f;
            float bgSimplify = (m_faceMask[idx] < 64) ? params.backgroundSimplify : 1.0f;

            // Lower thresholds for important regions (face)
            // Higher thresholds for background
            float localHighThresh = params.cannyHigh / faceBoost * bgSimplify;
            float localLowThresh = params.cannyLow / faceBoost * bgSimplify;

            if (m_edgeData[idx] >= static_cast<uint8_t>(localHighThresh)) {
                result[idx] = 255;
                visited[idx] = true;
                queue.push(idx);
            }
        }
    }

    // Trace weak edges connected to strong edges
    const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};

    while (!queue.empty()) {
        const int idx = queue.front();
        queue.pop();

        const int x = idx % w;
        const int y = idx / w;

        // Adjust low threshold based on region
        float importance = m_importanceMask[idx] / 255.0f;
        float faceBoost = (m_faceMask[idx] > 128) ? params.faceEmphasis : 1.0f;
        float bgSimplify = (m_faceMask[idx] < 64) ? params.backgroundSimplify : 1.0f;
        float localLowThresh = params.cannyLow / faceBoost * bgSimplify;

        for (int i = 0; i < 8; ++i) {
            const int nx = x + dx[i];
            const int ny = y + dy[i];

            if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
                const int nidx = ny * w + nx;
                if (!visited[nidx] && m_edgeData[nidx] >= static_cast<uint8_t>(localLowThresh)) {
                    result[nidx] = 255;
                    visited[nidx] = true;
                    queue.push(nidx);
                }
            }
        }
    }

    m_edgeData = std::move(result);
}

} // namespace oscilloplot

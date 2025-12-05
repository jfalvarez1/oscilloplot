#include "test_pattern.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace oscilloplot {
namespace generators {

//==============================================================================
// Basic Shapes
//==============================================================================

void generateSineWave(Pattern& pattern, int numPoints, float cycles) {
    pattern.resize(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / (numPoints - 1);
        pattern.x[i] = t * 2.0f - 1.0f;  // -1 to 1
        pattern.y[i] = std::sin(cycles * 2.0f * static_cast<float>(M_PI) * t);
    }
}

void generateCircle(Pattern& pattern, int numPoints) {
    pattern.resize(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / numPoints * 2.0f * static_cast<float>(M_PI);
        pattern.x[i] = std::cos(t);
        pattern.y[i] = std::sin(t);
    }
}

void generateEllipse(Pattern& pattern, int numPoints, float aspectX, float aspectY) {
    pattern.resize(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / numPoints * 2.0f * static_cast<float>(M_PI);
        pattern.x[i] = aspectX * std::cos(t);
        pattern.y[i] = aspectY * std::sin(t);
    }
}

//==============================================================================
// Lissajous Curves
//==============================================================================

void generateLissajous(Pattern& pattern, int numPoints, int a, int b, float delta) {
    pattern.resize(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / numPoints * 2.0f * static_cast<float>(M_PI);
        pattern.x[i] = std::sin(a * t + delta);
        pattern.y[i] = std::sin(b * t);
    }
}

//==============================================================================
// Stars & Flowers
//==============================================================================

void generateStar(Pattern& pattern, int numPoints, int numSpikes, float innerRadius) {
    pattern.resize(numPoints);

    // Clamp parameters
    numSpikes = std::max(3, numSpikes);
    innerRadius = std::max(0.1f, std::min(0.9f, innerRadius));

    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / numPoints * 2.0f * static_cast<float>(M_PI);

        // Calculate which segment we're in (0 to 2*numSpikes-1)
        float segment = t * numSpikes / static_cast<float>(M_PI);
        float segmentFrac = std::fmod(segment, 2.0f);

        // Triangle wave: goes from innerRadius at 0.5 to 1.0 at 0 and 1.0
        // Creates sharp star points
        float r;
        if (segmentFrac < 1.0f) {
            // Going from outer (1.0) down to inner
            r = 1.0f - (1.0f - innerRadius) * segmentFrac;
        } else {
            // Going from inner back to outer (1.0)
            r = innerRadius + (1.0f - innerRadius) * (segmentFrac - 1.0f);
        }

        pattern.x[i] = r * std::cos(t);
        pattern.y[i] = r * std::sin(t);
    }
}

void generateFlower(Pattern& pattern, int numPoints, int numPetals, float petalDepth) {
    pattern.resize(numPoints);

    // Clamp parameters
    numPetals = std::max(2, numPetals);
    petalDepth = std::max(0.1f, std::min(0.9f, petalDepth));

    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / numPoints * 2.0f * static_cast<float>(M_PI);
        // Flower with numPetals petals - using absolute value of cos for proper petal shape
        float r = (1.0f - petalDepth) + petalDepth * std::abs(std::cos(numPetals * t * 0.5f));
        pattern.x[i] = r * std::cos(t);
        pattern.y[i] = r * std::sin(t);
    }
}

void generateRoseCurve(Pattern& pattern, int numPoints, int k) {
    pattern.resize(numPoints);

    // Clamp k to valid range
    k = std::max(1, k);

    // Rose curve: r = cos(n * theta)
    // For n petals: use n directly in formula
    // Odd n: n petals when theta goes 0 to PI
    // Even n: 2n petals when theta goes 0 to 2*PI, OR n petals with n/2 formula
    //
    // We want k to always mean k petals, so:
    // - Odd k: use k in formula, theta from 0 to PI
    // - Even k: use k in formula, theta from 0 to PI (gives k petals, not 2k)

    float maxTheta = static_cast<float>(M_PI);  // PI gives k petals for any k

    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / numPoints * maxTheta;
        float r = std::cos(static_cast<float>(k) * t);
        pattern.x[i] = r * std::cos(t);
        pattern.y[i] = r * std::sin(t);
    }
}

//==============================================================================
// Spirals
//==============================================================================

void generateSpiral(Pattern& pattern, int numPoints, float turns, float startRadius, float endRadius) {
    pattern.resize(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / (numPoints - 1);
        float angle = turns * 2.0f * static_cast<float>(M_PI) * t;
        float r = startRadius + (endRadius - startRadius) * t;

        pattern.x[i] = r * std::cos(angle);
        pattern.y[i] = r * std::sin(angle);
    }
}

void generateLogSpiral(Pattern& pattern, int numPoints, float a, float b, float maxAngle) {
    pattern.resize(numPoints);

    float maxR = 0.0f;
    // First pass to find max radius for normalization
    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / (numPoints - 1) * maxAngle;
        float r = a * std::exp(b * t);
        if (r > maxR) maxR = r;
    }

    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / (numPoints - 1) * maxAngle;
        float r = a * std::exp(b * t) / maxR;  // Normalize

        pattern.x[i] = r * std::cos(t);
        pattern.y[i] = r * std::sin(t);
    }
}

//==============================================================================
// Knots & 3D-Style Curves
//==============================================================================

void generateHelix(Pattern& pattern, int numPoints) {
    pattern.resize(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / numPoints * 2.0f * static_cast<float>(M_PI);
        pattern.x[i] = std::cos(t) * 0.7f;
        pattern.y[i] = std::sin(t) * 0.7f + (t - static_cast<float>(M_PI)) / (2.0f * static_cast<float>(M_PI)) * 0.5f;
    }
}

void generateTrefoilKnot(Pattern& pattern, int numPoints) {
    pattern.resize(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / numPoints * 2.0f * static_cast<float>(M_PI);
        pattern.x[i] = (std::sin(t) + 2.0f * std::sin(2.0f * t)) * 0.3f;
        pattern.y[i] = (std::cos(t) - 2.0f * std::cos(2.0f * t)) * 0.3f;
    }
}

void generateTorusKnot(Pattern& pattern, int numPoints, int p, int q) {
    pattern.resize(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / numPoints * 2.0f * static_cast<float>(M_PI);
        float r = 2.0f + std::cos(q * t);
        pattern.x[i] = r * std::cos(p * t) * 0.3f;
        pattern.y[i] = r * std::sin(p * t) * 0.3f;
    }
}

//==============================================================================
// Complex Curves
//==============================================================================

void generateHypotrochoid(Pattern& pattern, int numPoints, float R, float r, float d) {
    pattern.resize(numPoints);

    float maxVal = 0.0f;
    // First pass to find max for normalization
    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / numPoints * 4.0f * static_cast<float>(M_PI);
        float x = (R - r) * std::cos(t) + d * std::cos((R - r) / r * t);
        float y = (R - r) * std::sin(t) - d * std::sin((R - r) / r * t);
        float val = std::max(std::abs(x), std::abs(y));
        if (val > maxVal) maxVal = val;
    }

    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / numPoints * 4.0f * static_cast<float>(M_PI);
        pattern.x[i] = ((R - r) * std::cos(t) + d * std::cos((R - r) / r * t)) / maxVal;
        pattern.y[i] = ((R - r) * std::sin(t) - d * std::sin((R - r) / r * t)) / maxVal;
    }
}

void generateEpitrochoid(Pattern& pattern, int numPoints, float R, float r, float d) {
    pattern.resize(numPoints);

    float maxVal = 0.0f;
    // First pass to find max for normalization
    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / numPoints * 4.0f * static_cast<float>(M_PI);
        float x = (R + r) * std::cos(t) - d * std::cos((R + r) / r * t);
        float y = (R + r) * std::sin(t) - d * std::sin((R + r) / r * t);
        float val = std::max(std::abs(x), std::abs(y));
        if (val > maxVal) maxVal = val;
    }

    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / numPoints * 4.0f * static_cast<float>(M_PI);
        pattern.x[i] = ((R + r) * std::cos(t) - d * std::cos((R + r) / r * t)) / maxVal;
        pattern.y[i] = ((R + r) * std::sin(t) - d * std::sin((R + r) / r * t)) / maxVal;
    }
}

void generateButterfly(Pattern& pattern, int numPoints) {
    pattern.resize(numPoints);

    float maxVal = 0.0f;
    // First pass to find max for normalization
    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / numPoints * 12.0f * static_cast<float>(M_PI);
        float r = std::exp(std::cos(t)) - 2.0f * std::cos(4.0f * t) - std::pow(std::sin(t / 12.0f), 5);
        float x = std::sin(t) * r;
        float y = std::cos(t) * r;
        float val = std::max(std::abs(x), std::abs(y));
        if (val > maxVal) maxVal = val;
    }

    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / numPoints * 12.0f * static_cast<float>(M_PI);
        float r = std::exp(std::cos(t)) - 2.0f * std::cos(4.0f * t) - std::pow(std::sin(t / 12.0f), 5);
        pattern.x[i] = std::sin(t) * r / maxVal;
        pattern.y[i] = std::cos(t) * r / maxVal;
    }
}

void generateCardioid(Pattern& pattern, int numPoints) {
    pattern.resize(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / numPoints * 2.0f * static_cast<float>(M_PI);
        float r = 1.0f - std::cos(t);
        pattern.x[i] = r * std::cos(t) * 0.5f;
        pattern.y[i] = r * std::sin(t) * 0.5f;
    }
}

void generateDeltoid(Pattern& pattern, int numPoints) {
    pattern.resize(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / numPoints * 2.0f * static_cast<float>(M_PI);
        pattern.x[i] = (2.0f * std::cos(t) + std::cos(2.0f * t)) / 3.0f;
        pattern.y[i] = (2.0f * std::sin(t) - std::sin(2.0f * t)) / 3.0f;
    }
}

//==============================================================================
// Special Shapes
//==============================================================================

void generateFigure8(Pattern& pattern, int numPoints) {
    pattern.resize(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / numPoints * 2.0f * static_cast<float>(M_PI);
        pattern.x[i] = std::sin(t);
        pattern.y[i] = std::sin(2.0f * t);
    }
}

void generateInfinity(Pattern& pattern, int numPoints) {
    pattern.resize(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / numPoints * 2.0f * static_cast<float>(M_PI);
        pattern.x[i] = std::cos(t);
        pattern.y[i] = std::sin(2.0f * t);
    }
}

void generateHeart(Pattern& pattern, int numPoints) {
    pattern.resize(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / numPoints * 2.0f * static_cast<float>(M_PI);

        // Parametric heart curve
        pattern.x[i] = 16.0f * std::pow(std::sin(t), 3) / 17.0f;
        pattern.y[i] = (13.0f * std::cos(t) - 5.0f * std::cos(2.0f*t) - 2.0f * std::cos(3.0f*t) - std::cos(4.0f*t)) / 17.0f;
    }
}

//==============================================================================
// Waveforms
//==============================================================================

void generateSquareWave(Pattern& pattern, int numPoints, float frequency) {
    pattern.resize(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / (numPoints - 1);
        pattern.x[i] = t * 2.0f - 1.0f;

        // Square wave using sign of sine
        float sineVal = std::sin(frequency * 2.0f * static_cast<float>(M_PI) * t);
        pattern.y[i] = (sineVal >= 0) ? 1.0f : -1.0f;
    }
}

void generateSawtoothWave(Pattern& pattern, int numPoints, float frequency) {
    pattern.resize(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / (numPoints - 1);
        pattern.x[i] = t * 2.0f - 1.0f;

        // Sawtooth wave
        float phase = std::fmod(frequency * t, 1.0f);
        pattern.y[i] = 2.0f * phase - 1.0f;
    }
}

void generateTriangleWave(Pattern& pattern, int numPoints, float frequency) {
    pattern.resize(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / (numPoints - 1);
        pattern.x[i] = t * 2.0f - 1.0f;

        // Triangle wave
        float phase = std::fmod(frequency * t, 1.0f);
        pattern.y[i] = 4.0f * std::abs(phase - 0.5f) - 1.0f;
    }
}

} // namespace generators
} // namespace oscilloplot

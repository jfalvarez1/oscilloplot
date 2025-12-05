#include "echo.hpp"
#include <algorithm>

namespace oscilloplot {
namespace effects {

void EchoEffect::process(Pattern& pattern, float time, float deltaTime) {
    (void)time;
    (void)deltaTime;

    if (pattern.empty() || m_echoCount <= 0) return;

    size_t originalSize = pattern.size();
    size_t delaySamples = static_cast<size_t>(originalSize * m_delayPercent);

    if (delaySamples == 0) return;

    // Store original pattern
    std::vector<float> origX = pattern.x;
    std::vector<float> origY = pattern.y;

    // Add echoes
    float amplitude = 1.0f;
    for (int echo = 1; echo <= m_echoCount; ++echo) {
        amplitude *= m_decay;
        size_t offset = echo * delaySamples;

        for (size_t i = 0; i < originalSize; ++i) {
            size_t srcIdx = (i + originalSize - offset) % originalSize;
            pattern.x[i] += origX[srcIdx] * amplitude;
            pattern.y[i] += origY[srcIdx] * amplitude;
        }
    }

    // Normalize to prevent clipping
    float maxVal = 0.0f;
    for (size_t i = 0; i < pattern.size(); ++i) {
        maxVal = std::max(maxVal, std::abs(pattern.x[i]));
        maxVal = std::max(maxVal, std::abs(pattern.y[i]));
    }

    if (maxVal > 1.0f) {
        for (size_t i = 0; i < pattern.size(); ++i) {
            pattern.x[i] /= maxVal;
            pattern.y[i] /= maxVal;
        }
    }
}

void EchoEffect::reset() {
    // No state to reset
}

} // namespace effects
} // namespace oscilloplot

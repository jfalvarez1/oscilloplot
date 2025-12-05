#include "kaleidoscope.hpp"
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace oscilloplot {
namespace effects {

void KaleidoscopeEffect::process(Pattern& pattern, float time, float deltaTime) {
    (void)time;

    if (pattern.empty() || m_segments < 2) return;

    m_rotation += m_rotationSpeed * deltaTime;

    // Store original pattern
    std::vector<float> origX = pattern.x;
    std::vector<float> origY = pattern.y;
    size_t origSize = pattern.size();

    // Expand pattern to include all segments
    pattern.resize(origSize * m_segments);

    float segmentAngle = 2.0f * M_PI / m_segments;

    for (int seg = 0; seg < m_segments; ++seg) {
        float angle = seg * segmentAngle + m_rotation;
        float cosA = std::cos(angle);
        float sinA = std::sin(angle);

        // Mirror every other segment
        float mirrorX = (seg % 2 == 0) ? 1.0f : -1.0f;

        for (size_t i = 0; i < origSize; ++i) {
            float x = origX[i] * mirrorX;
            float y = origY[i];

            // Rotate
            float rx = x * cosA - y * sinA;
            float ry = x * sinA + y * cosA;

            size_t idx = seg * origSize + i;
            pattern.x[idx] = rx;
            pattern.y[idx] = ry;
        }
    }
}

} // namespace effects
} // namespace oscilloplot

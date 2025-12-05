#include "rotation.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace oscilloplot {
namespace effects {

void RotationEffect::process(Pattern& pattern, float time, float deltaTime) {
    (void)time;

    if (m_mode == Mode::Off || pattern.empty()) return;

    float angleDeg;
    switch (m_mode) {
        case Mode::Static:
            angleDeg = m_staticAngle;
            break;
        case Mode::Clockwise:
            m_currentAngle += m_speed * deltaTime;
            angleDeg = m_currentAngle;
            break;
        case Mode::CounterClockwise:
            m_currentAngle -= m_speed * deltaTime;
            angleDeg = m_currentAngle;
            break;
        default:
            return;
    }

    float angleRad = angleDeg * M_PI / 180.0f;
    float cosA = std::cos(angleRad);
    float sinA = std::sin(angleRad);

    for (size_t i = 0; i < pattern.size(); ++i) {
        float x = pattern.x[i];
        float y = pattern.y[i];
        pattern.x[i] = x * cosA - y * sinA;
        pattern.y[i] = x * sinA + y * cosA;
    }
}

void RotationEffect::reset() {
    m_currentAngle = 0.0f;
}

} // namespace effects
} // namespace oscilloplot

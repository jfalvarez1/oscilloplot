#include "fade.hpp"
#include <algorithm>

namespace oscilloplot {
namespace effects {

void FadeEffect::process(Pattern& pattern, float time, float deltaTime) {
    (void)time;
    (void)deltaTime;

    if (pattern.empty()) return;

    // Calculate fade factors
    float xScale = 1.0f;
    float yScale = 1.0f;

    if (m_shrinkEnabled && m_shrinkSteps > 1) {
        float t = static_cast<float>(m_currentStep % (m_shrinkSteps * 2));
        if (t >= m_shrinkSteps) t = m_shrinkSteps * 2 - t;
        float scale = t / m_shrinkSteps;
        xScale *= scale;
        yScale *= scale;
    }

    if (m_xFadeEnabled && m_xFadeSteps > 1) {
        float t = static_cast<float>(m_currentStep % (m_xFadeSteps * 2));
        if (t >= m_xFadeSteps) t = m_xFadeSteps * 2 - t;
        xScale *= t / m_xFadeSteps;
    }

    if (m_yFadeEnabled && m_yFadeSteps > 1) {
        float t = static_cast<float>(m_currentStep % (m_yFadeSteps * 2));
        if (t >= m_yFadeSteps) t = m_yFadeSteps * 2 - t;
        yScale *= t / m_yFadeSteps;
    }

    // Apply scaling
    for (size_t i = 0; i < pattern.size(); ++i) {
        pattern.x[i] *= xScale;
        pattern.y[i] *= yScale;
    }

    // Advance step counter
    ++m_repeatCounter;
    int speed = std::max({m_xFadeSpeed, m_yFadeSpeed, m_shrinkSpeed});
    if (m_repeatCounter >= speed) {
        m_repeatCounter = 0;
        ++m_currentStep;
    }
}

void FadeEffect::reset() {
    m_currentStep = 0;
    m_repeatCounter = 0;
}

} // namespace effects
} // namespace oscilloplot

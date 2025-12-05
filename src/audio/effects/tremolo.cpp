#include "tremolo.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace oscilloplot {
namespace effects {

void TremoloEffect::process(Pattern& pattern, float time, float deltaTime) {
    (void)deltaTime;

    if (pattern.empty()) return;

    float phase = time * m_rate * 2.0f * M_PI;
    float modValue;

    switch (m_waveform) {
        case Waveform::Sine:
            modValue = (std::sin(phase) + 1.0f) / 2.0f;
            break;
        case Waveform::Triangle:
            modValue = std::abs(std::fmod(time * m_rate * 2.0f, 2.0f) - 1.0f);
            break;
        case Waveform::Square:
            modValue = (std::sin(phase) >= 0.0f) ? 1.0f : 0.0f;
            break;
        default:
            modValue = 1.0f;
    }

    float amplitude = 1.0f - m_depth * (1.0f - modValue);

    for (size_t i = 0; i < pattern.size(); ++i) {
        pattern.x[i] *= amplitude;
        pattern.y[i] *= amplitude;
    }
}

} // namespace effects
} // namespace oscilloplot

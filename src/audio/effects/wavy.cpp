#include "wavy.hpp"
#include <cmath>

namespace oscilloplot {
namespace effects {

void WavyEffect::process(Pattern& pattern, float time, float deltaTime) {
    (void)deltaTime;

    if (pattern.empty()) return;

    for (size_t i = 0; i < pattern.size(); ++i) {
        float t = time + static_cast<float>(i) / pattern.size();

        if (m_xWavyEnabled) {
            pattern.x[i] += m_xAmp * std::sin(m_xFreq * t);
        }
        if (m_yWavyEnabled) {
            pattern.y[i] += m_yAmp * std::sin(m_yFreq * t);
        }
    }
}

} // namespace effects
} // namespace oscilloplot

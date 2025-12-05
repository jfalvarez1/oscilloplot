#include "noise.hpp"

namespace oscilloplot {
namespace effects {

void NoiseEffect::process(Pattern& pattern, float time, float deltaTime) {
    (void)time;
    (void)deltaTime;

    if (pattern.empty()) return;

    for (size_t i = 0; i < pattern.size(); ++i) {
        if (m_xNoiseEnabled) {
            pattern.x[i] += m_dist(m_rng) * m_xNoiseAmp;
        }
        if (m_yNoiseEnabled) {
            pattern.y[i] += m_dist(m_rng) * m_yNoiseAmp;
        }
    }
}

} // namespace effects
} // namespace oscilloplot

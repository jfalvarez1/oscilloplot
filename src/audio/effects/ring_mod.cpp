#include "ring_mod.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace oscilloplot {
namespace effects {

void RingModEffect::process(Pattern& pattern, float time, float deltaTime) {
    (void)deltaTime;

    if (pattern.empty()) return;

    for (size_t i = 0; i < pattern.size(); ++i) {
        float t = time + static_cast<float>(i) / pattern.size();
        float carrier = std::sin(2.0f * M_PI * m_carrierFreq * t);

        float modX = pattern.x[i] * carrier;
        float modY = pattern.y[i] * carrier;

        pattern.x[i] = pattern.x[i] * (1.0f - m_mix) + modX * m_mix;
        pattern.y[i] = pattern.y[i] * (1.0f - m_mix) + modY * m_mix;
    }
}

} // namespace effects
} // namespace oscilloplot

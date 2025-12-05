#include "harmonics.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace oscilloplot {
namespace generators {

void generateHarmonics(Pattern& pattern, int numPoints,
                       const std::vector<Harmonic>& harmonics,
                       float fundamentalFreq) {
    pattern.resize(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / numPoints * 2.0f * M_PI;
        float x = 0.0f;
        float y = 0.0f;

        for (const auto& h : harmonics) {
            float freq = fundamentalFreq * h.harmonic;
            x += h.amplitudeX * std::sin(freq * t + h.phaseX);
            y += h.amplitudeY * std::sin(freq * t + h.phaseY);
        }

        pattern.x[i] = x;
        pattern.y[i] = y;
    }

    pattern.normalize();
}

void generateXYHarmonics(Pattern& pattern, int numPoints,
                         const std::vector<Harmonic>& xHarmonics,
                         const std::vector<Harmonic>& yHarmonics) {
    pattern.resize(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / numPoints * 2.0f * M_PI;
        float x = 0.0f;
        float y = 0.0f;

        for (const auto& h : xHarmonics) {
            x += h.amplitudeX * std::sin(h.harmonic * t + h.phaseX);
        }

        for (const auto& h : yHarmonics) {
            y += h.amplitudeY * std::sin(h.harmonic * t + h.phaseY);
        }

        pattern.x[i] = x;
        pattern.y[i] = y;
    }

    pattern.normalize();
}

} // namespace generators
} // namespace oscilloplot

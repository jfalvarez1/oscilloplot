#include "random_harmonics.hpp"
#include <cmath>
#include <random>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace oscilloplot {
namespace generators {

void generateRandomHarmonics(Pattern& pattern, int numPoints,
                             int numHarmonics, unsigned int seed) {
    pattern.resize(numPoints);

    std::mt19937 rng(seed == 0 ? std::random_device{}() : seed);
    std::uniform_real_distribution<float> ampDist(0.1f, 1.0f);
    std::uniform_real_distribution<float> phaseDist(0.0f, 2.0f * M_PI);
    std::uniform_int_distribution<int> harmDist(1, 8);

    struct RandomHarmonic {
        int harmonic;
        float ampX, ampY;
        float phaseX, phaseY;
    };

    std::vector<RandomHarmonic> harmonics(numHarmonics);
    for (auto& h : harmonics) {
        h.harmonic = harmDist(rng);
        h.ampX = ampDist(rng);
        h.ampY = ampDist(rng);
        h.phaseX = phaseDist(rng);
        h.phaseY = phaseDist(rng);
    }

    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / numPoints * 2.0f * M_PI;
        float x = 0.0f;
        float y = 0.0f;

        for (const auto& h : harmonics) {
            x += h.ampX * std::sin(h.harmonic * t + h.phaseX);
            y += h.ampY * std::sin(h.harmonic * t + h.phaseY);
        }

        pattern.x[i] = x;
        pattern.y[i] = y;
    }

    pattern.normalize();
}

} // namespace generators
} // namespace oscilloplot

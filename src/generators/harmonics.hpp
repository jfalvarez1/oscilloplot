#pragma once

#include "data/pattern.hpp"
#include <vector>

namespace oscilloplot {
namespace generators {

struct Harmonic {
    int harmonic;      // Harmonic number (1 = fundamental)
    float amplitudeX;  // Amplitude for X channel
    float amplitudeY;  // Amplitude for Y channel
    float phaseX;      // Phase offset for X (radians)
    float phaseY;      // Phase offset for Y (radians)
};

// Generate pattern from sum of harmonics
void generateHarmonics(Pattern& pattern, int numPoints,
                       const std::vector<Harmonic>& harmonics,
                       float fundamentalFreq = 1.0f);

// Generate Lissajous-like pattern from X and Y harmonic series
void generateXYHarmonics(Pattern& pattern, int numPoints,
                         const std::vector<Harmonic>& xHarmonics,
                         const std::vector<Harmonic>& yHarmonics);

} // namespace generators
} // namespace oscilloplot

#pragma once

#include "data/pattern.hpp"
#include <string>
#include <vector>

namespace oscilloplot {

class WavExport {
public:
    // Export pattern as stereo WAV (X = left channel, Y = right channel)
    static bool exportToWav(const std::string& filename,
                           const std::vector<float>& audioBuffer,
                           int sampleRate);

    // Export pattern directly
    static bool exportPattern(const std::string& filename,
                             const Pattern& pattern,
                             int sampleRate,
                             int repeats);
};

} // namespace oscilloplot

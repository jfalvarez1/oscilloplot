#include "wav_export.hpp"
#include <sndfile.h>
#include <iostream>

namespace oscilloplot {

bool WavExport::exportToWav(const std::string& filename,
                           const std::vector<float>& audioBuffer,
                           int sampleRate) {
    SF_INFO sfinfo;
    sfinfo.channels = 2;
    sfinfo.samplerate = sampleRate;
    sfinfo.format = SF_FORMAT_WAV | SF_FORMAT_FLOAT;

    SNDFILE* file = sf_open(filename.c_str(), SFM_WRITE, &sfinfo);
    if (!file) {
        std::cerr << "Failed to open WAV file: " << sf_strerror(nullptr) << std::endl;
        return false;
    }

    sf_count_t written = sf_write_float(file, audioBuffer.data(),
                                        static_cast<sf_count_t>(audioBuffer.size()));

    sf_close(file);

    return written == static_cast<sf_count_t>(audioBuffer.size());
}

bool WavExport::exportPattern(const std::string& filename,
                             const Pattern& pattern,
                             int sampleRate,
                             int repeats) {
    if (pattern.empty()) return false;

    // Build interleaved stereo buffer
    size_t samplesPerPattern = pattern.size();
    size_t totalSamples = samplesPerPattern * repeats;
    std::vector<float> buffer(totalSamples * 2);

    for (int r = 0; r < repeats; ++r) {
        for (size_t i = 0; i < samplesPerPattern; ++i) {
            size_t idx = (r * samplesPerPattern + i) * 2;
            buffer[idx] = pattern.x[i];
            buffer[idx + 1] = pattern.y[i];
        }
    }

    return exportToWav(filename, buffer, sampleRate);
}

} // namespace oscilloplot

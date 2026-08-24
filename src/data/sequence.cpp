#include "sequence.hpp"

#include <cstdio>
#include <fstream>
#include <sstream>

namespace oscilloplot {

namespace {

// Sample a pattern at normalised position p in [0,1] with linear interpolation
// between neighbouring points.
inline void sampleAt(const Pattern& p, float u, float& outX, float& outY) {
    if (p.empty()) { outX = outY = 0.0f; return; }
    if (p.size() == 1) { outX = p.x[0]; outY = p.y[0]; return; }

    float pos = u * static_cast<float>(p.size() - 1);
    size_t i0 = static_cast<size_t>(pos);
    if (i0 >= p.size() - 1) { outX = p.x.back(); outY = p.y.back(); return; }

    float frac = pos - static_cast<float>(i0);
    outX = p.x[i0] + frac * (p.x[i0 + 1] - p.x[i0]);
    outY = p.y[i0] + frac * (p.y[i0 + 1] - p.y[i0]);
}

} // namespace

void blendPatterns(const Pattern& from, const Pattern& to, float t, Pattern& out) {
    if (t <= 0.0f) { out = from; return; }
    if (t >= 1.0f) { out = to; return; }

    if (from.empty()) { out = to; return; }
    if (to.empty())   { out = from; return; }

    const size_t n = (from.size() > to.size()) ? from.size() : to.size();
    out.clear();
    out.reserve(n);

    const float inv = (n > 1) ? 1.0f / static_cast<float>(n - 1) : 0.0f;
    for (size_t i = 0; i < n; ++i) {
        const float u = static_cast<float>(i) * inv;
        float ax, ay, bx, by;
        sampleAt(from, u, ax, ay);
        sampleAt(to,   u, bx, by);
        out.push_back(ax + (bx - ax) * t, ay + (by - ay) * t);
    }
}

bool saveSequence(const std::string& path, const Sequence& seq, std::string& error) {
    error.clear();
    if (seq.steps.empty()) {
        error = "Sequence is empty";
        return false;
    }

    std::ofstream f(path);
    if (!f.is_open()) {
        error = "Could not create file";
        return false;
    }

    // Version 2 adds the crossfade field. Version 1 files still load.
    f << "OSEQ 2\n";
    f << "loop " << (seq.loop ? 1 : 0) << "\n";
    f << "crossfade " << seq.crossfade << "\n";
    f << "steps " << seq.steps.size() << "\n";
    for (const auto& step : seq.steps) {
        f << "step " << step.cycles << " " << step.pattern.size()
          << " " << step.name << "\n";
        for (size_t i = 0; i < step.pattern.size(); ++i) {
            f << step.pattern.x[i] << " " << step.pattern.y[i] << "\n";
        }
    }

    if (!f.good()) {
        error = "Write failed";
        return false;
    }
    return true;
}

bool loadSequence(const std::string& path, Sequence& seq, std::string& error) {
    error.clear();

    std::ifstream f(path);
    if (!f.is_open()) {
        error = "Could not open file";
        return false;
    }

    std::string magic;
    int version = 0;
    if (!(f >> magic >> version) || magic != "OSEQ" || version < 1 || version > 2) {
        error = "Not an Oscilloplot sequence file";
        return false;
    }

    Sequence loaded;
    std::string key;
    int loopFlag = 1;

    if (!(f >> key >> loopFlag) || key != "loop") {
        error = "Malformed header";
        return false;
    }
    loaded.loop = (loopFlag != 0);

    if (version >= 2) {
        float cf = 0.0f;
        if (!(f >> key >> cf) || key != "crossfade") {
            error = "Malformed header";
            return false;
        }
        loaded.crossfade = cf;
    }

    size_t stepCount = 0;
    if (!(f >> key >> stepCount) || key != "steps" || stepCount > MAX_SEQUENCE_STEPS) {
        error = "Malformed step count";
        return false;
    }

    for (size_t s = 0; s < stepCount; ++s) {
        SequenceStep step;
        size_t npoints = 0;
        if (!(f >> key >> step.cycles >> npoints) || key != "step") {
            error = "Malformed step header";
            return false;
        }
        if (npoints > MAX_PATTERN_POINTS) {
            error = "Step has too many points";
            return false;
        }

        // Rest of the line is the (possibly spaced) step name.
        std::string nameRest;
        std::getline(f, nameRest);
        size_t begin = nameRest.find_first_not_of(" \t");
        if (begin != std::string::npos) nameRest = nameRest.substr(begin);
        // Trim trailing CR from CRLF files read in text mode on non-Windows
        while (!nameRest.empty() &&
               (nameRest.back() == '\r' || nameRest.back() == '\n')) {
            nameRest.pop_back();
        }
        snprintf(step.name, sizeof(step.name), "%s",
                 nameRest.empty() ? "Step" : nameRest.c_str());

        step.pattern.reserve(npoints);
        for (size_t i = 0; i < npoints; ++i) {
            float x, y;
            if (!(f >> x >> y)) {
                error = "File is truncated";
                return false;
            }
            step.pattern.push_back(x, y);
        }
        loaded.steps.push_back(std::move(step));
    }

    seq = std::move(loaded);
    return true;
}

} // namespace oscilloplot

#pragma once

#include "pattern.hpp"

#include <string>
#include <vector>

namespace oscilloplot {

//==============================================================================
// Pattern sequence: an ordered list of patterns, each held for a number of
// playback cycles, optionally cross-faded into the next.
//
// Kept free of any UI dependency so the format can be unit-tested and reused.
//==============================================================================

struct SequenceStep {
    Pattern pattern;
    int cycles = 200;                 // Playback repetitions before advancing
    char name[32] = "Step";
};

struct Sequence {
    std::vector<SequenceStep> steps;
    bool loop = true;
    float crossfade = 0.0f;           // Fraction of a step spent blending (0-0.5)

    bool empty() const { return steps.empty(); }
    size_t size() const { return steps.size(); }
};

//------------------------------------------------------------------------------
// Blend two patterns. `t` runs 0 (all `from`) to 1 (all `to`).
//
// The two patterns rarely share a point count, so both are sampled by
// normalised position rather than index; the result takes the larger size so
// no detail is dropped from either side.
//------------------------------------------------------------------------------
void blendPatterns(const Pattern& from, const Pattern& to, float t, Pattern& out);

//------------------------------------------------------------------------------
// .oseq serialisation. Returns false and fills `error` on failure.
//------------------------------------------------------------------------------
bool saveSequence(const std::string& path, const Sequence& seq, std::string& error);
bool loadSequence(const std::string& path, Sequence& seq, std::string& error);

// Maximum steps accepted from a file, to reject malformed input early.
constexpr size_t MAX_SEQUENCE_STEPS = 10000;

} // namespace oscilloplot

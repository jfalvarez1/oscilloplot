#pragma once

#include "pattern.hpp"

#include <string>
#include <vector>

namespace oscilloplot {

struct EffectParams;

//==============================================================================
// A preset captures a pattern together with the effect settings that were
// applied to it, so a whole look can be recalled in one action. Effects
// otherwise reset to defaults on every launch.
//
// Stored as key/value text: unknown keys are ignored on load, so presets
// written by a newer build still open in an older one.
//==============================================================================

struct Preset {
    std::string name;
    Pattern pattern;

    // Effect settings as flat key/value pairs. Kept generic rather than a
    // mirror of EffectParams so adding an effect does not invalidate old files.
    std::vector<std::pair<std::string, float>> effectValues;

    float valueOr(const std::string& key, float fallback) const;
    void set(const std::string& key, float value);
};

// Copy effect settings between a live EffectParams and a preset.
void captureEffects(const EffectParams& params, Preset& preset);
void applyEffects(const Preset& preset, EffectParams& params);

bool savePreset(const std::string& path, const Preset& preset, std::string& error);
bool loadPreset(const std::string& path, Preset& preset, std::string& error);

} // namespace oscilloplot

#include "preset.hpp"
#include "audio/audio_engine.hpp"

#include <fstream>
#include <sstream>

namespace oscilloplot {

float Preset::valueOr(const std::string& key, float fallback) const {
    for (const auto& kv : effectValues) {
        if (kv.first == key) return kv.second;
    }
    return fallback;
}

void Preset::set(const std::string& key, float value) {
    for (auto& kv : effectValues) {
        if (kv.first == key) { kv.second = value; return; }
    }
    effectValues.emplace_back(key, value);
}

//------------------------------------------------------------------------------
// The capture/apply pair below is the one place that must be kept in step with
// EffectParams. Anything missing simply keeps its default on load.
//------------------------------------------------------------------------------

#define CAPTURE_F(field) preset.set(#field, params.field.load())
#define CAPTURE_B(field) preset.set(#field, params.field.load() ? 1.0f : 0.0f)
#define CAPTURE_E(field) preset.set(#field, static_cast<float>(static_cast<int>(params.field.load())))

void captureEffects(const EffectParams& params, Preset& preset) {
    preset.effectValues.clear();

    CAPTURE_E(rotationMode);
    CAPTURE_F(rotationAngle);
    CAPTURE_F(rotationSpeed);

    CAPTURE_B(fadeXEnabled);   CAPTURE_B(fadeYEnabled);   CAPTURE_B(shrinkEnabled);
    CAPTURE_F(fadeXSteps);     CAPTURE_F(fadeYSteps);     CAPTURE_F(shrinkSteps);
    CAPTURE_F(fadeXSpeed);     CAPTURE_F(fadeYSpeed);     CAPTURE_F(shrinkSpeed);
    CAPTURE_B(alternateXYFade);

    CAPTURE_B(noiseXEnabled);  CAPTURE_B(noiseYEnabled);
    CAPTURE_F(noiseXAmount);   CAPTURE_F(noiseYAmount);

    CAPTURE_B(wavyXEnabled);   CAPTURE_B(wavyYEnabled);
    CAPTURE_F(wavyXAmplitude); CAPTURE_F(wavyYAmplitude);
    CAPTURE_F(wavyXFrequency); CAPTURE_F(wavyYFrequency);

    CAPTURE_B(tremoloEnabled); CAPTURE_F(tremoloDepth); CAPTURE_F(tremoloRate);
    CAPTURE_E(tremoloWaveform);

    CAPTURE_B(ringModEnabled); CAPTURE_F(ringModFreq); CAPTURE_F(ringModMix);

    CAPTURE_B(mirrorX); CAPTURE_B(mirrorY); CAPTURE_B(mirrorXY);

    CAPTURE_B(echoEnabled); CAPTURE_F(echoCount); CAPTURE_F(echoDecay); CAPTURE_F(echoDelay);

    CAPTURE_B(kaleidoscopeEnabled); CAPTURE_F(kaleidoscopeSections);
    CAPTURE_B(kaleidoscopeMirror);  CAPTURE_F(kaleidoscopeRotation);

    CAPTURE_B(distortionEnabled); CAPTURE_E(distortionType);
    CAPTURE_F(distortionThreshold); CAPTURE_F(distortionDrive);
}

#define APPLY_F(field, def) params.field.store(preset.valueOr(#field, def))
#define APPLY_B(field, def) params.field.store(preset.valueOr(#field, (def) ? 1.0f : 0.0f) != 0.0f)
#define APPLY_I(field, type, def) \
    params.field.store(static_cast<type>(static_cast<int>(preset.valueOr(#field, static_cast<float>(def)))))

void applyEffects(const Preset& preset, EffectParams& params) {
    APPLY_I(rotationMode, EffectParams::RotationMode, 0);
    APPLY_F(rotationAngle, 0.0f);
    APPLY_F(rotationSpeed, 5.0f);

    APPLY_B(fadeXEnabled, false);  APPLY_B(fadeYEnabled, false);  APPLY_B(shrinkEnabled, false);
    APPLY_I(fadeXSteps, uint16_t, 10); APPLY_I(fadeYSteps, uint16_t, 10);
    APPLY_I(shrinkSteps, uint16_t, 10);
    APPLY_I(fadeXSpeed, uint16_t, 1);  APPLY_I(fadeYSpeed, uint16_t, 1);
    APPLY_I(shrinkSpeed, uint16_t, 1);
    APPLY_B(alternateXYFade, false);

    APPLY_B(noiseXEnabled, false); APPLY_B(noiseYEnabled, false);
    APPLY_F(noiseXAmount, 0.05f);  APPLY_F(noiseYAmount, 0.05f);

    APPLY_B(wavyXEnabled, false);  APPLY_B(wavyYEnabled, false);
    APPLY_F(wavyXAmplitude, 0.2f); APPLY_F(wavyYAmplitude, 0.2f);
    APPLY_F(wavyXFrequency, 3.0f); APPLY_F(wavyYFrequency, 3.0f);

    APPLY_B(tremoloEnabled, false); APPLY_F(tremoloDepth, 0.5f); APPLY_F(tremoloRate, 2.0f);
    APPLY_I(tremoloWaveform, EffectParams::TremoloWave, 0);

    APPLY_B(ringModEnabled, false); APPLY_F(ringModFreq, 200.0f); APPLY_F(ringModMix, 0.5f);

    APPLY_B(mirrorX, false); APPLY_B(mirrorY, false); APPLY_B(mirrorXY, false);

    APPLY_B(echoEnabled, false); APPLY_I(echoCount, uint8_t, 3);
    APPLY_F(echoDecay, 0.5f);    APPLY_F(echoDelay, 0.2f);

    APPLY_B(kaleidoscopeEnabled, false); APPLY_I(kaleidoscopeSections, uint8_t, 6);
    APPLY_B(kaleidoscopeMirror, true);   APPLY_F(kaleidoscopeRotation, 0.0f);

    APPLY_B(distortionEnabled, false);
    APPLY_I(distortionType, EffectParams::DistortionType, 0);
    APPLY_F(distortionThreshold, 0.8f); APPLY_F(distortionDrive, 1.5f);
}

//------------------------------------------------------------------------------

bool savePreset(const std::string& path, const Preset& preset, std::string& error) {
    error.clear();

    std::ofstream f(path);
    if (!f.is_open()) {
        error = "Could not create file";
        return false;
    }

    f << "OPRESET 1\n";
    f << "name " << (preset.name.empty() ? "Preset" : preset.name) << "\n";
    f << "effects " << preset.effectValues.size() << "\n";
    for (const auto& kv : preset.effectValues) {
        f << kv.first << " " << kv.second << "\n";
    }
    f << "points " << preset.pattern.size() << "\n";
    for (size_t i = 0; i < preset.pattern.size(); ++i) {
        f << preset.pattern.x[i] << " " << preset.pattern.y[i] << "\n";
    }

    if (!f.good()) {
        error = "Write failed";
        return false;
    }
    return true;
}

bool loadPreset(const std::string& path, Preset& preset, std::string& error) {
    error.clear();

    std::ifstream f(path);
    if (!f.is_open()) {
        error = "Could not open file";
        return false;
    }

    std::string magic;
    int version = 0;
    if (!(f >> magic >> version) || magic != "OPRESET" || version != 1) {
        error = "Not an Oscilloplot preset file";
        return false;
    }

    Preset loaded;
    std::string key;

    if (!(f >> key) || key != "name") { error = "Malformed header"; return false; }
    std::string nameRest;
    std::getline(f, nameRest);
    size_t begin = nameRest.find_first_not_of(" \t");
    if (begin != std::string::npos) nameRest = nameRest.substr(begin);
    while (!nameRest.empty() && (nameRest.back() == '\r' || nameRest.back() == '\n')) {
        nameRest.pop_back();
    }
    loaded.name = nameRest.empty() ? "Preset" : nameRest;

    size_t effectCount = 0;
    if (!(f >> key >> effectCount) || key != "effects" || effectCount > 4096) {
        error = "Malformed effect count";
        return false;
    }
    for (size_t i = 0; i < effectCount; ++i) {
        std::string k; float v;
        if (!(f >> k >> v)) { error = "File is truncated"; return false; }
        loaded.set(k, v);
    }

    size_t npoints = 0;
    if (!(f >> key >> npoints) || key != "points" || npoints > MAX_PATTERN_POINTS) {
        error = "Malformed point count";
        return false;
    }
    loaded.pattern.reserve(npoints);
    for (size_t i = 0; i < npoints; ++i) {
        float x, y;
        if (!(f >> x >> y)) { error = "File is truncated"; return false; }
        loaded.pattern.push_back(x, y);
    }

    preset = std::move(loaded);
    return true;
}

} // namespace oscilloplot

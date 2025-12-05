#pragma once

#include "effect_base.hpp"
#include <random>

namespace oscilloplot {
namespace effects {

class NoiseEffect : public Effect {
public:
    std::string getName() const override { return "Noise"; }

    void process(Pattern& pattern, float time, float deltaTime) override;

    void setXNoiseEnabled(bool enabled) { m_xNoiseEnabled = enabled; }
    void setXNoiseAmplitude(float amp) { m_xNoiseAmp = amp; }

    void setYNoiseEnabled(bool enabled) { m_yNoiseEnabled = enabled; }
    void setYNoiseAmplitude(float amp) { m_yNoiseAmp = amp; }

private:
    bool m_xNoiseEnabled = false;
    float m_xNoiseAmp = 0.05f;

    bool m_yNoiseEnabled = false;
    float m_yNoiseAmp = 0.05f;

    std::mt19937 m_rng{std::random_device{}()};
    std::uniform_real_distribution<float> m_dist{-1.0f, 1.0f};
};

} // namespace effects
} // namespace oscilloplot

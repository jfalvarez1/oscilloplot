#pragma once

#include "effect_base.hpp"

namespace oscilloplot {
namespace effects {

class WavyEffect : public Effect {
public:
    std::string getName() const override { return "Wavy"; }

    void process(Pattern& pattern, float time, float deltaTime) override;

    void setXWavyEnabled(bool enabled) { m_xWavyEnabled = enabled; }
    void setXAmplitude(float k) { m_xAmp = k; }
    void setXFrequency(float omega) { m_xFreq = omega; }

    void setYWavyEnabled(bool enabled) { m_yWavyEnabled = enabled; }
    void setYAmplitude(float k) { m_yAmp = k; }
    void setYFrequency(float omega) { m_yFreq = omega; }

private:
    bool m_xWavyEnabled = false;
    float m_xAmp = 0.2f;
    float m_xFreq = 10.0f;

    bool m_yWavyEnabled = false;
    float m_yAmp = 0.2f;
    float m_yFreq = 10.0f;
};

} // namespace effects
} // namespace oscilloplot

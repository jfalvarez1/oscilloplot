#pragma once

#include "effect_base.hpp"

namespace oscilloplot {
namespace effects {

class TremoloEffect : public Effect {
public:
    enum class Waveform { Sine, Triangle, Square };

    std::string getName() const override { return "Tremolo"; }

    void process(Pattern& pattern, float time, float deltaTime) override;

    void setDepth(float percent) { m_depth = percent / 100.0f; }
    float getDepth() const { return m_depth * 100.0f; }

    void setRate(float hz) { m_rate = hz; }
    float getRate() const { return m_rate; }

    void setWaveform(Waveform wf) { m_waveform = wf; }
    Waveform getWaveform() const { return m_waveform; }

private:
    float m_depth = 0.5f;   // 0-1
    float m_rate = 2.0f;    // Hz
    Waveform m_waveform = Waveform::Sine;
};

} // namespace effects
} // namespace oscilloplot

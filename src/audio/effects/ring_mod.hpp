#pragma once

#include "effect_base.hpp"

namespace oscilloplot {
namespace effects {

class RingModEffect : public Effect {
public:
    std::string getName() const override { return "RingMod"; }

    void process(Pattern& pattern, float time, float deltaTime) override;

    void setCarrierFrequency(float hz) { m_carrierFreq = hz; }
    float getCarrierFrequency() const { return m_carrierFreq; }

    void setMix(float percent) { m_mix = percent / 100.0f; }
    float getMix() const { return m_mix * 100.0f; }

private:
    float m_carrierFreq = 200.0f;
    float m_mix = 0.5f;
};

} // namespace effects
} // namespace oscilloplot

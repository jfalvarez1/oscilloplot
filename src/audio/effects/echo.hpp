#pragma once

#include "effect_base.hpp"
#include <vector>

namespace oscilloplot {
namespace effects {

class EchoEffect : public Effect {
public:
    std::string getName() const override { return "Echo"; }

    void process(Pattern& pattern, float time, float deltaTime) override;
    void reset() override;

    void setEchoCount(int count) { m_echoCount = count; }
    int getEchoCount() const { return m_echoCount; }

    void setDecay(float decay) { m_decay = decay; }
    float getDecay() const { return m_decay; }

    void setDelayPercent(float percent) { m_delayPercent = percent / 100.0f; }
    float getDelayPercent() const { return m_delayPercent * 100.0f; }

private:
    int m_echoCount = 3;
    float m_decay = 0.7f;
    float m_delayPercent = 0.1f;  // 10% of pattern length
};

} // namespace effects
} // namespace oscilloplot

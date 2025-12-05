#pragma once

#include "effect_base.hpp"

namespace oscilloplot {
namespace effects {

class FadeEffect : public Effect {
public:
    std::string getName() const override { return "Fade"; }

    void process(Pattern& pattern, float time, float deltaTime) override;
    void reset() override;

    // X fade
    void setXFadeEnabled(bool enabled) { m_xFadeEnabled = enabled; }
    void setXFadeSteps(int steps) { m_xFadeSteps = steps; }
    void setXFadeSpeed(int repeatsPerStep) { m_xFadeSpeed = repeatsPerStep; }

    // Y fade
    void setYFadeEnabled(bool enabled) { m_yFadeEnabled = enabled; }
    void setYFadeSteps(int steps) { m_yFadeSteps = steps; }
    void setYFadeSpeed(int repeatsPerStep) { m_yFadeSpeed = repeatsPerStep; }

    // Shrink (both axes together)
    void setShrinkEnabled(bool enabled) { m_shrinkEnabled = enabled; }
    void setShrinkSteps(int steps) { m_shrinkSteps = steps; }
    void setShrinkSpeed(int repeatsPerStep) { m_shrinkSpeed = repeatsPerStep; }

    // Alternate X/Y
    void setAlternateXY(bool enabled) { m_alternateXY = enabled; }

private:
    bool m_xFadeEnabled = false;
    int m_xFadeSteps = 10;
    int m_xFadeSpeed = 1;

    bool m_yFadeEnabled = false;
    int m_yFadeSteps = 10;
    int m_yFadeSpeed = 1;

    bool m_shrinkEnabled = false;
    int m_shrinkSteps = 10;
    int m_shrinkSpeed = 1;

    bool m_alternateXY = false;

    int m_currentStep = 0;
    int m_repeatCounter = 0;
};

} // namespace effects
} // namespace oscilloplot

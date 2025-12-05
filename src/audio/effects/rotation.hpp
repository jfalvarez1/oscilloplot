#pragma once

#include "effect_base.hpp"

namespace oscilloplot {
namespace effects {

class RotationEffect : public Effect {
public:
    enum class Mode { Off, Static, Clockwise, CounterClockwise };

    std::string getName() const override { return "Rotation"; }

    void process(Pattern& pattern, float time, float deltaTime) override;
    void reset() override;

    void setMode(Mode mode) { m_mode = mode; }
    Mode getMode() const { return m_mode; }

    void setStaticAngle(float degrees) { m_staticAngle = degrees; }
    float getStaticAngle() const { return m_staticAngle; }

    void setSpeed(float degreesPerCycle) { m_speed = degreesPerCycle; }
    float getSpeed() const { return m_speed; }

private:
    Mode m_mode = Mode::Off;
    float m_staticAngle = 0.0f;  // degrees
    float m_speed = 5.0f;        // degrees per cycle
    float m_currentAngle = 0.0f;
};

} // namespace effects
} // namespace oscilloplot

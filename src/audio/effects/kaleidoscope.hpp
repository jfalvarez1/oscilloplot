#pragma once

#include "effect_base.hpp"

namespace oscilloplot {
namespace effects {

class KaleidoscopeEffect : public Effect {
public:
    std::string getName() const override { return "Kaleidoscope"; }

    void process(Pattern& pattern, float time, float deltaTime) override;

    void setSegments(int segments) { m_segments = segments; }
    int getSegments() const { return m_segments; }

    void setRotationSpeed(float speed) { m_rotationSpeed = speed; }
    float getRotationSpeed() const { return m_rotationSpeed; }

private:
    int m_segments = 6;
    float m_rotationSpeed = 0.0f;
    float m_rotation = 0.0f;
};

} // namespace effects
} // namespace oscilloplot

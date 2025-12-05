#pragma once

#include "effect_base.hpp"
#include <vector>
#include <memory>

namespace oscilloplot {
namespace effects {

class EffectChain {
public:
    void addEffect(std::unique_ptr<Effect> effect);
    void removeEffect(const std::string& name);
    void clear();

    Effect* getEffect(const std::string& name);

    void process(Pattern& pattern, float time, float deltaTime);
    void reset();

    const std::vector<std::unique_ptr<Effect>>& getEffects() const { return m_effects; }

private:
    std::vector<std::unique_ptr<Effect>> m_effects;
};

} // namespace effects
} // namespace oscilloplot

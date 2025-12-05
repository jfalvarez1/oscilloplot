#include "effect_chain.hpp"
#include <algorithm>

namespace oscilloplot {
namespace effects {

void EffectChain::addEffect(std::unique_ptr<Effect> effect) {
    m_effects.push_back(std::move(effect));
}

void EffectChain::removeEffect(const std::string& name) {
    m_effects.erase(
        std::remove_if(m_effects.begin(), m_effects.end(),
            [&name](const auto& e) { return e->getName() == name; }),
        m_effects.end()
    );
}

void EffectChain::clear() {
    m_effects.clear();
}

Effect* EffectChain::getEffect(const std::string& name) {
    for (auto& effect : m_effects) {
        if (effect->getName() == name) {
            return effect.get();
        }
    }
    return nullptr;
}

void EffectChain::process(Pattern& pattern, float time, float deltaTime) {
    for (auto& effect : m_effects) {
        if (effect->isEnabled()) {
            effect->process(pattern, time, deltaTime);
        }
    }
}

void EffectChain::reset() {
    for (auto& effect : m_effects) {
        effect->reset();
    }
}

} // namespace effects
} // namespace oscilloplot

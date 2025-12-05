#pragma once

#include "data/pattern.hpp"
#include <string>

namespace oscilloplot {
namespace effects {

// Base class for all effects
class Effect {
public:
    virtual ~Effect() = default;

    virtual std::string getName() const = 0;
    virtual bool isEnabled() const { return m_enabled; }
    virtual void setEnabled(bool enabled) { m_enabled = enabled; }

    // Process pattern in-place
    virtual void process(Pattern& pattern, float time, float deltaTime) = 0;

    // Reset effect state
    virtual void reset() {}

protected:
    bool m_enabled = false;
};

} // namespace effects
} // namespace oscilloplot

#pragma once

#include "data/pattern.hpp"
#include <vector>

namespace oscilloplot {

class DrawingCanvas {
public:
    void render();

    bool isOpen() const { return m_isOpen; }
    void open() { m_isOpen = true; }
    void close() { m_isOpen = false; }

    // Get the drawn pattern
    const Pattern& getPattern() const { return m_pattern; }
    void clearPattern();

private:
    bool m_isOpen = false;
    Pattern m_pattern;
    bool m_isDrawing = false;
};

} // namespace oscilloplot

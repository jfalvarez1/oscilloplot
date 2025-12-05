#pragma once

#include "data/pattern.hpp"

namespace oscilloplot {

class SoundPad {
public:
    void render();

    bool isOpen() const { return m_isOpen; }
    void open() { m_isOpen = true; }
    void close() { m_isOpen = false; }

    const Pattern& getPattern() const { return m_pattern; }

private:
    bool m_isOpen = false;
    Pattern m_pattern;

    int m_gridRows = 4;
    int m_gridCols = 4;
    int m_activeCell = -1;
};

} // namespace oscilloplot

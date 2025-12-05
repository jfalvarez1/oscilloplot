#pragma once

#include "pattern.hpp"
#include <string>

namespace oscilloplot {

class FileLoader {
public:
    // Load pattern from text file (X,Y pairs per line)
    static bool loadTextFile(const std::string& filename, Pattern& pattern);

    // Load pattern from MATLAB-like .m file (simple array extraction)
    static bool loadMatlabFile(const std::string& filename, Pattern& pattern);

    // Load pattern from binary format
    static bool loadBinaryFile(const std::string& filename, Pattern& pattern);
};

} // namespace oscilloplot

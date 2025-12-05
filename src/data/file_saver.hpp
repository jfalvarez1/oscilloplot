#pragma once

#include "pattern.hpp"
#include <string>

namespace oscilloplot {

class FileSaver {
public:
    // Save pattern to text file
    static bool saveTextFile(const std::string& filename, const Pattern& pattern);

    // Save pattern to binary format
    static bool saveBinaryFile(const std::string& filename, const Pattern& pattern);
};

} // namespace oscilloplot

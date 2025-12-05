#include "file_saver.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>

namespace oscilloplot {

bool FileSaver::saveTextFile(const std::string& filename, const Pattern& pattern) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to create file: " << filename << std::endl;
        return false;
    }

    file << "# Oscilloplot pattern file\n";
    file << "# Format: X, Y\n";
    file << std::fixed << std::setprecision(6);

    for (size_t i = 0; i < pattern.size(); ++i) {
        file << pattern.x[i] << ", " << pattern.y[i] << "\n";
    }

    return file.good();
}

bool FileSaver::saveBinaryFile(const std::string& filename, const Pattern& pattern) {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Failed to create file: " << filename << std::endl;
        return false;
    }

    // Write header
    file.write("OSCP", 4);  // Magic bytes

    uint32_t version = 1;
    uint32_t numPoints = static_cast<uint32_t>(pattern.size());

    file.write(reinterpret_cast<const char*>(&version), sizeof(version));
    file.write(reinterpret_cast<const char*>(&numPoints), sizeof(numPoints));

    // Write data
    file.write(reinterpret_cast<const char*>(pattern.xData()), numPoints * sizeof(float));
    file.write(reinterpret_cast<const char*>(pattern.yData()), numPoints * sizeof(float));

    return file.good();
}

} // namespace oscilloplot

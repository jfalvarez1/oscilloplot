#include "file_loader.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <regex>

namespace oscilloplot {

bool FileLoader::loadTextFile(const std::string& filename, Pattern& pattern) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return false;
    }

    pattern.clear();
    std::string line;

    while (std::getline(file, line)) {
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        float x, y;

        // Try comma-separated
        char comma;
        if (iss >> x >> comma >> y && comma == ',') {
            pattern.push_back(x, y);
        }
        // Try space-separated
        else {
            iss.clear();
            iss.str(line);
            if (iss >> x >> y) {
                pattern.push_back(x, y);
            }
        }
    }

    if (pattern.empty()) {
        std::cerr << "No valid data points found in file" << std::endl;
        return false;
    }

    pattern.normalize();
    return true;
}

bool FileLoader::loadMatlabFile(const std::string& filename, Pattern& pattern) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return false;
    }

    std::string content((std::istreambuf_iterator<char>(file)),
                        std::istreambuf_iterator<char>());

    // Simple regex to find array assignments like: x = [1, 2, 3, ...];
    std::regex arrayRegex(R"((\w+)\s*=\s*\[([\d\s,.\-e+]+)\])");
    std::smatch match;

    std::vector<float> xData, yData;

    std::string::const_iterator searchStart(content.cbegin());
    while (std::regex_search(searchStart, content.cend(), match, arrayRegex)) {
        std::string varName = match[1];
        std::string values = match[2];

        // Parse values
        std::vector<float> data;
        std::istringstream iss(values);
        std::string token;
        while (std::getline(iss, token, ',')) {
            try {
                data.push_back(std::stof(token));
            } catch (...) {
                // Skip invalid values
            }
        }

        // Assign to x or y based on variable name
        if (varName == "x" || varName == "X") {
            xData = data;
        } else if (varName == "y" || varName == "Y") {
            yData = data;
        }

        searchStart = match.suffix().first;
    }

    if (xData.empty() || yData.empty()) {
        std::cerr << "Could not find x and y arrays in MATLAB file" << std::endl;
        return false;
    }

    size_t numPoints = std::min(xData.size(), yData.size());
    pattern.resize(numPoints);

    for (size_t i = 0; i < numPoints; ++i) {
        pattern.x[i] = xData[i];
        pattern.y[i] = yData[i];
    }

    pattern.normalize();
    return true;
}

bool FileLoader::loadBinaryFile(const std::string& filename, Pattern& pattern) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return false;
    }

    // Read header
    char magic[4];
    file.read(magic, 4);
    if (std::string(magic, 4) != "OSCP") {
        std::cerr << "Invalid file format" << std::endl;
        return false;
    }

    uint32_t version, numPoints;
    file.read(reinterpret_cast<char*>(&version), sizeof(version));
    file.read(reinterpret_cast<char*>(&numPoints), sizeof(numPoints));

    pattern.resize(numPoints);
    file.read(reinterpret_cast<char*>(pattern.xData()), numPoints * sizeof(float));
    file.read(reinterpret_cast<char*>(pattern.yData()), numPoints * sizeof(float));

    return file.good();
}

} // namespace oscilloplot

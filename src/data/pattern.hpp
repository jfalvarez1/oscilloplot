#pragma once

#include <vector>
#include <cstddef>

namespace oscilloplot {

struct Pattern {
    std::vector<float> x;
    std::vector<float> y;

    Pattern() = default;

    Pattern(size_t size) : x(size), y(size) {}

    void resize(size_t size) {
        x.resize(size);
        y.resize(size);
    }

    void clear() {
        x.clear();
        y.clear();
    }

    void reserve(size_t size) {
        x.reserve(size);
        y.reserve(size);
    }

    void push_back(float px, float py) {
        x.push_back(px);
        y.push_back(py);
    }

    size_t size() const { return x.size(); }
    bool empty() const { return x.empty(); }

    float* xData() { return x.data(); }
    float* yData() { return y.data(); }
    const float* xData() const { return x.data(); }
    const float* yData() const { return y.data(); }

    // Normalize to [-1, 1] range
    void normalize() {
        if (empty()) return;

        float minX = x[0], maxX = x[0];
        float minY = y[0], maxY = y[0];

        for (size_t i = 1; i < size(); ++i) {
            if (x[i] < minX) minX = x[i];
            if (x[i] > maxX) maxX = x[i];
            if (y[i] < minY) minY = y[i];
            if (y[i] > maxY) maxY = y[i];
        }

        float rangeX = maxX - minX;
        float rangeY = maxY - minY;
        float maxRange = (rangeX > rangeY) ? rangeX : rangeY;

        if (maxRange > 0.0f) {
            float centerX = (minX + maxX) / 2.0f;
            float centerY = (minY + maxY) / 2.0f;
            float scale = 2.0f / maxRange;

            for (size_t i = 0; i < size(); ++i) {
                x[i] = (x[i] - centerX) * scale;
                y[i] = (y[i] - centerY) * scale;
            }
        }
    }
};

} // namespace oscilloplot

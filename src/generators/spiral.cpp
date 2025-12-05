#include "spiral.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace oscilloplot {
namespace generators {

void generateArchimedeanSpiral(Pattern& pattern, int numPoints,
                                float a, float b, float maxTheta) {
    pattern.resize(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        float theta = maxTheta * static_cast<float>(i) / (numPoints - 1);
        float r = a + b * theta;

        pattern.x[i] = r * std::cos(theta);
        pattern.y[i] = r * std::sin(theta);
    }

    pattern.normalize();
}

void generateLogarithmicSpiral(Pattern& pattern, int numPoints,
                                float a, float b, float maxTheta) {
    pattern.resize(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        float theta = maxTheta * static_cast<float>(i) / (numPoints - 1);
        float r = a * std::exp(b * theta);

        pattern.x[i] = r * std::cos(theta);
        pattern.y[i] = r * std::sin(theta);
    }

    pattern.normalize();
}

void generateFermatSpiral(Pattern& pattern, int numPoints,
                          float a, float maxTheta) {
    pattern.resize(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        float theta = maxTheta * static_cast<float>(i) / (numPoints - 1);
        float r = a * std::sqrt(theta);

        pattern.x[i] = r * std::cos(theta);
        pattern.y[i] = r * std::sin(theta);
    }

    pattern.normalize();
}

} // namespace generators
} // namespace oscilloplot

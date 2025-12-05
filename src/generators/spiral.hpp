#pragma once

#include "data/pattern.hpp"

namespace oscilloplot {
namespace generators {

// Generate Archimedean spiral: r = a + b*theta
void generateArchimedeanSpiral(Pattern& pattern, int numPoints,
                                float a, float b, float maxTheta);

// Generate logarithmic spiral: r = a * e^(b*theta)
void generateLogarithmicSpiral(Pattern& pattern, int numPoints,
                                float a, float b, float maxTheta);

// Generate Fermat spiral: r = a * sqrt(theta)
void generateFermatSpiral(Pattern& pattern, int numPoints,
                          float a, float maxTheta);

} // namespace generators
} // namespace oscilloplot

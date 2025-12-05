#pragma once

#include "data/pattern.hpp"

namespace oscilloplot {
namespace generators {

// Generate random harmonics pattern
void generateRandomHarmonics(Pattern& pattern, int numPoints,
                             int numHarmonics = 5,
                             unsigned int seed = 0);

} // namespace generators
} // namespace oscilloplot

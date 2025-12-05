#pragma once

#include "data/pattern.hpp"

namespace oscilloplot {
namespace generators {

//==============================================================================
// Basic Shapes
//==============================================================================

// Generate a sine wave pattern: X = t, Y = sin(cycles * 2 * PI * t)
void generateSineWave(Pattern& pattern, int numPoints, float cycles);

// Generate a circle pattern
void generateCircle(Pattern& pattern, int numPoints);

// Generate an ellipse pattern
void generateEllipse(Pattern& pattern, int numPoints, float aspectX, float aspectY);

//==============================================================================
// Lissajous Curves
//==============================================================================

// Generate a Lissajous curve: X = sin(a*t + delta), Y = sin(b*t)
void generateLissajous(Pattern& pattern, int numPoints, int a, int b, float delta);

//==============================================================================
// Stars & Flowers
//==============================================================================

// Generate a star pattern (5-point by default)
void generateStar(Pattern& pattern, int numPoints, int numSpikes, float innerRadius);

// Generate a flower pattern with petals
void generateFlower(Pattern& pattern, int numPoints, int numPetals, float petalDepth);

// Generate a rose curve (rhodonea): r = cos(k*theta)
void generateRoseCurve(Pattern& pattern, int numPoints, int k);

//==============================================================================
// Spirals
//==============================================================================

// Generate an Archimedean spiral: r = a + b*theta
void generateSpiral(Pattern& pattern, int numPoints, float turns, float startRadius, float endRadius);

// Generate a logarithmic spiral: r = a*e^(b*theta)
void generateLogSpiral(Pattern& pattern, int numPoints, float a, float b, float maxAngle);

//==============================================================================
// Knots & 3D-Style Curves
//==============================================================================

// Generate a helix (3D spiral projected to 2D)
void generateHelix(Pattern& pattern, int numPoints);

// Generate a trefoil knot
void generateTrefoilKnot(Pattern& pattern, int numPoints);

// Generate a torus knot
void generateTorusKnot(Pattern& pattern, int numPoints, int p, int q);

//==============================================================================
// Complex Curves
//==============================================================================

// Generate a hypotrochoid
void generateHypotrochoid(Pattern& pattern, int numPoints, float R, float r, float d);

// Generate an epitrochoid
void generateEpitrochoid(Pattern& pattern, int numPoints, float R, float r, float d);

// Generate a butterfly curve
void generateButterfly(Pattern& pattern, int numPoints);

// Generate a cardioid
void generateCardioid(Pattern& pattern, int numPoints);

// Generate a deltoid
void generateDeltoid(Pattern& pattern, int numPoints);

//==============================================================================
// Special Shapes
//==============================================================================

// Generate a figure-8 (lemniscate)
void generateFigure8(Pattern& pattern, int numPoints);

// Generate an infinity symbol
void generateInfinity(Pattern& pattern, int numPoints);

// Generate a heart shape
void generateHeart(Pattern& pattern, int numPoints);

//==============================================================================
// Waveforms
//==============================================================================

// Generate a square wave
void generateSquareWave(Pattern& pattern, int numPoints, float frequency);

// Generate a sawtooth wave
void generateSawtoothWave(Pattern& pattern, int numPoints, float frequency);

// Generate a triangle wave
void generateTriangleWave(Pattern& pattern, int numPoints, float frequency);

} // namespace generators
} // namespace oscilloplot

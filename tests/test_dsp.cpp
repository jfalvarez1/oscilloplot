//==============================================================================
// Tests for the DSP helpers and pattern generators. These run on the audio
// thread in production, so accuracy and range matter more than usual.
//==============================================================================

#include "test_framework.hpp"

#include "generators/test_pattern.hpp"
#include "utils/dsp_core.hpp"

#include <cmath>

using namespace oscilloplot;

namespace {

// Every generated pattern must stay inside the display, contain no NaN, and
// have the requested number of points.
void checkPatternSane(const Pattern& p, const char* what, size_t expected = 0) {
    if (p.empty()) {
        ::testing::fail(__FILE__, __LINE__, std::string(what) + " produced no points");
        return;
    }
    if (expected && p.size() != expected) {
        ::testing::fail(__FILE__, __LINE__,
            std::string(what) + " size " + std::to_string(p.size()) +
            " != " + std::to_string(expected));
    }
    for (size_t i = 0; i < p.size(); ++i) {
        if (!std::isfinite(p.x[i]) || !std::isfinite(p.y[i])) {
            ::testing::fail(__FILE__, __LINE__,
                std::string(what) + " has a non-finite point at " + std::to_string(i));
            return;
        }
        if (std::fabs(p.x[i]) > 2.0f || std::fabs(p.y[i]) > 2.0f) {
            ::testing::fail(__FILE__, __LINE__,
                std::string(what) + " point " + std::to_string(i) +
                " is far outside the display: (" + std::to_string(p.x[i]) +
                ", " + std::to_string(p.y[i]) + ")");
            return;
        }
    }
}

} // namespace

//==============================================================================
// Sine lookup table
//==============================================================================

TEST(SineLUT, matches_std_sin_within_table_resolution) {
    double worst = 0.0;
    for (int i = 0; i < 2000; ++i) {
        float phase = static_cast<float>(i) / 2000.0f;      // [0, 1)
        float got = dsp::SineLUT::sin(phase);
        double want = std::sin(phase * 6.28318530718);
        worst = std::fmax(worst, std::fabs(got - want));
    }
    // Linear interpolation over the table; a few thousandths is expected.
    CHECK(worst < 0.01);
}

TEST(SineLUT, wraps_phase_outside_unit_range) {
    // Negative and >1 phases must fold back, not read out of bounds.
    CHECK_NEAR(dsp::SineLUT::sin(0.25f), dsp::SineLUT::sin(1.25f), 1e-5);
    CHECK_NEAR(dsp::SineLUT::sin(0.25f), dsp::SineLUT::sin(-0.75f), 1e-5);
    CHECK_NEAR(dsp::SineLUT::sin(0.0f),  dsp::SineLUT::sin(5.0f),  1e-5);
}

TEST(SineLUT, stays_in_range_across_the_cycle) {
    for (int i = -500; i < 1500; ++i) {
        float v = dsp::SineLUT::sin(static_cast<float>(i) / 500.0f);
        CHECK(v >= -1.001f && v <= 1.001f);
        CHECK(std::isfinite(v));
    }
}

TEST(SineLUT, cos_leads_sin_by_a_quarter_turn) {
    for (int i = 0; i < 100; ++i) {
        float phase = static_cast<float>(i) / 100.0f;
        CHECK_NEAR(dsp::SineLUT::cos(phase),
                   dsp::SineLUT::sin(phase + 0.25f), 1e-4);
    }
}

//==============================================================================
// Generators
//==============================================================================

TEST(Generators, circle_is_a_unit_circle) {
    Pattern p;
    generators::generateCircle(p, 360);
    checkPatternSane(p, "circle", 360);
    for (size_t i = 0; i < p.size(); ++i) {
        double r = std::sqrt(p.x[i] * p.x[i] + p.y[i] * p.y[i]);
        CHECK_NEAR(r, r, 0.0);            // finite
        CHECK(r > 0.1 && r < 1.5);
    }
}

TEST(Generators, sine_wave_spans_the_display) {
    Pattern p;
    generators::generateSineWave(p, 500, 3.0f);
    checkPatternSane(p, "sine wave", 500);

    float minY = p.y[0], maxY = p.y[0];
    for (size_t i = 0; i < p.size(); ++i) {
        minY = std::fmin(minY, p.y[i]);
        maxY = std::fmax(maxY, p.y[i]);
    }
    CHECK(maxY - minY > 1.0f);            // it actually oscillates
}

TEST(Generators, lissajous_is_bounded) {
    Pattern p;
    generators::generateLissajous(p, 1000, 3, 2, 0.0f);
    checkPatternSane(p, "lissajous", 1000);
}

TEST(Generators, spiral_grows_outward) {
    Pattern p;
    generators::generateSpiral(p, 800, 5.0f, 0.1f, 1.0f);
    checkPatternSane(p, "spiral", 800);

    double rFirst = std::sqrt(p.x[0] * p.x[0] + p.y[0] * p.y[0]);
    double rLast  = std::sqrt(p.x[p.size()-1] * p.x[p.size()-1] +
                              p.y[p.size()-1] * p.y[p.size()-1]);
    CHECK(rLast > rFirst);
}

TEST(Generators, star_and_flower_are_sane) {
    Pattern star, flower, rose;
    generators::generateStar(star, 600, 5, 0.5f);
    generators::generateFlower(flower, 600, 6, 0.5f);
    generators::generateRoseCurve(rose, 600, 4);
    checkPatternSane(star, "star", 600);
    checkPatternSane(flower, "flower", 600);
    checkPatternSane(rose, "rose", 600);
}

TEST(Generators, knots_and_helix_are_sane) {
    Pattern helix, trefoil, torus;
    generators::generateHelix(helix, 500);
    generators::generateTrefoilKnot(trefoil, 500);
    generators::generateTorusKnot(torus, 500, 2, 3);
    checkPatternSane(helix, "helix", 500);
    checkPatternSane(trefoil, "trefoil", 500);
    checkPatternSane(torus, "torus knot", 500);
}

TEST(Generators, trochoids_are_sane) {
    Pattern hypo, epi;
    generators::generateHypotrochoid(hypo, 800, 5.0f, 3.0f, 5.0f);
    generators::generateEpitrochoid(epi, 800, 5.0f, 2.0f, 2.0f);
    checkPatternSane(hypo, "hypotrochoid", 800);
    checkPatternSane(epi, "epitrochoid", 800);
}

TEST(Generators, special_shapes_are_sane) {
    Pattern fig8, inf, heart, butterfly, cardioid, deltoid;
    generators::generateFigure8(fig8, 400);
    generators::generateInfinity(inf, 400);
    generators::generateHeart(heart, 400);
    generators::generateButterfly(butterfly, 2000);
    generators::generateCardioid(cardioid, 400);
    generators::generateDeltoid(deltoid, 400);
    checkPatternSane(fig8, "figure-8", 400);
    checkPatternSane(inf, "infinity", 400);
    checkPatternSane(heart, "heart", 400);
    checkPatternSane(butterfly, "butterfly", 2000);
    checkPatternSane(cardioid, "cardioid", 400);
    checkPatternSane(deltoid, "deltoid", 400);
}

TEST(Generators, waveforms_are_sane) {
    Pattern sq, saw, tri;
    generators::generateSquareWave(sq, 500, 3.0f);
    generators::generateSawtoothWave(saw, 500, 3.0f);
    generators::generateTriangleWave(tri, 500, 3.0f);
    checkPatternSane(sq, "square", 500);
    checkPatternSane(saw, "sawtooth", 500);
    checkPatternSane(tri, "triangle", 500);
}

TEST(Generators, tiny_point_counts_do_not_crash) {
    // The UI clamps to 100, but nothing stops a preset or file from asking for
    // less; the generators must not divide by zero or read out of bounds.
    for (int n : {1, 2, 3}) {
        Pattern p;
        generators::generateCircle(p, n);
        for (size_t i = 0; i < p.size(); ++i) {
            CHECK(std::isfinite(p.x[i]));
            CHECK(std::isfinite(p.y[i]));
        }
    }
}

//==============================================================================
// Screen curvature
//
// The transform lives on UIManager, but the maths is a plain barrel
// distortion; this reproduces it so the shape of the curve stays pinned.
//==============================================================================

namespace {
void barrel(float k, float x, float y, float& ox, float& oy) {
    const float r2 = x * x + y * y;
    const float f = 1.0f + k * r2;
    ox = x * f;
    oy = y * f;
}
}

TEST(Curvature, centre_point_does_not_move) {
    float ox, oy;
    barrel(0.35f, 0.0f, 0.0f, ox, oy);
    CHECK_NEAR(ox, 0.0, 1e-6);
    CHECK_NEAR(oy, 0.0, 1e-6);
}

TEST(Curvature, edges_move_outward_more_than_the_middle) {
    float nearX, nearY, farX, farY;
    barrel(0.35f, 0.25f, 0.0f, nearX, nearY);
    barrel(0.35f, 1.00f, 0.0f, farX,  farY);

    const double nearShift = nearX - 0.25;
    const double farShift  = farX  - 1.00;
    CHECK(nearShift > 0.0);
    CHECK(farShift > nearShift * 4.0);   // grows with r^2, not linearly
}

TEST(Curvature, is_symmetric_about_both_axes) {
    float ax, ay, bx, by;
    barrel(0.35f,  0.6f,  0.4f, ax, ay);
    barrel(0.35f, -0.6f, -0.4f, bx, by);
    CHECK_NEAR(ax, -bx, 1e-6);
    CHECK_NEAR(ay, -by, 1e-6);
}

TEST(Curvature, zero_strength_is_identity) {
    float ox, oy;
    barrel(0.0f, 0.73f, -0.41f, ox, oy);
    CHECK_NEAR(ox,  0.73, 1e-6);
    CHECK_NEAR(oy, -0.41, 1e-6);
}

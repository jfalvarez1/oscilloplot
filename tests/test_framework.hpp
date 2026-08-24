#pragma once
//==============================================================================
// Minimal test framework
//
// Deliberately dependency-free: adding Catch2/GTest would pull another package
// into vcpkg and Homebrew and lengthen every CI run, for a suite this size.
// Register cases with TEST(suite, name), assert with CHECK/CHECK_EQ/etc.
//==============================================================================

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace testing {

struct TestCase {
    std::string suite;
    std::string name;
    std::function<void()> fn;
};

inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> cases;
    return cases;
}

// Failures accumulate per-case so one case can report several problems.
inline std::vector<std::string>& failures() {
    static std::vector<std::string> f;
    return f;
}

struct Registrar {
    Registrar(const char* suite, const char* name, std::function<void()> fn) {
        registry().push_back({suite, name, std::move(fn)});
    }
};

inline void fail(const char* file, int line, const std::string& msg) {
    char buf[1024];
    snprintf(buf, sizeof(buf), "    %s:%d: %s", file, line, msg.c_str());
    failures().push_back(buf);
}

inline bool nearlyEqual(double a, double b, double tol) {
    return std::fabs(a - b) <= tol;
}

inline int runAll(const char* filter) {
    int passed = 0, failed = 0;
    std::string lastSuite;

    for (const auto& tc : registry()) {
        if (filter && *filter && tc.suite.find(filter) == std::string::npos &&
            tc.name.find(filter) == std::string::npos) {
            continue;
        }
        if (tc.suite != lastSuite) {
            printf("\n[%s]\n", tc.suite.c_str());
            lastSuite = tc.suite;
        }

        failures().clear();
        tc.fn();

        if (failures().empty()) {
            printf("  PASS  %s\n", tc.name.c_str());
            ++passed;
        } else {
            printf("  FAIL  %s\n", tc.name.c_str());
            for (const auto& f : failures()) printf("%s\n", f.c_str());
            ++failed;
        }
    }

    printf("\n===============================\n");
    printf("  %d passed, %d failed, %d total\n", passed, failed, passed + failed);
    printf("===============================\n");
    return failed == 0 ? 0 : 1;
}

} // namespace testing

#define TEST(suite_, name_)                                                      \
    static void test_##suite_##_##name_();                                       \
    static ::testing::Registrar reg_##suite_##_##name_(                          \
        #suite_, #name_, test_##suite_##_##name_);                               \
    static void test_##suite_##_##name_()

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) ::testing::fail(__FILE__, __LINE__, "CHECK(" #cond ")");     \
    } while (0)

#define CHECK_EQ(a, b)                                                           \
    do {                                                                         \
        auto _va = (a); auto _vb = (b);                                          \
        if (!(_va == _vb)) {                                                     \
            ::testing::fail(__FILE__, __LINE__,                                  \
                std::string("CHECK_EQ(" #a ", " #b ") -> ") +                    \
                std::to_string(_va) + " != " + std::to_string(_vb));             \
        }                                                                        \
    } while (0)

#define CHECK_STR_EQ(a, b)                                                       \
    do {                                                                         \
        std::string _va = (a); std::string _vb = (b);                            \
        if (_va != _vb) {                                                        \
            ::testing::fail(__FILE__, __LINE__,                                  \
                std::string("CHECK_STR_EQ(" #a ", " #b ") -> '") +               \
                _va + "' != '" + _vb + "'");                                     \
        }                                                                        \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                    \
    do {                                                                         \
        double _va = (a), _vb = (b);                                             \
        if (!::testing::nearlyEqual(_va, _vb, (tol))) {                          \
            ::testing::fail(__FILE__, __LINE__,                                  \
                std::string("CHECK_NEAR(" #a ", " #b ") -> ") +                  \
                std::to_string(_va) + " vs " + std::to_string(_vb));             \
        }                                                                        \
    } while (0)

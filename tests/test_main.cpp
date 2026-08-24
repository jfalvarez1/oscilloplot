#include "test_framework.hpp"

#include <cstring>

int main(int argc, char** argv) {
    const char* filter = (argc > 1) ? argv[1] : "";
    printf("Oscilloplot test suite\n");
    if (filter && *filter) printf("filter: %s\n", filter);
    return ::testing::runAll(filter);
}

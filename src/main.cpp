#include "app.hpp"
#include <SDL.h>
#include <iostream>

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    try {
        oscilloplot::App app;

        if (!app.init()) {
            std::cerr << "Failed to initialize application" << std::endl;
            return 1;
        }

        app.run();
        app.shutdown();

    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

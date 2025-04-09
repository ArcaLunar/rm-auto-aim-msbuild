#include "debug_options.hpp"

extern DebugOptions options;

int main() {
    if (options.camera.initialization) {
        spdlog::info("Camera initialization debug is enabled.");
        return 0;
    } else {
        spdlog::info("Camera initialization debug is disabled.");
        return 1;
    }
}
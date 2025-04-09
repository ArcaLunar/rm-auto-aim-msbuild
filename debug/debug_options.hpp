#ifndef __DEBUG_OPTIONS_HPP__
#define __DEBUG_OPTIONS_HPP__

#include "spdlog/spdlog.h"
#include "toml++/toml.h"
#include <stdexcept>

struct DebugOptions {
    // *======== Debug Config Area ========* //
    struct {
        bool initialization = false;
        bool capture        = false;
    } camera;

    DebugOptions() {
        try {
            toml::table config = toml::parse_file("../config/debug.toml");

            // * init for camera debugging
            camera.initialization = config["camera"]["initialization"].value_or(false);
            camera.capture        = config["camera"]["capture"].value_or(false);
        } catch (const toml::parse_error &err) {
            spdlog::info("Error parsing debug.toml: {}", err.description());
            throw std::runtime_error("Failed to parse debug.toml. Please check the path or the file.");
        }
    }
};

#endif
#ifndef __DEBUG_OPTIONS_HPP__
#define __DEBUG_OPTIONS_HPP__

#include "spdlog/spdlog.h"
#include "toml++/toml.h"
#include <atomic>
#include <stdexcept>

struct DebugOptions {
    // *======== Debug Config Area ========* //
    std::atomic_bool spd_timer = true;

    struct {
        std::atomic_bool initialization = true;
        std::atomic_bool capture        = true;
    } camera;

    struct {
        std::atomic_bool initialization = true;
        std::atomic_bool inspect_data   = true;
    } port;

    struct {
        std::atomic_bool show_detail = true;
    } lightbar;

    struct {
        std::atomic_bool show_detail      = true;
        std::atomic_bool procedure_detail = true;
    } armor;

    struct {
        std::atomic_bool init         = true;
        std::atomic_bool show_roi     = true;
        std::atomic_bool model_output = true;
        std::atomic_bool show_ignore  = true;
    } classifier;

    struct {
        std::atomic_bool init             = true;
        std::atomic_bool display_image    = false;
        std::atomic_bool preprocess       = true;
        std::atomic_bool detect_lightbars = true;
        std::atomic_bool pair_lightbars   = true;
    } detector;

    DebugOptions() {
        try {
            toml::table config = toml::parse_file("../config/debug.toml");

            // * init for camera debugging
            camera.initialization = config["camera"]["initialization"].value_or(false);
            camera.capture        = config["camera"]["capture"].value_or(false);

            // * init for port debugging
            port.initialization = config["port"]["initialization"].value_or(false);
            port.inspect_data   = config["port"]["inspect_data"].value_or(false);

            // * init for light bar debugging
            lightbar.show_detail = config["detector"]["lightbar"]["show_detail"].value_or(false);
            // * init for armor debugging
            armor.show_detail      = config["detector"]["armor"]["show_detail"].value_or(false);
            armor.procedure_detail = config["detector"]["armor"]["procedure_detail"].value_or(false);
        } catch (const toml::parse_error &err) {
            spdlog::info("Error parsing debug.toml: {}", err.description());
            throw std::runtime_error("Failed to parse debug.toml. Please check the path or the file.");
        }
    }
};

#endif
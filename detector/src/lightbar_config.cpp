#include "structs.hpp"

#include "debug_options.hpp"
#include "spdlog/spdlog.h"

extern DebugOptions options;

LightBarConfig::LightBarConfig(std::string path) {
    if (options.lightbar.show_detail)
        spdlog::info("initializing LightBarConfig with config file: \"{}\"", path);

    try {
        auto T                     = toml::parse_file(path);
        this->min_area             = T["light_bar"]["min_area"].value_or(30.0);
        this->max_area             = T["light_bar"]["max_area"].value_or(5e4);
        this->min_solidity         = T["light_bar"]["min_solidity"].value_or(0.5);
        this->min_aspect_ratio     = T["light_bar"]["min_aspect_ratio"].value_or(1.5);
        this->max_aspect_ratio     = T["light_bar"]["max_aspect_ratio"].value_or(15);
        this->max_angle            = T["light_bar"]["max_angle"].value_or(60.0);
        this->brightness_threshold = T["threshold"]["brightness"].value_or(60);
        this->color_threshold      = T["threshold"]["color"].value_or(60);

        // 输出调试
        if (options.lightbar.show_detail) {
            spdlog::info("LightBarConfig:");
            spdlog::info("  min_area: {}", this->min_area);
            spdlog::info("  max_area: {}", this->max_area);
            spdlog::info("  min_solidity: {}", this->min_solidity);
            spdlog::info("  min_aspect_ratio: {}", this->min_aspect_ratio);
            spdlog::info("  max_aspect_ratio: {}", this->max_aspect_ratio);
            spdlog::info("  max_angle: {}", this->max_angle);
            spdlog::info("  brightness_threshold: {}", this->brightness_threshold);
            spdlog::info("  color_threshold: {}", this->color_threshold);
        }
    } catch (const toml::parse_error &e) {
        spdlog::error("Error parsing config file: \"{}\" for LightBarConfig, using fallback", e.what());
    }

    if (options.lightbar.show_detail)
        spdlog::info("LightBarConfig initialization done.");
}
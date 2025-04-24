#include "armor.hpp"

#include "debug_options.hpp"
#include "spdlog/spdlog.h"
#include "toml++/toml.hpp"

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

ArmorConfig::ArmorConfig(std::string path) {
    if (options.armor.show_detail)
        spdlog::info("initializing ArmorConfig with config file: \"{}\"", path);

    try {
        auto T                               = toml::parse_file(path);
        this->max_angle_diff                 = T["armor"]["max_angle_diff"].value_or(15.0);
        this->max_height_diff_ratio          = T["armor"]["max_height_diff_ratio"].value_or(0.6);
        this->max_Y_diff_ratio               = T["armor"]["max_Y_diff_ratio"].value_or(2.0);
        this->min_X_diff_ratio               = T["armor"]["min_X_diff_ratio"].value_or(0.5);
        this->big_armor_ratio                = T["armor"]["big_armor_ratio"].value_or(3.0);
        this->small_armor_ratio              = T["armor"]["small_armor_ratio"].value_or(1.6);
        this->min_aspect_ratio               = T["armor"]["min_aspect_ratio"].value_or(1.0);
        this->max_aspect_ratio               = T["armor"]["max_aspect_ratio"].value_or(4.5);
        this->min_area                       = T["armor"]["min_area"].value_or(100.0);
        this->max_roll_angle                 = T["armor"]["max_roll_angle"].value_or(60.0);
        this->max_light_bar_armor_area_ratio = T["armor"]["max_light_bar_armor_area_ratio"].value_or(0.5);
        this->area_normalized_base           = T["armor"]["area_normalized_base"].value_or(1000.0);
        this->sight_offset_normalized_base   = T["armor"]["sight_offset_normalized_base"].value_or(200.0);
        this->lightbar_area_ratio            = T["armor"]["lightbar_area_ratio"].value_or(5);

        // 输出调试
        if (options.armor.show_detail) {
            spdlog::info("ArmorConfig:");
            spdlog::info("  max_angle_diff: {}", this->max_angle_diff);
            spdlog::info("  max_height_diff_ratio: {}", this->max_height_diff_ratio);
            spdlog::info("  max_Y_diff_ratio: {}", this->max_Y_diff_ratio);
            spdlog::info("  min_X_diff_ratio: {}", this->min_X_diff_ratio);
            spdlog::info("  big_armor_ratio: {}", this->big_armor_ratio);
            spdlog::info("  small_armor_ratio: {}", this->small_armor_ratio);
            spdlog::info("  min_aspect_ratio: {}", this->min_aspect_ratio);
            spdlog::info("  max_aspect_ratio: {}", this->max_aspect_ratio);
            spdlog::info("  min_area: {}", this->min_area);
            spdlog::info("  max_roll_angle: {}", this->max_roll_angle);
            spdlog::info("  max_light_bar_armor_area_ratio: {}", this->max_light_bar_armor_area_ratio);
            spdlog::info("  area_normalized_base: {}", this->area_normalized_base);
            spdlog::info("  sight_offset_normalized_base: {}", this->sight_offset_normalized_base);
            spdlog::info("  lightbar_area_ratio: {}", this->lightbar_area_ratio);
        }
    } catch (const toml::parse_error &e) {
        spdlog::error("Error parsing config file \"{}\" for ArmorConfig, using fallback", e.what());
    }

    if (options.armor.show_detail)
        spdlog::info("ArmorConfig initialization done.");
}
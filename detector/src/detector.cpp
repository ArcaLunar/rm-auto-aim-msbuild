#include "detector.hpp"

//! LightBarConfig
AutoAim::LightBarConfig::LightBarConfig(std::string path) {
    try {
        auto T          = toml::parse_file(path);
        this->min_ratio = T["min_ratio"].value_or(0.1);
        this->max_ratio = T["max_ratio"].value_or(0.5);
        this->max_angle = T["max_angle"].value_or(45.0);
    } catch (const toml::parse_error &e) {
        spdlog::error("Error parsing config file: {}, using fallback", e.what());
    }
}

//! ArmorConfig
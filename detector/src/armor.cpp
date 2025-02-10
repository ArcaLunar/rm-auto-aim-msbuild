#include "armor.hpp"

#include <spdlog/spdlog.h>
#include <toml++/toml.hpp>

//! LightBar
AutoAim::LightBar::LightBar(cv::RotatedRect &rect) : cv::RotatedRect{rect} {
    cv::Point2f vertices[4];
    rect.points(vertices);
    std::copy(vertices, vertices + 4, points.begin());

    std::sort(points.begin(), points.end(), [](const cv::Point2f &a, const cv::Point2f &b) {
        return a.y < b.y;
    });
    top    = (points[0] + points[1]) / 2;
    bottom = (points[2] + points[3]) / 2;
    length = cv::norm(top - bottom);
    width  = cv::norm(points[0] - points[1]);

    tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
    tilt_angle = tilt_angle * kRadianToDegree;
}

bool AutoAim::LightBar::isValid(const LightBarConfig &config) const {
    double ratio  = length / width;
    bool ratio_ok = config.min_ratio <= ratio && ratio <= config.max_ratio;
    bool angle_ok = tilt_angle <= config.max_angle;

    bool is_light = ratio_ok && angle_ok;
    spdlog::info("LightBar(ratio: {}, angle: {}) is_light: {}", ratio, tilt_angle, is_light);

    return is_light;
}

//! LightBarConfig
AutoAim::LightBarConfig::LightBarConfig(std::string path) {
    spdlog::info("initializing LightBarConfig with config file: \"{}\"", path);
    try {
        auto T          = toml::parse_file(path);
        this->min_ratio = T["min_ratio"].value_or(0.1);
        this->max_ratio = T["max_ratio"].value_or(0.5);
        this->max_angle = T["max_angle"].value_or(45.0);
    } catch (const toml::parse_error &e) {
        spdlog::error("Error parsing config file: \"{}\" for LightBarConfig, using fallback", e.what());
    }
    spdlog::info("LightBarConfig initialization done.");
}

//! ArmorConfig
AutoAim::ArmorConfig::ArmorConfig(std::string path) {
    spdlog::info("initializing ArmorConfig with config file: \"{}\"", path);
    try {
        auto T                 = toml::parse_file(path);
        this->binary_threshold = T["binary_threshold"].value_or(100);
    } catch (const toml::parse_error &e) {
        spdlog::error("Error parsing config file \"{}\" for ArmorConfig, using fallback", e.what());
    }
    spdlog::info("ArmorConfig initializzation done.");
}

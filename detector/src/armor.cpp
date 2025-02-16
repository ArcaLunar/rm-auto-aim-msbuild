#include "armor.hpp"

#include <spdlog/spdlog.h>
#include <toml++/toml.hpp>

//! LightBar
AutoAim::LightBar::LightBar(const cv::RotatedRect &rect) : cv::RotatedRect{rect} {
    cv::Point2f vertices[4];
    rect.points(vertices);
    std::copy_n(vertices, 4, points.begin());

    std::ranges::sort(points, [](const cv::Point2f &a, const cv::Point2f &b) { return a.y < b.y; });
    top    = (points[0] + points[1]) / 2;
    bottom = (points[2] + points[3]) / 2;
    length = cv::norm(top - bottom);
    width  = cv::norm(points[0] - points[1]);

    tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
    tilt_angle = tilt_angle * kRadianToDegree;
}

bool AutoAim::LightBar::isValid(const LightBarConfig &config) const {
    double ratio        = length / width;
    const bool ratio_ok = config.min_ratio <= ratio && ratio <= config.max_ratio;
    const bool angle_ok = tilt_angle <= config.max_angle;

    bool is_light = ratio_ok && angle_ok;

    if constexpr (not SUPPRESS_VALIDATION_SPDLOG)
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

//! Armor
AutoAim::Armor::Armor(const LightBar &l1, const LightBar &l2) : left(l1), right(l2) {
    if (left.center.x > right.center.x) std::swap(left, right);
    center = (left.center + right.center) / 2;
}

bool AutoAim::Armor::isValid(const ArmorConfig &config, const LightBar &left, const LightBar &right) {
    double lightbar_length_ratio = std::min(left.length, right.length) / std::max(left.length, right.length);
    const bool is_lightbar_length_ok   = lightbar_length_ratio > config.min_light_ratio;

    // 矩形的长宽比
    const double avg_light_length = (left.length + right.length) / 2;
    double center_distance  = cv::norm(left.center - right.center) / avg_light_length;
    bool center_dist_ok
        = (config.min_small_center_distance <= center_distance && center_distance <= config.max_small_center_distance);
    center_dist_ok
        |= (config.min_large_center_distance <= center_distance && center_distance <= config.max_large_center_distance);

    // 矩形的朝向角度
    cv::Vec2f dif = left.center - right.center;
    double angle  = std::abs(std::atan2(dif[1], dif[0])) * kRadianToDegree;
    bool angle_ok = angle <= config.max_angle;

    bool is_armor = is_lightbar_length_ok && center_dist_ok && angle_ok;

    // 输出调试信息
    if constexpr (!SUPPRESS_VALIDATION_SPDLOG) {
        spdlog::info(
            "Armor(length_ratio: {}, center_distance: {}, angle: {}) is_armor: {}",
            lightbar_length_ratio,
            center_distance,
            angle,
            is_armor
        );
    }

    return is_armor;
}
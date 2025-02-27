#include "config.hpp"
#include "structs.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <toml++/toml.hpp>

// ========================================================
// Light Bar Config
// ========================================================

AutoAim::LightBarConfig::LightBarConfig(std::string path) {
    if constexpr (InitializationDebug)
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
        if constexpr (InitializationDebug)
            spdlog::info(
                "LightBarConfig(min_area: {}, max_area: {}, min_solidity: {}, min_aspect_ratio: {}, max_aspect_ratio: "
                "{}, max_angle: {})",
                min_area,
                max_area,
                min_solidity,
                min_aspect_ratio,
                max_aspect_ratio,
                max_angle
            );
    } catch (const toml::parse_error &e) {
        spdlog::error("Error parsing config file: \"{}\" for LightBarConfig, using fallback", e.what());
    }
    if constexpr (InitializationDebug)
        spdlog::info("LightBarConfig initialization done.");
}

// ========================================================
// Armor Config
// ========================================================

AutoAim::ArmorConfig::ArmorConfig(std::string path) {
    if constexpr (InitializationDebug)
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
    } catch (const toml::parse_error &e) {
        spdlog::error("Error parsing config file \"{}\" for ArmorConfig, using fallback", e.what());
    }

    if constexpr (InitializationDebug)
        spdlog::info("ArmorConfig initialization done.");
}

// ========================================================
// Light Bar
// ========================================================

AutoAim::LightBar::LightBar() = default;

AutoAim::LightBar::LightBar(const std::vector<cv::Point> &contour) {
    this->contour      = contour;
    this->ellipse      = cv::fitEllipse(contour);
    this->short_axis   = this->ellipse.size.width;
    this->long_axis    = this->ellipse.size.height;
    this->angle        = this->ellipse.angle;
    this->ellipse_area = CV_PI * short_axis * long_axis / 4;
    this->contour_area = cv::contourArea(contour);
    this->solidity     = contour_area / ellipse_area;

    while (angle >= 90)
        angle -= 180;
    while (angle < -90)
        angle += 180;

    if (angle >= 45)
        std::swap(long_axis, short_axis), angle -= 90;
    if (angle <= -45)
        std::swap(long_axis, short_axis), angle += 90;
}

AutoAim::LightBar::~LightBar() = default;

bool AutoAim::LightBar::is_valid(const LightBarConfig &config) const {
    double aspect_ratio = long_axis / short_axis;
    if (this->ellipse_area < config.min_area || this->ellipse_area > config.max_area)
        return false;
    if (this->solidity < config.min_solidity)
        return false;
    if (aspect_ratio < config.min_aspect_ratio || aspect_ratio > config.max_aspect_ratio)
        return false;
    if (std::abs(angle) > config.max_angle)
        return false;
    return true;
}

cv::Point2f AutoAim::LightBar::center() const { return ellipse.center; }

// ========================================================
// Armor
// ========================================================

void rearrange_vertices(std::vector<cv::Point2f> &dst, cv::Point2f *src1, cv::Point2f *src2) {
    std::sort(src1, src1 + 4, [](const cv::Point2f &a, const cv::Point2f &b) { return a.y < b.y; });
    std::sort(src2, src2 + 4, [](const cv::Point2f &a, const cv::Point2f &b) { return a.y < b.y; });

    std::vector<cv::Point2f> tmp;
    tmp.emplace_back((src1[0] + src1[1]) / 2.0); // 左上
    tmp.emplace_back((src2[0] + src2[1]) / 2.0); // 右上
    tmp.emplace_back((src2[2] + src2[3]) / 2.0); // 右下
    tmp.emplace_back((src1[2] + src1[3]) / 2.0); // 左下
    std::sort(tmp.begin(), tmp.end(), [](const cv::Point2f &a, const cv::Point2f &b) { return a.x < b.x; });

    dst[0] = tmp[0].y < tmp[1].y ? tmp[0] : tmp[1]; // tl
    dst[3] = tmp[0].y > tmp[1].y ? tmp[0] : tmp[1]; // bl
    dst[1] = tmp[2].y < tmp[3].y ? tmp[2] : tmp[3]; // tr
    dst[2] = tmp[2].y > tmp[3].y ? tmp[2] : tmp[3]; // br
}

AutoAim::Armor::Armor(const LightBar &l1, const LightBar &l2) : left(l1), right(l2) {
    [&] { // 处理出装甲板的四个顶点和中心
        if constexpr (DetectorDebug)
            spdlog::info("ectracting vertices from ellipse");
        cv::Point2f leftLightRectPts[4], rightLightRectPts[4];
        left.ellipse.points(leftLightRectPts);
        right.ellipse.points(rightLightRectPts);
        if constexpr (DetectorDebug)
            spdlog::info("rearanging");
        this->vertices.resize(4);
        rearrange_vertices(this->vertices, leftLightRectPts, rightLightRectPts);

        this->center = (vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4.0;
    }();
    if constexpr (DetectorDebug)
        spdlog::info("generating min_rect");
    this->min_rect  = cv::minAreaRect(this->vertices);
    double distance = cv::norm(left.center() - right.center());
    this->angle     = std::asin(std::abs(left.center().y - right.center().y) / distance) * 180.0 / CV_PI;
}

bool AutoAim::Armor::is_valid(const ArmorConfig &config) {
    // 根据 y坐标 过滤装甲板
    if constexpr (DetectorDebug)
        spdlog::info("doing ellipse bounding rect check...");
    if (this->left.ellipse.boundingRect2f().br().y < this->right.ellipse.boundingRect2f().tl().y) {
        if constexpr (DetectorDebug)
            spdlog::error("failed");
        return false;
    }
    if (this->right.ellipse.boundingRect2f().br().y < this->left.ellipse.boundingRect2f().tl().y) {
        if constexpr (DetectorDebug)
            spdlog::error("failed");
        return false;
    }
    if constexpr (DetectorDebug)
        spdlog::info("passed");

    // 根据 灯条面积比例 过滤装甲板
    if constexpr (DetectorDebug)
        spdlog::info("doing area ratio check...");
    double area_ratio = this->left.ellipse_area / this->right.ellipse_area;
    if (area_ratio > config.lightbar_area_ratio || area_ratio < 1.0 / config.lightbar_area_ratio) {
        if constexpr (DetectorDebug)
            spdlog::error("failed");
        return false;
    }
    if constexpr (DetectorDebug)
        spdlog::info("passed");

    // 根据 装甲板面积 过滤装甲板
    double armor_area = min_rect.size.area();
    if constexpr (DetectorDebug)
        spdlog::info("doing armor_area check");
    if (armor_area < config.min_area) {
        if constexpr (DetectorDebug)
            spdlog::error("armor_area check failed: armor_area = {}, min_area = {}", armor_area, config.min_area);
        return false;
    }
    if constexpr (DetectorDebug)
        spdlog::info("passed");

    // 根据 灯条面积占占装甲板面积的比值 过滤装甲板
    double lightbar_area_over_armor_area_ratio = (left.ellipse_area + right.ellipse_area) / armor_area;
    if constexpr (DetectorDebug)
        spdlog::info("doing lightbar area ratio check");
    if (lightbar_area_over_armor_area_ratio > config.max_light_bar_armor_area_ratio) {
        if constexpr (DetectorDebug)
            spdlog::error(
                "lightbar area ratio check failed: ratio = {}, max allowed = {}",
                lightbar_area_over_armor_area_ratio,
                config.max_light_bar_armor_area_ratio
            );
        return false;
    }
    if constexpr (DetectorDebug)
        spdlog::info("passed");

    // 根据 倾斜角度 过滤装甲板
    if constexpr (DetectorDebug)
        spdlog::info("doing roll angle check");
    if (std::abs(angle) > config.max_roll_angle) {
        if constexpr (DetectorDebug)
            spdlog::error("roll angle check failed: angle = {}, max allowed = {}", angle, config.max_roll_angle);
        return false;
    }
    if constexpr (DetectorDebug)
        spdlog::info("passed");

    // 根据 高度差比例 过滤装甲板
    double mean_length       = (left.long_axis + right.long_axis) / 2.0;
    double height_diff_ratio = std::abs(left.long_axis - right.long_axis) / std::max(left.long_axis, right.long_axis);
    if constexpr (DetectorDebug)
        spdlog::info("doing height difference ratio check");
    if (height_diff_ratio > config.max_height_diff_ratio) {
        if constexpr (DetectorDebug)
            spdlog::error(
                "height difference ratio check failed: ratio = {}, max allowed = {}",
                height_diff_ratio,
                config.max_height_diff_ratio
            );
        return false;
    }
    if constexpr (DetectorDebug)
        spdlog::info("passed");

    // 根据 y坐标差比例 过滤装甲板
    double y_diff_ratio = std::abs(left.center().y - right.center().y) / mean_length;
    if constexpr (DetectorDebug)
        spdlog::info("doing Y difference ratio check");
    if (y_diff_ratio > config.max_Y_diff_ratio) {
        if constexpr (DetectorDebug)
            spdlog::error(
                "Y difference ratio check failed: ratio = {}, max allowed = {}", y_diff_ratio, config.max_Y_diff_ratio
            );
        return false;
    }
    if constexpr (DetectorDebug)
        spdlog::info("passed");

    // 根据 x坐标差比例 过滤装甲板
    double x_diff_ratio = cv::norm(left.center() - right.center()) / mean_length;
    if constexpr (DetectorDebug)
        spdlog::info("doing X difference ratio check");
    if (x_diff_ratio < config.min_X_diff_ratio) {
        if constexpr (DetectorDebug)
            spdlog::error(
                "X difference ratio check failed: ratio = {}, min allowed = {}", x_diff_ratio, config.min_X_diff_ratio
            );
        return false;
    }
    if constexpr (DetectorDebug)
        spdlog::info("passed");

    // 根据 装甲板的宽高比 过滤装甲板
    double aspect_ratio = cv::norm(left.center() - right.center()) / mean_length;
    if constexpr (DetectorDebug)
        spdlog::info("doing armor aspect ratio check");
    if (aspect_ratio < config.min_aspect_ratio || aspect_ratio > config.max_aspect_ratio) {
        if constexpr (DetectorDebug)
            spdlog::error(
                "armor aspect ratio check failed: ratio = {}, allowed range = [{}, {}]",
                aspect_ratio,
                config.min_aspect_ratio,
                config.max_aspect_ratio
            );
        return false;
    }
    if constexpr (DetectorDebug)
        spdlog::info("passed");

    // 根据 灯条的角度差 过滤装甲板
    double angle_diff = std::abs(left.angle - right.angle);
    if (angle_diff > 180)
        angle_diff -= 180;
    else if (angle_diff > 170)
        angle_diff = 180 - angle_diff;
    if constexpr (DetectorDebug)
        spdlog::info("doing lightbar angle difference check");
    if (angle_diff > config.max_angle_diff) {
        if constexpr (DetectorDebug)
            spdlog::error(
                "lightbar angle difference check failed: angle_diff = {}, max allowed = {}",
                angle_diff,
                config.max_angle_diff
            );
        return false;
    }
    if constexpr (DetectorDebug)
        spdlog::info("passed");

    // 对装甲板进行分类
    if (aspect_ratio > config.big_armor_ratio)
        this->type = ArmorType::Large;
    else
        this->type = ArmorType::Small;

    return true;
}
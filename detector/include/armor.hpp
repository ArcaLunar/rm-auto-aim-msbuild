#ifndef ARMOR_HPP_
#define ARMOR_HPP

#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

#include <config.hpp>

namespace AutoAim {

struct LightBarConfig {
    double min_ratio, max_ratio;
    double max_angle;

    explicit LightBarConfig(std::string path = "../config/detection_tr.toml");
};

struct ArmorConfig {
    int binary_threshold;
    double min_light_ratio;
    double min_small_center_distance, max_small_center_distance;
    double min_large_center_distance, max_large_center_distance;
    double max_angle;

    explicit ArmorConfig(std::string path = "../config/detection_tr.toml");
};

/**
 * @brief 灯条 class
 * @remark chenjunnn/rm_auto_aim
 */
struct LightBar : public cv::RotatedRect {
    double length, width;            // 灯条的长度和宽度
    double tilt_angle;               // 倾斜角度
    std::vector<cv::Point2f> points; // 矩形的四个顶点
    cv::Point2f top, bottom;
    std::string color;

    // 从配置文件中读取灯条的配置
    explicit LightBar() = default;
    explicit LightBar(const cv::RotatedRect &rect);

    /// 判断灯条是否合法
    bool isValid(const LightBarConfig &config) const;
};

/**
 * @brief 装甲板 class
 */
struct Armor {
    //* 灯条
    LightBar left, right; // 左右灯条
    cv::Point2f center;   // 装甲板中心

    //* 装甲板上的数字/图案
    cv::Mat number_img;               // 用于识别数字的装甲板图像
    double confidence;                // 识别的置信度
    std::string classification_result; // 识别的结果
    std::string number;               // 识别的数字

    Armor() = default;
    explicit Armor(const LightBar &l1, const LightBar &l2);

    // 判断装甲板是否合法
    static bool isValid(const ArmorConfig &config, const LightBar& left, const LightBar& right);
};

} // namespace AutoAim

#endif // __ARMOR_HPP__
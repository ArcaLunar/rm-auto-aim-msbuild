#ifndef __ARMOR_HPP__
#define __ARMOR_HPP__

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

    LightBarConfig(std::string path = "../config/detection_tr.toml");
};

struct ArmorConfig {
    int binary_threshold;

    ArmorConfig(std::string path = "../config/detection_tr.toml");
};

/**
 * !! 灯条 class
 * @ref chenjunnn/rm_auto_aim
 */
struct LightBar : public cv::RotatedRect {
    double length, width;            // 灯条的长度和宽度
    double tilt_angle;               // 倾斜角度
    std::vector<cv::Point2f> points; // 矩形的四个顶点
    cv::Point2f top, bottom;

    explicit LightBar() = default;
    explicit LightBar(cv::RotatedRect &rect);
    bool isValid(const LightBarConfig &config) const;
};

struct Armor {
    //* 灯条
    LightBar left, right; // 左右灯条
    cv::Point2f center;   // 装甲板中心

    //* 装甲板上的数字/图案
    cv::Mat number_img;               // 用于识别数字的装甲板图像
    double confidence;                // 识别的置信度
    std::string classfication_result; // 识别的结果
    std::string number;               // 识别的数字

    Armor() = default;
    explicit Armor(const LightBar &l1, const LightBar &l2) {
        left = l1, right = l2;
        if (left.center.x > right.center.x) std::swap(left, right);
        center = (left.center + right.center) / 2;
    }
};

} // namespace AutoAim

#endif // __ARMOR_HPP__
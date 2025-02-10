#ifndef __ARMOR_HPP__
#define __ARMOR_HPP__

#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

#include <config.hpp>

namespace AutoAim {

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
    explicit LightBar(cv::RotatedRect &rect) : cv::RotatedRect{rect} {
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
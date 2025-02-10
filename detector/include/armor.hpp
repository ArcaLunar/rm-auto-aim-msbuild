#ifndef __ARMOR_HPP__
#define __ARMOR_HPP__

#include <algorithm>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

namespace AutoAim {

/**
 * !! 灯条 class
 */
struct LightBar : public cv::RotatedRect {
    double length, width;
    double tilt_angle;
    std::vector<cv::Point2f> points;
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
    }
};

struct Armor {};

} // namespace AutoAim

#endif // __ARMOR_HPP__
#include "opencv2/imgproc.hpp"
#include "structs.hpp"

LightBar::LightBar() = default;

LightBar::LightBar(const std::vector<cv::Point> &contour) {
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

LightBar::~LightBar() = default;

bool LightBar::is_valid(const LightBarConfig &config) const {
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

cv::Point2f LightBar::center() const { return ellipse.center; }

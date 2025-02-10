#ifndef __ARMOR_HPP__
#define __ARMOR_HPP__

#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

namespace AutoAim {

/**
 * !! 灯条 class
 */
struct LightBar : public cv::RotatedRect {
    double length, width;
};

} // namespace AutoAim

#endif // __ARMOR_HPP__
#ifndef __TRACKER_BASE_FILTER_HPP__
#define __TRACKER_BASE_FILTER_HPP__

#include "structs.hpp"
#include "Eigen/Dense"

class BaseFilter {
  public:
    virtual void init();
    virtual cv::Mat update(const Armor3d &info);
    virtual PredictedPosition pipeline(const Armor3d &info);
    virtual PredictedPosition predict(const cv::Mat &est, const Armor3d &info);
    virtual cv::Mat get_state();
};

#endif
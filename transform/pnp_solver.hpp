#ifndef __PNPSOLVER_HPP__
#define __PNPSOLVER_HPP__

#include "structs.hpp"

#include <opencv2/opencv.hpp>

namespace AutoAim {

class PnPSolver {

  private:
    cv::Mat cam_mat_, distort_mat_;
};

} // namespace AutoAim

#endif
#ifndef __PNPSOLVER_HPP__
#define __PNPSOLVER_HPP__

#include "structs.hpp"

#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <array>

namespace AutoAim {

class PnPSolver {

  public:
    PnPSolver(const std::array<double, 9> &cam_mat, const std::vector<double> &distort_mat);
    /**
     * @brief 根据甲板进行解算
     */
    bool solve_pnp(const Armor &armor, cv::Mat &rvec, cv::Mat &tvec);
    double distance_to_center(const cv::Point2f &img_point);

  private:
    cv::Mat cam_mat_, distort_mat_;
    std::vector<cv::Point3f> armor_points_;

    //! 甲板参数
    static constexpr double armor_width  = 180;
    static constexpr double armor_height = 60;
};

} // namespace AutoAim

#endif
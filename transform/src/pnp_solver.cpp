#include "pnp_solver.hpp"
#include "config.hpp"
#include "structs.hpp"

#include <opencv2/calib3d.hpp>
#include <spdlog/spdlog.h>

AutoAim::PnPSolver::PnPSolver(const std::array<double, 9> &cam_mat, const std::vector<double> &distort_mat) {
    cam_mat_     = cv::Mat(3, 3, CV_64F, const_cast<double *>(cam_mat.data())).clone();
    distort_mat_ = cv::Mat(1, 5, CV_64F, const_cast<double *>(distort_mat.data())).clone();
}

bool AutoAim::PnPSolver::solve_pnp(const Armor &armor, cv::Mat &rvec, cv::Mat &tvec) {
    double half_w = (armor.type == ArmorType::Small ? SmallArmorWidth : LargeArmorWidth) / 2;
    double half_h = (armor.type == ArmorType::Small ? SmallArmorHeight : LargeArmorHeight) / 2;

    armor_points_ = {
        cv::Point3f(-half_w, -half_h, 0),
        cv::Point3f(half_w, -half_h, 0),
        cv::Point3f(half_w, half_h, 0),
        cv::Point3f(-half_w, half_h, 0),
    };
    // solve pnp
    return cv::solvePnP(armor_points_, armor.vertices, cam_mat_, distort_mat_, rvec, tvec, false, cv::SOLVEPNP_IPPE);
}

double AutoAim::PnPSolver::distance_to_center(const cv::Point2f &img_point) {
    double cx = cam_mat_.at<double>(0, 2);
    double cy = cam_mat_.at<double>(1, 2);

    return cv::norm(img_point - cv::Point2f(cx, cy));
}
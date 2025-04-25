#ifndef __POSE_CVT_HPP__
#define __POSE_CVT_HPP__

#include "structs.hpp"
#include <string>

class PoseCVT {
  public:
    PoseCVT(std::string path);

    Armor3d solve_absolute(const AnnotatedArmorInfo &info);

  protected:
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;

    cv::Mat T_camera_to_barrel; // meters

    cv::Mat T_camera_to_imu; // meters
    cv::Mat R_camera_to_imu; // radians

    cv::Mat T_base_to_barrel; // meters
    cv::Mat R_base_to_barrel; // meters

    double bullet_velosity; // m/s

  private:
    /**
     * @brief Utilize the result provided by solvePnP().
     * @details Armor(2D) -> Camera coord
     */
    cv::Mat from_armor_to_camera(const pose_under_camera_coord &relative);
    cv::Mat from_camera_to_imu();
    cv::Mat from_imu_to_base(const IMUInfo &imu);
    cv::Mat from_base_to_barrel();
};

#endif
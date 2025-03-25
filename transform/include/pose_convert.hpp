#ifndef __POSE_CONVERT_HPP__
#define __POSE_CONVERT_HPP__

#include "structs.hpp"

#include <memory>
#include <opencv2/core/mat.hpp>
#include <spdlog/logger.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace AutoAim {

/**
 共有： base, IMU, barrel, camera, armor 五个坐标系
------
 - 我们假设装甲板总是投影到 camera 系里的固定位置。
 - 此时经过 solvepnp() 解算出来的是 armor 系到 camera 系的变换，因此 armor 系代表真实世界中的坐标系
------
 - base 系是刚开机时确定的 imu 的朝向、状态。
 - imu 系跟随云台一起转动，相对 base 系没有平移只有旋转
 - 由于 camera 和 barrel 并不在 imu 的原点，所以会相对 imu 有平移和旋转（跟随 imu 一起旋转）
 - 由于 camera 固定在 barrel 上，所以这两个坐标系之间只有平移没有旋转
 - 由于子弹从 barrel 出来，所以 barrel 系是我们的目标坐标系（我们想知道装甲板相对 barrel 的位置）
 */
class PoseConvert {
  public:
    PoseConvert(const std::string &cfg_path);
    /**
     * @brief 解算出 base 系下的装甲板中心坐标
     *
     * @param armor_info
     * @return Armor3d
     */
    Armor3d solve_absolute(const AnnotatedArmorInfo &armor_info);

  protected:
    std::shared_ptr<spdlog::logger> log_;

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

} // namespace AutoAim

#endif
#include <spdlog/spdlog.h>
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include "config.hpp"
#include "pose_convert.hpp"
#include "structs.hpp"
#include "transform.hpp"

#include <opencv2/calib3d.hpp>
#include <toml++/toml.h>

// ========================================================
// implementation of poses
// ========================================================

void pose_under_camera_coord::load_from_imu(const IMUInfo &imu, const cv::Mat &T_camera_to_barrel) {
    this->tvec /= 1000; // convert to meters

    this->center_3d = this->tvec + T_camera_to_barrel;
    this->distance  = cv::norm(this->center_3d);

    this->roll  = std::atan2(this->center_3d.at<double>(1), this->center_3d.at<double>(0)) * kRadianToDegree;
    this->pitch = std::atan2(this->center_3d.at<double>(1), this->center_3d.at<double>(2)) * kRadianToDegree;
    this->yaw   = -std::atan2(this->center_3d.at<double>(0), this->center_3d.at<double>(2)) * kRadianToDegree;

    cv::Mat R;
    cv::Rodrigues(this->rvec, R);
    R               = R.t();
    this->direction = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0)) * kRadianToDegree;
}

// ========================================================
// implementation of PoseConvert
// ========================================================

AutoAim::PoseConvert::PoseConvert(const std::string &cfg_path) {
    this->log_ = spdlog::stdout_color_mt("PoseConvert");
    this->log_->set_level(spdlog::level::trace);
    this->log_->set_pattern("[%H:%M:%S, +%4oms] [%15s:%3# in %!] [%^%l%$] %v");

    try {
        auto config = toml::parse(cfg_path);

        auto F = [&](const std::string &_s, cv::Mat &_res) {
            auto cam2barrel = config[_s];
            std::vector<double> data;
            if (const auto *arr = cam2barrel.as_array()) {
                for (const auto &elem : *arr)
                    data.push_back(elem.as_floating_point()->get());
            }
            _res = cv::Mat(data, true).reshape(3, 1);
        };
        F("cameraToBarrel", this->T_camera_to_barrel);
        F("cameraToIMU", this->T_camera_to_imu);
        cv::Mat R;
        F("cameraToIMURotation", R);
        this->R_camera_to_imu = Transform::Functions::get_rotation_matrix(
            R.at<double>(0, 0) * kDegreeToRadian,
            R.at<double>(1, 0) * kDegreeToRadian,
            R.at<double>(2, 0) * kDegreeToRadian
        );

    } catch (const std::exception &e) {
        SPDLOG_LOGGER_CRITICAL(this->log_, "failed to init pose transformer: {}", e.what());
    }
}

Armor3d AutoAim::PoseConvert::solve_absolute(const AnnotatedArmorInfo &info) {
    Armor3d result;

    //^ solvepnp
    std::vector<cv::Point3f> projected_points;
    if (info.armor.type == ArmorType::Large) {
        projected_points = {
            cv::Point3f(-LargeArmorWidth / 2, -LargeArmorHeight / 2, 0),
            cv::Point3f(LargeArmorWidth / 2, -LargeArmorHeight / 2, 0),
            cv::Point3f(LargeArmorWidth / 2, LargeArmorHeight / 2, 0),
            cv::Point3f(-LargeArmorWidth / 2, LargeArmorHeight / 2, 0),
        };
    } else if (info.armor.type == ArmorType::Small) {
        projected_points = {
            cv::Point3f(-SmallArmorWidth / 2, -SmallArmorHeight / 2, 0),
            cv::Point3f(SmallArmorWidth / 2, -SmallArmorHeight / 2, 0),
            cv::Point3f(SmallArmorWidth / 2, SmallArmorHeight / 2, 0),
            cv::Point3f(-SmallArmorWidth / 2, SmallArmorHeight / 2, 0),
        };
    }
    cv::solvePnP(
        projected_points,
        info.armor.vertices,
        camera_matrix,
        dist_coeffs,
        result.p_a2c.rvec,
        result.p_a2c.tvec,
        cv::SOLVEPNP_IPPE
    );

    //* solve relative pose
    result.p_a2c.load_from_imu(info.imu_info, this->T_camera_to_barrel);

    cv::Mat armor_to_barrel =                     // solving coord tf
        this->from_base_to_barrel() *             // base --> barrel
        this->from_imu_to_base(info.imu_info) *   // imu --> base
        this->from_camera_to_imu() *              // camera --> imu
        this->from_armor_to_camera(result.p_a2c); // armor --> camera

    //* solve absolute pose
    std::tie(result.R_armor_to_barrel, result.T_armor_to_barrel)
        = Transform::Functions::get_rotation_translation_from_homography_matrix(armor_to_barrel, false);

    if constexpr (PoseConvertDebug) {
        SPDLOG_LOGGER_INFO(
            this->log_,
            "armor center under barrel: ({},{},{})",
            result.p_barrel.center_3d.at<double>(0),
            result.p_barrel.center_3d.at<double>(1),
            result.p_barrel.center_3d.at<double>(2)
        );
    }

    // fill in data fields
    result.p_barrel.center_3d = result.T_armor_to_barrel;
    result.p_barrel.distance  = cv::norm(result.p_barrel.center_3d);
    result.p_barrel.direction
        = std::atan2(result.R_armor_to_barrel.at<double>(1, 0), result.R_armor_to_barrel.at<double>(0, 0))
        * kRadianToDegree;

    result.p_barrel.roll = std::atan2(result.p_barrel.center_3d.at<double>(2), result.p_barrel.center_3d.at<double>(1))
                         * kRadianToDegree;
    result.p_barrel.pitch = info.imu_info.pitch + result.p_a2c.pitch;
    result.p_barrel.yaw   = std::atan2(result.p_barrel.center_3d.at<double>(1), result.p_barrel.center_3d.at<double>(0))
                        * kRadianToDegree;

    //* relative pitch and yaw to barrel
    // pitch, yaw 用 armor->camera 近似 armor->barrel
    result.pitch_relative_to_barrel = result.p_a2c.pitch;
    result.yaw_relative_to_barrel   = result.p_a2c.yaw;

    //* bullet flying time
    double imu_pitch = info.imu_info.pitch * kDegreeToRadian;
    double pnp_pitch = std::atan2(result.p_barrel.center_3d.at<double>(1), result.p_barrel.center_3d.at<double>(2));
    result.bullet_flying_time = result.p_barrel.distance * std::cos(std::abs(imu_pitch) - std::abs(pnp_pitch))
                              / (bullet_velosity * std::cos(imu_pitch));

    if constexpr (PoseConvertDebug) {
        SPDLOG_LOGGER_INFO(this->log_, "estimated bullet flying time: {}", result.bullet_flying_time);
    }

    return result;
}

cv::Mat AutoAim::PoseConvert::from_armor_to_camera(const pose_under_camera_coord &relative) {
    cv::Mat armor_to_camera
        = Transform::Functions::get_homography_matrix_from_rotation_translation(relative.rvec, relative.tvec);
    return armor_to_camera;
}

cv::Mat AutoAim::PoseConvert::from_camera_to_imu() {
    cv::Mat camera_to_imu = Transform::Functions::get_homography_matrix_from_rotation_translation(
        this->R_camera_to_imu.t(), this->T_camera_to_imu
    );
    return camera_to_imu;
}

cv::Mat AutoAim::PoseConvert::from_imu_to_base(const IMUInfo &imu) {
    cv::Mat imu_to_base = Transform::Functions::get_homography_matrix_from_rotation_translation(
        imu.rotation(), cv::Mat::zeros(3, 1, CV_64F)
    );
    return imu_to_base;
}

cv::Mat AutoAim::PoseConvert::from_base_to_barrel() {
    cv::Mat base_to_barrel = Transform::Functions::get_homography_matrix_from_rotation_translation(
        this->R_base_to_barrel, this->T_base_to_barrel
    );
    return base_to_barrel;
}
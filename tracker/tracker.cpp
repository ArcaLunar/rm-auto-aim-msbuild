#include "config.hpp"
#include "kf.hpp"
#include "structs.hpp"
#include <algorithm>
#include <chrono>
#include <opencv2/core.hpp>
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include "tracker.hpp"

#include <spdlog/spdlog.h>
#include <stdexcept>

AutoAim::Tracker::Tracker(const Labels &label, const std::string &config_path) {
    this->log_ = spdlog::stdout_color_mt("Tracker");
    this->log_->set_level(spdlog::level::trace);
    this->log_->set_pattern("[%H:%M:%S, +%4oms] [%15s:%3# in %!] [%^%l%$] %v");

    this->status_          = TrackingStatus::LOST;
    this->tracked_id_      = label;
    this->last_track_time_ = std::chrono::high_resolution_clock::now();

    this->_forward_and_init();
    this->low_pass_.set_alpha(0.75);

    try {
        throw std::runtime_error("reading config not implemented yet");
    } catch (const std::exception &e) {
        SPDLOG_LOGGER_CRITICAL(this->log_, "Error: {}", e.what());
    }
}

PredictedPosition AutoAim::Tracker::update(const Armor3d &armor3d) {
    if (this->status_ == TrackingStatus::LOST) {
        this->status_ = TrackingStatus::FITTING;
    }

    cv::Mat estimate   = this->_forward_and_update(armor3d);
    return this->pred_ = this->_forward_and_predict(estimate, armor3d);
}

void AutoAim::Tracker::_forward_and_init() {
    if constexpr (std::is_same_v<decltype(this->kf_), KalmanFilter::KF>) {
        // * 如果使用线性卡尔曼滤波
        const double &dt = this->cfg_.dt;
        // clang-format off
        this->kf_.transitionMatrix = (cv::Mat_<float>(this->state_dim, this->state_dim) <<
            1, 0, 0, dt, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, dt, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, dt, 0, 0, 0, 0,
            0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 1, dt, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1, dt,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 1);
        // clang-format on
        cv::setIdentity(this->kf_.measurementMatrix);
        cv::setIdentity(this->kf_.processNoiseCov, cv::Scalar::all(this->cfg_.kf_q));
        cv::setIdentity(this->kf_.measurementNoiseCov, cv::Scalar::all(this->cfg_.kf_r));
        cv::setIdentity(this->kf_.errorCovPost, cv::Scalar::all(1));
        cv::randn(this->kf_.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));
    }
}

PredictedPosition AutoAim::Tracker::_forward_and_predict(const cv::Mat &estimate, const Armor3d &armor) {
    PredictedPosition result;
    result.tracking_id = armor.result;

    if constexpr (std::is_same_v<decltype(this->kf_), KalmanFilter::KF>) {
        double est_x      = estimate.at<double>(0);
        double est_y      = estimate.at<double>(1);
        double est_z      = estimate.at<double>(2);
        double est_vx     = estimate.at<double>(3);
        double est_vy     = estimate.at<double>(4);
        double est_vz     = estimate.at<double>(5);
        double est_dir    = estimate.at<double>(6);
        double est_vdir   = estimate.at<double>(7);
        double est_pitch  = estimate.at<double>(8);
        double est_vpitch = estimate.at<double>(9);

        double t_fly = armor.bullet_flying_time + this->fire_cfg_.time_dalay;

        result.x         = armor.p_barrel.center_3d.at<double>(0) + est_vx * t_fly;
        result.y         = armor.p_barrel.center_3d.at<double>(1) + est_vy * t_fly;
        result.z         = armor.p_barrel.center_3d.at<double>(2) + est_vz * t_fly;
        result.direction = armor.p_barrel.direction + est_vdir * t_fly;
        result.center_3d = (cv::Mat_<double>(3, 1) << result.x, result.y, result.z);
        result.pitch     = armor.p_barrel.pitch + est_vpitch * t_fly;
        result.yaw       = std::atan2(result.y, result.x) * kRadianToDegree;
        result.yaw       = this->low_pass_.filter(result.yaw);

        return result;
    }
}

cv::Mat AutoAim::Tracker::_forward_and_update(const Armor3d &armor3d) {
    if constexpr (std::is_same_v<decltype(this->kf_), KalmanFilter::KF>) {
        //* construct observation vector
        double vx, vy, vz;
        using namespace std::chrono;

        if (this->last_track_time_ == high_resolution_clock::time_point{}) {
            vx = vy = vz = 0;
        } else {
            double duration = duration_cast<seconds>(high_resolution_clock::now() - this->last_track_time_).count();
            vx              = std::clamp(
                (prev_state_.p_barrel.center_3d.at<double>(0) - armor3d.p_barrel.center_3d.at<double>(0)) / duration,
                this->cfg_.max_speed * -1.0,
                this->cfg_.max_speed
            );
            vy = std::clamp(
                (prev_state_.p_barrel.center_3d.at<double>(1) - armor3d.p_barrel.center_3d.at<double>(1)) / duration,
                this->cfg_.max_speed * -1.0,
                this->cfg_.max_speed
            );
            vz = std::clamp(
                (prev_state_.p_barrel.center_3d.at<double>(2) - armor3d.p_barrel.center_3d.at<double>(2)) / duration,
                this->cfg_.max_speed * -1.0,
                this->cfg_.max_speed
            );
        }

        // clang-format off
        cv::Mat observation = (cv::Mat_<float>(this->observe_dim, 1) <<
            armor3d.p_barrel.center_3d.at<double>(0),
            armor3d.p_barrel.center_3d.at<double>(1),
            armor3d.p_barrel.center_3d.at<double>(2),
            vx, vy, vz,
            armor3d.p_barrel.direction, armor3d.p_barrel.pitch
        );
        // clang-format on

        // * update previous state to current
        prev_state_      = armor3d;
        last_track_time_ = armor3d.timestamp;

        // * update kalman filter
        this->kf_.predict();
        auto estimate = this->kf_.correct(observation);

        return estimate;
    }
}

void AutoAim::Tracker::check_status() {
    using namespace std::chrono;
    if (this->status_ == TrackingStatus::LOST)
        return;

    auto now = high_resolution_clock::now();
    if (duration_cast<seconds>(now - this->last_track_time_).count() > this->cfg_.lost_timeout) {
        this->status_ = TrackingStatus::LOST;
        this->_forward_and_init();
    }
}
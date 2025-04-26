#ifndef __TRACKER_KF_HPP__
#define __TRACKER_KF_HPP__

#include "Eigen/Dense"
#include "base_filter.hpp"
#include "base_model.hpp"
#include "low_pass_filter.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/video/tracking.hpp"
#include "structs.hpp"

/**
 * @brief Classic Linear Kalman Filter
 *
 * Assume constant velocity, constant angular velocity
 * modeling mass center of enemy
 *
 * State: [x, y, z, rz, pitch, vx, vy, vz, vrz, vpitch]
 * Obsrv: [x, y, z, rz, pitch]
 * rz: direction
 *
 */

class KFPredictor : public BaseFilter {
  public:
    KFPredictor(int state, int obs, int id) : tracking_id(id) {
        this->statedim               = state;
        this->obsdim                 = obs;
        this->dt                     = 0.005; // default time step
        this->kf                     = cv::KalmanFilter(state, obs, 0);
        this->kf.processNoiseCov     = cv::Mat::eye(state, state, CV_64F) * 1e-5;
        this->kf.measurementNoiseCov = cv::Mat::eye(obs, obs, CV_64F) * 1e-2;

        this->kf.transitionMatrix  = cv::Mat::eye(state, state, CV_64F);
        this->kf.measurementMatrix = cv::Mat::eye(obs, state, CV_64F);
        for (int i = 0; i < 5; i++) {
            this->kf.transitionMatrix.at<double>(i, i + 5) = dt;
        }

        randn(this->kf.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1)); // 初始化前一时刻状态为随机值
    }
    void init() {
        randn(this->kf.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1)); // 初始化前一时刻状态为随机值
    }

    cv::Mat update(const Armor3d &info) {
        // clang-format off
        cv::Mat observation = (cv::Mat_<float>(this->obsdim, 1) <<
            info.p_barrel.center_3d.at<double>(0),
            info.p_barrel.center_3d.at<double>(1),
            info.p_barrel.center_3d.at<double>(2),
            info.p_barrel.direction, info.p_barrel.pitch
        );
        // clang-format on

        prev_pose     = info;
        auto estimate = this->kf.correct(observation);

        return estimate;
    }

    PredictedPosition predict(const cv::Mat &est, const Armor3d &info) {
        PredictedPosition result;
        result.tracking_id = info.result;

        auto prediction  = this->kf.predict();
        result.x         = prediction.at<double>(0);
        result.y         = prediction.at<double>(1);
        result.z         = prediction.at<double>(2);
        result.direction = prediction.at<double>(3);
        result.pitch     = prediction.at<double>(4);

        double t_fly = info.bullet_flying_time; // TODO: + flying time delay

        result.yaw      = std::atan2(result.y, result.x) * kRadianToDegree;
        result.yaw      = this->lp.filter(result.yaw);
        result.distance = std::sqrt(result.x * result.x + result.y * result.y + result.z * result.z);

        return result;
    }

  private:
    int statedim, obsdim;
    double dt;
    int tracking_id;
    cv::KalmanFilter kf;
    Armor3d prev_pose;
    LowPassFilter lp;
};

#endif
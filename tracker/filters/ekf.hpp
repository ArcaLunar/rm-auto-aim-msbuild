#ifndef __TRACKER_FILTERS_EKF_HPP__
#define __TRACKER_FILTERS_EKF_HPP__

#include <Eigen/Dense>
#include <functional>

namespace Tracker {

/**
 * @brief 扩展卡尔曼滤波
 * @details x = [x, y, z, yaw, vx, vy, vz, vyaw, r]^T
 * @ref chenjunnn/rm_auto_aim
 */
class EKF {
    using Vec2Vec  = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;
    using Vec2Mat  = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;
    using Void2Mat = std::function<Eigen::MatrixXd(void)>;

  public:
    void set_initial_state(const Eigen::VectorXd &x0, const Eigen::MatrixXd &P_post0);

    void reset_state();

    void set_state_transition(const Vec2Vec &f, const Vec2Mat &j_f);

    void set_observation(const Vec2Vec &h, const Vec2Mat &j_h);

    void set_process_noise_covariance(const Void2Mat &update_Q);

    void set_measurement_noise_covariance(const Vec2Mat &update_R);

    /**
     * @brief 预测下一时刻的状态
     */
    Eigen::MatrixXd predict();

    /**
     * @brief 通过观测值更新预测的状态，并返回之
     */
    Eigen::MatrixXd update(const Eigen::VectorXd &z);

  protected:
    Vec2Vec f;          // function for State Transition
    Vec2Mat jacobian_f; // Jacobian of F
    Eigen::MatrixXd F;

    Vec2Vec h;          // function for Observation
    Vec2Mat jacobian_h; // Jacobian of H
    Eigen::MatrixXd H;

    Void2Mat update_Q; // function for Process Noise Covariance
    Eigen::MatrixXd Q;

    Vec2Mat update_R; // function for Measurement Noise Covariance
    Eigen::MatrixXd R;

    Eigen::MatrixXd P_pri;  // Priori Error Covariance
    Eigen::MatrixXd P_post; // Posteriori Error Covariance

    Eigen::VectorXd x_pri;  // Priori State
    Eigen::VectorXd x_post; // Posteriori State

    Eigen::MatrixXd I; // Identity Matrix
    Eigen::MatrixXd K; // Kalman Gain

    int n; // State Dimension
};

} // namespace Tracker

#endif // __TRACKER_FILTERS_EKF_HPP__
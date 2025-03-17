#include "ekf.hpp"

void KalmanFilter::EKF::set_initial_state(const Eigen::VectorXd &x0, const Eigen::MatrixXd &P_post0) {
    x_post = x0;
    P_post = P_post0;

    n      = P_post0.rows();
    x_pri  = Eigen::VectorXd::Zero(n);
    x_post = Eigen::VectorXd::Zero(n);
    I      = Eigen::MatrixXd::Identity(n, n);
}

void KalmanFilter::EKF::reset_state() {
    x_post = Eigen::VectorXd::Zero(x_post.size());
    P_post = Eigen::MatrixXd::Identity(P_post.rows(), P_post.cols());
}

void KalmanFilter::EKF::set_state_transition(const Vec2Vec &f, const Vec2Mat &j_f) {
    this->f          = f;
    this->jacobian_f = j_f;
}

void KalmanFilter::EKF::set_observation(const Vec2Vec &h, const Vec2Mat &j_h) {
    this->h          = h;
    this->jacobian_h = j_h;
}

void KalmanFilter::EKF::set_process_noise_covariance(const Void2Mat &update_Q) { this->update_Q = update_Q; }

void KalmanFilter::EKF::set_measurement_noise_covariance(const Vec2Mat &update_R) { this->update_R = update_R; }

Eigen::MatrixXd KalmanFilter::EKF::predict() {
    F = jacobian_f(x_post);
    Q = update_Q();

    x_pri = f(x_post);
    P_pri = F * P_post * F.transpose() + Q;

    x_post = x_pri;
    P_post = P_pri;

    return x_pri;
}

Eigen::MatrixXd KalmanFilter::EKF::update(const Eigen::VectorXd &z) {
    H = jacobian_h(x_post);
    R = update_R(z);

    K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();

    x_post = x_pri + K * (z - h(x_pri));
    P_post = (I - K * H) * P_pri;

    return x_post;
}
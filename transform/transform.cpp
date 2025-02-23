#include "config.hpp"
#include "structs.hpp"
#include "transform.hpp"

#include <cassert>
#include <cstddef>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <queue>
#include <spdlog/spdlog.h>

// ========================================================
// Implement for IMU pose
// ========================================================

void IMUInfo::load_from_recvmsg(const VisionPLCRecvMsg &msg) {
    roll  = msg.imu_roll;
    pitch = msg.imu_pitch;
    yaw   = msg.imu_yaw;
}

cv::Mat IMUInfo::rotation() const {
    return HerkulesTransform::Functions::get_rotation_matrix(
        roll * kDegreeToRadian, pitch * kDegreeToRadian, yaw * kDegreeToRadian
    );
}

// ========================================================
// Implement for Coordinate Transform
// ========================================================

cv::Mat HerkulesTransform::Functions::get_homography_matrix_from_rotation_translation(
    const cv::Mat &rotation, const cv::Mat &translation
) {
    // check size
    CV_Assert(rotation.total() == 3 || rotation.total() == 9); // either r_vec or r_mat
    CV_Assert(translation.total() == 3);                       // t_vec

    cv::Mat R; // 3x3 rotation matrix
    if (rotation.total() == 3) cv::Rodrigues(rotation, R);
    else R = rotation;

    cv::Mat H{cv::Mat::eye(4, 4, R.type())}; // homography matrix
    R.copyTo(H(cv::Rect(0, 0, 3, 3)));
    translation.copyTo(H(cv::Rect(3, 0, 1, 3)));

    return H;
}

std::pair<cv::Mat, cv::Mat> HerkulesTransform::Functions::get_rotation_translation_from_homography_matrix(
    const cv::Mat &homography, bool return_rvec
) {
    // check size
    CV_Assert(homography.total() == 16);

    cv::Mat R{homography(cv::Rect(0, 0, 3, 3)).clone()};
    cv::Mat t{homography(cv::Rect(3, 0, 1, 3)).clone()};

    if (return_rvec) cv::Rodrigues(R, R);

    return {R, t};
}

cv::Mat HerkulesTransform::Functions::rotate_around_x(const double &angle) {
    double x = angle * kDegreeToRadian;
    return
        // clang-format off
    cv::Mat_<double>(3, 3) <<
    1,      0,       0,
    0, cos(x), -sin(x),
    0, sin(x),  cos(x);
    // clang-format on
}

cv::Mat HerkulesTransform::Functions::rotate_around_y(const double &angle) {
    double y = angle * kDegreeToRadian;
    return
        // clang-format off
    cv::Mat_<double>(3, 3) <<
     cos(y), 0, sin(y),
          0, 1,      0,
    -sin(y), 0, cos(y);
    // clang-format on
}

cv::Mat HerkulesTransform::Functions::rotate_around_z(const double &angle) {
    double z = angle * kDegreeToRadian;
    return
        // clang-format off
    cv::Mat_<double>(3, 3) <<
    cos(z), -sin(z), 0,
    sin(z),  cos(z), 0,
         0,       0, 1;
    // clang-format on
}

cv::Mat HerkulesTransform::Functions::get_translation_vector(const double &dx, const double &dy, const double &dz) {
    return
        // clang-format off
    cv::Mat_<double>(3, 1) <<
    dx,
    dy,
    dz;
    // clang-format on
}

cv::Mat HerkulesTransform::Functions::get_rotation_matrix(const double &rx, const double &ry, const double &rz) {
    return rotate_around_x(rx) * rotate_around_y(ry) * rotate_around_z(rz);
}

// ========================================================
// Implement for Coordinate Manager
// ========================================================

HerkulesTransform::CoordinateManager::CoordinateManager() : node_cnt_(0) {}

void HerkulesTransform::CoordinateManager::register_tf(
    const std::string &from, const std::string &to, const Eigen::Matrix4d &tf
) {
    if (!node_.contains(from)) {
        node_[from] = node_cnt_;
        G_.push_back({});
        spdlog::info("coordman register node name = \"{:<10}\" id = {:<10}", from, node_cnt_++);
    }
    if (!node_.contains(to)) {
        node_[to] = node_cnt_;
        G_.push_back({});
        spdlog::info("coordman register node name = \"{:<10}\" id = {:<10}", to, node_cnt_++);
    }

    // add bidirectional edge
    size_t edge_id = edges_.size();
    edges_.emplace_back(node_[from], node_[to], tf);
    G_[node_[from]].emplace_back(node_[to], edge_id);
    edge_id = edges_.size();
    edges_.emplace_back(node_[to], node_[from], tf.inverse());
    G_[node_[to]].emplace_back(node_[from], edge_id);
}

Eigen::Matrix4d
HerkulesTransform::CoordinateManager::extract_tf_matrix(const std::string &from, const std::string &to) {
    assert(node_.contains(from) && node_.contains(to));

    std::vector<Eigen::Matrix4d> tf_res(node_cnt_, Eigen::Matrix4d::Identity());
    std::vector<int> vis(node_cnt_, 0);
    int start = node_[from], end = node_[to];

    std::queue<int> q;
    q.push(start);
    while (!q.empty()) {
        int u = q.front();
        q.pop();
        if (vis[u]) continue;
        vis[u] = 1;
        for (auto &[v, id] : G_[u]) {
            if (vis[v]) continue;
            tf_res[v] = tf_res[u] * std::get<2>(edges_[id]);
            q.push(v);
        }
    }

    return tf_res[node_[to]];
}

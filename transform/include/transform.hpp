#ifndef __TRANSFORM_HPP__
#define __TRANSFORM_HPP__

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include <Eigen/Core>
#include <Eigen/Dense>
#include <map>
#include <opencv2/core/core.hpp>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace HerkulesTransform {

namespace Functions {

/**
 * @brief 从旋转矩阵和平移矩阵计算 homography matrix。返回 homography matrix
 */
cv::Mat get_homography_matrix_from_rotation_translation(const cv::Mat &rotation, const cv::Mat &translation);

/**
 * @brief 从 homography 矩阵计算旋转矩阵和平移矩阵
 * @return std::pair<cv::Mat, cv::Mat> [0]=旋转矩阵，[1]=平移矩阵
 */
std::pair<cv::Mat, cv::Mat>
get_rotation_translation_from_homography_matrix(const cv::Mat &homography, bool return_rvec);

/**
 * @brief 计算绕 x 轴旋转 angle 角（角度）的旋转矩阵
 */
cv::Mat rotate_around_x(const double &angle);

/**
 * @brief 计算绕 y 轴旋转 angle 角（角度）的旋转矩阵
 */
cv::Mat rotate_around_y(const double &angle);

/**
 * @brief 计算绕 z 轴旋转 angle 角（角度）的旋转矩阵
 */
cv::Mat rotate_around_z(const double &angle);

/**
 * @brief 从沿三个轴平移的 dx, dy, dz 得到平移矩阵（平移向量）
 */
cv::Mat get_translation_vector(const double &dx, const double &dy, const double &dz);

/**
 * @brief 从绕三个轴旋转的 rx, ry, rz 得到旋转矩阵
 */
cv::Mat get_rotation_matrix(const double &rx, const double &ry, const double &rz);

double rad_to_deg(const double &rad);
double deg_to_rad(const double &deg);

} // namespace Functions

class CoordinateManager {
  public:
    CoordinateManager();
    void register_tf(const std::string &from, const std::string &to, const Eigen::Matrix4d &tf);
    Eigen::Matrix4d extract_tf_matrix(const std::string &from, const std::string &to);

  protected:
    std::map<std::string, int> node_; // 用 string 保留语义，用 int 保留节点编号
    int node_cnt_;                    // 节点计数

    using _Edge = std::tuple<int, int, Eigen::Matrix4d>;
    std::vector<_Edge> edges_;                        // 边
    std::vector<std::vector<std::pair<int, int>>> G_; // transform 矩阵图信息，边：G[u] = (v, edge_id)
};

} // namespace HerkulesTransform

#endif
#ifndef __TRANSFORM_HPP__
#define __TRANSFORM_HPP__

#include <opencv2/core/core.hpp>
#include <utility>

namespace Transform {

namespace Functions {

/**
 * @brief 从旋转矩阵和平移矩阵计算 homography matrix。返回 homography matrix
 */
cv::Mat get_homography_matrix_from_rotation_translation(const cv::Mat &rotation, const cv::Mat &translation);

/**
 * @brief 从 homography 矩阵计算旋转矩阵和平移矩阵
 * @return std::pair<cv::Mat, cv::Mat> [0]=旋转矩阵，[1]=平移矩阵
 */
std::pair<cv::Mat, cv::Mat> get_rotation_translation_from_homography_matrix(const cv::Mat &homography, bool return_rvec);

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

} // namespace Functions

} // namespace Transform

#endif
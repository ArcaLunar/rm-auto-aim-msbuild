#ifndef __DETECTOR_HPP__
#define __DETECTOR_HPP__

#include "structs.hpp"

#include <opencv2/core.hpp>
#include <spdlog/spdlog.h>
#include <toml++/toml.hpp>

namespace AutoAim {

class Detector {
    LightBarConfig light_bar_config_;
    ArmorConfig armor_config_;
    cv::Mat debug_frame;

    /* ==== Functions ==== */

    // 按灰度阈值二值化图像
    cv::Mat preprocess_image(const cv::Mat &src);

    // 检测灯条
    std::vector<LightBar> detect_lightbars(const cv::Mat &rgb, const cv::Mat &binary);

    // 将一组灯条匹配成装甲板
    std::vector<Armor> pair_lightbars(std::vector<LightBar> &lights);

    // 检测两个匹配的灯条之间是否还有其他灯条
    bool check_mispair(const Armor &armor, const std::vector<LightBar> &lights);

  public:
    // 载入配置文件
    Detector(std::string path = "../config/detection_tr.toml");

    // 检测装甲板
    std::vector<Armor> detect(const cv::Mat &img);

    // （调试用）将检测结果绘制到图像上
    void draw_results_to_image(cv::Mat &img, const std::vector<Armor> &armors);
};

} // namespace AutoAim

#endif // __DETECTOR_HPP__
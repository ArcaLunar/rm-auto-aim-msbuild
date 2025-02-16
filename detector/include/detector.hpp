#ifndef __DETECTOR_HPP__
#define __DETECTOR_HPP__

#include "armor.hpp"

#include <opencv2/core.hpp>
#include <spdlog/spdlog.h>
#include <toml++/toml.hpp>

namespace AutoAim {

class Detector {
    LightBarConfig light_bar_config_;
    ArmorConfig armor_config_;

    /* ==== Functions ==== */
    // 按灰度阈值二值化图像
    cv::Mat PreprocessImage(const cv::Mat &src);

    // 检测灯条
    std::vector<LightBar> DetectLightBars(const cv::Mat &rgb, const cv::Mat &binary);

    // 将一组灯条匹配成装甲板
    std::vector<Armor> MatchLightBars(const std::vector<LightBar> &lights);

    // 检测两个匹配的灯条之间是否还有其他灯条
    bool ContainAnotherLightBar(const LightBar &light1, const LightBar &light2, const std::vector<LightBar> &lights);

    void draw_results_to_image(cv::Mat &img, const std::vector<Armor>& armors);

  public:
    // 载入配置文件
    Detector(std::string path = "../config/detection_tr.toml");
};

} // namespace AutoAim

#endif // __DETECTOR_HPP__
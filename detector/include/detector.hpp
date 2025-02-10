#ifndef __DETECTOR_HPP__
#define __DETECTOR_HPP__

#include "armor.hpp"
#include "config.hpp"
#include "id_classify.hpp"
#include "structs.hpp"

#include <memory>

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

  public:
    // 载入配置文件
    Detector(std::string path = "../config/detection_tr.toml");
};

} // namespace AutoAim

#endif // __DETECTOR_HPP__
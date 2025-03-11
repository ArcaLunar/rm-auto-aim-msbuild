#ifndef __PUBLISHER_HPP__
#define __PUBLISHER_HPP__

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include "classifier.hpp"
#include "detector.hpp"
#include "structs.hpp"

#include <memory>
#include <vector>

namespace AutoAim {

class Publisher {
  public:
    Publisher(const std::string &config_path);

    /**
     * @brief 传入一帧图像，返回所有识别到的装甲板信息
     * @param raw 原始图像
     * @param imu 此时的 IMU 信息
     * @return std::vector<AnnotatedArmorInfo> 该帧图像中所有识别到的装甲板信息
     */
    std::vector<AnnotatedArmorInfo> annotate_image(const RawFrameInfo &raw, const IMUInfo &imu);

  protected:
    std::shared_ptr<Detector> detector_;
    std::shared_ptr<Classifier> classifier_;
};

} // namespace AutoAim

#endif
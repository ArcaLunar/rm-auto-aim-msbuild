#ifndef __DETECTOR_HPP__
#define __DETECTOR_HPP__

#include "structs.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class OpenCVDetector {
  public:
    OpenCVDetector(std::string path);
    void draw_results_to_image(cv::Mat &img, const std::vector<RawArmor> &armors);
    /**
     * @brief 对读取的图像进行识别
     */
    std::vector<RawArmor> detect(const cv::Mat &image);

  private:
    LightBarConfig lbcfg;
    ArmorConfig acfg;
    cv::Mat debug_frame;

  protected:
    /**
     * @brief 图像预处理，灰度+二值化
     */
    cv::Mat preprocess_image(const cv::Mat &src);

    /**
     * @brief 检测灯条
     */
    std::vector<LightBar> detect_lightbars(const cv::Mat &rgb, const cv::Mat &binary);

    /**
     * @brief 配对灯条，并且检测装甲板
     */
    std::vector<RawArmor> pair_lightbars(std::vector<LightBar> &lights);

    /**
     * @brief 检测装甲板是否正确匹配
     */
    bool check_mispair(const RawArmor &armor, const std::vector<LightBar> &lights);
};

#endif
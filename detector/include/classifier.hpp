#ifndef __CLASSIFIER_HPP__
#define __CLASSIFIER_HPP__

#include "structs.hpp"

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace AutoAim {

class Classifier {
  public:
    Classifier(const std::string &config_path);

    /**
     * @brief 对给定的装甲板数字区域进行分类
     */
    std::string classify(const cv::Mat &roi);

    /**
     * @brief 从图像里提取装甲板数字区域
     */
    cv::Mat extract_region_of_interest(const cv::Mat &img, const Armor &armor);
    std::vector<cv::Mat> extract_region_of_interest(const cv::Mat &img, const std::vector<Armor> &armors);

  protected:
    cv::dnn::Net net_;
    std::vector<std::string> labels_, ignore_;
    double confidence_threshold_;

  private:
    cv::Mat softmax(const cv::Mat &src);
    void preprocess(cv::Mat &src);
    std::string inference(cv::Mat &src);
}; // class Classifier

} // namespace AutoAim

#endif // __CLASSIFIER_HPP__
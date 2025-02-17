#ifndef __CLASSIFIER_HPP__
#define __CLASSIFIER_HPP__

#include "armor.hpp"

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace AutoAim {

class Classifier {
  public:
    Classifier(const std::string &config_path = "../config/detection_tr.toml");

    void classify(std::vector<Armor> &armors);
    void extract_region_of_interest(const cv::Mat &img, const std::vector<Armor> &armors);

  private:
    cv::dnn::Net net_;
    std::vector<std::string> labels_, ignore_;
    double threshold_;
};

} // namespace AutoAim

#endif // __CLASSIFIER_HPP__
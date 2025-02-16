#ifndef __CLASSIFIER_HPP__
#define __CLASSIFIER_HPP__

#include "armor.hpp"

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace AutoAim {

class Classifier {
  public:
    Classifier(
        const std::string &model_path, const std::string &label_path, const double &threshold,
        const std::vector<std::string> &ignore = {}
    );

    void classify(std::vector<Armor> &armors);

  private:
    cv::dnn::Net net_;
    std::vector<std::string> labels_, ignore_;
    double threshold_;
};

} // namespace AutoAim

#endif // __CLASSIFIER_HPP__
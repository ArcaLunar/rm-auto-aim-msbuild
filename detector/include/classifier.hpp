#ifndef __CLASSIFIER_HPP__
#define __CLASSIFIER_HPP__

#include "opencv2/dnn/dnn.hpp"
#include "structs.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

class Classifier {
  public:
    Classifier(std::string path);
    /**
     * @brief extract the class from ROI
     *
     * @param roi
     * @return int representing the class
     */
    int classify(cv::Mat &roi);

    /**
     * @brief extract region of interest from the image
     *
     * @param img
     * @param armor
     * @return cv::Mat
     */
    cv::Mat extract_region_of_interest(const cv::Mat &img, const RawArmor &armor);
    /**
     * @brief Execute `extract_roi()` by batch
     *
     * @param img
     * @param armors
     * @return std::vector<cv::Mat>
     */
    std::vector<cv::Mat> extract_region_of_interest(const cv::Mat &img, const std::vector<RawArmor> &armors);

  private:
    cv::dnn::Net net;
    double confidence_threshold;

  protected:
    cv::Mat softmax(const cv::Mat &prob);
    void preprocess(cv::Mat &src);
    int inference(cv::Mat &src);
};

#endif
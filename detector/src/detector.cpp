#include "detector.hpp"

//! Detector
AutoAim::Detector::Detector(std::string path) : light_bar_config_(path), armor_config_(path) {
    spdlog::info("Detector initialized with config file: \"{}\"", path);
}

cv::Mat AutoAim::Detector::PreprocessImage(const cv::Mat &src) {
    cv::Mat gray;
    cv::cvtColor(src, gray, cv::COLOR_RGB2GRAY);

    cv::Mat binary;
    cv::threshold(gray, binary, this->armor_config_.binary_threshold, 255, cv::THRESH_BINARY);
    
    return binary;
}
#include "classifier.hpp"
#include "debug_options.hpp"
#include "opencv2/dnn/dnn.hpp"
#include "spdlog/spdlog.h"
#include "toml++/toml.h"

extern DebugOptions options;

Classifier::Classifier(std::string path) {
    if (options.classifier.init)
        spdlog::info("Loading model from {}", path);

    try {
        auto T                     = toml::parse_file(path);
        const auto model_path      = T["model_path"].value_or<std::string>("../model.onnx");
        this->confidence_threshold = T["confidence_threshold"].value_or(0.5);

        this->net = cv::dnn::readNetFromONNX(model_path);

    } catch (const std::exception &e) {
        spdlog::error("Failed to load model: {}", e.what());
        exit(-1);
    }
}

cv::Mat Classifier::extract_region_of_interest(const cv::Mat &img, const RawArmor &armor) {
    if (options.classifier.show_roi)
        spdlog::info("extracting ROI from armor, performing test");

    if (armor.vertices.size() != 4) {
        spdlog::error("invalid armor vertices size: {}", armor.vertices.size());
        return {};
    } else if (options.classifier.show_roi)
        spdlog::info("armor test passed, extracting ROI");

    // 计算数字区域
    cv::Point2f vecLeft = armor.vertices[3] - armor.vertices[0];
    double lenLeft      = cv::norm(armor.vertices[3] - armor.vertices[0]);
    vecLeft /= cv::norm(vecLeft);
    cv::Point2f top_left    = armor.vertices[0] - (lenLeft / 3) * vecLeft;
    cv::Point2f bottom_left = armor.vertices[3] + (lenLeft / 3) * vecLeft;

    cv::Point2f vecRight = armor.vertices[2] - armor.vertices[1];
    double lenRight      = cv::norm(armor.vertices[2] - armor.vertices[1]);
    vecRight /= cv::norm(vecRight);
    cv::Point2f top_right    = armor.vertices[1] - (lenRight / 3) * vecRight;
    cv::Point2f bottom_right = armor.vertices[2] + (lenRight / 3) * vecRight;

    double horizontal_len = (cv::norm(top_left - top_right) + cv::norm(bottom_left - bottom_right)) / 2;
    top_left.x += horizontal_len * 0.3;
    top_right.x -= horizontal_len * 0.3;
    bottom_left.x += horizontal_len * 0.3;
    bottom_right.x -= horizontal_len * 0.3;

    // 限制边界
    std::clamp(top_left.x, 1.0f, static_cast<float>(img.cols) - 1);
    std::clamp(top_left.y, 1.0f, static_cast<float>(img.rows) - 1);
    std::clamp(top_right.x, 1.0f, static_cast<float>(img.cols) - 1);
    std::clamp(top_right.y, 1.0f, static_cast<float>(img.rows) - 1);
    std::clamp(bottom_left.x, 1.0f, static_cast<float>(img.cols) - 1);
    std::clamp(bottom_left.y, 1.0f, static_cast<float>(img.rows) - 1);
    std::clamp(bottom_right.x, 1.0f, static_cast<float>(img.cols) - 1);
    std::clamp(bottom_right.y, 1.0f, static_cast<float>(img.rows) - 1);

    // 透视变换
    cv::Mat pattern_img;
    cv::Point2f src_points[4] = {top_left, top_right, bottom_right, bottom_left};
    cv::Point2f dst_points[4]
        = {cv::Point2f(0, 0), cv::Point2f(64, 0), cv::Point2f(64, 64), cv::Point2f(0, 64)}; // model.onnx 为 64x64
    cv::Mat warpMatrix = cv::getPerspectiveTransform(src_points, dst_points);
    cv::warpPerspective(img, pattern_img, warpMatrix, cv::Size(64, 64));

    return pattern_img;
}

std::vector<cv::Mat> Classifier::extract_region_of_interest(const cv::Mat &img, const std::vector<RawArmor> &armors) {
    std::vector<cv::Mat> rois;
    for (const auto &armor : armors)
        rois.push_back(extract_region_of_interest(img, armor));
    return rois;
}

cv::Mat Classifier::softmax(const cv::Mat &src) {
    cv::Mat dst;
    float sum = 0.0, max = 0.0;

    max = *std::max_element(src.begin<float>(), src.end<float>());
    cv::exp(src - max, dst);
    sum = cv::sum(dst)[0];
    dst /= sum;

    return dst;
}

int Classifier::inference(cv::Mat &src) {
    preprocess(src); //* 预处理

    cv::Mat inputBlob = cv::dnn::blobFromImage(src, 1.0 / 255, cv::Size(64, 64), cv::Scalar(0), false, false);
    this->net.setInput(inputBlob);
    cv::Mat output = this->net.forward();

    cv::Mat prob = softmax(output.reshape(1, 1));

    cv::Point classIdPoint;
    double confidence;
    cv::minMaxLoc(prob, nullptr, &confidence, nullptr, &classIdPoint);
    int classId = classIdPoint.x;

    if (confidence > this->confidence_threshold)
        return classIdPoint.x;
    else
        return -1;
}

void Classifier::preprocess(cv::Mat &src) { cv::cvtColor(src, src, cv::COLOR_BGR2GRAY); }

int Classifier::classify(cv::Mat &roi) { return this->inference(roi); }
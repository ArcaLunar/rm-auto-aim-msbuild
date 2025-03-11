#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include "classifier.hpp"
#include "config.hpp"
#include "structs.hpp"

#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <string>
#include <toml++/toml.h>

AutoAim::Classifier::Classifier(const std::string &config_path) {
    toml::table T;
    if constexpr (InitializationDebug)
        spdlog::info("initializing classifier with config file: \"{}\"", config_path);

    try {
        T                           = toml::parse_file(config_path);
        const auto &model_path      = T["mlp"]["model_path"].value<std::string>();
        const auto &labels          = T["mlp"]["labels"];
        this->confidence_threshold_ = T["mlp"]["confidence_threshold"].value_or(0);
        const auto &ignore_classes  = T["mlp"]["ignore_class"];

        if (!model_path.has_value())
            throw std::runtime_error("model_path not found in config file");
        if constexpr (InitializationDebug)
            spdlog::info("loading model from {}", model_path.value());

        net_ = cv::dnn::readNetFromONNX(model_path.value());

        // 提取识别标签
        if (toml::array *arr = labels.as_array()) {
            arr->for_each([&](auto &&element) {
                if (const auto &str_element = element.as_string())
                    labels_.push_back(str_element->get());
            });
        }
        // 提取忽略标签
        if (toml::array *arr = ignore_classes.as_array()) {
            arr->for_each([&](auto &&element) {
                if (const auto &str_element = element.as_string())
                    ignore_.push_back(str_element->get());
            });
        }
    } catch (const toml::parse_error &err) {
        spdlog::error("fail to parse config file, {}", err.description());
        exit(-1);
    } catch (const std::exception &e) {
        spdlog::error("fail to parse config file, {}", e.what());
        exit(-1);
    }

    if constexpr (InitializationDebug)
        spdlog::info("classifier initialization done");
}

cv::Mat AutoAim::Classifier::extract_region_of_interest(const cv::Mat &img, const Armor &armor) {
    if constexpr (ClassifierDebug)
        spdlog::info("extracting ROI from armor, performing test");

    if (armor.vertices.size() != 4) {
        spdlog::error("invalid armor vertices size: {}", armor.vertices.size());
        exit(-1);
    } else if constexpr (ClassifierDebug)
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
        = {cv::Point2f(0, 0),
           cv::Point2f(ModelInputWidth, 0),
           cv::Point2f(ModelInputWidth, ModelInputHeight),
           cv::Point2f(0, ModelInputHeight)}; // model.onnx 为 64x64
    cv::Mat warpMatrix = cv::getPerspectiveTransform(src_points, dst_points);
    cv::warpPerspective(img, pattern_img, warpMatrix, cv::Size(ModelInputWidth, ModelInputHeight));

    return pattern_img;
}

std::vector<cv::Mat>
AutoAim::Classifier::extract_region_of_interest(const cv::Mat &img, const std::vector<Armor> &armors) {
    std::vector<cv::Mat> rois;
    for (const auto &armor : armors)
        rois.push_back(extract_region_of_interest(img, armor));
    return rois;
}

AutoAim::Labels AutoAim::Classifier::classify(cv::Mat &roi) {
    auto result = inference(roi);
    switch (result) {
        case 1: return Labels::Hero;
        case 2: return Labels::Engineer;
        case 3: return Labels::Infantry3;
        case 4: return Labels::Infantry4;
        case 5: return Labels::Infantry5;
        case 6: return Labels::Sentry;
        case 7: return Labels::Outpost;
        case 8: return Labels::Base;
        default: return Labels::None;
    }
}

cv::Mat AutoAim::Classifier::softmax(const cv::Mat &src) {
    cv::Mat dst;
    float sum = 0.0, max = 0.0;

    max = *std::max_element(src.begin<float>(), src.end<float>());
    cv::exp(src - max, dst);
    sum = cv::sum(dst)[0];
    dst /= sum;

    return dst;
}

int AutoAim::Classifier::inference(cv::Mat &src) {
    preprocess(src); //* 预处理

    cv::Mat inputBlob = cv::dnn::blobFromImage(
        src, 1.0 / 255, cv::Size(ModelInputWidth, ModelInputHeight), cv::Scalar(0), false, false
    );
    net_.setInput(inputBlob);
    cv::Mat output = net_.forward();

    cv::Mat prob = softmax(output.reshape(1, 1));

    cv::Point classIdPoint;
    double confidence;
    cv::minMaxLoc(prob, nullptr, &confidence, nullptr, &classIdPoint);
    int classId = classIdPoint.x;

    if (confidence > confidence_threshold_)
        return classIdPoint.x;
    else
        return -1;
}

void AutoAim::Classifier::preprocess(cv::Mat &src) { cv::cvtColor(src, src, cv::COLOR_BGR2GRAY); }
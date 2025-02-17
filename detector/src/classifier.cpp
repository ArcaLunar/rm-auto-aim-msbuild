#include "classifier.hpp"

#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <toml++/toml.h>

void AutoAim::Classifier::extract_region_of_interest(const cv::Mat &img, const std::vector<Armor> &armors) {
    const int warp_height  = 28;
    const int light_length = 12;
    const cv::Size roi_size(20, 28);

    spdlog::info("extracting region of interest from armor...");
    //* 从装甲板中心提取数字区域
    for (auto &armor : armors) {
        // 透视变换
        cv::Point2f src_points[] = {armor.left.bottom, armor.right.top, armor.right.top, armor.right.bottom};
        const int top_light_y    = (warp_height - light_length) / 2 - 1;
        const int bottom_light_y = top_light_y + light_length;
        const int warp_width     = 32;

        cv::Point2f dst_points[]
            = {cv::Point2f(0, bottom_light_y),
               cv::Point2f(0, top_light_y),
               cv::Point2f(warp_width - 1, top_light_y),
               cv::Point2f(warp_width - 1, bottom_light_y)};

        cv::Mat number_img;
        auto rotation_matrix = cv::getPerspectiveTransform(src_points, dst_points);
        cv::warpPerspective(img, number_img, rotation_matrix, cv::Size(warp_width, warp_height));

        // 提取数字区域
        number_img = number_img(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

        // 二值化
        cv::cvtColor(number_img, number_img, cv::COLOR_BGR2GRAY);
        cv::threshold(number_img, number_img, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        number_img.copyTo(armor.number_img);
    }
    spdlog::info("batch extracting finished");
}

AutoAim::Classifier::Classifier(const std::string &config_path) {
    toml::table T;
    try {
        T                      = toml::parse_file(config_path);
        const auto &model_path = T["model_path"].value<std::string>();
        const auto &labels     = T["labels"];
        threshold_             = T["threshold"].value_or(0);
        // ignore_classes =

        spdlog::info("loading model from {}", model_path.value());
        net_ = cv::dnn::readNetFromONNX(model_path.value());

        // Get all labels as strings
        if (toml::array *arr = labels.as_array()) {
            arr->for_each([&](auto &&element) {
                if (const auto &str_element = element.as_string()) labels_.push_back(str_element->get());
            });
        }
    } catch (const toml::parse_error &err) {
        spdlog::error("fail to parse config file, {}", err.description());
        exit(-1);
    }
}

void AutoAim::Classifier::classify(std::vector<Armor> &armors) {
    spdlog::info("classifying armor...");
    for (auto &armor : armors) {
        cv::Mat img = armor.number_img.clone();

        // Normalize
        img.convertTo(img, CV_32F, 1.0 / 255);

        // Create blob
        cv::Mat blob;
        cv::dnn::blobFromImage(img, blob);

        // 准备 MLP
        net_.setInput(blob);
        cv::Mat output = net_.forward();

        // Softmax
        float max_prob = *std::max_element(output.begin<float>(), output.end<float>());
        cv::Mat softmax_prob;
        cv::exp(output - max_prob, softmax_prob);
        float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
        softmax_prob /= sum;

        // 计算置信度
        double confidence;
        cv::Point class_id_point;
        cv::minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
        int class_id = class_id_point.x;

        armor.confidence = confidence;
        armor.result     = labels_[class_id];
        armor.summary    = fmt::format("result: {}, confidence: {:.2f}%", armor.result, armor.confidence * 100.0);
    }

    // Filter armors that do not meet the requirements
    armors.erase(std::remove_if(armors.begin(), armors.end(), [this](const Armor &armor) {
        if (armor.confidence < threshold_) {
            spdlog::info("droping: confidence too low, ignore armor with confidence: {:.2f}", armor.confidence);
            return true;
        }

        for (const auto &ignore_class : ignore_) {
            if (armor.result == ignore_class) {
                spdlog::info("droping: ignore class, ignore armor with result: {}", armor.result);
                return true;
            }
        }

        return false;
    }));
}

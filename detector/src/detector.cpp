#include "structs.hpp"
#include "config.hpp"
#include "detector.hpp"
#include <opencv2/opencv.hpp>

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

std::vector<AutoAim::LightBar> AutoAim::Detector::DetectLightBars(const cv::Mat &rgb, const cv::Mat &binary) {
    // 用 contour 轮廓找出灯条
    std::vector<std::vector<cv::Point2f>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<LightBar> lights;
    for (const auto &contour : contours) {
        if (contour.size() < 5) continue;

        auto r_rect = cv::minAreaRect(contour);
        auto light  = LightBar(r_rect);

        if (light.isValid(this->light_bar_config_)) {
            auto rect = light.boundingRect();
            // 过滤不可能的灯条
            if (rect.x < 0 || rect.y < 0 || rect.x + rect.width > rgb.cols || rect.y + rect.height > rgb.rows
                || rect.width < 0 || rect.height < 0) {
                continue;
            }

            int sum_r = 0, sum_b = 0;
            auto roi = rgb(rect); // RGB 下灯条区域
            for (int i = 0; i < roi.rows; i++) {
                for (int j = 0; j < roi.cols; j++) {
                    sum_r += roi.at<cv::Vec3b>(i, j)[0];
                    sum_b += roi.at<cv::Vec3b>(i, j)[2];
                }
            }

            // 用红色、蓝色灯条的像素值的和 判断灯条颜色
            light.color = sum_r > sum_b ? "red" : "blue";
            spdlog::info(
                "detected lightbar: color={}, angle={}, length={}, width={}",
                light.color,
                light.angle,
                light.length,
                light.width
            );
            lights.push_back(light);
        }
    }

    return lights;
}

std::vector<AutoAim::Armor> AutoAim::Detector::MatchLightBars(const std::vector<AutoAim::LightBar> &lights) {
    std::vector<AutoAim::Armor> armors;

    // 两两枚举进行匹配
    for (auto light1 = lights.begin(); light1 != lights.end(); light1++) {
        for (auto light2 = light1 + 1; light2 != lights.end(); light2++) {
            if (light1->color != COLOR_TO_DETECT || light2->color != COLOR_TO_DETECT) continue;

            if (this->ContainAnotherLightBar(*light1, *light2, lights)) continue;
            if (Armor::isValid(this->armor_config_, *light1, *light2)) armors.emplace_back(*light1, *light2);
        }
    }
    return armors;
}

bool AutoAim::Detector::ContainAnotherLightBar(
    const AutoAim::LightBar &light1, const AutoAim::LightBar &light2, const std::vector<AutoAim::LightBar> &lights
) {
    auto points = std::vector<cv::Point2f>{light1.top, light1.bottom, light2.top, light2.bottom};
    auto rect   = cv::boundingRect(points);

    for (const auto &light : lights) {
        if (light.center == light1.center || light.center == light2.center) continue;

        if (rect.contains(light.center) || rect.contains(light.top) || rect.contains(light.bottom)) return true;
    }

    return false;
}

void AutoAim::Detector::draw_results_to_image(cv::Mat &img, const std::vector<Armor> &armors) {
    if constexpr (!SHOW_ANNOTATED_IMAGE) return;

    // draw armors
    for(const auto &armor: armors) {
        cv::line(img, armor.left.top, armor.right.bottom, cv::Scalar(0, 255, 0), 2);
        cv::line(img, armor.left.bottom, armor.right.top, cv::Scalar(0, 255, 0), 2);
    }
}
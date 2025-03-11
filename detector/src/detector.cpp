#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include "detector.hpp"
#include "config.hpp"
#include "structs.hpp"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

//! Detector
AutoAim::Detector::Detector(std::string path) : light_bar_config_(path), armor_config_(path) {
    spdlog::info("Detector initialized with config file: \"{}\"", path);
}

std::vector<AutoAim::Armor> AutoAim::Detector::detect(const cv::Mat &img) {
    auto binary = this->preprocess_image(img);
    cv::imshow("binary", binary);
    cv::waitKey();
    auto lights = this->detect_lightbars(img, binary);
    auto armors = this->pair_lightbars(lights);

    return armors;
}

cv::Mat AutoAim::Detector::preprocess_image(const cv::Mat &src) {
    cv::Mat gray, grayColor, binary_color, binary, binary_brightness;

    // 提取亮度
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, binary_brightness, this->light_bar_config_.brightness_threshold, 255, cv::THRESH_BINARY);

    // 红蓝通道作差，提取颜色
    std::vector<cv::Mat> channels;
    cv::split(src, channels);
    int enemy_color = EnemyColor == RMColor::Blue ? 0 : 2;
    int ally_color  = EnemyColor == RMColor::Blue ? 2 : 0;
    cv::subtract(channels[enemy_color], channels[ally_color], grayColor);

    cv::threshold(grayColor, binary_color, light_bar_config_.color_threshold, 255, cv::THRESH_BINARY);
    cv::bitwise_and(binary_brightness, binary_color, binary);
    // cv::erode(binary, binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)), cv::Point(0, 0), 7);
    cv::dilate(binary, binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)), cv::Point(-1, -1), 7);

    if constexpr (DetectorDebug)
        spdlog::info("preprocessed image");

    return binary;
}

std::vector<AutoAim::LightBar> AutoAim::Detector::detect_lightbars(const cv::Mat &rgb, const cv::Mat &binary) {
    // 用 contour 轮廓找出灯条
    if constexpr (DetectorDebug)
        spdlog::info("detecting lightbars");

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

    if constexpr (DetectorDebug)
        spdlog::info("found {} contours, filtering lightbars", contours.size());

    std::vector<LightBar> lights;
    for (int i = 0; i < contours.size(); i++) {
        if (contours[i].size() < 5) {
            spdlog::error("skipping, less than 5 points in the contour");
            continue;
        }
        if (hierarchy[i][3] != -1) {
            spdlog::error("skipping, has parent hierachy");
            continue;
        }

        LightBar light(contours[i]);
        if (light.is_valid(this->light_bar_config_))
            lights.push_back(light);

        // 绘制灯条（调试用）
        if constexpr (DetectorDebug)
            cv::ellipse(this->debug_frame, light.ellipse, cv::Scalar(0, 0, 255), 2);
    }

    if constexpr (DetectorDebug)
        spdlog::info("detected {} lightbars", lights.size());

    return lights;
}

std::vector<AutoAim::Armor> AutoAim::Detector::pair_lightbars(std::vector<AutoAim::LightBar> &lights) {
    if constexpr (DetectorDebug)
        spdlog::info("start pairing");
    std::vector<AutoAim::Armor> armors;
    std::sort(lights.begin(), lights.end(), [](const LightBar &a, const LightBar &b) {
        return a.center().x < b.center().x;
    });

    // 两两枚举进行匹配
    for (auto light1 = lights.begin(); light1 != lights.end(); light1++) {
        for (auto light2 = light1 + 1; light2 != lights.end(); light2++) {
            Armor tmp(*light1, *light2);
            // 检查灯条中间是否还夹着其他灯条，是的话不可能组成装甲板
            if constexpr (DetectorDebug)
                spdlog::info("checking mispair");
            if (this->check_mispair(tmp, lights)) {
                if constexpr (DetectorDebug)
                    spdlog::error("mispair_check failed");
                continue;
            } else if constexpr (DetectorDebug)
                spdlog::info("mispair_check passed");

            // 检查组成的装甲板是否合法
            if constexpr (DetectorDebug)
                spdlog::info("doing armor_validation check");
            if (tmp.is_valid(this->armor_config_)) {
                if constexpr (DetectorDebug)
                    spdlog::info("armor_validation passed");
                armors.push_back(tmp);
            } else if constexpr (DetectorDebug)
                spdlog::error("armor_validation failed");
        }
    }
    if constexpr (DetectorDebug)
        spdlog::info("paired {} armors", armors.size());

    return armors;
}

bool AutoAim::Detector::check_mispair(const Armor &armor, const std::vector<AutoAim::LightBar> &lights) {
    auto &points = armor.vertices;
    auto rect    = cv::boundingRect(points);
    for (const auto &light : lights) {
        if (light.center() == armor.left.center() || light.center() == armor.right.center())
            continue; // 忽略已经匹配的灯条

        if (cv::pointPolygonTest(points, light.center(), false) >= 0)
            return true;
    }

    return false;
}

void AutoAim::Detector::draw_results_to_image(cv::Mat &img, const std::vector<Armor> &armors) {
    if constexpr (!DisplayAnnotatedImageDebug)
        return;

    // draw armors
    for (const auto &armor : armors) {
        std::vector<cv::Point> v1{armor.vertices[0], armor.vertices[1]};
        std::vector<cv::Point> v2{armor.vertices[1], armor.vertices[2]};
        std::vector<cv::Point> v3{armor.vertices[2], armor.vertices[3]};
        std::vector<cv::Point> v4{armor.vertices[3], armor.vertices[0]};
        cv::polylines(img, v1, true, cv::Scalar(0, 0, 255), 2);
        cv::polylines(img, v2, true, cv::Scalar(0, 0, 255), 2);
        cv::polylines(img, v3, true, cv::Scalar(0, 0, 255), 2);
        cv::polylines(img, v4, true, cv::Scalar(0, 0, 255), 2);
        cv::putText(
            img,
            armor.type == ArmorType::Small ? "Small" : "Big",
            armor.center,
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            cv::Scalar(0, 255, 0),
            2
        );
    }

    // draw image
    cv::Mat tmp;
    cv::imshow("Annotated Image", img);
    cv::waitKey();
}
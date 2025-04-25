#include "detector.hpp"
#include "debug_options.hpp"
#include "opencv2/highgui.hpp"
#include "structs.hpp"

extern DebugOptions options;

OpenCVDetector::OpenCVDetector(std::string path) : lbcfg(path), acfg(path) {
    spdlog::info("OpenCVDetector initialized with config path: {}", path);
}

std::vector<RawArmor> OpenCVDetector::detect(const cv::Mat &image) {
    auto binary = this->preprocess_image(image);

    if (options.detector.display_image) {
        cv::imshow("binary", binary);
        cv::waitKey();
    }
    auto lights = this->detect_lightbars(image, binary);
    auto armors = this->pair_lightbars(lights);

    return armors;
}

cv::Mat OpenCVDetector::preprocess_image(const cv::Mat &src) {
    cv::Mat gray, grayColor, binary_color, binary, binary_brightness;

    // 提取亮度
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, binary_brightness, this->lbcfg.brightness_threshold, 255, cv::THRESH_BINARY);

    // 红蓝通道作差，提取颜色
    std::vector<cv::Mat> channels;
    cv::split(src, channels);
    RMColor color   = RMColor();
    int enemy_color = color.enemy == RMColor::Blue ? 0 : 2;
    int ally_color  = color.enemy == RMColor::Blue ? 2 : 0;
    cv::subtract(channels[enemy_color], channels[ally_color], grayColor);

    cv::threshold(grayColor, binary_color, this->lbcfg.color_threshold, 255, cv::THRESH_BINARY);
    cv::bitwise_and(binary_brightness, binary_color, binary);
    // cv::erode(binary, binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)), cv::Point(0, 0), 7);
    cv::dilate(binary, binary, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)), cv::Point(-1, -1), 7);

    if (options.detector.preprocess)
        spdlog::info("preprocessed image");

    return binary;
}

std::vector<LightBar> OpenCVDetector::detect_lightbars(const cv::Mat &rgb, const cv::Mat &binary) {
    // 用 contour 轮廓找出灯条
    if (options.detector.detect_lightbars)
        spdlog::info("detecting lightbars");

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

    if (options.detector.detect_lightbars)
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
        if (light.is_valid(this->lbcfg))
            lights.push_back(light);

        // 绘制灯条（调试用）
        if (options.detector.detect_lightbars)
            cv::ellipse(this->debug_frame, light.ellipse, cv::Scalar(0, 0, 255), 2);
    }

    if (options.detector.detect_lightbars)
        spdlog::info("detected {} lightbars", lights.size());

    return lights;
}

std::vector<RawArmor> OpenCVDetector::pair_lightbars(std::vector<LightBar> &lights) {
    if (options.detector.pair_lightbars)
        spdlog::info("start pairing");
    std::vector<RawArmor> armors;
    std::sort(lights.begin(), lights.end(), [](const LightBar &a, const LightBar &b) {
        return a.center().x < b.center().x;
    });

    // 两两枚举进行匹配
    for (auto light1 = lights.begin(); light1 != lights.end(); light1++) {
        for (auto light2 = light1 + 1; light2 != lights.end(); light2++) {
            RawArmor tmp(*light1, *light2);
            // 检查灯条中间是否还夹着其他灯条，是的话不可能组成装甲板
            if (options.detector.pair_lightbars)
                spdlog::info("checking mispair");
            if (this->check_mispair(tmp, lights)) {
                if (options.detector.pair_lightbars)
                    spdlog::error("mispair_check failed");
                continue;
            } else if (options.detector.pair_lightbars)
                spdlog::info("mispair_check passed");

            // 检查组成的装甲板是否合法
            if (options.detector.pair_lightbars)
                spdlog::info("doing armor_validation check");
            if (tmp.is_valid(this->acfg)) {
                if (options.detector.pair_lightbars)
                    spdlog::info("armor_validation passed");
                armors.push_back(tmp);
            } else if (options.detector.pair_lightbars)
                spdlog::error("armor_validation failed");
        }
    }
    if (options.detector.pair_lightbars)
        spdlog::info("paired {} armors", armors.size());

    return armors;
}
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include "publisher.hpp"
#include "config.hpp"
#include "structs.hpp"

#include <spdlog/spdlog.h>

AutoAim::Publisher::Publisher(const std::string &config_path) {
    if constexpr (PublisherDebug)
        spdlog::info("Publisher::initializing with config path: {}", config_path);
    detector_   = std::make_shared<Detector>(config_path);
    classifier_ = std::make_shared<Classifier>(config_path);
}

std::vector<AnnotatedArmorInfo> AutoAim::Publisher::annotate_image(const RawFrameInfo &raw, const IMUInfo &imu) {
    if constexpr (PublisherDebug)
        spdlog::info("Publisher::annotating image");

    auto armors = detector_->detect(raw.frame);
    if constexpr (PublisherDebug)
        spdlog::info("Publisher::number of armors detected: {}", armors.size());

    if constexpr (PublisherDiaplayImageDebug) {
        cv::Mat frame = raw.frame;
        detector_->draw_results_to_image(frame, armors);
    }

    std::vector<AnnotatedArmorInfo> annotated;

    for (auto &armor : armors) {
        auto roi   = classifier_->extract_region_of_interest(raw.frame, armor);
        auto label = classifier_->classify(roi);
        if constexpr (PublisherDebug)
            spdlog::info("Publisher::label: {}", (int)label);

        annotated.emplace_back(armor, label, imu, raw.timestamp);
    }

    return annotated;
}
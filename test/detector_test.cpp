#include "classifier.hpp"
#include "detector.hpp"
#include "publisher.hpp"

#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>

int main() {
    // auto detector
    //     = std::make_shared<AutoAim::Detector>("/media/arca/ArcaEXT4/codebases/pred_v2/config/detection_tr.toml");

    // auto img_path = "/home/arca/ArcaEXT4/_datasets/rm-2025/green-armor/JPEGImages/BB1_0_0_12_1.jpg";
    // auto classifier =
    // std::make_shared<AutoAim::Classifier>("/media/arca/ArcaEXT4/codebases/pred_v2/config/detection_tr.toml");

    // cv::Mat img   = cv::imread(img_path);
    // auto armors   = detector->detect(img);

    // spdlog::info("number of armors detected: {}", armors.size());
    // detector->draw_results_to_image(img, armors);

    // for(auto &armor: armors) {
    //     auto roi = classifier->extract_region_of_interest(img, armor);
    //     auto label = classifier->classify(roi);
    //     spdlog::info("label: {}", (int)label);
    // }
    auto publisher
        = std::make_shared<AutoAim::Publisher>("/media/arca/ArcaEXT4/codebases/pred_v2/config/detection_tr.toml");
    auto img_path = "/home/arca/ArcaEXT4/_datasets/rm-2025/green-armor/JPEGImages/BB1_0_0_12_1.jpg";
    cv::Mat img   = cv::imread(img_path);
    auto armors   = publisher->annotate_image(
        {img, std::chrono::system_clock::now()}, {0, 0, 0, std::chrono::system_clock::now()}
    );

    for(auto &armor: armors) {
        spdlog::info("label: {}", (int)armor.result);
    }
}
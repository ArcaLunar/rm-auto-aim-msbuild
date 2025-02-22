#include <opencv2/core/utility.hpp>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>

#include "cam_capture.hpp"

int main() {
    spdlog::info("starting auto_aim");
    spdlog::info("activating camera");

    HikCamera camera;
    while (true) {
        auto start = cv::getTickCount();
        cv::Mat frame = camera.get_frame().frame;
        auto end = cv::getTickCount();

        spdlog::info("fps: {}", cv::getTickFrequency() / (end - start));

        cv::imshow("frame", frame);
        auto k = cv::waitKey(1) & 0xFF;
        if (k == 27) break;
    }
}
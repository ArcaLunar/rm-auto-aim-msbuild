#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>

#include "cam_capture.hpp"

int main() {
    spdlog::info("starting auto_aim");
    spdlog::info("activating camera");

    HikCamera camera;
    while (true) {
        cv::Mat frame = camera.get_frame();
        cv::imshow("frame", frame);
        auto k = cv::waitKey(1) & 0xFF;
        if (k == 27) break;
    }
}
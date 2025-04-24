#include "camera.hpp"
#include <atomic>
#include <opencv2/highgui.hpp>
#include <spdlog/spdlog.h>

volatile std::atomic_bool require_exit = false;

void press_enter_to_exit() {
    int c;
    while ((c = getchar()) != '\n' && c != EOF)
        continue;

    spdlog::error("[FROM main] Press Enter to exit");
    while (getchar() != '\n')
        continue;

    require_exit = true;
    sleep(1);
}

int main() {
    HikCamera camera;

    std::thread run([&] {
        while (true) {
            auto start_time     = std::chrono::steady_clock::now();
            RawImageFrame frame = camera.get_frame();

            if (frame.image.empty()) {
                spdlog::error("Failed to get frame");
                continue;
            }

            // display image
            // cv::imshow("Camera", frame.image);
            auto end_time = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

            // print some stats
            spdlog::info("frame size: {}x{}", frame.image.cols, frame.image.rows);

            spdlog::info("capture and display used {} ms", duration.count());

            if (require_exit) {
                spdlog::info("Exiting...");
                break;
            }
        }
    });

    press_enter_to_exit();
    run.join();

    spdlog::info("exit successfully");
    return 0;
}
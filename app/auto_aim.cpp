#include <exception>
#include <spdlog/spdlog.h>

#include "cam_capture.hpp"
#include "structs.hpp"
#include "work_queue.hpp"

int main() {
    spdlog::info("starting auto_aim");
    spdlog::info("activating camera");

    WorkQueue<AnnotatedArmorInfo> work_queue;
    work_queue.set_producer(
        [&]() -> AnnotatedArmorInfo {
            try {
                spdlog::info("image producer has loaded.");
            } catch (std::exception &err) {
                spdlog::critical("error in auto_aim.main(): {}", err.what());
                exit(-1);
            }
        },
        1
    );
    work_queue.set_consumer(
        [&](const AnnotatedArmorInfo &info) {
            try {
                spdlog::info("image consumer has loaded.");
            } catch (std::exception &err) {
                spdlog::critical("error in auto_aim.main(): {}", err.what());
                exit(-1);
            }
        },
        1
    );
    work_queue.start();

    sleep(1);
    work_queue.stop();
}
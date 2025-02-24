#include "cam_capture.hpp"
#include "serial_port.hpp"
#include "structs.hpp"
#include "work_queue.hpp"
#include <spdlog/spdlog.h>
#include <thread>

int main() {
    // HikCamera cam;
    SerialPort port("/media/arca/ArcaEXT4/codebases/pred_v2/config/comm.toml");
    port.initialize_port();

    std::thread([&]() { port.read_raw_data_from_port(); }).detach();

    std::thread([&]() { port.process_raw_data_from_buffer(); }).detach();

    spdlog::info("start receiving test");
    while (true) {
        auto msg = port.get_data();
        if (msg.has_value()) {
            spdlog::info(
                "data received: roll={} pitch={} yaw={}",
                msg.value().imu_roll,
                msg.value().imu_pitch,
                msg.value().imu_yaw
            );
        }
        // } else spdlog::warn("no data received");
    }
}
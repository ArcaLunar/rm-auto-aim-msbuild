#include "cam_capture.hpp"
#include "serial_port.hpp"
#include "structs.hpp"
#include "work_queue.hpp"
#include <memory>
#include <spdlog/spdlog.h>
#include <thread>

int main() {
    auto cam  = std::make_shared<HikCamera>("/media/arca/ArcaEXT4/codebases/pred_v2/config/cam.toml");
    auto port = std::make_shared<SerialPort>("/media/arca/ArcaEXT4/codebases/pred_v2/config/comm.toml");
    port->initialize_port();

    std::thread([&]() { port->read_raw_data_from_port(); }).detach();
    std::thread([&]() { port->process_raw_data_from_buffer(); }).detach();

    auto raw_img = std::make_shared<DataTransmitter<RawFrameInfo>>();
    raw_img->register_producer([cam]() -> RawFrameInfo { return cam->get_frame(); });
    raw_img->register_consumer([&](const RawFrameInfo &frame) {
        spdlog::info("Frame captured at {}", frame.timestamp.time_since_epoch().count());
        auto recv_msg = port->get_data();
        if (recv_msg.has_value()) {
            IMUInfo imu;
            imu.load_from_recvmsg(recv_msg.value());
            spdlog::info(
                "frame width: {}, height: {}, imu.roll: {}, imu.pitch: {}, imu.yaw: {}",
                frame.frame.cols,
                frame.frame.rows,
                imu.roll,
                imu.pitch,
                imu.yaw
            );
        }
    });

    raw_img->start();

    sleep(60);
    spdlog::info("stopping data transfer");
    raw_img->stop();
    return 0;
}
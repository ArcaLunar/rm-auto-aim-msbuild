#include "camera.hpp"
#include "port.hpp"
#include "structs.hpp"

#include "spdlog/spdlog.h"

int main() {
    spdlog::info("Starting main application...");

    SerialPort port;

    // start port receive
    std::thread port_reader(&SerialPort::read_raw, &port);
    std::thread port_processor(&SerialPort::process_raw, &port);
    std::thread port_checker(&SerialPort::check_reconnect, &port);

    // image producer
    std::thread img_producer([&] {
        HikCamera cam;
        while (true) {
            auto frame = cam.get_frame();
        }
    });
}
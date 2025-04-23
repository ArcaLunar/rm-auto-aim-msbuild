#include "camera.hpp"
#include "port.hpp"
#include "structs.hpp"

#include "spdlog/spdlog.h"

int main() {
    spdlog::info("Starting main application...");

    SerialPort<SentryVisionRecvMsg> port;

    // start port receive
    std::thread port_reader(&SerialPort<SentryVisionRecvMsg>::read_raw, &port);
    std::thread port_processor(&SerialPort<SentryVisionRecvMsg>::process_raw, &port);
    std::thread port_checker(&SerialPort<SentryVisionRecvMsg>::check_reconnect, &port);

    // image producer
    std::thread img_producer([&] {
        HikCamera cam;
        while (true) {
            auto frame = cam.get_frame();
        }
    });
}
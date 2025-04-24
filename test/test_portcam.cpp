#include "camera.hpp"
#include "circular_buffer.hpp"
#include "port.hpp"
#include "spdlog/spdlog.h"
#include "structs.hpp"
#include <cstdlib>
#include <memory>
#include <thread>

int main() {
    SerialPort<SentryVisionRecvMsg> port("../config/port.toml");

    // start port receive
    std::thread port_reader(&SerialPort<SentryVisionRecvMsg>::read_raw, &port);
    std::thread port_processor(&SerialPort<SentryVisionRecvMsg>::process_raw, &port);
    std::thread port_checker(&SerialPort<SentryVisionRecvMsg>::check_reconnect, &port);

    // image producer
    std::shared_ptr<CircularBuffer<RawImageFrame>> img_buffer = std::make_shared<CircularBuffer<RawImageFrame>>(100);
    std::thread img_producer([&] {
        try {
            HikCamera cam;
            while (true) {
                RawImageFrame frame = cam.get_frame();
                if (frame.image.empty()) {
                    spdlog::error("Failed to get frame");
                    std::this_thread::yield();
                    continue;
                }

                img_buffer->push(frame);
            }
        } catch (const std::exception &e) {
            spdlog::error("Error in image producer: {}", e.what());
            std::exit(-1);
        }
    });

    std::thread img_consumer([&] {
        while (true) {
            auto [frame, verdict] = img_buffer->pop();
            if (!verdict) {
                spdlog::warn("No image available");
                std::this_thread::yield();
                continue;
            }

            // simply print some stats here
            spdlog::info("Image size: {}x{}", frame.image.cols, frame.image.rows);
            spdlog::info("Image size: {} bytes", frame.image.total() * frame.image.elemSize());
            spdlog::info("Image timestamp: {}", frame.timestamp.time_since_epoch().count());
        }
    });

    // join them all
    port_reader.join();
    port_processor.join();
    port_checker.join();
    img_producer.join();
    img_consumer.join();
    
    return 0;
}
#include "annotator.hpp"
#include "camera.hpp"
#include "debug_options.hpp"
#include "port.hpp"
#include "structs.hpp"
#include <sstream>

extern DebugOptions options;

int main() {
    options.camera.initialization = false;
    options.camera.capture        = false;
    options.port.initialization   = false;
    options.port.inspect_data     = false;

    std::shared_ptr<SerialPort<SentryVisionRecvMsg>> port
        = std::make_shared<SerialPort<SentryVisionRecvMsg>>("../config/port.toml");

    // start port receive
    std::thread port_reader([&] { port->read_raw(); });
    std::thread port_processor([&] { port->process_raw(); });
    std::thread port_checker([&] { port->check_reconnect(); });

    // image producer
    std::shared_ptr<CircularBuffer<RawImageFrame>> img_buffer = std::make_shared<CircularBuffer<RawImageFrame>>(100);
    std::thread img_producer([&] {
        HikCamera cam;
        spdlog::info("initialized");
        while (true) {
            RawImageFrame frame = cam.get_frame();
            spdlog::info("get frame");
            if (frame.image.empty()) {
                spdlog::error("Failed to get frame");
                continue;
            }

            spdlog::info("Got frame: {}x{}", frame.image.cols, frame.image.rows);
            spdlog::info("Frame size: {} bytes", frame.image.total() * frame.image.elemSize());
            img_buffer->push(frame);
        }
    });

    std::thread img_consumer([&] {
        Annotator annotator;
        while (true) {
            auto [frame, verdict] = img_buffer->pop();
            if (!verdict) {
                continue;
            }

            // simply print some stats here
            spdlog::info("Image size: {}x{}", frame.image.cols, frame.image.rows);
            spdlog::info("Image size: {} bytes", frame.image.total() * frame.image.elemSize());
            spdlog::info("Image timestamp: {}", frame.timestamp.time_since_epoch().count());

            auto msg    = port->get_imu();
            auto armors = annotator.annotate(frame, msg);

            std::stringstream ss;
            ss << "Annotated armors: ";
            for (const auto &armor : armors) {
                ss << armor.result << " ";
            }
            spdlog::info("detected: {}", ss.str());
            spdlog::info("IMU data: roll={} pitch={} yaw={}", msg.roll, msg.pitch, msg.yaw);
        }
    });

    port_reader.join();
    port_processor.join();
    port_checker.join();
    img_producer.join();
    img_consumer.join();
    return 0;
}
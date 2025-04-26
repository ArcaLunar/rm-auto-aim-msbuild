#include "annotator.hpp"
#include "camera.hpp"
#include "circular_buffer.hpp"
#include "debug_options.hpp"
#include "port.hpp"
#include "pose_convert.hpp"
#include "structs.hpp"
#include <memory>
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

    auto armor_buffer = std::make_shared<CircularBuffer<std::vector<AnnotatedArmorInfo>>>(100);
    auto to_filter    = std::make_shared<CircularBuffer<std::vector<Armor3d>>>(100);
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

            armor_buffer->push(armors);
            spdlog::info("Pushed {} armors to buffer", armors.size());
        }
    });
    auto posetf = std::make_shared<PoseCVT>("../config/transform.toml");
    std::thread armor_tf([&] {
        std::vector<Armor3d> arms;
        u8 upd = 1;
        while (true) {
            auto [armors, verdict] = armor_buffer->pop();
            if (!verdict)
                continue;

            arms.clear();
            for (const auto &armor : armors) {
                auto tf_armor = posetf->solve_absolute(armor);

                // print some stats
                spdlog::info(
                    "Armor center: ({},{},{})",
                    tf_armor.p_barrel.center_3d.at<double>(0),
                    tf_armor.p_barrel.center_3d.at<double>(1),
                    tf_armor.p_barrel.center_3d.at<double>(2)
                );
                spdlog::info("Armor distance: {}", tf_armor.p_barrel.distance);
                spdlog::info("Armor roll: {}", tf_armor.p_barrel.roll);
                spdlog::info("Armor pitch: {}", tf_armor.p_barrel.pitch);
                spdlog::info("Armor yaw: {}", tf_armor.p_barrel.yaw);
                spdlog::info("Armor direction: {}", tf_armor.p_barrel.direction);
                spdlog::info("Armor bullet flying time: {}", tf_armor.bullet_flying_time);
                spdlog::info("Armor pitch relative to barrel: {}", tf_armor.pitch_relative_to_barrel);
                spdlog::info("Armor yaw relative to barrel: {}", tf_armor.yaw_relative_to_barrel);
            }
            // to_filter->push(arms);

            // as test, directly send the coord to port
            port->send_message(VisionPLCSendMsg{
                .start            = 0xA3,
                .pitch            = static_cast<float>(arms[0].p_barrel.pitch),
                .yaw              = static_cast<float>(arms[0].p_barrel.yaw),
                .flag_found       = 1,
                .flag_fire        = 1,
                .flag_donefitting = 1,
                .flag_patrol      = 0,
                .flag_updated     = (upd = upd == 1 ? 0 : 1),
                .end              = 0xAA,
            });
        }
    });

    port_reader.join();
    port_processor.join();
    port_checker.join();
    img_producer.join();
    img_consumer.join();
    armor_tf.join();

    return 0;
}
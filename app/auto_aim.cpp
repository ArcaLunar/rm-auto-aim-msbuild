#include "config.hpp"
#include "tracker.hpp"
#include <map>
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#define PWD std::string("/media/arca/ArcaEXT4/codebases/pred_v2/")
#define CONFIG_PATH std::string(PWD + "config/")

#include <boost/exception/diagnostic_information.hpp>
#include <boost/exception/exception.hpp>
#include <exception>
#include <memory>
#include <spdlog/spdlog.h>

#include "cam_capture.hpp"
#include "firing.hpp"
#include "policy.hpp"
#include "pose_convert.hpp"
#include "publisher.hpp"
#include "serial_port.hpp"
#include "structs.hpp"
#include "work_queue.hpp"

int main() {
    try {
        std::shared_ptr<spdlog::logger> log = spdlog::stdout_color_mt("main");
        log->set_level(spdlog::level::trace);
        log->set_pattern("[%H:%M:%S, +%4oms] [%15s:%3# in %!] [%^%l%$] %v");
        //! open and initialize camera
        auto cam = std::make_shared<HikCamera>(CONFIG_PATH + "cam.toml");

        //! open and initialize port
        auto port = std::make_shared<SerialPort>(CONFIG_PATH + "comm.toml");
        port->initialize_port();

        //! create a detector for detecting armor
        auto detector = std::make_shared<AutoAim::Publisher>(CONFIG_PATH + "detection_tr.toml");

        //! create a coordinate transformer
        auto pose_transformer = std::make_shared<AutoAim::PoseConvert>(CONFIG_PATH + "transform.toml");

        //* start thread to read and process raw data from port
        std::thread read_from_port([&] { port->read_raw_data_from_port(); });
        SPDLOG_LOGGER_INFO(log, "reading-raw-from-port thread started");
        std::thread process_port_data([&] { port->process_raw_data_from_buffer(); });
        SPDLOG_LOGGER_INFO(log, "processing-raw-from-port thread started");

        //! create several sync queues for
        // 1. "image + imu => armors (2D) + label + type(large/small) + imu"
        // 2. "armors(2D) => armors(3D), done in the same sync queue"
        // 3. "armors(3D) => forward to corresponding tracker"
        // 4. "armors(3D) => forward to filtering Policy"
        SPDLOG_LOGGER_INFO(log, "creating sync queues");
        auto to_tf     = std::make_shared<SyncQueue<std::vector<AnnotatedArmorInfo>>>();
        auto to_filter = std::make_shared<SyncQueue<std::vector<Armor3d>>>();

        // producer (1): raw image from camera
        std::thread annotate_img([&] {
            RawFrameInfo raw_frame;
            IMUInfo imu_info;

            auto time          = std::chrono::system_clock::now();
            auto msg_grep_time = std::chrono::system_clock::now();
            auto annotate_time = std::chrono::system_clock::now();

            SPDLOG_LOGGER_INFO(log, "start annotating image");

            while (true) {
                using namespace std::chrono;
                SPDLOG_LOGGER_INFO(log, "getting raw frame");
                raw_frame = cam->get_frame();
                time      = system_clock::now();

                //* get correct recv msg
                SPDLOG_LOGGER_INFO(log, "getting correct recv msg");
                auto recv_msg = port->get_data();
                for (auto duration = time - recv_msg->timestamp; duration > milliseconds(10);
                     recv_msg      = port->get_data())
                    duration = time - recv_msg->timestamp;
                if constexpr (AnnotateImageBenchmark) {
                    msg_grep_time = system_clock::now();
                    spdlog::info("Get msg consumes {} ms", duration_cast<milliseconds>(msg_grep_time - time).count());
                }

                //* annotate
                SPDLOG_LOGGER_INFO(log, "annotating image");
                imu_info.load_from_recvmsg(*recv_msg);
                auto armor_info = detector->annotate_image(raw_frame, imu_info);
                if constexpr (AnnotateImageBenchmark) {
                    annotate_time = system_clock::now();
                    spdlog::info(
                        "Annotate image consumes {} ms",
                        duration_cast<milliseconds>(annotate_time - msg_grep_time).count()
                    );
                }

                //* push to next queue
                to_tf->write_data(armor_info);

                //* Benchmark test
                if constexpr (AnnotateImageBenchmark) {
                    auto end_time = system_clock::now();
                    spdlog::info(
                        "annotating and passing (in total) consumes {} ms",
                        duration_cast<milliseconds>(end_time - time).count()
                    );
                }
            }
        });

        //! first, create trackers for every enemy.
        std::map<AutoAim::Labels, std::shared_ptr<AutoAim::Tracker>> trackers;
        [&] {
            const std::string cfg                = CONFIG_PATH + "tracking.toml";
            trackers[AutoAim::Labels::Hero]      = std::make_shared<AutoAim::Tracker>(AutoAim::Labels::Hero, cfg);
            trackers[AutoAim::Labels::Infantry3] = std::make_shared<AutoAim::Tracker>(AutoAim::Labels::Infantry3, cfg);
            trackers[AutoAim::Labels::Engineer]  = std::make_shared<AutoAim::Tracker>(AutoAim::Labels::Engineer, cfg);
            trackers[AutoAim::Labels::Infantry4] = std::make_shared<AutoAim::Tracker>(AutoAim::Labels::Infantry4, cfg);
            trackers[AutoAim::Labels::Infantry5] = std::make_shared<AutoAim::Tracker>(AutoAim::Labels::Infantry5, cfg);
            trackers[AutoAim::Labels::Outpost]   = std::make_shared<AutoAim::Tracker>(AutoAim::Labels::Outpost, cfg);
            trackers[AutoAim::Labels::Sentry]    = std::make_shared<AutoAim::Tracker>(AutoAim::Labels::Sentry, cfg);
            trackers[AutoAim::Labels::Base]      = std::make_shared<AutoAim::Tracker>(AutoAim::Labels::Base, cfg);
        }();

        //* Transform coordinate from 2D to 3D
        //* And update tracker
        std::thread transform([&] {
            std::vector<Armor3d> arms;
            while (true) {
                auto armors = to_tf->pop_data();
                if (!armors.has_value())
                    continue;

                //* transform
                arms.clear();
                for (const auto &armor : armors.value()) {
                    auto tf_arm = pose_transformer->solve_absolute(armor);
                    spdlog::info("transformed data writen");

                    //* update tracker
                    trackers[armor.result]->update(tf_arm);
                    spdlog::info("tracker updated");

                    arms.push_back(tf_arm);
                }

                //* push to next queue
                to_filter->write_data(arms);
            }
        });

        //! filter based on policy predefined
        auto policy          = std::make_shared<SelectingPolicy>();
        auto fire_controller = std::make_shared<FireController>();

        fire_controller->set_port(port);

        std::thread filter_and_grant_fire([&] {
            while (true) {
                auto armors = to_filter->pop_data();
                if (!armors.has_value())
                    continue;

                auto which = policy->select(armors.value());
                auto state = trackers[which]->get_pred();

                spdlog::info(
                    "selected state: ({}, {}, {}) dist={} direction={}",
                    state.x,
                    state.y,
                    state.z,
                    state.distance,
                    state.direction
                );

                fire_controller->set_allow(which);
                fire_controller->try_fire(state, armors.value());
            }
        });

        read_from_port.join();
        process_port_data.join();
        annotate_img.join();
        transform.join();
        filter_and_grant_fire.join();
    } catch (std::exception &E) {
        spdlog::critical("{}", E.what());
    } catch (boost::exception &E) {
        spdlog::critical("Error with port: {}", boost::diagnostic_information(E));
    }
}
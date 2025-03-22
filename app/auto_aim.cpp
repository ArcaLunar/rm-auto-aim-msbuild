#include "config.hpp"
#include "tracker.hpp"
#include <map>
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include <boost/exception/diagnostic_information.hpp>
#include <boost/exception/exception.hpp>
#include <exception>
#include <memory>
#include <spdlog/spdlog.h>

#include "cam_capture.hpp"
#include "policy.hpp"
#include "pose_convert.hpp"
#include "publisher.hpp"
#include "serial_port.hpp"
#include "structs.hpp"
#include "work_queue.hpp"

int main() {
    try {
        //! open and initialize camera
        auto cam = std::make_shared<HikCamera>("/media/arca/ArcaEXT4/codebases/pred_v2/config/cam.toml");

        //! open and initialize port
        auto port = std::make_shared<SerialPort>("/media/arca/ArcaEXT4/codebases/pred_v2/config/comm.toml");
        port->initialize_port();

        //! create a detector for detecting armor
        auto detector
            = std::make_shared<AutoAim::Publisher>("/media/arca/ArcaEXT4/codebases/pred_v2/config/detection_tr.toml");

        //! create a coordinate transformer
        auto pose_transformer = std::make_shared<AutoAim::PoseConvert>();

        //* start thread to read and process raw data from port
        std::thread read_from_port([&] { port->read_raw_data_from_port(); });
        std::thread process_port_data([&] { port->process_raw_data_from_buffer(); });

        //! create several sync queues for
        // 1. "image + imu => armors (2D) + label + type(large/small) + imu"
        // 2. "armors(2D) => armors(3D), done in the same sync queue"
        // 3. "armors(3D) => forward to corresponding tracker"
        // 4. "armors(3D) => forward to filtering Policy"

        // auto to_annotate = std::make_shared<SyncQueue<RawFrameInfo>>(); // 1.
        auto to_transform = std::make_shared<SyncQueue<AnnotatedArmorInfo>>(); // 2.
        auto to_filter    = std::make_shared<SyncQueue<std::vector<AnnotatedArmorInfo>>>();
        // producer (1): raw image from camera
        std::thread annotate_img([&] {
            RawFrameInfo raw_frame;

            auto time          = std::chrono::system_clock::now();
            auto msg_grep_time = std::chrono::system_clock::now();
            auto annotate_time = std::chrono::system_clock::now();

            IMUInfo imu_info;
            while (true) {
                using namespace std::chrono;
                raw_frame = cam->get_frame();
                time      = system_clock::now();

                //* get correct recv msg
                auto recv_msg = port->get_data();
                for (auto duration = time - recv_msg->timestamp; duration > milliseconds(10);
                     recv_msg      = port->get_data())
                    duration = time - recv_msg->timestamp;
                if constexpr (AnnotateImageBenchmark) {
                    msg_grep_time = system_clock::now();
                    spdlog::info("Get msg consumes {} ms", duration_cast<milliseconds>(msg_grep_time - time).count());
                }

                //* annotate
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
                to_filter->write_data(armor_info);
                for (auto &info : armor_info)
                    to_transform->write_data(info);

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
            trackers[AutoAim::Labels::Hero]      = std::make_shared<AutoAim::Tracker>(AutoAim::Labels::Hero);
            trackers[AutoAim::Labels::Infantry3] = std::make_shared<AutoAim::Tracker>(AutoAim::Labels::Infantry3);
            trackers[AutoAim::Labels::Engineer]  = std::make_shared<AutoAim::Tracker>(AutoAim::Labels::Engineer);
            trackers[AutoAim::Labels::Infantry4] = std::make_shared<AutoAim::Tracker>(AutoAim::Labels::Infantry4);
            trackers[AutoAim::Labels::Infantry5] = std::make_shared<AutoAim::Tracker>(AutoAim::Labels::Infantry5);
            trackers[AutoAim::Labels::Outpost]   = std::make_shared<AutoAim::Tracker>(AutoAim::Labels::Outpost);
            trackers[AutoAim::Labels::Sentry]    = std::make_shared<AutoAim::Tracker>(AutoAim::Labels::Sentry);
            trackers[AutoAim::Labels::Base]      = std::make_shared<AutoAim::Tracker>(AutoAim::Labels::Base);
        }();

        //* Transform coordinate from 2D to 3D
        //* And update tracker
        auto armor3d = std::make_shared<SyncQueue<Armor3d>>();
        std::thread transform([&] {
            while (true) {
                auto armor = to_transform->pop_data();
                if (!armor.has_value())
                    continue;

                //* transform
                auto tf_armor = pose_transformer->solve_absolute(armor.value());
                armor3d->write_data(tf_armor);

                //* update tracker
                trackers[armor->result]->update(tf_armor);
            }
        });

        //! filter based on policy predefined
        SelectingPolicy policy;
        std::thread filter_and_grant_fire([&] {
            while (true) {
                auto armor = armor3d->pop_data();
                if (!armor.has_value())
                    continue;
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
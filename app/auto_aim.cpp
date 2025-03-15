#include "publisher.hpp"
#include <memory>
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include <boost/exception/diagnostic_information.hpp>
#include <boost/exception/exception.hpp>
#include <exception>
#include <spdlog/spdlog.h>

#include "cam_capture.hpp"
#include "detector.hpp"
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

        //* start thread to read and process raw data from port
        std::thread read_from_port([&]() { port->read_raw_data_from_port(); });
        std::thread process_port_data([&]() { port->process_raw_data_from_buffer(); });

        //! create several sync queues for
        // 1. "image + imu => armors (2D) + label + type(large/small) + imu"
        // 2. "armors(2D) => armors(3D), done in the same sync queue"
        // 3. "armors(3D) => forward to corresponding tracker"
        // 4. "armors(3D) => forward to filtering Policy"

        auto to_annotate = std::make_shared<SyncQueue<RawFrameInfo>>(); // 1.
        // producer (1): raw image from camera
        std::thread annotate_img([&]() {
            while (true) {
            }
        });
        auto to_filter = std::make_shared<SyncQueue<AnnotatedArmorInfo>>(); // 2.

    } catch (std::exception &E) {
        spdlog::critical("{}", E.what());
    } catch (boost::exception &E) {
        spdlog::critical("Error with port: {}", boost::diagnostic_information(E));
    }
}
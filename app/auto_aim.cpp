#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include <boost/exception/diagnostic_information.hpp>
#include <boost/exception/exception.hpp>
#include <exception>
#include <spdlog/spdlog.h>

#include "cam_capture.hpp"
#include "serial_port.hpp"
#include "structs.hpp"
#include "work_queue.hpp"

int main() {
    try {
        // open cam
        auto cam = std::make_shared<HikCamera>("/media/arca/ArcaEXT4/codebases/pred_v2/config/cam.toml");
        // open port
        auto port = std::make_shared<SerialPort>("/media/arca/ArcaEXT4/codebases/pred_v2/config/comm.toml");
    } catch (std::exception &E) {
        spdlog::critical("{}", E.what());
    } catch (boost::exception &E) {
        spdlog::critical("Error with port: {}", boost::diagnostic_information(E));  
    }
}
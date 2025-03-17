#ifndef __FIRING_HPP__
#define __FIRING_HPP__

#include "serial_port.hpp"
#include "structs.hpp"

#include <atomic>
#include <memory>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

class FireController {
  protected:
    volatile std::atomic<AutoAim::Labels> allowed_label_; // can be changed by other threads
    uint8_t updated{0};
    std::shared_ptr<SerialPort> port_;
    std::shared_ptr<spdlog::logger> log_;

  private:
    bool _check_fire();
    bool _check_patrol();
    bool _check_found();
    bool _check_done_fitting();
    VisionPLCSendMsg _pack(const PredictedPosition &pred);

  public:
    FireController();
    void set_port(std::shared_ptr<SerialPort> port);
    void set_allow(const AutoAim::Labels &label);
};

#endif // __FIRING_HPP__
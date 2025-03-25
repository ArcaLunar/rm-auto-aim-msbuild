#ifndef __FIRING_HPP__
#define __FIRING_HPP__

#include "serial_port.hpp"
#include "structs.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <vector>

constexpr static int kFiringTimeout = 5000; // ms

class FireController {
  protected:
    volatile std::atomic<AutoAim::Labels> allowed_label_; // can be changed by other threads
    uint8_t updated{0};
    std::shared_ptr<SerialPort> port_;
    std::shared_ptr<spdlog::logger> log_;

  private:
    bool _check_fire(const PredictedPosition &pred, const std::vector<Armor3d> &context);
    bool _check_patrol(const PredictedPosition &pred, const std::vector<Armor3d> &context);
    bool _check_found(const PredictedPosition &pred, const std::vector<Armor3d> &context);
    bool _check_done_fitting(const PredictedPosition &pred, const std::vector<Armor3d> &context);
    VisionPLCSendMsg _pack(const PredictedPosition &pred, const std::vector<Armor3d> &context);

    std::chrono::high_resolution_clock::time_point last_fire_time;

  public:
    FireController();
    void set_port(std::shared_ptr<SerialPort> port);
    void set_allow(const AutoAim::Labels &label);
    void try_fire(const PredictedPosition &pred, const std::vector<Armor3d> &context);
};

#endif // __FIRING_HPP__
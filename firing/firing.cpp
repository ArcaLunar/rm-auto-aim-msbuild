#include "firing.hpp"
#include "structs.hpp"
#include <cassert>

FireController::FireController() {
    this->log_ = spdlog::stdout_color_mt("FireController");
    this->log_->set_level(spdlog::level::trace);
    this->log_->set_pattern("[%H:%M:%S, +%4oms] [%15s:%3# in %!] [%^%l%$] %v");

    SPDLOG_LOGGER_INFO(this->log_, "FireController initialized");
}

void FireController::set_port(std::shared_ptr<SerialPort> port) { this->port_ = port; }

void FireController::set_allow(const AutoAim::Labels &label) { this->allowed_label_ = label; }

VisionPLCSendMsg FireController::_pack(const PredictedPosition &pred) {
    VisionPLCSendMsg msg;

    msg.pitch           = pred.pitch;
    msg.yaw             = pred.yaw;
    msg.flag_found      = this->_check_found();
    msg.flag_fire       = this->_check_fire();
    msg.flag_patrolling = this->_check_patrol();
    // done_fitting is defined only for windmill and outpost
    msg.flag_done_fitting = this->_check_done_fitting();
    msg.flag_have_updated = this->updated;
    this->updated         = 1 - this->updated;

    return msg;
}

bool FireController::_check_fire() {
    assert(false && "Not implemented");
    return false;
}

bool FireController::_check_patrol() {
    assert(false && "Not implemented");
    return false;
}

bool FireController::_check_found() {
    assert(false && "Not implemented");
    return false;
}

bool FireController::_check_done_fitting() {
    assert(false && "Not implemented");
    return false;
}


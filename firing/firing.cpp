#include "firing.hpp"
#include "config.hpp"
#include "structs.hpp"
#include <algorithm>
#include <cassert>
#include <spdlog/spdlog.h>

FireController::FireController() {
    this->log_ = spdlog::stdout_color_mt("FireController");
    this->log_->set_level(spdlog::level::trace);
    this->log_->set_pattern("[%H:%M:%S, +%4oms] [%15s:%3# in %!] [%^%l%$] %v");

    SPDLOG_LOGGER_INFO(this->log_, "FireController initialized");
}

void FireController::set_port(std::shared_ptr<SerialPort> port) { this->port_ = port; }

void FireController::set_allow(const AutoAim::Labels &label) { this->allowed_label_ = label; }

VisionPLCSendMsg FireController::_pack(const PredictedPosition &pred, const std::vector<Armor3d> &context) {
    VisionPLCSendMsg msg;

    msg.pitch      = pred.pitch;
    msg.yaw        = pred.yaw;
    msg.flag_found = this->_check_found(pred, context);
    if (msg.flag_found)
        msg.flag_fire = this->_check_fire(pred, context);

    if (!msg.flag_fire && !msg.flag_fire)
        msg.flag_patrolling = this->_check_patrol(pred, context);
    // done_fitting is defined only for windmill and outpost
    msg.flag_done_fitting = this->_check_done_fitting(pred, context);
    msg.flag_have_updated = this->updated;
    this->updated         = 1 - this->updated;

    return msg;
}

bool FireController::_check_fire(const PredictedPosition &pred, const std::vector<Armor3d> &context) {
    auto armor = std::find_if(context.begin(), context.end(), [&](const Armor3d &armor) {
        return armor.result == this->allowed_label_;
    });

    if (armor == context.end()) {
        SPDLOG_LOGGER_WARN(this->log_, "No armor found for label");
        return false;
    }

    bool result           = false;
    double relative_pitch = pred.pitch - armor->imu_info.pitch;
    double relative_yaw   = pred.yaw - armor->imu_info.yaw;
    double dist           = pred.distance;
    double armor_height   = armor->armor.type == AutoAim::ArmorType::Large ? LargeArmorHeight : SmallArmorHeight;
    double armor_width    = armor->armor.type == AutoAim::ArmorType::Large ? LargeArmorWidth : SmallArmorWidth;

    double fire_bias_pitch = std::atan2(1.0 * armor_height, dist) * kRadianToDegree;
    double fire_bias_yaw   = std::atan2(1.0 * armor_width, dist) * kRadianToDegree;

    bool fire_pitch = std::abs(relative_pitch) < std::abs(fire_bias_pitch);
    bool fire_yaw   = std::abs(relative_yaw) < std::abs(fire_bias_yaw);

    result = fire_pitch && fire_yaw;
    if (result)
        last_fire_time = std::chrono::high_resolution_clock::now();
    return result;
}

bool FireController::_check_patrol(const PredictedPosition &pred, const std::vector<Armor3d> &context) {
    auto cur_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<double, std::milli>(cur_time - last_fire_time).count();
}

bool FireController::_check_found(const PredictedPosition &pred, const std::vector<Armor3d> &context) {
    auto armor = std::find_if(context.begin(), context.end(), [&](const Armor3d &armor) {
        return armor.result == this->allowed_label_;
    });

    if (armor == context.end()) {
        SPDLOG_LOGGER_WARN(this->log_, "No armor found for label");
        return false;
    }

    return true;
}

bool FireController::_check_done_fitting(const PredictedPosition &pred, const std::vector<Armor3d> &context) {
    auto armor = std::find_if(context.begin(), context.end(), [&](const Armor3d &armor) {
        return armor.result == this->allowed_label_;
    });

    if (armor->result != AutoAim::Labels::Outpost)
        return false;

    return true;
}

void FireController::try_fire(const PredictedPosition &pred, const std::vector<Armor3d> &context) {
    auto pack = this->_pack(pred, context);
    this->port_->send_data(pack);
}
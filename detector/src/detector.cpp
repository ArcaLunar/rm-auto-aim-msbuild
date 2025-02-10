#include "detector.hpp"


//! Detector
AutoAim::Detector::Detector(std::string path) : light_bar_config_(path), armor_config_(path) {
    spdlog::info("Detector initialized with config file: \"{}\"", path);
}
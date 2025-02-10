#ifndef __DETECTOR_HPP__
#define __DETECTOR_HPP__

#include "config.hpp"
#include "id_classify.hpp"
#include "structs.hpp"

#include <memory>

#include <spdlog/spdlog.h>
#include <toml++/toml.hpp>

namespace AutoAim {

struct LightBarConfig {
    double min_ratio, max_ratio;
    double max_angle;

    LightBarConfig(std::string path = "../config/detection_tr.toml");
};

struct ArmorConfig {};

} // namespace AutoAim

#endif // __DETECTOR_HPP__
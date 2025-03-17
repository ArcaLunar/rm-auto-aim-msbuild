#ifndef __TRACKER_FILTERS_KF_HPP__
#define __TRACKER_FILTERS_KF_HPP__

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <toml++/toml.h>

/**
 * @brief state=`[x, y, z, vx, vy, vz, rz, vrz, Pitch, vPitch]`
 * observe=`[x, y, z, vx, vy, vz, rz, Pitch]`
 * @tparam state
 * @tparam observe
 */
namespace KalmanFilter {

class KF : public cv::KalmanFilter {};

} // namespace KalmanFilter
#endif // __TRACKER_FILTERS_KF_HPP__
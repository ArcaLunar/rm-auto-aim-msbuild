/**
 * @file structs.hpp
 * @author arca
 * @brief Defines commonly used data structures.
 * @version 0.1
 * @date 2025-02-06
 */
#pragma once

#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/core/matx.hpp>

// ========================================================
// Camera Related Data Structures
// ========================================================

/**
 * @brief Captured by camera. Is then passed to detector for further processing.
 * @details Records the frame and the timestamp at which it was captured.
 */
struct RawFrameInfo {
    cv::Mat frame;
    std::chrono::time_point<std::chrono::system_clock> timestamp;
};

// ========================================================
// IMU Related Data Structures
// ========================================================
/**
 * @brief Important information from the IMU sensor.
 *
 */
struct IMUInfo {
    double roll{}, pitch{}, yaw{};
};

// ========================================================
// Detector Related Data Structures
// ========================================================

struct Armor2dInfo {};

struct Armor3dInfo {
};

struct ProcessedFrameInfo {
    std::string armor_id;
};

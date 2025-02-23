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
// Serial Port Data Structures
// ========================================================
constexpr int kProtocolSendHead = 0xA3;
constexpr int kProtocolRecvHead = 0x3A;
constexpr int kProtocolTail     = 0xAA;

#pragma pack(push, 1)
struct VisionPLCSendMsg {
    uint8_t frame_head{kProtocolSendHead};
    float pitch{};
    float yaw{};
    uint8_t flag_found{};
    uint8_t flag_fire{};
    uint8_t flag_done_fitting{};
    uint8_t flag_patrolling{};
    uint8_t flag_have_updated{};
    uint8_t frame_tail{kProtocolTail};
};
#pragma pack(pop)

#pragma pack(push, 1)
struct VisionPLCRecvMsg {
    uint8_t frame_head{kProtocolRecvHead};
    float imu_roll{};
    float imu_pitch{};
    float imu_yaw{};
    uint8_t my_color{}; // 1 for red, 2 for blue
    uint8_t aim_mode;
    struct {
        uint8_t hero : 1;
        uint8_t engineer : 1;
        uint8_t infantry_3 : 1;
        uint8_t infantry_4 : 1;
        uint8_t infantry_5 : 1;
        uint8_t sentry : 1;
        uint8_t outpost : 1;
        uint8_t base : 1;
    } shoot_decision{};
    uint8_t frame_tail{kProtocolTail};
};
#pragma pack(pop)

struct SerialPortConfiguration {
    std::string port_name;
    int baud_rate{460800};
    int data_bits{8};
    int stop_bits{1};
    int parity{0};
    int sync{1};
    int send_interval{0};
};

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

namespace AutoAim {

struct LightBarConfig {
    double min_ratio, max_ratio;
    double max_angle;

    explicit LightBarConfig(std::string path = "../config/detection_tr.toml");
};

struct ArmorConfig {
    int binary_threshold;
    double min_light_ratio;
    double min_small_center_distance, max_small_center_distance;
    double min_large_center_distance, max_large_center_distance;
    double max_angle;

    explicit ArmorConfig(std::string path = "../config/detection_tr.toml");
};

/**
 * @brief 灯条 class
 * @remark chenjunnn/rm_auto_aim
 */
struct LightBar : public cv::RotatedRect {
    double length, width;            // 灯条的长度和宽度
    double tilt_angle;               // 倾斜角度
    std::vector<cv::Point2f> points; // 矩形的四个顶点
    cv::Point2f top, bottom;
    std::string color;

    // 从配置文件中读取灯条的配置
    explicit LightBar() = default;
    explicit LightBar(const cv::RotatedRect &rect);

    /// 判断灯条是否合法
    bool is_valid(const LightBarConfig &config) const;
};

/**
 * @brief 装甲板 class
 */
struct Armor {
    //* 灯条
    LightBar left, right; // 左右灯条
    cv::Point2f center;   // 装甲板中心

    //* 装甲板上的数字/图案
    cv::Mat number_img;  // 用于识别数字的装甲板图像
    double confidence;   // 识别的置信度
    std::string summary; // 识别的结果
    std::string result;  // 识别的数字

    Armor() = default;
    explicit Armor(const LightBar &l1, const LightBar &l2);

    // 判断装甲板是否合法
    static bool is_valid(const ArmorConfig &config, const LightBar &left, const LightBar &right);
};

} // namespace AutoAim

/**
 * @brief 经过识别、坐标变换后的装甲板信息
 *
 */
struct AnnotatedArmorInfo {
    std::vector<cv::Point3f> corners;
    std::string result;
    IMUInfo imu_info;
    std::chrono::time_point<std::chrono::system_clock> timestamp;
};

// ========================================================
// Tracker Related Data Structures
// ========================================================

namespace Tracker {

enum class TrackingStatus {
    FITTING,
    TRACKING,
    TEMPORARY_LOST,
    LOST,
};
enum class ArmorCount {
    OUTPOST = 3,
    NORMAL  = 4,
};

struct TrackingConfig {
    int lost_timeout;
};

} // namespace Tracker
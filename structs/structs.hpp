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
 * @brief IMU 传感器数据。用于记录当前的姿态信息。辅助 base 系 -> IMU 系的坐标转换
 *
 */
struct IMUInfo {
    double roll{}, pitch{}, yaw{};
    std::chrono::time_point<std::chrono::system_clock> timestamp;

    void load_from_recvmsg(const VisionPLCRecvMsg &msg);
    cv::Mat rotation() const;
};

// ========================================================
// Detector Related Data Structures
// ========================================================

namespace AutoAim {

enum class ArmorType {
    None,
    Small,
    Large,
};

enum class Labels {
    None,
    Hero,
    Engineer,
    Infantry3,
    Infantry4,
    Infantry5,
    Sentry,
    Outpost,
    Base,
};

/**
 * @brief 灯条过滤参数
 *
 */
struct LightBarConfig {
    double min_area, max_area;
    double min_solidity;
    double min_aspect_ratio, max_aspect_ratio;
    double max_angle;
    int brightness_threshold;
    int color_threshold;

    explicit LightBarConfig(std::string path = "../config/detection_tr.toml");
};

/**
 * @brief 装甲板过滤参数
 *
 */
struct ArmorConfig {
    double max_angle_diff;
    double max_height_diff_ratio;
    double max_Y_diff_ratio;
    double min_X_diff_ratio;
    double big_armor_ratio, small_armor_ratio;
    double min_aspect_ratio, max_aspect_ratio;
    double min_area;
    double max_roll_angle;
    double max_light_bar_armor_area_ratio;
    double area_normalized_base;
    double sight_offset_normalized_base;
    double lightbar_area_ratio;

    explicit ArmorConfig(std::string path = "../config/detection_tr.toml");
};

/**
 * @brief 灯条 class
 * @remark chenjunnn/rm_auto_aim
 */
struct LightBar {
    std::vector<cv::Point> contour; // 灯条轮廓
    cv::RotatedRect ellipse;
    std::vector<cv::Point2f> vertices;
    double long_axis, short_axis;      // 长短轴
    double angle;                      // 倾斜角度, [-90, 90), deg
    double ellipse_area, contour_area; // 椭圆面积、轮廓面积
    double solidity;                   // 轮廓面积/椭圆面积

    explicit LightBar();
    explicit LightBar(const std::vector<cv::Point> &contour);
    ~LightBar();

    cv::Point2f center() const;
    /// 判断灯条是否合法
    bool is_valid(const LightBarConfig &config) const;
};

/**
 * @brief 装甲板 class
 */
struct Armor {
    //* 灯条
    LightBar left, right;              // 左右灯条
    std::vector<cv::Point2f> vertices; // 装甲板四个顶点
    cv::Point2f center;                // 装甲板中心
    ArmorType type;                    // 装甲板类型
    cv::RotatedRect min_rect;          // 装甲板最小外接矩形
    double angle;                      // 装甲板倾斜角度, [-90, 90), deg

    Armor() = default;
    explicit Armor(const LightBar &l1, const LightBar &l2);

    // 判断装甲板是否合法
    bool is_valid(const ArmorConfig &config);
};

} // namespace AutoAim

/**
 * @brief 经过识别、坐标变换后的装甲板信息
 *
 */
struct AnnotatedArmorInfo {
    AutoAim::Armor armor;
    AutoAim::Labels result;
    IMUInfo imu_info;
    std::chrono::time_point<std::chrono::system_clock> timestamp;
};

// ========================================================
// Tracker Related Data Structures
// ========================================================

namespace Tracker {

/**
 * @brief Tracker 状态。
 * `FITTING` = 正在拟合车辆运动
 * `TRACKING` = 正在追踪车辆
 * `TEMPORARY_LOST` = 短暂丢失
 * `LOST` = 完全丢失（丢失超过一定时间）
 */
enum class TrackingStatus {
    FITTING,
    TRACKING,
    TEMPORARY_LOST,
    LOST,
};

/**
 * @brief 车上的甲板数量。用于根据甲板种类，计算（待击打）甲板的位置
 */
enum class ArmorCount {
    OUTPOST = 3,
    NORMAL  = 4,
};

/**
 * @brief 跟踪器参数
 *
 */
struct TrackingConfig {
    int lost_timeout;
};

} // namespace Tracker
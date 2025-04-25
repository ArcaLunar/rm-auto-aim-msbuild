#ifndef __STRUCTS_HPP__
#define __STRUCTS_HPP__

//! ========================================================
//! Useful Macro Definitions
//! ========================================================

#include "toml++/toml.h"
#include <chrono>
#include <cstdint>
#include <opencv2/core.hpp>

#define mvcheck(func, params...)                                                                                       \
    {                                                                                                                  \
        int nret_##func = func(params);                                                                                \
        if (nret_##func != MV_OK) {                                                                                    \
            spdlog::error("Error in {}({})", #func, #params);                                                          \
            throw std::runtime_error("Error in " #func);                                                               \
        } else {                                                                                                       \
            spdlog::debug("Success in {}({})", #func, #params);                                                        \
        }                                                                                                              \
    }

#define conditioned_log(func, waiter, params...)                                                                       \
    [&] {                                                                                                              \
        if (waiter) {                                                                                                  \
            spdlog::info("calling {}({})", #func, #params);                                                            \
        }                                                                                                              \
        func(params);                                                                                                  \
        if (waiter) {                                                                                                  \
            spdlog::info("{}({}) done", #func, #params);                                                               \
        }                                                                                                              \
    }();

#define spd_timer(func, params...)                                                                                     \
    {                                                                                                                  \
        auto start = std::chrono::steady_clock::now();                                                                 \
        func(params);                                                                                                  \
        auto end      = std::chrono::steady_clock::now();                                                              \
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();                    \
        spdlog::info("{}({}) took {} ms", #func, #params, duration);                                                   \
    }

#define spd_result_timer(func, params...)                                                                              \
    [&] {                                                                                                              \
        auto start    = std::chrono::steady_clock::now();                                                              \
        auto result   = func(params);                                                                                  \
        auto end      = std::chrono::steady_clock::now();                                                              \
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();                    \
        spdlog::info("{}({}) took {} ms", #func, #params, duration);                                                   \
        return result;                                                                                                 \
    }();

//! 赛场参数
constexpr int ModelInputWidth     = 64;  // 摄像头输入图像宽度
constexpr int ModelInputHeight    = 64;  // 摄像头输入图像高度
constexpr double SmallArmorWidth  = 135; // 单位 mm
constexpr double SmallArmorHeight = 56;  // 单位 mm
constexpr double LargeArmorWidth  = 230; // 单位 mm
constexpr double LargeArmorHeight = 56;  // 单位 mm
constexpr double kDegreeToRadian = 0.017453292519943295769236907684886;
constexpr double kRadianToDegree = 57.295779513082320876798154814105;

//^ ========================================================
//^ Macro Definition Ends
//^ ========================================================

//! ========================================================
//! Serial Port Data Classes
//! ========================================================
using u8 = uint8_t;
#pragma pack(push, 1)
struct VisionPLCSendMsg {
    u8 start;
    float pitch;
    float yaw;
    u8 flag_found;
    u8 flag_fire;
    u8 flag_donefitting;
    u8 flag_patrol;
    u8 flag_updated;
    u8 end;
};
struct VisionPLCRecvMsg {
    u8 start;
    float roll;
    float pitch;
    float yaw;
    u8 ally_color;
    u8 vision_mode;
    struct armor_select_t {
        u8 hero : 1;
        u8 engineer : 1;
        u8 infantry3 : 1;
        u8 infantry4 : 1;
        u8 infantry5 : 1;
        u8 sentry : 1;
        u8 outpost : 1;
        u8 base : 1;
    } shoot_decision;
    u8 end;
};
struct SentryVisionRecvMsg {
    u8 start;
    float roll;
    float pitch;
    float yaw;
    u8 ally_color;
    u8 vision_mode;
    struct armor_select_t {
        u8 hero : 1;
        u8 engineer : 1;
        u8 infantry3 : 1;
        u8 infantry4 : 1;
        u8 infantry5 : 1;
        u8 sentry : 1;
        u8 outpost : 1;
        u8 base : 1;
    } shoot_decision;
    struct robot_remain_hp_t {
        uint8_t m_Hero : 1;
        uint8_t m_Engineer : 1;
        uint8_t m_Infantry3 : 1;
        uint8_t m_Infantry4 : 1;
        uint8_t m_Sentry : 1;
    } remaining_hp;
    u8 end;
};
#pragma pack(pop)

struct PortConfig {
    int baudrate;
    int stopbit;
    int databit;
    int parity;

    u8 startbyte, endbyte;
};

//! ========================================================
//!!!!! IMPORTANT !!!!!
//! ========================================================
using RecvMsgType = SentryVisionRecvMsg;

struct StampedRecvMsg {
    std::chrono::steady_clock::time_point timestamp;
    RecvMsgType msg;
};

struct RMColor {
    static constexpr int Blue    = 0;
    static constexpr int Unknown = 1;
    static constexpr int Red     = 2;
    int ally, enemy;

    RMColor() {
        auto T   = toml::parse_file("../config/game.toml");
        auto str = T["ally_color"].value_or("blue");

        if (str == "blue")
            this->ally = RMColor::Blue, this->enemy = RMColor::Red;
        else if (str == "red")
            this->ally = RMColor::Red, this->enemy = RMColor::Blue;
        else
            this->ally = this->enemy = RMColor::Unknown;
    }
};

//! ========================================================
//! Camera Data Classes
//! ========================================================

struct RawImageFrame {
    cv::Mat image;
    std::chrono::steady_clock::time_point timestamp;
};

//^ ========================================================
//^ Camera Data Classes Ends
//^ ========================================================

//! ========================================================
//! IMU Data Classes
//! ========================================================

struct IMUInfo {
    double roll{}, pitch{}, yaw{};
    std::chrono::steady_clock::time_point timestamp;

    void load_from_recvmsg(const StampedRecvMsg &msg);
    cv::Mat rotation() const;
};

//! ========================================================
//! Detector Data Classes
//! ========================================================

enum class ArmorType {
    None,
    Small,
    Large,
};

struct LightBarConfig {
    double min_area, max_area;
    double min_solidity;
    double min_aspect_ratio, max_aspect_ratio;
    double max_angle;
    int brightness_threshold;
    int color_threshold;

    explicit LightBarConfig(std::string path);
};

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

    explicit ArmorConfig(std::string path);
};

struct LightBar {
    std::vector<cv::Point> contour; // 灯条轮廓
    cv::RotatedRect ellipse;
    std::vector<cv::Point2f> vertices;
    double long_axis, short_axis;      // 长短轴
    double angle;                      // 倾斜角度, [-90, 90), deg
    double ellipse_area, contour_area; // 椭圆面积、轮廓面积
    double solidity;                   // 轮廓面积/椭圆面积

    LightBar();
    LightBar(const std::vector<cv::Point> &contour);
    ~LightBar();

    cv::Point2f center() const;
    /// 判断灯条是否合法
    bool is_valid(const LightBarConfig &config) const;
};

struct RawArmor {
    //* 灯条
    LightBar left, right;              // 左右灯条
    std::vector<cv::Point2f> vertices; // 装甲板四个顶点
    cv::Point2f center;                // 装甲板中心
    ArmorType type;                    // 装甲板类型
    cv::RotatedRect min_rect;          // 装甲板最小外接矩形
    double angle;                      // 装甲板倾斜角度, [-90, 90), deg

    RawArmor() = default;
    explicit RawArmor(const LightBar &l1, const LightBar &l2);

    // 判断装甲板是否合法
    bool is_valid(const ArmorConfig &config);
};

struct AnnotatedArmorInfo {
    RawArmor armor;
    int result; // 代表兵种
    IMUInfo imu_info;
    std::chrono::steady_clock::time_point timestamp;
};

//! ========================================================
//! Coordinate Transform Related Data Structures
//! ========================================================

//! 单位均为 meter
struct pose_under_camera_coord {
    double roll{}, pitch{}, yaw{};
    cv::Mat rvec, tvec;
    double direction;  // 装甲板朝向
    double distance;   // 装甲板中心到相机的距离
    cv::Mat center_3d; // 装甲板中心在相机坐标系下的坐标

    void load_from_imu(const IMUInfo &imu, const cv::Mat &T_camera_to_barrel);
};
//! 单位均为 meter
//! Absolute: relative to barrel
struct poes_under_barrel_coord {
    double roll{}, pitch{}, yaw{};
    double direction; // 装甲板朝向
    double distance;
    cv::Mat center_3d;
};
//! 单位均为 meter
struct Armor3d : AnnotatedArmorInfo {
    double bullet_flying_time;
    double pitch_relative_to_barrel, yaw_relative_to_barrel;

    cv::Mat T_armor_to_barrel; // meters
    cv::Mat R_armor_to_barrel; // radians

    //* transform information
    pose_under_camera_coord p_a2c;
    poes_under_barrel_coord p_barrel;
};

#endif
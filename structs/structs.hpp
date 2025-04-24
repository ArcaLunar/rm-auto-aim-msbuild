#ifndef __STRUCTS_HPP__
#define __STRUCTS_HPP__

//! ========================================================
//! Useful Macro Definitions
//! ========================================================

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

//^ =========================================================
//^ Macro Definition Ends
//^ =========================================================

//! ========================================================
//! Camera Data Classes
//! ========================================================

#include <chrono>
#include <opencv2/core.hpp>

struct RawImageFrame {
    cv::Mat image;
    std::chrono::steady_clock::time_point timestamp;
};

//^ ========================================================
//^ Camera Data Classes Ends
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
#pragma pack(pop)

#pragma pack(push, 1)
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
#pragma pack(pop)

#pragma pack(push, 1)
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

#endif
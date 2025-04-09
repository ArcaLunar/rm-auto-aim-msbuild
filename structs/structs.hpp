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

#define spd_wrapper(func, waiter, params...)                                                                           \
    {                                                                                                                  \
        if (waiter) {                                                                                                  \
            spdlog::info("calling {}({})", #func, #params);                                                            \
        }                                                                                                              \
        func(params);                                                                                                  \
        if (waiter) {                                                                                                  \
            spdlog::info("{}({}) done", #func, #params);                                                               \
        }                                                                                                              \
    }

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
    std::chrono::system_clock::time_point timestamp;
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
    } shoot_decision;
    u8 end;
};

#endif
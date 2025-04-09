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


#include <chrono>
#include <opencv2/core.hpp>

struct RawImageFrame {
    cv::Mat image;
    std::chrono::system_clock::time_point timestamp;
};

#endif
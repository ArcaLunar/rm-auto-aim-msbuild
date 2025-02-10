#pragma once

#include <string>

// 角度制、弧度制转换
constexpr double kDegreeToRadian = 0.017453292519943295769236907684886;
constexpr double kRadianToDegree = 57.295779513082320876798154814105;

// 调试选项
constexpr bool DEBUG_MODE                 = "debug"; // 是否开启调试模式
constexpr bool SHOW_ANNOTATED_IMAGE       = true;    // 识别完装甲板后是否显示标注装甲板的图像
constexpr bool SUPPRESS_VALIDATION_SPDLOG = true;    // 抑制 armor.cpp 里的 spdlog 输出（减少日志数量）

//! 赛场参数
constexpr std::string COLOR_TO_DETECT = "red"; // 要检测的颜色
#pragma once

// 角度制、弧度制转换
constexpr double kDegreeToRadian = 0.017453292519943295769236907684886;
constexpr double kRadianToDegree = 57.295779513082320876798154814105;

#define EnableAllDebug false

//! 调试选项
constexpr bool SuppressValidationDebug = true; // 抑制 armor.cpp 里的 spdlog 输出（减少日志数量）

constexpr bool InitializationDebug
    = true && EnableAllDebug;                         // set to `false` to disable debug messages for initialization
constexpr bool CameraDebug = false && EnableAllDebug; // set to `false` to disable debug messages from camera capture
constexpr bool SerialPortDebug = false && EnableAllDebug; // set to `false` to disable debug messages from serial port
constexpr bool ProducerConsumerModelDebug
    = false && EnableAllDebug; // set to `false` to disable messages from work_queue.hpp
constexpr bool DetectorDebug   = false && EnableAllDebug;
constexpr bool ClassifierDebug = false && EnableAllDebug;
constexpr bool PublisherDebug  = true && EnableAllDebug;
constexpr bool DisplayAnnotatedImageDebug = true && EnableAllDebug; // 识别完装甲板后是否显示标注装甲板的图像
constexpr bool PublisherDiaplayImageDebug = false && EnableAllDebug;

//! 赛场参数
constexpr int ModelInputWidth     = 64;  // 摄像头输入图像宽度
constexpr int ModelInputHeight    = 64;  // 摄像头输入图像高度
constexpr double SmallArmorWidth  = 135; // 单位 mm
constexpr double SmallArmorHeight = 56;  // 单位 mm
constexpr double LargeArmorWidth  = 230; // 单位 mm
constexpr double LargeArmorHeight = 56;  // 单位 mm

// ========================================================
// Global Settings
// ========================================================
enum class RMColor {
    Blue,
    Unknown,
    Red,
};
static RMColor EnemyColor = RMColor::Blue;
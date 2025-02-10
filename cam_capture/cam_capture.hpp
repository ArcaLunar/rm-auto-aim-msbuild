#ifndef __CAM_CAPTURE_HPP__
#define __CAM_CAPTURE_HPP__

#include "CameraParams.h"
#include <opencv2/core/core.hpp>

enum HikCameraProperties {
    CAP_PROP_FRAMERATE_ENABLE,  // 帧数可调
    CAP_PROP_FRAMERATE,         // 帧数
    CAP_PROP_BURSTFRAMECOUNT,   // 外部一次触发帧数
    CAP_PROP_HEIGHT,            // 图像高度
    CAP_PROP_WIDTH,             // 图像宽度
    CAP_PROP_EXPOSURE_TIME,     // 曝光时间
    CAP_PROP_GAMMA_ENABLE,      // 伽马因子可调
    CAP_PROP_GAMMA,             // 伽马因子
    CAP_PROP_GAINAUTO,          // 亮度
    CAP_PROP_SATURATION_ENABLE, // 饱和度可调
    CAP_PROP_SATURATION,        // 饱和度
    CAP_PROP_OFFSETX,           // X偏置
    CAP_PROP_OFFSETY,           // Y偏置
    CAP_PROP_TRIGGER_MODE,      // 外部触发
    CAP_PROP_TRIGGER_SOURCE,    // 触发源
    CAP_PROP_LINE_SELECTOR      // 触发线

};

struct CameraConfig {
    bool AcquisitionFrameRateEnable;
    int AcquisitionFrameRate;

    int Width, Height;
    double exposure_time;

    bool adjustable_gamma;
    int gamma;

    int gain_auto;

    bool adjustable_saturation;
    int saturation;

    int offset_x, offset_y;

    int TriggerMode;
    int TriggerSource;
    int line_selector;
    int PayloadSize;
};

class HikCamera {
  private:
    /// \brief 枚举所有已连接的设备
    void EnumDevices();

    /// \brief 输出所有设备的信息
    void DebugDevices();

    /// \brief 打印设备信息
    void PrintDeviceInfo(MV_CC_DEVICE_INFO *);

    /// \brief 创建用于获取图像的句柄
    void CreateHandle();

    /// \brief 打开相机
    void OpenCamera();

    /// \brief 设置一系列参数配置
    void LoadConfig();
    void Setup();

    /// \brief 初始化图像捕获
    void InitRetrieveImage();

    /// \brief 转换为 OpenCV 矩阵

  protected:
    /// \brief 设备列表
    MV_CC_DEVICE_INFO_LIST devicelist_;
    unsigned int camIndex;
    void *handle_;
    CameraConfig config_;
    MV_FRAME_OUT buffer_;

  public:
    HikCamera();
    ~HikCamera();
    void *GetHandle() { return handle_; }
    static cv::Mat ConvertRawToMat(MV_FRAME_OUT_INFO_EX *, MV_FRAME_OUT *);
    cv::Mat GetFrame();
};

#endif // __CAM_CAPTURE_HPP__
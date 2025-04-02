#ifndef __CAM_CAPTURE_HPP__
#define __CAM_CAPTURE_HPP__

#include <memory>
#include <spdlog/logger.h>
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include "CameraParams.h"
#include "structs.hpp"

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
    bool acquisition_frame_rate_enable{false};
    int acquisition_frame_rate;

    int width, height;

    int auto_exposure{0};
    double exposure_time{1000};

    int adc_bit_depth{2};

    bool adjustable_gamma{true};
    double gamma{0.5};

    int gain_auto{0};

    bool adjustable_saturation;
    int saturation;

    int offset_x, offset_y;

    int trigger_mode{MV_CAM_TRIGGER_MODE::MV_TRIGGER_MODE_OFF};
    int trigger_source;
    int line_selector;
    int payload_size;

    std::string pixel_format{"BayerRG8"};
};

class HikCamera {
  private:
    /// \brief 枚举所有已连接的设备
    void enum_devices();

    /// \brief 输出所有设备的信息
    void debug_devices();

    /// \brief 打印设备信息
    void print_device_info(MV_CC_DEVICE_INFO *);

    /// \brief 创建用于获取图像的句柄
    void create_handle();

    /// \brief 打开相机
    void open_camera();

    /// \brief 设置一系列参数配置
    CameraConfig load_config(const std::string &config_path);
    void setup(const std::string &config_path);

    /// \brief 初始化图像捕获
    void initialize_image_retrieval();

    /// \brief 获取一帧图像 (`cv::Mat`)
    cv::Mat __get_frame();

  protected:
    /// \brief 设备列表
    MV_CC_DEVICE_INFO_LIST devicelist_;
    unsigned int camIndex;
    void *handle_;
    CameraConfig config_;
    MV_FRAME_OUT buffer_;
    std::shared_ptr<spdlog::logger> log_;

  public:
    HikCamera(const std::string &config_file);
    ~HikCamera();
    /// \brief 获取相机的处理 agent
    void *get_handle() { return handle_; }
    /// \brief （辅助函数）将捕获的图像转换为 OpenCV 矩阵
    cv::Mat convert_raw_to_mat(MV_FRAME_OUT_INFO_EX *, MV_FRAME_OUT *);

    /**
     * @brief 获取一帧 BGR 图像，打上时间戳
     */
    RawFrameInfo get_frame();
};

#endif // __CAM_CAPTURE_HPP__
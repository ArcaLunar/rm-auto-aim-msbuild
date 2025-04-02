#include "PixelType.h"
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include "cam_capture.hpp"
#include "config.hpp"
#include "structs.hpp"

#include "MvCameraControl.h"
#include "toml++/impl/parse_error.hpp"
#include "toml++/impl/parser.hpp"

#include <opencv2/core/persistence.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "spdlog/sinks/stdout_color_sinks.h"
#include <spdlog/spdlog.h>
#include <toml++/toml.hpp>

HikCamera::HikCamera(const std::string &config_path) {
    this->log_ = spdlog::stdout_color_mt("HikCamera");
    this->log_->set_level(spdlog::level::trace);
    this->log_->set_pattern("[%H:%M:%S, +%4oms] [%15s:%3# in %!] [%^%l%$] %v");

    // 初始化 SDK
    MV_CC_Initialize();
    // 枚举设备
    this->enum_devices();
    // 输出设备信息
    this->debug_devices();
    // 创建句柄
    this->create_handle();
    // 打开相机
    this->open_camera();
    // 设置相机参数
    this->setup(config_path);
    // 初始化图像捕获，准备捕获图像
    this->initialize_image_retrieval();
}

HikCamera::~HikCamera() {
    SPDLOG_LOGGER_INFO(this->log_, "exiting......");
    // 停止捕获图像
    SPDLOG_LOGGER_INFO(this->log_, "stop retrieving image");
    int result = MV_CC_StopGrabbing(this->handle_);
    if (result != MV_OK)
        SPDLOG_LOGGER_CRITICAL(this->log_, "error stopping");
    SPDLOG_LOGGER_INFO(this->log_, "stop succeed");

    // 关闭相机
    SPDLOG_LOGGER_INFO(this->log_, "closing camera");
    result = MV_CC_CloseDevice(this->handle_);
    if (result != MV_OK)
        SPDLOG_LOGGER_CRITICAL(this->log_, "error closing");
    SPDLOG_LOGGER_INFO(this->log_, "close succeed");

    // 销毁句柄
    SPDLOG_LOGGER_INFO(this->log_, "destroying handle");
    result = MV_CC_DestroyHandle(this->handle_);
    if (result != MV_OK)
        SPDLOG_LOGGER_CRITICAL(this->log_, "error destroying");
    SPDLOG_LOGGER_INFO(this->log_, "destroy succeed");

    // 释放 SDK
    MV_CC_Finalize();
    SPDLOG_LOGGER_INFO(this->log_, "exit successfully");
}

void HikCamera::enum_devices() {
    // 枚举相机设备
    SPDLOG_LOGGER_INFO(this->log_, "retriving available cam list ...");
    std::memset(&this->devicelist_, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    int nResult = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &this->devicelist_);
    if (nResult != MV_OK) {
        SPDLOG_LOGGER_CRITICAL(this->log_, "retrieving fails.");
        exit(-1);
    }
    SPDLOG_LOGGER_INFO(this->log_, "retrieving succeed");
}

void HikCamera::debug_devices() {
    // 相机的设备 ID
    SPDLOG_LOGGER_INFO(this->log_, "finding Hik Camera");
    if (devicelist_.nDeviceNum == 0) {
        SPDLOG_LOGGER_CRITICAL(this->log_, "no device found.");
        exit(-1);
    }
    for (int i = 0, n = devicelist_.nDeviceNum; i < n; i++) {
        MV_CC_DEVICE_INFO *pDeviceInfo = devicelist_.pDeviceInfo[i];
        SPDLOG_LOGGER_INFO(this->log_, "checking device {}", i);
        if (pDeviceInfo == nullptr)
            break;
        this->print_device_info(pDeviceInfo);
        camIndex = i;
    }
    SPDLOG_LOGGER_INFO(this->log_, "finding succeed");
}

void HikCamera::print_device_info(MV_CC_DEVICE_INFO *pDeviceInfo) {
    if (pDeviceInfo == nullptr) { // 设备不存在
        SPDLOG_LOGGER_CRITICAL(this->log_, "device info pointer is null.");
        exit(-1);
    }

    if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) { // USB 相机
        SPDLOG_LOGGER_INFO(this->log_, "device type: USB");
    } else if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
        SPDLOG_LOGGER_INFO(this->log_, "device type: GIGE");
    } else {
        SPDLOG_LOGGER_ERROR(this->log_, "unknown device type, cannot continue");
        exit(-1);
    }
}

void HikCamera::create_handle() {
    SPDLOG_LOGGER_INFO(this->log_, "creating handle_ for camera {}", this->camIndex);
    int result = MV_CC_CreateHandle(&this->handle_, devicelist_.pDeviceInfo[camIndex]);
    if (result != MV_OK) {
        SPDLOG_LOGGER_CRITICAL(this->log_, "critical creating handle_");
        exit(-1);
    }
    SPDLOG_LOGGER_INFO(this->log_, "create success");
}

void HikCamera::open_camera() {
    SPDLOG_LOGGER_INFO(this->log_, "opening camera");
    int result = MV_CC_OpenDevice(this->handle_);
    if (result != MV_OK) {
        SPDLOG_LOGGER_CRITICAL(this->log_, "open camera fails");
        exit(-1);
    }
    SPDLOG_LOGGER_INFO(this->log_, "open succeed");
}

cv::Mat HikCamera::convert_raw_to_mat(MV_FRAME_OUT_INFO_EX *pstImageInfo, MV_FRAME_OUT *pstImage) {
    cv::Mat result;
    auto mark           = pstImageInfo->enPixelType;
    auto channel_type   = CV_8UC3;
    auto transform_type = cv::COLOR_BayerGB2RGB;
    cv::Mat src(pstImageInfo->nHeight, pstImageInfo->nWidth, channel_type, pstImage->pBufAddr);

    switch (mark) {
        case PixelType_Gvsp_BayerRG8: {
            cv::cvtColor(src, result, cv::COLOR_BayerRG2BGR);
            break;
        }
        case PixelType_Gvsp_BayerGB8: {
            cv::cvtColor(src, result, cv::COLOR_BayerGB2BGR);
            break;
        }
        case PixelType_Gvsp_BGR8_Packed: {
            result = src;
            break;
        }
    }

    return result;
}

CameraConfig HikCamera::load_config(const std::string &config_path) {
    // ! 打开配置文件
    SPDLOG_LOGGER_INFO(this->log_, "reading from .config");

    toml::table T;
    struct CameraConfig config;
    try {
        T = toml::parse_file(config_path);

        config.pixel_format                  = T["pixel_format"].value_or("BayerRG8");
        config.adc_bit_depth                 = T["adc_bit_depth"].value_or(2);
        config.trigger_mode                  = T["trigger_mode"].value_or(0);
        config.auto_exposure                 = T["auto_exposure"].value_or(0);
        config.exposure_time                 = T["exposure_time"].value_or(1000);
        config.gain_auto                     = T["gain_auto"].value_or(0);
        config.adjustable_gamma              = T["adjustable_gamma"].value_or(true);
        config.gamma                         = T["gamma"].value_or(0.5);
        config.acquisition_frame_rate_enable = T["acquisition_frame_rate_enable"].value_or(false);
    } catch (const toml::parse_error &e) {
        SPDLOG_LOGGER_ERROR(this->log_, "error parsing config file: {}, using fallback", e.what());
    }

    return config;
}

void HikCamera::setup(const std::string &config_path) {
    auto config = load_config(config_path);

#define SET_PARAM(func, value, item)                                                                                   \
    if (MV_CC_Set##func(this->handle_, item, value) != MV_OK)                                                          \
        SPDLOG_LOGGER_CRITICAL(this->log_, "setting {} to {} failed", item, value);                                    \
    else                                                                                                               \
        SPDLOG_LOGGER_INFO(this->log_, "setting {} to {} succeeded.", item, value);

    //* Pixel Format
    if (config.pixel_format == "BayerRG8") {
        if (MV_OK != MV_CC_SetEnumValue(this->handle_, "PixelFormat", PixelType_Gvsp_BayerRG8)) {
            SPDLOG_LOGGER_CRITICAL(this->log_, "setting pixel format BayerRG8 failed");
            exit(-1);
        } else
            SPDLOG_LOGGER_INFO(this->log_, "setting pixel format BayerRG8 succeeded");
    } else if (config.pixel_format == "BayerGB8") {
        if (MV_OK != MV_CC_SetEnumValue(this->handle_, "PixelFormat", PixelType_Gvsp_BayerGB8)) {
            SPDLOG_LOGGER_CRITICAL(this->log_, "setting pixel format BayerGB8 failed");
            exit(-1);
        } else {
            SPDLOG_LOGGER_INFO(this->log_, "setting pixel format BayerGB8 succeeded");
        }
    } else {
        SPDLOG_LOGGER_CRITICAL(this->log_, "unsupported pixel format");
        exit(-1);
    }
    //* Trigger mode
    SET_PARAM(EnumValue, config.trigger_mode, "TriggerMode");
    //* Depth
    SET_PARAM(EnumValue, config.adc_bit_depth, "ADCBitDepth");
    //* 设置曝光
    SET_PARAM(EnumValue, config.auto_exposure, "ExposureAuto");
    SET_PARAM(FloatValue, config.exposure_time, "ExposureTime");
    //* Gain
    SET_PARAM(EnumValue, config.gain_auto, "GainAuto");
    SET_PARAM(FloatValue, 0, "Gain");
    //* Set Gamma
    SET_PARAM(BoolValue, config.adjustable_gamma, "GammaEnable");
    if (config.adjustable_gamma) {
        SET_PARAM(EnumValue, 1, "GammaSelector");
        SET_PARAM(FloatValue, config.gamma, "Gamma");
    }

#undef SET_PARAM
}

void HikCamera::initialize_image_retrieval() {
    SPDLOG_LOGGER_INFO(this->log_, "initializing image retrieving");
    int result = MV_CC_StartGrabbing(this->handle_);
    if (result != MV_OK) {
        SPDLOG_LOGGER_CRITICAL(this->log_, "initialization fails");
        exit(-1);
    }
    SPDLOG_LOGGER_INFO(this->log_, "initialization succeed");
}

cv::Mat HikCamera::__get_frame() {
    std::memset(&this->buffer_, 0, sizeof(MV_FRAME_OUT));
    int n_ret = MV_CC_GetImageBuffer(this->handle_, &this->buffer_, 1000);
    if (n_ret != MV_OK) {
        SPDLOG_LOGGER_ERROR(this->log_, "failed to get image buffer, no data");
        return cv::Mat();
    }

    if constexpr (CameraDebug)
        SPDLOG_LOGGER_INFO(
            this->log_,
            "retrieving image buffer, w={}, h={}",
            this->buffer_.stFrameInfo.nWidth,
            this->buffer_.stFrameInfo.nHeight
        );
    auto img = convert_raw_to_mat(&this->buffer_.stFrameInfo, &this->buffer_);
    MV_CC_FreeImageBuffer(this->handle_, &this->buffer_);
    return img;
}

RawFrameInfo HikCamera::get_frame() {
    RawFrameInfo result;
    result.frame     = this->__get_frame();
    result.timestamp = std::chrono::system_clock::now();
    return result;
}
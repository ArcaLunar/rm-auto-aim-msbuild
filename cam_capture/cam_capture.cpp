#include "cam_capture.hpp"

#include "MvCameraControl.h"
#include "toml++/impl/parse_error.hpp"
#include "toml++/impl/parser.hpp"

#include <opencv2/core/persistence.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <spdlog/spdlog.h>
#include <toml++/toml.hpp>

constexpr char ConfigFilePath[] = "../../config/camera_config.json";

HikCamera::HikCamera() {
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
    this->setup();
    // 初始化图像捕获，准备捕获图像
    this->initialize_image_retrieval();
}

HikCamera::~HikCamera() {
    spdlog::info("exiting......");
    // 停止捕获图像
    spdlog::info("stop retrieving image");
    int result = MV_CC_StopGrabbing(this->handle_);
    if (result != MV_OK) spdlog::critical("error stopping");
    spdlog::info("stop succeed");

    // 关闭相机
    spdlog::info("closing camera");
    result = MV_CC_CloseDevice(this->handle_);
    if (result != MV_OK) spdlog::critical("error closing");
    spdlog::info("close succeed");

    // 销毁句柄
    spdlog::info("destroying handle");
    result = MV_CC_DestroyHandle(this->handle_);
    if (result != MV_OK) spdlog::critical("error destroying");
    spdlog::info("destroy succeed");

    // 释放 SDK
    MV_CC_Finalize();
    spdlog::info("exit successfully");
}

void HikCamera::enum_devices() {
    // 枚举相机设备
    spdlog::info("retriving available cam list ...");
    std::memset(&this->devicelist_, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    int nResult = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &this->devicelist_);
    if (nResult != MV_OK) {
        spdlog::critical("retrieving fails.");
        exit(-1);
    }
    spdlog::info("retrieving succeed");
}

void HikCamera::debug_devices() {
    // 相机的设备 ID
    spdlog::info("finding Hik Camera");
    if (devicelist_.nDeviceNum == 0) {
        spdlog::critical("no device found.");
        exit(-1);
    }
    for (int i = 0, n = devicelist_.nDeviceNum; i < n; i++) {
        MV_CC_DEVICE_INFO *pDeviceInfo = devicelist_.pDeviceInfo[i];
        spdlog::info("checking device {}", i);
        if (pDeviceInfo == nullptr) break;
        this->print_device_info(pDeviceInfo);
        camIndex = i;
    }
    spdlog::info("finding succeed");
}

void HikCamera::print_device_info(MV_CC_DEVICE_INFO *pDeviceInfo) {
    if (pDeviceInfo == nullptr) { // 设备不存在
        spdlog::critical("device info pointer is null.");
        exit(-1);
    }

    if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) { // USB 相机
        spdlog::info("device type: USB");
    } else if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
        spdlog::info("device type: GIGE");
    } else {
        spdlog::error("unknown device type, cannot continue");
        exit(-1);
    }
}

void HikCamera::create_handle() {
    spdlog::info("creating handle_ for camera {}", this->camIndex);
    int result = MV_CC_CreateHandle(&this->handle_, devicelist_.pDeviceInfo[camIndex]);
    if (result != MV_OK) {
        spdlog::critical("critical creating handle_");
        exit(-1);
    }
    spdlog::info("create success");
}

void HikCamera::open_camera() {
    spdlog::info("opening camera");
    int result = MV_CC_OpenDevice(this->handle_);
    if (result != MV_OK) {
        spdlog::critical("open camera fails");
        exit(-1);
    }
    spdlog::info("open succeed");
}

cv::Mat HikCamera::convert_raw_to_mat(MV_FRAME_OUT_INFO_EX *pstImageInfo, MV_FRAME_OUT *pstImage) {
    cv::Mat result;
    auto mark           = pstImageInfo->enPixelType;
    auto channel_type   = CV_8UC3;
    auto transform_type = cv::COLOR_BayerGB2RGB;

    if (mark == PixelType_Gvsp_BayerGB8) {
        channel_type   = CV_8UC1;
        transform_type = cv::COLOR_BayerRG2RGB;
    } else if (mark == PixelType_Gvsp_BGR8_Packed) {
        channel_type   = CV_8UC3;
        transform_type = cv::COLOR_BGR2GRAY;
    } else if (mark == PixelType_Gvsp_BayerGB8) {
        channel_type   = CV_8UC1;
        transform_type = cv::COLOR_BayerGB2RGB;
    } else spdlog::error("unsupported pixel format");

    cv::Mat src(pstImageInfo->nHeight, pstImageInfo->nWidth, channel_type, pstImage->pBufAddr);
    if (transform_type == cv::COLOR_BGR2GRAY) result = src;
    else cv::cvtColor(src, result, transform_type);

    return result;
}

CameraConfig HikCamera::load_config() {
    // ! 打开配置文件
    spdlog::info("reading from .config");

    toml::table T;
    struct CameraConfig config;
    try {
        T = toml::parse_file("../config/cam.toml");

        config.pixel_format     = T["pixel_format"].value_or("BayerRG8");
        // config.adc_bit_depth    = T["adc_bit_depth"].value_or(8);
        config.trigger_mode     = T["trigger_mode"].value_or(0);
        config.auto_exposure    = T["auto_exposure"].value_or(0);
        config.exposure_time    = T["exposure_time"].value_or(1000);
        config.gain_auto        = T["gain_auto"].value_or(0);
        config.adjustable_gamma = T["adjustable_gamma"].value_or(true);
        config.gamma            = T["gamma"].value_or(0.5);
    } catch (const toml::parse_error &e) { spdlog::error("error parsing config file: {}, using fallback", e.what()); }

    return config;
}

void HikCamera::setup() {
    auto config = load_config();

#define SET_PARAM(func, value, item)                                                                                   \
    if (MV_CC_Set##func(this->handle_, item, value) != MV_OK) {                                                        \
        spdlog::critical("setting {} to {} failed", item, value);                                                      \
    } else spdlog::info("setting {} to {} succeeded.", item, value);

    //* Pixel Format
    // if (config.pixel_format == "BayerRG8") {
    //     SET_PARAM(EnumValue, PixelType_Gvsp_BayerRG8, "PixelFormat");
    // } else if (config.pixel_format == "BayerGB8") {
    //     SET_PARAM(EnumValue, PixelType_Gvsp_BayerGB8, "PixelFormat");
    // } else if (config.pixel_format == "BGR8") {
    //     SET_PARAM(EnumValue, PixelType_Gvsp_BGR8_Packed, "PixelFormat");
    // } else {
    //     spdlog::critical("unsupported pixel format");
    //     exit(-1);
    // }
    //* Trigger mode
    SET_PARAM(EnumValue, config.trigger_mode, "TriggerMode");
    //* PayLoad
    // SET_PARAM(IntValue, config.payload_size, "PayloadSize");
    //* Depth
    // SET_PARAM(EnumValue, config.adc_bit_depth, "ADCBitDepth");
    //* 设置曝光
    SET_PARAM(EnumValue, config.auto_exposure, "ExposureAuto");
    SET_PARAM(FloatValue, config.exposure_time, "ExposureTime");
    //* Gain
    SET_PARAM(EnumValue, config.gain_auto, "GainAuto");
    //* Set Gamma
    if (config.adjustable_gamma) {
        SET_PARAM(EnumValue, 1, "GammaSelector");
        // SET_PARAM(FloatValue, config.gamma, "Gamma");
    }

#undef SET_PARAM
}

void HikCamera::initialize_image_retrieval() {
    spdlog::info("initializing image retrieving");
    int result = MV_CC_StartGrabbing(this->handle_);
    if (result != MV_OK) {
        spdlog::critical("initialization fails");
        exit(-1);
    }
    spdlog::info("initialization succeed");
}

cv::Mat HikCamera::get_frame() {
    std::memset(&this->buffer_, 0, sizeof(MV_FRAME_OUT));
    int n_ret = MV_CC_GetImageBuffer(this->handle_, &this->buffer_, 1000);
    if (n_ret != MV_OK) {
        spdlog::error("failed to get image buffer, no data");
        return cv::Mat();
    }

    spdlog::info(
        "retrieving image buffer, w={}, h={}", this->buffer_.stFrameInfo.nWidth, this->buffer_.stFrameInfo.nHeight
    );
    auto img = convert_raw_to_mat(&this->buffer_.stFrameInfo, &this->buffer_);
    MV_CC_FreeImageBuffer(this->handle_, &this->buffer_);
    return img;
}
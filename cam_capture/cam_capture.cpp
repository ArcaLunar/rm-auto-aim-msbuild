#include "cam_capture.hpp"

#include "MvCameraControl.h"

#include <opencv2/core/persistence.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <spdlog/spdlog.h>

constexpr char ConfigFilePath[] = "../../config/camera_config.json";

HikCamera::HikCamera() {
    // 初始化 SDK
    MV_CC_Initialize();
    // 枚举设备
    this->EnumDevices();
    // 输出设备信息
    this->DebugDevices();
    // 创建句柄
    this->CreateHandle();
    // 打开相机
    this->OpenCamera();
    // 设置相机参数
    this->Setup();
    // 初始化图像捕获，准备捕获图像
    this->InitRetrieveImage();
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

void HikCamera::EnumDevices() {
    // 枚举相机设备
    spdlog::info("retriving available cam list ...");
    std::memset(&this->devicelist_, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    int nResult = MV_CC_EnumDevices(
        MV_GIGE_DEVICE | MV_USB_DEVICE, &this->devicelist_
    );
    if (nResult != MV_OK) {
        spdlog::critical("retrieving fails.");
        exit(-1);
    }
    spdlog::info("retrieving succeed");
}

void HikCamera::DebugDevices() {
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
        this->PrintDeviceInfo(pDeviceInfo);
        camIndex = i;
    }
    spdlog::info("finding succeed");
}

void HikCamera::PrintDeviceInfo(MV_CC_DEVICE_INFO *pDeviceInfo) {
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

void HikCamera::CreateHandle() {
    spdlog::info("creating handle_ for camera {}", this->camIndex);
    int result = MV_CC_CreateHandle(
        &this->handle_, devicelist_.pDeviceInfo[camIndex]
    );
    if (result != MV_OK) {
        spdlog::critical("critical creating handle_");
        exit(-1);
    }
    spdlog::info("create success");
}

void HikCamera::OpenCamera() {
    spdlog::info("opening camera");
    int result = MV_CC_OpenDevice(this->handle_);
    if (result != MV_OK) {
        spdlog::critical("open camera fails");
        exit(-1);
    }
    spdlog::info("open succeed");
}

cv::Mat HikCamera::ConvertRawToMat(
    MV_FRAME_OUT_INFO_EX *pstImageInfo, MV_FRAME_OUT *pstImage
) {
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

    cv::Mat src(
        pstImageInfo->nHeight,
        pstImageInfo->nWidth,
        channel_type,
        pstImage->pBufAddr
    );
    if (transform_type == cv::COLOR_BGR2GRAY) result = src;
    else cv::cvtColor(src, result, transform_type);

    return result;
}

void HikCamera::LoadConfig() {
    // ! 打开配置文件
    cv::FileStorage fs(ConfigFilePath, cv::FileStorage::READ);
    spdlog::info("reading from .config");

    fs["Width"] >> this->config_.Width;
    fs["Height"] >> this->config_.Height;
    spdlog::info(
        "reading from config: W={}, H={}",
        this->config_.Width,
        this->config_.Height
    );

    fs["TriggerMode"] >> this->config_.TriggerMode;
    fs["AcquisitionFrameRateEnable"] >> this->config_.AcquisitionFrameRateEnable;
}

void HikCamera::Setup() {
    LoadConfig();

    // 辅助匿名函数
    auto SetAttr = [&](std::string attr, auto val) {
        spdlog::info("setting camera, {} -> {}", attr, val);
        int result = MV_CC_SetEnumValue(this->handle_, attr.c_str(), val);
        if (result != MV_OK)
            spdlog::critical("setting {} to {} failed", attr, val);
        spdlog::info("setting {} to {} succeeded.", attr, val);
    };

    // Trigger mode 设置为 off
    SetAttr("TriggerMode", this->config_.TriggerMode);
}

void HikCamera::InitRetrieveImage() {
    spdlog::info("initializing image retrieving");
    int result = MV_CC_StartGrabbing(this->handle_);
    if (result != MV_OK) {
        spdlog::critical("initialization fails");
        exit(-1);
    }
    spdlog::info("initialization succeed");
}

cv::Mat HikCamera::GetFrame() {
    std::memset(&this->buffer_, 0, sizeof(MV_FRAME_OUT));
    int n_ret = MV_CC_GetImageBuffer(
        this->handle_, &this->buffer_, 1000
    );
    if (n_ret != MV_OK) {
        spdlog::error("failed to get image buffer, no data");
        return cv::Mat();
    }

    spdlog::info("retrieving image buffer, w={}, h={}", this->buffer_.stFrameInfo.nWidth, this->buffer_.stFrameInfo.nHeight);
    auto img = ConvertRawToMat(&this->buffer_.stFrameInfo, &this->buffer_);
    MV_CC_FreeImageBuffer(this->handle_, &this->buffer_);
    return img;
}
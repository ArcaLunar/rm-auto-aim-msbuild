#include "camera.hpp"
#include "CameraParams.h"
#include "MvCameraControl.h"
#include "PixelType.h"
#include "debug_options.hpp"
#include "spdlog/spdlog.h"
#include "structs.hpp"

#include <chrono>
#include <cstring>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

extern DebugOptions options;

HikCamera::HikCamera() {
    spdlog::info("HikCamera initializing");

    //~ 1. initialize SDK
    spd_wrapper(mvcheck, options.camera.initialization, MV_CC_Initialize);

    //~ 2. enum devices. But no need to do this, there's only one camera.
    spd_wrapper(this->list_devices, options.camera.initialization);

    //~ 3. create handle
    spd_wrapper(this->create_handle, options.camera.initialization);

    //~ 4. open device
    spd_wrapper(this->open_device, options.camera.initialization);

    //~ 5. set configs
    spd_wrapper(this->set_configs, options.camera.initialization, "../config/camera.toml");

    //~ 6. start grab
    spd_wrapper(mvcheck, options.camera.initialization, MV_CC_StartGrabbing, this->handle);

    spdlog::info("HikCamera has initialized successfully");
}

cv::Mat convert_to_rgb(MV_FRAME_OUT *frame) {
    cv::Mat result;
    MV_FRAME_OUT_INFO_EX info = frame->stFrameInfo;

    if (info.enPixelType == PixelType_Gvsp_BayerRG8) {
        cv::Mat src(info.nExtendHeight, info.nWidth, CV_8UC1, frame->pBufAddr);
        cv::cvtColor(src, result, cv::COLOR_BayerBG2RGB);
    } else if (info.enPixelType == PixelType_Gvsp_BayerBG8) {
        cv::Mat src(info.nExtendHeight, info.nWidth, CV_8UC1, frame->pBufAddr);
        cv::cvtColor(src, result, cv::COLOR_BayerRG2RGB);
    } else if (info.enPixelType == PixelType_Gvsp_RGB8_Packed) {
        cv::Mat src(info.nExtendHeight, info.nWidth, CV_8UC3, frame->pBufAddr);
        result = src.clone();
    } else {
        spdlog::error("Unsupported pixel format");
        return cv::Mat();
    }
    return result;
}

RawImageFrame HikCamera::get_frame() {
    RawImageFrame result;

    memset(&this->frame, 0, sizeof(MV_FRAME_OUT));
    // wait for 1 sec, capture image
    spd_wrapper(mvcheck, options.camera.capture, MV_CC_GetImageBuffer, this->handle, &this->frame, 1000);

    result.image     = convert_to_rgb(&this->frame);     // convert to cv::Mat
    result.timestamp = std::chrono::system_clock::now(); // assign current time

    spd_wrapper(mvcheck, options.camera.capture, MV_CC_FreeImageBuffer, this->handle, &this->frame); // free buffer
    return result;
}

void HikCamera::list_devices() {
    camera_id = 0;
    memset(&devices, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    mvcheck(MV_CC_EnumDevices, MV_USB_DEVICE | MV_GIGE_DEVICE, &this->devices);
}

void HikCamera::create_handle() { mvcheck(MV_CC_CreateHandle, &this->handle, devices.pDeviceInfo[this->camera_id]); }

void HikCamera::open_device() { mvcheck(MV_CC_OpenDevice, this->handle); }

void HikCamera::set_configs(std::string path) {
    toml::table config = toml::parse_file(path);

    //~ 1. Trigger mode
    int trigger_mode = config["trigger_mode"].value_or(0);
    mvcheck(MV_CC_SetEnumValue, this->handle, "TriggerMode", trigger_mode);

    //~ 2. disable Acquisition frame rate
    bool acquisition_frame_rate = config["acquisition_frame_rate"].value_or(false);
    mvcheck(MV_CC_SetBoolValue, this->handle, "AcquisitionFrameRateEnable", acquisition_frame_rate);

    //~ 3. Pixel format
    std::string pixel_format = config["pixel_format"].value_or("BayerRG8");
    int format               = pixel_format == "BayerRG8" ? PixelType_Gvsp_BayerRG8 : PixelType_Gvsp_BayerBG8;
    mvcheck(MV_CC_SetEnumValue, this->handle, "PixelFormat", format);

    //~ 4. ADC bit depth
    int adc_bit_depth = config["adc_bit_depth"].value_or(2);
    mvcheck(MV_CC_SetEnumValue, this->handle, "ADCBitDepth", adc_bit_depth);

    //~ 5. exposure auto
    int exposure_auto = config["exposure_auto"].value_or(0);
    mvcheck(MV_CC_SetEnumValue, this->handle, "ExposureAuto", exposure_auto);

    //~ 6. exposure time
    int exposure_time = config["exposure_time"].value_or(1000);
    mvcheck(MV_CC_SetFloatValue, this->handle, "ExposureTime", exposure_time);

    //~ 7. gain auto
    int gain_auto = config["gain_auto"].value_or(0);
    mvcheck(MV_CC_SetEnumValue, this->handle, "GainAuto", gain_auto);

    //~ 8. gain
    int gain = config["gain"].value_or(0);
    mvcheck(MV_CC_SetFloatValue, this->handle, "Gain", gain);

    //~ 9. gamma enable
    bool gamma_enable = config["gamma_enable"].value_or(false);
    mvcheck(MV_CC_SetBoolValue, this->handle, "GammaEnable", gamma_enable);

    //~ 10. gamma
    if (gamma_enable) {
        int gamma_select = config["gamma_select"].value_or(1);
        mvcheck(MV_CC_SetEnumValue, this->handle, "GammaSelect", gamma_select);

        int gamma = config["gamma"].value_or(0.5);
        mvcheck(MV_CC_SetFloatValue, this->handle, "Gamma", gamma);
    }

    //~ 11. width, height
    int width  = config["width"].value_or(1440);
    int height = config["height"].value_or(1080);
    mvcheck(MV_CC_SetIntValue, this->handle, "Width", width);
    mvcheck(MV_CC_SetIntValue, this->handle, "Height", height);

    //~ 12. offset
    int offset_x = config["offset_x"].value_or(0);
    int offset_y = config["offset_y"].value_or(0);
    mvcheck(MV_CC_SetIntValue, this->handle, "OffsetX", offset_x);
    mvcheck(MV_CC_SetIntValue, this->handle, "OffsetY", offset_y);
}

#undef mvcheck
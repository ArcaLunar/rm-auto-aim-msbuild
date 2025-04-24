#include "CameraParams.h"
#include "MvCameraControl.h"
#include "debug_options.hpp"
#include "camera.hpp"

// extern DebugOptions options;

int main() {
    // HikCamera cam;
    MV_CC_Initialize();

    MV_CC_DEVICE_INFO_LIST devices;
    MV_FRAME_OUT fl;
    memset(&devices, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &devices);

    void *h;
    MV_CC_CreateHandle(&h, devices.pDeviceInfo[0]);
    MV_CC_OpenDevice(h);
}
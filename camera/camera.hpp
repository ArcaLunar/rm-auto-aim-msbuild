#ifndef __CAMERA_HPP__
#define __CAMERA_HPP__

#include "CameraParams.h"
#include "structs.hpp"
#include <string>

class HikCamera {
  public:
    HikCamera();
    RawImageFrame get_frame();

  private:
    MV_CC_DEVICE_INFO_LIST devices;
    int camera_id;

    void *handle;

    MV_FRAME_OUT frame;

  protected:
    void list_devices();
    void create_handle();
    void open_device();
    void set_configs(std::string config_file);
};

#endif
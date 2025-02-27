#ifndef __PUBLISHER_HPP__
#define __PUBLISHER_HPP__

#include "classifier.hpp"
#include "detector.hpp"
#include "structs.hpp"

#include <memory>
#include <vector>

namespace AutoAim {

class Publisher {
  public:
    Publisher(const std::string &config_path);

    std::vector<AnnotatedArmorInfo> annotate_image(const RawFrameInfo &raw, const IMUInfo &imu);

  protected:
    std::shared_ptr<Detector> detector_;
    std::shared_ptr<Classifier> classifier_;
};

} // namespace AutoAim

#endif
#pragma once

#include "id_classify.hpp"
#include "structs.hpp"
#include "config.hpp"

#include <memory>

class Detector {
    std::unique_ptr<Classifier> classifier_;

  public:
    ProcessedFrameInfo Detect();
};
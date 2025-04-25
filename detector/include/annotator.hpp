#ifndef __ANNOTATOR_HPP__
#define __ANNOTATOR_HPP__

#include "classifier.hpp"
#include "detector.hpp"
#include "structs.hpp"
#include <memory>
#include <vector>

class Annotator {
  public:
    Annotator()
        : detector(std::make_shared<OpenCVDetector>("../config/detector.toml")),
          classifier(std::make_shared<Classifier>("../config/classifier.toml")) {}

    /**
     * @brief 根据摄像机拍到的图像和接受到的电控消息，进行标定、识别
     *
     * @param frame 摄像机拍到的图像
     * @param msg 电控消息
     * @return std::vector<AnnotatedArmorInfo> 一帧图像中所有的装甲板信息
     */
    std::vector<AnnotatedArmorInfo> annotate(const RawImageFrame &frame, const RecvMsgType &msg);

  private:
    std::shared_ptr<OpenCVDetector> detector;
    std::shared_ptr<Classifier> classifier;
    std::vector<int> labels;
};

#endif
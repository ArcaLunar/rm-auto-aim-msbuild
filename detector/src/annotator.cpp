#include "annotator.hpp"
#include "spdlog/spdlog.h"
#include "structs.hpp"

std::vector<AnnotatedArmorInfo> Annotator::annotate(const RawImageFrame &frame, const RecvMsgType &msg) {
    // 解析电控消息
    const uint8_t *targets = reinterpret_cast<const uint8_t *>(&msg.shoot_decision);
    for (int i = 0; i < 8; i++)
        if (*targets & (1 << i))
            this->labels.push_back(i + 1);

    // 提取装甲板
    std::vector<AnnotatedArmorInfo> result;
    auto armors = this->detector->detect(frame.image);
    if (armors.empty()) {
        spdlog::warn("No armor detected");
        return result;
    }

    // 识别数字
    auto rois = this->classifier->extract_region_of_interest(frame.image, armors);
    IMUInfo imu = {msg.roll, msg.pitch, msg.yaw, frame.timestamp};
    for (size_t i = 0; i < armors.size(); ++i) {
        AnnotatedArmorInfo info;

        info.armor  = armors[i];
        info.result = this->classifier->classify(rois[i]);
        info.imu_info = imu;
        info.timestamp = frame.timestamp;

        // level 1 过滤
        if (std::find(this->labels.begin(), this->labels.end(), info.result) == this->labels.end()) {
            spdlog::warn("Armor {} not in labels, needs to be ignored", info.result);
            continue;
        }
        
        result.push_back(info);
    }

    return result;
}
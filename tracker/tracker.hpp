#ifndef __TRACKER_HPP__
#define __TRACKER_HPP__

#include "kf.hpp"
#include "low_pass_filter.hpp"
#include "structs.hpp"

#include <chrono>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace AutoAim {

class Tracker {
  public:
    Tracker(
        const Labels &label,
        const std::string &config_path = "/media/arca/ArcaEXT4/codebases/pred_v2/config/tracking.toml"
    );
    /**
     * @brief 定期检查跟踪状态。超过一定时间未检测到目标则认为丢失。参数由配置文件指定。
     * @remark Should run as a `thread`
     */
    void check_status();

    /**
     * @brief 更新跟踪状态
     * @param armor3d
     */
    PredictedPosition update(const Armor3d &armor3d);

    PredictedPosition get_pred() { return pred_; }

  protected:
    TrackingConfig cfg_;
    FiringConfig fire_cfg_;

    TrackingStatus status_;
    Armor3d prev_state_;
    std::chrono::high_resolution_clock::time_point last_track_time_;

    ArmorCount armor_count_;
    Labels tracked_id_;
    // KalmanFilter::EKF ekf_;

    LowPassFilter low_pass_;
    int state_dim, observe_dim;
    KalmanFilter::KF kf_;
    PredictedPosition pred_;

    std::shared_ptr<spdlog::logger> log_;

  private:
    /**
     * @brief 根据过滤过的状态，预测下一帧的位置
     */
    PredictedPosition _forward_and_predict(const cv::Mat &estimate, const Armor3d &armor);

    void _forward_and_init();

    /**
     * @brief 返回过滤后的状态
     */
    cv::Mat _forward_and_update(const Armor3d &armor3d);
};

} // namespace AutoAim

#endif // __TRACKER_HPP__
#ifndef __TRACKER_HPP__
#define __TRACKER_HPP__

#include "ekf.hpp"
#include "structs.hpp"
#include <chrono>

namespace Tracker {

class Tracker {
  public:
    Tracker() = default;
    /**
     * @brief 定期检查跟踪状态。超过一定时间未检测到目标则认为丢失。参数由配置文件指定。
     * @remark Should run as a `thread`
     */
    void update_status();

  protected:
    TrackingConfig cfg_;

    TrackingStatus status_;
    std::chrono::high_resolution_clock::time_point last_lost_time_;

    ArmorCount armor_count_;
    EKF ekf_;

  private:
};

} // namespace Tracker

#endif // __TRACKER_HPP__
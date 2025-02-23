#ifndef __TRACKER_HPP__
#define __TRACKER_HPP__

#include "ekf.hpp"
#include "structs.hpp"

namespace Tracker {

class Tracker {
  public:
  protected:
    TrackingStatus status_;
    ArmorCount armor_count_;
    EKF ekf_;

  private:
};

} // namespace Tracker

#endif // __TRACKER_HPP__
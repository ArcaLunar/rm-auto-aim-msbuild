#ifndef __TRACKER_HPP__
#define __TRACKER_HPP__

namespace Tracker {

enum class TrackingStatus {
    FITTING,
    TRACKING,
    TEMPORARY_LOST,
    LOST,
};

class Tracker {
  public:
  protected:
    TrackingStatus status_;

  private:
};

} // namespace Tracker

#endif // __TRACKER_HPP__
#ifndef __THPOOL_HPP__
#define __THPOOL_HPP__

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

template <typename WorkType>
class WorkQueue {
  public:
  protected:
    std::queue<WorkType> work_queue_;  // work queue
    std::mutex wq_mtx_;                // work queue mutex
    std::condition_variable consumer_; // to inform consumer
    std::condition_variable producer_; // to inform producer

    static constexpr size_t MAX_QUEUE_SIZE = 10000; // max queue size
};

#endif
#ifndef __THPOOL_HPP__
#define __THPOOL_HPP__

#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>

template <typename WorkType>
class WorkQueue {
  public:
    // 设置生产者任务和数量
    void set_producer(const std::function<WorkType()> &producer, int count = 1);
    // 设置消费者任务和数量
    void set_consumer(const std::function<void(WorkType)> &consumer, int count = 1);
    // 启动工作队列
    void start() {
        for (int i = 0; i < this->producer_count_; i++) {
            std::thread([&]() {});
        }
    }

  protected:
    std::queue<WorkType> work_queue_;  // work queue
    std::mutex wq_mtx_;                // work queue mutex
    std::condition_variable consumer_; // to inform consumer
    std::condition_variable producer_; // to inform producer

    std::function<WorkType()> producer_worker_;
    int producer_count_;
    std::function<void(WorkType)> consumer_worker_;
    int consumer_count_;

    static constexpr size_t MAX_QUEUE_SIZE = 10000; // max queue size
};

#endif
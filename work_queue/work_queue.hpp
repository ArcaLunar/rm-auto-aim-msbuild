#ifndef __THPOOL_HPP__
#define __THPOOL_HPP__

#include "circular_buffer.hpp"

#include <atomic>
#include <functional>
#include <pthread.h>
#include <semaphore.h>
#include <spdlog/spdlog.h>
#include <vector>

template <typename WorkType, int MAX_SIZE = 10000>
class WorkQueue {
  public:
    WorkQueue() : producer_count_(0), consumer_count_(0) {}

    /**
     * @brief 设置生产者行为和数量
     */
    void set_producer(std::function<WorkType()> producer, int count) {
        producer_       = producer;
        producer_count_ = count;
    }

    /**
     * @brief 设置消费者行为和数量
     */
    void set_consumer(std::function<void(WorkType)> consumer, int count) {
        consumer_       = consumer;
        consumer_count_ = count;
    }

    /**
     * @brief 启动工作队列
     */
    void start() {
        // Set up the producer threads
        for (int i = 0; i < producer_count_; ++i) {
            producer_threads_.emplace_back([&]() {
                while (!stop_) {
                    this->not_full_.acquire(); // 等待队列有空位
                    if (stop_) break;

                    WorkType work = this->producer_();
                    this->buffer_.push(work); // 将任务放入队列

                    this->not_empty_.release(); // 通知消费者队列不为空
                }
            });
        }

        // Set up the consumer threads
        for (int i = 0; i < consumer_count_; ++i) {
            consumer_threads_.emplace_back([&]() {
                while (!stop_) {
                    this->not_empty_.acquire(); // 等待队列不为空
                    if (stop_) break;

                    auto work = this->buffer_.pop(); // 从队列中取出任务
                    if (work.has_value()) {
                        this->consumer_(work.value()); // 处理任务

                        this->not_full_.release(); // 通知生产者队列不满
                    }
                }
            });
        }
    }

    /**
     * @brief 停止工作队列
     */
    void stop() {
        stop_ = true;
        for (auto &t : producer_threads_) not_empty_.release();
        for (auto &t : consumer_threads_) not_full_.release();
        for (auto &t : producer_threads_) t.join();
        for (auto &t : consumer_threads_) t.join();
    }

  protected:
    CircularBuffer<WorkType> buffer_{MAX_SIZE};                   // 待处理的任务队列
    std::counting_semaphore<> not_empty_{0}, not_full_{MAX_SIZE}; // 信号量，用于控制队列是否为空或者是否已满
    std::atomic<bool> stop_{false};

    std::function<WorkType()> producer_;
    int producer_count_;
    std::vector<std::thread> producer_threads_;

    std::function<void(WorkType)> consumer_;
    int consumer_count_;
    std::vector<std::thread> consumer_threads_;
};

#endif
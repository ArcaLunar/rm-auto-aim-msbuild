#ifndef __THPOOL_HPP__
#define __THPOOL_HPP__

#include "circular_buffer.hpp"

#include <atomic>
#include <functional>
#include <memory>
#include <pthread.h>
#include <semaphore>
#include <spdlog/spdlog.h>
#include <vector>

/**
 * @brief 线程安全多生产者多消费者数据传输器
 * @remark producer() 和 consumer() 里需要使用外部变量的话，必须使用 std::shared_ptr<> 传递外部变量。
 * 使用例见 `test/dataflow_cam2image_test.cpp`
 * 
 * @tparam DataType 数据类型
 * @tparam BUFFER_SIZE 缓冲大小
 */
template <typename DataType, int BUFFER_SIZE = 1024>
class DataTransmitter {
  public:
    void register_producer(std::function<DataType()> producer, int count = 1) {
        for (int i = 0; i < count; i++) {
            producer_threads_.emplace_back([this, producer] {
                start_producer_.acquire();
                while (!stop_) {
                    empty_.acquire();
                    if (stop_)
                        break;

                    spdlog::info("producing data");
                    auto data = producer();
                    buffer_.push(data);
                    filled_.release();
                }
            });
        }
    }

    void register_consumer(std::function<void(const DataType &)> consumer, int count = 1) {
        for (int i = 0; i < count; i++) {
            consumer_threads_.emplace_back([this, consumer] {
                start_consumer_.acquire();
                while (!stop_) {
                    filled_.acquire();
                    if (stop_)
                        break;

                    spdlog::info("consuming data");
                    auto data = buffer_.pop();
                    if (data.has_value())
                        consumer(data.value());
                    empty_.release();
                }
            });
        }
    }

    void start() {
        spdlog::info("start data transfer");

        start_producer_.release(producer_threads_.size());
        start_consumer_.release(producer_threads_.size());
    }

    void stop() {
        stop_ = true;
        for (auto &t : producer_threads_)
            empty_.release();
        for (auto &t : consumer_threads_)
            filled_.release();
    }

  protected:
    CircularBuffer<DataType> buffer_{BUFFER_SIZE};
    std::counting_semaphore<> empty_{BUFFER_SIZE}, filled_{0};

    std::atomic<bool> stop_{false};

    std::function<void()> producer_;
    std::counting_semaphore<> start_producer_{0};
    std::vector<std::thread> producer_threads_;

    std::function<void()> consumer_;
    std::counting_semaphore<> start_consumer_{0};
    std::vector<std::thread> consumer_threads_;

  private:
};

#endif
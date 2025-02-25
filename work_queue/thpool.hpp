/**
 * @file thpool.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-02-25
 * 
 * @copyright Copyright (c) 2025
 * 
 * !!! UNFINISHED !!!
 */

#ifndef __THREAD_POOL_HPP__
#define __THREAD_POOL_HPP__

/**
 * @brief Dedicated thread pool for executing certain work
 */
#include <functional>
#include <memory>
#include <pthread.h>
#include <semaphore.h>

template <size_t MAX_THREADPOOL_SIZE = 16>
class ThreadPool {
  public:
    ThreadPool() : head_(nullptr), tail_(nullptr),queue_length_(0) {
        sem_init(&q_io_mutex, 0, 1);
        sem_init(&q_has_task, 0, 0);
    }

  protected:
    struct task_t {
        std::function<void(void *)>;
        void *args;
        task_t *next;
    }; // 任务类型

    // 任务队列以及对应 semaphore
    volatile size_t queue_length_;
    std::unique_ptr<task_t> head_, tail_;
    sem_t q_io_mutex; // io control
    sem_t q_has_task; // no task

    // 线程池相关
    std::unique_ptr<pthread_t[]> threads_;
    volatile bool not_shutdown_;
    volatile int working_thread_num_;
    volatile int online_thread_num_;
    sem_t work_lock_;
    sem_t online_lock_;
    sem_t idle_all_;

  private:
};

#endif
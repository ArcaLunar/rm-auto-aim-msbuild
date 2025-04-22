#ifndef __CIRCULAR_BUFFER_HPP__
#define __CIRCULAR_BUFFER_HPP__

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <utility>

template <typename T>
class CircularBuffer {
  public:
    explicit CircularBuffer(size_t size = 1) : data(std::unique_ptr<T[]>(new T[size])), max_size{size} {}

    void push(T item) {
        std::lock_guard<std::mutex> lock(this->mtx);

        this->data[head] = item;
        if (this->full)
            this->tail = (this->tail + 1) % this->max_size;
        this->head = (this->head + 1) % this->max_size;
        this->full = this->head == this->tail;
    }

    std::pair<T, bool> pop() {
        std::lock_guard<std::mutex> lock(this->mtx);
        if (this->is_empty())
            return {T{}, false};

        auto item  = std::move(this->data[this->tail]);
        this->full = false;
        this->tail = (this->tail + 1) % this->max_size;
        return {item, true};
    }

    void reset() {
        std::lock_guard<std::mutex> lock(this->mtx);
        this->head = this->tail;
        this->full = false;
    }

    bool is_empty() const {
        return (!this->full && (this->head == this->tail));
    }

    bool is_full() const { return this->full; }

    size_t size() const {
        std::lock_guard<std::mutex> lock(this->mtx);
        if (this->full)
            return this->max_size;
        if (this->head >= this->tail)
            return this->head - this->tail;
        return this->max_size + this->head - this->tail;
    }

  private:
    std::mutex mtx;
    std::unique_ptr<T[]> data;
    size_t head{0}, tail{0}, max_size{0};
    std::atomic_bool full{false};
};

#endif
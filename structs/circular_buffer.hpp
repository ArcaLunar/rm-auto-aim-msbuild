/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef __CIRCULAR_BUFFER_HPP__
#define __CIRCULAR_BUFFER_HPP__

#include <atomic>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>

template <typename T>
class CircularBuffer {
  public:
    explicit CircularBuffer(size_t size = 1) : buffer_(std::unique_ptr<T[]>(new T[size])), max_size_(size) {}
    /**
     * @brief Add an item to the buffer.
     */
    void push(const T &item) {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_[head_] = std::move(item);
        if (is_full())
            tail_ = (tail_ + 1) % max_size_;
        head_ = (head_ + 1) % max_size_;
        full_ = head_ == tail_;
    }

    std::optional<T> pop() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (is_empty())
            return std::nullopt;
        T result = std::move(buffer_[tail_]);
        full_    = false;
        tail_    = (tail_ + 1) % max_size_;
        return std::make_optional(std::move(result));
    }

    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        head_ = tail_;
        full_ = false;
    }

    // return true if the buffer is empty
    bool is_empty() { return (!full_ && (head_ == tail_)); }

    // return true if the buffer is full
    bool is_full() { return full_; }

    // return the number of elements in the buffer
    size_t size() {
        std::lock_guard<std::mutex> lock(mutex_);
        size_t size = max_size_;
        if (!is_full()) {
            if (head_ >= tail_)
                size = head_ - tail_;
            else
                size = max_size_ + head_ - tail_;
        }
        return size;
    }

    size_t capacity() const { return max_size_; }

  private:
    std::mutex mutex_;
    std::unique_ptr<T[]> buffer_;
    size_t head_{0};
    size_t tail_{0};
    size_t max_size_;
    std::atomic<bool> full_ = false;
};

#endif // __CIRCULAR_BUFFER_HPP__
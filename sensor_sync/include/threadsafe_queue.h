#ifndef THREADSAFE_QUEUE_H
#define THREADSAFE_QUEUE_H

#include <queue>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <stdexcept>

// 有界线程安全队列
template <typename T>
class ThreadSafeQueue {
public:
    explicit ThreadSafeQueue(size_t max_size)
        : max_size_(max_size), closed_(false) {}

    // 入队：队列满时阻塞，队列关闭时抛出异常
    void push(T value) {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_full_.wait(lock, [this]() {
            return queue_.size() < max_size_ || closed_;
        });
        if (closed_) {
            throw std::runtime_error("Queue is closed");
        }
        queue_.push(std::move(value));
        cond_empty_.notify_one();
    }

    // 尝试入队：队列满或关闭时返回false
    bool try_push(T value) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (closed_ || queue_.size() >= max_size_) {
            return false;
        }
        queue_.push(std::move(value));
        cond_empty_.notify_one();
        return true;
    }

    // 出队：队列为空时阻塞，队列关闭且为空时返回false
    bool wait_and_pop(T& value) {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_empty_.wait(lock, [this]() {
            return !queue_.empty() || closed_;
        });
        if (queue_.empty()) {
            return false; // 队列已关闭且为空
        }
        value = std::move(queue_.front());
        queue_.pop();
        cond_full_.notify_one();
        return true;
    }

    // 尝试出队：队列为空时返回false
    bool try_pop(T& value) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return false;
        }
        value = std::move(queue_.front());
        queue_.pop();
        cond_full_.notify_one();
        return true;
    }

    // 关闭队列：之后无法再入队，但消费者仍可取完剩余数据
    void close() {
        std::lock_guard<std::mutex> lock(mutex_);
        closed_ = true;
        cond_empty_.notify_all(); // 唤醒所有等待的消费者
        cond_full_.notify_all();  // 唤醒所有等待的生产者
    }

    bool is_closed() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return closed_;
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable cond_empty_; // 队列非空信号（消费者等待）
    std::condition_variable cond_full_;  // 队列非满信号（生产者等待）
    std::queue<T> queue_;
    size_t max_size_;
    bool closed_;
};

#endif // THREADSAFE_QUEUE_H

#pragma once

#include <mutex>
#include <condition_variable>
#include <queue>
#include <chrono>
#include <iostream>

namespace SpinDevices {
    namespace SpinBase {
    
        template<typename T>
        class ThreadedQueue {
        private:
            std::mutex m_push_pull_mutex;
            std::condition_variable m_cv;
            std::queue<T> m_queue;
        public:
            ThreadedQueue() = default;
            /**
             * Rule of five
             */
            ~ThreadedQueue() = default;
            ThreadedQueue(const ThreadedQueue&) = delete;
            ThreadedQueue(ThreadedQueue&&) noexcept = default;
            ThreadedQueue& operator=(const ThreadedQueue&) = delete;
            ThreadedQueue& operator=(ThreadedQueue&&) noexcept = default;
    
            void swap(ThreadedQueue&&);
            bool empty();
            T pop();
            void push(T&&);
        };
    
    } //SpinBase
    
    // -------------------------------------------------- //
    //                    ThreadedQueue
    // -------------------------------------------------- //
    template<typename T>
    void SpinBase::ThreadedQueue<T>::swap(ThreadedQueue&& in) {
        T tmp = std::move(in);
        in = std::move(*this);
        *this = std::move(tmp);
    }
    template<typename T>
    bool SpinBase::ThreadedQueue<T>::empty() {
        std::unique_lock<std::mutex> lk{m_push_pull_mutex};
        return m_queue.empty();
    }
    template<typename T>
    T SpinBase::ThreadedQueue<T>::pop() {
        std::unique_lock<std::mutex> lk{m_push_pull_mutex};
        m_cv.wait_for(lk, std::chrono::milliseconds(5), [this]() {
                        return !m_queue.empty();
                    }
                );
        if(m_queue.empty()) { //mutex is locked by m_cv so ok
            throw std::runtime_error("Queue still empty after 5ms");
        } 
        T out = std::move( m_queue.front() );
        m_queue.pop();
        return out;
    }
    template<typename T>
    void SpinBase::ThreadedQueue<T>::push(T&& in) {
        std::lock_guard<std::mutex> lk{m_push_pull_mutex};
        m_queue.push( std::move(in) );
        m_cv.notify_one();
    }
} // SpinDevices

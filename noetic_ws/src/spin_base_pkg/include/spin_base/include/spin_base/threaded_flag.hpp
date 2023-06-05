#pragma once

#include <mutex>
#include <iostream>

namespace SpinDevices{
    namespace SpinBase {
        class ThreadedFlag {
        private:
            std::mutex m_set_get_mutex;
            bool m_state;
        public:
            ThreadedFlag();
            /**
             * Rule of five
             */
            ~ThreadedFlag() = default;
            ThreadedFlag(const ThreadedFlag&) = delete;
            ThreadedFlag(ThreadedFlag&&);
            ThreadedFlag& operator=(const ThreadedFlag&) = delete;
            ThreadedFlag& operator=(ThreadedFlag&&);
            void set();
            bool get();
        };
    } // SpinBase
} // SpinDevices

#include "spin_base/threaded_flag.hpp"

namespace SpinDevices{
    namespace SpinBase{
        ThreadedFlag::ThreadedFlag():
            m_state{false} {
        }
        ThreadedFlag::ThreadedFlag(ThreadedFlag&& in):
            m_set_get_mutex() {
            std::unique_lock<std::mutex> lk1{in.m_set_get_mutex};
            std::unique_lock<std::mutex> lk2{m_set_get_mutex};
            m_state = in.m_state; 
        }
        ThreadedFlag& ThreadedFlag::operator=(ThreadedFlag&& in) {
            if(this == &in) return *this; //prevent double locking 
            
            std::unique_lock<std::mutex> lk1{in.m_set_get_mutex};
            std::unique_lock<std::mutex> lk2{m_set_get_mutex};

            this->m_state = in.m_state;
            return *this;
        }
        void ThreadedFlag::set() {
            std::unique_lock<std::mutex> lk{m_set_get_mutex};
            m_state = true;
        }
        bool ThreadedFlag::get() {
            std::unique_lock<std::mutex> lk{m_set_get_mutex}; 
            bool out = m_state;
            m_state = false;
            return out;
        }
    } // SpinBase
} // SpinDevices

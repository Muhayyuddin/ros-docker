#pragma once

#include "socketcan_interface/interface.h"

#include <iostream>
#include <mutex>
#include <shared_mutex>
#include <condition_variable>
#include <queue>
#include <stdexcept>
#include <thread>
#include <vector>
#include <memory>
#include <chrono>
#include <functional>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
//#include <linux/if.h>
//#include <thread>

#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/xmlrpc_settings.h>
#include <socketcan_interface/threading.h>

namespace SpinDevices {
    namespace SpinBase {
        
        using TCAN_SPtr = can::ThreadedSocketCANInterfaceSharedPtr;
    
        /**
         * Common interface for all devices produced
         * which communicate on CAN
         */
        class ISpinDevice_CAN {
        private:
            TCAN_SPtr m_can_socket;
        protected:
            mutable can::Frame m_can_frame;
    
            /**
             * Wrapper for Socketcan utility
             */
            virtual bool can_send() const {
                return m_can_socket->send(m_can_frame);
            }
            template<typename ...Args> 
            can::FrameListenerConstSharedPtr createMsgListener(Args... args) {
                return m_can_socket->createMsgListenerM(args...);
            }
        public:
            //Rule of five
            ISpinDevice_CAN(TCAN_SPtr _m_can_socket): m_can_socket{_m_can_socket} { }
            ISpinDevice_CAN( const ISpinDevice_CAN& ) = delete;
            ISpinDevice_CAN( ISpinDevice_CAN&&) noexcept = default;
            ISpinDevice_CAN& operator=( const ISpinDevice_CAN& ) = delete;
            ISpinDevice_CAN& operator=( ISpinDevice_CAN&& ) noexcept = default;
    
            virtual ~ISpinDevice_CAN() {
                m_can_socket->shutdown(); 
            }
            
        };
    
        /**
         * Common interface for every device produced
         * which communicates on CAN and require a monitoring
         * thread
         */
        class ISpinDevice_CAN_Threaded : public ISpinDevice_CAN {
        protected:
            std::thread m_worker;
            mutable std::mutex m_mutex;
            bool m_do_work = false;
            virtual void task() = 0;
            virtual void m_work() {
                bool work = true;
                while(work) {
                    task();
                    {
                        std::lock_guard<std::mutex> lk{m_mutex};
                        work = m_do_work;
                    }
                }
            }
        public:
            ISpinDevice_CAN_Threaded(TCAN_SPtr sckt): ISpinDevice_CAN(sckt) {}
            virtual ~ISpinDevice_CAN_Threaded() = default;
            virtual void init(std::function<void(void)>&&=[]{}) { }
            void run() {
                if(!is_running()) {
                    {
                        std::lock_guard<std::mutex> lk{m_mutex};
                        m_do_work = true;
                    }
                    m_worker = std::thread{&SpinBase::ISpinDevice_CAN_Threaded::m_work,this};
                }
            }
            void stop() {
                if(is_running()) {
                    {
                        std::unique_lock<std::mutex> lk{m_mutex};
                        m_do_work = false;    
                    }
                    m_worker.join();
                    std::cout << "Thread stopped correctly" << std::endl;
                } else {
                    std::cout << "Thread inactive, no need to stop it" << std::endl; 
                }
            }
            bool is_running() {
                std::unique_lock<std::mutex> lk{m_mutex}; 
                return m_do_work;
            }
        };
    
    } //SpinBase
} // SpinDevices

#pragma once

#include <vector>
#include <mutex>

namespace SpinDevices{
    namespace SpinBase {
        class CanOpenMsg {
        private:
            uint16_t m_obj;
            uint8_t m_idx;
        public:
    
            CanOpenMsg(uint16_t,uint8_t);
            virtual ~CanOpenMsg() = default;
            CanOpenMsg& operator=(const CanOpenMsg&) = default;
    
            bool operator==(const CanOpenMsg&) const;
            bool operator!=(const CanOpenMsg&) const;
            bool operator<=(const CanOpenMsg&) const;
            bool operator>=(const CanOpenMsg&) const;
            bool operator<(const CanOpenMsg&) const;
            bool operator>(const CanOpenMsg&) const;
    
            uint16_t obj() const;
            uint8_t idx() const;
        };
    } // SpinBase
} // SpinDevices

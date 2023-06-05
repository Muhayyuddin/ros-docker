#include <spin_base/canopen_msg.hpp>

namespace SpinDevices{
    namespace SpinBase{
        CanOpenMsg::CanOpenMsg(uint16_t obj, uint8_t idx):
            m_obj{obj}, m_idx{idx} { }
        bool CanOpenMsg::operator==(const CanOpenMsg& cmp) const {
            return m_idx == cmp.idx() && m_obj == cmp.obj();
        } 
        bool CanOpenMsg::operator!=(const CanOpenMsg& cmp) const {
            return !(*this == cmp); 
        }
        bool CanOpenMsg::operator<(const CanOpenMsg& cmp) const {
            return m_obj < cmp.obj() || (/**/m_obj == cmp.obj() && m_idx < cmp.idx()/**/); 
        }
        bool CanOpenMsg::operator>(const CanOpenMsg& cmp) const {
            return m_obj > cmp.obj() || (/**/m_obj == cmp.obj() && m_idx > cmp.idx()/**/); 
        }
        bool CanOpenMsg::operator<=(const CanOpenMsg& cmp) const {
            return *this < cmp || *this == cmp; 
        }
        bool CanOpenMsg::operator>=(const CanOpenMsg& cmp) const {
            return *this > cmp || *this == cmp; 
        }
        uint16_t CanOpenMsg::obj() const {
            return m_obj; 
        }
        uint8_t CanOpenMsg::idx() const {
            return m_idx;
        }
    } // SpinBase
} // SpinDevices

#include "spin_base/vehicle.hpp"

namespace SpinDevices {
    namespace SpinBase{
    
        Vehicle::~Vehicle() {
            set_control_state(0);
            disable();
        }
        void Vehicle::enable() {
            m_can_frame.dlc = 2;
            m_can_frame.is_extended = true;
            m_can_frame.id = (uint16_t)(m_dev_type << 8) | 0x1C;
            m_can_frame.data[0] = m_id;
            m_can_frame.data[1] = 1;
            can_send();
        }
        void Vehicle::disable() {
            m_can_frame.dlc = 2;
            m_can_frame.is_extended = true;
            m_can_frame.id = (uint16_t)(m_dev_type << 8) | 0x1C;
            m_can_frame.data[0] = m_id;
            m_can_frame.data[1] = 0;
            can_send();
        }
        void Vehicle::set_control_state(bool state) {
            m_can_frame.id = (uint16_t)(m_dev_type << 8) | 0x1A;
            m_can_frame.is_extended = true;
            m_can_frame.dlc = 2;
            m_can_frame.data[0] = m_id;
            m_can_frame.data[1] = state;
            can_send();
        }
        void Vehicle::send_thrust(int16_t left, int16_t right) {
            m_can_frame.id = (uint16_t)(m_dev_type << 8) | 0x02;
            m_can_frame.is_extended = true;
            m_can_frame.dlc = 5;
            m_can_frame.data[0] = m_id;
            *((int16_t*)(m_can_frame.data.data() + 1)) = left;
            *((int16_t*)(m_can_frame.data.data() + 3)) = right;
            can_send();
    
        }
        void Vehicle::send_velocity(float vel, float ome) {
            m_can_frame.is_extended = true;
            m_can_frame.dlc = 5;
            m_can_frame.data[0] = m_id;
            
            m_can_frame.id = (uint16_t)(m_dev_type << 8) | 0x34;
            *((int32_t*)(m_can_frame.data.data() + 1)) = *(int32_t*)&vel;
            can_send();
    
            m_can_frame.id = (uint16_t)(m_dev_type << 8) | 0x3A;
            *((int32_t*)(m_can_frame.data.data() + 1)) = *(int32_t*)&ome;
            can_send();
        }
        void Vehicle::send_param(const float *pid_pointer, float ome_coef) {
            m_can_frame.is_extended = true;
            m_can_frame.id = (uint16_t)(m_dev_type << 8) | 0x30;
            m_can_frame.dlc = 6;
            m_can_frame.data[0] = m_id;
            for(int index{0}; index < 3; ++index) {
                m_can_frame.data[1] = index;
                *((int32_t*)(m_can_frame.data.data() + 2)) = *((int32_t*)(pid_pointer + index));
                can_send();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } 
    
            m_can_frame.id = (uint16_t)(m_dev_type << 8) | 0x3E;
            m_can_frame.dlc = 5;
            *((int32_t*)(m_can_frame.data.data() + 1)) = *(int32_t*)&ome_coef;
            can_send(); 
        }
        void Vehicle::configure_DAC(int channel, const float* x_vals, const float* y_vals) {
    
            //std::cout << x_vals[0] << " "<< x_vals[1] << " "<< y_vals[0] << " "<< y_vals[1] << std::endl;
            float mV_to_levels = 4096.0/5000.0;
            //std::cout << "mV to levels " << mV_to_levels << std::endl;
            float m = ( y_vals[1] - y_vals[0] ) / ( x_vals[1] - x_vals[0]);
            float q = ( y_vals[0] - m * x_vals[0] );
            m *= mV_to_levels;
            q *= mV_to_levels;
            //std::cout << "m " << m << std::endl;
            //std::cout << "q " << q << std::endl;
    
            m_can_frame.is_extended = true;
            m_can_frame.id = (uint16_t)(m_dev_type << 8) | 0x40;
            m_can_frame.dlc = 7;
            m_can_frame.data[0] = m_id;
            m_can_frame.data[1] = channel;
    
            m_can_frame.data[2] = 0;
            *((int32_t*)(m_can_frame.data.data() + 3)) = *(int32_t*)&m;
            can_send();
    
            m_can_frame.data[2] = 1;
            *((int32_t*)(m_can_frame.data.data() + 3)) = *(int32_t*)&q;
            can_send();
        }
    
    } // SpinBase
} // SpinDevices

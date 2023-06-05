#pragma once
#include "ISpinDevice.hpp"
#include <chrono>

namespace SpinDevices {
    namespace SpinBase {
    
        /**
         * Model of the actual catamaran board
         */
        class Vehicle : public ISpinDevice_CAN {
        protected:
            //{ REGION: CAN
            uint8_t m_id;
            uint8_t m_dev_type;
    
            /**
             * Allow the device to be controlled via can
             */
            void enable();
    
            /**
             * Disable the can control of the device
             */
            void disable();
    
            /**
             * Set the control mode on one between 'thrust control'
             * and 'velocity control'
             *
             * ...
             *
             * @param state Indicates which control mode to use:
             *              - 0 is thruster control
             *              - 1 is velocity control
             */
            void set_control_state(bool state);
            
            /**
             * Commands the thrust value to the left and right thrusters
             *
             * ...
             *
             * @param left  Left thrust, from -100 to 100
             * @param right Right thrust, from -100 to 100
             */
            void send_thrust(int16_t left, int16_t right);
    
            /**
             * Commands the desired velocity to the board
             *
             * ...
             *
             * @param vel   Linear velocity, directed in font of the
             *              vehicle
             * @param ome   Angular velocity, positive if counter-clock-wise
             *              negative is clock-wise
             */
            void send_velocity(float vel, float ome);
    
            /**
             * Send the parameters of the PID controller for the velocity
             *
             * ...
             *
             * @param pid_pointer   Points to a 3 float array containing PID values
             *                      ordered as: 1) P, 2) I, 3) D
             * @param ome_coeff     A proportional coefficient to model the angular velocity
             */
            void send_param(const float *pid_pointer, float ome_coef);
    
            /**
             * Send the right mappings for the two output
             * channel of the DAC
             *
             * ...
             *
             * @param channel   Represent the channel whose mapping is being computed
             * @param x_vals    Represent the pointer to the minimum and maximum values of the control input
             * @param y_vals    Represent the pointer to the minimum and maximum values of the voltage output
             *                  expressed in [mV]
             */
            void configure_DAC(int channel, const float *x_vals, const float* y_vals);
            //}
    
        public:
            Vehicle(TCAN_SPtr _m_can_socket, uint8_t _dev_type, uint8_t id):
                ISpinDevice_CAN(_m_can_socket),
                m_dev_type(_dev_type),
                m_id(id) {enable();}
    
            /**
             * Destructor set the board to "Non Operative"
             */
            virtual ~Vehicle() override;
        };
    
    } // SpinBase
} // SpinDevices

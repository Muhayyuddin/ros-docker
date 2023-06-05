#include <iostream>
#include <ros/ros.h>
#include "socketcan_interface/interface.h"
#include "spin_base/ISpinDevice.hpp"
#include "ros/subscriber.h"
#include "std_msgs/Int16MultiArray.h"

int safety_cnt = 40;
int16_t steer_setpoint[2];
int16_t pod_setpoint[2];

template<typename T>
bool in_range(const T& in_value, const T& min, const T& max) {
    return in_value >= min && in_value <= max;
}

void rot_listener(SpinDevices::SpinBase::TCAN_SPtr sckt, const std_msgs::Int16MultiArrayConstPtr& value) {
    if(value.get()->data.size() != 2) {
        ROS_ERROR("Number of arguments not conformant: must be `2`");
    } else {
        if( in_range(value->data[0], (int16_t)-90, (int16_t)90) && in_range(value->data[1], (int16_t)-90, (int16_t)90)) {
            safety_cnt = 40;
            *((int32_t*)steer_setpoint) = *((int32_t*)(value->data.data()));
        } else ROS_ERROR("Input argument out of range!");
    }
}

void pod_listener(SpinDevices::SpinBase::TCAN_SPtr sckt, const std_msgs::Int16MultiArrayConstPtr& value) {
    if(value.get()->data.size() != 2) {
        ROS_ERROR("Number of arguments not conformant: must be `2`");
    } else {
        if( in_range(value->data[0], (int16_t)-100, (int16_t)100) && in_range(value->data[1], (int16_t)-100, (int16_t)100)) {
            safety_cnt = 40;
            *((int32_t*)pod_setpoint) = *((int32_t*)(value->data.data()));
        } else ROS_ERROR("Input argument out of range!");
    }
}

void send_on_can(SpinDevices::SpinBase::TCAN_SPtr sckt) {
   can::Frame out_frame; 
   out_frame.is_extended = true;
   out_frame.dlc = 7;
   out_frame.id = 0x2E0;
   *((int16_t*)(out_frame.data.data() + 1)) = 0x00;

   out_frame.data.data()[0] = 0x01;
   *((int16_t*)(out_frame.data.data() + 1)) = (safety_cnt != 0) ? pod_setpoint[0] : 0;
   *((int16_t*)(out_frame.data.data() + 3)) = (safety_cnt != 0) ? steer_setpoint[0] : 0;
   *((int16_t*)(out_frame.data.data() + 5)) = 0x00;
   sckt->send(out_frame);
   
   out_frame.data.data()[0] = 0x02;
   *((int16_t*)(out_frame.data.data() + 1)) = (safety_cnt != 0) ? pod_setpoint[1] : 0;
   *((int16_t*)(out_frame.data.data() + 3)) = (safety_cnt != 0) ? steer_setpoint[1] : 0;
   *((int16_t*)(out_frame.data.data() + 5)) = 0x00;
   sckt->send(out_frame);
}

int main( int argc, char* argv[] ) {
    /******************
     * Initialization *
     * ****************/
    ros::init(argc,argv,"rot_pod_node");
    ros::NodeHandle nh, nh_params("~");

    std::string can_port;
    nh.param("/can_port", can_port, std::string("vcan0"));

    std::cout << "Binding to " <<  can_port << std::endl;

    SpinDevices::SpinBase::TCAN_SPtr sckt = std::make_shared<can::ThreadedSocketCANInterface>(); 
    if(!sckt->init( can_port , 0, XmlRpcSettings::create(nh_params) )) {
        ROS_FATAL("Could not bind to port: %s", can_port.c_str());
        return 1;
    }
    else {
        ROS_INFO("Successfully connected to port: %s", can_port.c_str()); 
    }

    //Insert code here
    boost::function<void(const std_msgs::Int16MultiArrayConstPtr&)> rot_tmp = [&sckt](const std_msgs::Int16MultiArrayConstPtr& in_val){rot_listener(sckt,in_val);};
    ros::Subscriber rot_sub = nh_params.subscribe("rotation", 100, rot_tmp);
    boost::function<void(const std_msgs::Int16MultiArrayConstPtr&)> pod_tmp = [&sckt](const std_msgs::Int16MultiArrayConstPtr& in_val){pod_listener(sckt,in_val);};
    ros::Subscriber pod_sub = nh_params.subscribe("thrust", 100, pod_tmp);
    //

    ros::Rate rate(20);
    while(nh.ok()) {
        rate.sleep();
        ros::spinOnce();
        send_on_can(sckt);
        if(safety_cnt) --safety_cnt;
    }
    return 0;
}
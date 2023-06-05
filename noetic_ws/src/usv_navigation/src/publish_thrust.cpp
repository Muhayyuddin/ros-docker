
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"

#include <sstream>

/**
 * This tutorial demonstrates  sending the thrust command to the USV.
 */
int main(int argc, char **argv)
{
  std_msgs::Int16 left_thrust;
  std_msgs::Int16 right_thrust;
  std_msgs::Int16 left_pod_angle;
  std_msgs::Int16 right_pod_angle;

  left_thrust.data = 50;
  right_thrust.data  = 50;
  left_pod_angle.data  = 45;
  right_pod_angle.data  = 45;

  ros::init(argc, argv, "usv_navigation");

  ros::NodeHandle n;

  ros::Publisher thruster = n.advertise<std_msgs::Int16MultiArray >("/test_setup/mini_cat/thrust", 10);
  ros::Publisher pods_angle = n.advertise<std_msgs::Int16MultiArray >("/test_setup/mini_cat/rotation", 10);

  std_msgs::Int16MultiArray thrust;
  std_msgs::Int16MultiArray pod_angle;
  thrust.data.resize(2);
  pod_angle.data.resize(2);

  thrust.data[0] = left_thrust.data;
  thrust.data[1] = right_thrust.data;
  pod_angle.data[0] = left_pod_angle.data;
  pod_angle.data[1] = right_pod_angle.data;

  ros::Rate loop_rate(10);
  int i{0};
  while (i<=200)
  {

    if(i<=99)
    {
      pod_angle.data[0] = 30;
      pod_angle.data[1] = 30;
    }
    if(i>99 && i<=110)
    {
      pod_angle.data[0] = 0;
      pod_angle.data[1] = 0;
    }
    else  if(i>110 && i<=190)
    {
      pod_angle.data[0] = -30;
      pod_angle.data[1] = -30;
    }
    else  if(i>190)
    {
      pod_angle.data[0] = 0;
      pod_angle.data[1] = 0;
    }
    std::cout << "thrust values are  [ left: " << thrust.data[0] <<", right: " <<thrust.data[1]<<" ]"<< std::endl;
    std::cout << "pos's angle is   [ left: " << pod_angle.data[0] <<", right: " <<pod_angle.data[1]<<" ]"<< std::endl;
    thruster.publish(thrust);
    pods_angle.publish(pod_angle);
    
    ros::spinOnce();
    loop_rate.sleep();
    i ++;
  }

  // The pods will not reset it to the zero position when we will restart. 
  // when we will turn on the system we need to read the current position of the joint (joint angle)
  // if it is not zero we can send a command to move it to zero 
  return 0;
}
//rostopic pub --once /thrust std_msgs/Int16MultiArray '{data: [45,45]}'
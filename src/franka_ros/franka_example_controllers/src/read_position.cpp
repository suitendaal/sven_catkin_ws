#include <pluginlib/class_list_macros.h>

#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float32.h"
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <franka/exception.h>
#include <franka_example_controllers/cartesian_impedance_example_controller.h>
#include <cmath>
#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include "franka_example_controllers/pseudo_inversion.h"
#include "franka_msgs/FrankaState.h"


std::array<double, 3> Position;
void chatterCallback(franka_msgs::FrankaState robot_state) //Probably here is missing a pointer but, if I add the pointer the code is not compiling 
{
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Position[0]= position[0];
  Position[1]= position[1];
  Position[2]= position[2];
  //std::cout << position;
  //position=msg.O_T_EE;
  //robot_state = state_handle_->getRobotState().O_T_EE_d;
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "Position_EE");


  ros::NodeHandle node_position;
  ros::Rate loop_rate(30); // was 100

  ros::Subscriber sub = node_position.subscribe("/franka_state_controller/franka_states", 1000, chatterCallback);

  ros::Publisher pub = node_position.advertise<geometry_msgs::Point>("/cartesian_position", 1000);
  

  //std_msgs::Float32MultiArray msg;
  geometry_msgs::Point msg;
  //std_msgs::Float32 msg;
  //std::array<double, 16> pose;
  

  while (ros::ok())
  {
   

    //Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    
    msg.x=Position[0];
    msg.y=Position[1];
    msg.z=Position[2];
    
    pub.publish(msg);
    

    ros::spinOnce();

    loop_rate.sleep();
  }
  




  //ros::spin();


  return 0;
}

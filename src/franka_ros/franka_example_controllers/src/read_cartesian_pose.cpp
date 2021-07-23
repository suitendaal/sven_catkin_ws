#include <pluginlib/class_list_macros.h>

#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
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

ros::Publisher pub;

std::array<double, 3> Position;
std::array<double, 4> Orientation;
void chatterCallback(franka_msgs::FrankaState robot_state) //Probably here is missing a pointer but, if I add the pointer the code is not compiling 
{
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  
  Eigen::Vector3d position(transform.translation());
  Position[0] = position[0];
  Position[1] = position[1];
  Position[2] = position[2];
  
  Eigen::Quaterniond orientation(Eigen::Quaterniond(transform.linear()));
  Orientation[0] = orientation.coeffs()[0];
  Orientation[1] = orientation.coeffs()[1];
  Orientation[2] = orientation.coeffs()[2];
  Orientation[3] = orientation.coeffs()[3];

  geometry_msgs::PoseStamped msg;
  geometry_msgs::Pose msg_pose;
  geometry_msgs::Point msg_pos;
  geometry_msgs::Quaternion msg_ori;

  msg_pos.x = Position[0];
  msg_pos.y = Position[1];
  msg_pos.z = Position[2];

  msg_ori.x = Orientation[0];
  msg_ori.y = Orientation[1];
  msg_ori.z = Orientation[2];
  msg_ori.w = Orientation[3];

  msg_pose.position = msg_pos;
  msg_pose.orientation = msg_ori;
  msg.pose = msg_pose;
  msg.header = robot_state.header;
  pub.publish(msg);
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "Pose_EE");


  ros::NodeHandle node_position;
  // ros::Rate loop_rate(100); // was 100

  pub = node_position.advertise<geometry_msgs::PoseStamped>("/cartesian_pose", 1000);
  ros::Subscriber sub = node_position.subscribe("/franka_state_controller/franka_states", 1000, chatterCallback);

  ros::spin();

  // geometry_msgs::Pose msg;
  // geometry_msgs::Point msg_pos;
  // geometry_msgs::Quaternion msg_ori;
  

  // while (ros::ok())
  // {
   

    //Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    
    // msg_pos.x = Position[0];
    // msg_pos.y = Position[1];
    // msg_pos.z = Position[2];

    // msg_ori.x = Orientation[0];
    // msg_ori.y = Orientation[1];
    // msg_ori.z = Orientation[2];
    // msg_ori.w = Orientation[3];

    // msg.position = msg_pos;
    // msg.orientation = msg_ori;
    
    // pub.publish(msg);
    

    // ros::spinOnce();

    // loop_rate.sleep();
  // }
  




  //ros::spin();


  return 0;
}

#ifndef JUMP_DETECTOR_NODE_H
#define JUMP_DETECTOR_NODE_H

#include <ros/ros.h>

#include "jump_detector/jump_detector.h"

#include <sensor_msgs/JointState.h>
#include <sven_ros/BoolStamped.h>

class JumpDetectorNode {

  private:
    JumpDetector* detector_;
    int joint_;
    ros::Publisher jump_detector_pub;
	ros::Subscriber joint_data_sub;
    
    void joint_state_received(const sensor_msgs::JointStateConstPtr &msg);
    
  public:
    ros::NodeHandle nh;
    
    JumpDetectorNode(int joint, JumpDetector &detector);
    JumpDetectorNode(ros::NodeHandle nh, int joint, JumpDetector &detector);
    ~JumpDetectorNode();
    void run();

};

#endif // JUMP_DETECTOR_NODE_H

#ifndef JUMP_DETECTOR_NODE_H
#define JUMP_DETECTOR_NODE_H

#include "jump_detector/jump_detector.h"
#include <ros/ros.h>

class JumpDetectorNode {

  private:
    JumpDetector* detector_;
    
  public:
    ros::NodeHandle nh;
    
    JumpDetectorNode(JumpDetector* detector);
    ~JumpDetectorNode();
    void run();
    
    // TODO: callback function on joint data received.

};

#endif // JUMP_DETECTOR_NODE_H

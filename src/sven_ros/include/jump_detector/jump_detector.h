#ifndef JUMP_DETECTOR_H
#define JUMP_DETECTOR_H

#include <queue>

#include "jump_detector/datapoint.h"

class JumpDetector {

  protected:
  	std::queue<DataPoint> data_;
  	int max_window_length_;
  
  
  public:
    JumpDetector(int max_window_length)
    : max_window_length_(max_window_length),
    data_()
    {}
  
  	virtual bool datapoint_arrived(double time, double value) {
  	  DataPoint datapoint(time,value);
  	  data_.push(datapoint);
  	  if (data_.size() > max_window_length_ + 1) {
  	    data_.pop();
  	  }
  	  return false;
  	}

};

#endif // JUMP_DETECTOR_H

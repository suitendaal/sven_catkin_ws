#ifndef DATA_CONTAINER_H
#define DATA_CONTAINER_H

#include <queue>

#include "jump_detector/datapoint.h"

class DataContainer {

  protected:
  	std::queue<DataPoint> data_;
  	unsigned int max_window_length_;
  
  public:
    DataContainer(unsigned int max_window_length)
    : data_(),
    max_window_length_(max_window_length)
    {}
  
  	virtual bool update(double time, double value) {
  	  DataPoint datapoint(time,value);
  	  return this->update(datapoint);
  	}
  	
  	virtual bool update(DataPoint &datapoint){
  		this->data_.push(datapoint);
  	  while (this->data_.size() > this->max_window_length_) {
  	    this->data_.pop();
  	  }
  	  return false;
  	}
  	
  	virtual void reset() {
  		this->data_ = {};
  	}
  	
  	void set_window_length(unsigned int window_length) {
  		this->max_window_length_ = window_length;
  	}
  	
  	std::vector<DataPoint> get_data() const {
  		std::queue<DataPoint> q = this->data_;
  		std::vector<DataPoint> v;
  		while (!q.empty())
		  {
		    v.push_back(q.front());
		    q.pop();
		  }
		  return v;
  	}

};

#endif // DATA_CONTAINER_H

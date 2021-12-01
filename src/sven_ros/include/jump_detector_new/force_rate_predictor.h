#ifndef FORCE_RATE_PREDICTOR_H
#define FORCE_RATE_PREDICTOR_H

#include "jump_detector_new/least_squares_predictor.h"

#include <iostream>

class ForceRatePredictor : public LeastSquaresPredictor {

  protected:
  double time_window_;
  double last_impact_time_;
  
  public:
  ForceRatePredictor(unsigned int max_window_length, double time_window)
  : LeastSquaresPredictor(0, max_window_length),
  time_window_(time_window),
  last_impact_time_(-1)
  {}
  
  ForceRatePredictor(double time_window)
  : ForceRatePredictor(0, time_window)
  {}
  
  virtual bool enough_data(const DataPoint &datapoint) const {
/*    std::cout << "Enough data: " << this->last_impact_time_ << ", " << datapoint.time << ", " << this->time_window_ << std::endl;*/
    return (LeastSquaresPredictor::enough_data(datapoint) && (this->last_impact_time_ < 0 || datapoint.time > this->last_impact_time_ + this->time_window_));
  }
  
  virtual bool update(DataPoint &datapoint){
    if (datapoint.time <= this->last_impact_time_) {
      this->last_impact_time_ = -1;
    }
    return LeastSquaresPredictor::update(datapoint);
  }
	
	virtual void reset() {
	  if (this->get_data().size() > 0) {
	    this->last_impact_time_ = this->get_data()[this->get_data().size()-1].time;
	  }
		LeastSquaresPredictor::reset();
	}
	
	virtual bool predict(const DataPoint &datapoint, std::vector<double> &value) const {
/*		std::cout << "this->last_impact_time_: " << this->last_impact_time_ << ", datapoint.time: " << datapoint.time << ", this->time_window_: " << this->time_window_ << std::endl;*/
		if (this->last_impact_time_ < 0 || datapoint.time - this->last_impact_time_ > this->time_window_) {
			return LeastSquaresPredictor::predict(datapoint, value);
		}
		return false;
	}
};

#endif // FORCE_RATE_PREDICTOR_H


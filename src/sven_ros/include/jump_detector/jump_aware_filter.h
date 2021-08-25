#ifndef JUMP_AWARE_FILTER_H
#define JUMP_AWARE_FILTER_H

#include <vector>

#include "jump_detector/jump_detector.h"
#include "jump_detector/bounder.h"
#include "jump_detector/predictor.h"

class JumpAwareFilter : public JumpDetector {

private:
    Predictor* predictor_;
    Bounder* bounder_;
    unsigned int window_length_;
    
  public:
    JumpAwareFilter(int max_window_length, Predictor &predictor, Bounder &bounder)
    : JumpDetector(max_window_length),
    window_length_(0),
    predictor_(&predictor),
    bounder_(&bounder)
    {}
    
    // Processed an incoming datapoint. Returns true if a jump is detected.
    bool update(DataPoint datapoint) {
    	// Detect jump
    	bool jump_detected = this->detect_jump(datapoint);
    	
    	// Update window length
    	if (jump_detected) {
    		this->window_length_ = 0;
    	}
    	else {
    		if (this->window_length_ < this->max_window_length_) {
    			this->window_length_++;
    		}
    	}
    	this->predictor_->set_window_length(this->window_length_);
    	this->bounder_->set_window_length(this->window_length_);
    	
    	// Update data
  		unsigned int tmp = this->max_window_length_;
  		this->max_window_length_ = this->window_length_;
  		DataContainer::update(datapoint);
  		this->predictor_->update(datapoint);
  		this->bounder_->update(datapoint);
  		this->max_window_length_ = tmp;
  		
  		return jump_detected;
  	}
    	
    bool detect_jump(DataPoint datapoint) const {
    	
    	// Initialize variables
    	double predicted_value; 
      double bounded_value;
      bool jump_detected = false;
      
      // Detect the jump
      if (predictor_->predict(datapoint, predicted_value) && bounder_->bound(datapoint, bounded_value)) {
      	jump_detected = abs(predicted_value - datapoint.value) > bounded_value;
      }
      
      return jump_detected;
    }
    
    double get_window_length() const {
      return window_length_;
    }

};

#endif // JUMP_AWARE_FILTER_H


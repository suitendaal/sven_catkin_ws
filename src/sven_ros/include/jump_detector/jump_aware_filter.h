#ifndef JUMP_AWARE_FILTER_H
#define JUMP_AWARE_FILTER_H

#include <vector>
#include <iostream>

#include "jump_detector/jump_detector.h"
#include "jump_detector/bounder.h"
#include "jump_detector/predictor.h"
#include "jump_detector/datapoint.h"

class JumpAwareFilter : public JumpDetector {

private:
    Predictor* predictor_;
    Bounder* bounder_;
    int window_length_;
    
  public:
    JumpAwareFilter(int max_window_length, Predictor &predictor, Bounder &bounder)
    : JumpDetector(max_window_length),
    window_length_(0),
    predictor_(&predictor),
    bounder_(&bounder)
    {}
    
    // Processed an incoming datapoint. Returns true if a jump is detected.
    bool datapoint_arrived(double time, double value) {
      
      std::queue<DataPoint> q = data_;
      std::vector<DataPoint> v;
      while (!q.empty())
      {
        if (q.size() <= window_length_ + 1) {
          v.push_back(q.front());
        }
        q.pop();
      }
      double predicted_value; 
      double bounded_value;
      bool jump_detected = false;
      
      if (predictor_->predict(v, time, predicted_value) && bounder_->bound(v, time, bounded_value)) {
      	jump_detected = abs(predicted_value - value) > bounded_value;
      	std::cout << "Value: " << value << ", Predicted value: " << predicted_value << ", Jump detected: " << jump_detected << std::endl;
      }
      
      if (jump_detected) {
        window_length_ = 0;
      }
      else if (window_length_ < max_window_length_) {
        window_length_++;
      }
      
      JumpDetector::datapoint_arrived(time, value);
      return jump_detected;
    }
    
    double get_current_window_length() const {
      return window_length_;
    }

};

#endif // JUMP_AWARE_FILTER_H

